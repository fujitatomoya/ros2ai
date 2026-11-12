# Copyright 2026 Tomoya Fujita <tomoya.fujita825@gmail.com>.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""End-to-end tests of the `ai` command with a fake OpenAI client, no network access required."""

import argparse
import importlib
import json
import subprocess
from types import SimpleNamespace

import pytest
from ros2ai.api import agent as agent_module
import ros2ai.api.constants as constants
from ros2ai.command.ai import AiCommand


class FakeCompletion:

    def __init__(self, content, finish_reason='stop', tool_calls=None):
        message = SimpleNamespace(content=content, tool_calls=tool_calls)
        if tool_calls:
            finish_reason = 'tool_calls'
        self.choices = [SimpleNamespace(finish_reason=finish_reason, message=message)]

    def __repr__(self):
        return f'FakeCompletion({self.choices[0].message.content!r})'


def fake_tool_calls(calls):
    """Build the tool calls of a completion from (name, arguments) pairs."""
    return [
        SimpleNamespace(
            id=f'call_{index}', type='function',
            function=SimpleNamespace(name=name, arguments=json.dumps(arguments)))
        for index, (name, arguments) in enumerate(calls)
    ]


class FakeRos2:
    """Recorded `ros2` executions and their canned output."""

    stdout = 'fake ros2 output'
    returncode = 0
    calls = []
    # canned stdout per command (tuple of argv), `stdout` is used if the command is absent
    outputs = {}


def fake_chunks(contents, tool_calls=None, finish_reason='stop'):
    """Stream the contents and the tool calls as chunks, the arguments are split in two."""
    def chunk(content=None, tool_calls=None, finish_reason=None):
        delta = SimpleNamespace(content=content, tool_calls=tool_calls)
        return SimpleNamespace(
            choices=[SimpleNamespace(delta=delta, finish_reason=finish_reason)])
    for content in contents:
        yield chunk(content=content)
    for index, tool_call in enumerate(tool_calls or []):
        arguments = tool_call.function.arguments
        half = len(arguments) // 2
        yield chunk(tool_calls=[SimpleNamespace(
            index=index, id=tool_call.id, type='function',
            function=SimpleNamespace(name=tool_call.function.name, arguments=arguments[:half]))])
        yield chunk(tool_calls=[SimpleNamespace(
            index=index, id=None, type=None,
            function=SimpleNamespace(name=None, arguments=arguments[half:]))])
    yield chunk(finish_reason=finish_reason)
    # usage chunk without choices
    yield SimpleNamespace(choices=[])


class FakeOpenAI:
    """Stand-in for openai.OpenAI recording constructor and completion arguments."""

    content = 'fake answer'
    finish_reason = 'stop'
    error = None
    model_ids = ['model-a', 'model-b']
    instances = []
    session_file = None
    # scripted responses consumed in order before falling back to `content`:
    # str for the content, list of (name, arguments) for tool calls, Exception to raise
    responses = []

    def __init__(self, api_key=None, base_url=None, default_headers=None, **kwargs):
        self.api_key = api_key
        self.base_url = base_url
        self.default_headers = default_headers
        self.create_calls = []
        self.chat = SimpleNamespace(completions=SimpleNamespace(create=self._create))
        self.models = SimpleNamespace(list=self._list_models)
        FakeOpenAI.instances.append(self)

    def _create(self, **kwargs):
        self.create_calls.append(kwargs)
        if FakeOpenAI.error is not None:
            raise FakeOpenAI.error
        stream = kwargs.get('stream')
        if FakeOpenAI.responses:
            response = FakeOpenAI.responses.pop(0)
            if isinstance(response, Exception):
                raise response
            if isinstance(response, list):
                tool_calls = fake_tool_calls(response)
                if stream:
                    return fake_chunks([], tool_calls)
                return FakeCompletion(None, tool_calls=tool_calls)
            if stream:
                return fake_chunks([response])
            return FakeCompletion(response)
        if stream:
            return fake_chunks(
                [FakeOpenAI.content, None, '!'], finish_reason=FakeOpenAI.finish_reason)
        return FakeCompletion(FakeOpenAI.content, FakeOpenAI.finish_reason)

    def _list_models(self):
        return [SimpleNamespace(id=model_id) for model_id in FakeOpenAI.model_ids]


@pytest.fixture
def fake_openai(monkeypatch, tmp_path):
    FakeOpenAI.content = 'fake answer'
    FakeOpenAI.finish_reason = 'stop'
    FakeOpenAI.error = None
    FakeOpenAI.model_ids = ['model-a', 'model-b']
    FakeOpenAI.instances = []
    FakeOpenAI.responses = []
    FakeRos2.stdout = 'fake ros2 output'
    FakeRos2.returncode = 0
    FakeRos2.calls = []
    FakeRos2.outputs = {}
    # patch the OpenAI class wherever ros2ai imported it
    for module_name in ('ros2ai.api.config', 'ros2ai.api.openai', 'ros2ai.api.client'):
        try:
            module = importlib.import_module(module_name)
        except ImportError:
            continue
        if hasattr(module, 'OpenAI'):
            monkeypatch.setattr(module, 'OpenAI', FakeOpenAI)

    # `ros2` tool executions are recorded, model listing may go through curl
    def fake_run(cmd, **kwargs):
        if cmd[0] == 'ros2':
            assert kwargs.get('shell') is None
            FakeRos2.calls.append(cmd)
            stdout = FakeRos2.outputs.get(tuple(cmd), FakeRos2.stdout)
            return subprocess.CompletedProcess(
                cmd, FakeRos2.returncode, stdout=stdout, stderr='')
        assert cmd[0] == 'curl'
        data = {'data': [{'id': model_id} for model_id in FakeOpenAI.model_ids]}
        return subprocess.CompletedProcess(cmd, 0, stdout=json.dumps(data), stderr='')
    monkeypatch.setattr(subprocess, 'run', fake_run)
    # never block on the confirmation prompt, tests opt in with `fake_confirm`
    monkeypatch.setattr(
        agent_module, 'confirm_execution',
        lambda command: pytest.fail(f'unexpected confirmation for {command}'))

    for name in (
        constants.ROS2AI_API_KEY_ENV_VAR,
        constants.ROS2AI_MODEL_NAME_ENV_VAR,
        constants.ROS2AI_ENDPOINT_ENV_VAR,
        constants.ROS2AI_TEMPERATURE_ENV_VAR,
        constants.ROS2AI_ROLE_SYSTEM_ENV_VAR,
        constants.ROS2AI_CONTEXT_LIMIT_ENV_VAR,
        constants.ROS_OPENAI_API_KEY_ENV_VAR,
        constants.ROS_OPENAI_MODEL_NAME_ENV_VAR,
        constants.ROS_OPENAI_ENDPOINT_ENV_VAR,
        constants.ROS_OPENAI_TEMPERATURE_ENV_VAR,
        constants.ROLE_SYSTEM_ENV_VAR,
    ):
        monkeypatch.delenv(name, raising=False)
    monkeypatch.setenv(constants.ROS2AI_API_KEY_ENV_VAR, 'sk-test')
    monkeypatch.setenv('ROS_DISTRO', 'rolling')
    # the session file of the user is never touched
    FakeOpenAI.session_file = tmp_path / 'state' / 'session.json'
    monkeypatch.setenv(constants.ROS2AI_SESSION_FILE_ENV_VAR, str(FakeOpenAI.session_file))
    return FakeOpenAI


def run_command(argv):
    parser = argparse.ArgumentParser(prog='ros2 ai')
    command = AiCommand()
    command.add_arguments(parser, 'ros2 ai')
    return command.main(parser=parser, args=parser.parse_args(argv))


def test_status(fake_openai, capsys):
    assert run_command(['-s']) == 0
    out = capsys.readouterr().out
    assert out == 'fake answer\n'
    client = fake_openai.instances[0]
    assert client.api_key == 'sk-test'
    assert client.base_url == constants.ROS2AI_DEFAULT_ENDPOINT
    assert client.default_headers is None
    call = client.create_calls[0]
    assert call['model'] == constants.ROS2AI_DEFAULT_MODEL
    assert call['messages'] == [{'role': 'user', 'content': 'Are you available?'}]
    assert call['temperature'] == constants.ROS2AI_DEFAULT_TEMPERATURE
    assert call['max_tokens'] is None


def test_status_verbose_list(fake_openai, capsys):
    assert run_command(['-vl', '-m', 'gpt-x', '-t', '7']) == 0
    out = capsys.readouterr().out
    assert out == (
        '----- api_model: gpt-x\n'
        f'----- api_endpoint: {constants.ROS2AI_DEFAULT_ENDPOINT}\n'
        '----- api_token: 7\n'
        '----- api_temperature: 1.0\n'
        'fake answer\n'
        '[SUCCESS] Valid API key.\n'
        '[SUCCESS] Retrieved list of models.\n'
        'Available Models:\n'
        'model-a\n'
        'model-b\n'
    )
    call = fake_openai.instances[0].create_calls[0]
    assert call['model'] == 'gpt-x'
    assert call['max_tokens'] == 7


def test_status_invalid_key(fake_openai, capsys):
    fake_openai.error = RuntimeError('boom')
    assert run_command(['-s']) == 1
    out = capsys.readouterr().out
    assert 'Failed to call LLM API: boom' in out
    assert '[FAILURE] Invalid API key.' in out


def test_status_not_finished(fake_openai, capsys):
    fake_openai.finish_reason = 'length'
    assert run_command(['-s']) == 1
    out = capsys.readouterr().out
    assert 'Failed chat completion with: length' in out
    assert '[FAILURE] Invalid API key.' in out


def test_answer_without_tool_calls(fake_openai, capsys):
    assert run_command(['what', 'is ros']) == 0
    assert capsys.readouterr().out == 'fake answer!\n'
    call = fake_openai.instances[0].create_calls[0]
    assert call['stream'] is True
    assert call['messages'] == [
        {'role': 'system', 'content': constants.ROLE_SYSTEM_DEFAULT.format('rolling')},
        {'role': 'user', 'content': 'what is ros'},
    ]
    assert FakeRos2.calls == []


def test_answer_nostream(fake_openai, capsys):
    assert run_command(['hello', '-n']) == 0
    assert capsys.readouterr().out == 'fake answer\n'
    assert fake_openai.instances[0].create_calls[0]['stream'] is False


def test_answer_verbose(fake_openai, capsys):
    assert run_command(['hello', '-v']) == 0
    out = capsys.readouterr().out
    assert out.startswith('System role:\n' + constants.ROLE_SYSTEM_DEFAULT.format('rolling'))
    # raw completion is only available without streaming
    assert 'FakeCompletion' not in out
    assert out.endswith('fake answer!\n')


def test_answer_nostream_verbose(fake_openai, capsys):
    assert run_command(['hello', '-nv']) == 0
    out = capsys.readouterr().out
    assert out.startswith('System role:\n' + constants.ROLE_SYSTEM_DEFAULT.format('rolling'))
    assert "FakeCompletion('fake answer')" in out
    assert out.endswith('fake answer\n')


def test_answer_not_finished(fake_openai, capsys):
    fake_openai.finish_reason = 'length'
    # the partial content is streamed before the reason
    assert run_command(['hello']) == 1
    assert capsys.readouterr().out == 'fake answer!\nFailed chat completion with: length\n'
    assert run_command(['hello', '-n']) == 1
    assert capsys.readouterr().out == 'Failed chat completion with: length\n'


def test_role_argument_and_env(fake_openai, monkeypatch):
    assert run_command(['hello', '-r', 'custom {} role']) == 0
    call = fake_openai.instances[0].create_calls[0]
    assert call['messages'][0] == {'role': 'system', 'content': 'custom rolling role'}
    monkeypatch.setenv(constants.ROLE_SYSTEM_ENV_VAR, 'env {} role')
    assert run_command(['hello']) == 0
    call = fake_openai.instances[1].create_calls[0]
    assert call['messages'][0] == {'role': 'system', 'content': 'env rolling role'}


def test_global_arguments(fake_openai, monkeypatch):
    monkeypatch.setenv(constants.ROS2AI_TEMPERATURE_ENV_VAR, '0.3')
    argv = ['hello', '-m', 'llama3.1', '-u', 'http://127.0.0.1:11434/v1', '-t', '100']
    assert run_command(argv) == 0
    client = fake_openai.instances[0]
    assert client.base_url == 'http://127.0.0.1:11434/v1'
    call = client.create_calls[0]
    assert call['model'] == 'llama3.1'
    assert call['max_tokens'] == 100
    assert call['temperature'] == 0.3


def test_status_prevails_over_request(fake_openai, capsys):
    assert run_command(['hello', '-s']) == 0
    assert capsys.readouterr().out == 'fake answer\n'
    call = fake_openai.instances[0].create_calls[0]
    assert call['messages'] == [{'role': 'user', 'content': 'Are you available?'}]


@pytest.fixture
def fake_confirm(monkeypatch):
    """Record the confirmation prompts, answering with `fake_confirm.answer`."""
    recorder = SimpleNamespace(answer=True, commands=[])

    def confirm(command):
        recorder.commands.append(command)
        return recorder.answer
    monkeypatch.setattr(agent_module, 'confirm_execution', confirm)
    return recorder


def test_dry_run(fake_openai, capsys):
    fake_openai.responses = [
        [('ros2_topic_list', {})],
        [('ros2_topic_pub', {'topic': '/chatter', 'type': 'std_msgs/msg/String'})],
        'never requested',
    ]
    FakeRos2.stdout = '/chatter [std_msgs/msg/String]\n'
    assert run_command(['publish on', '/chatter', '--dry-run']) == 0
    out = capsys.readouterr().out
    assert out == (
        '[ros2ai]: ros2 topic list -t\n/chatter [std_msgs/msg/String]\n'
        "Command Candidate: 'ros2 topic pub --once -w 0 /chatter std_msgs/msg/String'\n")
    assert FakeRos2.calls == [['ros2', 'topic', 'list', '-t']]
    client = fake_openai.instances[0]
    assert len(client.create_calls) == 2
    call = client.create_calls[0]
    assert call['stream'] is True
    # the conversation keeps growing after the call, only the prompt is checked here
    assert call['messages'][:2] == [
        {'role': 'system', 'content': constants.ROLE_SYSTEM_DEFAULT.format('rolling')},
        {'role': 'user', 'content': 'publish on /chatter'},
    ]
    tool_names = [tool['function']['name'] for tool in call['tools']]
    assert 'ros2_topic_list' in tool_names
    assert 'ros2_cli' in tool_names
    assert 'tool_choice' not in call


def test_verbose_dry_run(fake_openai, capsys):
    fake_openai.responses = [[('ros2_cli', {'arguments': ['daemon', 'stop']})]]
    assert run_command(['stop daemon', '-nv', '--dry-run']) == 0
    out = capsys.readouterr().out
    assert out.startswith('System role:\n' + constants.ROLE_SYSTEM_DEFAULT.format('rolling'))
    assert 'FakeCompletion(None)' in out
    assert 'Tool call: ros2_cli({"arguments": ["daemon", "stop"]})\n' in out
    assert out.endswith(
        "Command Candidate: 'ros2 daemon stop'\n"
        'Tool result: dry-run: the command was not executed.\n')
    assert FakeRos2.calls == []


def test_read_only_tools(fake_openai, capsys):
    fake_openai.responses = [
        [('ros2_node_list', {})],
        [('ros2_node_info', {'node': '/talker'})],
        'The /talker node publishes /chatter.',
    ]
    FakeRos2.stdout = '/talker\n'
    assert run_command(['give me', 'all nodes']) == 0
    assert FakeRos2.calls == [['ros2', 'node', 'list'], ['ros2', 'node', 'info', '/talker']]
    out = capsys.readouterr().out
    assert out == (
        '[ros2ai]: ros2 node list\n/talker\n'
        '[ros2ai]: ros2 node info /talker\n/talker\n'
        'The /talker node publishes /chatter.\n')
    calls = fake_openai.instances[0].create_calls
    assert len(calls) == 3
    assert calls[2]['messages'][2:] == [
        {'role': 'assistant', 'tool_calls': [{
            'id': 'call_0', 'type': 'function',
            'function': {'name': 'ros2_node_list', 'arguments': '{}'}}]},
        {'role': 'tool', 'tool_call_id': 'call_0', 'content': '/talker'},
        {'role': 'assistant', 'tool_calls': [{
            'id': 'call_0', 'type': 'function',
            'function': {'name': 'ros2_node_info', 'arguments': '{"node": "/talker"}'}}]},
        {'role': 'tool', 'tool_call_id': 'call_0', 'content': '/talker'},
    ]


def test_read_only_tools_nostream(fake_openai, capsys):
    fake_openai.responses = [[('ros2_node_list', {})], 'done']
    assert run_command(['give me', 'all nodes', '-n']) == 0
    assert FakeRos2.calls == [['ros2', 'node', 'list']]
    assert capsys.readouterr().out == '[ros2ai]: ros2 node list\nfake ros2 output\ndone\n'
    calls = fake_openai.instances[0].create_calls
    assert all(call['stream'] is False for call in calls)
    assert calls[1]['messages'][2]['tool_calls'][0]['id'] == 'call_0'


def test_mutating_tool_confirmed(fake_openai, fake_confirm, capsys):
    fake_openai.responses = [
        [('ros2_topic_pub', {'topic': '/chatter', 'type': 'std_msgs/msg/String',
                             'values': '{data: hello}'})],
        'published',
    ]
    assert run_command(['say hello', 'on /chatter']) == 0
    assert fake_confirm.commands == [
        "ros2 topic pub --once -w 0 /chatter std_msgs/msg/String '{data: hello}'"]
    assert FakeRos2.calls == [
        ['ros2', 'topic', 'pub', '--once', '-w', '0', '/chatter', 'std_msgs/msg/String',
         '{data: hello}']]
    assert capsys.readouterr().out.endswith('published\n')


def test_mutating_tool_declined(fake_openai, fake_confirm, capsys):
    fake_confirm.answer = False
    fake_openai.responses = [[('ros2_param_set', {'node': '/n', 'name': 'p', 'value': '1'})], 'ok']
    assert run_command(['set p']) == 0
    assert fake_confirm.commands == ['ros2 param set /n p 1']
    assert FakeRos2.calls == []
    tool_message = fake_openai.instances[0].create_calls[1]['messages'][3]
    assert tool_message['content'] == 'the user declined to execute the command.'


def test_mutating_tool_yes(fake_openai, capsys):
    fake_openai.responses = [
        [('ros2_cli', {'arguments': ['lifecycle', 'set', '/n', 'configure']})], 'configured']
    assert run_command(['configure /n', '-y']) == 0
    assert FakeRos2.calls == [['ros2', 'lifecycle', 'set', '/n', 'configure']]


def test_long_running_tool(fake_openai, fake_confirm, monkeypatch, capsys):
    executed = []
    monkeypatch.setattr(
        agent_module, 'run_executable', lambda *, command: executed.append(command) or 7)
    fake_openai.responses = [
        [('ros2_run', {'package': 'demo_nodes_cpp', 'executable': 'talker'})],
        'never requested',
    ]
    assert run_command(['start the talker']) == 7
    assert fake_confirm.commands == ['ros2 run demo_nodes_cpp talker']
    assert executed == [['ros2', 'run', 'demo_nodes_cpp', 'talker']]
    assert len(fake_openai.instances[0].create_calls) == 1
    assert capsys.readouterr().out == '[ros2ai]: ros2 run demo_nodes_cpp talker\n'


def test_shell_characters_are_literal(fake_openai):
    topic = '/chatter; rm -rf / $(whoami) > /tmp/x'
    fake_openai.responses = [[('ros2_topic_echo', {'topic': topic})], 'done']
    assert run_command(['echo it']) == 0
    assert FakeRos2.calls == [['ros2', 'topic', 'echo', '--once', topic]]


def test_tool_error_recovered(fake_openai, capsys):
    fake_openai.responses = [[('ros2_node_info', {})], [('ros2_node_info', {'node': '/t'})], 'ok']
    assert run_command(['info']) == 0
    assert FakeRos2.calls == [['ros2', 'node', 'info', '/t']]
    tool_message = fake_openai.instances[0].create_calls[1]['messages'][3]
    assert tool_message['content'] == "error: missing required parameter 'node'"
    assert '[ros2ai]: tool call ros2_node_info failed' in capsys.readouterr().out


def test_api_failure(fake_openai, capsys):
    fake_openai.error = RuntimeError('boom')
    assert run_command(['hello']) == 1
    assert capsys.readouterr().out == 'Failed to call LLM API: boom\n'
    assert len(fake_openai.instances[0].create_calls) == 1


def test_fallback_without_tools(fake_openai, capsys):
    fake_openai.responses = [
        RuntimeError('model does not support tools'),
        'Run `ros2 topic list` to see the topics.',
    ]
    assert run_command(['give me', 'all topics']) == 0
    out = capsys.readouterr().out
    assert out == (
        'Failed to call LLM API: model does not support tools\n'
        'Tool calling is not supported, answering without tools.\n'
        'Run `ros2 topic list` to see the topics.\n')
    assert FakeRos2.calls == []
    calls = fake_openai.instances[0].create_calls
    assert len(calls) == 2
    assert 'tools' in calls[0]
    assert 'tools' not in calls[1]
    assert calls[1]['messages'] == calls[0]['messages']


def test_fallback_failure(fake_openai, capsys):
    fake_openai.responses = [RuntimeError('no tool support'), RuntimeError('still failing')]
    assert run_command(['hello']) == 1
    assert capsys.readouterr().out.endswith('Failed to call LLM API: still failing\n')


def test_fallback_only_on_first_step(fake_openai, capsys):
    fake_openai.responses = [[('ros2_node_list', {})], RuntimeError('tool call id invalid')]
    assert run_command(['nodes']) == 1
    assert 'answering without tools' not in capsys.readouterr().out
    assert len(fake_openai.instances[0].create_calls) == 2


def test_anthropic_endpoint_headers(fake_openai):
    argv = ['hello', '-u', constants.ROS_ANTHROPIC_DEFAULT_ENDPOINT, '-m', 'claude-opus-5-5']
    assert run_command(argv) == 0
    client = fake_openai.instances[0]
    assert client.api_key == 'sk-test'
    assert client.base_url == constants.ROS_ANTHROPIC_DEFAULT_ENDPOINT
    assert client.default_headers == {
        'x-api-key': 'sk-test',
        'anthropic-version': constants.ROS_ANTHROPIC_API_VERSION,
    }
    assert client.create_calls[0]['model'] == 'claude-opus-5-5'


def test_legacy_environment_variables(fake_openai, monkeypatch):
    monkeypatch.delenv(constants.ROS2AI_API_KEY_ENV_VAR)
    monkeypatch.setenv(constants.ROS_OPENAI_API_KEY_ENV_VAR, 'sk-legacy')
    monkeypatch.setenv(constants.ROS_OPENAI_MODEL_NAME_ENV_VAR, 'llama3.1')
    monkeypatch.setenv(
        constants.ROS_OPENAI_ENDPOINT_ENV_VAR, constants.ROS_OLLAMA_DEFAULT_ENDPOINT)
    assert run_command(['hello']) == 0
    client = fake_openai.instances[0]
    assert client.api_key == 'sk-legacy'
    assert client.base_url == constants.ROS_OLLAMA_DEFAULT_ENDPOINT
    assert client.default_headers is None
    assert client.create_calls[0]['model'] == 'llama3.1'
    # generic variables prevail over the legacy ones
    monkeypatch.setenv(constants.ROS2AI_MODEL_NAME_ENV_VAR, 'gemma4')
    monkeypatch.setenv(constants.ROS2AI_ENDPOINT_ENV_VAR, 'http://10.0.0.1:11434/v1')
    assert run_command(['hello']) == 0
    client = fake_openai.instances[1]
    assert client.base_url == 'http://10.0.0.1:11434/v1'
    assert client.create_calls[0]['model'] == 'gemma4'


def test_docs_tools(fake_openai, monkeypatch, tmp_path, capsys):
    from test_docs import FakeDownload, make_archive, QOS_PAGE
    from ros2ai.api import docs
    page = 'Concepts/Intermediate/About-Quality-of-Service-Settings'
    url = f'https://docs.ros.org/en/rolling/{page}.html'
    monkeypatch.setenv(constants.ROS2AI_DOCS_CACHE_ENV_VAR, str(tmp_path))
    monkeypatch.setattr(docs, 'urlopen', FakeDownload(make_archive({page: QOS_PAGE})))
    fake_openai.responses = [
        [('search_ros2_docs', {'query': 'quality of service reliability'})],
        [('read_ros2_docs', {'page': page})],
        f'Reliable delivery guarantees the samples, see {url}',
    ]
    assert run_command(['what is the reliable QoS?']) == 0
    assert FakeRos2.calls == []
    out = capsys.readouterr().out
    assert out == (
        "[ros2ai]: search_ros2_docs(query='quality of service reliability')\n"
        f'[ros2ai]:   downloaded the ROS 2 rolling documentation to {tmp_path}/rolling\n'
        f'[ros2ai]:   {url}\n'
        f'[ros2ai]: read_ros2_docs(page={page})\n'
        f'[ros2ai]:   {url}\n'
        f'Reliable delivery guarantees the samples, see {url}\n')
    calls = fake_openai.instances[0].create_calls
    assert calls[0]['messages'][0]['content'] == \
        constants.ROLE_SYSTEM_DEFAULT.format('rolling')
    assert 'rolling documentation' in calls[0]['messages'][0]['content']
    names = {tool['function']['name'] for tool in calls[0]['tools']}
    assert {'search_ros2_docs', 'read_ros2_docs'} <= names
    search_message = calls[1]['messages'][3]
    assert search_message['role'] == 'tool'
    assert search_message['content'].startswith('query: quality of service reliability\n')
    assert f'  url: {url}\n' in search_message['content']
    read_message = calls[2]['messages'][5]
    assert read_message['content'].startswith(f'page: {page}\n')
    assert 'Quality of Service settings in ROS 2 Rolling' in read_message['content']


def test_exec_composite_tool(fake_openai, monkeypatch, capsys):
    fake_openai.responses = [
        [('get_node_status', {'node': '/talker', 'window': 1})],
        'The /talker node is healthy.',
    ]
    FakeRos2.outputs = {
        ('ros2', 'node', 'info', '/talker'): (
            '/talker\n  Subscribers:\n\n  Publishers:\n'
            '    /chatter: std_msgs/msg/String\n'
            '    /rosout: rcl_interfaces/msg/Log\n'
            '  Service Servers:\n'
            '    /talker/get_parameters: rcl_interfaces/srv/GetParameters\n'),
        ('ros2', 'param', 'list', '/talker'): '  use_sim_time\n',
    }

    class FakePopen:

        def __init__(self, argv, **kwargs):
            assert argv == ['ros2', 'topic', 'hz', '/chatter']
            assert kwargs.get('shell') is None
            assert kwargs['env']['PYTHONUNBUFFERED'] == '1'
            FakeRos2.calls.append(argv)
            self.returncode = None

        def poll(self):
            return self.returncode

        def send_signal(self, sig):
            self.returncode = 0

        def communicate(self, timeout=None):
            return 'average rate: 1.000\n\tmin: 1.000s max: 1.000s\n', None
    monkeypatch.setattr(subprocess, 'Popen', FakePopen)

    assert run_command(['is /talker healthy?']) == 0
    assert FakeRos2.calls == [
        ['ros2', 'node', 'info', '/talker'],
        ['ros2', 'topic', 'hz', '/chatter'],
        ['ros2', 'param', 'list', '/talker'],
    ]
    out = capsys.readouterr().out
    assert out.startswith(
        '[ros2ai]: get_node_status(node=/talker, window=1)\n'
        '[ros2ai]:   ros2 node info /talker\n'
        '[ros2ai]:   ros2 topic hz /chatter\n'
        '[ros2ai]:   ros2 param list /talker\n'
        'node: /talker\n'
        'found: true\n'
        'lifecycle: not a lifecycle node\n'
        'publishers:\n'
        '- topic: /chatter\n'
        '  type: std_msgs/msg/String\n'
        '  hz: 1.0\n')
    assert out.endswith('warnings: []\nThe /talker node is healthy.\n')
    tool_message = fake_openai.instances[0].create_calls[1]['messages'][3]
    assert tool_message['role'] == 'tool'
    assert tool_message['content'].startswith('node: /talker\nfound: true\n')
    assert 'parameters:\n- use_sim_time\n' in tool_message['content']


def session_turns():
    """Read the turns of the session file written by the command."""
    return json.loads(FakeOpenAI.session_file.read_text())['turns']


def test_request_saves_the_session(fake_openai):
    assert run_command(['what', 'is ros']) == 0
    data = json.loads(fake_openai.session_file.read_text())
    assert data['model'] == constants.ROS2AI_DEFAULT_MODEL
    assert data['endpoint'] == constants.ROS2AI_DEFAULT_ENDPOINT
    assert data['turns'] == [[
        {'role': 'user', 'content': 'what is ros'},
        {'role': 'assistant', 'content': 'fake answer!'},
    ]]


def test_continue_session(fake_openai, capsys):
    fake_openai.responses = [
        [('ros2_topic_list', {})], 'Yes, /chatter is available.',
        [('ros2_topic_echo', {'topic': '/chatter'})], 'Here is a message of /chatter.',
    ]
    FakeRos2.stdout = '/chatter [std_msgs/msg/String]\n'
    assert run_command(['do we have', '/chatter topic?']) == 0
    assert run_command(['-c', 'subscribe it']) == 0
    assert FakeRos2.calls == [
        ['ros2', 'topic', 'list', '-t'], ['ros2', 'topic', 'echo', '--once', '/chatter']]
    # the second command sends the whole first turn before the new request
    first = fake_openai.instances[1].create_calls[0]['messages']
    assert first[:2] == [
        {'role': 'system', 'content': constants.ROLE_SYSTEM_DEFAULT.format('rolling')},
        {'role': 'user', 'content': 'do we have /chatter topic?'},
    ]
    assert first[2]['tool_calls'][0]['function']['name'] == 'ros2_topic_list'
    assert first[3] == {
        'role': 'tool', 'tool_call_id': 'call_0', 'content': '/chatter [std_msgs/msg/String]'}
    assert first[4:] == [
        {'role': 'assistant', 'content': 'Yes, /chatter is available.'},
        {'role': 'user', 'content': 'subscribe it'},
    ]
    assert len(session_turns()) == 2
    assert 'no previous session' not in capsys.readouterr().out


def test_request_without_continue_starts_a_new_session(fake_openai):
    assert run_command(['first']) == 0
    assert run_command(['second']) == 0
    messages = fake_openai.instances[1].create_calls[0]['messages']
    assert [m['content'] for m in messages[1:]] == ['second']
    assert len(session_turns()) == 1


def test_continue_without_previous_session(fake_openai, capsys):
    assert run_command(['-c', 'hello']) == 0
    out = capsys.readouterr().out
    assert out.startswith('[ros2ai]: no previous session to continue (')
    assert out.endswith('fake answer!\n')
    assert len(session_turns()) == 1


def test_continue_invalid_session_file(fake_openai, capsys):
    fake_openai.session_file.parent.mkdir(parents=True)
    fake_openai.session_file.write_text('not json')
    assert run_command(['-c', 'hello']) == 0
    out = capsys.readouterr().out
    assert '[ros2ai]: failed to read the session file' in out
    assert len(session_turns()) == 1


def test_continue_verbose(fake_openai, capsys):
    assert run_command(['first']) == 0
    assert run_command(['-cv', 'second']) == 0
    out = capsys.readouterr().out
    assert '[ros2ai]: continuing the session of 1 turns, model: gpt-5, updated: ' in out


def test_failed_request_is_not_saved(fake_openai):
    assert run_command(['first']) == 0
    fake_openai.error = RuntimeError('boom')
    assert run_command(['-c', 'second']) == 1
    assert len(session_turns()) == 1


def test_session_save_failure_is_reported(fake_openai, monkeypatch, capsys):
    monkeypatch.setenv(constants.ROS2AI_SESSION_FILE_ENV_VAR, '/dev/null/session.json')
    assert run_command(['hello']) == 0
    out = capsys.readouterr().out
    assert 'fake answer!\n' in out
    assert '[ros2ai]: failed to save the session to /dev/null/session.json' in out


def test_context_limit_env(fake_openai, monkeypatch, capsys):
    # the system role is counted (about 175 tokens), each turn of 100 characters is about 45
    monkeypatch.setenv(constants.ROS2AI_CONTEXT_LIMIT_ENV_VAR, '240')
    assert run_command(['x' * 100]) == 0
    assert len(session_turns()) == 1
    assert run_command(['-c', 'y' * 100]) == 0
    # the first turn does not fit with the second one and is dropped
    messages = fake_openai.instances[1].create_calls[0]['messages']
    assert [m['content'] for m in messages[1:]] == ['y' * 100]
    assert len(session_turns()) == 1
    monkeypatch.setenv(constants.ROS2AI_CONTEXT_LIMIT_ENV_VAR, 'many')
    assert run_command(['hello']) == 0
    assert 'Invalid ROS2AI_CONTEXT_LIMIT: many, use default' in capsys.readouterr().out


def test_context_exceeded_retry(fake_openai, capsys):
    assert run_command(['first']) == 0
    fake_openai.responses = [RuntimeError('context_length_exceeded'), 'short answer']
    assert run_command(['-c', 'second']) == 0
    out = capsys.readouterr().out
    assert '[ros2ai]: the context length is exceeded, the oldest turn is dropped.' in out
    assert out.endswith('short answer\n')
    calls = fake_openai.instances[1].create_calls
    assert len(calls) == 2
    assert [m['content'] for m in calls[1]['messages'][1:]] == ['second']
    assert len(session_turns()) == 1


@pytest.fixture
def fake_input(monkeypatch):
    """Feed the interactive session with scripted lines, EOF at the end."""
    lines = []

    def read_line(prompt):
        print(prompt, end='')
        if not lines:
            raise EOFError()
        return lines.pop(0)
    monkeypatch.setattr('builtins.input', read_line)
    monkeypatch.setattr('sys.stdin', SimpleNamespace(isatty=lambda: False))
    return lines


def test_interactive_session(fake_openai, fake_input, capsys):
    fake_input.extend(['do we have /chatter?', 'subscribe it', 'exit'])
    fake_openai.responses = ['yes', 'done']
    assert run_command([]) == 0
    out = capsys.readouterr().out
    assert out == (
        f'ros2ai interactive session (model: {constants.ROS2AI_DEFAULT_MODEL}, '
        f'endpoint: {constants.ROS2AI_DEFAULT_ENDPOINT})\n'
        'Type /help for the commands, exit or Ctrl-D to end the session.\n'
        'ros2ai> yes\n'
        'ros2ai> done\n'
        'ros2ai> ')
    calls = fake_openai.instances[0].create_calls
    assert len(calls) == 2
    assert [m['content'] for m in calls[1]['messages'][1:]] == [
        'do we have /chatter?', 'yes', 'subscribe it']
    assert len(session_turns()) == 2


def test_interactive_session_clear_and_eof(fake_openai, fake_input, capsys):
    fake_input.extend(['first', '/clear', 'second'])
    assert run_command([]) == 0
    calls = fake_openai.instances[0].create_calls
    assert [m['content'] for m in calls[1]['messages'][1:]] == ['second']
    out = capsys.readouterr().out
    assert 'The context of the session is cleared.' in out
    assert out.endswith('ros2ai> \n')


def test_interactive_session_clear_saves_the_session(fake_openai, fake_input):
    fake_input.extend(['first', '/clear', 'exit'])
    assert run_command([]) == 0
    assert session_turns() == []


def test_interactive_session_continues(fake_openai, fake_input):
    assert run_command(['first']) == 0
    fake_input.extend(['second', 'exit'])
    assert run_command(['-c']) == 0
    calls = fake_openai.instances[1].create_calls
    assert [m['content'] for m in calls[0]['messages'][1:]] == ['first', 'fake answer!', 'second']
    assert len(session_turns()) == 2


def test_interactive_session_long_running_tool(fake_openai, fake_input, monkeypatch, capsys):
    executed = []
    monkeypatch.setattr(
        agent_module, 'run_executable', lambda *, command: executed.append(command) or 0)
    fake_input.extend(['start the talker', 'what did you run?', 'exit'])
    fake_openai.responses = [
        [('ros2_run', {'package': 'demo_nodes_cpp', 'executable': 'talker'})],
        'I ran the talker.',
    ]
    assert run_command(['-y']) == 0
    assert executed == [['ros2', 'run', 'demo_nodes_cpp', 'talker']]
    calls = fake_openai.instances[0].create_calls
    assert len(calls) == 2
    assert calls[1]['messages'][3] == {
        'role': 'tool', 'tool_call_id': 'call_0',
        'content': 'the command was executed in the foreground and exited with code 0.'}
    assert capsys.readouterr().out.endswith(
        'ros2ai> [ros2ai]: ros2 run demo_nodes_cpp talker\n'
        'ros2ai> I ran the talker.\n'
        'ros2ai> ')
