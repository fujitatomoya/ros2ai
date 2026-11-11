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

"""End-to-end tests of the verbs with a fake OpenAI client, no network access required."""

import argparse
import importlib
import json
import subprocess
from types import SimpleNamespace

import pytest
from ros2ai.api import agent as agent_module
import ros2ai.api.constants as constants
from ros2ai.verb.exec import ExecVerb
from ros2ai.verb.query import QueryVerb
from ros2ai.verb.status import StatusVerb


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


def fake_chunks(contents):
    for content in contents:
        yield SimpleNamespace(choices=[SimpleNamespace(delta=SimpleNamespace(content=content))])


class FakeOpenAI:
    """Stand-in for openai.OpenAI recording constructor and completion arguments."""

    content = 'fake answer'
    finish_reason = 'stop'
    error = None
    model_ids = ['model-a', 'model-b']
    instances = []
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
        if FakeOpenAI.responses:
            response = FakeOpenAI.responses.pop(0)
            if isinstance(response, Exception):
                raise response
            if isinstance(response, list):
                return FakeCompletion(None, tool_calls=fake_tool_calls(response))
            return FakeCompletion(response)
        if kwargs.get('stream'):
            return fake_chunks([FakeOpenAI.content, None, '!'])
        return FakeCompletion(FakeOpenAI.content, FakeOpenAI.finish_reason)

    def _list_models(self):
        return [SimpleNamespace(id=model_id) for model_id in FakeOpenAI.model_ids]


@pytest.fixture
def fake_openai(monkeypatch):
    FakeOpenAI.content = 'fake answer'
    FakeOpenAI.finish_reason = 'stop'
    FakeOpenAI.error = None
    FakeOpenAI.model_ids = ['model-a', 'model-b']
    FakeOpenAI.instances = []
    FakeOpenAI.responses = []
    FakeRos2.stdout = 'fake ros2 output'
    FakeRos2.returncode = 0
    FakeRos2.calls = []
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
            return subprocess.CompletedProcess(
                cmd, FakeRos2.returncode, stdout=FakeRos2.stdout, stderr='')
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
        constants.ROS_OPENAI_API_KEY_ENV_VAR,
        constants.ROS_OPENAI_MODEL_NAME_ENV_VAR,
        constants.ROS_OPENAI_ENDPOINT_ENV_VAR,
        constants.ROS_OPENAI_TEMPERATURE_ENV_VAR,
        constants.ROLE_SYSTEM_ENV_VAR,
    ):
        monkeypatch.delenv(name, raising=False)
    monkeypatch.setenv(constants.ROS2AI_API_KEY_ENV_VAR, 'sk-test')
    monkeypatch.setenv('ROS_DISTRO', 'rolling')
    return FakeOpenAI


def run_verb(verb, argv):
    parser = argparse.ArgumentParser(prog='ros2 ai')
    verb.add_arguments(parser, 'ros2 ai')
    return verb.main(args=parser.parse_args(argv))


def test_status(fake_openai, capsys):
    assert run_verb(StatusVerb(), []) == 0
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
    assert run_verb(StatusVerb(), ['-vl', '-m', 'gpt-x', '-t', '7']) == 0
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
    assert run_verb(StatusVerb(), []) == 1
    out = capsys.readouterr().out
    assert 'Failed to call LLM API: boom' in out
    assert '[FAILURE] Invalid API key.' in out


def test_status_not_finished(fake_openai, capsys):
    fake_openai.finish_reason = 'length'
    assert run_verb(StatusVerb(), []) == 1
    out = capsys.readouterr().out
    assert 'Failed chat completion with: length' in out
    assert '[FAILURE] Invalid API key.' in out


def test_query_stream(fake_openai, capsys):
    assert run_verb(QueryVerb(), ['what', 'is ros']) == 0
    assert capsys.readouterr().out == 'fake answer!\n\n'
    call = fake_openai.instances[0].create_calls[0]
    assert call['stream'] is True
    assert call['messages'] == [
        {'role': 'system', 'content': constants.ROLE_SYSTEM_QUERY_DEFAULT.format('rolling')},
        {'role': 'user', 'content': 'what is ros'},
    ]


def test_query_nostream(fake_openai, capsys):
    assert run_verb(QueryVerb(), ['hello', '-n']) == 0
    assert capsys.readouterr().out == 'fake answer\n'
    assert fake_openai.instances[0].create_calls[0]['stream'] is False


def test_query_nostream_verbose(fake_openai, capsys):
    assert run_verb(QueryVerb(), ['hello', '-nv']) == 0
    out = capsys.readouterr().out
    assert "FakeCompletion('fake answer')" in out
    assert 'System role:\n' + constants.ROLE_SYSTEM_QUERY_DEFAULT.format('rolling') in out


def test_query_verbose_requires_nostream(fake_openai, capsys):
    assert run_verb(QueryVerb(), ['hello', '-v']) == 0
    assert capsys.readouterr().out == 'fake answer!\n\n'


def test_query_role_argument_and_env(fake_openai, monkeypatch):
    assert run_verb(QueryVerb(), ['hello', '-r', 'custom {} role']) == 0
    call = fake_openai.instances[0].create_calls[0]
    assert call['messages'][0] == {'role': 'system', 'content': 'custom rolling role'}
    monkeypatch.setenv(constants.ROLE_SYSTEM_ENV_VAR, 'env {} role')
    assert run_verb(QueryVerb(), ['hello']) == 0
    call = fake_openai.instances[1].create_calls[0]
    assert call['messages'][0] == {'role': 'system', 'content': 'env rolling role'}


def test_query_global_arguments(fake_openai, monkeypatch):
    monkeypatch.setenv(constants.ROS2AI_TEMPERATURE_ENV_VAR, '0.3')
    argv = ['hello', '-m', 'llama3.1', '-u', 'http://127.0.0.1:11434/v1', '-t', '100']
    assert run_verb(QueryVerb(), argv) == 0
    client = fake_openai.instances[0]
    assert client.base_url == 'http://127.0.0.1:11434/v1'
    call = client.create_calls[0]
    assert call['model'] == 'llama3.1'
    assert call['max_tokens'] == 100
    assert call['temperature'] == 0.3


@pytest.fixture
def fake_confirm(monkeypatch):
    """Record the confirmation prompts, answering with `fake_confirm.answer`."""
    recorder = SimpleNamespace(answer=True, commands=[])

    def confirm(command):
        recorder.commands.append(command)
        return recorder.answer
    monkeypatch.setattr(agent_module, 'confirm_execution', confirm)
    return recorder


def test_exec_dry_run(fake_openai, capsys):
    fake_openai.responses = [
        [('ros2_topic_list', {})],
        [('ros2_topic_pub', {'topic': '/chatter', 'type': 'std_msgs/msg/String'})],
        'never requested',
    ]
    FakeRos2.stdout = '/chatter [std_msgs/msg/String]\n'
    assert run_verb(ExecVerb(), ['publish on', '/chatter', '--dry-run']) == 0
    out = capsys.readouterr().out
    assert out == (
        '[ros2ai]: ros2 topic list -t\n/chatter [std_msgs/msg/String]\n'
        "Command Candidate: 'ros2 topic pub --once -w 0 /chatter std_msgs/msg/String'\n")
    assert FakeRos2.calls == [['ros2', 'topic', 'list', '-t']]
    client = fake_openai.instances[0]
    assert len(client.create_calls) == 2
    call = client.create_calls[0]
    assert call['stream'] is False
    # the conversation keeps growing after the call, only the prompt is checked here
    assert call['messages'][:2] == [
        {'role': 'system', 'content': constants.ROLE_SYSTEM_EXEC_DEFAULT.format('rolling')},
        {'role': 'user', 'content': 'publish on /chatter'},
    ]
    tool_names = [tool['function']['name'] for tool in call['tools']]
    assert 'ros2_topic_list' in tool_names
    assert 'ros2_cli' in tool_names
    assert 'tool_choice' not in call


def test_exec_debug_dry_run(fake_openai, capsys):
    fake_openai.responses = [[('ros2_cli', {'arguments': ['daemon', 'stop']})]]
    assert run_verb(ExecVerb(), ['stop daemon', '-d', '--dry-run']) == 0
    out = capsys.readouterr().out
    assert out.startswith('System role:\n' + constants.ROLE_SYSTEM_EXEC_DEFAULT.format('rolling'))
    assert 'FakeCompletion(None)' in out
    assert 'Tool call: ros2_cli({"arguments": ["daemon", "stop"]})\n' in out
    assert out.endswith(
        "Command Candidate: 'ros2 daemon stop'\n"
        'Tool result: dry-run: the command was not executed.\n')
    assert FakeRos2.calls == []


def test_exec_read_only_tools(fake_openai, capsys):
    fake_openai.responses = [
        [('ros2_node_list', {})],
        [('ros2_node_info', {'node': '/talker'})],
        'The /talker node publishes /chatter.',
    ]
    FakeRos2.stdout = '/talker\n'
    assert run_verb(ExecVerb(), ['give me', 'all nodes']) == 0
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


def test_exec_mutating_tool_confirmed(fake_openai, fake_confirm, capsys):
    fake_openai.responses = [
        [('ros2_topic_pub', {'topic': '/chatter', 'type': 'std_msgs/msg/String',
                             'values': '{data: hello}'})],
        'published',
    ]
    assert run_verb(ExecVerb(), ['say hello', 'on /chatter']) == 0
    assert fake_confirm.commands == [
        "ros2 topic pub --once -w 0 /chatter std_msgs/msg/String '{data: hello}'"]
    assert FakeRos2.calls == [
        ['ros2', 'topic', 'pub', '--once', '-w', '0', '/chatter', 'std_msgs/msg/String',
         '{data: hello}']]
    assert capsys.readouterr().out.endswith('published\n')


def test_exec_mutating_tool_declined(fake_openai, fake_confirm, capsys):
    fake_confirm.answer = False
    fake_openai.responses = [[('ros2_param_set', {'node': '/n', 'name': 'p', 'value': '1'})], 'ok']
    assert run_verb(ExecVerb(), ['set p']) == 0
    assert fake_confirm.commands == ['ros2 param set /n p 1']
    assert FakeRos2.calls == []
    tool_message = fake_openai.instances[0].create_calls[1]['messages'][3]
    assert tool_message['content'] == 'the user declined to execute the command.'


def test_exec_mutating_tool_yes(fake_openai, capsys):
    fake_openai.responses = [
        [('ros2_cli', {'arguments': ['lifecycle', 'set', '/n', 'configure']})], 'configured']
    assert run_verb(ExecVerb(), ['configure /n', '-y']) == 0
    assert FakeRos2.calls == [['ros2', 'lifecycle', 'set', '/n', 'configure']]


def test_exec_long_running_tool(fake_openai, fake_confirm, monkeypatch, capsys):
    executed = []
    monkeypatch.setattr(
        agent_module, 'run_executable', lambda *, command: executed.append(command) or 7)
    fake_openai.responses = [
        [('ros2_run', {'package': 'demo_nodes_cpp', 'executable': 'talker'})],
        'never requested',
    ]
    assert run_verb(ExecVerb(), ['start the talker']) == 7
    assert fake_confirm.commands == ['ros2 run demo_nodes_cpp talker']
    assert executed == [['ros2', 'run', 'demo_nodes_cpp', 'talker']]
    assert len(fake_openai.instances[0].create_calls) == 1
    assert capsys.readouterr().out == '[ros2ai]: ros2 run demo_nodes_cpp talker\n'


def test_exec_shell_characters_are_literal(fake_openai):
    topic = '/chatter; rm -rf / $(whoami) > /tmp/x'
    fake_openai.responses = [[('ros2_topic_echo', {'topic': topic})], 'done']
    assert run_verb(ExecVerb(), ['echo it']) == 0
    assert FakeRos2.calls == [['ros2', 'topic', 'echo', '--once', topic]]


def test_exec_tool_error_recovered(fake_openai, capsys):
    fake_openai.responses = [[('ros2_node_info', {})], [('ros2_node_info', {'node': '/t'})], 'ok']
    assert run_verb(ExecVerb(), ['info']) == 0
    assert FakeRos2.calls == [['ros2', 'node', 'info', '/t']]
    tool_message = fake_openai.instances[0].create_calls[1]['messages'][3]
    assert tool_message['content'] == "error: missing required parameter 'node'"
    assert '[ros2ai]: tool call ros2_node_info failed' in capsys.readouterr().out


def test_exec_api_failure(fake_openai, capsys):
    fake_openai.error = RuntimeError('boom')
    assert run_verb(ExecVerb(), ['hello']) == 1
    assert capsys.readouterr().out == 'Failed to call LLM API: boom\n'
    assert len(fake_openai.instances[0].create_calls) == 1


def test_exec_fallback_single_command(fake_openai, capsys):
    fake_openai.responses = [
        RuntimeError('model does not support tools'),
        'Sure:\n```\nros2 topic list && ros2 node list\n```',
    ]
    FakeRos2.stdout = '/chatter\n'
    assert run_verb(ExecVerb(), ['give me', 'all topics']) == 0
    out = capsys.readouterr().out
    assert 'Failed to call LLM API: model does not support tools\n' in out
    assert 'Tool calling is not supported, falling back to the single command mode.\n' in out
    assert 'Warning: Multiple commands detected' in out
    assert out.endswith('[ros2ai]: ros2 topic list\n/chatter\n')
    assert FakeRos2.calls == [['ros2', 'topic', 'list']]
    calls = fake_openai.instances[0].create_calls
    assert len(calls) == 2
    assert 'tools' in calls[0]
    assert 'tools' not in calls[1]
    assert calls[1]['messages'][0] == {
        'role': 'system', 'content': constants.ROLE_SYSTEM_EXEC_COMMAND.format('rolling')}


def test_exec_fallback_dry_run_and_debug(fake_openai, capsys):
    fake_openai.responses = [RuntimeError('no tool support'), '`ros2 param set /n p 1`']
    assert run_verb(ExecVerb(), ['set p', '-d', '--dry-run']) == 0
    out = capsys.readouterr().out
    assert "FakeCompletion('`ros2 param set /n p 1`')" in out
    assert 'System role:\n' + constants.ROLE_SYSTEM_EXEC_COMMAND.format('rolling') in out
    assert out.endswith("Command Candidate: 'ros2 param set /n p 1'\n")
    assert FakeRos2.calls == []
    # read-only commands are executed even in dry-run
    fake_openai.responses = [RuntimeError('no tool support'), 'ros2 node list']
    assert run_verb(ExecVerb(), ['nodes', '--dry-run']) == 0
    assert FakeRos2.calls == [['ros2', 'node', 'list']]


def test_exec_fallback_mutating_confirmed(fake_openai, fake_confirm):
    fake_openai.responses = [RuntimeError('no tool support'), 'ros2 param set /n p 1']
    FakeRos2.returncode = 2
    assert run_verb(ExecVerb(), ['set p']) == 2
    assert fake_confirm.commands == ['ros2 param set /n p 1']
    assert FakeRos2.calls == [['ros2', 'param', 'set', '/n', 'p', '1']]


def test_exec_fallback_rejects_non_ros2(fake_openai, capsys):
    fake_openai.responses = [RuntimeError('no tool support'), 'ls -la; rm -rf /']
    assert run_verb(ExecVerb(), ['list files']) == 1
    assert "Not a ros2 command: 'ls -la'" in capsys.readouterr().out
    assert FakeRos2.calls == []


def test_exec_fallback_only_on_first_step(fake_openai, capsys):
    fake_openai.responses = [[('ros2_node_list', {})], RuntimeError('tool call id invalid')]
    assert run_verb(ExecVerb(), ['nodes']) == 1
    assert 'falling back' not in capsys.readouterr().out
    assert len(fake_openai.instances[0].create_calls) == 2


def test_exec_role_argument(fake_openai, capsys):
    fake_openai.responses = ['nothing to do']
    assert run_verb(ExecVerb(), ['hello', '-r', 'exec {} role']) == 0
    call = fake_openai.instances[0].create_calls[0]
    assert call['messages'][0] == {'role': 'system', 'content': 'exec rolling role'}
    assert capsys.readouterr().out == 'nothing to do\n'


def test_anthropic_endpoint_headers(fake_openai):
    argv = ['hello', '-u', constants.ROS_ANTHROPIC_DEFAULT_ENDPOINT, '-m', 'claude-opus-5-5']
    assert run_verb(QueryVerb(), argv) == 0
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
    assert run_verb(QueryVerb(), ['hello']) == 0
    client = fake_openai.instances[0]
    assert client.api_key == 'sk-legacy'
    assert client.base_url == constants.ROS_OLLAMA_DEFAULT_ENDPOINT
    assert client.default_headers is None
    assert client.create_calls[0]['model'] == 'llama3.1'
    # generic variables prevail over the legacy ones
    monkeypatch.setenv(constants.ROS2AI_MODEL_NAME_ENV_VAR, 'gemma4')
    monkeypatch.setenv(constants.ROS2AI_ENDPOINT_ENV_VAR, 'http://10.0.0.1:11434/v1')
    assert run_verb(QueryVerb(), ['hello']) == 0
    client = fake_openai.instances[1]
    assert client.base_url == 'http://10.0.0.1:11434/v1'
    assert client.create_calls[0]['model'] == 'gemma4'
