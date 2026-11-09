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
import ros2ai.api.constants as constants
from ros2ai.verb.exec import ExecVerb
from ros2ai.verb.query import QueryVerb
from ros2ai.verb.status import StatusVerb


class FakeCompletion:

    def __init__(self, content, finish_reason='stop'):
        message = SimpleNamespace(content=content)
        self.choices = [SimpleNamespace(finish_reason=finish_reason, message=message)]

    def __repr__(self):
        return f'FakeCompletion({self.choices[0].message.content!r})'


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
    # patch the OpenAI class wherever ros2ai imported it
    for module_name in ('ros2ai.api.config', 'ros2ai.api.openai', 'ros2ai.api.client'):
        try:
            module = importlib.import_module(module_name)
        except ImportError:
            continue
        if hasattr(module, 'OpenAI'):
            monkeypatch.setattr(module, 'OpenAI', FakeOpenAI)

    # model listing may go through curl, serve the same model list there
    def fake_run(cmd, **kwargs):
        assert cmd[0] == 'curl'
        data = {'data': [{'id': model_id} for model_id in FakeOpenAI.model_ids]}
        return subprocess.CompletedProcess(cmd, 0, stdout=json.dumps(data), stderr='')
    monkeypatch.setattr(subprocess, 'run', fake_run)

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


def test_exec_dry_run(fake_openai, capsys):
    fake_openai.content = 'Sure:\n```\nros2 topic list && ros2 node list\n```'
    assert run_verb(ExecVerb(), ['give me', 'all topics', '--dry-run']) == 0
    out = capsys.readouterr().out
    assert 'Warning: Multiple commands detected' in out
    assert out.endswith("Command Candidate: 'ros2 topic list'\n")
    call = fake_openai.instances[0].create_calls[0]
    assert call['stream'] is False
    assert call['messages'] == [
        {'role': 'system', 'content': constants.ROLE_SYSTEM_EXEC_DEFAULT.format('rolling')},
        {'role': 'user', 'content': 'give me all topics'},
    ]


def test_exec_debug_dry_run(fake_openai, capsys):
    fake_openai.content = '`ros2 node list`'
    assert run_verb(ExecVerb(), ['nodes', '-d', '--dry-run']) == 0
    out = capsys.readouterr().out
    assert "FakeCompletion('`ros2 node list`')" in out
    assert 'System role:\n' + constants.ROLE_SYSTEM_EXEC_DEFAULT.format('rolling') in out
    assert out.endswith("Command Candidate: 'ros2 node list'\n")


def test_exec_runs_command(fake_openai):
    fake_openai.content = 'exit 5'
    assert run_verb(ExecVerb(), ['fail please']) == 5
    fake_openai.content = 'true'
    assert run_verb(ExecVerb(), ['succeed please']) == 0


def test_exec_role_argument(fake_openai):
    fake_openai.content = 'true'
    assert run_verb(ExecVerb(), ['hello', '-r', 'exec {} role']) == 0
    call = fake_openai.instances[0].create_calls[0]
    assert call['messages'][0] == {'role': 'system', 'content': 'exec rolling role'}


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
