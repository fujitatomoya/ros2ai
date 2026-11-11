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

"""Tests of the agent loop with fake client and provider, no network and no ROS 2 required."""

import json
from types import SimpleNamespace

import pytest
from ros2ai.api import agent as agent_module
from ros2ai.api.agent import Agent
from ros2ai.api.agent import AgentOptions
from ros2ai.api.agent import confirm_execution
from ros2ai.api.tools import ToolCommand
from ros2ai.api.tools import ToolError
from ros2ai.api.tools import ToolPolicy
from ros2ai.api.tools import ToolProvider
from ros2ai.api.tools import ToolResult


def tool_call(call_id, name, arguments):
    function = SimpleNamespace(name=name, arguments=json.dumps(arguments))
    return SimpleNamespace(id=call_id, type='function', function=function)


def message(content=None, tool_calls=None):
    return SimpleNamespace(content=content, tool_calls=tool_calls)


class FakeClient:
    """Stand-in for ChatCompletionClient returning scripted messages."""

    def __init__(self, messages):
        self._messages = list(messages)
        self.calls = []
        self._current = None

    def call(self, parameters):
        self.calls.append(parameters)
        self._current = self._messages.pop(0) if self._messages else None
        return self._current is not None

    def get_message(self):
        return self._current

    def get_error(self):
        return None

    def print_result(self):
        print('printed result')

    def print_all(self):
        print('printed all')


class FakeProvider(ToolProvider):
    """Provider with two tools, recording the executed commands."""

    def __init__(self):
        self.executed = []

    def definitions(self):
        return [{'type': 'function', 'function': {'name': 'read'}},
                {'type': 'function', 'function': {'name': 'write'}}]

    def resolve(self, name, arguments):
        policies = {
            'read': ToolPolicy.READ_ONLY,
            'write': ToolPolicy.MUTATING,
            'spin': ToolPolicy.LONG_RUNNING,
        }
        if name not in policies:
            raise ToolError(f"unknown tool '{name}'")
        return ToolCommand(['ros2', name, *arguments.get('args', [])], policies[name])

    def run(self, command):
        self.executed.append(command.argv)
        return ToolResult(f'output of {command.argv[1]}', 0)


@pytest.fixture
def no_confirm():
    return lambda command: pytest.fail(f'unexpected confirmation for {command}')


def test_answer_without_tools(capsys, no_confirm):
    client = FakeClient([message(content='just an answer')])
    agent = Agent(client, FakeProvider(), AgentOptions(), confirm=no_confirm)
    assert agent.run('system', 'request') == 0
    assert capsys.readouterr().out == 'just an answer\n'
    assert agent.steps == 1
    params = client.calls[0]
    assert params.stream is False
    assert params.tools == FakeProvider().definitions()
    assert params.messages == [
        {'role': 'system', 'content': 'system'},
        {'role': 'user', 'content': 'request'},
    ]


def test_answer_without_tools_offered(capsys, no_confirm):
    client = FakeClient([message(content='plain answer')])
    agent = Agent(client, FakeProvider(), AgentOptions(), confirm=no_confirm)
    assert agent.run('system', 'request', tools=False) == 0
    assert capsys.readouterr().out == 'plain answer\n'
    assert client.calls[0].tools is None


def test_read_only_tool_loop(capsys, no_confirm):
    client = FakeClient([
        message(tool_calls=[tool_call('c1', 'read', {'args': ['a']})]),
        message(content='done', tool_calls=[]),
    ])
    provider = FakeProvider()
    agent = Agent(client, provider, AgentOptions(), confirm=no_confirm)
    assert agent.run('system', 'request') == 0
    assert provider.executed == [['ros2', 'read', 'a']]
    assert capsys.readouterr().out == '[ros2ai]: ros2 read a\noutput of read\ndone\n'
    # conversation sent on the second step includes the assistant and the tool messages
    assert client.calls[1].messages[2:] == [
        {'role': 'assistant', 'tool_calls': [{
            'id': 'c1', 'type': 'function',
            'function': {'name': 'read', 'arguments': '{"args": ["a"]}'}}]},
        {'role': 'tool', 'tool_call_id': 'c1', 'content': 'output of read'},
    ]


def test_assistant_content_kept_with_tool_calls(no_confirm):
    client = FakeClient([
        message(content='let me check', tool_calls=[tool_call('c1', 'read', {})]),
        message(content='done'),
    ])
    agent = Agent(client, FakeProvider(), AgentOptions(), confirm=no_confirm)
    assert agent.run('system', 'request') == 0
    assert client.calls[1].messages[2]['content'] == 'let me check'


def test_parallel_tool_calls(no_confirm):
    client = FakeClient([
        message(tool_calls=[
            tool_call('c1', 'read', {}), tool_call('c2', 'read', {'args': ['b']})]),
        message(content='done'),
    ])
    provider = FakeProvider()
    agent = Agent(client, provider, AgentOptions(), confirm=no_confirm)
    assert agent.run('system', 'request') == 0
    assert provider.executed == [['ros2', 'read'], ['ros2', 'read', 'b']]
    tool_messages = client.calls[1].messages[3:]
    assert [m['tool_call_id'] for m in tool_messages] == ['c1', 'c2']


def test_mutating_tool_confirmed(capsys):
    client = FakeClient([
        message(tool_calls=[tool_call('c1', 'write', {'args': ['x']})]),
        message(content='done'),
    ])
    provider = FakeProvider()
    asked = []

    def confirm(command):
        asked.append(command)
        return True
    agent = Agent(client, provider, AgentOptions(), confirm=confirm)
    assert agent.run('system', 'request') == 0
    assert asked == ['ros2 write x']
    assert provider.executed == [['ros2', 'write', 'x']]


def test_mutating_tool_declined(capsys):
    client = FakeClient([
        message(tool_calls=[tool_call('c1', 'write', {})]),
        message(content='ok, not done'),
    ])
    provider = FakeProvider()
    agent = Agent(client, provider, AgentOptions(), confirm=lambda command: False)
    assert agent.run('system', 'request') == 0
    assert provider.executed == []
    assert client.calls[1].messages[3] == {
        'role': 'tool', 'tool_call_id': 'c1',
        'content': 'the user declined to execute the command.'}
    assert capsys.readouterr().out == 'ok, not done\n'


def test_mutating_tool_auto_approve(no_confirm):
    client = FakeClient([
        message(tool_calls=[tool_call('c1', 'write', {})]),
        message(content='done'),
    ])
    provider = FakeProvider()
    agent = Agent(client, provider, AgentOptions(auto_approve=True), confirm=no_confirm)
    assert agent.run('system', 'request') == 0
    assert provider.executed == [['ros2', 'write']]


def test_long_running_tool_runs_foreground(monkeypatch, capsys):
    client = FakeClient([
        message(tool_calls=[tool_call('c1', 'spin', {'args': ['talker']})]),
        message(content='never requested'),
    ])
    executed = []
    monkeypatch.setattr(
        agent_module, 'run_executable', lambda *, command: executed.append(command) or 42)
    provider = FakeProvider()
    agent = Agent(client, provider, AgentOptions(), confirm=lambda command: True)
    assert agent.run('system', 'request') == 42
    assert executed == [['ros2', 'spin', 'talker']]
    assert provider.executed == []
    assert len(client.calls) == 1
    assert capsys.readouterr().out == '[ros2ai]: ros2 spin talker\n'


def test_dry_run_executes_read_only_prints_candidates_and_stops(capsys, no_confirm):
    client = FakeClient([
        message(tool_calls=[tool_call('c1', 'read', {})]),
        message(tool_calls=[
            tool_call('c2', 'write', {'args': ['x']}), tool_call('c3', 'spin', {})]),
        message(content='never requested'),
    ])
    provider = FakeProvider()
    agent = Agent(client, provider, AgentOptions(dry_run=True), confirm=no_confirm)
    assert agent.run('system', 'request') == 0
    assert provider.executed == [['ros2', 'read']]
    assert len(client.calls) == 2
    assert agent.candidates == 2
    assert capsys.readouterr().out == (
        '[ros2ai]: ros2 read\noutput of read\n'
        "Command Candidate: 'ros2 write x'\n"
        "Command Candidate: 'ros2 spin'\n")


def test_dry_run_without_candidates_finishes(capsys, no_confirm):
    client = FakeClient([message(tool_calls=[tool_call('c1', 'read', {})]), message(content='ok')])
    agent = Agent(client, FakeProvider(), AgentOptions(dry_run=True), confirm=no_confirm)
    assert agent.run('system', 'request') == 0
    assert agent.candidates == 0
    assert capsys.readouterr().out.endswith('ok\n')


def test_tool_errors_returned_to_model(capsys, no_confirm):
    bad_json = SimpleNamespace(
        id='c2', type='function', function=SimpleNamespace(name='read', arguments='{bad'))
    client = FakeClient([
        message(tool_calls=[tool_call('c1', 'nope', {}), bad_json]),
        message(content='sorry'),
    ])
    provider = FakeProvider()
    agent = Agent(client, provider, AgentOptions(), confirm=no_confirm)
    assert agent.run('system', 'request') == 0
    assert provider.executed == []
    tool_messages = client.calls[1].messages[3:]
    assert tool_messages[0]['content'] == "error: unknown tool 'nope'"
    assert tool_messages[1]['content'].startswith('error: Expecting property name')
    out = capsys.readouterr().out
    assert "[ros2ai]: tool call nope failed, unknown tool 'nope'" in out
    assert '[ros2ai]: tool call read failed, Expecting property name' in out


def test_output_truncated_for_model_but_printed(capsys, no_confirm):
    class LongOutputProvider(FakeProvider):

        def run(self, command):
            return ToolResult('x' * 100, 0)

    client = FakeClient([message(tool_calls=[tool_call('c1', 'read', {})]), message(content='')])
    agent = Agent(
        client, LongOutputProvider(), AgentOptions(output_limit=30), confirm=no_confirm)
    assert agent.run('system', 'request') == 0
    content = client.calls[1].messages[3]['content']
    assert content == 'x' * 20 + '\n... [70 characters truncated] ...\n' + 'x' * 10
    assert 'x' * 100 in capsys.readouterr().out


def test_empty_output(no_confirm):
    class EmptyProvider(FakeProvider):

        def run(self, command):
            return ToolResult('', 0)

    client = FakeClient([message(tool_calls=[tool_call('c1', 'read', {})]), message(content='')])
    agent = Agent(client, EmptyProvider(), AgentOptions(), confirm=no_confirm)
    assert agent.run('system', 'request') == 0
    assert client.calls[1].messages[3]['content'] == '(no output)'


def test_max_steps(capsys, no_confirm):
    client = FakeClient([message(tool_calls=[tool_call('c1', 'read', {})])] * 5)
    agent = Agent(client, FakeProvider(), AgentOptions(max_steps=3), confirm=no_confirm)
    assert agent.run('system', 'request') == 1
    assert len(client.calls) == 3
    assert agent.steps == 3
    assert 'Reached the maximum number of steps (3).' in capsys.readouterr().out


def test_api_failure(capsys, no_confirm):
    agent = Agent(FakeClient([]), FakeProvider(), AgentOptions(), confirm=no_confirm)
    assert agent.run('system', 'request') == 1
    assert agent.steps == 1


def test_not_finished(capsys, no_confirm):
    class NotFinishedClient(FakeClient):

        def get_message(self):
            return None

    agent = Agent(
        NotFinishedClient([message(content='x')]), FakeProvider(), AgentOptions(),
        confirm=no_confirm)
    assert agent.run('system', 'request') == 1
    assert capsys.readouterr().out == 'printed result\n'


def test_verbose_prints(capsys, no_confirm):
    client = FakeClient([message(tool_calls=[tool_call('c1', 'read', {'args': ['a']})]),
                         message(content='done')])
    agent = Agent(client, FakeProvider(), AgentOptions(verbose=True), confirm=no_confirm)
    assert agent.run('system', 'request') == 0
    out = capsys.readouterr().out
    assert out.count('printed all') == 2
    assert 'Tool call: read({"args": ["a"]})\n' in out
    assert 'Tool result: output of read\n' in out


def test_execute_command(capsys, monkeypatch):
    provider = FakeProvider()
    agent = Agent(FakeClient([]), provider, AgentOptions(), confirm=lambda command: False)
    assert agent.execute_command(ToolCommand(['ros2', 'read'], ToolPolicy.READ_ONLY)) == 0
    assert provider.executed == [['ros2', 'read']]
    assert agent.execute_command(ToolCommand(['ros2', 'write'], ToolPolicy.MUTATING)) == 1
    agent = Agent(FakeClient([]), provider, AgentOptions(dry_run=True))
    assert agent.execute_command(ToolCommand(['ros2', 'write'], ToolPolicy.MUTATING)) == 0
    assert agent.execute_command(ToolCommand(['ros2', 'read'], ToolPolicy.READ_ONLY)) == 0
    assert provider.executed == [['ros2', 'read'], ['ros2', 'read']]
    assert capsys.readouterr().out.count("Command Candidate: 'ros2 write'") == 1
    monkeypatch.setattr(agent_module, 'run_executable', lambda *, command: 3)
    agent = Agent(FakeClient([]), provider, AgentOptions(auto_approve=True))
    assert agent.execute_command(ToolCommand(['ros2', 'spin'], ToolPolicy.LONG_RUNNING)) == 3


def test_confirm_execution_without_terminal(monkeypatch, capsys):
    monkeypatch.setattr('sys.stdin', SimpleNamespace(isatty=lambda: False))
    assert confirm_execution('ros2 write') is False
    assert 'requires a terminal, use --yes' in capsys.readouterr().out


@pytest.mark.parametrize('answer, expected', [
    ('y', True), ('Y', True), ('yes', True), (' YES ', True),
    ('', False), ('n', False), ('no', False), ('maybe', False),
])
def test_confirm_execution_answers(monkeypatch, answer, expected):
    monkeypatch.setattr('sys.stdin', SimpleNamespace(isatty=lambda: True))
    prompts = []

    def fake_input(prompt):
        prompts.append(prompt)
        return answer
    monkeypatch.setattr('builtins.input', fake_input)
    assert confirm_execution('ros2 write') is expected
    assert prompts == ["Execute 'ros2 write'? [y/N]: "]


def test_confirm_execution_eof(monkeypatch):
    monkeypatch.setattr('sys.stdin', SimpleNamespace(isatty=lambda: True))

    def fake_input(prompt):
        raise EOFError()
    monkeypatch.setattr('builtins.input', fake_input)
    assert confirm_execution('ros2 write') is False
