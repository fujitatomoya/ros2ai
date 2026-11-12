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

"""Tests of the conversation history, the session file and the interactive loop."""

import json
from pathlib import Path

import pytest
import ros2ai.api.constants as constants
from ros2ai.api.session import Conversation
from ros2ai.api.session import default_session_file
from ros2ai.api.session import estimate_tokens
from ros2ai.api.session import interactive_loop
from ros2ai.api.session import SESSION_FILE_VERSION


def turn(conversation, request, answer):
    """Commit a plain turn of a request and its answer."""
    conversation.begin(request)
    conversation.append({'role': 'assistant', 'content': answer})
    conversation.commit()


def test_messages_of_turns():
    conversation = Conversation('system')
    assert conversation.messages() == [{'role': 'system', 'content': 'system'}]
    assert conversation.turns == 0
    assert conversation.active is False
    conversation.begin('first')
    assert conversation.active is True
    assert conversation.messages() == [
        {'role': 'system', 'content': 'system'},
        {'role': 'user', 'content': 'first'},
    ]
    conversation.append({'role': 'assistant', 'content': 'answer'})
    conversation.commit()
    assert conversation.turns == 1
    assert conversation.active is False
    conversation.begin('second')
    assert conversation.messages() == [
        {'role': 'system', 'content': 'system'},
        {'role': 'user', 'content': 'first'},
        {'role': 'assistant', 'content': 'answer'},
        {'role': 'user', 'content': 'second'},
    ]


def test_abort_discards_the_turn():
    conversation = Conversation('system')
    turn(conversation, 'first', 'answer')
    conversation.begin('second')
    conversation.append({'role': 'assistant', 'content': 'partial'})
    conversation.abort()
    assert conversation.turns == 1
    assert conversation.messages()[-1] == {'role': 'assistant', 'content': 'answer'}
    # a new request discards an unfinished turn as well
    conversation.begin('second')
    conversation.begin('third')
    assert conversation.messages()[-1] == {'role': 'user', 'content': 'third'}


def test_append_requires_a_turn():
    with pytest.raises(RuntimeError):
        Conversation('system').append({'role': 'assistant', 'content': 'x'})


def test_clear():
    conversation = Conversation('system')
    turn(conversation, 'first', 'answer')
    conversation.begin('second')
    conversation.clear()
    assert conversation.turns == 0
    assert conversation.active is False
    assert conversation.messages() == [{'role': 'system', 'content': 'system'}]


def test_estimate_tokens():
    messages = [{'role': 'user', 'content': 'x' * 100}]
    size = len(json.dumps(messages[0]))
    expected = (size + constants.ROS2AI_CHARS_PER_TOKEN - 1) // constants.ROS2AI_CHARS_PER_TOKEN
    assert estimate_tokens(messages) == expected
    assert estimate_tokens([]) == 0


def test_trim_drops_oldest_turns_as_a_whole():
    conversation = Conversation('system', limit=100)
    for index in range(5):
        conversation.begin(f'request {index}')
        conversation.append({'role': 'assistant', 'tool_calls': [
            {'id': f'c{index}', 'type': 'function',
             'function': {'name': 'read', 'arguments': '{}'}}]})
        conversation.append({'role': 'tool', 'tool_call_id': f'c{index}', 'content': 'x' * 50})
        conversation.append({'role': 'assistant', 'content': f'answer {index}'})
        conversation.commit()
    assert estimate_tokens(conversation.messages()) <= 100
    assert 0 < conversation.turns < 5
    assert conversation.dropped == 5 - conversation.turns
    messages = conversation.messages()
    # the oldest turns are gone, the kept turns are complete
    assert messages[1] == {'role': 'user', 'content': f'request {5 - conversation.turns}'}
    assert messages[-1] == {'role': 'assistant', 'content': 'answer 4'}
    tool_call_ids = [m['tool_call_id'] for m in messages if m['role'] == 'tool']
    called_ids = [m['tool_calls'][0]['id'] for m in messages if 'tool_calls' in m]
    assert tool_call_ids == called_ids


def test_begin_trims_to_the_limit():
    conversation = Conversation('system', limit=60)
    turn(conversation, 'first', 'x' * 100)
    assert conversation.turns == 1
    conversation.begin('y' * 100)
    assert conversation.turns == 0
    assert conversation.messages()[1:] == [{'role': 'user', 'content': 'y' * 100}]


def test_trim_never_drops_the_turn_in_progress():
    conversation = Conversation('system', limit=1)
    turn(conversation, 'first', 'answer')
    assert conversation.turns == 0
    conversation.begin('x' * 100)
    assert conversation.trim() == 0
    assert conversation.messages()[-1] == {'role': 'user', 'content': 'x' * 100}
    conversation.append({'role': 'assistant', 'content': 'answer'})
    conversation.commit()
    # the only turn exceeds the limit by itself and is dropped on commit
    assert conversation.turns == 0


def test_drop_oldest():
    conversation = Conversation('system')
    assert conversation.drop_oldest() is False
    turn(conversation, 'first', 'a')
    turn(conversation, 'second', 'b')
    assert conversation.drop_oldest() is True
    assert conversation.turns == 1
    assert conversation.dropped == 1
    assert conversation.messages()[1] == {'role': 'user', 'content': 'second'}


def test_save_and_load(tmp_path):
    path = tmp_path / 'state' / 'session.json'
    conversation = Conversation('system')
    turn(conversation, 'first', 'answer')
    conversation.begin('unfinished')
    conversation.save(path, {'model': 'gpt-x', 'endpoint': 'http://h/v1'})
    data = json.loads(path.read_text())
    assert data['version'] == SESSION_FILE_VERSION
    assert data['model'] == 'gpt-x'
    assert data['endpoint'] == 'http://h/v1'
    assert 'updated' in data
    # the turn in progress is not saved
    assert data['turns'] == [[
        {'role': 'user', 'content': 'first'},
        {'role': 'assistant', 'content': 'answer'},
    ]]
    assert not (tmp_path / 'state' / 'session.json.tmp').exists()

    loaded = Conversation('other system')
    metadata = loaded.load(path)
    assert metadata['model'] == 'gpt-x'
    assert 'turns' not in metadata
    assert loaded.turns == 1
    assert loaded.active is False
    assert loaded.messages() == [
        {'role': 'system', 'content': 'other system'},
        {'role': 'user', 'content': 'first'},
        {'role': 'assistant', 'content': 'answer'},
    ]


def test_load_trims_to_the_limit(tmp_path):
    path = tmp_path / 'session.json'
    conversation = Conversation('system')
    for index in range(3):
        turn(conversation, f'request {index}', 'x' * 200)
    conversation.save(path)
    loaded = Conversation('system', limit=100)
    loaded.load(path)
    assert loaded.turns == 1
    assert loaded.messages()[1] == {'role': 'user', 'content': 'request 2'}


@pytest.mark.parametrize('content', [
    'not json', '[]', '{"version": 999, "turns": []}', '{"version": 1, "turns": "x"}'])
def test_load_invalid_file(tmp_path, content):
    path = tmp_path / 'session.json'
    path.write_text(content)
    conversation = Conversation('system')
    turn(conversation, 'kept', 'answer')
    with pytest.raises(ValueError):
        conversation.load(path)
    # the history is kept on failure
    assert conversation.turns == 1


def test_load_missing_file(tmp_path):
    with pytest.raises(ValueError):
        Conversation('system').load(tmp_path / 'missing.json')


def test_default_session_file(monkeypatch, tmp_path):
    monkeypatch.delenv(constants.ROS2AI_SESSION_FILE_ENV_VAR, raising=False)
    monkeypatch.delenv('XDG_STATE_HOME', raising=False)
    monkeypatch.setenv('HOME', str(tmp_path))
    assert default_session_file() == (
        tmp_path / '.local' / 'state' / 'ros2ai' / constants.ROS2AI_SESSION_FILE_NAME)
    monkeypatch.setenv('XDG_STATE_HOME', str(tmp_path / 'xdg'))
    assert default_session_file() == tmp_path / 'xdg' / 'ros2ai' / 'session.json'
    monkeypatch.setenv(constants.ROS2AI_SESSION_FILE_ENV_VAR, '~/my-session.json')
    assert default_session_file() == tmp_path / 'my-session.json'


def scripted(lines):
    """Read the lines in order, raising the exceptions among them, EOF at the end."""
    lines = list(lines)
    prompts = []

    def read_line(prompt):
        prompts.append(prompt)
        if not lines:
            raise EOFError()
        line = lines.pop(0)
        if isinstance(line, BaseException):
            raise line
        return line
    read_line.prompts = prompts
    return read_line


def test_interactive_loop_requests(capsys):
    conversation = Conversation('system')
    requests = []
    read_line = scripted(['  hello  ', '', 'world'])
    assert interactive_loop(conversation, requests.append, read_line=read_line) == 0
    assert requests == ['hello', 'world']
    assert read_line.prompts == [constants.ROS2AI_PROMPT] * 4
    # the newline after EOF ends the prompt line
    assert capsys.readouterr().out == '\n'


@pytest.mark.parametrize('word', ['exit', 'quit', 'EXIT', '/exit', '/quit'])
def test_interactive_loop_exit(word):
    requests = []
    read_line = scripted([word, 'never read'])
    assert interactive_loop(Conversation('s'), requests.append, read_line=read_line) == 0
    assert requests == []
    assert len(read_line.prompts) == 1


def test_interactive_loop_clear_and_help(capsys):
    conversation = Conversation('system')
    turn(conversation, 'first', 'answer')
    cleared = []
    read_line = scripted(['/help', '/clear', 'exit'])
    assert interactive_loop(
        conversation, lambda request: 0, read_line=read_line,
        on_clear=lambda: cleared.append(True)) == 0
    assert conversation.turns == 0
    assert cleared == [True]
    out = capsys.readouterr().out
    assert '/clear' in out and 'exit, quit' in out
    assert 'The context of the session is cleared.' in out


def test_interactive_loop_keyboard_interrupt(capsys):
    conversation = Conversation('system')
    requests = []

    def run_turn(request):
        requests.append(request)
        conversation.begin(request)
        if request == 'abort me':
            raise KeyboardInterrupt()
        conversation.append({'role': 'assistant', 'content': 'ok'})
        conversation.commit()
        return 0
    # Ctrl-C at the prompt is ignored, Ctrl-C during the request aborts the turn only
    read_line = scripted([KeyboardInterrupt(), 'abort me', 'next'])
    assert interactive_loop(conversation, run_turn, read_line=read_line) == 0
    assert requests == ['abort me', 'next']
    assert conversation.turns == 1
    assert conversation.messages()[1] == {'role': 'user', 'content': 'next'}
    assert '[ros2ai]: the request is aborted.' in capsys.readouterr().out


def test_interactive_loop_default_input(monkeypatch, capsys):
    lines = ['hello', 'exit']
    monkeypatch.setattr('builtins.input', lambda prompt: lines.pop(0))
    monkeypatch.setattr('sys.stdin', type('Stdin', (), {'isatty': lambda self: False})())
    requests = []
    assert interactive_loop(Conversation('s'), requests.append) == 0
    assert requests == ['hello']


def test_session_file_is_a_path():
    assert isinstance(default_session_file(), Path)
