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

"""Conversation history of a session, its storage and the interactive prompt loop."""

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys
from typing import Callable, List, Optional

import ros2ai.api.constants as constants

# format version of the session file
SESSION_FILE_VERSION = 1


def default_session_file() -> Path:
    """
    Get the path of the session file.

    `ROS2AI_SESSION_FILE` prevails, otherwise the file is placed in the XDG state directory
    (`$XDG_STATE_HOME/ros2ai/session.json`, `~/.local/state` if not set).

    :return: path of the session file.
    """
    path = os.environ.get(constants.ROS2AI_SESSION_FILE_ENV_VAR)
    if path:
        return Path(path).expanduser()
    state_home = os.environ.get('XDG_STATE_HOME') or Path.home() / '.local' / 'state'
    return Path(state_home) / 'ros2ai' / constants.ROS2AI_SESSION_FILE_NAME


def estimate_tokens(messages: List[dict]) -> int:
    """
    Estimate the number of tokens of the messages.

    The tokenizer differs per model, so the estimate is based on the number of characters
    of the serialized messages.

    :param messages: chat completion messages.
    :return: estimated number of tokens.
    """
    size = sum(len(json.dumps(message, ensure_ascii=False)) for message in messages)
    return (size + constants.ROS2AI_CHARS_PER_TOKEN - 1) // constants.ROS2AI_CHARS_PER_TOKEN


class Conversation:
    """
    Messages of a session, grouped by turns.

    A turn starts with the user request and contains every message the agent loop appended
    (assistant messages, tool calls and tool results) until the final answer. The turns are
    the unit of trimming, so that a tool call is never separated from its results.
    The system role is kept outside of the history and prepended to every request.
    """

    def __init__(self, system_role: str, limit: int = constants.ROS2AI_CONTEXT_LIMIT):
        self.system_role = system_role
        # maximum estimated number of tokens of the messages, oldest turns are dropped beyond it
        self.limit = limit
        self._turns: List[List[dict]] = []
        self._pending: Optional[List[dict]] = None
        # number of turns dropped by the trimming, for the information of the user
        self.dropped = 0

    @property
    def turns(self) -> int:
        """Get the number of committed turns."""
        return len(self._turns)

    @property
    def active(self) -> bool:
        """Check if a turn is in progress."""
        return self._pending is not None

    def begin(self, request: str):
        """
        Start a turn with the user request, the previous unfinished turn is discarded.

        The oldest turns are dropped so that the request fits into the limit.
        """
        self._pending = [{'role': 'user', 'content': request}]
        self.trim()

    def append(self, message: dict):
        """Append a message to the turn in progress."""
        if self._pending is None:
            raise RuntimeError('no turn in progress')
        self._pending.append(message)

    def commit(self):
        """Keep the turn in progress in the history, dropping the oldest turns over the limit."""
        if self._pending is not None:
            self._turns.append(self._pending)
            self._pending = None
        self.trim()

    def abort(self):
        """Discard the turn in progress."""
        self._pending = None

    def clear(self):
        """Forget the whole history."""
        self._turns = []
        self._pending = None

    def messages(self) -> List[dict]:
        """Get the messages to be sent to the chat completion, system role first."""
        messages = [{'role': 'system', 'content': self.system_role}]
        for turn in self._turns:
            messages.extend(turn)
        if self._pending is not None:
            messages.extend(self._pending)
        return messages

    def drop_oldest(self) -> bool:
        """
        Drop the oldest committed turn.

        :return: True if a turn was dropped, False if the history is empty.
        """
        if not self._turns:
            return False
        del self._turns[0]
        self.dropped += 1
        return True

    def trim(self) -> int:
        """
        Drop the oldest turns while the estimated size exceeds the limit.

        The turn in progress is never dropped, the request would be lost.

        :return: number of dropped turns.
        """
        dropped = 0
        while self._turns and estimate_tokens(self.messages()) > self.limit:
            self.drop_oldest()
            dropped += 1
        return dropped

    def save(self, path: Path, metadata: Optional[dict] = None):
        """
        Save the committed turns to the session file.

        :param path: path of the session file, the parent directories are created.
        :param metadata: extra information stored with the history (e.g model, endpoint).
        """
        data = {
            'version': SESSION_FILE_VERSION,
            'updated': datetime.now(timezone.utc).isoformat(timespec='seconds'),
            **(metadata or {}),
            'turns': self._turns,
        }
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_name(path.name + '.tmp')
        tmp.write_text(json.dumps(data, ensure_ascii=False, indent=1) + '\n', encoding='utf-8')
        tmp.replace(path)

    def load(self, path: Path) -> dict:
        """
        Load the committed turns from the session file, replacing the history.

        :param path: path of the session file.
        :return: the metadata stored with the history.
        :raises ValueError: if the file is not a session file.
        """
        try:
            data = json.loads(path.read_text(encoding='utf-8'))
        except (OSError, ValueError) as e:
            raise ValueError(f'failed to read the session file {path}: {e}')
        turns = data.get('turns') if isinstance(data, dict) else None
        if not isinstance(turns, list) or data.get('version') != SESSION_FILE_VERSION:
            raise ValueError(f'{path} is not a ros2ai session file.')
        self._turns = [list(turn) for turn in turns if turn]
        self._pending = None
        self.trim()
        return {key: value for key, value in data.items() if key != 'turns'}


# words that end the interactive session, and the command that clears the history
EXIT_WORDS = ('exit', 'quit', '/exit', '/quit')
CLEAR_COMMAND = '/clear'
HELP_COMMAND = '/help'
HELP_TEXT = """\
Type your question or request, the assistant keeps the context of the session.
  /clear       forget the context of the session
  /help        print this help
  exit, quit   end the session (Ctrl-D), Ctrl-C aborts the current request
"""


def interactive_loop(
    conversation: Conversation,
    run_turn: Callable[[str], int],
    read_line: Optional[Callable[[str], str]] = None,
    on_clear: Optional[Callable[[], None]] = None,
    prompt: str = constants.ROS2AI_PROMPT,
) -> int:
    """
    Read the requests from the user until the session ends.

    Each request is processed by `run_turn` with the conversation, so that the follow-up
    requests are answered in the context of the previous ones. Ctrl-C aborts the request
    in progress and returns to the prompt, EOF (Ctrl-D) ends the session.

    :param conversation: conversation of the session, cleared by the `/clear` command.
    :param run_turn: callback processing one request, returning the return code.
    :param read_line: callback reading a line with the prompt, `input` if None.
    :param on_clear: callback invoked after the `/clear` command cleared the conversation.
    :param prompt: prompt printed before reading a request.
    :return: 0 when the session ends normally.
    """
    if read_line is None:
        read_line = input
        if sys.stdin.isatty():
            try:
                # line editing and history of the prompt
                import readline  # noqa: F401
            except ImportError:
                pass
    while True:
        try:
            line = read_line(prompt)
        except EOFError:
            print()
            return 0
        except KeyboardInterrupt:
            print()
            continue
        request = line.strip()
        if not request:
            continue
        if request.lower() in EXIT_WORDS:
            return 0
        if request == CLEAR_COMMAND:
            conversation.clear()
            if on_clear is not None:
                on_clear()
            print('The context of the session is cleared.')
            continue
        if request == HELP_COMMAND:
            print(HELP_TEXT, end='')
            continue
        try:
            run_turn(request)
        except KeyboardInterrupt:
            conversation.abort()
            print('\n[ros2ai]: the request is aborted.')
