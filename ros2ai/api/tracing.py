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

"""
Trace of the request printed to the user, the tool calls and the executed commands.

The trace is not printed by default, the user asked a question and gets the answer.
It is enabled with the `--trace` option to see how the assistant completes the request:
the tool calls, the commands executed under them and their output.

The notices are printed regardless of the trace: the commands that change the system
(the user must know what was executed, with `--yes` nothing else tells), the failures
and the warnings.
"""

PREFIX = '[ros2ai]: '
# the lines under a tool call (sub-commands, pages, notices) are indented with the trace
NESTED_INDENT = '  '

_enabled = False


def enable(enabled: bool = True) -> None:
    """Enable or disable the trace for the process."""
    global _enabled
    _enabled = enabled


def enabled() -> bool:
    """Check if the trace is enabled."""
    return _enabled


def trace(line: str, nested: bool = False) -> None:
    """
    Print a line of the trace, only when the trace is enabled.

    :param line: the tool call or the command.
    :param nested: the line belongs under the previous tool call, e.g a sub-command.
    """
    if _enabled:
        print(PREFIX + (NESTED_INDENT if nested else '') + line)


def output(text: str) -> None:
    """Print the output of an executed command, only when the trace is enabled."""
    if _enabled:
        print(text)


def notice(line: str, nested: bool = False) -> None:
    """
    Print a notice to the user, regardless of the trace.

    :param line: the executed command that changes the system, the failure or the warning.
    :param nested: the line belongs under the previous tool call, indented only with the trace
        since the tool call is not printed otherwise.
    """
    print(PREFIX + (NESTED_INDENT if nested and _enabled else '') + line)
