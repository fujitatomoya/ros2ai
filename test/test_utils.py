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

from ros2ai.api.utils import get_ros_distro
from ros2ai.api.utils import remove_backticks
from ros2ai.api.utils import ros2_single_command
from ros2ai.api.utils import run_executable
from ros2ai.api.utils import truncate_before_substring
from ros2ai.api.utils import truncate_output


def test_truncate_before_substring_found_with_newline():
    text = 'Sure, here it is:\nros2 topic list\nThat lists topics.'
    assert truncate_before_substring(original=text, substring='ros2') == 'ros2 topic list'


def test_truncate_before_substring_found_without_newline():
    text = 'answer: ros2 node list'
    assert truncate_before_substring(original=text, substring='ros2') == 'ros2 node list'


def test_truncate_before_substring_not_found():
    text = 'no command here\nat all'
    assert truncate_before_substring(original=text, substring='ros2') == text


def test_remove_backticks():
    assert remove_backticks('```ros2 topic list```') == 'ros2 topic list'
    assert remove_backticks('ros2 topic list') == 'ros2 topic list'


def test_ros2_single_command_passthrough(capsys):
    assert ros2_single_command('  ros2 topic list  ') == 'ros2 topic list'
    assert capsys.readouterr().out == ''


def test_ros2_single_command_multiple_returns_first_ros2(capsys):
    result = ros2_single_command('source setup.bash && ros2 topic list; ros2 node list')
    assert result == 'ros2 topic list'
    assert 'Warning: Multiple commands detected' in capsys.readouterr().out


def test_ros2_single_command_multiple_without_ros2_returns_first(capsys):
    assert ros2_single_command('echo a | grep b') == 'echo a'
    assert 'Warning: Multiple commands detected' in capsys.readouterr().out


def test_get_ros_distro_from_env(monkeypatch, capsys):
    monkeypatch.setenv('ROS_DISTRO', 'Kilted')
    assert get_ros_distro() == 'kilted'
    assert capsys.readouterr().out == ''


def test_get_ros_distro_fallback(monkeypatch, capsys):
    monkeypatch.delenv('ROS_DISTRO', raising=False)
    assert get_ros_distro() == 'rolling'
    assert 'ROS_DISTRO env value is not set, use rolling' in capsys.readouterr().out


def test_run_executable_return_code():
    assert run_executable(command='true') == 0
    assert run_executable(command=['false']) == 1
    assert run_executable(command=['python3', '-c', 'raise SystemExit(3)']) == 3


def test_run_executable_no_shell(capsys):
    # shell builtins, expansions and chaining are not available without a shell
    assert run_executable(command='exit 3') == 1
    assert 'failed to execute exit 3' in capsys.readouterr().out
    assert run_executable(command=['python3', '-c', 'import sys; sys.exit(len(sys.argv))',
                                   '$(true)', 'a; b']) == 3
    # `&&` and `true` are plain arguments of `false`, nothing is chained
    assert run_executable(command='false && true') == 1
    assert run_executable(command='') == 1
    assert run_executable(command=[]) == 1


def test_truncate_output():
    assert truncate_output('short', 10) == 'short'
    assert truncate_output('0123456789', 10) == '0123456789'
    assert truncate_output('0123456789abcdef', 12) == \
        '01234567\n... [4 characters truncated] ...\ncdef'
