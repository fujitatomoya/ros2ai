# Copyright 2023 Tomoya Fujita <tomoya.fujita825@gmail.com>.
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

import os
import re
import shlex
import signal
import subprocess


def run_executable(*, command) -> int:
    """
    Execute command line interface with arguments. (Interactive).

    The command is never executed via shell, so that no shell expansion or chaining happens.

    :param command: command to be executed as argument list, or a string split by shlex.
    :return: the child process return code.
    """
    argv = shlex.split(command) if isinstance(command, str) else list(command)
    if not argv:
        print('[ros2ai]: empty command, nothing to execute.')
        return 1
    try:
        process = subprocess.Popen(argv)
    except OSError as e:
        print(f'[ros2ai]: failed to execute {shlex.join(argv)}: {e}')
        return 1

    # add signal handler for the parent process, so that we can finalize the child process
    # child process could be `ros2 run` process and that also should pass the signal to
    # executables running underneath. finally executables can handle the signals.
    def signal_handler(sig, frame):
        print('[ros2ai]:', 'Received signal: ', signal.strsignal(sig))
        if process.poll() is None:
            # If child process is running, forward the signal to it
            process.send_signal(sig)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    while process.returncode is None:
        try:
            process.communicate()
        except KeyboardInterrupt:
            pass
    if process.returncode != 0:
        if -process.returncode in signal.valid_signals():
            print(signal.strsignal(-process.returncode))
    return process.returncode


def get_ros_distro() -> str:
    """
    Fetch ROS_DISTRO environmental variable.

    :return: string of distribution name, 'rolling' if not set.
    """
    distro = os.environ.get('ROS_DISTRO')
    if not distro:
        print('ROS_DISTRO env value is not set, use rolling')
        return 'rolling'
    return distro.lower()


def truncate_before_substring(*, original, substring) -> str:
    """
    Truncate a string to the line that starts at a specified substring.

    Anything before the substring and anything after the end of that line is removed.

    :param original: original string to be truncated.
    :param substring: substring before which the string should be truncated.
    :return: the line starting at the substring, or the original string if not found.
    """
    index = original.find(substring)
    if index == -1:
        return original
    return original[index:].split('\n', 1)[0]


def remove_backticks(original) -> str:
    """
    Remove all backticks from a given string.

    :param original: The original input string.
    :return: The string without backticks.
    """
    return original.replace('`', '')


def ros2_single_command(original) -> str:
    """
    Check if the command string contains only a single command (no command separators).

    If multiple commands are found, extract the first 'ros2' command and warn the user.

    :param original: The original command string to validate.
    :return: The validated single command string, or the first 'ros2' command if multiple found.
    """
    # Common command separators in shell
    separators = [';', '&&', '||', '&', '|']

    cleaned_command = original.strip()
    if not any(separator in cleaned_command for separator in separators):
        return cleaned_command

    print(f'Warning: Multiple commands detected in "{original}". '
          "Only the first 'ros2' command will be executed.")

    # Split by all separators and find first ros2 command
    pattern = '|'.join(re.escape(separator) for separator in separators)
    commands = [command.strip() for command in re.split(pattern, cleaned_command)]
    for command in commands:
        if command.startswith('ros2'):
            return command

    # If no ros2 command found, return the first command
    return commands[0] if commands else ''


def extract_ros2_command(response) -> str:
    """
    Extract the single ros2 command string from the chat completion response.

    :param response: content of the chat completion.
    :return: command string to be executed.
    """
    command = truncate_before_substring(original=response, substring='ros2')
    command = remove_backticks(command)
    return ros2_single_command(command)


def truncate_output(output: str, limit: int) -> str:
    """
    Truncate a command output to the limit, keeping the head and the tail.

    :param output: original output.
    :param limit: maximum number of characters.
    :return: the original output if within the limit, otherwise the truncated output.
    """
    if len(output) <= limit:
        return output
    head = limit * 2 // 3
    tail = limit - head
    removed = len(output) - limit
    return output[:head] + f'\n... [{removed} characters truncated] ...\n' + output[-tail:]
