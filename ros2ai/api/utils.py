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

import signal
import subprocess
import os

def run_command(*, command, argv = None, prefix = None):
    """
    Execute command line interface with arguments. (Blocking)

    :command: commands to be executed, possibly comes with arguments.
    :argv: extra arguments, added after command argument.
    :prefix: prefix added to before command argument.
    """
    if prefix is not None:
        command = prefix + command
    try:
        result = subprocess.run(
            command, shell = True, capture_output = True, text = True, check = True)
        print(result.stdout)
    except subprocess.CalledProcessError as ex:
        print(f"Error executing the command. Return Code: {ex.returncode} {ex.stderr}")
    except Exception as ex:
        print(f"Subprocess generates exception: {ex}")

def run_executable(*, command, argv = None, prefix=None):
    """
    Execute command line interface with arguments. (Interactive)

    :command: commands to be executed, possibly comes with arguments.
    :argv: extra arguments, added after command argument.
    :prefix: prefix added to before command argument.
    :return: the child process return code
    """
    if prefix is not None:
        command = prefix + command

    process = subprocess.Popen(command, shell = True)
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

    :return: string of distribution name.
    """
    distro = os.environ.get('ROS_DISTRO')
    if not distro:
        print('ROS_DISTRO env value is not set.')
        return None
    return distro.lower()
