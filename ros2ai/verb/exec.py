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

from ros2ai.api import add_global_arguments
from ros2ai.api.config import get_role_system
from ros2ai.api.constants import ROLE_SYSTEM_EXEC_DEFAULT
from ros2ai.api.openai import ChatCompletionClient, ChatCompletionParameters
from ros2ai.api.utils import get_ros_distro, run_executable, truncate_before_substring, remove_backticks
from ros2ai.verb import VerbExtension


class ExecVerb(VerbExtension):
    """Execute a ROS 2 CLI based on user query to OpenAI API."""

    def add_arguments(self, parser, cli_name):
        # add global arguments
        add_global_arguments(parser)

        # sub-command arguments
        parser.add_argument(
            "request", nargs="+", default=[],
            help="Multiple requests separated by spaces, enclosed in double quotes.")
        parser.add_argument(
            '-d',
            '--debug',
            action='store_true',
            help='Prints detailed information and behavior of OpenAI (debug use only)')
        parser.add_argument(
            '--dry-run',
            action='store_true',
            help='Prints the command instead of executing it.')
        parser.add_argument(
            '-r',
            '--role',
            metavar='<role>',
            type=str,
            default=None,
            help='Define the prompt\'s system role.')

    def main(self, *, args):
        request = ''
        if (args.request):
            request = " ".join(args.request)
        if (request == ''):
            print('Please insert your request! (I am not AI)')

        distro = get_ros_distro()
        system_role = get_role_system(default_role_system=ROLE_SYSTEM_EXEC_DEFAULT)
        if args.role and args.role != system_role:
            system_role = args.role
        system_role = system_role.format(distro)
        user_request = [
            {"role": "system", "content": f"{system_role}"},
            {"role": "user", "content": f"{request}"}
        ]

        completion_params = ChatCompletionParameters(messages = user_request, stream = False)
        client = ChatCompletionClient(args)
        client.call(completion_params)
        if (args.debug is True):
            client.print_all()
            print(f"System role:\n{system_role}")
        command_str = truncate_before_substring(
            original = client.get_result(), substring = 'ros2')
        command_str = remove_backticks(command_str)
        if not args.dry_run:
            return_code = run_executable(command = command_str)
            return return_code
        else:
            print(f"Command Candidate: '{command_str}'")
            return 0
