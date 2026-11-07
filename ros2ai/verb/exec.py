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

from ros2ai.api.arguments import add_global_arguments, add_role_argument
from ros2ai.api.client import chat_messages, ChatCompletionClient, ChatCompletionParameters
from ros2ai.api.config import OpenAiConfig, resolve_role_system
from ros2ai.api.constants import ROLE_SYSTEM_EXEC_DEFAULT
from ros2ai.api.utils import extract_ros2_command, run_executable
from ros2ai.verb import VerbExtension


class ExecVerb(VerbExtension):
    """Execute a ROS 2 CLI based on user query to OpenAI API."""

    def add_arguments(self, parser, cli_name):
        # add global arguments
        add_global_arguments(parser)

        # sub-command arguments
        parser.add_argument(
            'request', nargs='+', default=[],
            help='Multiple requests separated by spaces, enclosed in double quotes.')
        parser.add_argument(
            '-d',
            '--debug',
            action='store_true',
            help='Prints detailed information and behavior of OpenAI (debug use only)')
        parser.add_argument(
            '--dry-run',
            action='store_true',
            help='Prints the command instead of executing it.')
        add_role_argument(parser)

    def main(self, *, args):
        request = ' '.join(args.request)
        if request == '':
            print('Please insert your request! (I am not AI)')

        system_role = resolve_role_system(args, ROLE_SYSTEM_EXEC_DEFAULT)
        completion_params = ChatCompletionParameters(
            messages=chat_messages(system_role, request), stream=False)
        client = ChatCompletionClient(OpenAiConfig(args))
        if not client.call(completion_params):
            return 1
        if args.debug:
            client.print_all()
            print(f'System role:\n{system_role}')

        response = client.get_result()
        if response is None:
            # completion did not finish, the reason is printed
            client.print_result()
            return 1
        command_str = extract_ros2_command(response)
        if args.dry_run:
            print(f"Command Candidate: '{command_str}'")
            return 0
        return run_executable(command=command_str)
