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

import shlex

from ros2ai.api.agent import Agent, AgentOptions
from ros2ai.api.arguments import add_global_arguments, add_role_argument
from ros2ai.api.client import chat_messages, ChatCompletionClient, ChatCompletionParameters
from ros2ai.api.config import OpenAiConfig, resolve_role_system
from ros2ai.api.constants import ROLE_SYSTEM_EXEC_COMMAND, ROLE_SYSTEM_EXEC_DEFAULT
from ros2ai.api.tools import GENERIC_TOOL_NAME, Ros2CliProvider, ToolError
from ros2ai.api.utils import extract_ros2_command
from ros2ai.verb import VerbExtension


class ExecVerb(VerbExtension):
    """Operate ROS 2 system based on user request, LLM calls the ROS 2 CLI tools."""

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
            help='Prints detailed information and behavior of LLM (debug use only)')
        parser.add_argument(
            '--dry-run',
            action='store_true',
            help='Prints the commands that change the system instead of executing them, '
                 'read-only commands are still executed.')
        parser.add_argument(
            '-y',
            '--yes',
            action='store_true',
            help='Executes the commands that change the system without confirmation.')
        add_role_argument(parser)

    def main(self, *, args):
        request = ' '.join(args.request)
        if request == '':
            print('Please insert your request! (I am not AI)')

        options = AgentOptions(dry_run=args.dry_run, auto_approve=args.yes, debug=args.debug)
        client = ChatCompletionClient(OpenAiConfig(args))
        provider = Ros2CliProvider()
        agent = Agent(client, provider, options)

        system_role = resolve_role_system(args, ROLE_SYSTEM_EXEC_DEFAULT)
        if args.debug:
            print(f'System role:\n{system_role}')
        returncode = agent.run(system_role, request)
        if returncode != 1 or agent.steps != 1 or not _tools_unsupported(client.get_error()):
            return returncode

        # endpoint or model does not support tool calling, fall back to the single command
        print('Tool calling is not supported, falling back to the single command mode.')
        return self._single_command(args, client, provider, agent, request)

    def _single_command(self, args, client, provider, agent, request):
        """Ask the model for a single `ros2` command line and execute it as a tool."""
        system_role = resolve_role_system(args, ROLE_SYSTEM_EXEC_COMMAND)
        completion_params = ChatCompletionParameters(
            messages=chat_messages(system_role, request), stream=False)
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
        try:
            argv = shlex.split(extract_ros2_command(response))
        except ValueError as e:
            print(f'Failed to parse the command: {e}')
            return 1
        if not argv or argv[0] != 'ros2':
            print(f"Not a ros2 command: '{' '.join(argv)}'")
            return 1
        try:
            command = provider.resolve(GENERIC_TOOL_NAME, {'arguments': argv[1:]})
        except ToolError as e:
            print(f'Failed to resolve the command: {e}')
            return 1
        return agent.execute_command(command)


def _tools_unsupported(error) -> bool:
    """Check if the API error reports that tools (function calling) are not supported."""
    return error is not None and 'tool' in str(error).lower()
