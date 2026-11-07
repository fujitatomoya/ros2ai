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
from ros2ai.api.constants import ROLE_SYSTEM_QUERY_DEFAULT
from ros2ai.verb import VerbExtension


class QueryVerb(VerbExtension):
    """Query a single completion to OpenAI API."""

    def add_arguments(self, parser, cli_name):
        # add global arguments
        add_global_arguments(parser)

        # sub-command arguments
        parser.add_argument(
            'questions', nargs='+', default=[],
            help='Multiple sentences separated by spaces, enclosed in double quotes.')
        parser.add_argument(
            '-n',
            '--nostream',
            action='store_true',
            help='Prints using no streaming completion responses')
        parser.add_argument(
            '-v',
            '--verbose',
            action='store_true',
            help='Prints detailed response information (only available with nostream option)')
        add_role_argument(parser)

    def main(self, *, args):
        sentence = ' '.join(args.questions)
        if sentence == '':
            print('Dont be shy, put some questions! (I am not AI)')

        system_role = resolve_role_system(args, ROLE_SYSTEM_QUERY_DEFAULT)
        completion_params = ChatCompletionParameters(
            messages=chat_messages(system_role, sentence), stream=not args.nostream)
        client = ChatCompletionClient(OpenAiConfig(args))
        if not client.call(completion_params):
            return 1
        if args.verbose and args.nostream:
            client.print_all()
            print(f'System role:\n{system_role}')
        else:
            client.print_result()
        return 0
