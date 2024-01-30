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
from ros2ai.api.openai import ChatCompletionClient, ChatCompletionParameters
from ros2ai.api.utils import get_ros_distro
from ros2ai.verb import VerbExtension

import ros2ai.api.constants as constants

class QueryVerb(VerbExtension):
    """Query a single completion to OpenAI API."""

    def add_arguments(self, parser, cli_name):
        # add global arguments
        add_global_arguments(parser)

        # sub-command arguments
        parser.add_argument(
            "questions", nargs="+", default=[],
            help="Multiple sentences separated by spaces, enclosed in double quotes.")
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

    def main(self, *, args):
        sentence = ''
        if (args.questions):
            sentence = " ".join(args.questions)
        if (sentence == ''):
            print('Dont be shy, put some questions! (I am not AI)') 

        distro = get_ros_distro()
        if distro is None:
            distro = 'rolling' # fallback to rolling in default
        system_role = constants.ROLE_SYSTEM_QUERY_DEFAULT.format(distro)
        user_messages = [
            {"role": "system", "content": f"{system_role}"},
            {"role": "user", "content": f"{sentence}"}
        ]

        completion_params = ChatCompletionParameters(
            messages = user_messages, stream = (not args.nostream))
        client = ChatCompletionClient(args)
        client.call(completion_params)
        if (args.verbose is True and args.nostream is True):
            client.print_all()
        else:
            client.print_result()
