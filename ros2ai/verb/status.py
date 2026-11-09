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

from ros2ai.api.arguments import add_global_arguments
from ros2ai.api.client import ChatCompletionClient, ChatCompletionParameters
from ros2ai.api.config import OpenAiConfig
from ros2ai.verb import VerbExtension

# user message sent to check that the configured model answers
AVAILABILITY_CHECK_MESSAGES = [{'role': 'user', 'content': 'Are you available?'}]


class StatusVerb(VerbExtension):
    """Check LLM API status and configuration."""

    def add_arguments(self, parser, cli_name):
        # add global arguments
        add_global_arguments(parser)

        # sub-command arguments
        parser.add_argument(
            '-v',
            '--verbose',
            action='store_true',
            help='Prints detailed configuration information')
        parser.add_argument(
            '-l',
            '--list',
            action='store_true',
            help='Prints all available models')

    def main(self, *, args):
        config = OpenAiConfig(args)
        if args.verbose:
            config.display_all()

        # try to call LLM API with user configured setting
        client = ChatCompletionClient(config)
        if client.call(ChatCompletionParameters(
                messages=AVAILABILITY_CHECK_MESSAGES, stream=False)):
            client.print_result()
        if client.get_result() is None:
            print('[FAILURE] Invalid API key.')
            return 1
        if args.verbose:
            print('[SUCCESS] Valid API key.')

        # try to list the all models via user configured api key
        can_get_models, model_list = client.list_models()
        if not can_get_models:
            print('[FAILURE] Could not retrieved list of models.')
            return 1
        if args.verbose:
            print('[SUCCESS] Retrieved list of models.')

        if args.list:
            print('Available Models:')
            for model in model_list:
                print(model)

        return 0
