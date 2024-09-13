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

from ollama import Client

import ros2ai.api.config as config

from ros2ai.api import ollama_get_models, openai_get_models, add_global_arguments
from ros2ai.verb import VerbExtension


class StatusVerb(VerbExtension):
    """Check OpenAI/Ollama API status and configuration."""

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
        ai_config = config.AiConfig(args)
        if (args.verbose is True):
            ai_config.display_all()

        can_get_models = None
        model_list = []
        if config.use_openai():
            # try to call OpenAI API with user configured setting
            is_valid = ai_config.is_api_key_valid()

            if is_valid:
                if args.verbose:
                    print("[SUCCESS] Valid OpenAI API key.")
            else:
                print("[FAILURE] Invalid OpenAI API key.")
                return 1

            # try to list the all models via user configured api key
            headers = {"Authorization": "Bearer " + ai_config.get_value('api_key')}
            can_get_models, model_list = openai_get_models(
                ai_config.get_value('api_endpoint') + "/models",
                headers
            )
        else:
            # try to call Ollama API
            ollama_client = Client(host=ai_config.get_value('api_endpoint'))
            response = ollama_client.generate(
                model = ai_config.get_value('api_model'),
                prompt = 'Are you in available?'
            )
            if response['done'] is not True:
                print("[FAILURE] Failed to call ollama API.")
                if args.verbose is True:
                    print(response)
                return 1
            else:
                print(response['response'])
            # try to get available model list via curl
            can_get_models, model_list = ollama_get_models(
                ai_config.get_value('api_endpoint') + "/api/tags"
            )

        if can_get_models:
            print("[SUCCESS] Retrieved list of models.")
        else:
            print("[FAILURE] Could not retrieved list of models.")
            return 1

        if (args.list is True):
            print("Available Models:")
            for model in model_list:
                print(model)

        return 0
