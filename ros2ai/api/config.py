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

import ros2ai.api.constants as constants

from openai import OpenAI

def use_openai() -> bool:
    if constants.ROS_OPENAI_API_KEY_ENV_VAR in os.environ:
        return True
    else:
        return False

def get_api_key() -> str:
    """
    Get OpenAI API Key from OPENAI_API_KEY environment variable.

    OpenAI API Key can be set by ros2ai user with OPENAI_API_KEY environment variable.

    :return: string of OpenAI API Key or None
    """
    key_name = os.environ.get(constants.ROS_OPENAI_API_KEY_ENV_VAR, "None")
    return key_name

def get_ai_model() -> str:
    """
    Get AI Model from AI_MODEL_NAME environment variable as optional.

    :return: string of AI Model.
    """
    model_name = os.environ.get(constants.ROS_AI_MODEL_NAME_ENV_VAR)
    if not model_name:
        if use_openai():
            return constants.ROS_OPENAI_DEFAULT_MODEL
        else:
            return constants.ROS_OLLAMA_DEFAULT_MODEL
    else:
        return model_name

def get_endpoint_url() -> str:
    """
    Get AI API service endpoint URL from AI_ENDPOINT environment variable as optional.

    :return: string of AI API service endpoint URL.
    """
    url = os.environ.get(constants.ROS_AI_ENDPOINT_ENV_VAR)
    if not url:
        if use_openai():
            return constants.ROS_OPENAI_DEFAULT_ENDPOINT
        else:
            return constants.ROS_OLLAMA_DEFAULT_ENDPOINT
    else:
        return url

def get_temperature() -> float:
    """
    Get temperature parameter to be used with AI.

    :return: temperature, could be None.
    """
    temperature = os.environ.get(constants.ROS_AI_TEMPERATURE_ENV_VAR)
    if not temperature:
        return float(constants.ROS_AI_DEFAULT_TEMPERATURE)
    else:
        return float(temperature)

def get_role_system(default_role_system: str = None) -> str:
    return os.environ.get(constants.ROLE_SYSTEM_ENV_VAR, default_role_system)

class AiConfig:
    """
    Collect all AI API related configuration from user setting as key-value pair.
    """
    def __init__(self, args):
        self.config_pair = {}

        # api key is mandatory for OpenAI, if this is set, it will use OpenAI API
        self.config_pair['api_key'] = get_api_key()

        # ai model is optional, command line argument prevails
        self.config_pair['api_model'] = get_ai_model()
        if args.model and args.model != self.config_pair['api_model']:
            self.config_pair['api_model'] = args.model

        # api endpoint is optional, command line argument prevails
        self.config_pair['api_endpoint'] = get_endpoint_url()
        if args.url and args.url != self.config_pair['api_endpoint']:
            self.config_pair['api_endpoint'] = args.url

        # api token is optional, only available via command line argument
        self.config_pair['api_token'] = args.token

        # temperature is optional, only available via environmental variable
        self.config_pair['api_temperature'] = get_temperature()

    def set_value(self, key, value):
        # Set a key-value pair
        self.config_pair[key] = value

    def get_value(self, key):
        # Get the value for a given key
        return self.config_pair.get(key, None)

    def remove_key(self, key):
        # Remove a key and its associated value
        if key in self.config_pair:
            del self.config_pair[key]

    def display_all(self):
        # Display all key-value pairs
        for key, value in self.config_pair.items():
            # we should never print the api key for the security
            if key != 'api_key':
                print(f"----- {key}: {value}")

    def is_api_key_valid(self):
        # Validate api key, model and endpoint to post the API
        client = OpenAI(
            api_key=self.get_value('api_key'),
            base_url=self.get_value('api_endpoint')
        )
        try:
            completion = client.chat.completions.create(
                model = self.get_value('api_model'),
                messages = [
                    {
                        "role": "user",
                        "content": "Are you in service?",
                    },
                ],
                temperature = self.get_value('api_temperature'),
                max_tokens = self.get_value('api_token')
            )
        except Exception as e:
            print('Failed to call OpenAI API: ' + str(e))
            return False
        else:
            if (completion.choices[0].finish_reason != 'stop'):
                print('Failed chat completion with: ' + completion.choices[0].finish_reason)
                return False
            else:
                print(completion.choices[0].message.content)
                return True
