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


def get_api_key() -> str:
    """
    Get OpenAI API Key from OPENAI_API_KEY environment variable.

    OpenAI API Key must be set by ros2ai user with OPENAI_API_KEY environment variable.

    :return: string of OpenAI API Key.
    :raises: if OPENAI_API_KEY is not set.
    """
    key_name = os.environ.get(constants.ROS_OPENAI_API_KEY_ENV_VAR, "")
    return key_name

def get_ai_model() -> str:
    """
    Get OpenAI Model from OPENAI_MODEL_NAME environment variable.

    OpenAI Model is optional, in default to gpt-4o

    :return: string of OpenAI Model.
    """
    model_name = os.environ.get(constants.ROS_OPENAI_MODEL_NAME_ENV_VAR)
    if not model_name:
        # TODO(@fujitatomoya):better to print info here that using default model.
        return constants.ROS_OPENAI_DEFAULT_MODEL
    else:
        return model_name

def get_endpoint_url() -> str:
    """
    Get OpenAI API service endpoint URL from OPENAI_ENDPOINT environment variable.

    OpenAI API service endpoint URL is optional, in default fallback to openai.

    :return: string of OpenAI API service endpoint URL, could be None.
    """
    url = os.environ.get(constants.ROS_OPENAI_ENDPOINT_ENV_VAR)
    # TODO(@fujitatomoya):check if that is valid url before return.
    if not url:
        return constants.ROS_OPENAI_DEFAULT_ENDPOINT
    else:
        return url

def get_temperature() -> float:
    """
    Get temperature parameter to be used with OpenAI API.

    :return: temperature, could be None.
    """
    temperature = os.environ.get(constants.ROS_OPENAI_TEMPERATURE_ENV_VAR)
    if not temperature:
        return float(constants.ROS_OPENAI_DEFAULT_TEMPERATURE)
    else:
        return float(temperature)

class OpenAiConfig:
    """
    Collect all OpenAI API related configuration from user setting as key-value pair.
    """
    def __init__(self, args):
        self.config_pair = {}

        # api key is mandatory, this could throw the exception if not set
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
            api_key=self.get_value('api_key')
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
