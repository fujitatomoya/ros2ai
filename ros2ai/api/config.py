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
from ros2ai.api.utils import get_ros_distro

import validators


def get_api_key() -> str:
    """
    Get OpenAI API Key from OPENAI_API_KEY environment variable.

    :return: string of OpenAI API Key, 'None' if not set.
    """
    return os.environ.get(constants.ROS_OPENAI_API_KEY_ENV_VAR, 'None')


def get_ai_model() -> str:
    """
    Get OpenAI Model from OPENAI_MODEL_NAME environment variable.

    :return: string of OpenAI Model, default model if not set.
    """
    return os.environ.get(constants.ROS_OPENAI_MODEL_NAME_ENV_VAR) or \
        constants.ROS_OPENAI_DEFAULT_MODEL


def get_endpoint_url() -> str:
    """
    Get OpenAI API service endpoint URL from OPENAI_ENDPOINT environment variable.

    :return: string of OpenAI API service endpoint URL, default endpoint if not set or invalid.
    """
    url = os.environ.get(constants.ROS_OPENAI_ENDPOINT_ENV_VAR)
    if not url:
        return constants.ROS_OPENAI_DEFAULT_ENDPOINT
    try:
        # this returns ValidationError with `http://localhost:11434/v1`
        if validators.url(url):
            return url
    except Exception as e:
        print(f'Error validating URL: {e}')
    return constants.ROS_OPENAI_DEFAULT_ENDPOINT


def get_temperature() -> float:
    """
    Get temperature parameter to be used with OpenAI API.

    :return: temperature, default temperature if not set.
    """
    return float(
        os.environ.get(constants.ROS_OPENAI_TEMPERATURE_ENV_VAR) or
        constants.ROS_OPENAI_DEFAULT_TEMPERATURE)


def get_role_system(default_role_system: str = None) -> str:
    """
    Get the system role from OPENAI_ROLE_SYSTEM environment variable.

    :param default_role_system: role returned if the environment variable is not set.
    :return: string of the system role.
    """
    return os.environ.get(constants.ROLE_SYSTEM_ENV_VAR, default_role_system)


def resolve_role_system(args, default_role_system: str) -> str:
    """
    Resolve the system role for the prompt, formatted with the ROS distribution.

    The command line argument prevails over the environment variable,
    which prevails over the default role.

    :param args: parsed arguments, `role` attribute is used if set.
    :param default_role_system: role used if nothing else is configured.
    :return: string of the system role with the distribution name applied.
    """
    role = get_role_system(default_role_system=default_role_system)
    if args.role and args.role != role:
        role = args.role
    return role.format(get_ros_distro())


class OpenAiConfig:
    """Collect all OpenAI API related configuration from user setting as key-value pair."""

    def __init__(self, args):
        # api key is only required when accessing api.openai.com
        # ai model and api endpoint are optional, command line argument prevails
        # api token is optional, only available via command line argument
        # temperature is optional, only available via environmental variable
        self._config = {
            'api_key': get_api_key(),
            'api_model': args.model or get_ai_model(),
            'api_endpoint': args.url or get_endpoint_url(),
            'api_token': args.token,
            'api_temperature': get_temperature(),
        }

    def get_value(self, key):
        """Get the value for a given key, None if the key does not exist."""
        return self._config.get(key, None)

    def display_all(self):
        """Print all key-value pairs except the api key."""
        for key, value in self._config.items():
            # we should never print the api key for the security
            if key != 'api_key':
                print(f'----- {key}: {value}')
