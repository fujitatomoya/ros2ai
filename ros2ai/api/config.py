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
from urllib.parse import urlparse

import ros2ai.api.constants as constants
from ros2ai.api.utils import get_ros_distro

import validators


def get_env(name: str, legacy_name: str, default=None):
    """
    Get the configuration from the environment variable.

    :param name: name of the environment variable.
    :param legacy_name: name of the legacy environment variable, used if `name` is not set.
    :param default: value returned if none of the environment variables is set.
    :return: value of the environment variable, or the default.
    """
    return os.environ.get(name) or os.environ.get(legacy_name) or default


def get_api_key() -> str:
    """
    Get API Key from ROS2AI_API_KEY (or legacy OPENAI_API_KEY) environment variable.

    :return: string of API Key, 'None' if not set.
    """
    return get_env(
        constants.ROS2AI_API_KEY_ENV_VAR, constants.ROS_OPENAI_API_KEY_ENV_VAR, 'None')


def get_ai_model() -> str:
    """
    Get model from ROS2AI_MODEL_NAME (or legacy OPENAI_MODEL_NAME) environment variable.

    :return: string of model name, default model if not set.
    """
    return get_env(
        constants.ROS2AI_MODEL_NAME_ENV_VAR, constants.ROS_OPENAI_MODEL_NAME_ENV_VAR,
        constants.ROS2AI_DEFAULT_MODEL)


def get_endpoint_url() -> str:
    """
    Get API endpoint URL from ROS2AI_ENDPOINT (or legacy OPENAI_ENDPOINT) environment variable.

    :return: string of API endpoint URL, default endpoint if not set or invalid.
    """
    url = get_env(constants.ROS2AI_ENDPOINT_ENV_VAR, constants.ROS_OPENAI_ENDPOINT_ENV_VAR)
    if not url:
        return constants.ROS2AI_DEFAULT_ENDPOINT
    try:
        # this returns ValidationError with `http://localhost:11434/v1`
        if validators.url(url):
            return url
    except Exception as e:
        print(f'Error validating URL: {e}')
    return constants.ROS2AI_DEFAULT_ENDPOINT


def get_temperature() -> float:
    """
    Get temperature from ROS2AI_TEMPERATURE (or legacy OPENAI_TEMPERATURE) environment variable.

    :return: temperature, default temperature if not set.
    """
    return float(get_env(
        constants.ROS2AI_TEMPERATURE_ENV_VAR, constants.ROS_OPENAI_TEMPERATURE_ENV_VAR,
        constants.ROS2AI_DEFAULT_TEMPERATURE))


def get_role_system(default_role_system: str = None) -> str:
    """
    Get the system role from ROS2AI_ROLE_SYSTEM (or legacy OPENAI_ROLE_SYSTEM) variable.

    :param default_role_system: role returned if the environment variable is not set.
    :return: string of the system role.
    """
    return get_env(
        constants.ROS2AI_ROLE_SYSTEM_ENV_VAR, constants.ROLE_SYSTEM_ENV_VAR, default_role_system)


def get_context_limit() -> int:
    """
    Get the context limit (estimated tokens) from ROS2AI_CONTEXT_LIMIT environment variable.

    :return: the limit, default limit if not set or invalid.
    """
    value = os.environ.get(constants.ROS2AI_CONTEXT_LIMIT_ENV_VAR)
    if not value:
        return constants.ROS2AI_CONTEXT_LIMIT
    try:
        limit = int(value)
        if limit > 0:
            return limit
    except ValueError:
        pass
    print(f'Invalid {constants.ROS2AI_CONTEXT_LIMIT_ENV_VAR}: {value}, '
          f'use default {constants.ROS2AI_CONTEXT_LIMIT}')
    return constants.ROS2AI_CONTEXT_LIMIT


def is_anthropic_endpoint(url: str) -> bool:
    """
    Check if the API endpoint URL is the Anthropic API.

    :param url: API endpoint URL.
    :return: True if the endpoint host is the Anthropic API.
    """
    return urlparse(url).hostname == constants.ROS_ANTHROPIC_API_HOST


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
    """Collect all LLM API related configuration from user setting as key-value pair."""

    def __init__(self, args):
        # api key is only required when accessing the managed service such as OpenAI or Anthropic
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
