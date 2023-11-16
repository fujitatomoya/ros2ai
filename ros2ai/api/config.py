# Copyright 2023 Open Source Robotics Foundation, Inc.
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


def get_api_key() -> str:
    """
    Get OpenAI API Key from OPENAI_API_KEY environment variable.

    OpenAI API Key must be set by ros2ai user with OPENAI_API_KEY environment variable.

    :return: string of OpenAI API Key.
    :raises: if OPENAI_API_KEY is not set.
    """
    key_name = os.environ.get(constants.ROS_OPENAI_API_KEY_ENV_VAR)
    if not key_name:
        raise EnvironmentError(
            f"'{constants.ROS_OPENAI_API_KEY_ENV_VAR}' environment variable is not set'"
        )
    else:
        return key_name

def get_ai_model() -> str:
    """
    Get OpenAI Model from OPENAI_MODEL_NAME environment variable.

    OpenAI Model is optional, in default to gpt-3.5-turbo

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
        return None
    else:
        return url
