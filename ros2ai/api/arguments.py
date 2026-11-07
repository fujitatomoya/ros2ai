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

import ros2ai.api.constants as constants


def add_global_arguments(parser):
    """Add the arguments shared by the `ai` command and every verb."""
    default_models = constants.ROS_OPENAI_DEFAULT_MODEL + ', ' + constants.ROS_OLLAMA_DEFAULT_MODEL
    parser.add_argument(
        '-m', '--model', metavar='<model>', type=str, default=None,
        help=f'Set OpenAI API model (e.g {default_models}) or '
             f'use {constants.ROS_OPENAI_MODEL_NAME_ENV_VAR} environment variable. '
             '(argument prevails)')
    parser.add_argument(
        '-u', '--url', metavar='<url>', type=str, default=None,
        help=f'Set OpenAI API endpoint URL (e.g {constants.ROS_OPENAI_DEFAULT_ENDPOINT}) or '
             f'use {constants.ROS_OPENAI_ENDPOINT_ENV_VAR} environment variable. '
             '(argument prevails)')
    parser.add_argument(
        '-t', '--token', metavar='<token>', type=int, default=None,
        help='Set OpenAI API maximum token (default %(default)s)')


def add_role_argument(parser):
    """Add the system role argument shared by the verbs that prompt the model."""
    parser.add_argument(
        '-r', '--role', metavar='<role>', type=str, default=None,
        help="Define the prompt's system role.")
