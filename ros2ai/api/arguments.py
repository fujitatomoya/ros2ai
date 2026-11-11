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

# examples printed at the end of `ros2 ai -h`
EXAMPLES = """\
examples:
  ros2 ai "how can I check the available topics?"
  ros2 ai "give me all topics"
  ros2 ai "publish hello message on /chatter topic" -y
  ros2 ai --status --list-models
"""


def add_arguments(parser):
    """Add the arguments of the `ai` command, the request and the options."""
    default_models = ', '.join([
        constants.ROS_OPENAI_DEFAULT_MODEL,
        constants.ROS_ANTHROPIC_DEFAULT_MODEL,
        constants.ROS_OLLAMA_DEFAULT_MODEL])
    default_endpoints = ', '.join([
        constants.ROS_OPENAI_DEFAULT_ENDPOINT,
        constants.ROS_ANTHROPIC_DEFAULT_ENDPOINT,
        constants.ROS_OLLAMA_DEFAULT_ENDPOINT])
    parser.add_argument(
        'request', nargs='*', default=[], metavar='<request>',
        help='Question or operation request to the ROS 2 assistant, '
             'multiple words are joined with spaces, or enclosed in double quotes.')
    parser.add_argument(
        '-m', '--model', metavar='<model>', type=str, default=None,
        help=f'Set LLM model (e.g {default_models}) or '
             f'use {constants.ROS2AI_MODEL_NAME_ENV_VAR} environment variable. '
             '(argument prevails)')
    parser.add_argument(
        '-u', '--url', metavar='<url>', type=str, default=None,
        help=f'Set LLM API endpoint URL (e.g {default_endpoints}) or '
             f'use {constants.ROS2AI_ENDPOINT_ENV_VAR} environment variable. '
             '(argument prevails)')
    parser.add_argument(
        '-t', '--token', metavar='<token>', type=int, default=None,
        help='Set LLM API maximum token (default %(default)s)')
    parser.add_argument(
        '-r', '--role', metavar='<role>', type=str, default=None,
        help="Define the prompt's system role, `{}` is replaced with the ROS distribution name.")
    parser.add_argument(
        '-v', '--verbose',
        action='store_true',
        help='Prints detailed information: configuration, system role, raw completions, '
             'tool calls and results.')
    parser.add_argument(
        '-y', '--yes',
        action='store_true',
        help='Executes the commands that change the system without confirmation.')
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Prints the commands that change the system instead of executing them, '
             'read-only commands are still executed.')
    parser.add_argument(
        '-s', '--status',
        action='store_true',
        help='Checks LLM API status and configuration instead of sending the request.')
    parser.add_argument(
        '-l', '--list-models',
        action='store_true',
        help='Prints all available models on the endpoint (implies --status).')
