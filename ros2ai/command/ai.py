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

import ros2ai.api.config as config
import ros2ai.api.constants as constants

from ros2cli.command import add_subparsers_on_demand
from ros2cli.command import CommandExtension


class AiCommand(CommandExtension):
    """
    ROS 2 OpenAI command line interface. ('OPENAI_API_KEY' is required)
    """

    def add_arguments(self, parser, cli_name):
        self._subparser = parser

        # add global arguments
        parser.add_argument(
            '-m', '--model', metavar='<model>', type=str, default=constants.ROS_OPENAI_DEFAULT_MODEL,
            help=f'Set OpenAI API model (default %(default)s) or '
              f'use {constants.ROS_OPENAI_MODEL_NAME_ENV_VAR} environment variable. (argument prevails)')
        parser.add_argument(
            '-u', '--url', metavar='<url>', type=str, default=constants.ROS_OPENAI_DEFAULT_ENDPOINT,
            help='Set OpenAI API endpoint URL (default %(default)s) or '
              f'use {constants.ROS_OPENAI_ENDPOINT_ENV_VAR} environment variable. (argument prevails)')
        parser.add_argument(
            '-t', '--token', metavar='<token>', type=int, default=None,
            help='Set OpenAI API maximum token (default %(default)s)')

        # add arguments and sub-commands of verbs
        add_subparsers_on_demand(
            parser, cli_name, '_verb', 'ros2ai.verb', required=False)

    def main(self, *, parser, args):
        if not hasattr(args, '_verb'):
            # in case no verb was passed
            self._subparser.print_help()
            return 0

        extension = getattr(args, '_verb')

        # call the verb's main method
        return extension.main(args=args)
