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

from ros2ai.api import add_global_arguments

from ros2cli.command import add_subparsers_on_demand
from ros2cli.command import CommandExtension


class AiCommand(CommandExtension):
    """
    ROS 2 OpenAI command line interface. ('OPENAI_API_KEY' is required)
    """

    def add_arguments(self, parser, cli_name):
        self._subparser = parser

        # add global arguments
        add_global_arguments(parser)

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
