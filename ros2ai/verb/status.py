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

from ros2ai.verb import VerbExtension


class StatusVerb(VerbExtension):
    """Check OpenAI API status and configuration."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            '--verbose',
            '-v',
            action='store_true',
            help='Prints detailed configuration information')

    def main(self, *, args):
        print(config.get_api_key())
        print(config.get_ai_model())
        print(config.get_endpoint_url())
