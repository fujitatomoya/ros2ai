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

from typing import Optional, List, Tuple

import json
import subprocess

import ros2ai.api.constants as constants

def add_global_arguments(parser):
        # add global arguments
    parser.add_argument(
        '-m', '--model', metavar='<model>', type=str, default=None,
        help=f'Set OpenAI API model (default %(default)s) or '
            f'use {constants.ROS_OPENAI_MODEL_NAME_ENV_VAR} environment variable. (argument prevails)')
    parser.add_argument(
        '-u', '--url', metavar='<url>', type=str, default=None,
        help='Set OpenAI API endpoint URL (default %(default)s) or '
            f'use {constants.ROS_OPENAI_ENDPOINT_ENV_VAR} environment variable. (argument prevails)')
    parser.add_argument(
        '-t', '--token', metavar='<token>', type=int, default=None,
        help='Set OpenAI API maximum token (default %(default)s)')


def curl_get_request(url, headers=None) -> Tuple[bool, Optional[List[str]]]:
    response_list = []
    # only supports basic curl subprocess command
    curl_cmd = ["curl", url]

    if headers:
        for key, value in headers.items():
            curl_cmd.extend(["-H", f"{key}: {value}"])

    try:
        # execute the curl command in subprocess
        result = subprocess.run(curl_cmd, capture_output=True, text=True, check=True)
        if result.returncode == 0:
            # error code is integrated in the response, so we need to check the response
            parsed_data = json.loads(result.stdout)
            if 'error' in parsed_data:
                print(parsed_data['error'])
                return False, None
            else:
                for data in parsed_data['data']:
                    response_list.append(data['id'])
                return True, response_list
        else:
            # this means, subprocess returns failure
            print(result.stdout)
            return False, None
    except Exception as e:
        print(f"Error executing curl command: {e}")
        return False, None
