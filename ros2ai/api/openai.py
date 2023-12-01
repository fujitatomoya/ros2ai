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

from ros2ai.api.config import OpenAiConfig

from openai import OpenAI


class ChatCompletionClient(OpenAiConfig):
    """
    Create single chat completion client w/o session and call OpenAI.
    """
    def __init__(self, args):
        # get OpenAI configuration
        super().__init__(args)

        self.client_ = OpenAI(
            api_key=self.get_value('api_key')
        )
        self.completion_ = None

    def call(self, sentence):
        try:
            self.completion_ = self.client_.chat.completions.create(
                model=self.get_value('api_model'),
                messages=[
                    {
                        "role": "user",
                        "content": f"{sentence}",
                    },
                ],
            )
        except Exception as e:
            print('Failed to call OpenAI API: ' + str(e))
        else:
            pass

    def response_content(self):
        return self.completion_.choices[0].message.content

    def response_all(self):
        return self.completion_
