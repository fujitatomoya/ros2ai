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
        self.stream_ = True

    def call(self, parameteters):
        self.stream_ = parameteters.stream
        try:
            self.completion_ = self.client_.chat.completions.create(
                model = self.get_value('api_model'),
                messages = parameteters.messages,
                stream = parameteters.stream,
                temperature = self.get_value('api_temperature'),
                max_tokens = self.get_value('api_token')
            )
        except Exception as e:
            print('Failed to call OpenAI API: ' + str(e))
        else:
            pass

    def print_result(self):
        if self.completion_ is None:
            pass
        if self.stream_ is True:
            # TODO@fujitatomoya: check `finish_reason` in stream if available?
            for chunk in self.completion_:
                if chunk.choices[0].delta.content is not None:
                    print(chunk.choices[0].delta.content, end="")
            print("\n")
        else:
            if (self.completion_.choices[0].finish_reason != 'stop'):
                print('Failed chat completion with: ' + self.completion_.choices[0].finish_reason)
            else:
                print(self.completion_.choices[0].message.content)

    def get_result(self) -> str:
        if self.completion_ is None:
            return None
        if self.stream_ is True:
            return None
        else:
            if (self.completion_.choices[0].finish_reason != 'stop'):
                return None
            else:
                return self.completion_.choices[0].message.content

    def print_all(self):
        if self.stream_ is False:
            print(self.completion_)

class ChatCompletionParameters:
    """
    Create chat completion client parameters.
    """
    def __init__(self, messages = [], stream = True):
        self.messages_ = messages
        self.stream_ = stream

    @property
    def stream(self):
        """Getter method for the streaming mode property."""
        return self.stream_

    @stream.setter
    def stream(self, flag):
        """Setter method for the streaming mode property."""
        if isinstance(flag, bool):
            self.stream_ = flag
        else:
            raise ValueError("stream must be a bool.")

    @property
    def messages(self):
        """Getter method for the messages list property."""
        return self.messages_

    @messages.setter
    def messages(self, messages):
        """Setter method for the messages list property."""
        if isinstance(messages, list):
            self.messages_ = messages
        else:
            raise ValueError("messages must be a list.")
