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

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from openai import OpenAI

from ros2ai.api.config import is_anthropic_endpoint, OpenAiConfig
import ros2ai.api.constants as constants


@dataclass
class ChatCompletionParameters:
    """Chat completion parameters."""

    messages: list = field(default_factory=list)
    stream: bool = True
    # tool definitions in the OpenAI function calling format, no tools if None
    tools: Optional[list] = None


def chat_messages(system_role: str, user_content: str) -> list:
    """
    Create the chat completion messages for a single user prompt.

    :param system_role: content of the system message.
    :param user_content: content of the user message.
    :return: list of messages to be sent to the chat completion.
    """
    return [
        {'role': 'system', 'content': system_role},
        {'role': 'user', 'content': user_content},
    ]


class ChatCompletionClient:
    """Single chat completion client w/o session to call LLM API via OpenAI Python API."""

    def __init__(self, config: OpenAiConfig):
        self._config = config
        default_headers = None
        if is_anthropic_endpoint(config.get_value('api_endpoint')):
            # Anthropic native endpoints (e.g /v1/models) validate `x-api-key` instead of bearer
            default_headers = {
                'x-api-key': config.get_value('api_key'),
                'anthropic-version': constants.ROS_ANTHROPIC_API_VERSION,
            }
        self._client = OpenAI(
            api_key=config.get_value('api_key'),
            base_url=config.get_value('api_endpoint'),
            default_headers=default_headers)
        self._completion = None
        self._stream = True
        self._error = None

    def call(self, parameters: ChatCompletionParameters) -> bool:
        """
        Call the chat completion with the configured model.

        :param parameters: messages, streaming mode and tools of the completion.
        :return: True if the API call succeeded, False otherwise.
        """
        self._stream = parameters.stream
        self._completion = None
        self._error = None
        kwargs = {}
        if parameters.tools:
            # `tool_choice` is intentionally left to the default (auto), forcing the tool use
            # is not supported by every OpenAI compatible endpoint.
            kwargs['tools'] = parameters.tools
        try:
            self._completion = self._client.chat.completions.create(
                model=self._config.get_value('api_model'),
                messages=parameters.messages,
                stream=parameters.stream,
                temperature=self._config.get_value('api_temperature'),
                max_tokens=self._config.get_value('api_token'),
                **kwargs)
        except Exception as e:
            self._error = e
            print('Failed to call LLM API: ' + str(e))
            return False
        return True

    def get_error(self) -> Optional[Exception]:
        """Get the exception of the last failed call, None if the last call succeeded."""
        return self._error

    def _choice(self):
        return self._completion.choices[0]

    def print_result(self):
        """Print the completion result, nothing is printed if the call failed."""
        if self._completion is None:
            return
        if self._stream:
            # TODO@fujitatomoya: check `finish_reason` in stream if available?
            for chunk in self._completion:
                if chunk.choices[0].delta.content is not None:
                    print(chunk.choices[0].delta.content, end='')
            print('\n')
        elif self._choice().finish_reason != 'stop':
            print('Failed chat completion with: ' + self._choice().finish_reason)
        else:
            print(self._choice().message.content)

    def get_result(self) -> Optional[str]:
        """
        Get the completion result.

        :return: content of the completion, None if streaming, failed or not finished.
        """
        if self._completion is None or self._stream:
            return None
        if self._choice().finish_reason != 'stop':
            return None
        return self._choice().message.content

    def get_message(self):
        """
        Get the completion message including the tool calls.

        :return: message of the completion, None if streaming, failed or not finished.
        """
        if self._completion is None or self._stream:
            return None
        choice = self._choice()
        if choice.finish_reason != 'stop' and not getattr(choice.message, 'tool_calls', None):
            return None
        return choice.message

    def print_all(self):
        """Print the raw completion, only available without streaming."""
        if not self._stream:
            print(self._completion)

    def list_models(self) -> Tuple[bool, Optional[List[str]]]:
        """
        List the models available on the configured endpoint.

        :return: tuple of success flag and list of model identifiers.
        """
        try:
            return True, [model.id for model in self._client.models.list()]
        except Exception as e:
            print(f'Failed to list models: {e}')
            return False, None
