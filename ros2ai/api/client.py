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
from typing import Callable, List, Optional, Tuple

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


@dataclass
class ToolCallFunction:
    """Function of a tool call, in the shape of the OpenAI completion message."""

    name: str = ''
    arguments: str = ''


@dataclass
class ToolCall:
    """Tool call of a completion message, in the shape of the OpenAI completion message."""

    # the attribute names follow the OpenAI object so that the agent can use either
    id: Optional[str] = None  # noqa: A003
    function: ToolCallFunction = field(default_factory=ToolCallFunction)
    type: str = 'function'  # noqa: A003


@dataclass
class CompletionMessage:
    """Completion message assembled from the streamed chunks."""

    content: Optional[str] = None
    tool_calls: List[ToolCall] = field(default_factory=list)


def assemble_stream(chunks, on_content: Optional[Callable[[str], None]] = None):
    """
    Assemble the streamed chunks into a completion message.

    The content deltas are passed to `on_content` as they arrive, and the tool call deltas
    are accumulated by their index, the arguments are concatenated.

    :param chunks: iterable of the streamed chunks of the chat completion.
    :param on_content: callback invoked with each content delta.
    :return: tuple of the assembled message and the finish reason (None if not reported).
    """
    content = []
    tool_calls = {}
    finish_reason = None
    for chunk in chunks:
        choices = getattr(chunk, 'choices', None)
        if not choices:
            # e.g the usage chunk at the end of the stream
            continue
        choice = choices[0]
        delta = getattr(choice, 'delta', None)
        if delta is not None:
            text = getattr(delta, 'content', None)
            if text:
                content.append(text)
                if on_content is not None:
                    on_content(text)
            for delta_call in getattr(delta, 'tool_calls', None) or []:
                next_index = max(tool_calls, default=-1) + 1
                index = getattr(delta_call, 'index', None)
                if index is None:
                    index = next_index
                call_id = getattr(delta_call, 'id', None)
                tool_call = tool_calls.get(index)
                if tool_call is not None and call_id and tool_call.id and call_id != tool_call.id:
                    # some endpoints reuse the index for every tool call
                    index, tool_call = next_index, None
                if tool_call is None:
                    tool_call = ToolCall()
                    tool_calls[index] = tool_call
                if call_id:
                    tool_call.id = call_id
                function = getattr(delta_call, 'function', None)
                if function is not None:
                    name = getattr(function, 'name', None)
                    if name:
                        tool_call.function.name = name
                    arguments = getattr(function, 'arguments', None)
                    if arguments:
                        tool_call.function.arguments += arguments
        if getattr(choice, 'finish_reason', None):
            finish_reason = choice.finish_reason
    calls = [tool_calls[index] for index in sorted(tool_calls)]
    for number, tool_call in enumerate(calls):
        if not tool_call.id:
            # the id is required to return the tool result to the model
            tool_call.id = f'call_{number}'
    message = CompletionMessage(content=''.join(content) or None, tool_calls=calls)
    return message, finish_reason


def _print_delta(text: str):
    print(text, end='', flush=True)


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
        """Print the completion result, only available without streaming."""
        if self._completion is None or self._stream:
            return
        if self._choice().finish_reason != 'stop':
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

        When streaming, the content is printed as it arrives and the message is assembled
        from the chunks. The reason is printed if the completion did not finish.

        :return: message of the completion, None if failed or not finished.
        """
        if self._completion is None:
            return None
        if self._stream:
            message, finish_reason = assemble_stream(self._completion, on_content=_print_delta)
            if message.content:
                # end the line of the streamed content
                print()
        else:
            choice = self._choice()
            message, finish_reason = choice.message, choice.finish_reason
        if finish_reason not in (None, 'stop', 'tool_calls') and not message.tool_calls:
            print('Failed chat completion with: ' + finish_reason)
            return None
        return message

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
