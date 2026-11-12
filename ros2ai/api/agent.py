# Copyright 2026 Tomoya Fujita <tomoya.fujita825@gmail.com>.
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

"""Agent loop that lets the model operate the ROS 2 system via tool calling."""

from dataclasses import dataclass
import json
import sys
from typing import Callable, List, Optional

from ros2ai.api.client import ChatCompletionClient, ChatCompletionParameters
import ros2ai.api.constants as constants
from ros2ai.api.session import Conversation
from ros2ai.api.tools import ToolCommand, ToolError, ToolPolicy, ToolProvider, ToolResult
from ros2ai.api.utils import run_executable, truncate_output


@dataclass
class AgentOptions:
    """User options of the agent loop."""

    # print the commands that change the system instead of executing them
    dry_run: bool = False
    # execute the commands that change the system without confirmation
    auto_approve: bool = False
    # stream the completions, the content is printed as it arrives
    stream: bool = True
    # print the raw completions, the tool calls and the tool results
    verbose: bool = False
    max_steps: int = constants.ROS2AI_MAX_STEPS
    output_limit: int = constants.ROS2AI_TOOL_OUTPUT_LIMIT


def confirm_execution(command: str) -> bool:
    """
    Ask the user to confirm the execution of a command.

    :param command: command line to be executed.
    :return: True if the user confirms, False otherwise or if no terminal is available.
    """
    if not sys.stdin.isatty():
        print(f"Skip '{command}': confirmation requires a terminal, use --yes to approve.")
        return False
    try:
        answer = input(f"Execute '{command}'? [y/N]: ")
    except (EOFError, KeyboardInterrupt):
        print()
        return False
    return answer.strip().lower() in ('y', 'yes')


def context_exceeded(error) -> bool:
    """Check if the API error reports that the context length of the model is exceeded."""
    if error is None:
        return False
    text = str(error).lower()
    return (
        'context_length' in text or 'context length' in text or 'context window' in text
        or 'too long' in text or ('maximum' in text and 'tokens' in text))


class _Exit(Exception):
    """Internal: stop the agent loop with the return code."""

    def __init__(self, returncode: int):
        super().__init__(returncode)
        self.returncode = returncode


class Agent:
    """Tool calling loop between the chat completion client and the tool provider."""

    def __init__(
        self,
        client: ChatCompletionClient,
        provider: ToolProvider,
        options: AgentOptions,
        confirm: Optional[Callable[[str], bool]] = None,
    ):
        self._client = client
        self._provider = provider
        self._options = options
        self._confirm = confirm if confirm is not None else confirm_execution
        # number of chat completions requested by the last run
        self.steps = 0
        # number of command candidates printed instead of being executed (dry-run)
        self.candidates = 0

    def run(self, conversation: Conversation, request: str, *, tools: bool = True) -> int:
        """
        Process the user request until the model answers without tool calls.

        The request starts a turn of the conversation, the previous turns give the context.
        The turn is kept in the conversation when the request is processed, so that the
        following requests can refer to it, and discarded if the API call fails.

        :param conversation: conversation of the session, the system role and the history.
        :param request: content of the user message.
        :param tools: offer the tools of the provider to the model, a plain completion if False.
        :return: 0 on success, 1 on failure, or the return code of a long-running command.
        """
        conversation.begin(request)
        tools = self._provider.definitions() if tools else None
        self.steps = 0
        self.candidates = 0
        for _ in range(self._options.max_steps):
            message = self._complete(conversation, tools)
            if message is None:
                conversation.abort()
                return 1
            tool_calls = getattr(message, 'tool_calls', None) or []
            if not tool_calls:
                if not self._options.stream:
                    # the streamed content has been printed already
                    print(message.content or '')
                conversation.append({'role': 'assistant', 'content': message.content or ''})
                conversation.commit()
                return 0
            conversation.append(_assistant_message(message))
            pending = list(tool_calls)
            try:
                while pending:
                    conversation.append(self._handle(pending[0]))
                    pending.pop(0)
            except _Exit as e:
                # a long-running command ends the request when it exits, the result is
                # recorded so that the following requests know what happened
                conversation.append(_tool_message(
                    pending[0].id,
                    f'the command was executed in the foreground and exited with code '
                    f'{e.returncode}.'))
                for tool_call in pending[1:]:
                    conversation.append(_tool_message(
                        tool_call.id, 'not executed: the previous command ended the request.'))
                conversation.commit()
                return e.returncode
            if self.candidates:
                # dry-run: the commands that change the system are printed, stop here
                conversation.commit()
                return 0
        print(f'Reached the maximum number of steps ({self._options.max_steps}).')
        conversation.commit()
        return 1

    def execute(self, command: ToolCommand) -> ToolResult:
        """
        Execute a resolved command according to its policy and the user options.

        :param command: resolved command.
        :return: the result to be returned to the model.
        :raises _Exit: when a long-running command finishes, with its return code.
        """
        display = command.display()
        if command.policy != ToolPolicy.READ_ONLY:
            if self._options.dry_run:
                print(f"Command Candidate: '{display}'")
                self.candidates += 1
                return ToolResult('dry-run: the command was not executed.', 0)
            if not self._options.auto_approve and not self._confirm(display):
                return ToolResult('the user declined to execute the command.', 1)
        print(f'[ros2ai]: {display}')
        if command.policy == ToolPolicy.LONG_RUNNING:
            raise _Exit(run_executable(command=command.argv))
        result = self._provider.run(command)
        if result.output and result.printable:
            print(result.output)
        return result

    def execute_command(self, command: ToolCommand) -> int:
        """
        Execute a single resolved command outside of the loop.

        :param command: resolved command.
        :return: the return code of the command.
        """
        try:
            return self.execute(command).returncode
        except _Exit as e:
            return e.returncode

    def _complete(self, conversation: Conversation, tools: Optional[List[dict]]):
        """
        Request the chat completion with the messages of the conversation.

        If the endpoint reports that the context length is exceeded, the oldest turn of the
        conversation is dropped and the request is retried.
        """
        self.steps += 1
        while not self._client.call(self._parameters(conversation, tools)):
            if not context_exceeded(self._client.get_error()) or not conversation.drop_oldest():
                return None
            print('[ros2ai]: the context length is exceeded, the oldest turn is dropped.')
        if self._options.verbose:
            # raw completion is only available without streaming
            self._client.print_all()
        # None if the completion did not finish, the reason is printed by the client
        return self._client.get_message()

    def _parameters(self, conversation: Conversation, tools: Optional[List[dict]]):
        return ChatCompletionParameters(
            messages=conversation.messages(), stream=self._options.stream, tools=tools)

    def _handle(self, tool_call) -> dict:
        """Resolve and execute a tool call, returning the tool message for the model."""
        function = getattr(tool_call, 'function', None)
        name = getattr(function, 'name', None) or '(unknown)'
        try:
            arguments = json.loads(getattr(function, 'arguments', None) or '{}')
            if self._options.verbose:
                print(f'Tool call: {name}({json.dumps(arguments)})')
            command = self._provider.resolve(name, arguments)
            result = self.execute(command)
            content = truncate_output(result.output, self._options.output_limit) or '(no output)'
        except (json.JSONDecodeError, ToolError) as e:
            content = f'error: {e}'
            print(f'[ros2ai]: tool call {name} failed, {e}')
        if self._options.verbose:
            print(f'Tool result: {content}')
        return _tool_message(tool_call.id, content)


def _tool_message(tool_call_id: str, content: str) -> dict:
    """Create the tool message returning the result of a tool call to the model."""
    return {'role': 'tool', 'tool_call_id': tool_call_id, 'content': content}


def _assistant_message(message) -> dict:
    """Convert the completion message into the assistant message of the conversation."""
    assistant = {'role': 'assistant'}
    if message.content:
        assistant['content'] = message.content
    assistant['tool_calls'] = [
        {
            'id': tool_call.id,
            'type': 'function',
            'function': {
                'name': tool_call.function.name,
                'arguments': tool_call.function.arguments,
            },
        }
        for tool_call in message.tool_calls
    ]
    return assistant
