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

from ros2ai.api.client import chat_messages, ChatCompletionClient, ChatCompletionParameters
import ros2ai.api.constants as constants
from ros2ai.api.tools import ToolCommand, ToolError, ToolPolicy, ToolProvider, ToolResult
from ros2ai.api.utils import run_executable, truncate_output


@dataclass
class AgentOptions:
    """User options of the agent loop."""

    # print the commands that change the system instead of executing them
    dry_run: bool = False
    # execute the commands that change the system without confirmation
    auto_approve: bool = False
    # print the tool calls and the tool results
    debug: bool = False
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

    def run(self, system_role: str, request: str) -> int:
        """
        Process the user request until the model answers without tool calls.

        :param system_role: content of the system message.
        :param request: content of the user message.
        :return: 0 on success, 1 on failure, or the return code of a long-running command.
        """
        messages = chat_messages(system_role, request)
        tools = self._provider.definitions()
        self.steps = 0
        self.candidates = 0
        try:
            for _ in range(self._options.max_steps):
                message = self._complete(messages, tools)
                if message is None:
                    return 1
                tool_calls = getattr(message, 'tool_calls', None) or []
                if not tool_calls:
                    print(message.content or '')
                    return 0
                messages.append(_assistant_message(message))
                for tool_call in tool_calls:
                    messages.append(self._handle(tool_call))
                if self.candidates:
                    # dry-run: the commands that change the system are printed, stop here
                    return 0
        except _Exit as e:
            return e.returncode
        print(f'Reached the maximum number of steps ({self._options.max_steps}).')
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
        if result.output:
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

    def _complete(self, messages: List[dict], tools: List[dict]):
        parameters = ChatCompletionParameters(messages=messages, stream=False, tools=tools)
        self.steps += 1
        if not self._client.call(parameters):
            return None
        if self._options.debug:
            self._client.print_all()
        message = self._client.get_message()
        if message is None:
            # completion did not finish, the reason is printed
            self._client.print_result()
        return message

    def _handle(self, tool_call) -> dict:
        """Resolve and execute a tool call, returning the tool message for the model."""
        function = getattr(tool_call, 'function', None)
        name = getattr(function, 'name', None) or '(unknown)'
        try:
            arguments = json.loads(getattr(function, 'arguments', None) or '{}')
            if self._options.debug:
                print(f'Tool call: {name}({json.dumps(arguments)})')
            command = self._provider.resolve(name, arguments)
            result = self.execute(command)
            content = truncate_output(result.output, self._options.output_limit) or '(no output)'
        except (json.JSONDecodeError, ToolError) as e:
            content = f'error: {e}'
            print(f'[ros2ai]: tool call {name} failed, {e}')
        if self._options.debug:
            print(f'Tool result: {content}')
        return {'role': 'tool', 'tool_call_id': tool_call.id, 'content': content}


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
