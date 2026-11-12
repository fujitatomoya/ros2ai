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

from pathlib import Path
from typing import Tuple

from ros2ai.api.agent import Agent, AgentOptions
from ros2ai.api.arguments import add_arguments, EXAMPLES
from ros2ai.api.client import ChatCompletionClient, ChatCompletionParameters
from ros2ai.api.config import get_context_limit, OpenAiConfig, resolve_role_system
from ros2ai.api.constants import ROLE_SYSTEM_DEFAULT
from ros2ai.api.session import Conversation, default_session_file, interactive_loop
from ros2ai.api.tools import Ros2CliProvider
from ros2cli.command import CommandExtension

# user message sent to check that the configured model answers
AVAILABILITY_CHECK_MESSAGES = [{'role': 'user', 'content': 'Are you available?'}]


class AiCommand(CommandExtension):
    """
    ROS 2 LLM command line interface. ('ROS2AI_API_KEY' is required for OpenAI and Anthropic).

    Ask any question about ROS 2 or request to operate the ROS 2 system in natural language,
    the assistant answers or calls the ROS 2 CLI tools on behalf of the user.
    Without a request, the interactive session starts and keeps the context of the requests.
    Any LLM service that provides the OpenAI compatible API can be configured.
    """

    def add_arguments(self, parser, cli_name):
        self._parser = parser
        parser.epilog = EXAMPLES
        add_arguments(parser)

    def main(self, *, parser, args):
        if args.status or args.list_models:
            return check_status(args)
        request = ' '.join(args.request)
        if request == '':
            return run_session(args)
        return run_request(args, request)


def check_status(args) -> int:
    """
    Check that the configured LLM API endpoint and model answer.

    :param args: parsed arguments.
    :return: 0 if the endpoint answers and the models can be listed, 1 otherwise.
    """
    config = OpenAiConfig(args)
    if args.verbose:
        config.display_all()

    # try to call LLM API with user configured setting
    client = ChatCompletionClient(config)
    if client.call(ChatCompletionParameters(
            messages=AVAILABILITY_CHECK_MESSAGES, stream=False)):
        client.print_result()
    if client.get_result() is None:
        print('[FAILURE] Invalid API key.')
        return 1
    if args.verbose:
        print('[SUCCESS] Valid API key.')

    # try to list the all models via user configured api key
    can_get_models, model_list = client.list_models()
    if not can_get_models:
        print('[FAILURE] Could not retrieved list of models.')
        return 1
    if args.verbose:
        print('[SUCCESS] Retrieved list of models.')

    if args.list_models:
        print('Available Models:')
        for model in model_list:
            print(model)

    return 0


class Session:
    """
    Session of the assistant, the agent and the conversation shared by the requests.

    The conversation is saved to the session file after every request, so that a later
    invocation can continue it with `--continue`.
    """

    def __init__(self, args):
        self._args = args
        self._config = OpenAiConfig(args)
        self._client = ChatCompletionClient(self._config)
        options = AgentOptions(
            dry_run=args.dry_run, auto_approve=args.yes, stream=not args.nostream,
            verbose=args.verbose)
        self._agent = Agent(self._client, Ros2CliProvider(), options)
        self._path = default_session_file()
        system_role = resolve_role_system(args, ROLE_SYSTEM_DEFAULT)
        if args.verbose:
            print(f'System role:\n{system_role}')
        self.conversation = Conversation(system_role, limit=get_context_limit())
        if args.continue_session:
            self._load()

    def _load(self):
        if not self._path.exists():
            print(f'[ros2ai]: no previous session to continue ({self._path}).')
            return
        try:
            metadata = self.conversation.load(self._path)
        except ValueError as e:
            print(f'[ros2ai]: {e}')
            return
        if self._args.verbose:
            print(f'[ros2ai]: continuing the session of {self.conversation.turns} turns, '
                  f"model: {metadata.get('model')}, updated: {metadata.get('updated')}.")

    def save(self):
        """Save the conversation to the session file, the failure is reported but not fatal."""
        metadata = {
            'model': self._config.get_value('api_model'),
            'endpoint': self._config.get_value('api_endpoint'),
        }
        try:
            self.conversation.save(self._path, metadata)
        except OSError as e:
            print(f'[ros2ai]: failed to save the session to {self._path}: {e}')

    def run(self, request: str) -> int:
        """
        Process the user request with the agent loop, the model answers or calls the tools.

        :param request: the user request.
        :return: 0 on success, 1 on failure, or the return code of a long-running command.
        """
        agent, conversation = self._agent, self.conversation
        returncode = agent.run(conversation, request)
        if returncode == 1 and agent.steps == 1 and _tools_unsupported(self._client.get_error()):
            # endpoint or model does not support tool calling, the model can still answer
            print('Tool calling is not supported, answering without tools.')
            returncode = agent.run(conversation, request, tools=False)
        self.save()
        return returncode

    def path(self) -> Path:
        """Get the path of the session file."""
        return self._path

    def describe(self) -> Tuple[str, str]:
        """Get the model and the endpoint of the session."""
        return self._config.get_value('api_model'), self._config.get_value('api_endpoint')


def run_request(args, request: str) -> int:
    """
    Process a single user request, continuing the previous session with `--continue`.

    :param args: parsed arguments.
    :param request: the user request.
    :return: 0 on success, 1 on failure, or the return code of a long-running command.
    """
    return Session(args).run(request)


def run_session(args) -> int:
    """
    Run the interactive session, the requests are read from the prompt until the session ends.

    :param args: parsed arguments.
    :return: 0 when the session ends.
    """
    session = Session(args)
    model, endpoint = session.describe()
    print(f'ros2ai interactive session (model: {model}, endpoint: {endpoint})\n'
          'Type /help for the commands, exit or Ctrl-D to end the session.')
    return interactive_loop(session.conversation, session.run, on_clear=session.save)


def _tools_unsupported(error) -> bool:
    """Check if the API error reports that tools (function calling) are not supported."""
    return error is not None and 'tool' in str(error).lower()
