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

"""
Tools that the LLM can call to inspect and operate the ROS 2 system.

The default provider maps every tool to the `ros2` command line interface, which is the stable
public interface of each ROS distribution. Commands are always executed as an argument list
without a shell, `argv[0]` is set by ros2ai, never by the model.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum
import shlex
import subprocess
from typing import Dict, List, Optional, Sequence, Tuple

import ros2ai.api.constants as constants


class ToolPolicy(Enum):
    """How a tool call is allowed to be executed."""

    # does not change the ROS 2 system, executed automatically
    READ_ONLY = 'read-only'
    # changes the ROS 2 system, requires the user confirmation
    MUTATING = 'mutating'
    # runs until interrupted (e.g `ros2 run`), requires the user confirmation and runs foreground
    LONG_RUNNING = 'long-running'


class ToolError(Exception):
    """Raised when a tool call can not be resolved into a command."""


@dataclass
class ToolCommand:
    """Resolved tool call, ready to be executed."""

    argv: List[str]
    policy: ToolPolicy
    timeout: float = constants.ROS2AI_TOOL_TIMEOUT_SEC

    def display(self) -> str:
        """Return the command line string for the user."""
        return shlex.join(self.argv)


@dataclass
class ToolResult:
    """Output of an executed tool call, returned to the model."""

    output: str
    returncode: int


class ToolProvider(ABC):
    """Interface of a tool provider, e.g ROS 2 CLI or MCP server."""

    @abstractmethod
    def definitions(self) -> List[dict]:
        """Return the tool definitions in the OpenAI function calling format."""

    @abstractmethod
    def resolve(self, name: str, arguments: dict) -> ToolCommand:
        """
        Validate the arguments and resolve the tool call into a command.

        :raises ToolError: if the tool is unknown or the arguments are invalid.
        """

    @abstractmethod
    def run(self, command: ToolCommand) -> ToolResult:
        """Execute a resolved command and capture its output."""


@dataclass(frozen=True)
class Parameter:
    """Parameter of a ROS 2 CLI tool, translated into a command line argument."""

    description: str
    required: bool = True
    # 'string', 'integer' or 'array' (of strings)
    kind: str = 'string'
    # option flag that precedes the value, positional argument if None
    flag: Optional[str] = None


@dataclass(frozen=True)
class Ros2Tool:
    """ROS 2 CLI tool definition."""

    name: str
    description: str
    policy: ToolPolicy
    # command line arguments after `ros2`, before the parameters
    base: Tuple[str, ...]
    parameters: Dict[str, Parameter] = field(default_factory=dict)
    timeout: float = constants.ROS2AI_TOOL_TIMEOUT_SEC


# (command, verb) pairs of the generic `ros2` tool that do not change the ROS 2 system.
# `None` means every verb of the command is read-only.
READ_ONLY_VERBS = {
    'node': {'list', 'info'},
    'topic': {'list', 'info', 'type', 'find', 'echo', 'hz', 'bw', 'delay'},
    'service': {'list', 'type', 'find', 'info', 'echo'},
    'action': {'list', 'info', 'type', 'find'},
    'param': {'list', 'get', 'describe', 'dump'},
    'interface': {'list', 'show', 'package', 'packages', 'proto'},
    'pkg': {'list', 'executables', 'prefix', 'xml'},
    'lifecycle': {'list', 'get', 'nodes'},
    'bag': {'info', 'list'},
    'component': {'list', 'types'},
    'daemon': {'status'},
    'doctor': None,
    'wtf': None,
}

# (command, verb) pairs of the generic `ros2` tool that run until interrupted.
# `None` means every verb of the command is long-running.
LONG_RUNNING_VERBS = {
    'run': None,
    'launch': None,
    'bag': {'record', 'play'},
}

ROS2_TOOLS = (
    Ros2Tool(
        'ros2_node_list', 'List the running ROS 2 nodes.',
        ToolPolicy.READ_ONLY, ('node', 'list')),
    Ros2Tool(
        'ros2_node_info', 'Show the subscribers, publishers, services and actions of a node.',
        ToolPolicy.READ_ONLY, ('node', 'info'),
        {'node': Parameter('Node name, e.g /talker')}),
    Ros2Tool(
        'ros2_topic_list', 'List the available topics with their message types.',
        ToolPolicy.READ_ONLY, ('topic', 'list', '-t')),
    Ros2Tool(
        'ros2_topic_info', 'Show the message type, publishers and subscribers of a topic.',
        ToolPolicy.READ_ONLY, ('topic', 'info', '-v'),
        {'topic': Parameter('Topic name, e.g /chatter')}),
    Ros2Tool(
        'ros2_topic_echo', 'Receive a single message published on a topic.',
        ToolPolicy.READ_ONLY, ('topic', 'echo', '--once'),
        {'topic': Parameter('Topic name, e.g /chatter')},
        timeout=constants.ROS2AI_TOOL_ECHO_TIMEOUT_SEC),
    Ros2Tool(
        'ros2_service_list', 'List the available services with their service types.',
        ToolPolicy.READ_ONLY, ('service', 'list', '-t')),
    Ros2Tool(
        'ros2_action_list', 'List the available actions with their action types.',
        ToolPolicy.READ_ONLY, ('action', 'list', '-t')),
    Ros2Tool(
        'ros2_param_list', 'List the parameters of all nodes, or of a single node.',
        ToolPolicy.READ_ONLY, ('param', 'list'),
        {'node': Parameter('Node name, all nodes if omitted', required=False)}),
    Ros2Tool(
        'ros2_param_get', 'Get the value of a node parameter.',
        ToolPolicy.READ_ONLY, ('param', 'get'),
        {'node': Parameter('Node name'), 'name': Parameter('Parameter name')}),
    Ros2Tool(
        'ros2_interface_show', 'Show the definition of a message, service or action type.',
        ToolPolicy.READ_ONLY, ('interface', 'show'),
        {'type': Parameter('Interface type, e.g std_msgs/msg/String')}),
    Ros2Tool(
        'ros2_pkg_executables', 'List the executables of a package.',
        ToolPolicy.READ_ONLY, ('pkg', 'executables'),
        {'package': Parameter('Package name, e.g demo_nodes_cpp')}),
    Ros2Tool(
        'ros2_doctor', 'Check the ROS 2 setup and report potential issues.',
        ToolPolicy.READ_ONLY, ('doctor',)),
    Ros2Tool(
        'ros2_topic_pub', 'Publish a single message on a topic without waiting for subscribers.',
        # `--once` waits for a matching subscription by default, `-w 0` publishes immediately
        ToolPolicy.MUTATING, ('topic', 'pub', '--once', '-w', '0'),
        {
            'topic': Parameter('Topic name, e.g /chatter'),
            'type': Parameter('Message type, e.g std_msgs/msg/String'),
            'values': Parameter(
                'Message values in YAML, e.g "{data: hello}"', required=False),
        }),
    Ros2Tool(
        'ros2_service_call', 'Call a service.',
        ToolPolicy.MUTATING, ('service', 'call'),
        {
            'service': Parameter('Service name, e.g /add_two_ints'),
            'type': Parameter('Service type, e.g example_interfaces/srv/AddTwoInts'),
            'values': Parameter('Request values in YAML, e.g "{a: 1, b: 2}"', required=False),
        }),
    Ros2Tool(
        'ros2_param_set', 'Set the value of a node parameter.',
        ToolPolicy.MUTATING, ('param', 'set'),
        {
            'node': Parameter('Node name'),
            'name': Parameter('Parameter name'),
            'value': Parameter('Parameter value in YAML'),
        }),
    Ros2Tool(
        'ros2_action_send_goal', 'Send a goal to an action server and wait for the result.',
        ToolPolicy.MUTATING, ('action', 'send_goal'),
        {
            'action': Parameter('Action name, e.g /fibonacci'),
            'type': Parameter('Action type, e.g example_interfaces/action/Fibonacci'),
            'values': Parameter('Goal values in YAML, e.g "{order: 5}"', required=False),
        }),
    Ros2Tool(
        'ros2_run', 'Run an executable of a package, this keeps running until interrupted.',
        ToolPolicy.LONG_RUNNING, ('run',),
        {
            'package': Parameter('Package name, e.g demo_nodes_cpp'),
            'executable': Parameter('Executable name, e.g talker'),
            'args': Parameter(
                'Arguments passed to the executable', required=False, kind='array'),
        }),
    Ros2Tool(
        'ros2_launch', 'Start a launch file of a package, this keeps running until interrupted.',
        ToolPolicy.LONG_RUNNING, ('launch',),
        {
            'package': Parameter('Package name, e.g demo_nodes_cpp'),
            'launch_file': Parameter('Launch file name, e.g talker_listener_launch.py'),
            'args': Parameter(
                'Launch arguments, e.g ["use_sim_time:=true"]', required=False, kind='array'),
        }),
    Ros2Tool(
        'ros2_cli',
        'Run any other `ros2` command line, use this only when no dedicated tool exists. '
        'Pass `-h` to learn the usage of a sub-command.',
        # the policy is decided from the arguments, see Ros2CliProvider.classify()
        ToolPolicy.MUTATING, (),
        {'arguments': Parameter(
            'Arguments after `ros2`, e.g ["topic", "hz", "/chatter"]', kind='array')}),
)

GENERIC_TOOL_NAME = 'ros2_cli'


def _json_type(kind: str) -> dict:
    if kind == 'array':
        return {'type': 'array', 'items': {'type': 'string'}}
    return {'type': kind}


def _check_value(name: str, parameter: Parameter, value) -> List[str]:
    """Validate a parameter value and translate it into command line arguments."""
    if parameter.kind == 'array':
        if not isinstance(value, list) or not all(isinstance(v, str) for v in value):
            raise ToolError(f"parameter '{name}' must be a list of strings")
        values = value
    elif parameter.kind == 'integer':
        if isinstance(value, bool) or not isinstance(value, int):
            raise ToolError(f"parameter '{name}' must be an integer")
        values = [str(value)]
    else:
        if not isinstance(value, str) or value == '':
            raise ToolError(f"parameter '{name}' must be a non-empty string")
        values = [value]
    if parameter.flag is None:
        return values
    args = []
    for v in values:
        args.extend([parameter.flag, v])
    return args


class Ros2CliProvider(ToolProvider):
    """Tool provider that executes the `ros2` command line interface."""

    def __init__(self, tools: Sequence[Ros2Tool] = ROS2_TOOLS):
        self._tools = {tool.name: tool for tool in tools}

    @property
    def names(self) -> List[str]:
        return list(self._tools.keys())

    def definitions(self) -> List[dict]:
        definitions = []
        for tool in self._tools.values():
            properties = {}
            required = []
            for name, parameter in tool.parameters.items():
                properties[name] = {'description': parameter.description}
                properties[name].update(_json_type(parameter.kind))
                if parameter.required:
                    required.append(name)
            definitions.append({
                'type': 'function',
                'function': {
                    'name': tool.name,
                    'description': tool.description,
                    'parameters': {
                        'type': 'object',
                        'properties': properties,
                        'required': required,
                        'additionalProperties': False,
                    },
                },
            })
        return definitions

    @staticmethod
    def classify(arguments: Sequence[str]) -> ToolPolicy:
        """
        Decide the policy of a generic `ros2` command line from its arguments.

        The policy is decided from the first (command, verb) pair, options are skipped.
        Unknown commands are considered to change the system.
        """
        if any(arg in ('-h', '--help') for arg in arguments):
            return ToolPolicy.READ_ONLY
        words = [arg for arg in arguments if not arg.startswith('-')]
        command = words[0] if words else ''
        verb = words[1] if len(words) > 1 else ''
        for verbs, policy in (
                (LONG_RUNNING_VERBS, ToolPolicy.LONG_RUNNING),
                (READ_ONLY_VERBS, ToolPolicy.READ_ONLY)):
            if command in verbs and (verbs[command] is None or verb in verbs[command]):
                return policy
        return ToolPolicy.MUTATING

    def resolve(self, name: str, arguments: dict) -> ToolCommand:
        tool = self._tools.get(name)
        if tool is None:
            raise ToolError(f"unknown tool '{name}'")
        if not isinstance(arguments, dict):
            raise ToolError('arguments must be a JSON object')
        unknown = set(arguments) - set(tool.parameters)
        if unknown:
            raise ToolError(f"unknown parameter(s) {sorted(unknown)} for tool '{name}'")
        argv = ['ros2', *tool.base]
        for parameter_name, parameter in tool.parameters.items():
            if parameter_name not in arguments or arguments[parameter_name] is None:
                if parameter.required:
                    raise ToolError(f"missing required parameter '{parameter_name}'")
                continue
            argv.extend(_check_value(parameter_name, parameter, arguments[parameter_name]))
        policy = tool.policy
        if name == GENERIC_TOOL_NAME:
            if len(argv) == 1:
                raise ToolError('arguments must not be empty')
            policy = self.classify(argv[1:])
        return ToolCommand(argv=argv, policy=policy, timeout=tool.timeout)

    def run(self, command: ToolCommand) -> ToolResult:
        try:
            completed = subprocess.run(
                command.argv, capture_output=True, text=True, timeout=command.timeout)
        except subprocess.TimeoutExpired as e:
            output = _combine_output(e.stdout, e.stderr)
            output += f'\n[timeout after {command.timeout} seconds, output above is partial]'
            return ToolResult(output=output.strip(), returncode=-1)
        except OSError as e:
            return ToolResult(output=f'failed to execute: {e}', returncode=-1)
        output = _combine_output(completed.stdout, completed.stderr)
        if completed.returncode != 0:
            output += f'\n[exit code {completed.returncode}]'
        return ToolResult(output=output.strip(), returncode=completed.returncode)


def _combine_output(stdout, stderr) -> str:
    """Join the stripped stdout and stderr, either may be None, str or bytes."""
    parts = []
    for stream in (stdout, stderr):
        if isinstance(stream, bytes):
            stream = stream.decode(errors='replace')
        if stream and stream.strip():
            parts.append(stream.strip())
    return '\n'.join(parts)
