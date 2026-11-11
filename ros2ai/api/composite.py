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
Composite tools that answer a diagnostic question with several `ros2` commands.

Each tool runs a fixed sequence of read-only `ros2` commands through the executor of the
provider, condenses their output and returns a single YAML document. The model gets the whole
picture in one tool call, within a bounded size, instead of deciding the command sequence
itself with one completion per command.
"""

import re
from typing import Any, Dict, List, Optional, Sequence, Tuple

import ros2ai.api.constants as constants
from ros2ai.api.tools import CompositeTool, Executor, Parameter, ToolPolicy, ToolResult

import yaml

# Topics that every node publishes, hidden from the status to keep it short.
STANDARD_TOPICS = ('/parameter_events', '/rosout')
# Services that every node (parameters, type description) or every lifecycle node serves,
# relative to the node name, hidden from the status to keep it short.
STANDARD_SERVICES = (
    '/describe_parameters', '/get_parameter_types', '/get_parameters', '/list_parameters',
    '/set_parameters', '/set_parameters_atomically', '/get_type_description',
    '/change_state', '/get_state', '/get_available_states', '/get_available_transitions',
    '/get_transition_graph',
)
LIFECYCLE_GET_STATE_TYPE = 'lifecycle_msgs/srv/GetState'
TRANSITION_EVENT_TYPE = 'lifecycle_msgs/msg/TransitionEvent'
NODE_INFO_SECTIONS = (
    'Subscribers', 'Publishers', 'Service Servers', 'Service Clients',
    'Action Servers', 'Action Clients',
)
UNKNOWN_NODE_NAME = '_NODE_NAME_UNKNOWN_'

_ANSI_ESCAPE = re.compile(r'\x1b\[[0-9;]*m')
_LIFECYCLE_STATE = re.compile(r'^\s*([a-z_]+) \[(\d+)\]\s*$')
_AVERAGE_RATE = re.compile(r'average rate: ([0-9.]+)')


def parse_names_and_types(output: str) -> Dict[str, List[Tuple[str, str]]]:
    """
    Parse the output of `ros2 node info` into the (name, type) pairs of each section.

    :param output: output of the command.
    :return: the pairs of each section in `NODE_INFO_SECTIONS`, empty if absent.
    """
    sections: Dict[str, List[Tuple[str, str]]] = {section: [] for section in NODE_INFO_SECTIONS}
    current = None
    for line in _ANSI_ESCAPE.sub('', output).splitlines():
        stripped = line.strip()
        if stripped.endswith(':') and stripped[:-1] in sections:
            current = sections[stripped[:-1]]
        elif current is not None and line.startswith('    ') and ': ' in stripped:
            name, type_name = stripped.split(': ', 1)
            current.append((name, type_name))
    return sections


def parse_lifecycle_state(output: str) -> Optional[str]:
    """
    Parse the output of `ros2 lifecycle get` into the state label.

    :param output: output of the command, e.g `active [3]`.
    :return: the state label, None if the output does not contain a state.
    """
    state = None
    for line in output.splitlines():
        match = _LIFECYCLE_STATE.match(line)
        if match:
            state = match.group(1)
    return state


def parse_parameter_names(output: str) -> List[str]:
    """
    Parse the output of `ros2 param list` into the parameter names.

    :param output: output of the command, the names are indented, one per line.
    :return: the parameter names.
    """
    names = []
    for line in output.splitlines():
        # `--param-type` appends ` (type: ...)` to the name
        name = line.strip().split(' (type: ')[0]
        # the parameter names have no blank, unlike the node headers (`/talker:`), the DDS
        # noise and the error messages. the indentation is not reliable, the provider strips
        # the output of the command.
        if name and ' ' not in name and not name.endswith(':'):
            names.append(name)
    return names


def parse_rate(output: str) -> Optional[float]:
    """
    Parse the output of `ros2 topic hz` into the last reported average rate.

    :param output: output of the command, possibly empty when no message was received.
    :return: the average rate in Hz, None if no rate was reported.
    """
    rates = _AVERAGE_RATE.findall(output)
    if not rates:
        return None
    try:
        return round(float(rates[-1]), 2)
    except ValueError:
        return None


def parse_topic_info(output: str) -> Dict[str, Any]:
    """
    Parse the verbose output of `ros2 topic info -v` into the type and the endpoints.

    :param output: output of the command.
    :return: dict with the `type` and the `publishers` and `subscribers` endpoints, each
        endpoint has the `node`, `reliability` and `durability` keys.
    """
    info: Dict[str, Any] = {'type': None, 'publishers': [], 'subscribers': []}
    endpoint = None
    namespace = ''
    for line in output.splitlines():
        stripped = line.strip()
        if stripped.startswith('Type: '):
            info['type'] = stripped[len('Type: '):]
        elif stripped.startswith('Node name: '):
            endpoint = {'node': stripped[len('Node name: '):]}
            namespace = ''
        elif endpoint is None:
            continue
        elif stripped.startswith('Node namespace: '):
            namespace = stripped[len('Node namespace: '):]
        elif stripped.startswith('Endpoint type: '):
            endpoint['node'] = _full_node_name(namespace, endpoint['node'])
            endpoint['reliability'] = None
            endpoint['durability'] = None
            kind = stripped[len('Endpoint type: '):]
            key = 'publishers' if kind == 'PUBLISHER' else 'subscribers'
            info[key].append(endpoint)
        elif stripped.startswith('Reliability: '):
            endpoint['reliability'] = stripped[len('Reliability: '):]
        elif stripped.startswith('Durability: '):
            endpoint['durability'] = stripped[len('Durability: '):]
    return info


def _full_node_name(namespace: str, name: str) -> str:
    """Join the node namespace and name, unknown remote participants are reported as such."""
    if name == UNKNOWN_NODE_NAME:
        return 'unknown'
    return namespace.rstrip('/') + '/' + name


def _absolute(name: str) -> str:
    """Return the name with the leading slash, as reported by the `ros2` commands."""
    return name if name.startswith('/') else '/' + name


def _first_line(output: str) -> str:
    """Return the first non-empty line of an output, for the warnings."""
    for line in output.splitlines():
        if line.strip():
            return line.strip()
    return '(no output)'


def _limited(items: Sequence[str], limit: int = constants.ROS2AI_LIST_LIMIT) -> List[str]:
    """Return the first items, with a trailer telling how many are omitted."""
    items = list(items)
    if len(items) <= limit:
        return items
    return items[:limit] + [f'... and {len(items) - limit} more']


def _dump(status: Dict[str, Any]) -> str:
    """Serialize the status into YAML, keeping the key order."""
    return yaml.safe_dump(
        status, sort_keys=False, default_flow_style=False, allow_unicode=True, width=200,
    ).rstrip()


def _window(arguments: Dict[str, Any]) -> int:
    return arguments.get('window', constants.ROS2AI_RATE_WINDOW_SEC)


_WINDOW_PARAMETER = Parameter(
    'Seconds to measure the publishing rates, '
    f'1 to {constants.ROS2AI_RATE_WINDOW_MAX_SEC} (default {constants.ROS2AI_RATE_WINDOW_SEC}), '
    'use a longer window for slow topics',
    required=False, kind='integer', minimum=1, maximum=constants.ROS2AI_RATE_WINDOW_MAX_SEC)


class NodeStatusTool(CompositeTool):
    """Status of a node: interfaces, publishing rates, lifecycle state and parameters."""

    name = 'get_node_status'
    description = (
        'Report the status of a node in one call: publishers with their publishing rates, '
        'subscribers, services, actions, lifecycle state and parameter names. '
        'Use this for health or status questions about a node, instead of calling '
        'node info, lifecycle get, param list and topic hz separately.')
    policy = ToolPolicy.READ_ONLY
    parameters = {
        'node': Parameter('Node name, e.g /talker'),
        'window': _WINDOW_PARAMETER,
    }

    def run(self, executor: Executor, arguments: Dict[str, Any]) -> ToolResult:
        node = _absolute(arguments['node'])
        window = _window(arguments)
        status: Dict[str, Any] = {'node': node, 'found': False}
        warnings: List[str] = []

        info = executor.run(['node', 'info', node])
        if info.returncode != 0:
            warnings.append(f'ros2 node info {node} failed: {_first_line(info.output)}')
            nodes = executor.run(['node', 'list'])
            status['available_nodes'] = _limited(nodes.output.split())
            status['warnings'] = warnings
            return ToolResult(output=_dump(status), returncode=1)
        status['found'] = True
        sections = parse_names_and_types(info.output)

        # lifecycle state, only for the lifecycle nodes to avoid waiting for a missing service
        lifecycle = any(
            name == f'{node}/get_state' and type_name == LIFECYCLE_GET_STATE_TYPE
            for name, type_name in sections['Service Servers'])
        if lifecycle:
            result = executor.run(
                ['lifecycle', 'get', node], timeout=constants.ROS2AI_COMPOSITE_STEP_TIMEOUT_SEC)
            state = parse_lifecycle_state(result.output)
            if state is None:
                warnings.append(
                    f'ros2 lifecycle get {node} failed: {_first_line(result.output)}')
                status['lifecycle'] = 'unknown'
            else:
                status['lifecycle'] = state
                if state != 'active':
                    warnings.append(f"lifecycle state is '{state}', not active")
        else:
            status['lifecycle'] = 'not a lifecycle node'

        # interfaces, the standard ones every node has are hidden
        publishers = [
            (name, type_name) for name, type_name in sections['Publishers']
            if name not in STANDARD_TOPICS]
        subscribers = [
            (name, type_name) for name, type_name in sections['Subscribers']
            if name not in STANDARD_TOPICS]
        standard_services = {f'{node}{service}' for service in STANDARD_SERVICES}

        # publishing rates, measured in parallel within the window
        candidates = [name for name, type_name in publishers if type_name != TRANSITION_EVENT_TYPE]
        measured = candidates[:constants.ROS2AI_RATE_TOPICS_LIMIT]
        if len(candidates) > len(measured):
            warnings.append(
                f'publishing rates are measured for the first {len(measured)} topics only')
        rates: Dict[str, Any] = {name: 'not measured' for name, _ in publishers}
        if measured:
            results = executor.sample([['topic', 'hz', name] for name in measured], window)
            for name, result in zip(measured, results):
                rates[name] = parse_rate(result.output)
                if rates[name] is None:
                    warnings.append(
                        f"no publishing rate measured on '{name}' within {window} seconds "
                        '(idle or slow topic)')
        status['publishers'] = [
            {'topic': name, 'type': type_name, 'hz': rates[name]}
            for name, type_name in publishers]
        status['subscribers'] = dict(subscribers)
        status['service_servers'] = {
            name: type_name for name, type_name in sections['Service Servers']
            if name not in standard_services}
        status['service_clients'] = dict(sections['Service Clients'])
        status['action_servers'] = dict(sections['Action Servers'])
        status['action_clients'] = dict(sections['Action Clients'])

        # parameter names, the values are available with the parameter tools
        result = executor.run(
            ['param', 'list', node], timeout=constants.ROS2AI_COMPOSITE_STEP_TIMEOUT_SEC)
        names = parse_parameter_names(result.output)
        if result.returncode != 0 and not names:
            warnings.append(f'ros2 param list {node} failed: {_first_line(result.output)}')
            status['parameters'] = None
        else:
            status['parameters'] = _limited(names)

        status['warnings'] = warnings
        return ToolResult(output=_dump(status), returncode=0)


class TopicStatusTool(CompositeTool):
    """Status of a topic: type, endpoints with QoS, publishing rate and a sample message."""

    name = 'get_topic_status'
    description = (
        'Report the status of a topic in one call: message type, publisher and subscriber '
        'nodes with their QoS, publishing rate, a sample message and QoS incompatibilities. '
        'Use this for health or status questions about a topic, instead of calling '
        'topic info, topic hz and topic echo separately.')
    policy = ToolPolicy.READ_ONLY
    parameters = {
        'topic': Parameter('Topic name, e.g /chatter'),
        'window': _WINDOW_PARAMETER,
    }

    def run(self, executor: Executor, arguments: Dict[str, Any]) -> ToolResult:
        topic = _absolute(arguments['topic'])
        window = _window(arguments)
        status: Dict[str, Any] = {'topic': topic, 'found': False}
        warnings: List[str] = []

        info = executor.run(['topic', 'info', '-v', topic])
        if info.returncode != 0:
            warnings.append(f'ros2 topic info -v {topic} failed: {_first_line(info.output)}')
            topics = executor.run(['topic', 'list'])
            status['available_topics'] = _limited(topics.output.split())
            status['warnings'] = warnings
            return ToolResult(output=_dump(status), returncode=1)
        status['found'] = True
        parsed = parse_topic_info(info.output)
        status['type'] = parsed['type']
        status['publishers'] = parsed['publishers']
        status['subscribers'] = parsed['subscribers']
        warnings.extend(_qos_warnings(parsed['publishers'], parsed['subscribers']))

        status['hz'] = None
        status['sample'] = None
        if not parsed['publishers']:
            warnings.append('no publishers on the topic')
        else:
            results = executor.sample(
                [['topic', 'hz', topic], ['topic', 'echo', '--once', topic]], window)
            status['hz'] = parse_rate(results[0].output)
            if status['hz'] is None:
                warnings.append(
                    f'no publishing rate measured within {window} seconds (idle or slow topic)')
            status['sample'] = _sample(results[1].output)
            if status['sample'] is None:
                warnings.append(f'no message received within {window} seconds')

        status['warnings'] = warnings
        return ToolResult(output=_dump(status), returncode=0)


def _qos_warnings(publishers: List[dict], subscribers: List[dict]) -> List[str]:
    """Report the subscribers that can not receive from a publisher because of the QoS."""
    warnings = []
    for subscriber in subscribers:
        for publisher in publishers:
            for policy, requested, offered in (
                    ('reliability', 'RELIABLE', 'BEST_EFFORT'),
                    ('durability', 'TRANSIENT_LOCAL', 'VOLATILE')):
                if subscriber[policy] == requested and publisher[policy] == offered:
                    warnings.append(
                        f"incompatible {policy}: subscriber {subscriber['node']} requests "
                        f"{requested} but publisher {publisher['node']} offers {offered}")
    return warnings


def _sample(output: str) -> Optional[str]:
    """Return the sample message of `ros2 topic echo --once`, limited in lines."""
    lines = [line for line in output.splitlines() if line.strip() != '---']
    if not any(line.strip() for line in lines):
        return None
    if len(lines) > constants.ROS2AI_SAMPLE_LINES_LIMIT:
        omitted = len(lines) - constants.ROS2AI_SAMPLE_LINES_LIMIT
        lines = lines[:constants.ROS2AI_SAMPLE_LINES_LIMIT] + [f'... [{omitted} lines omitted]']
    return '\n'.join(lines)


COMPOSITE_TOOLS = (NodeStatusTool(), TopicStatusTool())
