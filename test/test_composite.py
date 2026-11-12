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

"""Tests of the composite tools with a fake executor, no ROS 2 graph required."""

import subprocess

import pytest
from ros2ai.api.composite import COMPOSITE_TOOLS
from ros2ai.api.composite import NodeStatusTool
from ros2ai.api.composite import parse_lifecycle_state
from ros2ai.api.composite import parse_names_and_types
from ros2ai.api.composite import parse_parameter_names
from ros2ai.api.composite import parse_rate
from ros2ai.api.composite import parse_topic_info
from ros2ai.api.composite import TopicStatusTool
import ros2ai.api.constants as constants
from ros2ai.api.tools import CompositeCommand
from ros2ai.api.tools import CompositeTool
from ros2ai.api.tools import Executor
from ros2ai.api.tools import ROS2_TOOLS
from ros2ai.api.tools import Ros2CliProvider
from ros2ai.api.tools import ToolError
from ros2ai.api.tools import ToolPolicy
from ros2ai.api.tools import ToolResult
import yaml

# Outputs captured from `ros2` on Rolling and Humble.
NODE_INFO_TALKER_ROLLING = """\
/talker
  Subscribers:

  Publishers:
    /chatter: example_interfaces/msg/String
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /talker/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /talker/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /talker/get_parameters: rcl_interfaces/srv/GetParameters
    /talker/get_type_description: type_description_interfaces/srv/GetTypeDescription
    /talker/list_parameters: rcl_interfaces/srv/ListParameters
    /talker/set_parameters: rcl_interfaces/srv/SetParameters
    /talker/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:

"""
NODE_INFO_TALKER_HUMBLE = """\
/talker
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /chatter: std_msgs/msg/String
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /talker/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /talker/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /talker/get_parameters: rcl_interfaces/srv/GetParameters
    /talker/list_parameters: rcl_interfaces/srv/ListParameters
    /talker/set_parameters: rcl_interfaces/srv/SetParameters
    /talker/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:

"""
NODE_INFO_LC_TALKER = """\
/lc_talker
  Subscribers:

  Publishers:
    /lc_talker/transition_event: lifecycle_msgs/msg/TransitionEvent
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /lc_talker/change_state: lifecycle_msgs/srv/ChangeState
    /lc_talker/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /lc_talker/get_available_states: lifecycle_msgs/srv/GetAvailableStates
    /lc_talker/get_available_transitions: lifecycle_msgs/srv/GetAvailableTransitions
    /lc_talker/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /lc_talker/get_parameters: rcl_interfaces/srv/GetParameters
    /lc_talker/get_state: lifecycle_msgs/srv/GetState
    /lc_talker/get_transition_graph: lifecycle_msgs/srv/GetAvailableTransitions
    /lc_talker/get_type_description: type_description_interfaces/srv/GetTypeDescription
    /lc_talker/list_parameters: rcl_interfaces/srv/ListParameters
    /lc_talker/set_parameters: rcl_interfaces/srv/SetParameters
    /lc_talker/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:

"""
NODE_INFO_FIBONACCI = """\
/fibonacci_action_server
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /fibonacci_action_server/get_parameters: rcl_interfaces/srv/GetParameters
    /add_two_ints: example_interfaces/srv/AddTwoInts
  Service Clients:
    /other/get_state: lifecycle_msgs/srv/GetState
  Action Servers:
    /fibonacci: example_interfaces/action/Fibonacci
  Action Clients:

"""
PARAM_LIST_TALKER = """\
  qos_overrides./parameter_events.publisher.depth
  qos_overrides./parameter_events.publisher.durability
  qos_overrides./parameter_events.publisher.history
  qos_overrides./parameter_events.publisher.reliability
  use_sim_time
"""
HZ_CHATTER = 'average rate: 1.000\n\tmin: 1.000s max: 1.000s std dev: 0.00000s window: 1\n'
HZ_ROSOUT = (
    'average rate: 10.945\n\tmin: 0.000s max: 0.890s std dev: 0.26618s window: 10\n'
    'average rate: 5.226\n\tmin: 0.000s max: 0.890s std dev: 0.35068s window: 10\n')
HZ_UNPUBLISHED = 'WARNING: topic [/nonexistent] does not appear to be published yet\n'
TOPIC_INFO_CHATTER_ROLLING = """\
Type: example_interfaces/msg/String

Publisher count: 1

Node name: talker
Node namespace: /
Topic type: example_interfaces/msg/String
Topic type hash: RIHS01_5509d866a579951f2fc6c19577c32605ba16f308cae7b498341d79536d4eb06b
Endpoint type: PUBLISHER
GID: 01.0f.8a.2f.72.00.03.16.00.00.00.00.00.00.13.03
QoS profile:
  Reliability: RELIABLE
  History (Depth): KEEP_LAST (7)
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Subscription count: 1

Node name: listener
Node namespace: /
Topic type: example_interfaces/msg/String
Topic type hash: RIHS01_5509d866a579951f2fc6c19577c32605ba16f308cae7b498341d79536d4eb06b
Endpoint type: SUBSCRIPTION
GID: 01.0f.8a.2f.73.00.2d.68.00.00.00.00.00.00.13.04
QoS profile:
  Reliability: RELIABLE
  History (Depth): KEEP_LAST (10)
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

"""
TOPIC_INFO_CHATTER_HUMBLE = """\
Type: std_msgs/msg/String

Publisher count: 1

Node name: talker
Node namespace: /
Topic type: std_msgs/msg/String
Endpoint type: PUBLISHER
GID: 01.0f.70.b7.47.01.5c.19.00.00.00.00.00.00.12.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Subscription count: 1

Node name: listener
Node namespace: /
Topic type: std_msgs/msg/String
Endpoint type: SUBSCRIPTION
GID: 01.0f.70.b7.45.01.45.17.00.00.00.00.00.00.12.04.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

"""
TOPIC_INFO_SCAN_MISMATCH = """\
Type: sensor_msgs/msg/LaserScan

Publisher count: 1

Node name: lidar
Node namespace: /sensors
Topic type: sensor_msgs/msg/LaserScan
Endpoint type: PUBLISHER
GID: 01.0f.70.b7.47.01.5c.19.00.00.00.00.00.00.12.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: BEST_EFFORT
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Subscription count: 2

Node name: _NODE_NAME_UNKNOWN_
Node namespace: _NODE_NAMESPACE_UNKNOWN_
Topic type: sensor_msgs/msg/LaserScan
Endpoint type: SUBSCRIPTION
GID: 01.0f.70.b7.45.01.45.17.00.00.00.00.00.00.12.04.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE
  History (Depth): UNKNOWN
  Durability: TRANSIENT_LOCAL
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Node name: viewer
Node namespace: /
Topic type: sensor_msgs/msg/LaserScan
Endpoint type: SUBSCRIPTION
GID: 01.0f.70.b7.45.01.45.17.00.00.00.00.00.00.12.04.00.00.00.00.00.00.00.01
QoS profile:
  Reliability: BEST_EFFORT
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

"""
TOPIC_INFO_NO_PUBLISHER = """\
Type: std_msgs/msg/String

Publisher count: 0

Subscription count: 1

Node name: listener
Node namespace: /
Topic type: std_msgs/msg/String
Endpoint type: SUBSCRIPTION
GID: 01.0f.70.b7.45.01.45.17.00.00.00.00.00.00.12.04.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

"""
ECHO_CHATTER = "data: 'Hello World: 18'\n---\n"


class FakeExecutor(Executor):
    """Executor returning canned outputs keyed by the arguments after `ros2`."""

    def __init__(self, outputs):
        # {arguments tuple: output string, or (output, returncode) tuple}
        self.outputs = outputs
        self.calls = []
        self.samples = []

    def _result(self, arguments):
        if tuple(arguments) not in self.outputs:
            pytest.fail(f'unexpected command: ros2 {" ".join(arguments)}')
        value = self.outputs[tuple(arguments)]
        if isinstance(value, str):
            value = (value, 0)
        # the provider strips the output of the commands, so does the fake
        return ToolResult(value[0].strip(), value[1])

    def run(self, arguments, timeout=constants.ROS2AI_TOOL_TIMEOUT_SEC):
        self.calls.append((list(arguments), timeout))
        return self._result(arguments)

    def sample(self, arguments_list, seconds):
        self.samples.append(([list(arguments) for arguments in arguments_list], seconds))
        return [self._result(arguments) for arguments in arguments_list]


def run_tool(tool, outputs, **arguments):
    """Run a composite tool with the canned outputs, returning the parsed YAML status."""
    executor = FakeExecutor(outputs)
    result = tool.run(executor, arguments)
    return executor, result, yaml.safe_load(result.output)


# parsers


@pytest.mark.parametrize('output, chatter_type, subscribers', [
    (NODE_INFO_TALKER_ROLLING, 'example_interfaces/msg/String', []),
    (NODE_INFO_TALKER_HUMBLE, 'std_msgs/msg/String',
     [('/parameter_events', 'rcl_interfaces/msg/ParameterEvent')]),
])
def test_parse_names_and_types(output, chatter_type, subscribers):
    sections = parse_names_and_types(output)
    assert sections['Subscribers'] == subscribers
    assert sections['Publishers'] == [
        ('/chatter', chatter_type),
        ('/parameter_events', 'rcl_interfaces/msg/ParameterEvent'),
        ('/rosout', 'rcl_interfaces/msg/Log'),
    ]
    assert ('/talker/get_parameters', 'rcl_interfaces/srv/GetParameters') \
        in sections['Service Servers']
    assert sections['Service Clients'] == []
    assert sections['Action Servers'] == []
    assert sections['Action Clients'] == []


def test_parse_names_and_types_all_sections_and_colors():
    colored = NODE_INFO_FIBONACCI.replace('  Action Servers:', '\x1b[32m  Action Servers:\x1b[0m')
    sections = parse_names_and_types(colored)
    assert sections['Service Clients'] == [('/other/get_state', 'lifecycle_msgs/srv/GetState')]
    assert sections['Action Servers'] == [('/fibonacci', 'example_interfaces/action/Fibonacci')]
    assert sections['Action Clients'] == []
    assert parse_names_and_types("Unable to find node '/x'") == {
        section: [] for section in parse_names_and_types('')}


@pytest.mark.parametrize('output, state', [
    ('active [3]\n', 'active'),
    ('unconfigured [1]', 'unconfigured'),
    ('sequence size exceeds remaining buffer\nsequence size exceeds remaining buffer\n'
     'inactive [2]\n', 'inactive'),
    ('Node not found\n[exit code 1]', None),
    ('', None),
])
def test_parse_lifecycle_state(output, state):
    assert parse_lifecycle_state(output) == state


def test_parse_parameter_names():
    assert parse_parameter_names(PARAM_LIST_TALKER) == [
        'qos_overrides./parameter_events.publisher.depth',
        'qos_overrides./parameter_events.publisher.durability',
        'qos_overrides./parameter_events.publisher.history',
        'qos_overrides./parameter_events.publisher.reliability',
        'use_sim_time',
    ]
    noisy = 'sequence size exceeds remaining buffer\n/talker:\n  use_sim_time (type: boolean)\n'
    assert parse_parameter_names(noisy) == ['use_sim_time']
    assert parse_parameter_names('Node not found\n[exit code 1]') == []
    # the provider strips the output, the first name loses its indentation
    assert parse_parameter_names(PARAM_LIST_TALKER.strip())[0] == \
        'qos_overrides./parameter_events.publisher.depth'


@pytest.mark.parametrize('output, rate', [
    (HZ_CHATTER, 1.0),
    (HZ_ROSOUT, 5.23),
    (HZ_UNPUBLISHED, None),
    ('', None),
])
def test_parse_rate(output, rate):
    assert parse_rate(output) == rate


@pytest.mark.parametrize('output, type_name', [
    (TOPIC_INFO_CHATTER_ROLLING, 'example_interfaces/msg/String'),
    (TOPIC_INFO_CHATTER_HUMBLE, 'std_msgs/msg/String'),
])
def test_parse_topic_info(output, type_name):
    assert parse_topic_info(output) == {
        'type': type_name,
        'publishers': [{'node': '/talker', 'reliability': 'RELIABLE', 'durability': 'VOLATILE'}],
        'subscribers': [
            {'node': '/listener', 'reliability': 'RELIABLE', 'durability': 'VOLATILE'}],
    }


def test_parse_topic_info_namespace_and_unknown_node():
    info = parse_topic_info(TOPIC_INFO_SCAN_MISMATCH)
    assert info['publishers'] == [
        {'node': '/sensors/lidar', 'reliability': 'BEST_EFFORT', 'durability': 'VOLATILE'}]
    assert [s['node'] for s in info['subscribers']] == ['unknown', '/viewer']
    assert parse_topic_info("Unknown topic '/x'") == {
        'type': None, 'publishers': [], 'subscribers': []}


# get_node_status


def test_node_status_regular_node():
    executor, result, status = run_tool(NodeStatusTool(), {
        ('node', 'info', '/talker'): NODE_INFO_TALKER_ROLLING,
        ('topic', 'hz', '/chatter'): HZ_CHATTER,
        ('param', 'list', '/talker'): PARAM_LIST_TALKER,
    }, node='/talker')
    assert result.returncode == 0
    assert executor.calls == [
        (['node', 'info', '/talker'], constants.ROS2AI_TOOL_TIMEOUT_SEC),
        (['param', 'list', '/talker'], constants.ROS2AI_COMPOSITE_STEP_TIMEOUT_SEC),
    ]
    assert executor.samples == [([['topic', 'hz', '/chatter']], constants.ROS2AI_RATE_WINDOW_SEC)]
    assert status == {
        'node': '/talker',
        'found': True,
        'lifecycle': 'not a lifecycle node',
        'publishers': [{'topic': '/chatter', 'type': 'example_interfaces/msg/String', 'hz': 1.0}],
        'subscribers': {},
        'service_servers': {},
        'service_clients': {},
        'action_servers': {},
        'action_clients': {},
        'parameters': [
            'qos_overrides./parameter_events.publisher.depth',
            'qos_overrides./parameter_events.publisher.durability',
            'qos_overrides./parameter_events.publisher.history',
            'qos_overrides./parameter_events.publisher.reliability',
            'use_sim_time',
        ],
        'warnings': [],
    }
    # the key order of the document is stable, the model reads the same layout every time
    assert list(status) == [
        'node', 'found', 'lifecycle', 'publishers', 'subscribers', 'service_servers',
        'service_clients', 'action_servers', 'action_clients', 'parameters', 'warnings']


def test_node_status_humble_node_hides_standard_interfaces():
    _, _, status = run_tool(NodeStatusTool(), {
        ('node', 'info', '/talker'): NODE_INFO_TALKER_HUMBLE,
        ('topic', 'hz', '/chatter'): HZ_CHATTER,
        ('param', 'list', '/talker'): PARAM_LIST_TALKER,
    }, node='/talker')
    assert status['publishers'] == [
        {'topic': '/chatter', 'type': 'std_msgs/msg/String', 'hz': 1.0}]
    assert status['subscribers'] == {}
    assert status['service_servers'] == {}


def test_node_status_lifecycle_node_inactive():
    executor, result, status = run_tool(NodeStatusTool(), {
        ('node', 'info', '/lc_talker'): NODE_INFO_LC_TALKER,
        ('lifecycle', 'get', '/lc_talker'): 'inactive [2]\n',
        ('param', 'list', '/lc_talker'): '  use_sim_time\n',
    }, node='/lc_talker')
    assert result.returncode == 0
    assert executor.calls[1] == (
        ['lifecycle', 'get', '/lc_talker'], constants.ROS2AI_COMPOSITE_STEP_TIMEOUT_SEC)
    # the transition event topic is idle by nature, its rate is not measured
    assert executor.samples == []
    assert status['lifecycle'] == 'inactive'
    assert status['publishers'] == [{
        'topic': '/lc_talker/transition_event', 'type': 'lifecycle_msgs/msg/TransitionEvent',
        'hz': 'not measured'}]
    assert status['service_servers'] == {}
    assert status['warnings'] == ["lifecycle state is 'inactive', not active"]


def test_node_status_lifecycle_node_active_with_own_interfaces():
    info = NODE_INFO_LC_TALKER.replace(
        '  Service Servers:\n', '  Service Servers:\n    /lc_talker/reset: std_srvs/srv/Empty\n')
    info = info.replace(
        '  Publishers:\n', '  Publishers:\n    /lc_chatter: std_msgs/msg/String\n')
    executor, _, status = run_tool(NodeStatusTool(), {
        ('node', 'info', '/lc_talker'): info,
        ('lifecycle', 'get', '/lc_talker'): 'active [3]\n',
        ('topic', 'hz', '/lc_chatter'): HZ_ROSOUT,
        ('param', 'list', '/lc_talker'): '',
    }, node='/lc_talker')
    assert executor.samples == [([['topic', 'hz', '/lc_chatter']], 5)]
    assert status['lifecycle'] == 'active'
    assert status['publishers'] == [
        {'topic': '/lc_chatter', 'type': 'std_msgs/msg/String', 'hz': 5.23},
        {'topic': '/lc_talker/transition_event', 'type': 'lifecycle_msgs/msg/TransitionEvent',
         'hz': 'not measured'},
    ]
    assert status['service_servers'] == {'/lc_talker/reset': 'std_srvs/srv/Empty'}
    assert status['parameters'] == []
    assert status['warnings'] == []


def test_node_status_lifecycle_get_failed():
    _, _, status = run_tool(NodeStatusTool(), {
        ('node', 'info', '/lc_talker'): NODE_INFO_LC_TALKER,
        ('lifecycle', 'get', '/lc_talker'): (
            'Timed out waiting for get_state response\n[exit code 1]', 1),
        ('param', 'list', '/lc_talker'): '',
    }, node='/lc_talker')
    assert status['lifecycle'] == 'unknown'
    assert status['warnings'] == [
        'ros2 lifecycle get /lc_talker failed: Timed out waiting for get_state response']


def test_node_status_all_sections():
    _, _, status = run_tool(NodeStatusTool(), {
        ('node', 'info', '/fibonacci_action_server'): NODE_INFO_FIBONACCI,
        ('param', 'list', '/fibonacci_action_server'): '  use_sim_time\n',
    }, node='/fibonacci_action_server')
    assert status['publishers'] == []
    assert status['subscribers'] == {}
    assert status['service_servers'] == {'/add_two_ints': 'example_interfaces/srv/AddTwoInts'}
    assert status['service_clients'] == {'/other/get_state': 'lifecycle_msgs/srv/GetState'}
    assert status['action_servers'] == {'/fibonacci': 'example_interfaces/action/Fibonacci'}
    assert status['action_clients'] == {}
    assert status['warnings'] == []


def test_node_status_not_found():
    executor, result, status = run_tool(NodeStatusTool(), {
        ('node', 'info', '/talkr'): ("Unable to find node '/talkr'\n[exit code 1]", 1),
        ('node', 'list'): '/lc_talker\n/listener\n/talker\n',
    }, node='/talkr')
    assert result.returncode == 1
    assert [arguments for arguments, _ in executor.calls] == [
        ['node', 'info', '/talkr'], ['node', 'list']]
    assert status == {
        'node': '/talkr',
        'found': False,
        'available_nodes': ['/lc_talker', '/listener', '/talker'],
        'warnings': ["ros2 node info /talkr failed: Unable to find node '/talkr'"],
    }


def test_node_status_name_without_slash():
    executor, _, status = run_tool(NodeStatusTool(), {
        ('node', 'info', '/talker'): NODE_INFO_TALKER_ROLLING,
        ('topic', 'hz', '/chatter'): HZ_CHATTER,
        ('param', 'list', '/talker'): '',
    }, node='talker')
    assert executor.calls[0][0] == ['node', 'info', '/talker']
    assert status['node'] == '/talker'


def test_node_status_idle_publisher_and_window():
    executor, _, status = run_tool(NodeStatusTool(), {
        ('node', 'info', '/talker'): NODE_INFO_TALKER_ROLLING,
        ('topic', 'hz', '/chatter'): '',
        ('param', 'list', '/talker'): '',
    }, node='/talker', window=2)
    assert executor.samples == [([['topic', 'hz', '/chatter']], 2)]
    assert status['publishers'][0]['hz'] is None
    assert status['warnings'] == [
        "no publishing rate measured on '/chatter' within 2 seconds (idle or slow topic)"]


def test_node_status_rate_topics_limit():
    limit = constants.ROS2AI_RATE_TOPICS_LIMIT
    topics = [f'/topic{i}' for i in range(limit + 2)]
    info = NODE_INFO_TALKER_ROLLING.replace(
        '    /chatter: example_interfaces/msg/String\n',
        ''.join(f'    {topic}: std_msgs/msg/String\n' for topic in topics))
    outputs = {
        ('node', 'info', '/talker'): info,
        ('param', 'list', '/talker'): '',
    }
    outputs.update({('topic', 'hz', topic): HZ_CHATTER for topic in topics})
    executor, _, status = run_tool(NodeStatusTool(), outputs, node='/talker')
    assert executor.samples[0][0] == [['topic', 'hz', topic] for topic in topics[:limit]]
    assert [p['hz'] for p in status['publishers']] == [1.0] * limit + ['not measured'] * 2
    assert status['warnings'] == [
        f'publishing rates are measured for the first {limit} topics only']


def test_node_status_parameters_limited_or_failed():
    limit = constants.ROS2AI_LIST_LIMIT
    names = ''.join(f'  param{i:02d}\n' for i in range(limit + 5))
    outputs = {
        ('node', 'info', '/talker'): NODE_INFO_TALKER_ROLLING,
        ('topic', 'hz', '/chatter'): HZ_CHATTER,
        ('param', 'list', '/talker'): names,
    }
    _, _, status = run_tool(NodeStatusTool(), outputs, node='/talker')
    assert len(status['parameters']) == limit + 1
    assert status['parameters'][-1] == '... and 5 more'
    assert status['parameters'][0] == 'param00'

    outputs[('param', 'list', '/talker')] = ('Node not found\n[exit code 1]', 1)
    _, _, status = run_tool(NodeStatusTool(), outputs, node='/talker')
    assert status['parameters'] is None
    assert status['warnings'] == ['ros2 param list /talker failed: Node not found']


# get_topic_status


@pytest.mark.parametrize('info, type_name', [
    (TOPIC_INFO_CHATTER_ROLLING, 'example_interfaces/msg/String'),
    (TOPIC_INFO_CHATTER_HUMBLE, 'std_msgs/msg/String'),
])
def test_topic_status(info, type_name):
    executor, result, status = run_tool(TopicStatusTool(), {
        ('topic', 'info', '-v', '/chatter'): info,
        ('topic', 'hz', '/chatter'): HZ_CHATTER,
        ('topic', 'echo', '--once', '/chatter'): ECHO_CHATTER,
    }, topic='/chatter')
    assert result.returncode == 0
    assert executor.calls == [
        (['topic', 'info', '-v', '/chatter'], constants.ROS2AI_TOOL_TIMEOUT_SEC)]
    assert executor.samples == [(
        [['topic', 'hz', '/chatter'], ['topic', 'echo', '--once', '/chatter']],
        constants.ROS2AI_RATE_WINDOW_SEC)]
    assert status == {
        'topic': '/chatter',
        'found': True,
        'type': type_name,
        'publishers': [{'node': '/talker', 'reliability': 'RELIABLE', 'durability': 'VOLATILE'}],
        'subscribers': [
            {'node': '/listener', 'reliability': 'RELIABLE', 'durability': 'VOLATILE'}],
        'hz': 1.0,
        'sample': "data: 'Hello World: 18'",
        'warnings': [],
    }


def test_topic_status_unknown():
    executor, result, status = run_tool(TopicStatusTool(), {
        ('topic', 'info', '-v', '/chat'): ("Unknown topic '/chat'\n[exit code 1]", 1),
        ('topic', 'list'): '/chatter\n/parameter_events\n/rosout\n',
    }, topic='chat')
    assert result.returncode == 1
    assert executor.samples == []
    assert status == {
        'topic': '/chat',
        'found': False,
        'available_topics': ['/chatter', '/parameter_events', '/rosout'],
        'warnings': ["ros2 topic info -v /chat failed: Unknown topic '/chat'"],
    }


def test_topic_status_no_publishers():
    executor, _, status = run_tool(TopicStatusTool(), {
        ('topic', 'info', '-v', '/chatter'): TOPIC_INFO_NO_PUBLISHER,
    }, topic='/chatter')
    assert executor.samples == []
    assert status['publishers'] == []
    assert status['subscribers'] == [
        {'node': '/listener', 'reliability': 'RELIABLE', 'durability': 'VOLATILE'}]
    assert status['hz'] is None
    assert status['sample'] is None
    assert status['warnings'] == ['no publishers on the topic']


def test_topic_status_idle():
    _, _, status = run_tool(TopicStatusTool(), {
        ('topic', 'info', '-v', '/chatter'): TOPIC_INFO_CHATTER_ROLLING,
        ('topic', 'hz', '/chatter'): HZ_UNPUBLISHED,
        ('topic', 'echo', '--once', '/chatter'): '',
    }, topic='/chatter', window=3)
    assert status['hz'] is None
    assert status['sample'] is None
    assert status['warnings'] == [
        'no publishing rate measured within 3 seconds (idle or slow topic)',
        'no message received within 3 seconds',
    ]


def test_topic_status_qos_mismatch():
    _, _, status = run_tool(TopicStatusTool(), {
        ('topic', 'info', '-v', '/sensors/scan'): TOPIC_INFO_SCAN_MISMATCH,
        ('topic', 'hz', '/sensors/scan'): HZ_ROSOUT,
        ('topic', 'echo', '--once', '/sensors/scan'): 'ranges: [1.0]\n---\n',
    }, topic='/sensors/scan')
    assert status['publishers'] == [
        {'node': '/sensors/lidar', 'reliability': 'BEST_EFFORT', 'durability': 'VOLATILE'}]
    assert status['warnings'] == [
        'incompatible reliability: subscriber unknown requests RELIABLE '
        'but publisher /sensors/lidar offers BEST_EFFORT',
        'incompatible durability: subscriber unknown requests TRANSIENT_LOCAL '
        'but publisher /sensors/lidar offers VOLATILE',
    ]


def test_topic_status_sample_limited():
    limit = constants.ROS2AI_SAMPLE_LINES_LIMIT
    echo = ''.join(f'field{i}: {i}\n' for i in range(limit + 3)) + '---\n'
    _, _, status = run_tool(TopicStatusTool(), {
        ('topic', 'info', '-v', '/chatter'): TOPIC_INFO_CHATTER_ROLLING,
        ('topic', 'hz', '/chatter'): HZ_CHATTER,
        ('topic', 'echo', '--once', '/chatter'): echo,
    }, topic='/chatter')
    lines = status['sample'].splitlines()
    assert len(lines) == limit + 1
    assert lines[0] == 'field0: 0'
    assert lines[-1] == '... [3 lines omitted]'


# provider integration


@pytest.fixture
def provider():
    return Ros2CliProvider()


def test_definitions_include_composites(provider):
    definitions = {d['function']['name']: d['function'] for d in provider.definitions()}
    assert {tool.name for tool in COMPOSITE_TOOLS} <= set(definitions)
    first = len(ROS2_TOOLS)
    assert provider.names[first:first + 2] == ['get_node_status', 'get_topic_status']
    node_status = definitions['get_node_status']
    assert node_status['parameters']['required'] == ['node']
    assert node_status['parameters']['properties']['window'] == {
        'description': node_status['parameters']['properties']['window']['description'],
        'type': 'integer', 'minimum': 1, 'maximum': constants.ROS2AI_RATE_WINDOW_MAX_SEC,
    }
    assert 'health' in node_status['description']
    assert definitions['get_topic_status']['parameters']['required'] == ['topic']


def test_resolve_composite(provider):
    command = provider.resolve('get_node_status', {'node': '/talker'})
    assert isinstance(command, CompositeCommand)
    assert command.argv == []
    assert command.policy == ToolPolicy.READ_ONLY
    assert command.timeout == constants.ROS2AI_TOOL_TIMEOUT_SEC
    assert command.name == 'get_node_status'
    assert command.arguments == {'node': '/talker'}
    assert command.display() == 'get_node_status(node=/talker)'
    command = provider.resolve('get_topic_status', {'topic': '/chatter', 'window': 3})
    assert command.arguments == {'topic': '/chatter', 'window': 3}
    assert command.display() == 'get_topic_status(topic=/chatter, window=3)'
    command = provider.resolve('get_node_status', {'node': 'a b', 'window': None})
    assert command.arguments == {'node': 'a b'}
    assert command.display() == "get_node_status(node='a b')"


@pytest.mark.parametrize('name, arguments, message', [
    ('get_node_status', {}, "missing required parameter 'node'"),
    ('get_node_status', {'node': ''}, "parameter 'node' must be a non-empty string"),
    ('get_node_status', {'node': '/t', 'window': 0}, "parameter 'window' must be at least 1"),
    ('get_node_status', {'node': '/t', 'window': 11}, "parameter 'window' must be at most 10"),
    ('get_node_status', {'node': '/t', 'window': '5'}, "parameter 'window' must be an integer"),
    ('get_node_status', {'node': '/t', 'window': True}, "parameter 'window' must be an integer"),
    ('get_topic_status', {'topic': '/c', 'node': '/t'},
     "unknown parameter(s) ['node'] for tool 'get_topic_status'"),
    ('get_topic_status', [], 'arguments must be a JSON object'),
])
def test_resolve_composite_errors(provider, name, arguments, message):
    with pytest.raises(ToolError) as excinfo:
        provider.resolve(name, arguments)
    assert str(excinfo.value) == message


def test_provider_without_composites_and_duplicates():
    names = Ros2CliProvider(composites=()).names
    assert names == [tool.name for tool in ROS2_TOOLS]
    assert 'get_node_status' not in names

    class Duplicate(NodeStatusTool):
        name = 'ros2_node_info'

    with pytest.raises(ValueError, match="duplicated tool name\\(s\\) \\['ros2_node_info'\\]"):
        Ros2CliProvider(composites=(Duplicate(),))


class FakeRun:
    """Stand-in for subprocess.run returning canned outputs keyed by argv."""

    def __init__(self, outputs):
        self.outputs = outputs
        self.calls = []

    def __call__(self, argv, **kwargs):
        self.calls.append(argv)
        assert 'shell' not in kwargs
        stdout, returncode = self.outputs[tuple(argv)]
        return subprocess.CompletedProcess(argv, returncode, stdout, '')


def test_run_composite_prints_sub_commands(provider, monkeypatch, capsys):
    fake_run = FakeRun({
        ('ros2', 'node', 'info', '/talker'): (NODE_INFO_TALKER_ROLLING, 0),
        ('ros2', 'param', 'list', '/talker'): (PARAM_LIST_TALKER, 0),
    })
    monkeypatch.setattr(subprocess, 'run', fake_run)
    sampled = []

    def fake_sample(commands, seconds):
        sampled.append(([command.argv for command in commands], seconds))
        assert all(command.policy == ToolPolicy.READ_ONLY for command in commands)
        return [ToolResult(HZ_CHATTER, 0) for _ in commands]
    monkeypatch.setattr(provider, 'sample', fake_sample)

    command = provider.resolve('get_node_status', {'node': '/talker', 'window': 2})
    result = provider.run(command)
    assert result.returncode == 0
    assert fake_run.calls == [
        ['ros2', 'node', 'info', '/talker'], ['ros2', 'param', 'list', '/talker']]
    assert sampled == [([['ros2', 'topic', 'hz', '/chatter']], 2)]
    assert capsys.readouterr().out == (
        '[ros2ai]:   ros2 node info /talker\n'
        '[ros2ai]:   ros2 topic hz /chatter\n'
        '[ros2ai]:   ros2 param list /talker\n')
    status = yaml.safe_load(result.output)
    assert status['found'] is True
    assert status['publishers'][0]['hz'] == 1.0
    assert status['parameters'][-1] == 'use_sim_time'


class _Composite(CompositeTool):
    """Composite tool running the given commands, to check the policy enforcement."""

    name = 'composite'
    description = 'test'

    def __init__(self, policy, commands, sampled=()):
        self.policy = policy
        self.commands = commands
        self.sampled = sampled

    def run(self, executor, arguments):
        outputs = [executor.run(command).output for command in self.commands]
        if self.sampled:
            outputs.extend(result.output for result in executor.sample(self.sampled, 0))
        return ToolResult('\n'.join(outputs), 0)


@pytest.mark.parametrize('policy, commands, message', [
    (ToolPolicy.READ_ONLY, [['topic', 'pub', '/c', 'std_msgs/msg/String']],
     "'ros2 topic pub /c std_msgs/msg/String' is mutating, not allowed in a read-only tool"),
    (ToolPolicy.READ_ONLY, [['node', 'list'], ['daemon', 'stop']],
     "'ros2 daemon stop' is mutating, not allowed in a read-only tool"),
    (ToolPolicy.MUTATING, [['run', 'demo_nodes_cpp', 'talker']],
     "'ros2 run demo_nodes_cpp talker' is long-running, not allowed in a mutating tool"),
    (ToolPolicy.LONG_RUNNING, [['launch', 'p', 'l.py']],
     "'ros2 launch p l.py' is long-running, not allowed in a long-running tool"),
])
def test_composite_policy_enforced(monkeypatch, capsys, policy, commands, message):
    fake_run = FakeRun({('ros2', 'node', 'list'): ('/talker', 0)})
    monkeypatch.setattr(subprocess, 'run', fake_run)
    provider = Ros2CliProvider(composites=(_Composite(policy, commands),))
    with pytest.raises(ToolError) as excinfo:
        provider.run(provider.resolve('composite', {}))
    assert str(excinfo.value) == message
    assert fake_run.calls == [command for command in [['ros2', 'node', 'list']]
                              if ['node', 'list'] in commands]


def test_composite_policy_allows_within(monkeypatch):
    fake_run = FakeRun({
        ('ros2', 'node', 'list'): ('/talker', 0),
        ('ros2', 'param', 'set', '/talker', 'p', '1'): ('Set parameter successful', 0),
    })
    monkeypatch.setattr(subprocess, 'run', fake_run)
    composite = _Composite(
        ToolPolicy.MUTATING, [['node', 'list'], ['param', 'set', '/talker', 'p', '1']])
    provider = Ros2CliProvider(composites=(composite,))
    command = provider.resolve('composite', {})
    assert command.policy == ToolPolicy.MUTATING
    assert provider.run(command).output == '/talker\nSet parameter successful'


def test_composite_sample_policy_enforced(monkeypatch):
    provider = Ros2CliProvider(composites=(_Composite(
        ToolPolicy.READ_ONLY, [], sampled=[['topic', 'hz', '/c'], ['bag', 'record', '-a']]),))
    monkeypatch.setattr(provider, 'sample', lambda commands, seconds: pytest.fail('not reached'))
    with pytest.raises(ToolError) as excinfo:
        provider.run(provider.resolve('composite', {}))
    assert str(excinfo.value) == \
        "'ros2 bag record -a' is long-running, not allowed in a read-only tool"
