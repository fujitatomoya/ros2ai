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

"""Tests of the ROS 2 CLI tool provider, no ROS 2 graph required."""

import os
import shlex
import signal
import subprocess

import pytest
from ros2ai.api.composite import COMPOSITE_TOOLS
import ros2ai.api.constants as constants
from ros2ai.api.docs import DOCS_TOOLS
from ros2ai.api.tools import GENERIC_TOOL_NAME
from ros2ai.api.tools import ROS2_TOOLS
from ros2ai.api.tools import Ros2CliProvider
from ros2ai.api.tools import ToolCommand
from ros2ai.api.tools import ToolError
from ros2ai.api.tools import ToolPolicy


@pytest.fixture
def provider():
    return Ros2CliProvider()


def test_definitions_format(provider):
    definitions = provider.definitions()
    assert len(definitions) == len(ROS2_TOOLS) + len(COMPOSITE_TOOLS) + len(DOCS_TOOLS)
    names = [definition['function']['name'] for definition in definitions]
    assert names == provider.names
    assert len(set(names)) == len(names)
    assert all(name.startswith('ros2_') for name in names[:len(ROS2_TOOLS)])
    for definition in definitions:
        assert definition['type'] == 'function'
        function = definition['function']
        assert function['description']
        parameters = function['parameters']
        assert parameters['type'] == 'object'
        assert parameters['additionalProperties'] is False
        for name in parameters['required']:
            assert name in parameters['properties']
        for prop in parameters['properties'].values():
            assert prop['type'] in ('string', 'integer', 'array')
            assert prop['description']


def test_definitions_args_array(provider):
    definitions = {d['function']['name']: d['function'] for d in provider.definitions()}
    run = definitions['ros2_run']
    assert run['parameters']['required'] == ['package', 'executable']
    assert run['parameters']['properties']['args'] == {
        'description': 'Arguments passed to the executable',
        'type': 'array', 'items': {'type': 'string'},
    }


@pytest.mark.parametrize('name, arguments, argv, policy', [
    ('ros2_node_list', {}, ['ros2', 'node', 'list'], ToolPolicy.READ_ONLY),
    ('ros2_node_info', {'node': '/talker'},
     ['ros2', 'node', 'info', '/talker'], ToolPolicy.READ_ONLY),
    ('ros2_topic_list', {}, ['ros2', 'topic', 'list', '-t'], ToolPolicy.READ_ONLY),
    ('ros2_topic_info', {'topic': '/chatter'},
     ['ros2', 'topic', 'info', '-v', '/chatter'], ToolPolicy.READ_ONLY),
    ('ros2_topic_echo', {'topic': '/chatter'},
     ['ros2', 'topic', 'echo', '--once', '/chatter'], ToolPolicy.READ_ONLY),
    ('ros2_service_list', {}, ['ros2', 'service', 'list', '-t'], ToolPolicy.READ_ONLY),
    ('ros2_action_list', {}, ['ros2', 'action', 'list', '-t'], ToolPolicy.READ_ONLY),
    ('ros2_param_list', {}, ['ros2', 'param', 'list'], ToolPolicy.READ_ONLY),
    ('ros2_param_list', {'node': '/talker'},
     ['ros2', 'param', 'list', '/talker'], ToolPolicy.READ_ONLY),
    ('ros2_param_get', {'node': '/talker', 'name': 'use_sim_time'},
     ['ros2', 'param', 'get', '/talker', 'use_sim_time'], ToolPolicy.READ_ONLY),
    ('ros2_interface_show', {'type': 'std_msgs/msg/String'},
     ['ros2', 'interface', 'show', 'std_msgs/msg/String'], ToolPolicy.READ_ONLY),
    ('ros2_pkg_executables', {'package': 'demo_nodes_cpp'},
     ['ros2', 'pkg', 'executables', 'demo_nodes_cpp'], ToolPolicy.READ_ONLY),
    ('ros2_doctor', {}, ['ros2', 'doctor'], ToolPolicy.READ_ONLY),
    ('ros2_topic_pub', {'topic': '/chatter', 'type': 'std_msgs/msg/String',
                        'values': '{data: hello}'},
     ['ros2', 'topic', 'pub', '--once', '-w', '0', '/chatter', 'std_msgs/msg/String',
      '{data: hello}'],
     ToolPolicy.MUTATING),
    ('ros2_topic_pub', {'topic': '/chatter', 'type': 'std_msgs/msg/String'},
     ['ros2', 'topic', 'pub', '--once', '-w', '0', '/chatter', 'std_msgs/msg/String'],
     ToolPolicy.MUTATING),
    ('ros2_service_call', {'service': '/add', 'type': 'x/srv/Add', 'values': '{a: 1}'},
     ['ros2', 'service', 'call', '/add', 'x/srv/Add', '{a: 1}'], ToolPolicy.MUTATING),
    ('ros2_param_set', {'node': '/talker', 'name': 'p', 'value': '1.5'},
     ['ros2', 'param', 'set', '/talker', 'p', '1.5'], ToolPolicy.MUTATING),
    ('ros2_action_send_goal', {'action': '/fib', 'type': 'x/action/Fib', 'values': '{order: 5}'},
     ['ros2', 'action', 'send_goal', '/fib', 'x/action/Fib', '{order: 5}'],
     ToolPolicy.MUTATING),
    ('ros2_run', {'package': 'demo_nodes_cpp', 'executable': 'talker'},
     ['ros2', 'run', 'demo_nodes_cpp', 'talker'], ToolPolicy.LONG_RUNNING),
    ('ros2_run', {'package': 'demo_nodes_cpp', 'executable': 'talker',
                  'args': ['--ros-args', '-r', '__node:=t']},
     ['ros2', 'run', 'demo_nodes_cpp', 'talker', '--ros-args', '-r', '__node:=t'],
     ToolPolicy.LONG_RUNNING),
    ('ros2_launch', {'package': 'demo_nodes_cpp', 'launch_file': 'talker_listener_launch.py',
                     'args': ['use_sim_time:=true']},
     ['ros2', 'launch', 'demo_nodes_cpp', 'talker_listener_launch.py', 'use_sim_time:=true'],
     ToolPolicy.LONG_RUNNING),
])
def test_resolve(provider, name, arguments, argv, policy):
    command = provider.resolve(name, arguments)
    assert command.argv == argv
    assert command.policy == policy
    assert command.display() == shlex.join(argv)


def test_resolve_timeouts(provider):
    assert provider.resolve('ros2_node_list', {}).timeout == constants.ROS2AI_TOOL_TIMEOUT_SEC
    echo = provider.resolve('ros2_topic_echo', {'topic': '/chatter'})
    assert echo.timeout == constants.ROS2AI_TOOL_ECHO_TIMEOUT_SEC


def test_resolve_shell_characters_are_literal(provider):
    command = provider.resolve('ros2_topic_echo', {'topic': '$(cat ~/.ssh/id_rsa) > /tmp/x'})
    assert command.argv == ['ros2', 'topic', 'echo', '--once', '$(cat ~/.ssh/id_rsa) > /tmp/x']
    assert command.display() == "ros2 topic echo --once '$(cat ~/.ssh/id_rsa) > /tmp/x'"


@pytest.mark.parametrize('name, arguments, message', [
    ('ros2_unknown', {}, "unknown tool 'ros2_unknown'"),
    ('ros2_node_list', [], 'arguments must be a JSON object'),
    ('ros2_node_list', {'node': '/x'}, "unknown parameter(s) ['node'] for tool 'ros2_node_list'"),
    ('ros2_node_info', {}, "missing required parameter 'node'"),
    ('ros2_node_info', {'node': None}, "missing required parameter 'node'"),
    ('ros2_node_info', {'node': ''}, "parameter 'node' must be a non-empty string"),
    ('ros2_node_info', {'node': 1}, "parameter 'node' must be a non-empty string"),
    ('ros2_run', {'package': 'p', 'executable': 'e', 'args': 'x'},
     "parameter 'args' must be a list of strings"),
    ('ros2_run', {'package': 'p', 'executable': 'e', 'args': [1]},
     "parameter 'args' must be a list of strings"),
    (GENERIC_TOOL_NAME, {'arguments': []}, 'arguments must not be empty'),
    (GENERIC_TOOL_NAME, {}, "missing required parameter 'arguments'"),
])
def test_resolve_errors(provider, name, arguments, message):
    with pytest.raises(ToolError) as excinfo:
        provider.resolve(name, arguments)
    assert str(excinfo.value) == message


@pytest.mark.parametrize('arguments, policy', [
    (['node', 'list'], ToolPolicy.READ_ONLY),
    (['topic', 'hz', '/chatter'], ToolPolicy.READ_ONLY),
    (['topic', '--include-hidden-topics', 'list'], ToolPolicy.READ_ONLY),
    (['param', 'dump', '/talker'], ToolPolicy.READ_ONLY),
    (['interface', 'list'], ToolPolicy.READ_ONLY),
    (['lifecycle', 'get', '/node'], ToolPolicy.READ_ONLY),
    (['doctor', '--report'], ToolPolicy.READ_ONLY),
    (['wtf'], ToolPolicy.READ_ONLY),
    (['bag', 'info', 'rosbag2'], ToolPolicy.READ_ONLY),
    (['topic', 'pub', '-h'], ToolPolicy.READ_ONLY),
    (['lifecycle', '--help'], ToolPolicy.READ_ONLY),
    (['topic', 'pub', '/chatter', 'std_msgs/msg/String'], ToolPolicy.MUTATING),
    (['lifecycle', 'set', '/node', 'configure'], ToolPolicy.MUTATING),
    (['param', 'load', '/node', 'p.yaml'], ToolPolicy.MUTATING),
    (['daemon', 'stop'], ToolPolicy.MUTATING),
    (['security', 'create_keystore', 'k'], ToolPolicy.MUTATING),
    (['unknown'], ToolPolicy.MUTATING),
    (['--version'], ToolPolicy.MUTATING),
    (['run', 'demo_nodes_cpp', 'talker'], ToolPolicy.LONG_RUNNING),
    (['launch', 'demo_nodes_cpp', 'talker_listener_launch.py'], ToolPolicy.LONG_RUNNING),
    (['bag', 'record', '-a'], ToolPolicy.LONG_RUNNING),
    (['bag', 'play', 'rosbag2'], ToolPolicy.LONG_RUNNING),
])
def test_generic_tool_policy(provider, arguments, policy):
    command = provider.resolve(GENERIC_TOOL_NAME, {'arguments': arguments})
    assert command.argv == ['ros2', *arguments]
    assert command.policy == policy
    assert Ros2CliProvider.classify(arguments) == policy


class FakeRun:
    """Stand-in for subprocess.run recording the arguments."""

    def __init__(self, stdout='', stderr='', returncode=0, error=None):
        self.stdout = stdout
        self.stderr = stderr
        self.returncode = returncode
        self.error = error
        self.calls = []

    def __call__(self, argv, **kwargs):
        self.calls.append((argv, kwargs))
        if self.error is not None:
            raise self.error
        return subprocess.CompletedProcess(argv, self.returncode, self.stdout, self.stderr)


def test_run_captures_output(provider, monkeypatch):
    fake = FakeRun(stdout='/talker\n/listener\n')
    monkeypatch.setattr(subprocess, 'run', fake)
    result = provider.run(ToolCommand(['ros2', 'node', 'list'], ToolPolicy.READ_ONLY, 7.0))
    assert result.output == '/talker\n/listener'
    assert result.returncode == 0
    argv, kwargs = fake.calls[0]
    assert argv == ['ros2', 'node', 'list']
    assert kwargs == {'capture_output': True, 'text': True, 'timeout': 7.0}
    assert 'shell' not in kwargs


def test_run_failure_includes_stderr_and_exit_code(provider, monkeypatch):
    fake = FakeRun(stdout='', stderr='Unknown node: /x\n', returncode=1)
    monkeypatch.setattr(subprocess, 'run', fake)
    result = provider.run(ToolCommand(['ros2', 'node', 'info', '/x'], ToolPolicy.READ_ONLY))
    assert result.output == 'Unknown node: /x\n[exit code 1]'
    assert result.returncode == 1


def test_run_timeout_returns_partial_output(provider, monkeypatch):
    error = subprocess.TimeoutExpired(['ros2', 'topic', 'hz'], 10.0, output=b'partial\n')
    monkeypatch.setattr(subprocess, 'run', FakeRun(error=error))
    result = provider.run(ToolCommand(['ros2', 'topic', 'hz', '/c'], ToolPolicy.READ_ONLY, 10.0))
    assert result.output == 'partial\n[timeout after 10.0 seconds, output above is partial]'
    assert result.returncode == -1


def test_run_os_error(provider, monkeypatch):
    monkeypatch.setattr(subprocess, 'run', FakeRun(error=FileNotFoundError('no ros2')))
    result = provider.run(ToolCommand(['ros2', 'node', 'list'], ToolPolicy.READ_ONLY))
    assert result.output == 'failed to execute: no ros2'
    assert result.returncode == -1


class FakePopen:
    """Stand-in for subprocess.Popen, running until interrupted unless `exited` is set."""

    instances = []

    def __init__(self, argv, stdout='', exited=None, hang=False, error=None):
        self.argv = argv
        self.stdout = stdout
        self.returncode = exited
        self.hang = hang
        self.error = error
        self.kwargs = None
        self.signals = []
        self.killed = False
        self.communicate_timeouts = []

    def start(self, argv, **kwargs):
        if self.error is not None:
            raise self.error
        assert argv == self.argv
        self.kwargs = kwargs
        FakePopen.instances.append(self)
        return self

    def poll(self):
        return self.returncode

    def send_signal(self, sig):
        self.signals.append(sig)
        self.returncode = -sig

    def kill(self):
        self.killed = True
        self.hang = False
        self.returncode = -signal.SIGKILL

    def communicate(self, timeout=None):
        self.communicate_timeouts.append(timeout)
        if self.hang:
            raise subprocess.TimeoutExpired(self.argv, timeout)
        return self.stdout, None


@pytest.fixture
def fake_popen(monkeypatch):
    """Patch subprocess.Popen to start the scripted FakePopen instances in order."""
    FakePopen.instances = []
    scripted = []

    def popen(argv, **kwargs):
        return scripted.pop(0).start(argv, **kwargs)
    monkeypatch.setattr(subprocess, 'Popen', popen)
    return scripted


def test_sample_interrupts_and_captures(provider, fake_popen):
    hz = ToolCommand(['ros2', 'topic', 'hz', '/chatter'], ToolPolicy.READ_ONLY)
    echo = ToolCommand(['ros2', 'topic', 'echo', '--once', '/chatter'], ToolPolicy.READ_ONLY)
    fake_popen.extend([
        FakePopen(hz.argv, stdout='average rate: 1.000\n\tmin: 1.000s\n'),
        FakePopen(echo.argv, stdout="data: 'hello'\n---\n", exited=0),
    ])
    results = provider.sample([hz, echo], 0.2)
    running, done = FakePopen.instances
    assert running.kwargs['stdout'] is subprocess.PIPE
    assert running.kwargs['stderr'] is subprocess.STDOUT
    assert running.kwargs['text'] is True
    assert running.kwargs['env']['PYTHONUNBUFFERED'] == '1'
    assert running.kwargs['env']['PATH'] == os.environ['PATH']
    assert 'shell' not in running.kwargs
    # the running command is interrupted, the finished one is left alone
    assert running.signals == [signal.SIGINT]
    assert done.signals == []
    assert running.communicate_timeouts == [constants.ROS2AI_SAMPLE_GRACE_SEC]
    assert results[0].output == 'average rate: 1.000\n\tmin: 1.000s'
    assert results[0].returncode == -signal.SIGINT
    assert results[1].output == "data: 'hello'\n---"
    assert results[1].returncode == 0


def test_sample_kills_after_grace_period(provider, fake_popen):
    hz = ToolCommand(['ros2', 'topic', 'hz', '/chatter'], ToolPolicy.READ_ONLY)
    fake_popen.append(FakePopen(hz.argv, stdout='partial', hang=True))
    results = provider.sample([hz], 0.1)
    process = FakePopen.instances[0]
    assert process.signals == [signal.SIGINT]
    assert process.killed is True
    assert process.communicate_timeouts == [constants.ROS2AI_SAMPLE_GRACE_SEC, None]
    assert results[0].output == 'partial'
    assert results[0].returncode == -signal.SIGKILL


def test_sample_os_error(provider, fake_popen):
    missing = ToolCommand(['ros2', 'topic', 'hz', '/a'], ToolPolicy.READ_ONLY)
    other = ToolCommand(['ros2', 'topic', 'hz', '/b'], ToolPolicy.READ_ONLY)
    fake_popen.extend([
        FakePopen(missing.argv, error=FileNotFoundError('no ros2')),
        FakePopen(other.argv, stdout='average rate: 2.000\n', exited=0),
    ])
    results = provider.sample([missing, other], 0.1)
    assert results[0].output == 'failed to execute: no ros2'
    assert results[0].returncode == -1
    assert results[1].output == 'average rate: 2.000'
    assert len(FakePopen.instances) == 1
