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

"""Tests of the streamed completion assembly, no network access required."""

from types import SimpleNamespace

from ros2ai.api.client import assemble_stream


def chunk(content=None, tool_calls=None, finish_reason=None):
    delta = SimpleNamespace(content=content, tool_calls=tool_calls)
    return SimpleNamespace(choices=[SimpleNamespace(delta=delta, finish_reason=finish_reason)])


def delta_call(index=None, call_id=None, name=None, arguments=None):
    function = SimpleNamespace(name=name, arguments=arguments)
    return SimpleNamespace(index=index, id=call_id, type='function', function=function)


def test_content_only():
    received = []
    message, finish_reason = assemble_stream(
        [chunk('Hello'), chunk(None), chunk(', world'), chunk(finish_reason='stop')],
        on_content=received.append)
    assert message.content == 'Hello, world'
    assert message.tool_calls == []
    assert finish_reason == 'stop'
    assert received == ['Hello', ', world']


def test_empty_stream():
    message, finish_reason = assemble_stream([])
    assert message.content is None
    assert message.tool_calls == []
    assert finish_reason is None


def test_tool_calls_accumulated_by_index():
    chunks = [
        chunk('Let me check.'),
        chunk(tool_calls=[delta_call(0, 'call_a', 'ros2_node_info', '{"node"')]),
        chunk(tool_calls=[delta_call(1, 'call_b', 'ros2_topic_list', '')]),
        chunk(tool_calls=[delta_call(0, None, None, ': "/talker"}')]),
        chunk(tool_calls=[delta_call(1, None, None, '{}')]),
        chunk(finish_reason='tool_calls'),
        # usage chunk at the end of the stream
        SimpleNamespace(choices=[]),
    ]
    message, finish_reason = assemble_stream(chunks)
    assert message.content == 'Let me check.'
    assert finish_reason == 'tool_calls'
    assert [(c.id, c.type, c.function.name, c.function.arguments) for c in message.tool_calls] == [
        ('call_a', 'function', 'ros2_node_info', '{"node": "/talker"}'),
        ('call_b', 'function', 'ros2_topic_list', '{}'),
    ]


def test_tool_calls_reusing_index_with_different_ids():
    chunks = [
        chunk(tool_calls=[delta_call(0, 'call_a', 'ros2_node_list', '{}')]),
        chunk(tool_calls=[delta_call(0, 'call_b', 'ros2_topic_list', '{}')]),
    ]
    message, _ = assemble_stream(chunks)
    assert [(c.id, c.function.name) for c in message.tool_calls] == [
        ('call_a', 'ros2_node_list'), ('call_b', 'ros2_topic_list')]


def test_tool_calls_reusing_index_with_sparse_indices():
    chunks = [
        chunk(tool_calls=[delta_call(0, 'call_a', 'ros2_node_list', '{}')]),
        chunk(tool_calls=[delta_call(2, 'call_b', 'ros2_topic_list', '{}')]),
        chunk(tool_calls=[delta_call(0, 'call_c', 'ros2_doctor', '{}')]),
        chunk(tool_calls=[delta_call(2, None, None, '')]),
    ]
    message, _ = assemble_stream(chunks)
    assert [c.id for c in message.tool_calls] == ['call_a', 'call_b', 'call_c']


def test_tool_calls_without_index_or_id():
    chunks = [
        chunk(tool_calls=[delta_call(None, None, 'ros2_node_list', '{}')]),
        chunk(tool_calls=[delta_call(None, None, 'ros2_topic_list', '{}')]),
    ]
    message, _ = assemble_stream(chunks)
    # the ids are generated so that the tool results can be returned to the model
    assert [(c.id, c.function.name) for c in message.tool_calls] == [
        ('call_0', 'ros2_node_list'), ('call_1', 'ros2_topic_list')]


def test_tool_calls_in_single_chunk():
    chunks = [chunk(tool_calls=[
        delta_call(0, 'call_a', 'ros2_node_list', '{}'),
        delta_call(1, 'call_b', 'ros2_topic_list', '{}'),
    ])]
    message, _ = assemble_stream(chunks)
    assert [c.id for c in message.tool_calls] == ['call_a', 'call_b']
