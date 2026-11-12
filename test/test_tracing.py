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

"""Tests of the trace printed to the user."""

import pytest
import ros2ai.api.tracing as tracing


@pytest.fixture(autouse=True)
def disabled():
    tracing.enable(False)
    yield
    tracing.enable(False)


def test_disabled_by_default(capsys):
    assert tracing.enabled() is False
    tracing.trace('ros2 topic list')
    tracing.trace('ros2 node info /talker', nested=True)
    tracing.output('/chatter')
    assert capsys.readouterr().out == ''


def test_enabled(capsys):
    tracing.enable()
    assert tracing.enabled() is True
    tracing.trace('get_node_status(node=/talker)')
    tracing.trace('ros2 node info /talker', nested=True)
    tracing.output('/talker')
    assert capsys.readouterr().out == (
        '[ros2ai]: get_node_status(node=/talker)\n'
        '[ros2ai]:   ros2 node info /talker\n'
        '/talker\n')
    tracing.enable(False)
    assert tracing.enabled() is False


def test_notice_regardless(capsys):
    tracing.notice('ros2 param set /n p 1')
    # not indented without the trace, the tool call above is not printed
    tracing.notice('downloaded the documentation', nested=True)
    assert capsys.readouterr().out == (
        '[ros2ai]: ros2 param set /n p 1\n'
        '[ros2ai]: downloaded the documentation\n')
    tracing.enable()
    tracing.notice('ros2 param set /n p 1')
    tracing.notice('downloaded the documentation', nested=True)
    assert capsys.readouterr().out == (
        '[ros2ai]: ros2 param set /n p 1\n'
        '[ros2ai]:   downloaded the documentation\n')
