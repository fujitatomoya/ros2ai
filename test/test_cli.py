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

import argparse
import sys

import pytest
from ros2ai.command.ai import AiCommand


def parse(argv):
    parser = argparse.ArgumentParser(prog='ros2 ai')
    AiCommand().add_arguments(parser, 'ros2 ai')
    return parser.parse_args(argv)


def test_defaults():
    args = parse([])
    assert args.request == []
    assert args.model is None
    assert args.url is None
    assert args.token is None
    assert args.role is None
    assert args.verbose is False
    assert args.yes is False
    assert args.dry_run is False
    assert args.status is False
    assert args.list_models is False


def test_request():
    args = parse(['give', 'me', 'all', 'topics'])
    assert args.request == ['give', 'me', 'all', 'topics']
    args = parse(['give me all topics'])
    assert args.request == ['give me all topics']
    # options can be placed before or after the request
    args = parse(['-y', 'publish hello', '--dry-run'])
    assert args.request == ['publish hello']
    assert args.yes is True
    assert args.dry_run is True


def test_global_arguments():
    args = parse(['hello', '-m', 'gpt-x', '-u', 'http://127.0.0.1:11434/v1', '-t', '12'])
    assert args.model == 'gpt-x'
    assert args.url == 'http://127.0.0.1:11434/v1'
    assert args.token == 12
    args = parse(['hello', '--model', 'gpt-y', '--url', 'http://h/v1', '--token', '3'])
    assert args.model == 'gpt-y'
    assert args.url == 'http://h/v1'
    assert args.token == 3


def test_request_arguments():
    args = parse(['list nodes', '-v', '--dry-run', '-y', '-r', 'custom role'])
    assert args.request == ['list nodes']
    assert args.verbose is True
    assert args.dry_run is True
    assert args.yes is True
    assert args.role == 'custom role'
    args = parse(['list nodes', '--verbose', '--yes', '--role', 'r'])
    assert args.verbose is True
    assert args.yes is True
    assert args.role == 'r'


def test_status_arguments():
    args = parse(['-s'])
    assert args.request == []
    assert args.status is True
    assert args.list_models is False
    args = parse(['-vl'])
    assert args.verbose is True
    assert args.list_models is True
    args = parse(['--status', '--list-models'])
    assert args.status is True
    assert args.list_models is True


def test_unknown_option():
    with pytest.raises(SystemExit):
        parse(['hello', '--nostream'])


def test_ros2_ai_help(capsys, monkeypatch):
    from ros2cli.cli import main
    monkeypatch.setattr(sys, 'argv', ['ros2', 'ai', '-h'])
    with pytest.raises(SystemExit) as exit_info:
        main(argv=['ai', '-h'])
    assert exit_info.value.code == 0
    out = capsys.readouterr().out
    assert '<request>' in out
    for option in (
            '-m', '--model', '-u', '--url', '-t', '--token', '-r', '--role', '-v', '--verbose',
            '-y', '--yes', '--dry-run', '-s', '--status', '-l', '--list-models'):
        assert option in out
    assert 'examples:' in out
    # no sub-commands
    assert 'Commands:' not in out
    assert 'query' not in out


def test_ros2_ai_without_request_prints_help(capsys, monkeypatch):
    from ros2cli.cli import main
    monkeypatch.setattr(sys, 'argv', ['ros2', 'ai'])
    assert main(argv=['ai']) == 0
    out = capsys.readouterr().out
    assert out.startswith('usage: ')
    assert '<request>' in out
    assert 'examples:' in out
