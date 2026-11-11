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
from ros2ai.verb.exec import ExecVerb
from ros2ai.verb.query import QueryVerb
from ros2ai.verb.status import StatusVerb


def parse(verb, argv):
    parser = argparse.ArgumentParser(prog='ros2 ai')
    verb.add_arguments(parser, 'ros2 ai')
    return parser.parse_args(argv)


GLOBAL_DEFAULTS = {'model': None, 'url': None, 'token': None}


def assert_global_defaults(args):
    for key, value in GLOBAL_DEFAULTS.items():
        assert getattr(args, key) == value


@pytest.mark.parametrize('verb', [StatusVerb(), QueryVerb(), ExecVerb()])
def test_global_arguments(verb):
    positional = [] if isinstance(verb, StatusVerb) else ['hello']
    args = parse(verb, positional + ['-m', 'gpt-x', '-u', 'http://127.0.0.1:11434/v1', '-t', '12'])
    assert args.model == 'gpt-x'
    assert args.url == 'http://127.0.0.1:11434/v1'
    assert args.token == 12
    args = parse(verb, positional + ['--model', 'gpt-y', '--url', 'http://h/v1', '--token', '3'])
    assert args.model == 'gpt-y'
    assert args.url == 'http://h/v1'
    assert args.token == 3


def test_status_arguments():
    args = parse(StatusVerb(), [])
    assert_global_defaults(args)
    assert args.verbose is False
    assert args.list is False
    args = parse(StatusVerb(), ['-vl'])
    assert args.verbose is True
    assert args.list is True
    args = parse(StatusVerb(), ['--verbose', '--list'])
    assert args.verbose is True
    assert args.list is True


def test_query_arguments():
    args = parse(QueryVerb(), ['what', 'is', 'ros'])
    assert_global_defaults(args)
    assert args.questions == ['what', 'is', 'ros']
    assert args.nostream is False
    assert args.verbose is False
    assert args.role is None
    args = parse(QueryVerb(), ['say hello', '-nv', '-r', 'custom role'])
    assert args.questions == ['say hello']
    assert args.nostream is True
    assert args.verbose is True
    assert args.role == 'custom role'
    args = parse(QueryVerb(), ['q', '--nostream', '--verbose', '--role', 'r'])
    assert args.nostream is True
    assert args.verbose is True
    assert args.role == 'r'
    with pytest.raises(SystemExit):
        parse(QueryVerb(), [])


def test_exec_arguments():
    args = parse(ExecVerb(), ['give', 'me', 'all', 'topics'])
    assert_global_defaults(args)
    assert args.request == ['give', 'me', 'all', 'topics']
    assert args.debug is False
    assert args.dry_run is False
    assert args.yes is False
    assert args.role is None
    args = parse(ExecVerb(), ['list nodes', '-d', '--dry-run', '-y', '-r', 'custom role'])
    assert args.request == ['list nodes']
    assert args.debug is True
    assert args.dry_run is True
    assert args.yes is True
    assert args.role == 'custom role'
    args = parse(ExecVerb(), ['list nodes', '--debug', '--yes', '--role', 'r'])
    assert args.debug is True
    assert args.yes is True
    assert args.role == 'r'
    with pytest.raises(SystemExit):
        parse(ExecVerb(), [])


def test_ros2_ai_help(capsys, monkeypatch):
    from ros2cli.cli import main
    # verb sub-parsers are added on demand based on sys.argv
    monkeypatch.setattr(sys, 'argv', ['ros2', 'ai', '-h'])
    with pytest.raises(SystemExit) as exit_info:
        main(argv=['ai', '-h'])
    assert exit_info.value.code == 0
    out = capsys.readouterr().out
    for verb in ('status', 'query', 'exec'):
        assert verb in out
    for option in ('-m', '--model', '-u', '--url', '-t', '--token'):
        assert option in out
