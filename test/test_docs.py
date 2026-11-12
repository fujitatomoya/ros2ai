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

"""Tests of the documentation tools with a fake archive download, no network access required."""

import io
import json
import os
import tarfile
import time
from urllib.error import URLError

import pytest
from ros2ai.api import docs
from ros2ai.api.composite import COMPOSITE_TOOLS
import ros2ai.api.constants as constants
from ros2ai.api.docs import DOCS_TOOLS
from ros2ai.api.docs import DocsCache
from ros2ai.api.docs import DocsError
from ros2ai.api.docs import expand_placeholders
from ros2ai.api.docs import normalize_page
from ros2ai.api.docs import page_title
from ros2ai.api.tools import CompositeCommand
from ros2ai.api.tools import ROS2_TOOLS
from ros2ai.api.tools import Ros2CliProvider
from ros2ai.api.tools import ToolError
from ros2ai.api.tools import ToolPolicy
import yaml

QOS_PAGE = """\
Quality of Service settings
===========================

.. contents:: Table of Contents
   :depth: 2
   :local:

Quality of Service settings in {PRODUCT} {DISTRO_TITLE} help you tune the communication
behaviour between nodes when reliability matters.

Reliability
-----------

* Best effort: attempt to deliver samples, but may lose them if the network is not robust.
* Reliable: guarantee that samples are delivered, may retry multiple times.

.. code-block:: bash

   source /opt/ros/{DISTRO}/setup.bash
"""
TOPICS_PAGE = """\
=======================
Understanding topics
=======================

Topics are a vital element of the ROS graph that act as a bus for nodes to exchange messages.
The reliability of a topic is a quality of service policy.
"""
INSTALL_PAGE = """\
Ubuntu (deb packages)
=====================

Deb packages for {DISTRO_TITLE_FULL} are available for Ubuntu, the branch is {REPOS_FILE_BRANCH}.
"""
# the page without heading, the title falls back to the name
NOTES_PAGE = 'Release notes of the distribution, see the migration guide.\n'

PAGES = {
    'Concepts/Intermediate/About-Quality-of-Service-Settings': QOS_PAGE,
    'Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics': TOPICS_PAGE,
    'Installation/Ubuntu-Install-Debs': INSTALL_PAGE,
    'Releases/Release_Notes': NOTES_PAGE,
}


def make_archive(pages, root='ros2_documentation-rolling', extras=()):
    """Build the branch archive in memory, like the GitHub archive of the branch."""
    buffer = io.BytesIO()
    with tarfile.open(fileobj=buffer, mode='w|gz') as archive:
        def add(name, content):
            data = content.encode()
            info = tarfile.TarInfo(name)
            info.size = len(data)
            archive.addfile(info, io.BytesIO(data))
        add(f'{root}/conf.py', 'macros = {}\n')
        add(f'{root}/source/_static/custom.css', 'body {}\n')
        add(f'{root}/source/images/logo.png', 'not a png')
        for page, content in pages.items():
            add(f'{root}/source/{page}.rst', content)
        for name, content in extras:
            add(name, content)
        directory = tarfile.TarInfo(f'{root}/source/Concepts')
        directory.type = tarfile.DIRTYPE
        archive.addfile(directory)
    buffer.seek(0)
    return buffer


class FakeDownload:
    """Stand-in for urlopen serving the archive, or failing."""

    def __init__(self, archive=None, error=None):
        self.archive = archive
        self.error = error
        self.requests = []

    def __call__(self, request, timeout=None):
        self.requests.append((request.full_url, request.get_header('User-agent'), timeout))
        if self.error is not None:
            raise self.error
        return io.BytesIO(self.archive.getvalue())


@pytest.fixture
def cache_root(tmp_path, monkeypatch):
    root = tmp_path / 'cache'
    monkeypatch.setenv(constants.ROS2AI_DOCS_CACHE_ENV_VAR, str(root))
    monkeypatch.setenv('ROS_DISTRO', 'rolling')
    return root


@pytest.fixture
def download(monkeypatch):
    fake = FakeDownload(make_archive(PAGES))
    monkeypatch.setattr(docs, 'urlopen', fake)
    return fake


@pytest.fixture
def cache(cache_root, download):
    return DocsCache('rolling')


@pytest.fixture
def provider():
    return Ros2CliProvider()


def test_cache_root_default(monkeypatch):
    monkeypatch.delenv(constants.ROS2AI_DOCS_CACHE_ENV_VAR, raising=False)
    assert docs.get_docs_cache_root() == os.path.expanduser(constants.ROS2AI_DOCS_CACHE_DEFAULT)
    monkeypatch.setenv(constants.ROS2AI_DOCS_CACHE_ENV_VAR, '~/elsewhere')
    assert docs.get_docs_cache_root() == os.path.expanduser('~/elsewhere')


def test_expand_placeholders():
    text = '{PRODUCT} {DISTRO_TITLE_FULL} ({DISTRO_TITLE}): /opt/ros/{DISTRO} {REPOS_FILE_BRANCH}'
    assert expand_placeholders(text, 'jazzy') == \
        'ROS 2 Jazzy Jalisco (Jazzy): /opt/ros/jazzy jazzy'
    assert expand_placeholders('{DISTRO_TITLE_FULL} {UNKNOWN}', 'next') == 'Next {UNKNOWN}'


@pytest.mark.parametrize('page, text, title', [
    ('a/b', QOS_PAGE, 'Quality of Service settings'),
    ('a/b', TOPICS_PAGE, 'Understanding topics'),
    ('Releases/Release_Notes', NOTES_PAGE, 'Release Notes'),
    ('How-To-Guides/Node-arguments', 'Short\n==\n', 'Node arguments'),
    ('a/b', '.. comment\n\n   indented\n---\nTitle\n-----\n', 'Title'),
])
def test_page_title(page, text, title):
    assert page_title(page, text) == title


@pytest.mark.parametrize('page, expected', [
    ('Concepts/About-QoS', 'Concepts/About-QoS'),
    ('/Concepts/About-QoS.rst', 'Concepts/About-QoS'),
    ('source/Concepts/About-QoS.html', 'Concepts/About-QoS'),
    (' https://docs.ros.org/en/rolling/Concepts/About-QoS.html ', 'Concepts/About-QoS'),
])
def test_normalize_page(page, expected):
    assert normalize_page('rolling', page) == expected


def test_download_extracts_pages_only(cache, download):
    assert cache.synced_time() is None
    assert cache.is_stale()
    assert cache.ensure() == [
        f'downloaded the ROS 2 rolling documentation to {cache.directory}']
    assert download.requests == [(
        constants.ROS2AI_DOCS_ARCHIVE_URL.format('rolling'), 'ros2ai',
        constants.ROS2AI_DOCS_DOWNLOAD_TIMEOUT_SEC)]
    extracted = sorted(
        os.path.relpath(os.path.join(directory, name), cache.directory)
        for directory, _, names in os.walk(cache.directory) for name in names)
    assert extracted == sorted(
        [page + '.rst' for page in PAGES] + [docs.SYNC_MARKER])
    with open(os.path.join(cache.directory, docs.SYNC_MARKER)) as f:
        marker = json.load(f)
    assert marker['pages'] == len(PAGES)
    assert marker['url'] == cache.url
    assert time.time() - marker['time'] < 60
    assert not cache.is_stale()
    # fresh cache: nothing to do, no download
    assert cache.ensure() == []
    assert len(download.requests) == 1


def test_download_ignores_unsafe_members(cache_root, monkeypatch):
    archive = make_archive(
        {'Page': 'Page\n====\n'},
        extras=[
            ('ros2_documentation-rolling/source/../escape.rst', 'escaped'),
            ('ros2_documentation-rolling/source//absolute.rst', 'absolute'),
            ('ros2_documentation-rolling/other/Not-A-Page.rst', 'other'),
            ('README.rst', 'root'),
        ])
    monkeypatch.setattr(docs, 'urlopen', FakeDownload(archive))
    cache = DocsCache('rolling')
    cache.download()
    assert sorted(cache.pages()) == ['Page']
    assert not os.path.exists(cache_root / 'escape.rst')


def test_download_failure_without_cache(cache_root, monkeypatch):
    monkeypatch.setattr(docs, 'urlopen', FakeDownload(error=URLError('no network')))
    cache = DocsCache('rolling')
    with pytest.raises(DocsError, match='failed to download .*rolling.tar.gz: .*no network'):
        cache.ensure()
    assert not os.path.exists(cache.directory)
    assert not os.path.exists(cache.directory + '.tmp')


def test_download_without_pages(cache_root, monkeypatch):
    monkeypatch.setattr(docs, 'urlopen', FakeDownload(make_archive({})))
    cache = DocsCache('rolling')
    with pytest.raises(DocsError, match='no documentation page found'):
        cache.download()
    assert not os.path.exists(cache.directory)


def test_refresh_stale_cache(cache, download, monkeypatch):
    cache.download()
    marker = os.path.join(cache.directory, docs.SYNC_MARKER)
    with open(marker, 'w') as f:
        json.dump({'time': time.time() - constants.ROS2AI_DOCS_REFRESH_SEC - 1}, f)
    assert cache.is_stale()
    # the refresh replaces the pages
    download.archive = make_archive({'New/Page': 'New page\n========\n'})
    assert cache.ensure() == [
        f'downloaded the ROS 2 rolling documentation to {cache.directory}']
    assert sorted(cache.pages()) == ['New/Page']
    assert not cache.is_stale()
    assert not os.path.exists(cache.directory + '.old')


def test_refresh_failure_keeps_cache(cache, download):
    cache.download()
    marker = os.path.join(cache.directory, docs.SYNC_MARKER)
    with open(marker, 'w') as f:
        json.dump({'time': 0}, f)
    download.error = URLError('offline')
    notices = cache.ensure()
    assert len(notices) == 1
    assert notices[0].startswith('refresh failed, using the cached documentation: ')
    assert 'offline' in notices[0]
    assert sorted(cache.pages()) == sorted(PAGES)
    # the broken marker is also considered stale
    with open(marker, 'w') as f:
        f.write('not json')
    assert cache.is_stale()


def test_pages_expanded_and_reloaded(cache):
    cache.download()
    pages = cache.pages()
    title, text = pages['Concepts/Intermediate/About-Quality-of-Service-Settings']
    assert title == 'Quality of Service settings'
    assert 'ROS 2 Rolling help you tune' in text
    assert 'source /opt/ros/rolling/setup.bash' in text
    assert pages['Installation/Ubuntu-Install-Debs'][1].startswith(
        'Ubuntu (deb packages)\n=====================\n\n'
        'Deb packages for Rolling Ridley are available for Ubuntu, the branch is rolling.')
    # loaded once per synchronization
    assert cache.pages() is pages
    cache.download()
    assert cache.pages() is not pages


def test_search_ranking(cache):
    cache.download()
    results = cache.search('quality of service', 5)
    assert [result['page'] for result in results] == [
        'Concepts/Intermediate/About-Quality-of-Service-Settings',
        'Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics',
    ]
    qos = results[0]
    assert qos['title'] == 'Quality of Service settings'
    assert qos['url'] == ('https://docs.ros.org/en/rolling/'
                          'Concepts/Intermediate/About-Quality-of-Service-Settings.html')
    # the markup (directives, underlines) is left out of the snippet
    assert qos['snippet'].startswith(
        'Quality of Service settings Quality of Service settings in ROS 2 Rolling help you')
    assert 'Table of Contents' not in qos['snippet']
    assert results[1]['snippet'].startswith('... ')
    assert 'quality of service policy' in results[1]['snippet']
    assert results[1]['snippet'].endswith(' ...') is False

    # the terms match the words sharing their stem (`reliable` is not `reliability`),
    # case-insensitive, the limit is honored
    assert [r['page'] for r in cache.search('RELIABLE', 5)] == [
        'Concepts/Intermediate/About-Quality-of-Service-Settings']
    assert len(cache.search('reliability', 5)) == 2
    assert len(cache.search('reliability', 1)) == 1
    assert [r['page'] for r in cache.search('exchanging messages', 5)] == [
        'Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics']
    assert [r['page'] for r in cache.search('deb', 5)] == ['Installation/Ubuntu-Install-Debs']
    # the page name counts too
    assert cache.search('ubuntu debs', 5)[0]['page'] == 'Installation/Ubuntu-Install-Debs'
    assert cache.search('nothing matches here', 5) == []
    assert cache.search('', 5) == []
    assert cache.search('a !', 5) == []
    # the stop words are ignored, unless there is nothing else
    assert [r['page'] for r in cache.search('what is the ros graph?', 5)] == [
        'Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics']
    assert len(cache.search('of the', 5)) == len(PAGES)


def test_definitions_include_docs_tools(provider):
    definitions = {d['function']['name']: d['function'] for d in provider.definitions()}
    assert provider.names == (
        [tool.name for tool in ROS2_TOOLS]
        + [tool.name for tool in COMPOSITE_TOOLS]
        + ['search_ros2_docs', 'read_ros2_docs'])
    search = definitions['search_ros2_docs']
    assert search['parameters']['required'] == ['query']
    assert search['parameters']['properties']['limit'] == {
        'description': search['parameters']['properties']['limit']['description'],
        'type': 'integer', 'minimum': 1, 'maximum': constants.ROS2AI_DOCS_SEARCH_LIMIT_MAX,
    }
    assert 'cite' in search['description']
    read = definitions['read_ros2_docs']
    assert read['parameters']['required'] == ['page']
    assert read['parameters']['properties']['offset']['minimum'] == 0
    for tool in DOCS_TOOLS:
        assert tool.policy == ToolPolicy.READ_ONLY


def test_resolve_docs_tools(provider):
    command = provider.resolve('search_ros2_docs', {'query': 'quality of service'})
    assert isinstance(command, CompositeCommand)
    assert command.policy == ToolPolicy.READ_ONLY
    assert command.display() == "search_ros2_docs(query='quality of service')"
    command = provider.resolve('read_ros2_docs', {'page': 'Concepts/QoS', 'offset': 10})
    assert command.display() == 'read_ros2_docs(page=Concepts/QoS, offset=10)'
    for name, arguments, message in [
        ('search_ros2_docs', {}, "missing required parameter 'query'"),
        ('search_ros2_docs', {'query': 'a', 'limit': 11}, "parameter 'limit' must be at most 10"),
        ('read_ros2_docs', {'page': 'a', 'offset': -1}, "parameter 'offset' must be at least 0"),
    ]:
        with pytest.raises(ToolError, match=message):
            provider.resolve(name, arguments)


def run_tool(provider, name, arguments):
    return provider.run(provider.resolve(name, arguments))


def test_search_tool(provider, cache_root, download, capsys):
    result = run_tool(provider, 'search_ros2_docs', {'query': 'quality of service', 'limit': 1})
    assert result.returncode == 0
    assert result.printable is False
    out = capsys.readouterr().out
    assert out == (
        f'[ros2ai]:   downloaded the ROS 2 rolling documentation to {cache_root}/rolling\n'
        '[ros2ai]:   https://docs.ros.org/en/rolling/'
        'Concepts/Intermediate/About-Quality-of-Service-Settings.html\n')
    data = yaml.safe_load(result.output)
    assert data['query'] == 'quality of service'
    assert data['distro'] == 'rolling'
    assert [page['page'] for page in data['pages']] == [
        'Concepts/Intermediate/About-Quality-of-Service-Settings']
    assert 'hint' not in data

    result = run_tool(provider, 'search_ros2_docs', {'query': 'nothing matches'})
    assert result.returncode == 0
    assert capsys.readouterr().out == ''
    data = yaml.safe_load(result.output)
    assert data['pages'] == []
    assert data['hint'] == 'no page matches, try fewer or different terms'
    # downloaded once
    assert len(download.requests) == 1


def test_read_tool(provider, cache_root, download, capsys, monkeypatch):
    monkeypatch.setattr(constants, 'ROS2AI_DOCS_PAGE_CHARS', 100)
    page = 'Concepts/Intermediate/About-Quality-of-Service-Settings'
    url = f'https://docs.ros.org/en/rolling/{page}.html'
    result = run_tool(provider, 'read_ros2_docs', {'page': url})
    assert result.returncode == 0
    assert result.printable is False
    out = capsys.readouterr().out
    assert out.endswith(f'[ros2ai]:   {url}\n')
    header, text = result.output.split('\n---\n', 1)
    expanded = expand_placeholders(QOS_PAGE, 'rolling')
    assert yaml.safe_load(header) == {
        'page': page,
        'title': 'Quality of Service settings',
        'url': url,
        'characters': f'0-100 of {len(expanded)}, call again with offset 100 for the rest',
    }
    assert text == expanded[:100]

    result = run_tool(provider, 'read_ros2_docs', {'page': page, 'offset': 100})
    header, text = result.output.split('\n---\n', 1)
    assert yaml.safe_load(header)['characters'] == \
        f'100-200 of {len(expanded)}, call again with offset 200 for the rest'
    assert text == expanded[100:200]

    last = (len(expanded) // 100) * 100
    result = run_tool(provider, 'read_ros2_docs', {'page': page, 'offset': last})
    header, text = result.output.split('\n---\n', 1)
    assert yaml.safe_load(header)['characters'] == \
        f'{last}-{len(expanded)} of {len(expanded)}, end of the page'
    assert text == expanded[last:]

    result = run_tool(provider, 'read_ros2_docs', {'page': page, 'offset': len(expanded)})
    assert result.returncode == 1
    assert result.printable is True
    assert result.output == \
        f'offset {len(expanded)} is beyond the end of the page ({len(expanded)} characters)'

    result = run_tool(provider, 'read_ros2_docs', {'page': 'Concepts/Missing'})
    assert result.returncode == 1
    assert result.output == \
        "unknown page 'Concepts/Missing', use search_ros2_docs to find the page name"
    assert capsys.readouterr().out.count('[ros2ai]:') == 3


def test_tools_without_documentation(provider, cache_root, monkeypatch, capsys):
    monkeypatch.setattr(docs, 'urlopen', FakeDownload(error=URLError('offline')))
    for name, arguments in [
            ('search_ros2_docs', {'query': 'topics'}),
            ('read_ros2_docs', {'page': 'Concepts/Topics'})]:
        result = run_tool(provider, name, arguments)
        assert result.returncode == 1
        assert result.printable is True
        assert result.output.startswith(
            'the ROS 2 rolling documentation is not available: failed to download ')
        assert result.output.endswith('<urlopen error offline>. Answer from your own knowledge.')
    assert capsys.readouterr().out == ''


def test_tools_follow_distro_and_cache(provider, cache_root, download, monkeypatch, tmp_path):
    run_tool(provider, 'search_ros2_docs', {'query': 'topics'})
    assert os.path.isdir(cache_root / 'rolling')
    monkeypatch.setenv('ROS_DISTRO', 'jazzy')
    download.archive = make_archive({'Jazzy/Page': 'Jazzy page\n==========\n'}, root='x-jazzy')
    result = run_tool(provider, 'search_ros2_docs', {'query': 'jazzy'})
    assert download.requests[-1][0] == constants.ROS2AI_DOCS_ARCHIVE_URL.format('jazzy')
    assert os.path.isdir(cache_root / 'jazzy')
    assert yaml.safe_load(result.output)['pages'][0]['url'] == \
        'https://docs.ros.org/en/jazzy/Jazzy/Page.html'
    other = tmp_path / 'other'
    monkeypatch.setenv(constants.ROS2AI_DOCS_CACHE_ENV_VAR, str(other))
    run_tool(provider, 'read_ros2_docs', {'page': 'Jazzy/Page'})
    assert os.path.isdir(other / 'jazzy')
    assert len(download.requests) == 3
