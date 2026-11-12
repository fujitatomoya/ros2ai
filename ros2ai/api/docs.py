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
Documentation tools that ground the answers on the ROS 2 documentation of the distribution.

The reStructuredText sources of https://github.com/ros2/ros2_documentation (one branch per
distribution) are cached locally: downloaded once from the branch archive, keeping only the
pages, and refreshed when older than a period. The lookup works offline afterwards.
The pages are published at https://docs.ros.org/en/<distro>/, the tools return the page URL
so that the model can cite it.
"""

import json
import os
import re
import shutil
import tarfile
import time
from typing import Any, Dict, List, Optional, Tuple
from urllib.request import Request, urlopen

import ros2ai.api.constants as constants
from ros2ai.api.tools import CompositeTool, Executor, Parameter, ToolPolicy, ToolResult
import ros2ai.api.tracing as tracing
from ros2ai.api.utils import get_ros_distro

import yaml

# name of the file recording the last synchronization in the cache of a distribution
SYNC_MARKER = 'sync.json'
# directory of the pages in the documentation repository
SOURCE_DIRECTORY = 'source'
PAGE_SUFFIX = '.rst'
USER_AGENT = 'ros2ai'

_WORD = re.compile(r'[a-z0-9_]+')
# directive or comment line (`.. note::`), its body is indented
_DIRECTIVE = re.compile(r'^\.\.(\s|$)')
# suffixes removed to match the words sharing the stem, e.g `creating` and `create`
_SUFFIXES = ('ing', 'ies', 'es', 'ed', 's', 'e')
# query words that do not select a page, ignored unless the query has nothing else
_STOP_WORDS = frozenset((
    'a', 'an', 'the', 'of', 'to', 'in', 'on', 'for', 'and', 'or', 'is', 'are', 'be', 'with',
    'how', 'what', 'do', 'does', 'i', 'my', 'me', 'it', 'this', 'that', 'can', 'use', 'using',
    'ros', 'ros2',
))
_UNDERLINE = re.compile(r'^([=\-~^"\'`#*+_:.])\1{2,}\s*$')


class DocsError(Exception):
    """Raised when the documentation cache can not be synchronized."""


def get_docs_cache_root() -> str:
    """
    Get the documentation cache root from ROS2AI_DOCS_CACHE environment variable.

    :return: the cache root directory, the default under the home directory if not set.
    """
    root = os.environ.get(constants.ROS2AI_DOCS_CACHE_ENV_VAR)
    return os.path.expanduser(root or constants.ROS2AI_DOCS_CACHE_DEFAULT)


def expand_placeholders(text: str, distro: str) -> str:
    """
    Expand the placeholders of the documentation sources, as the documentation build does.

    :param text: content of a page.
    :param distro: distribution name, e.g `rolling`.
    :return: the text with `{DISTRO}`, `{DISTRO_TITLE}` and the other placeholders expanded.
    """
    values = {
        'DISTRO': distro,
        'DISTRO_TITLE': distro.title(),
        'DISTRO_TITLE_FULL': constants.ROS_DISTRO_FULL_NAMES.get(distro, distro.title()),
        'REPOS_FILE_BRANCH': distro,
        'PRODUCT': 'ROS 2',
    }
    for name, value in values.items():
        text = text.replace('{' + name + '}', value)
    return text


def page_title(page: str, text: str) -> str:
    """
    Extract the title of a page, the first section heading of the reStructuredText.

    :param page: page name, used when the text has no heading.
    :param text: content of the page.
    :return: the title.
    """
    lines = text.splitlines()
    for index in range(len(lines) - 1):
        line, underline = lines[index].strip(), lines[index + 1]
        if (line and not _UNDERLINE.match(lines[index])
                and _UNDERLINE.match(underline) and len(underline.rstrip()) >= len(line)):
            return line
    return page.rsplit('/', 1)[-1].replace('-', ' ').replace('_', ' ')


def page_url(distro: str, page: str) -> str:
    """Return the URL of a page on the documentation site."""
    return constants.ROS2AI_DOCS_SITE_URL.format(distro) + page + '.html'


def normalize_page(distro: str, page: str) -> str:
    """
    Normalize a page name given by the model into the cache page name.

    Accepts the page URL, the source path and the `.rst` or `.html` suffix.
    """
    page = page.strip()
    site = constants.ROS2AI_DOCS_SITE_URL.format(distro)
    if page.startswith(site):
        page = page[len(site):]
    page = page.lstrip('/')
    if page.startswith(SOURCE_DIRECTORY + '/'):
        page = page[len(SOURCE_DIRECTORY) + 1:]
    for suffix in (PAGE_SUFFIX, '.html'):
        if page.endswith(suffix):
            page = page[:-len(suffix)]
    return page


def _terms(query: str) -> List[str]:
    """Split the query into the lowercase search terms, stop words and duplicates removed."""
    words = []
    for word in _WORD.findall(query.lower()):
        if len(word) >= 2 and word not in words:
            words.append(word)
    terms = [word for word in words if word not in _STOP_WORDS]
    return terms or words


def _stem(term: str) -> str:
    """Return the stem of a term, the term itself when too short."""
    for suffix in _SUFFIXES:
        if term.endswith(suffix) and len(term) - len(suffix) >= 3:
            return term[:-len(suffix)]
    return term


def _pattern(terms: List[str]) -> 're.Pattern':
    """Compile the pattern matching the words sharing the stems of the terms, in sequence."""
    stems = [r'\b' + re.escape(_stem(term)) + r'\w*' for term in terms]
    # up to two words (e.g the stop words) are allowed between the terms of a phrase
    return re.compile(r'(?:\W+\w+){0,2}\W+'.join(stems))


def _prose(text: str) -> str:
    """Return the text without the markup: directives with their body and heading underlines."""
    lines = []
    directive = False
    for line in text.splitlines():
        if not line.strip():
            continue
        if _DIRECTIVE.match(line):
            directive = True
            continue
        if directive and line[0].isspace():
            continue
        directive = False
        if not _UNDERLINE.match(line):
            lines.append(line)
    return '\n'.join(lines)


def _snippet(text: str, pattern: 're.Pattern', chars: int) -> str:
    """Return the prose around the first match, whitespace collapsed."""
    text = _prose(text)
    match = pattern.search(text.lower())
    if match is None:
        start = 0
    else:
        start = max(match.start() - chars // 3, 0)
    end = min(start + chars, len(text))
    snippet = ' '.join(text[start:end].split())
    if start > 0:
        snippet = '... ' + snippet
    if end < len(text):
        snippet += ' ...'
    return snippet


class DocsCache:
    """Local cache of the documentation pages of a distribution."""

    def __init__(self, distro: str, root: Optional[str] = None):
        self.distro = distro
        self.root = root if root is not None else get_docs_cache_root()
        self.directory = os.path.join(self.root, distro)
        self.url = constants.ROS2AI_DOCS_ARCHIVE_URL.format(distro)
        self._pages: Optional[Dict[str, Tuple[str, str]]] = None
        self._loaded_marker: Optional[float] = None

    def synced_time(self) -> Optional[float]:
        """Return the time of the last synchronization, None if never synchronized."""
        try:
            with open(os.path.join(self.directory, SYNC_MARKER)) as f:
                return float(json.load(f)['time'])
        except (OSError, ValueError, KeyError, TypeError):
            return None

    def is_stale(self, now: Optional[float] = None) -> bool:
        """Check if the cache is older than the refresh period."""
        synced = self.synced_time()
        if synced is None:
            return True
        now = time.time() if now is None else now
        return now - synced > constants.ROS2AI_DOCS_REFRESH_SEC

    def ensure(self) -> List[str]:
        """
        Make sure the cache is available, downloading or refreshing it as needed.

        A stale cache is kept when the refresh fails, the notices tell what happened.

        :return: notices for the user, empty if nothing was done.
        :raises DocsError: if the cache is not available and can not be downloaded.
        """
        if not self.is_stale():
            return []
        synced = self.synced_time()
        try:
            self.download()
        except DocsError as e:
            if synced is None:
                raise
            return [f'refresh failed, using the cached documentation: {e}']
        return [f'downloaded the ROS 2 {self.distro} documentation to {self.directory}']

    def download(self):
        """
        Download the branch archive and extract the pages into the cache.

        The archive is streamed, only the pages are extracted into a temporary directory
        which replaces the cache directory once complete.

        :raises DocsError: if the archive can not be downloaded or contains no page.
        """
        temporary = self.directory + '.tmp'
        shutil.rmtree(temporary, ignore_errors=True)
        os.makedirs(temporary)
        count = 0
        try:
            request = Request(self.url, headers={'User-Agent': USER_AGENT})
            response = urlopen(request, timeout=constants.ROS2AI_DOCS_DOWNLOAD_TIMEOUT_SEC)
            try:
                with tarfile.open(fileobj=response, mode='r|gz') as archive:
                    for member in archive:
                        page = _page_of_member(member)
                        if page is None:
                            continue
                        path = os.path.join(temporary, page + PAGE_SUFFIX)
                        os.makedirs(os.path.dirname(path), exist_ok=True)
                        with archive.extractfile(member) as source, open(path, 'wb') as f:
                            shutil.copyfileobj(source, f)
                        count += 1
            finally:
                response.close()
        except Exception as e:
            shutil.rmtree(temporary, ignore_errors=True)
            raise DocsError(f'failed to download {self.url}: {e}')
        if count == 0:
            shutil.rmtree(temporary, ignore_errors=True)
            raise DocsError(f'no documentation page found in {self.url}')
        with open(os.path.join(temporary, SYNC_MARKER), 'w') as f:
            json.dump({'time': time.time(), 'url': self.url, 'pages': count}, f)
        previous = self.directory + '.old'
        shutil.rmtree(previous, ignore_errors=True)
        if os.path.isdir(self.directory):
            os.rename(self.directory, previous)
        os.rename(temporary, self.directory)
        shutil.rmtree(previous, ignore_errors=True)
        self._pages = None

    def pages(self) -> Dict[str, Tuple[str, str]]:
        """
        Load the pages of the cache.

        :return: the (title, text) of each page by page name, placeholders expanded.
        """
        synced = self.synced_time()
        if self._pages is not None and self._loaded_marker == synced:
            return self._pages
        pages = {}
        for directory, _, names in os.walk(self.directory):
            for name in names:
                if not name.endswith(PAGE_SUFFIX):
                    continue
                path = os.path.join(directory, name)
                page = os.path.relpath(path, self.directory)[:-len(PAGE_SUFFIX)]
                page = page.replace(os.sep, '/')
                with open(path, encoding='utf-8', errors='replace') as f:
                    text = expand_placeholders(f.read(), self.distro)
                pages[page] = (page_title(page, text), text)
        self._pages = pages
        self._loaded_marker = synced
        return pages

    def search(self, query: str, limit: int) -> List[Dict[str, Any]]:
        """
        Search the pages for the query terms.

        The pages are scored by the terms found in the title, the page name and the text,
        with a bonus when every term or the whole phrase is found. The terms match the words
        sharing their stem, e.g `create` matches `creating`. The text matches count by their
        density, so that the long pages (e.g the changelogs) do not prevail, the titles decide.

        :param query: search query, split into terms.
        :param limit: maximum number of results.
        :return: the best pages, each with `page`, `title`, `url` and `snippet`.
        """
        terms = _terms(query)
        if not terms:
            return []
        patterns = [_pattern([term]) for term in terms]
        phrase = _pattern(terms)
        scored = []
        for page, (title, text) in self.pages().items():
            lower = text.lower()
            title_lower = title.lower()
            name_lower = page.lower().replace('-', ' ').replace('_', ' ').replace('/', ' ')
            score = 0.0
            matched = 0
            for pattern in patterns:
                count = len(pattern.findall(lower))
                if count:
                    matched += 1
                    # matches per 1000 characters, 10 at most
                    score += min(count * 1000 / len(text), 10)
                if pattern.search(title_lower):
                    score += 15
                if pattern.search(name_lower):
                    score += 8
            if not matched:
                continue
            if matched == len(terms):
                score += 10 * len(terms)
            if phrase.search(lower) or phrase.search(title_lower):
                score += 30
            scored.append((score, page, title, text, lower))
        scored.sort(key=lambda item: (-item[0], item[1]))
        results = []
        for score, page, title, text, lower in scored[:limit]:
            results.append({
                'page': page,
                'title': title,
                'url': page_url(self.distro, page),
                'snippet': _snippet(
                    text, _first_matching([phrase, *patterns], lower),
                    constants.ROS2AI_DOCS_SNIPPET_CHARS),
            })
        return results


def _first_matching(patterns, lower: str) -> 're.Pattern':
    """Return the first pattern found in the text, the last pattern if none matches."""
    for pattern in patterns:
        if pattern.search(lower):
            return pattern
    return patterns[-1]


def _page_of_member(member: tarfile.TarInfo) -> Optional[str]:
    """
    Return the page name of an archive member, None if the member is not a page.

    The pages are the `.rst` files under `<archive root>/source/`, any member that could
    escape the cache directory is ignored.
    """
    if not member.isfile() or not member.name.endswith(PAGE_SUFFIX):
        return None
    parts = member.name.split('/')
    if len(parts) < 3 or parts[1] != SOURCE_DIRECTORY:
        return None
    pages = parts[2:]
    if any(part in ('', '.', '..') for part in pages):
        return None
    return '/'.join(pages)[:-len(PAGE_SUFFIX)]


def _dump(result: Dict[str, Any]) -> str:
    """Serialize the result into YAML, keeping the key order."""
    return yaml.safe_dump(
        result, sort_keys=False, default_flow_style=False, allow_unicode=True, width=200,
    ).rstrip()


def _trace(line: str):
    # the pages are traced indented under the tool call, like the composite commands
    tracing.trace(line, nested=True)


class DocsTool(CompositeTool):
    """Base of the documentation tools, bound to the cache of the current distribution."""

    policy = ToolPolicy.READ_ONLY
    timeout = constants.ROS2AI_DOCS_DOWNLOAD_TIMEOUT_SEC

    def __init__(self):
        self._cache: Optional[DocsCache] = None

    def cache(self) -> DocsCache:
        """Return the cache of the current distribution, created or replaced when it changes."""
        distro = get_ros_distro()
        root = get_docs_cache_root()
        if self._cache is None or self._cache.distro != distro or self._cache.root != root:
            self._cache = DocsCache(distro, root)
        return self._cache

    def run(self, executor: Executor, arguments: Dict[str, Any]) -> ToolResult:
        cache = self.cache()
        try:
            for notice in cache.ensure():
                # the download takes a while and the refresh failure matters, printed regardless
                tracing.notice(notice, nested=True)
        except DocsError as e:
            return ToolResult(
                output=f'the ROS 2 {cache.distro} documentation is not available: {e}. '
                       'Answer from your own knowledge.',
                returncode=1)
        return self.lookup(cache, arguments)

    def lookup(self, cache: DocsCache, arguments: Dict[str, Any]) -> ToolResult:
        """Execute the tool on the synchronized cache."""
        raise NotImplementedError


class SearchDocsTool(DocsTool):
    """Search the documentation pages."""

    name = 'search_ros2_docs'
    description = (
        'Search the official ROS 2 documentation of the running distribution '
        '(concepts, tutorials, how-to guides, installation, tools and APIs). '
        'Returns the best matching pages with their URL and a snippet, '
        'read a page with read_ros2_docs. Use this before answering questions about ROS 2 '
        'and cite the page URL in the answer.')
    parameters = {
        'query': Parameter('Search terms, e.g "quality of service reliability"'),
        'limit': Parameter(
            f'Maximum number of pages, 1 to {constants.ROS2AI_DOCS_SEARCH_LIMIT_MAX} '
            f'(default {constants.ROS2AI_DOCS_SEARCH_LIMIT})',
            required=False, kind='integer',
            minimum=1, maximum=constants.ROS2AI_DOCS_SEARCH_LIMIT_MAX),
    }

    def lookup(self, cache: DocsCache, arguments: Dict[str, Any]) -> ToolResult:
        query = arguments['query']
        limit = arguments.get('limit', constants.ROS2AI_DOCS_SEARCH_LIMIT)
        pages = cache.search(query, limit)
        for page in pages:
            _trace(page['url'])
        result = {'query': query, 'distro': cache.distro, 'pages': pages}
        if not pages:
            result['hint'] = 'no page matches, try fewer or different terms'
        return ToolResult(output=_dump(result), returncode=0, printable=False)


class ReadDocsTool(DocsTool):
    """Read a documentation page."""

    name = 'read_ros2_docs'
    description = (
        'Read a page of the official ROS 2 documentation of the running distribution, '
        'found with search_ros2_docs. Returns the page text in reStructuredText with its URL, '
        f'{constants.ROS2AI_DOCS_PAGE_CHARS} characters at a time, '
        'call again with the offset to read the rest.')
    parameters = {
        'page': Parameter(
            'Page name as returned by search_ros2_docs, '
            'e.g Concepts/Intermediate/About-Quality-of-Service-Settings'),
        'offset': Parameter(
            'Character offset to continue reading a long page (default 0)',
            required=False, kind='integer', minimum=0),
    }

    def lookup(self, cache: DocsCache, arguments: Dict[str, Any]) -> ToolResult:
        page = normalize_page(cache.distro, arguments['page'])
        offset = arguments.get('offset', 0)
        pages = cache.pages()
        if page not in pages:
            return ToolResult(
                output=f"unknown page '{arguments['page']}', "
                       'use search_ros2_docs to find the page name',
                returncode=1)
        title, text = pages[page]
        url = page_url(cache.distro, page)
        _trace(url)
        if offset >= len(text):
            return ToolResult(
                output=f'offset {offset} is beyond the end of the page ({len(text)} characters)',
                returncode=1)
        end = min(offset + constants.ROS2AI_DOCS_PAGE_CHARS, len(text))
        header = {'page': page, 'title': title, 'url': url}
        if end < len(text):
            header['characters'] = \
                f'{offset}-{end} of {len(text)}, call again with offset {end} for the rest'
        else:
            header['characters'] = f'{offset}-{end} of {len(text)}, end of the page'
        output = _dump(header) + '\n---\n' + text[offset:end]
        return ToolResult(output=output, returncode=0, printable=False)


DOCS_TOOLS = (SearchDocsTool(), ReadDocsTool())
