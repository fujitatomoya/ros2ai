^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2ai
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2026-11-13)
------------------
* Adjust verification.sh to the session mode. (`#122 <https://github.com/fujitatomoya/ros2ai/issues/122>`_)
  `ros2 ai` without a request now starts the interactive session, so the
  verification blocked on the prompt. The interactive session is driven by
  the requests piped to the standard input instead, and the session file is
  isolated in a temporary directory so that the verification does not touch
  the session of the user. `--continue` and `--trace` are verified as well.
  Co-authored-by: Claude Fable 5.1 <noreply@anthropic.com>
* cosmetic documentation fixes.
* Hide the trace of the tool calls by default, --trace prints it. (`#120 <https://github.com/fujitatomoya/ros2ai/issues/120>`_) (`#121 <https://github.com/fujitatomoya/ros2ai/issues/121>`_)
  The tool calls, the executed commands and their output are not printed
  unless -T / --trace is given (-v implies it). The commands that change
  the system, the failures and the documentation download are printed
  regardless. The system role tells the model that the user does not see
  the tool results, so that the answer is self-contained.
  Co-authored-by: Claude Fable 5.1 <noreply@anthropic.com>
* Align documentation with the current implementation. (`#116 <https://github.com/fujitatomoya/ros2ai/issues/116>`_)
  * Align documentation with the current implementation.
  - README: ros2ai is released for every supported distribution, drop the
  unused ollama pip package, fix the PEP 668 link and add the command
  line options table.
  - Dockerfile: drop the unused ollama pip package.
  - Architecture diagram: add Anthropic, OpenAI compatible API and the
  ros2cli tool calling path.
  - overview: add the tool calling policy slide, remove the daemonized
  agent plan (`#35 <https://github.com/fujitatomoya/ros2ai/issues/35>`_) and regenerate the html and pdf exports.
  Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>
  * Drop the unused ollama pip package from the CI script as well.
  Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>
  * Ground the answers on the ROS 2 documentation of the distribution. (`#117 <https://github.com/fujitatomoya/ros2ai/issues/117>`_) (`#118 <https://github.com/fujitatomoya/ros2ai/issues/118>`_)
  Add the documentation tools `search_ros2_docs` and `read_ros2_docs` that
  look up the official ROS 2 documentation of the running distribution.
  - The reStructuredText sources of the `ROS_DISTRO` branch of
  ros2/ros2_documentation are cached under `~/.cache/ros2ai/docs/<distro>`
  (`ROS2AI_DOCS_CACHE`), only the pages are extracted from the branch
  archive, refreshed when older than a week, the lookup works offline.
  - `search_ros2_docs` scores the pages by the query terms in the title,
  the page name and the text (stem matching, density, phrase bonus) and
  returns the pages with their URL and a snippet.
  - `read_ros2_docs` returns the page text with the placeholders expanded,
  paged to fit the tool output limit, with the docs.ros.org URL.
  - The page content is only returned to the model (`ToolResult.printable`),
  the matched pages are printed under the tool call.
  - The system role asks to consult the documentation and to cite the page.
  - README: documentation lookup, `ROS2AI_DOCS_CACHE`, Ollama context length.
  - overview: documentation grounding slide, exports regenerated.
  Co-authored-by: Claude Fable 5.1 <noreply@anthropic.com>
  ---------
  Co-authored-by: Claude Fable 5.1 <noreply@anthropic.com>
* Support session mode to keep the context of the requests. (`#34 <https://github.com/fujitatomoya/ros2ai/issues/34>`_) (`#115 <https://github.com/fujitatomoya/ros2ai/issues/115>`_)
  `ros2 ai` without a request starts the interactive session, and
  `ros2 ai --continue "<request>"` continues the previous session saved
  to the session file, so that the follow-up requests can refer to the
  previous ones. The conversation is kept on the client side and sent
  with every request, grouped by turns so that the tool calls are never
  separated from their results. The oldest turns are dropped beyond
  ROS2AI_CONTEXT_LIMIT, or when the endpoint reports that the context
  length is exceeded.
  closes `#34 <https://github.com/fujitatomoya/ros2ai/issues/34>`_
  Co-authored-by: Claude Fable 5.1 <noreply@anthropic.com>
* Single entry point without sub-command: ros2 ai "<request>". (`#112 <https://github.com/fujitatomoya/ros2ai/issues/112>`_)
  * Single entry point without sub-command: ros2 ai "<request>".
  `ros2 ai` no longer has sub-commands, the user passes the question or the
  operation request, and the model decides by itself whether to answer or to
  call the ROS 2 CLI tools via the agent loop.
  - `query`, `exec` and `status` verbs are removed, with the `ros2ai.verb`
  extension point.
  - `--status` (`-s`) and `--list-models` (`-l`) replace the `status` verb.
  - `--verbose` (`-v`) replaces both `query -v` and `exec -d`.
  - One default system role for both answering and operating the system.
  - When the endpoint or the model does not support tool calling, the model
  answers without tools, the single command fallback is removed.
  - `ros2 ai -h` prints usage examples, bare `ros2 ai` prints the help.
  closes `#111 <https://github.com/fujitatomoya/ros2ai/issues/111>`_
  Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>
  * Stream the completions in the agent loop. (`#113 <https://github.com/fujitatomoya/ros2ai/issues/113>`_)
  The content is printed as it arrives, the tool call deltas are assembled
  by their index and executed when the stream ends, so that the answers to
  the questions are streamed as `ros2 ai query` did.
  - `--nostream` (`-n`) disables the streaming, the raw completions are
  printed with `-nv` only.
  - `assemble_stream()` tolerates endpoints that omit the tool call ids,
  reuse the index, or append usage chunks without choices.
  - The client prints the reason when the completion did not finish, in
  both streaming and non-streaming modes.
  Co-authored-by: Claude Fable 5.1 <noreply@anthropic.com>
  ---------
  Co-authored-by: Claude Fable 5.1 <noreply@anthropic.com>
* Add composite tools get_node_status and get_topic_status. (`#110 <https://github.com/fujitatomoya/ros2ai/issues/110>`_) (`#114 <https://github.com/fujitatomoya/ros2ai/issues/114>`_)
  Some diagnostic questions ("is /talker healthy?") need the output of
  several `ros2` commands at once. The model could chain the single command
  tools, at the cost of one completion per command and of the model deciding
  the sequence. Composite tools run a fixed sequence of read-only `ros2`
  commands in Python, behind the same `ToolProvider` interface, and return one
  YAML document of bounded size.
  - `ros2ai/api/tools.py`: `CompositeTool` and `Executor` interfaces,
  `CompositeCommand` resolved by the provider, the executor classifies every
  sub-command and refuses the ones beyond the policy of the composite tool
  (a read-only tool can not run a mutating command, nobody runs long-running
  commands). `Ros2CliProvider.sample()` runs commands such as `ros2 topic hz`
  in parallel for a window, interrupts them with SIGINT and captures the
  output (unbuffered, SIGKILL would lose it). Sub-commands are printed
  indented under the tool call, the argument validation gains integer ranges.
  - `ros2ai/api/composite.py`: `get_node_status(node, window)` with node info,
  lifecycle state (lifecycle nodes only), parameter names and the publishing
  rates of the published topics. `get_topic_status(topic, window)` with topic
  info, endpoints with QoS, publishing rate, a sample message and the QoS
  incompatibilities. Both report the warnings found and the available
  nodes/topics when the name is unknown, the model judges the health.
  - unit tests with a fake executor and outputs captured from Humble and
  Rolling, provider and verb tests for the composite path.
  - README: composite tools section with a transcript.
  Verified with scripts/github_workflows.sh on humble, jazzy, kilted, lyrical
  and rolling, and end-to-end with Ollama (qwen3.8:27b) on rolling and humble.
  Co-authored-by: Claude Fable 5.1 <noreply@anthropic.com>
* Operate ROS 2 system via tool calling with ROS 2 CLI tools. (`#109 <https://github.com/fujitatomoya/ros2ai/issues/109>`_)
  `ros2 ai exec` no longer asks the model for a shell command line that is
  executed with `shell=True`. The model now calls ROS 2 CLI tools (function
  calling) through an agent loop, observes the output and iterates until the
  request is done. Every command is executed as an argument list without a
  shell, `argv[0]` is always `ros2`.
  - `ros2ai/api/tools.py`: ToolProvider interface and the Ros2CliProvider that
  maps the tools to the `ros2` CLI (node, topic, service, action, param,
  interface, pkg, doctor, run, launch, plus a generic `ros2_cli` tool).
  Tools are classified as read-only (executed automatically), mutating
  (confirmation, `-y/--yes` skips) and long-running (confirmation, foreground).
  - `ros2ai/api/agent.py`: agent loop with maximum steps, tool output limit,
  errors returned to the model, `--dry-run` printing the command candidates
  that change the system.
  - `ros2ai/api/client.py`: `tools` support, `tool_choice` is left to auto.
  - fallback to the single command prompt when the endpoint or model does not
  support tool calling, still executed without a shell.
  - `run_executable()` never uses a shell anymore.
  - unit tests for tools, agent and the exec verb with fake client and `ros2`.
  Co-authored-by: Claude Fable 5.1 <noreply@anthropic.com>
* Support Anthropic API and generic ROS2AI\_* configuration. (`#108 <https://github.com/fujitatomoya/ros2ai/issues/108>`_)
  closes `#105 <https://github.com/fujitatomoya/ros2ai/issues/105>`_
  - ROS2AI_API_KEY, ROS2AI_MODEL_NAME, ROS2AI_ENDPOINT, ROS2AI_TEMPERATURE and
  ROS2AI_ROLE_SYSTEM configure ros2ai agnostic from the LLM managed service.
  legacy OPENAI\_* environment variables are still honored as fallback.
  - Anthropic API is supported via OpenAI SDK compatibility endpoint,
  `x-api-key` header is added for Anthropic so that /v1/models works.
  - user facing wording is agnostic from OpenAI (help, status messages).
  - README, overview slide deck (md/html/pdf), scripts are aligned.
  - verification.sh takes the endpoint and model from the environment.
  Co-authored-by: Claude Fable 5.1 <noreply@anthropic.com>
* Refactor implementation and add unit tests. (`#107 <https://github.com/fujitatomoya/ros2ai/issues/107>`_)
  No change in capabilities, command line arguments, options or output.
  - split ros2ai/api/__init_\_.py into arguments.py, drop the curl based
  model listing in favor of the OpenAI python client (curl dependency removed).
  - rename ros2ai/api/openai.py to client.py, the client takes OpenAiConfig
  by composition instead of inheritance, and reports API call failures
  instead of crashing with a traceback afterwards.
  - ChatCompletionParameters becomes a dataclass, chat_messages() helper.
  - config.py: resolve_role_system() shares the role resolution of query/exec,
  dead set_value()/remove_key() removed, is_api_key_valid() folded into status.
  - utils.py: dead run_command() and unused parameters removed,
  extract_ros2_command() composes the command extraction for exec.
  - make the code base pass ament_flake8/pep257/copyright/xmllint.
  - add unit tests (utils, config, argument parsing, mocked verbs) and
  run colcon test in the github workflow for every distribution.
  Co-authored-by: Claude Fable 5.1 <noreply@anthropic.com>
* Remove Gemini CLI github workflows. (`#104 <https://github.com/fujitatomoya/ros2ai/issues/104>`_)
  closes `#103 <https://github.com/fujitatomoya/ros2ai/issues/103>`_
  Co-authored-by: Claude Fable 5.1 <noreply@anthropic.com>
* Support Lyrical Luth with docker image. (`#102 <https://github.com/fujitatomoya/ros2ai/issues/102>`_)
  closes `#93 <https://github.com/fujitatomoya/ros2ai/issues/93>`_
  Co-authored-by: Claude Fable 5.1 <noreply@anthropic.com>
* 🛠️ Bump actions/create-github-app-token from 3.2.0 to 3.3.0 (`#101 <https://github.com/fujitatomoya/ros2ai/issues/101>`_)
  Bumps [actions/create-github-app-token](https://github.com/actions/create-github-app-token) from 3.2.0 to 3.3.0.
  - [Release notes](https://github.com/actions/create-github-app-token/releases)
  - [Changelog](https://github.com/actions/create-github-app-token/blob/main/CHANGELOG.md)
  - [Commits](https://github.com/actions/create-github-app-token/compare/bcd2ba49218906704ab6c1aa796996da409d3eb1...64ab729a6294000f5aa4a4bc0b73d1f32bc45510)
  ---
  updated-dependencies:
  - dependency-name: actions/create-github-app-token
  dependency-version: 3.3.0
  dependency-type: direct:production
  update-type: version-update:semver-minor
  ...
  Co-authored-by: dependabot[bot] <49699333+dependabot[bot]@users.noreply.github.com>
* 🛠️ Bump actions/stale from 10 to 11 (`#100 <https://github.com/fujitatomoya/ros2ai/issues/100>`_)
  Bumps [actions/stale](https://github.com/actions/stale) from 10 to 11.
  - [Release notes](https://github.com/actions/stale/releases)
  - [Changelog](https://github.com/actions/stale/blob/main/CHANGELOG.md)
  - [Commits](https://github.com/actions/stale/compare/v10...v11)
  ---
  updated-dependencies:
  - dependency-name: actions/stale
  dependency-version: '11'
  dependency-type: direct:production
  update-type: version-update:semver-major
  ...
  Co-authored-by: dependabot[bot] <49699333+dependabot[bot]@users.noreply.github.com>
* 🛠️ Bump actions/checkout from 7.0.0 to 7.0.1 (`#99 <https://github.com/fujitatomoya/ros2ai/issues/99>`_)
  Bumps [actions/checkout](https://github.com/actions/checkout) from 7.0.0 to 7.0.1.
  - [Release notes](https://github.com/actions/checkout/releases)
  - [Commits](https://github.com/actions/checkout/compare/v7...v7.0.1)
  ---
  updated-dependencies:
  - dependency-name: actions/checkout
  dependency-version: 7.0.1
  dependency-type: direct:production
  update-type: version-update:semver-patch
  ...
  Co-authored-by: dependabot[bot] <49699333+dependabot[bot]@users.noreply.github.com>
* 🛠️ Bump actions/setup-python from 6 to 7 (`#98 <https://github.com/fujitatomoya/ros2ai/issues/98>`_)
  Bumps [actions/setup-python](https://github.com/actions/setup-python) from 6 to 7.
  - [Release notes](https://github.com/actions/setup-python/releases)
  - [Commits](https://github.com/actions/setup-python/compare/v6...v7)
  ---
  updated-dependencies:
  - dependency-name: actions/setup-python
  dependency-version: '7'
  dependency-type: direct:production
  update-type: version-update:semver-major
  ...
  Co-authored-by: dependabot[bot] <49699333+dependabot[bot]@users.noreply.github.com>
* remove deprecated warnings from newer setuptools. (`#97 <https://github.com/fujitatomoya/ros2ai/issues/97>`_)
* fix rolling nightly build, that depends on statically ubuntu24.04. (`#96 <https://github.com/fujitatomoya/ros2ai/issues/96>`_)
* 🛠️ Bump actions/checkout from 6.0.3 to 7.0.0 (`#94 <https://github.com/fujitatomoya/ros2ai/issues/94>`_)
  Bumps [actions/checkout](https://github.com/actions/checkout) from 6.0.3 to 7.0.0.
  - [Release notes](https://github.com/actions/checkout/releases)
  - [Commits](https://github.com/actions/checkout/compare/v6.0.3...v7)
  ---
  updated-dependencies:
  - dependency-name: actions/checkout
  dependency-version: 7.0.0
  dependency-type: direct:production
  update-type: version-update:semver-major
  ...
  Co-authored-by: dependabot[bot] <49699333+dependabot[bot]@users.noreply.github.com>
* 🛠️ Bump actions/checkout from 6.0.2 to 6.0.3 (`#92 <https://github.com/fujitatomoya/ros2ai/issues/92>`_)
  Bumps [actions/checkout](https://github.com/actions/checkout) from 6.0.2 to 6.0.3.
  - [Release notes](https://github.com/actions/checkout/releases)
  - [Commits](https://github.com/actions/checkout/compare/v6.0.2...v6.0.3)
  ---
  updated-dependencies:
  - dependency-name: actions/checkout
  dependency-version: 6.0.3
  dependency-type: direct:production
  update-type: version-update:semver-patch
  ...
  Co-authored-by: dependabot[bot] <49699333+dependabot[bot]@users.noreply.github.com>
* 🛠️ Bump actions/create-github-app-token from 3.1.1 to 3.2.0 (`#91 <https://github.com/fujitatomoya/ros2ai/issues/91>`_)
  Bumps [actions/create-github-app-token](https://github.com/actions/create-github-app-token) from 3.1.1 to 3.2.0.
  - [Release notes](https://github.com/actions/create-github-app-token/releases)
  - [Changelog](https://github.com/actions/create-github-app-token/blob/main/CHANGELOG.md)
  - [Commits](https://github.com/actions/create-github-app-token/compare/1b10c78c7865c340bc4f6099eb2f838309f1e8c3...bcd2ba49218906704ab6c1aa796996da409d3eb1)
  ---
  updated-dependencies:
  - dependency-name: actions/create-github-app-token
  dependency-version: 3.2.0
  dependency-type: direct:production
  update-type: version-update:semver-minor
  ...
  Co-authored-by: dependabot[bot] <49699333+dependabot[bot]@users.noreply.github.com>
* 🛠️ Bump actions/create-github-app-token from 3.0.0 to 3.1.1 (`#90 <https://github.com/fujitatomoya/ros2ai/issues/90>`_)
  Bumps [actions/create-github-app-token](https://github.com/actions/create-github-app-token) from 3.0.0 to 3.1.1.
  - [Release notes](https://github.com/actions/create-github-app-token/releases)
  - [Commits](https://github.com/actions/create-github-app-token/compare/f8d387b68d61c58ab83c6c016672934102569859...1b10c78c7865c340bc4f6099eb2f838309f1e8c3)
  ---
  updated-dependencies:
  - dependency-name: actions/create-github-app-token
  dependency-version: 3.1.1
  dependency-type: direct:production
  update-type: version-update:semver-minor
  ...
  Co-authored-by: dependabot[bot] <49699333+dependabot[bot]@users.noreply.github.com>
  Co-authored-by: Tomoya Fujita <tomoya.fujita825@gmail.com>
* 🛠️ Bump actions/github-script from 8.0.0 to 9.0.0 (`#89 <https://github.com/fujitatomoya/ros2ai/issues/89>`_)
  Bumps [actions/github-script](https://github.com/actions/github-script) from 8.0.0 to 9.0.0.
  - [Release notes](https://github.com/actions/github-script/releases)
  - [Commits](https://github.com/actions/github-script/compare/ed597411d8f924073f98dfc5c65a23a2325f34cd...3a2844b7e9c422d3c10d287c895573f7108da1b3)
  ---
  updated-dependencies:
  - dependency-name: actions/github-script
  dependency-version: 9.0.0
  dependency-type: direct:production
  update-type: version-update:semver-major
  ...
  Co-authored-by: dependabot[bot] <49699333+dependabot[bot]@users.noreply.github.com>
* 🛠️ Bump actions/create-github-app-token from 2.2.1 to 3.0.0 (`#88 <https://github.com/fujitatomoya/ros2ai/issues/88>`_)
  Bumps [actions/create-github-app-token](https://github.com/actions/create-github-app-token) from 2.2.1 to 3.0.0.
  - [Release notes](https://github.com/actions/create-github-app-token/releases)
  - [Commits](https://github.com/actions/create-github-app-token/compare/29824e69f54612133e76f7eaac726eef6c875baf...f8d387b68d61c58ab83c6c016672934102569859)
  ---
  updated-dependencies:
  - dependency-name: actions/create-github-app-token
  dependency-version: 3.0.0
  dependency-type: direct:production
  update-type: version-update:semver-major
  ...
  Co-authored-by: dependabot[bot] <49699333+dependabot[bot]@users.noreply.github.com>
* 🛠️ Bump actions/checkout from 4.3.1 to 6.0.2 (`#87 <https://github.com/fujitatomoya/ros2ai/issues/87>`_)
  Bumps [actions/checkout](https://github.com/actions/checkout) from 4.3.1 to 6.0.2.
  - [Release notes](https://github.com/actions/checkout/releases)
  - [Commits](https://github.com/actions/checkout/compare/v4.3.1...v6.0.2)
  ---
  updated-dependencies:
  - dependency-name: actions/checkout
  dependency-version: 6.0.2
  dependency-type: direct:production
  update-type: version-update:semver-major
  ...
  Co-authored-by: dependabot[bot] <49699333+dependabot[bot]@users.noreply.github.com>
* 🛠️ Bump actions/github-script from 7.0.1 to 8.0.0 (`#85 <https://github.com/fujitatomoya/ros2ai/issues/85>`_)
  Bumps [actions/github-script](https://github.com/actions/github-script) from 7.0.1 to 8.0.0.
  - [Release notes](https://github.com/actions/github-script/releases)
  - [Commits](https://github.com/actions/github-script/compare/60a0d83039c74a4aee543508d2ffcb1c3799cdea...ed597411d8f924073f98dfc5c65a23a2325f34cd)
  ---
  updated-dependencies:
  - dependency-name: actions/github-script
  dependency-version: 8.0.0
  dependency-type: direct:production
  update-type: version-update:semver-major
  ...
  Co-authored-by: dependabot[bot] <49699333+dependabot[bot]@users.noreply.github.com>
  Co-authored-by: Tomoya Fujita <tomoya.fujita825@gmail.com>
* 🛠️ Bump actions/checkout from 3 to 6 (`#84 <https://github.com/fujitatomoya/ros2ai/issues/84>`_)
  Bumps [actions/checkout](https://github.com/actions/checkout) from 3 to 6.
  - [Release notes](https://github.com/actions/checkout/releases)
  - [Commits](https://github.com/actions/checkout/compare/v3...v6)
  ---
  updated-dependencies:
  - dependency-name: actions/checkout
  dependency-version: '6'
  dependency-type: direct:production
  update-type: version-update:semver-major
  ...
  Co-authored-by: dependabot[bot] <49699333+dependabot[bot]@users.noreply.github.com>
  Co-authored-by: Tomoya Fujita <tomoya.fujita825@gmail.com>
* 🛠️ Bump actions/setup-python from 5 to 6 (`#83 <https://github.com/fujitatomoya/ros2ai/issues/83>`_)
  Bumps [actions/setup-python](https://github.com/actions/setup-python) from 5 to 6.
  - [Release notes](https://github.com/actions/setup-python/releases)
  - [Commits](https://github.com/actions/setup-python/compare/v5...v6)
  ---
  updated-dependencies:
  - dependency-name: actions/setup-python
  dependency-version: '6'
  dependency-type: direct:production
  update-type: version-update:semver-major
  ...
  Co-authored-by: dependabot[bot] <49699333+dependabot[bot]@users.noreply.github.com>
  Co-authored-by: Tomoya Fujita <tomoya.fujita825@gmail.com>
* 🛠️ Bump actions/create-github-app-token from 2.1.1 to 2.2.1 (`#86 <https://github.com/fujitatomoya/ros2ai/issues/86>`_)
  Bumps [actions/create-github-app-token](https://github.com/actions/create-github-app-token) from 2.1.1 to 2.2.1.
  - [Release notes](https://github.com/actions/create-github-app-token/releases)
  - [Commits](https://github.com/actions/create-github-app-token/compare/a8d616148505b5069dccd32f177bb87d7f39123b...29824e69f54612133e76f7eaac726eef6c875baf)
  ---
  updated-dependencies:
  - dependency-name: actions/create-github-app-token
  dependency-version: 2.2.1
  dependency-type: direct:production
  update-type: version-update:semver-minor
  ...
  Co-authored-by: dependabot[bot] <49699333+dependabot[bot]@users.noreply.github.com>
* enable dependabot.yml. (`#82 <https://github.com/fujitatomoya/ros2ai/issues/82>`_)
* add binary package installation.
* Upgrade github action/run-gemini-cli workflows. (`#81 <https://github.com/fujitatomoya/ros2ai/issues/81>`_)
* enable actions/stale to close issues and PRs. (`#80 <https://github.com/fujitatomoya/ros2ai/issues/80>`_)
* Contributors: Tomoya Fujita, Tomoya.Fujita, dependabot[bot]

0.1.3 (2025-12-12)
------------------
* remove Ollama dependency. (`#78 <https://github.com/fujitatomoya/ros2ai/issues/78>`_)
* Correct the name of a dependent package (`#77 <https://github.com/fujitatomoya/ros2ai/issues/77>`_)

0.1.2 (2025-12-08)
------------------
* Use non-root user and group in default for Docker images. (`#75 <https://github.com/fujitatomoya/ros2ai/issues/75>`_)
* update overview slidedeck. (`#73 <https://github.com/fujitatomoya/ros2ai/issues/73>`_)
* make sure only single command starts with "ros2". (`#68 <https://github.com/fujitatomoya/ros2ai/issues/68>`_)

0.1.1 (2025-11-06)
------------------
* Support Kilted Kaiju with docker image. (`#67 <https://github.com/fujitatomoya/ros2ai/issues/67>`_)
* change OPENAI_TEMPERATURE default into 1.0.
* enable gemini-cli github action. (`#64 <https://github.com/fujitatomoya/ros2ai/issues/64>`_)
* support codespell check via github action. (`#61 <https://github.com/fujitatomoya/ros2ai/issues/61>`_)
* add nightly workflow files for each distro. (`#57 <https://github.com/fujitatomoya/ros2ai/issues/57>`_)
* remove Iron Irwini support, which is E.O.L. (`#56 <https://github.com/fujitatomoya/ros2ai/issues/56>`_)
* use 127.0.0.1 instead of localhost that is invalid URL by validator. (`#53 <https://github.com/fujitatomoya/ros2ai/issues/53>`_)
* Fix a typo in package.xml (`#50 <https://github.com/fujitatomoya/ros2ai/issues/50>`_)
* enable blank_issues_enabled.
* update issue templates.
* add issue templates. (`#48 <https://github.com/fujitatomoya/ros2ai/issues/48>`_)
* ROSCon 2024 talk slide (draft) (`#44 <https://github.com/fujitatomoya/ros2ai/issues/44>`_)
* github workflow status shows rolling branch.
* add --net=host option to docker command for Ollama use case. (`#46 <https://github.com/fujitatomoya/ros2ai/issues/46>`_)
* 20240916 overview update (`#43 <https://github.com/fujitatomoya/ros2ai/issues/43>`_)
* pull llama3.1 before example tutorial.
* add mirror-rolling-to-main.yaml.
* add architecture overview.
* Validate the URL before calling OpenAI APIs. (`#42 <https://github.com/fujitatomoya/ros2ai/issues/42>`_)
* Doc update for Ollama Support. (`#40 <https://github.com/fujitatomoya/ros2ai/issues/40>`_)
* remove backticks from the executable command line string.
* adjust some parameters and variables for Ollama OpenAI support (`#38 <https://github.com/fujitatomoya/ros2ai/issues/38>`_)
* add `--dry-run` to the test script.
* Enable the use of self-hosted LLMs (`#36 <https://github.com/fujitatomoya/ros2ai/issues/36>`_)
  Co-authored-by: Tomoya Fujita <Tomoya.Fujita@sony.com>
* joke sub-command is not gonna be developed.
* Jazzy Jalisco support (`#33 <https://github.com/fujitatomoya/ros2ai/issues/33>`_)
* docker command should be checked before docker login.
* some comment fixes with gpt-4o.
* configure AI model with `gpt-4o` by default. (`#32 <https://github.com/fujitatomoya/ros2ai/issues/32>`_)
* add signal handler for SIGINT and SIGTERM. (`#29 <https://github.com/fujitatomoya/ros2ai/issues/29>`_)
* add `--break-system-packages` option to pip install.
* fix openai rosdep (`#27 <https://github.com/fujitatomoya/ros2ai/issues/27>`_)
* minor script bug fix.
* ROS distribution is always set with completion API.
* add ROS Developers POdcast interview.
* fetch ROS_DISTRO env value to set completion request. (`#24 <https://github.com/fujitatomoya/ros2ai/issues/24>`_)
* script bug fix.
* add demo video with docker containers in marp.
* add doc section for docker container support. (`#20 <https://github.com/fujitatomoya/ros2ai/issues/20>`_)
* fujitatomoya/support dockerfiles and test (`#18 <https://github.com/fujitatomoya/ros2ai/issues/18>`_)
* add workflows badges in the top page.
* support github workflows with rolling distro. (`#15 <https://github.com/fujitatomoya/ros2ai/issues/15>`_)
* add overview slide deck on the top README.md
* update package.xml
* support temperature param for chat.completion.
* add html version presentation slide deck.
* update oveview markdown presentation.
* add overview markdown presentation with proposals.
* update README.md
* support execute subcommand. (`#9 <https://github.com/fujitatomoya/ros2ai/issues/9>`_)
* add system attribution for query command.
* change copyright into the correct one.
* remove redundant class just for stream option.
* support max token via command line option.
* change default model into gpt-4 since it has better response time.
* default streaming response.
* support streaming responses. (`#4 <https://github.com/fujitatomoya/ros2ai/issues/4>`_)
* support list option for status subcommand to show models.
* support query subcommand. (`#1 <https://github.com/fujitatomoya/ros2ai/issues/1>`_)
* add add_global_arguments method.
* ros2ai status subcommand support.
* add class OpenAiConfig.
* command stub completed for ai and status command.
* add config methods and constants.
* add mock command line for ros2ci.
* add basic ros2 package structure.
* Initial commit
* Contributors: Tomoya Fujita, Barry Xu, Jeremie Deray
