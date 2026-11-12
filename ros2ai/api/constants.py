# Copyright 2023 Tomoya Fujita <tomoya.fujita825@gmail.com>.
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

# ros2ai configuration is agnostic from the LLM managed service, any endpoint that provides
# the OpenAI compatible API can be configured with the following environment variables.
# The API key is only required when accessing the managed services such as OpenAI or Anthropic.
ROS2AI_API_KEY_ENV_VAR = 'ROS2AI_API_KEY'
ROS2AI_MODEL_NAME_ENV_VAR = 'ROS2AI_MODEL_NAME'
ROS2AI_ENDPOINT_ENV_VAR = 'ROS2AI_ENDPOINT'
ROS2AI_TEMPERATURE_ENV_VAR = 'ROS2AI_TEMPERATURE'
ROS2AI_ROLE_SYSTEM_ENV_VAR = 'ROS2AI_ROLE_SYSTEM'
ROS2AI_CONTEXT_LIMIT_ENV_VAR = 'ROS2AI_CONTEXT_LIMIT'
ROS2AI_SESSION_FILE_ENV_VAR = 'ROS2AI_SESSION_FILE'

# Legacy environment variables, still honored when the ROS2AI_* variables are not set.
ROS_OPENAI_API_KEY_ENV_VAR = 'OPENAI_API_KEY'
ROS_OPENAI_MODEL_NAME_ENV_VAR = 'OPENAI_MODEL_NAME'
ROS_OPENAI_ENDPOINT_ENV_VAR = 'OPENAI_ENDPOINT'
ROS_OPENAI_TEMPERATURE_ENV_VAR = 'OPENAI_TEMPERATURE'
ROLE_SYSTEM_ENV_VAR = 'OPENAI_ROLE_SYSTEM'

# Known LLM managed services and their defaults.
ROS_OPENAI_DEFAULT_MODEL = 'gpt-5'
ROS_OPENAI_DEFAULT_ENDPOINT = 'https://api.openai.com/v1'
ROS_ANTHROPIC_DEFAULT_MODEL = 'claude-opus-5-5'
ROS_ANTHROPIC_DEFAULT_ENDPOINT = 'https://api.anthropic.com/v1/'
ROS_OLLAMA_DEFAULT_MODEL = 'llama3.1'
ROS_OLLAMA_DEFAULT_ENDPOINT = 'http://127.0.0.1:11434/v1'

# Anthropic validates the API key with `x-api-key` header on the native endpoints (e.g /v1/models),
# this header is added when the configured endpoint is the Anthropic API.
ROS_ANTHROPIC_API_HOST = 'api.anthropic.com'
ROS_ANTHROPIC_API_VERSION = '2023-06-01'

# OpenAI is the default service when nothing is configured.
ROS2AI_DEFAULT_MODEL = ROS_OPENAI_DEFAULT_MODEL
ROS2AI_DEFAULT_ENDPOINT = ROS_OPENAI_DEFAULT_ENDPOINT

# The system message helps set the behavior of the assistant.
# For example, you can modify the personality of the assistant or provide specific
# instructions about how it should behave throughout the conversation.
# However note that the system message is optional and the model's behavior without a system
# message is likely to be similar to using a generic message such as "You are a helpful assistant."
# `{}` is replaced with the ROS distribution name.
ROLE_SYSTEM_DEFAULT = \
    'You are a Robot Operating System 2 (as known as ROS 2) {0} distribution ' \
    'professional assistant who can provide helpful answers against any questions. ' \
    'When the user asks to inspect or operate the ROS 2 system, use the provided tools: ' \
    'inspect the system with read-only tools before changing anything, ' \
    'use exactly the names reported by the tools, ' \
    'and finish with a brief summary of the result. ' \
    'When the user asks about ROS 2 concepts, tutorials, tools or APIs, ' \
    'look up the official {0} documentation with the documentation tools ' \
    'and cite the page URL in the answer. ' \
    'If no tools are provided, explain the commands that the user can run instead.'

# Tool calling (function calling) limits for the agent loop.
# Maximum number of chat completions (steps) for a single request.
ROS2AI_MAX_STEPS = 10
# Maximum number of characters of a tool output returned to the model, the head and the tail
# are kept. This is kept small so that the conversation fits into the small context window
# of the local models (e.g Ollama default), the whole output is always printed to the user.
ROS2AI_TOOL_OUTPUT_LIMIT = 4000
# Timeout in seconds for a tool command, and for receiving a single message.
ROS2AI_TOOL_TIMEOUT_SEC = 30.0
ROS2AI_TOOL_ECHO_TIMEOUT_SEC = 10.0

# Composite tools (e.g `get_node_status`) run several `ros2` commands for a single tool call.
# Timeout in seconds for each command that queries a node (e.g `ros2 lifecycle get`).
ROS2AI_COMPOSITE_STEP_TIMEOUT_SEC = 10.0
# Seconds to measure the publishing rates (`ros2 topic hz`) and to receive a sample message,
# the default and the maximum the model can request. `ros2 topic hz` needs a couple of messages
# before printing the rate, so a 1 Hz topic needs a few seconds.
ROS2AI_RATE_WINDOW_SEC = 5
ROS2AI_RATE_WINDOW_MAX_SEC = 10
# Maximum number of topics whose publishing rates are measured in parallel.
ROS2AI_RATE_TOPICS_LIMIT = 8
# Maximum number of items (parameters, nodes, topics) in a list of the result.
ROS2AI_LIST_LIMIT = 30
# Maximum number of lines of a sample message in the result.
ROS2AI_SAMPLE_LINES_LIMIT = 20
# Sampled commands are polled with this period and get this grace period after the interrupt.
ROS2AI_SAMPLE_POLL_SEC = 0.1
ROS2AI_SAMPLE_GRACE_SEC = 5.0

# Documentation tools (`search_ros2_docs`, `read_ros2_docs`) look up the ROS 2 documentation
# of the distribution, the reStructuredText sources of the distribution branch of the
# documentation repository are cached locally and refreshed when older than the period.
ROS2AI_DOCS_CACHE_ENV_VAR = 'ROS2AI_DOCS_CACHE'
ROS2AI_DOCS_CACHE_DEFAULT = '~/.cache/ros2ai/docs'
ROS2AI_DOCS_ARCHIVE_URL = 'https://github.com/ros2/ros2_documentation/archive/refs/heads/{}.tar.gz'
ROS2AI_DOCS_SITE_URL = 'https://docs.ros.org/en/{}/'
ROS2AI_DOCS_REFRESH_SEC = 7 * 24 * 60 * 60
ROS2AI_DOCS_DOWNLOAD_TIMEOUT_SEC = 120.0
# Number of pages returned by a search, the default and the maximum the model can request.
# The default is kept small so that the conversation fits into the small context window of
# the local models, the model asks for more when needed.
ROS2AI_DOCS_SEARCH_LIMIT = 3
ROS2AI_DOCS_SEARCH_LIMIT_MAX = 10
# Characters of the text around the first match returned as the snippet of a search result.
ROS2AI_DOCS_SNIPPET_CHARS = 160
# Characters of a page returned by a single read, the model reads the rest with the offset.
# This fits with the header within the tool output limit (`ROS2AI_TOOL_OUTPUT_LIMIT`).
ROS2AI_DOCS_PAGE_CHARS = 3500
# Full names of the distributions, used to expand the placeholders of the documentation.
ROS_DISTRO_FULL_NAMES = {
    'humble': 'Humble Hawksbill',
    'iron': 'Iron Irwini',
    'jazzy': 'Jazzy Jalisco',
    'kilted': 'Kilted Kaiju',
    'lyrical': 'Lyrical Luth',
    'rolling': 'Rolling Ridley',
}
# Session keeps the context of the previous requests, the messages of the previous turns are
# sent with every request. The oldest turns are dropped when the estimated number of tokens
# of the messages exceeds this limit (the tool definitions are not counted), so that the
# conversation fits into the context window of the model and the token consumption is bounded.
# Local models may need a smaller limit depending on their context window (e.g Ollama `num_ctx`).
ROS2AI_CONTEXT_LIMIT = 16000
# The tokenizer differs per model, the number of tokens is estimated from the characters.
ROS2AI_CHARS_PER_TOKEN = 4
# The session is saved to `$XDG_STATE_HOME/ros2ai/<file>` (`~/.local/state` if not set),
# `ROS2AI_SESSION_FILE` prevails. `--continue` continues the saved session.
ROS2AI_SESSION_FILE_NAME = 'session.json'
# Prompt of the interactive session.
ROS2AI_PROMPT = 'ros2ai> '

# Temperature controls the consistency for behavior.
# (range 0.0 - 2.0 for OpenAI, 0.0 - 1.0 for Anthropic)
# The lower the temperature is, the more deterministic behavior that LLM does.
# The higher temperature is, the more creative LLM can be.
# Temperature value based on the desired trade-off between coherence and creativity.
ROS2AI_DEFAULT_TEMPERATURE = 1.0
