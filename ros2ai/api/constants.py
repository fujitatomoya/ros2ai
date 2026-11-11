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
# However note that the system message is optional and the model’s behavior without a system
# message is likely to be similar to using a generic message such as "You are a helpful assistant."
# TODO@fujitatomoya: ROS_DISTRO would be better to assistant setting.
ROLE_SYSTEM_QUERY_DEFAULT = \
    'You are a Robot Operating System version 2 (as known as ROS2) {} distribution ' \
    'professional assistant who can provide helpful answers against any questions.'
ROLE_SYSTEM_EXEC_DEFAULT = \
    'You are a Robot Operating System 2 (as known as ROS 2) {} distribution operator. ' \
    'Use the provided tools to inspect and operate the ROS 2 system on behalf of the user. ' \
    'Inspect the system with read-only tools before changing anything, ' \
    'use exactly the names reported by the tools, ' \
    'and finish with a brief summary of the result.'
# Used when the endpoint or model does not support tool calling (function calling),
# the model is asked for a single `ros2` command line instead.
ROLE_SYSTEM_EXEC_COMMAND = \
    'You are a Robot Operating System 2 (as known as ROS 2) {} distribution ' \
    'command line executor, provides single executable command string only ' \
    'without any comments or code blocks or backticks.'

# Tool calling (function calling) limits for the `exec` agent loop.
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

# Temperature controls the consistency for behavior.
# (range 0.0 - 2.0 for OpenAI, 0.0 - 1.0 for Anthropic)
# The lower the temperature is, the more deterministic behavior that LLM does.
# The higher temperature is, the more creative LLM can be.
# Temperature value based on the desired trade-off between coherence and creativity.
ROS2AI_DEFAULT_TEMPERATURE = 1.0
