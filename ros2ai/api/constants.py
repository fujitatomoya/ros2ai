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

# This is only required for OpenAI API call
# If this is set, it falls back to call OpenAI API gracefully.
ROS_OPENAI_API_KEY_ENV_VAR = 'OPENAI_API_KEY'

ROS_OPENAI_DEFAULT_MODEL = 'gpt-4o'
ROS_OLLAMA_DEFAULT_MODEL = 'llama3.1'

ROS_AI_MODEL_NAME_ENV_VAR = 'AI_MODEL_NAME'
ROS_AI_ENDPOINT_ENV_VAR = 'AI_ENDPOINT'

ROS_OPENAI_DEFAULT_ENDPOINT = 'https://api.openai.com/v1'
ROS_OLLAMA_DEFAULT_ENDPOINT = 'http://localhost:11434'
ROS_OLLAMA_OPENAI_ENDPOINT = 'http://localhost:11434/v1'

# The system message helps set the behavior of the assistant.
# For example, you can modify the personality of the assistant or provide specific
# instructions about how it should behave throughout the conversation.
# However note that the system message is optional and the model’s behavior without a system message
# is likely to be similar to using a generic message such as "You are a helpful assistant."
# TODO@fujitatomoya: ROS_DISTRO would be better to assistant setting.
ROLE_SYSTEM_QUERY_DEFAULT = \
    'You are a Robot Operating System version 2 (as known as ROS2) {} distribution ' \
    'professional assistant who can provide helpful answers against any questions.'
ROLE_SYSTEM_EXEC_DEFAULT = \
    'You are a Robot Operating System 2 (as known as ROS2) {} distribution command line executor, ' \
    'provides executable command string only without any comments or code blocks.'
ROLE_SYSTEM_ENV_VAR = 'AI_ROLE_SYSTEM'

# Temperature controls the consistency for behavior. (range 0.0 - 2.0)
# The lower the temperature is, the more deterministic behavior that AI does.
# The higher temperature is, the more creative AI can be.
# Temperature value based on the desired trade-off between coherence and creativity.
ROS_AI_DEFAULT_TEMPERATURE = 0.5
ROS_AI_TEMPERATURE_ENV_VAR = 'AI_TEMPERATURE'
