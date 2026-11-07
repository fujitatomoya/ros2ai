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

from argparse import Namespace

import pytest
from ros2ai.api.config import get_ai_model
from ros2ai.api.config import get_api_key
from ros2ai.api.config import get_endpoint_url
from ros2ai.api.config import get_role_system
from ros2ai.api.config import get_temperature
from ros2ai.api.config import OpenAiConfig
import ros2ai.api.constants as constants


@pytest.fixture
def clean_env(monkeypatch):
    for name in (
        constants.ROS_OPENAI_API_KEY_ENV_VAR,
        constants.ROS_OPENAI_MODEL_NAME_ENV_VAR,
        constants.ROS_OPENAI_ENDPOINT_ENV_VAR,
        constants.ROS_OPENAI_TEMPERATURE_ENV_VAR,
        constants.ROLE_SYSTEM_ENV_VAR,
    ):
        monkeypatch.delenv(name, raising=False)
    return monkeypatch


def global_args(model=None, url=None, token=None):
    return Namespace(model=model, url=url, token=token)


def test_get_api_key(clean_env):
    assert get_api_key() == 'None'
    clean_env.setenv(constants.ROS_OPENAI_API_KEY_ENV_VAR, 'sk-test')
    assert get_api_key() == 'sk-test'


def test_get_ai_model(clean_env):
    assert get_ai_model() == constants.ROS_OPENAI_DEFAULT_MODEL
    clean_env.setenv(constants.ROS_OPENAI_MODEL_NAME_ENV_VAR, 'llama3.1')
    assert get_ai_model() == 'llama3.1'


def test_get_endpoint_url(clean_env, capsys):
    assert get_endpoint_url() == constants.ROS_OPENAI_DEFAULT_ENDPOINT
    clean_env.setenv(constants.ROS_OPENAI_ENDPOINT_ENV_VAR, 'http://127.0.0.1:11434/v1')
    assert get_endpoint_url() == 'http://127.0.0.1:11434/v1'
    clean_env.setenv(constants.ROS_OPENAI_ENDPOINT_ENV_VAR, 'not a url')
    assert get_endpoint_url() == constants.ROS_OPENAI_DEFAULT_ENDPOINT
    assert capsys.readouterr().out == ''


def test_get_temperature(clean_env):
    assert get_temperature() == constants.ROS_OPENAI_DEFAULT_TEMPERATURE
    clean_env.setenv(constants.ROS_OPENAI_TEMPERATURE_ENV_VAR, '0.25')
    assert get_temperature() == 0.25


def test_get_role_system(clean_env):
    assert get_role_system(default_role_system='default role') == 'default role'
    clean_env.setenv(constants.ROLE_SYSTEM_ENV_VAR, 'env role')
    assert get_role_system(default_role_system='default role') == 'env role'


def test_config_defaults(clean_env):
    config = OpenAiConfig(global_args())
    assert config.get_value('api_key') == 'None'
    assert config.get_value('api_model') == constants.ROS_OPENAI_DEFAULT_MODEL
    assert config.get_value('api_endpoint') == constants.ROS_OPENAI_DEFAULT_ENDPOINT
    assert config.get_value('api_token') is None
    assert config.get_value('api_temperature') == constants.ROS_OPENAI_DEFAULT_TEMPERATURE
    assert config.get_value('unknown') is None


def test_config_arguments_prevail(clean_env):
    clean_env.setenv(constants.ROS_OPENAI_API_KEY_ENV_VAR, 'sk-test')
    clean_env.setenv(constants.ROS_OPENAI_MODEL_NAME_ENV_VAR, 'env-model')
    clean_env.setenv(constants.ROS_OPENAI_ENDPOINT_ENV_VAR, 'http://127.0.0.1:11434/v1')
    clean_env.setenv(constants.ROS_OPENAI_TEMPERATURE_ENV_VAR, '0.5')
    config = OpenAiConfig(global_args(model='arg-model', url='http://10.0.0.1:8080/v1', token=42))
    assert config.get_value('api_key') == 'sk-test'
    assert config.get_value('api_model') == 'arg-model'
    assert config.get_value('api_endpoint') == 'http://10.0.0.1:8080/v1'
    assert config.get_value('api_token') == 42
    assert config.get_value('api_temperature') == 0.5


def test_config_display_all_hides_api_key(clean_env, capsys):
    clean_env.setenv(constants.ROS_OPENAI_API_KEY_ENV_VAR, 'sk-secret')
    OpenAiConfig(global_args(model='gpt-test', token=10)).display_all()
    out = capsys.readouterr().out
    assert 'sk-secret' not in out
    assert out == (
        '----- api_model: gpt-test\n'
        f'----- api_endpoint: {constants.ROS_OPENAI_DEFAULT_ENDPOINT}\n'
        '----- api_token: 10\n'
        '----- api_temperature: 1.0\n'
    )
