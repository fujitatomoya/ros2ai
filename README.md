[![humble](https://github.com/fujitatomoya/ros2ai/actions/workflows/humble.yml/badge.svg)](https://github.com/fujitatomoya/ros2ai/actions/workflows/humble.yml) [![iron](https://github.com/fujitatomoya/ros2ai/actions/workflows/iron.yml/badge.svg)](https://github.com/fujitatomoya/ros2ai/actions/workflows/iron.yml) [![rolling](https://github.com/fujitatomoya/ros2ai/actions/workflows/rolling.yml/badge.svg)](https://github.com/fujitatomoya/ros2ai/actions/workflows/rolling.yml)

# ros2ai 🤖

[ros2ai](https://github.com/fujitatomoya/ros2ai) is a <span style="color:red">next-generation</span> [ROS 2](https://github.com/ros2) command line interface extension with [OpenAI](https://openai.com/)

see [overview slide deck](https://raw.githack.com/fujitatomoya/ros2ai/rolling/doc/overview.html) for more information.

## Motivation

- (Just for fun 😝)
- Getting answers against the questions directly without browsing, clicking and typing many times.
- Easy to use for everyone, especially for [ROS 2](https://github.com/ros2) beginners and students who do not really know [ros2cli](https://github.com/ros2/ros2cli).
- Multiple language support.

## Demo 🖥️

See how it works 🔥

https://github.com/fujitatomoya/ros2ai/assets/43395114/78a0799b-40e3-4dc8-99cb-488994e94769

## Supported [ROS Distribution](https://docs.ros.org/en/rolling/Releases.html)

| Distribution      | Supported | Note |
| :---------------- | :-------- | :--- |
| Rolling Ridley    |    ✅     | Development / Mainstream Branch |
| Iron Irwini       |    ⛔     | W.I.P (No distro specific dependency, should work.) |
| Humble Hawksbill  |    ⛔     | W.I.P (No distro specific dependency, should work.) |

> [!NOTE]
> Verified on [Ubuntu 22.04 Jammy Jellyfish](https://releases.ubuntu.com/jammy/) only, other platform would also work.

## Installation

### Required Package

```bash
pip install openai
```

### Build

No released package is available, needs to be build in colcon workspace.

```bash
source /opt/ros/rolling/setup.bash
mkdir -p colcon_ws/src
cd colcon_ws/src
git clone https://github.com/fujitatomoya/ros2ai.git
cd ..
colcon build --symlink-install --packages-select ros2ai
```

## Usage

### Prerequisites

- `ros2ai` requires [OpenAI API key](https://platform.openai.com/docs/overview)

```bash
export OPENAI_API_KEY='your-api-key-here'
```

> [!CAUTION]
> Do not share or expose your OpenAI API key.

#### Optional

| environmental variable | default                     | Note                   |
| :----------------------| :-------------------------- | :--------------------- |
| OPENAI_MODEL_NAME      | 'gpt-4'                     | AI model to be used.   |
| OPENAI_ENDPOINT        | 'https://api.openai.com/v1' | API endpoint URL.      |
| OPENAI_TEMPERATURE     | 0.5                         | [OpenAI temperature](https://platform.openai.com/docs/guides/text-generation/how-should-i-set-the-temperature-parameter) |

> [!NOTE]
> These are optional environmental variables. if not set, default value will be used.

### Examples

#### Basics

- `status` to check OpenAI API key is valid.

```bash
root@tomoyafujita:~/docker_ws/ros2_colcon# ros2 ai status -v
----- api_model: gpt-4
----- api_endpoint: https://api.openai.com/v1
----- api_token: None
As an artificial intelligence, I do not have a physical presence, so I can't be "in service" in the traditional sense. But I am available to assist you 24/7.
[SUCCESS] Valid OpenAI API key.
```

- `query` to ask any questions to ROS 2 assistant.

```bash
root@tomoyafujita:~/docker_ws/ros2_colcon# ros2 ai query "Tell me how to check the available topics?"
To check the available topics in ROS 2, you can use the following command in the terminal:

---
ros2 topic list
---

After you enter this command, a list of all currently active topics in your ROS2 system will be displayed. This list includes all topics that nodes in your system are currently publishing to or subscribing from.
```

- `exec` that ROS 2 assistant can execute the appropriate command based on your request.

```bash
root@tomoyafujita:~/docker_ws/ros2_colcon# ros2 ai exec "give me all nodes"
/talker
root@tomoyafujita:~/docker_ws/ros2_colcon# ros2 ai exec "what topics available"
/chatter
/parameter_events
/rosout
root@tomoyafujita:~/docker_ws/ros2_colcon# ros2 ai exec "give me detailed info for topic /chatter"
Type: std_msgs/msg/String
Publisher count: 1
Subscription count: 0
```

#### Multiple Language

- Japanese (could be any language ❓)

```bash
root@tomoyafujita:~/docker_ws/ros2_colcon# ros2 ai query "パラメータのリスト取得方法を教えて"
ROS 2のパラメータリストを取得するには、コマンドラインインターフェース(CLI)を使います。具体的には、次のコマンドを使用します：

---code
ros2 param list
---

このコマンドは、現在動作しているすべてのノードのパラメーターをリストアップします。特定のノードのパラメータだけを見たい場合には、以下のようにノード名を指定することもできます。

---code
ros2 param list /node_name
---

このようにして、ROS2のパラメータリストの取得を行うことが可能です。なお、上述したコマンドはシェルから直接実行してください。
```

## Reference

- [OpenAI API documentation](https://platform.openai.com/docs)
- [OpenAI Python API](https://github.com/openai/openai-python)

Special thanks to [OpenAI API](https://platform.openai.com/) 🌟🌟🌟
