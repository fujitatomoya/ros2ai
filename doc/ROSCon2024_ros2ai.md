---
marp: true
theme: default # gaia, uncover, default
#class: invert
header: "***__ ros2ai Next-Gen ROS 2 CLI __***"
footer: "[ROSCon 2024](https://roscon.ros.org/2024/) by ***[fujitatomoya@github](https://github.com/fujitatomoya)*** / ***[tomoyafujita@linkedin](https://www.linkedin.com/in/tomoya-fujita-5bb656b6/)***"
_backgroundColor: white
page_number: true
---

# ***ros2ai Next-Gen ROS 2 CLI***

**[ros2ai](https://github.com/fujitatomoya/ros2ai)** is a <span style="color:red">**next-generation**</span> **[ROS 2](https://github.com/ros2)** command line interface with **[OpenAI](https://openai.com/)** and **[Ollama](https://github.com/ollama/ollama)**

<!---
# Comment Here
--->

---

# [Picture me for more information!!!](https://github.com/fujitatomoya/ros2ai)

<span style="color:red;">***Fully Open Sourced, Issues/PRs Always Welcome***</span>

![bg right:40% 50%](./images/qrcode_image.png)

<!---
# Comment Here
--->

---

# Demo

## **Talking is Easy, Show me the Code...** **<span style="color:red;">Seeing is More Believing</span>**

> Demo can be found on https://github.com/fujitatomoya/ros2ai

<!---
# Maybe question for audience if they have used ros2cli before.
# If they have, they are in the right place.
--->

---

# Motivation

- (Originally just for fun 😂)
- Quickly get the answers against questions.
- Multiple Language Support.
- Support beginners and students. (your best trainer)
- Easy to use for everyone.
- Bridge / Proxy to [LLM](https://en.wikipedia.org/wiki/Large_language_model)

<!---
# Comment Here
--->

---

# Background

## We want the answers w/o searching...

browsing, clicking and typing many times to get to the information does not work for us. We need ***what***, not ***where***. all these are small things, but can be easily compiled up to mountain, especially beginners.

<!---
# Comment Here
--->

---

## We really do not care how to use ros2cli...

many sub-commands, options, and arguments. besides, those could be deprecated or removed time to time... this does not work for us. actually we just want to ask "what parameters are available?", "check the details for the topic /chatter", but before that we need to know how to do that... this is already barrier for beginners.

<!---
# Comment Here
--->

---

## Multi-Language Documentation

OSS projects likely have multi-language support documentation under mainline doc. this is really likely, and they do. but the question is ***Does it really work?***... probably it does (will) not. eventually those docs will be outdated, not maintained, not scalable, not precise and once that happens these docs would be garbage.

that is exactly where AI comes to play. instead of paying the resources to non-scalable things, focus on the mainline with precise information which is available for any languages.

<!---
# Comment Here
--->

---

# Design

- <span style="color:red;">***SIMPLE***</span>. one of the original motivation, it has to be simple as much as possible. That is said that it would be even better to remove all sub-commands, just `ros2 ai <whatever your request>` if that is possible.

<!---
# Comment Here
--->

---

![bg 90%](./images/ros2ai_overview.png)

<!---
# Comment Here
--->

---

- `status` command is to check if `ros2ai` is configured properly.
- `query` command is to ask any questions related to ROS 2. This is a single-shot completion, no session is supported currently. OpenAI system role attribution is set to default, but can be reconfigurable.
- `exec` command is that AI executes the appropriate command based on the requests. OpenAI system role attribution is set to default, but can be reconfigurable.

---

# Proposals / What's more coming?

<!---
# These are just ideas, not even sure that is doable.
--->

---

# Parameter Adjustment

- more stable AI behavior.
- Latency for user experience.
- LLM Models.

---

# Session Mode

currently, `ros2ai` only supports single-shot completion API, that means we can not rely on the previous questions or answers to make further requests. e.g) "Is /chatter topic available?", "Subscribe it!". This ***it*** should be recognized in the same context or session. If this command is issued, prompt should be initiated to keep the session alive until the entire session is closed by the user.

<!---
# Comment Here
--->

---

# Proxy Agent

`ros2ai` can instantiate the agent daemon process as ROS 2 service, so that even other ROS 2 application running in the network can use OpenAI API via this proxy. those ROS 2 application can be agnostic from LLM specific APIs but ROS 2 service interface.

---

# Function Calling

for more user friendly experience, user should not be aware of `ros2ai` sub-command at all such as query, execute. this is actually against the design policy for `ros2ai`.
we could take advantage of [Function Calling](https://platform.openai.com/docs/guides/function-calling) to conceal these sub-commands, and internally categorize the request based on the user input.

<!---
# Comment Here
--->

---

# Fine-tuning (T.B.D)

- ROS 2 general fine-tuning
  - this requires scaled training dataset for ROS 2, agnostic from user environment. could be distro specific. so that AI can response more precisely based on questions and requests.
- User specified environment tuning
  - this dataset should be uploaded to help more user specific problems and questions. if this is doable, `ros2ai` agent can the 1st customer support for anyone?

<!---
# I am not even sure what could be done with it. need to come back here to consider the possibility.
--->

---

# Questions?

![bg right:40% 50%](./images/qrcode_image.png)
