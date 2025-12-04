---
marp: true
theme: uncover # gaia, uncover, default
header: "***__ ros2ai Next-Gen ROS 2 CLI __***"
footer: "by ***[fujitatomoya@github](https://github.com/fujitatomoya)*** / ***[tomoyafujita@linkedin](https://www.linkedin.com/in/tomoya-fujita-5bb656b6/)***"
_backgroundColor: white
page_number: true
---

# ***ros2ai Next-Gen ROS 2 CLI***

[ros2ai](https://github.com/fujitatomoya/ros2ai) is a <span style="color:red">next-generation</span> [ROS 2](https://github.com/ros2) command line interface extension with [OpenAI](https://openai.com/) and [Ollama](https://github.com/ollama/ollama)

<!---
# Comment Here
--->

---

# [Everything Here!!!](https://github.com/fujitatomoya/ros2ai)

<span style="color:red;">***Issues/PRs Welcome***</span>

![bg right:50% 50%](./images/qrcode_image.png)

<!---
# Comment Here
--->

---

<video controls="controls" width="1000" src="https://github.com/user-attachments/assets/a5d4dda6-cd6b-41d1-8038-40e4906082b0">

<!---
# This demo only includes status check and query command.
--->

---

<video controls="controls" width="1000" src="https://github.com/user-attachments/assets/6b6d8038-6d5a-4aaf-aace-bb9af4995145">

<!---
# Execution Demo only with multiple language.
--->

---

<video controls="controls" width="1000" src="https://github.com/fujitatomoya/ros2ai/assets/43395114/2af4fd44-2ccf-472c-9153-c3c19987dc96">

<!---
# Docker container demonstration
--->

---

# Motivation

- (Originally just for fun 😂)
- Quickly get the answers against questions.
- Multiple Language Support.
- Support beginners and students. (your best trainer)
- Easy to use for everyone.
- Dedicated / AI based Support (your own concierge)
- Bridge / Proxy to [LLM](https://en.wikipedia.org/wiki/Large_language_model)

<!---
# Comment Here
--->

---

# Background

## I want the answers w/o searching...

browsing, clicking and typing many times to get to the information does not work for me. I need ***what***, not ***where***. all these are small things, but can be easily compiled up to mountain, especially beginners.

<!---
# Comment Here
--->

---

## I really do not care how to use ros2cli...

many sub-commands, options, and arguments. besides, those could be deprecated or removed time to time... this does not work for me. actually i just want to ask "what parameters are available?", "check the details for the topic /chatter", but before that i need to know how to do that... this is already barrier for beginners.

it would be better for maintenance if someone abstracts this to the user, so that maintainers can just go ahead to remove, change options w/o deprecation or certain soak time...

<!---
# Comment Here
--->

---

## Multi-Language Documentation

OSS projects tend to add multi-language support documentation under mainline doc. this is really likely, and they do. but the question is ***Does it really work?***... IMO, it does (will) not. eventually those docs will be outdated, not maintained, not scalable, not precise and once that happens these docs would be garbage.

i believe that is exactly where AI comes to play. instead of paying the resources to non-scalable things, focus on the mainline with precise information which is available for any languages.

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

# Future Plans

<!---
# These are just ideas, not even sure that is doable.
--->

---

# Model Comparison

- Experimental performance and accuracy comparison over different LLM models.

<!---
# Comment Here
--->

---

# API Parameter Adjustment

- for more stable AI behavior.
- performance and accuracy compariwon.
- latency for user experience.

<!---
# Comment Here
--->

---

# Context-Awareness

currently, `ros2ai` only supports single-shot completion API, that means we can not rely on the previous questions or answers to make further requests. e.g) "Is /chatter topic available?", "Subscribe it!". This ***it*** should be recognized in the same context or session. If this command is issued, prompt should be initiated to keep the session alive until the entire session is closed by the user.

<!---
# probably with LangChain::ConversationBufferMemory
--->

---

# Daemonized Agent

`ros2ai` can instantiate the agent daemon process as ROS 2 service, so that even other ROS 2 application running in the network can use OpenAI API via this proxy. those ROS 2 application can be agnostic from LLM specific APIs but ROS 2 service interface.

<!---
# Comment Here
--->

---

# Audio Interface Support

It would be really helpful to iterate via audio system instead of typing anymore.
So the user can just talk to the `ros2ai` agent and then `ros2ai` answers any questions and executes the corresponding operations as user wants to do.

<!---
# Comment Here
--->

---

# Function Calling

for more user friendly experience, user should not be aware of `ros2ai` sub-command at all such as query, execute. this is actually against the design policy for `ros2ai`.
we could take advantage of [Function Calling](https://platform.openai.com/docs/guides/function-calling) to conceal these sub-commands, and internally categorize the request based on the user input.

<!---
# Comment Here
--->

---

# Fine-tuning

- ROS 2 general fine-tuning
  - this requires scaled training dataset for ROS 2, agnostic from user environment. could be distro specific. so that AI can response more precisely based on questions and requests.
- User specified environment tuning
  - this dataset should be uploaded to help more user specific problems and questions. if this is doable, `ros2ai` agent can the 1st customer support for anyone?

<!---
# what we need to do is automate this fune-tuning process based on user's environment including distro.
# this seems like a mandatory, because user's environment is really different one to another.
# data set creation must be done by eash user, and automated by script or some tools.
--->
