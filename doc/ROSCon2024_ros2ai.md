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
# Questions for audience like
# - if they have used ros2cli before?
# - if they have met the problem to figure out sub-command, parameter how to use those?
# If they have, they are in the right place.
#
# ros2ai is easy to use with container, just type `docker run` is all you need to do.
# 
# Demo:
# - checking the status with openAI, `ros2 ai status -lv` to see API works okay and all the available models
# - 1st demo is query to ask the question about ROS 2, ros2ai can answer your question about ROS 2 based on your current distribution environment.
# - `ros2 ai query "what is the qos durability?"`
# - `ros2 ai query "how can we create the package"`
# - next is to ask AI to execute the appropriate command based on your request. ros2ai can execute the command behalf of you, and give the response back to you. that means you can iterate with ROS 2 system without figuring out the command line tools.
# - `ros2 ai exec "give me all the topic"` and we can check the details, `ros2 ai exec "detailed info about topic /rosout"`
# - it works with any languages, `ros2 ai exec "パラメータリストをください"`
# Demo shows us what we can do with ros2ai!
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
# Just introduce the motivation quickly with adding extra information.
# We are gonna talk about these with the following slides as a background including feature plan.
--->

---

# Background

## We want the answers w/o searching...

browsing, clicking and typing many times to get to the information does not work for us. We need ***what***, not ***where***. all these are small things, but can be easily compiled up to mountain, especially beginners.

<!---
# this is what we do, but we do not want to if not necessary, so lets not do that.
# giving the example about QoS.
# before we get to know the QoS, we need to google, check the list, find appropriate site, and finally we still need to find the contents from ros2 documentation.
# this is expensive especially for beginners... nobody wants to do this.
--->

---

## We really do not care how to use ros2cli...

many sub-commands, options, and arguments. besides, those could be deprecated or removed time to time... this does not work for us. actually we just want to ask "what parameters are available?", "check the details for the topic /chatter", but before that we need to know how to do that... this is already barrier for beginners.

<!---
# might sound like a strong opinion, but this is true.
# we obviously do not want to remember or know the every single command line interfaces, options and parameters.
# this is not only for the ROS 2 CLI but any systems.
# if would be really easier for us that someone interprets the communication between us and system.
# this would be great for maintainers, deprecated commands and options can be concealed by ros2ai.
--->

---

## Multi-Language Documentation

OSS projects likely have multi-language support documentation under mainline doc. this is really likely, and they do. but the question is ***Does it really work?***... probably it does (will) not. eventually those docs will be outdated, not maintained, not scalable, not precise and once that happens these docs would be garbage.

that is exactly where AI comes to play. instead of paying the resources to non-scalable things, focus on the mainline with precise information which is available for any languages.

<!---
# there has been several discussion brought up that we should support multi-language documentation.
# most OSS community does if the number of users are large such as CNCF projects for example.
# but, with my personal experience, every time i visit the multi-language documentation, in my case that is Japanese, there is a warning on the top saying "This is not up-to-dated, please visit English page." or something like that.
# this does not work at all, this system is already broken in the 1st place...
# instead of having the dedicated human resource to support this, my opinion is to let the AI work as we ask in their language based on the central precise information from the mainline.
# so that we can focus and pay attention on the mainline documentation with precise and concise information with quality.
--->

---

# Design

- <span style="color:red;">***SIMPLE***</span>. one of the original motivation, it has to be simple as much as possible. That is said that it would be even better to remove all sub-commands, just `ros2 ai <whatever your request>` if that is possible.

<!---
# my focus is to keep it simple.
# implementation is really straight-forward using OpenAI Python API and ros2cli interfaces.
# unfortunately current ros2ai has sub-commands such as query and exec, but these need to be removed in the future.
# expecting this can be done with Function Calling feature that we are going to talk about in this presentation.
# i believe the most important thing is to keep is simple so that user can use without even thinking. 
--->

---

![bg 90%](./images/ros2ai_overview.png)

<!---
# ros2ai sits in between ROS 2 environment and OpenAI Python API like this.
# OpenAI Python API is compatible with Ollama, so that ros2ai can be agnostic from AI service via interface.
# you can configure backend AI service endpoint with using environmental variables.
# so every time you type the command in ros2ai, that is gonna process the request with OpenAI or Ollama, and iterate with ROS 2 system then comes back to you with result.
--->

---

- `status` command is to check if `ros2ai` is configured properly.
- `query` command is to ask any questions related to ROS 2. This is a single-shot completion, no session is supported currently. OpenAI system role attribution is set to default, but can be reconfigurable.
- `exec` command is that AI executes the appropriate command based on the requests. OpenAI system role attribution is set to default, but can be reconfigurable.

<!---
# As i introduced in the demonstration, we have now three sub-commands that are `status`, `query` and `exec`.
# if you query or execute the ros2ai, ros2ai fetches your ROS 2 environment setting such as what kind of distribution that you are using here, then integrate those information to the request completion field to AI backend service.
# that said, you can have the answer based on your local ROS 2 environment setting.
--->

---

# Proposals / What's more coming?

<!---
# Since ros2ai is new project and there is a huge area that we need to explore with using AI, let me explain about the future development and proposals in my mind.
--->

---

# Parameter Adjustment

- more stable AI behavior.
- Latency for user experience.
- LLM Models.

<!---
# OpenAI has many parameters available to tune the behavior of AI.
# as user experience, i would pay attention about stable behavior and response time.
# it would not be useful if the behavior is not predictable or stable for user experience...
# and also response time should be reasonably quick enough to get the answer from ros2ai, if that is lagging, that could be frustration for user.
# besides these, i would like to explore to more base models and model information for better experience.
--->

---

# Session Mode

currently, `ros2ai` only supports single-shot completion API, that means we can not rely on the previous questions or answers to make further requests. e.g) "Is /chatter topic available?", "Subscribe it!". This ***it*** should be recognized in the same context or session. If this command is issued, prompt should be initiated to keep the session alive until the entire session is closed by the user.

<!---
# for now, ros2ai does not support session cross-conversation result.
# that said, you can ask "/chatter topic available?", and ros2ai shows "/chatter", but you cannot ask "subscribe it".
# this is just because ros2ai is not aware of the session, so it does not understand this `it` for now.
# off the top of my head, it would be easy to support this with LangChain library to support the history just like other AI application does.
# but the question here is, history depth, where to cache, what the user interaction would be... those need to be well considered before implementation.
--->

---

# Proxy Agent

`ros2ai` can instantiate the agent daemon process as ROS 2 service, so that even other ROS 2 application running in the network can use OpenAI API via this proxy. those ROS 2 application can be agnostic from LLM specific APIs but ROS 2 service interface.

<!---
# For doing this ros2ai can act like a bridge between ROS 2 application and OpenAI or Ollama with text interfaces.
# this is totally doable, but expecting that application requirement could be different from the ros2ai.
# if that is the case, this would not be really that useful for other applications.
# at least, if we can spawn the daemon process by design, we can use ros2ai anywhere in the ROS 2 system, probably this can be useful for some cases.
--->

---

# Function Calling

for more user friendly experience, user should not be aware of `ros2ai` sub-command at all such as query, execute. this is actually against the design policy for `ros2ai`.
we could take advantage of [Function Calling](https://platform.openai.com/docs/guides/function-calling) to conceal these sub-commands, and internally categorize the request based on the user input.

<!---
# this is gonna be a good feature to support the most simple use of `ros2ai` without any options and sub-commands.
# currently it is the responsibility and burden for user to tell the sub-command category so that ros2ai can be told the context what needs to be done, question or execution?
# although this works for now, i believe that should be processed and categorized by AI, instead of user's burden.
# i think we can take advantage of Function Calling method to categorize the request with using AI, and then process the next user's request with that predicted context.
# i am expecting this would be useful to support more user friendly interfaces with ros2ai.
--->

---

# Fine-tuning (T.B.D)

- ROS 2 general fine-tuning
  - this requires scaled training dataset for ROS 2, agnostic from user environment. could be distro specific. so that AI can response more precisely based on questions and requests.
- User specified environment tuning
  - this dataset should be uploaded to help more user specific problems and questions. if this is doable, `ros2ai` agent can the 1st customer support for anyone?

<!---
# we have 2 aspects here, the one is general tuning, and the other is user's specific data based tuning.
# Since this requires huge amount of time and cost, i am not even sure what we can do with this.
# besides, I am not sure if we can fetch the user's private environmental information to make it better.
# probably some users do not want to do this because of data privacy, but that is required to upload if user wants the AI to answer the your environment specific questions or requests.
# this would be something we are going to find out how we would want to proceed with community...
--->

---

![bg right:40% 50%](./images/ROS2_AI_WG_QRcode.png)

# [Embodies AI Working Group](https://discourse.ros.org/t/embodied-ai-community-group-call-of-interest/39990)

# Join Us 😄🚀👨‍💻
## Let's talk about **Robotics & AI**

<!---
# i will be working with Robotec.AI as a member of ROS community to share experience and technical things.
# there will be AI WG, that is announced on this discourse thread.
# if you are interested, please feel free to join us and make difference together.
--->

---

# Questions?

![bg right:40% 50%](./images/qrcode_image.png)
