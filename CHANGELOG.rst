^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2ai
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
