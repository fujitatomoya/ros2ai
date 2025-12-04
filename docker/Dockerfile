# Build:
#  docker build --rm -f ./docker/Dockerfile --build-arg="ROS_DISTRO=rolling" --build-arg="COLCON_WS=/home/ros2ai/colcon_ws" -t <user_name>/ros2ai:rolling .
#
# Usage:
#  docker pull <user_name>/ros2ai:rolling
#
# Note: The container runs as non-root user 'ros2ai' (UID=1000, GID=1000) by default for security.
#       You can override the user at runtime with: docker run --user $(id -u):$(id -g) ...

# An ARG declared before a FROM is outside of a build stage,
# so it can't be used in any instruction after a FROM.
# To use the default value of an ARG declared before the first FROM
# use an ARG instruction without a value inside of a build stage:
ARG ROS_DISTRO=rolling
ARG COLCON_WS=/home/ros2ai/colcon_ws
ARG USERNAME=ros2ai
ARG USER_UID=1234
ARG USER_GID=1234

FROM ros:${ROS_DISTRO}

LABEL maintainer="Tomoya Fujita <tomoya.fujita825@gmail.com>"
LABEL version="1.0"
LABEL description="ros2ai ${ROS_DISTRO} docker image"

ARG ROS_DISTRO
ARG COLCON_WS
ARG USERNAME
ARG USER_UID
ARG USER_GID

SHELL ["/bin/bash","-c"]

# Create non-root user with sudo privileges
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && apt-get update \
    && apt-get install -y sudo \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p ${COLCON_WS}/src
COPY . ${COLCON_WS}/src/ros2ai/
RUN chown -R ${USER_UID}:${USER_GID} ${COLCON_WS}

# All apt-get commands start with an update, then install
# and finally, a cache cleanup to keep the image size small.

# Install packages
RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y \
    # Basic utilities just in case
    pip curl\
    --no-install-recommends \
    && rm -rf /var/lib/apt/lists/*

# Adapt pip install option based on distro (technically ubuntu version)
RUN if [ "$ROS_DISTRO" = "humble" ]; then \
        pip install openai ollama validators --ignore-installed; \
    else \
        pip install openai ollama validators --break-system-packages --ignore-installed; \
    fi

# Build and source colcon workspace
RUN cd $COLCON_WS \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && colcon build --symlink-install --packages-select ros2ai \
    && chown -R ${USER_UID}:${USER_GID} $COLCON_WS

# Add source environment in .bashrc
RUN echo -n -e "\n" >> /home/${USERNAME}/.bashrc
RUN echo "### ros2ai workspace setting" >> /home/${USERNAME}/.bashrc
RUN echo "cd $COLCON_WS && source ./install/setup.bash" >> /home/${USERNAME}/.bashrc

# Switch to non-root user
USER ${USERNAME}
WORKDIR /home/${USERNAME}

# Overwrite as environmental variable so that entrypoint can rely on those
# OPENAI_API_KEY should not be included here, that is required for the runtime
ENV COLCON_WS=${COLCON_WS}
ENV ROS_DISTRO=${ROS_DISTRO}
#ENTRYPOINT ["/ros_entrypoint.sh"]
