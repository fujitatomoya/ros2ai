# Build:
#  docker build --rm -f ./docker/Dockerfile --build-arg="ROS_DISTRO=rolling" --build-arg="COLCON_WS=/root/colcon_ws" -t <user_name>/ros2ai:rolling .
#
# Usage:
#  docker pull <user_name>/ros2ai:rolling

# An ARG declared before a FROM is outside of a build stage,
# so it can’t be used in any instruction after a FROM.
# To use the default value of an ARG declared before the first FROM
# use an ARG instruction without a value inside of a build stage:
ARG ROS_DISTRO=rolling
ARG COLCON_WS=/root/colcon_ws

FROM ros:${ROS_DISTRO}

LABEL maintainer="Tomoya Fujita <tomoya.fujita825@gmail.com>"
LABEL version="1.0"
LABEL description="ros2ai ${ROS_DISTRO} docker image"

ARG ROS_DISTRO
ARG COLCON_WS

SHELL ["/bin/bash","-c"]

RUN mkdir -p ${COLCON_WS}/src
COPY . ${COLCON_WS}/src/ros2ai/

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
RUN pip install openai

# Build and source colcon workspace
RUN cd $COLCON_WS \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && colcon build --symlink-install --packages-select ros2ai

# Add source environment in .bashrc
RUN echo -n -e "\n" >> /root/.bashrc
RUN echo "### ros2ai workspace setting" >> /root/.bashrc
RUN echo "cd $COLCON_WS && source ./install/setup.bash" >> /root/.bashrc

# Overwrite as environmental varialble so that entrypoint can rely on those
# OPENAI_API_KEY should not be included here, that is required for the runtime
ENV COLCON_WS=${COLCON_WS}
ENV ROS_DISTRO=${ROS_DISTRO}
#ENTRYPOINT ["/ros_entrypoint.sh"]
