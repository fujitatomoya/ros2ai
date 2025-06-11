#!/bin/bash

#####################################################################
# ros2ai ROS 2 Next-Generation Command Line Interface
#
# This script builds ros2ai within ros distro docker images.
#
# To avoid updating and modifying the files under `.github/workflows`,
# this scripts should be adjusted building process accordingly.
# And `.github/workflows` just calls this script in the workflow pipeline.
# This allows us to maintain the workflow process easier for contributors.
#
#####################################################################

########################
# Function Definitions #
########################

function mark {
    export $1=`pwd`;
}

function exit_trap() {
    # shellcheck disable=SC2317  # Don't warn about unreachable commands in this function
    if [ $? != 0 ]; then
        echo "Command [$BASH_COMMAND] is failed"
        exit 1
    fi
}

function get_ubuntu_version () {
    trap exit_trap ERR
    echo "[${FUNCNAME[0]}]: get ubuntu version."
    UBUNTU_VERSION=$(grep '^VERSION_ID=' /etc/os-release | awk -F'=' '{print $2}' | tr -d '"')
}

function install_prerequisites () {
    trap exit_trap ERR
    echo "[${FUNCNAME[0]}]: update and install dependent packages."
    apt update && apt upgrade -y
    # TODO@fujitatomoya: should install openai via package.xml
    apt install -y pip
    if [ $UBUNTU_VERSION == "24.04" ]; then
        pip install openai ollama validators --break-system-packages --ignore-installed
    else
        pip install openai ollama validators --ignore-installed
    fi
    #apt install -y ros-${ROS_DISTRO}-desktop --no-install-recommends
    cd $there
}

function setup_build_colcon_env () {
    trap exit_trap ERR
    echo "[${FUNCNAME[0]}]: set up colcon build environment."
    mkdir -p ${COLCON_WORKSPACE}/src
    cd ${COLCON_WORKSPACE}
    cp -rf $there ${COLCON_WORKSPACE}/src
}

function build_colcon_package () {
    trap exit_trap ERR
    echo "[${FUNCNAME[0]}]: build ros2ai package."
    source /opt/ros/${ROS_DISTRO}/setup.bash
    cd ${COLCON_WORKSPACE}
    # TODO@fujitatomoya: extend this with `colcon test`.
    #  Once ollama is supported, we can integrate `verification.sh` via github workflow as 1st.
    colcon build --symlink-install --packages-select ros2ai
}

########
# Main #
########

export DEBIAN_FRONTEND=noninteractive
export COLCON_WORKSPACE=/tmp/colcon_ws
export UBUNTU_VERSION=24.04

# mark the working space root directory, so that we can come back anytime with `cd $there`
mark there

# set the trap on error
trap exit_trap ERR

# call install functions in sequence
get_ubuntu_version
install_prerequisites
setup_build_colcon_env
build_colcon_package

exit 0
