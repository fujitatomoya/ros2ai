#!/bin/bash

#####################################################################
# This script verifies ros2ai on the localhost system.
#
# Localhost could be physical machine or container runtime environment,
# this script is agnostic from the system but just checks if ros2ai
# works okay with accessing the LLM API endpoints.
#####################################################################

################
# User Setting #
################

# ROS setting should be done before calling this scripts, falling back to defaults.
ROS_VERSION="${ROS_VERSION:-2}"
ROS_PYTHON_VERSION="${ROS_PYTHON_VERSION:-3}"
ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
ROS_DISTRO="${ROS_DISTRO:-rolling}"

# update this list to local verification
# CAUTION: BE ADVISED THAT THIS WILL USE YOUR API KEY.
command_list=(
    "ros2 ai"
    "ros2 ai -h"
    "ros2 ai status -h"
    "ros2 ai status"
    "ros2 ai status -vl"
    "ros2 ai query -h"
    "ros2 ai query \"say hello\""
    "ros2 ai query \"say hello\" -nv"
    "ros2 ai query \"say hello\" -m gpt-5 -t 100"
    "ros2 ai exec \"give me all topics\""
    "ros2 ai exec \"give me all topics\" --dry-run"
    "ros2 ai exec \"give me all topics\" -d"
)

########################
# Function Definitions #
########################

# Note: trap cannot handle return code by signals only. this only works with general error signal.
function exit_trap() {
    # shellcheck disable=SC2317  # Don't warn about unreachable commands in this function
    if [ $? != 0 ]; then
        echo "Command [$BASH_COMMAND] is failed"
        exit 1
    fi
}

function check_and_exit() {
    # shellcheck disable=SC2317  # Don't warn about unreachable commands in this function
    if [ $? -ne 0 ]; then
        echo "Error: [$BASH_COMMAND] failed with exit code $?"
        exit $?
    fi
}

function check_user_setting () {
    trap exit_trap ERR
    echo "[${FUNCNAME[0]}]: checking user setting and configuration."
    if [ -z "$OPENAI_API_KEY" ]; then
        echo "OPENAI_API_KEY is not set, if accessing OpenAI it will fail to call API."
    fi
    # check if ros2 environment setting (trap function can catch the error)
    ros2
}

function verify_ros2ai() {
    trap exit_trap ERR
    echo "[${FUNCNAME[0]}]: verifying ros2ai."
    # execute all commands in the list
    for command in "${command_list[@]}"; do
        echo "----- $command"
        eval $command
        check_and_exit
    done
    echo "----- all ros2ai commands return successfully!!! -----"
}

########
# Main #
########

# set the trap on error
trap exit_trap ERR

# call verification sequence
check_user_setting
verify_ros2ai

exit 0
