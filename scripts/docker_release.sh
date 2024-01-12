#!/bin/bash

#####################################################################
# This script builds, verifies and releases ros2ai docker images.
#####################################################################

################
# User Setting #
################

DOCKERHUB_USERNAME="${DOCKERHUB_USERNAME:-tomoyafujita}"
COLCON_WS="${COLCON_WS:-/root/colcon_ws}"

ros_distros=(
    "humble"
    "iron"
    "rolling"
)

######################
# Options (defaults) #
######################

build_image=false
verify_image=false
upload_image=false

########################
# Function Definitions #
########################

function print_usage() {
    echo "Usage: $0 [-b] [-v] [-u]"
    echo "Options(default):"
    echo "  -b : build docker container images (default: false)"
    echo "  -v : verify images to call ros2ai commands (default: false)"
    echo "  -u : uploade images to DockerHub (default: false)"
    exit 1
}

function exit_trap() {
    if [ $? != 0 ]; then
        echo "Command [$BASH_COMMAND] is failed"
        exit 1
    fi
}

function check_dockerhub_setting () {
    trap exit_trap ERR
    echo "[${FUNCNAME[0]}]: checking dockerhub setting and configuration."
    if [ -z "$DOCKERHUB_USERNAME" ]; then
        echo "DOCKERHUB_USERNAME is not set."
        exit 1
    fi
    # check if docker login succeeds
    docker login
}

function command_exist() {
    trap exit_trap ERR
    echo "[${FUNCNAME[0]}]: checking $1 command exists."
    if command -v "$1" >/dev/null 2>&1; then
        echo "Error: $1 not found."
        return 1
    else
        echo "$1 exists."
        return 0
    fi
}

function build_images() {
    trap exit_trap ERR
    echo "[${FUNCNAME[0]}]: building ros2ai docker container images."
    for distro in "${ros_distros[@]}"; do
        echo "----- $distro image building"
        docker build --rm -f ./docker/Dockerfile --build-arg="ROS_DISTRO=$distro" --build-arg="COLCON_WS=$COLCON_WS" -t $DOCKERHUB_USERNAME/ros2ai:$distro .
    done
    echo "----- all images successfully generated!!! -----"
}

function verify_images() {
    trap exit_trap ERR
    echo "[${FUNCNAME[0]}]: verifying ros2ai docker container images."
    for distro in "${ros_distros[@]}"; do
        echo "----- $distro image verifying"
        docker run -it --rm -e OPENAI_API_KEY=$OPENAI_API_KEY $DOCKERHUB_USERNAME/ros2ai:$distro \
            bash -c "/ros_entrypoint.sh && source /root/.bashrc && cd $COLCON_WS && source ./install/setup.bash && $COLCON_WS/src/ros2ai/scripts/verification.sh"
    done
    echo "----- all images successfully verified!!! -----"
}

function upload_images() {
    trap exit_trap ERR
    echo "[${FUNCNAME[0]}]: uploading ros2ai docker container images."
    for distro in "${ros_distros[@]}"; do
        echo "----- $distro image uploading"
        # TODO@fujitatomoya: support multi-arch docker images
        docker push $DOCKERHUB_USERNAME/ros2ai:$distro
    done
    echo "----- all images successfully verified!!! -----"
}

########
# Main #
########

# set the trap on error
trap exit_trap ERR

# parse command line options
while getopts ":bvu" opt; do
    case $opt in
        b)
            build_image=true
            ;;
        v)
            verify_image=true
            ;;
        u)
            upload_image=true
            ;;
        \?)
            echo "Invalid option: -$OPTARG"
            print_usage
            ;;
    esac
done
shift $((OPTIND-1))

# check settings
check_dockerhub_setting
if command_exist docker; then
    exit 1
fi

# building images
if [ "$build_image" = true ]; then
    build_images
fi

# verifying images
if [ "$verify_image" = true ]; then
    verify_images
fi

# upload images
if [ "$upload_image" = true ]; then
    upload_images
fi

exit 0
