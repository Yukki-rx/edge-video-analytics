#!/usr/bin/env bash

set -Eeuo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"

source "${ROS_SETUP}"
source "${WORKSPACE_DIR}/devel/setup.bash"

exec roslaunch robot_vision endpoint.launch "$@"
