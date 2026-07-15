#!/usr/bin/env bash

set -Eeuo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
EDGE_VENV="${EDGE_VENV:-${HOME}/venvs/edge_yolo}"

source "${ROS_SETUP}"
source "${EDGE_VENV}/bin/activate"
source "${WORKSPACE_DIR}/devel/setup.bash"

exec roslaunch edge_yolo_ros edge.launch "$@"
