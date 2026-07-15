#!/usr/bin/env bash

set -Eeuo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO:-noetic}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO_NAME}/setup.bash}"

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://192.168.0.100:11311}"
export ROS_IP="${ROS_IP:-192.168.0.100}"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "Error: ROS setup file not found: ${ROS_SETUP}" >&2
  echo "Set ROS_SETUP or ROS_DISTRO to match the installed ROS1 environment." >&2
  exit 1
fi

source "${ROS_SETUP}"

if ! command -v catkin_make >/dev/null 2>&1; then
  echo "Error: catkin_make is unavailable after sourcing ${ROS_SETUP}." >&2
  exit 1
fi

if [[ ! -e "${WORKSPACE_DIR}/src/CMakeLists.txt" ]]; then
  catkin_init_workspace "${WORKSPACE_DIR}/src"
fi

if [[ "${SKIP_BUILD:-0}" != "1" ]]; then
  catkin_make -C "${WORKSPACE_DIR}"
elif [[ ! -f "${WORKSPACE_DIR}/devel/setup.bash" ]]; then
  echo "Error: SKIP_BUILD=1, but the endpoint workspace has not been built." >&2
  exit 1
fi

source "${WORKSPACE_DIR}/devel/setup.bash"

echo "Starting endpoint device"
echo "  workspace:      ${WORKSPACE_DIR}"
echo "  ROS_MASTER_URI: ${ROS_MASTER_URI}"
echo "  ROS_IP:         ${ROS_IP}"

exec roslaunch robot_vision endpoint.launch "$@"
