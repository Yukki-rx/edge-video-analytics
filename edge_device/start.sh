#!/usr/bin/env bash

set -Eeuo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO:-noetic}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO_NAME}/setup.bash}"
EDGE_VENV="${EDGE_VENV:-${HOME}/venvs/edge_yolo}"

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://192.168.0.100:11311}"
export ROS_IP="${ROS_IP:-192.168.0.118}"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "Error: ROS setup file not found: ${ROS_SETUP}" >&2
  echo "Set ROS_SETUP or ROS_DISTRO to match the installed ROS1 environment." >&2
  exit 1
fi

if [[ ! -f "${EDGE_VENV}/bin/activate" ]]; then
  echo "Error: edge Python virtual environment not found: ${EDGE_VENV}" >&2
  echo "Set EDGE_VENV to the virtual environment containing PyTorch and Ultralytics." >&2
  exit 1
fi

source "${ROS_SETUP}"
source "${EDGE_VENV}/bin/activate"

if ! command -v catkin_make >/dev/null 2>&1; then
  echo "Error: catkin_make is unavailable after sourcing ${ROS_SETUP}." >&2
  exit 1
fi

if [[ ! -e "${WORKSPACE_DIR}/src/CMakeLists.txt" ]]; then
  catkin_init_workspace "${WORKSPACE_DIR}/src"
fi

if [[ "${SKIP_BUILD:-0}" != "1" ]]; then
  catkin_make -C "${WORKSPACE_DIR}" \
    -DPYTHON_EXECUTABLE="${EDGE_VENV}/bin/python3"
elif [[ ! -f "${WORKSPACE_DIR}/devel/setup.bash" ]]; then
  echo "Error: SKIP_BUILD=1, but the edge workspace has not been built." >&2
  exit 1
fi

source "${WORKSPACE_DIR}/devel/setup.bash"

echo "Starting edge device"
echo "  workspace:      ${WORKSPACE_DIR}"
echo "  ROS_MASTER_URI: ${ROS_MASTER_URI}"
echo "  ROS_IP:         ${ROS_IP}"
echo "  virtualenv:     ${EDGE_VENV}"

exec roslaunch edge_yolo_ros edge.launch "$@"
