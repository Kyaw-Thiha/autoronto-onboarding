#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# load env
set -a
[[ -f "$root/.env" ]] && source "$root/.env"
set +a

# source ROS and optional underlay (guard against nounset + unset AMENT_* vars)
set +u
: "${AMENT_TRACE_SETUP_FILES:=}"
: "${AMENT_PYTHON_EXECUTABLE:=$(command -v python3 || true)}"
source /opt/ros/${ROS_DISTRO:-kilted}/setup.bash
# Example underlay:
# if [[ -f /work/underlay_ws/install/setup.bash ]]; then source /work/underlay_ws/install/setup.bash; fi
set -u

cd "$root/ws"
colcon build --symlink-install "$@"

