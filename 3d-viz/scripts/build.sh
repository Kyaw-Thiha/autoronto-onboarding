#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# load env
set -a
[[ -f "$root/.env" ]] && source "$root/.env"
set +a

# source ROS and optional underlay
source /opt/ros/${ROS_DISTRO:-kilted}/setup.bash
# Example underlay:
# [ -f /work/underlay_ws/install/setup.bash ] && source /work/underlay_ws/install/setup.bash

cd "$root/ws"
colcon build --symlink-install "$@"
