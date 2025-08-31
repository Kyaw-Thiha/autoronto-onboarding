#!/usr/bin/env bash
set -euo pipefail
usage(){ echo "Usage: scripts/create_pkg.sh [--py] <package_name> [--dependencies ...]"; }

build_type="ament_cmake"
[[ $# -ge 1 ]] || { usage; exit 1; }

if [[ ${1:-} == "--py" ]]; then
  build_type="ament_python"
  shift
fi

pkg="$1"; shift || true
deps=("$@")
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$root/ws/src"
cd "$root/ws"
source /opt/ros/${ROS_DISTRO:-kilted}/setup.bash
ros2 pkg create "$pkg" "${deps[@]/#/--dependencies }" --build-type "$build_type" --destination-directory src
echo "Created $build_type package at ws/src/$pkg"
