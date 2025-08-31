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

# Guard against nounset + unset AMENT_* vars when sourcing ROS env
set +u
: "${AMENT_TRACE_SETUP_FILES:=}"
: "${AMENT_PYTHON_EXECUTABLE:=$(command -v python3 || true)}"
source /opt/ros/${ROS_DISTRO:-kilted}/setup.bash
set -u

# Build proper dependencies arg (accepts with or without a leading --dependencies)
dep_args=()
if [[ ${#deps[@]} -gt 0 ]]; then
  if [[ "${deps[0]}" == "--dependencies" ]]; then
    deps=("${deps[@]:1}")
  fi
  if [[ ${#deps[@]} -gt 0 ]]; then
    dep_args=(--dependencies "${deps[@]}")
  fi
fi

# Correct call: flags first, package name last
ros2 pkg create --build-type "$build_type" "${dep_args[@]}" --destination-directory src "$pkg"

echo "Created $build_type package at ws/src/$pkg"
