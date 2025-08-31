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

# --- Source this workspace's overlay if the script itself is being sourced ---
# Detect if we're being sourced (works for bash/zsh; sh can't reliably detect)
is_sourced=0
if [ -n "${ZSH_VERSION:-}" ]; then
  case "$ZSH_EVAL_CONTEXT" in *:file) is_sourced=1;; esac
elif [ -n "${BASH_VERSION:-}" ]; then
  if [ "${BASH_SOURCE[0]-}" != "$0" ]; then is_sourced=1; fi
fi

overlay_dir="$root/ws/install"

if [ "$is_sourced" -eq 1 ]; then
  set +u
  if [ -n "${ZSH_VERSION:-}" ] && [ -f "$overlay_dir/setup.zsh" ]; then
    source "$overlay_dir/setup.zsh"
  elif [ -n "${BASH_VERSION:-}" ] && [ -f "$overlay_dir/setup.bash" ]; then
    source "$overlay_dir/setup.bash"
  elif [ -f "$overlay_dir/setup.sh" ]; then
    . "$overlay_dir/setup.sh"   # POSIX generic
  elif [ -f "$overlay_dir/setup.bash" ]; then
    # fallback if only bash file exists
    source "$overlay_dir/setup.bash"
  fi
  set -u
else
  echo "[INFO] Build finished."
  if [ -n "${ZSH_VERSION:-}" ] && [ -f "$overlay_dir/setup.zsh" ]; then
    echo "[HINT] To overlay in this zsh:  source \"$overlay_dir/setup.zsh\""
  elif [ -f "$overlay_dir/setup.bash" ]; then
    echo "[HINT] To overlay in this shell: source \"$overlay_dir/setup.bash\""
  elif [ -f "$overlay_dir/setup.sh" ]; then
    echo "[HINT] To overlay in this shell: . \"$overlay_dir/setup.sh\""
  fi
  echo "[HINT] Or run:  source ./scripts/build.sh   # so the overlay sticks."
fi
