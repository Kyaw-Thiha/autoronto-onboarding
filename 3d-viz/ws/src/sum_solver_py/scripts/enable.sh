#!/usr/bin/env bash
set -euo pipefail

# -------- Config (override via env vars) -------------------------------------
PKG=${PKG:-sum_solver_py}
EXEC=${EXEC:-two_sum}
NODE_NAME=${NODE_NAME:-sum_solver_py}
TARGET=${TARGET:-9}
INPUT_CSV=${INPUT_CSV:-"2,7,11,15"}
WAIT_SECS=${WAIT_SECS:-16}
# -----------------------------------------------------------------------------

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[ERR] ros2 not on PATH. Source your ROS 2 environment first." >&2
  exit 1
fi

LOG="$(mktemp -t sum_solver_node.XXXX.log)"

cleanup() {
  set +e
  rm -f "$LOG"
}
trap cleanup EXIT INT TERM

# Start node ------------------------------------------------------------------
echo "[INFO] Starting node: ros2 run $PKG $EXEC --ros-args -r __node:=$NODE_NAME"
( set -o pipefail; stdbuf -oL ros2 run "$PKG" "$EXEC" --ros-args -r __node:="$NODE_NAME" 2>&1 | tee "$LOG" ) &
NODE_PID=$!

echo "$NODE_PID" > .twosum_node.pid

# Wait for node presence ------------------------------------------------------
echo -n "[INFO] Waiting for node '/$NODE_NAME' ..."
deadline=$((SECONDS + WAIT_SECS))
while (( SECONDS < deadline )); do
  if ros2 node list | grep -Eq "(/)?${NODE_NAME}$"; then
    echo " found."
    break
  fi
  sleep 0.2
done

# Start echo subscribers ------------------------------------------------------
echo "[INFO] Subscribing to /solution (background)"
ros2 topic echo /solution > .twosum_solution.log 2>&1 &
ECHO_PID=$!
echo "$ECHO_PID" > .twosum_echo.pid

# Publish initial input -------------------------------------------------------
echo "[INFO] Publishing target=$TARGET to /target"
ros2 topic pub --once /target std_msgs/msg/Int8 "{data: ${TARGET}}"

echo "[INFO] Publishing input=[${INPUT_CSV}] to /input"
ros2 topic pub --once /input std_msgs/msg/Int8MultiArray "{data: [${INPUT_CSV}]}"

echo "[INFO] Setup complete. Logs:"
echo "  Node log:      $LOG"
echo "  Solution echo: .twosum_solution.log"
echo "[INFO] Use disable.sh to stop everything."
