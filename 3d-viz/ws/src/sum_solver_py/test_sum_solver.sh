#!/usr/bin/env bash
set -euo pipefail

# -------- Config (you can override via env vars) -----------------------------
PKG=${PKG:-sum_solver_py}
EXEC=${EXEC:-two_sum}
NODE_NAME=${NODE_NAME:-two_sum_node}       # your node's name in code
TARGET=${TARGET:-9}                        # Int8 target
INPUT_CSV=${INPUT_CSV:-"2,7,11,15"}        # comma-separated ints for /input
EXPECT_A=${EXPECT_A:-0}                    # expected first index
EXPECT_B=${EXPECT_B:-1}                    # expected second index
WAIT_SECS=${WAIT_SECS:-8}                  # how long to wait for the node/topic
# ----------------------------------------------------------------------------

# # Try to source overlay from common locations if ROS isn't already set up
# if ! command -v ros2 >/dev/null 2>&1; then
#   for d in "." ".." "../.." ; do
#     if [[ -f "$d/install/setup.bash" ]]; then
#       # shellcheck disable=SC1090
#       source "$d/install/setup.bash"
#       break
#     fi
#   done
# fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[ERR] ros2 not on PATH. Source your ROS 2 environment first." >&2
  exit 1
fi

cleanup() {
  set +e
  if [[ -n "${NODE_PID:-}" ]]; then
    kill "$NODE_PID" 2>/dev/null || true
    wait "$NODE_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "[INFO] Starting node: ros2 run $PKG $EXEC"
ros2 run "$PKG" "$EXEC" &
NODE_PID=$!

# Wait for node to come up
echo -n "[INFO] Waiting for node '/$NODE_NAME' ..."
deadline=$((SECONDS + WAIT_SECS))
while (( SECONDS < deadline )); do
  if ros2 node list | grep -Eq "(/)?${NODE_NAME}$"; then
    echo " found."
    break
  fi
  sleep 0.2
done
if ! ros2 node list | grep -Eq "(/)?${NODE_NAME}$"; then
  echo -e "\n[ERR] Node '/$NODE_NAME' not found within ${WAIT_SECS}s."
  exit 1
fi

# Prime subscriber to capture exactly one solution message
echo "[INFO] Subscribing to /solution for one message..."
SOLN_RAW="$(timeout "${WAIT_SECS}s" ros2 topic echo -n 1 /solution 2>/dev/null || true)" &

# Publish target then input (node should solve on second message)
echo "[INFO] Publishing target=$TARGET to /target"
ros2 topic pub --once /target std_msgs/msg/Int8 "{data: ${TARGET}}"
echo "[INFO] Publishing input=[${INPUT_CSV}] to /input"
ros2 topic pub --once /input std_msgs/msg/Int8MultiArray "{data: [${INPUT_CSV}]}"

# Collect one message from /solution
SOLN_RAW="$(timeout "${WAIT_SECS}s" ros2 topic echo -n 1 /solution 2>/dev/null || true)"
if [[ -z "$SOLN_RAW" ]]; then
  echo "[ERR] No message received on /solution within ${WAIT_SECS}s."
  exit 2
fi

# Extract just the 'data' array (ignore layout zeros)
readarray -t DATA_LINES < <(printf "%s\n" "$SOLN_RAW" | awk '
  /^data:/ {in_data=1; next}
  in_data && /^[[:space:]]*-/ {print $2}
')
if ((${#DATA_LINES[@]} == 0)); then
  # Some distros print "data: [a, b]" on one line; handle that too
  mapfile -t DATA_LINES < <(printf "%s\n" "$SOLN_RAW" | sed -n 's/.*data:\s*\[\s*\([0-9-]\+\)\s*,\s*\([0-9-]\+\)\s*\].*/\1 \2/p')
fi

# Normalize to two integers A B
if ((${#DATA_LINES[@]} >= 2)); then
  A="${DATA_LINES[0]}"
  B="${DATA_LINES[1]}"
else
  # If the sed path was used, DATA_LINES[0] contains "A B"
  read -r A B <<< "${DATA_LINES[0]:-}"
fi

if [[ -z "${A:-}" || -z "${B:-}" ]]; then
  echo "[ERR] Could not parse indices from /solution."
  echo "------ raw message ------"
  echo "$SOLN_RAW"
  echo "-------------------------"
  exit 3
fi

echo "[INFO] Received /solution indices: [$A, $B]   (expected: [$EXPECT_A, $EXPECT_B])"

if [[ "$A" == "$EXPECT_A" && "$B" == "$EXPECT_B" ]]; then
  echo "[OK] Test passed ✅"
  exit 0
else
  echo "[FAIL] Test failed ❌"
  echo "------ raw message ------"
  echo "$SOLN_RAW"
  echo "-------------------------"
  exit 4
fi
