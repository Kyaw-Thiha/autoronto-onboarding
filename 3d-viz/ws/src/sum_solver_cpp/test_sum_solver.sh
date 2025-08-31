#!/usr/bin/env bash
set -euo pipefail

# -------- Config (override via env vars) -------------------------------------
PKG=${PKG:-sum_solver_cpp}         # was sum_solver_py
EXEC=${EXEC:-two_sum_node}         # was two_sum
NODE_NAME=${NODE_NAME:-sum_solver_cpp}  # was sum_solver_py
TARGET=${TARGET:-9}
INPUT_CSV=${INPUT_CSV:-"2,7,11,15"}
EXPECT_A=${EXPECT_A:-0}
EXPECT_B=${EXPECT_B:-1}
WAIT_SECS=${WAIT_SECS:-16}
# -----------------------------------------------------------------------------

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[ERR] ros2 not on PATH. Source your ROS 2 environment first." >&2
  exit 1
fi

LOG="$(mktemp -t sum_solver_node.XXXX.log)"
TMP_SOLN="$(mktemp -t sum_solver_soln.XXXX.txt)"
TMP_ERR="$(mktemp -t sum_solver_soln_err.XXXX.txt)"

cleanup() {
  set +e
  [[ -n "${ECHO_PID:-}" ]] && { kill "$ECHO_PID" 2>/dev/null || true; wait "$ECHO_PID" 2>/dev/null || true; }
  if [[ -n "${NODE_PID:-}" ]]; then
    kill "$NODE_PID" 2>/dev/null || true
    wait "$NODE_PID" 2>/dev/null || true
  fi
  rm -f "$TMP_SOLN" "$TMP_ERR" "$LOG"
}
trap cleanup EXIT INT TERM

# Helpers ---------------------------------------------------------------------
topic_counts() { # prints "<pubs> <subs>" or "0 0" if info unavailable
  local t="$1" info pubs subs
  info="$(ros2 topic info "$t" 2>/dev/null || true)"
  pubs="$(sed -n 's/.*Publisher count: \([0-9]\+\).*/\1/p' <<<"$info" | head -n1)"
  subs="$(sed -n 's/.*Subscription count: \([0-9]\+\).*/\1/p' <<<"$info" | head -n1)"
  echo "${pubs:-0} ${subs:-0}"
}

wait_for_counts() { # wait_for_counts <topic> <min_pubs> <min_subs> <timeout_secs>
  local t="$1" want_p="$2" want_s="$3" deadline=$((SECONDS + ${4:-$WAIT_SECS}))
  while (( SECONDS < deadline )); do
    read -r pubs subs < <(topic_counts "$t")
    (( pubs >= want_p && subs >= want_s )) && return 0
    sleep 0.1
  done
  echo "[WARN] Timeout waiting for $t pubs>=$want_p subs>=$want_s" >&2
  ros2 topic info "$t" || true
  return 1
}

# Start node ------------------------------------------------------------------
echo "[INFO] Starting node: ros2 run $PKG $EXEC --ros-args -r __node:=$NODE_NAME"
( set -o pipefail; stdbuf -oL ros2 run "$PKG" "$EXEC" --ros-args -r __node:="$NODE_NAME" 2>&1 | tee "$LOG" ) &
NODE_PID=$!

sleep 0.3
if ! kill -0 "$NODE_PID" 2>/dev/null; then
  echo "[ERR] Node exited immediately. Recent log:"
  tail -n +1 "$LOG"
  exit 1
fi

# Wait for node presence
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
  echo "---- node log ----"; tail -n +1 "$LOG"; echo "------------------"
  exit 1
fi

# Ensure graph is connected before we send/receive ----------------------------
# 1) Wait until the node has created its /solution publisher.
wait_for_counts /solution 1 0 "$WAIT_SECS" || true

# 2) Wait until the node is subscribed to /target and /input.
wait_for_counts /target   0 1 "$WAIT_SECS" || true
wait_for_counts /input    0 1 "$WAIT_SECS" || true

# 3) Start echo subscriber (transient_local to catch late-latched msgs), then ensure it is attached to /solution.
echo "[INFO] Listening for one /solution message..."
( set -o pipefail; stdbuf -oL ros2 topic echo /solution --once --qos-reliability reliable >"$TMP_SOLN" 2>"$TMP_ERR" ) &  # removed --qos-durability transient_local
ECHO_PID=$!
# Wait until /solution has at least one subscriber (our echo)
wait_for_counts /solution 1 1 "$WAIT_SECS" || true
sleep 0.2  # tiny settle

# Publish input ----------------------------------------------------------------
echo "[INFO] Publishing target=$TARGET to /target"
ros2 topic pub --once /target std_msgs/msg/Int8 "{data: ${TARGET}}"

echo "[INFO] Publishing input=[${INPUT_CSV}] to /input"
ros2 topic pub --once /input std_msgs/msg/Int8MultiArray "{data: [${INPUT_CSV}]}"

# Wait for /solution receipt with a manual timeout -----------------------------
deadline=$((SECONDS + WAIT_SECS))
while kill -0 "$ECHO_PID" 2>/dev/null && (( SECONDS < deadline )); do
  sleep 0.1
done

if kill -0 "$ECHO_PID" 2>/dev/null; then
  echo "[ERR] No message received on /solution within ${WAIT_SECS}s."
  echo "---- /solution echo stderr ----"; cat "$TMP_ERR" || true; echo "------------------------------"
  echo "---- /solution info ----";      ros2 topic info /solution || true; echo "--------------------------"
  echo "---- /target info ----";        ros2 topic info /target   || true; echo "--------------------------"
  echo "---- /input info ----";         ros2 topic info /input    || true; echo "--------------------------"
  echo "---- node log ----";            tail -n +1 "$LOG";              echo "--------------------------"
  exit 2
fi

SOLN_RAW="$(cat "$TMP_SOLN")"

# Parse the indices ------------------------------------------------------------
readarray -t DATA_LINES < <(printf "%s\n" "$SOLN_RAW" | awk '
  /^data:/ {in_data=1; next}
  in_data && /^[[:space:]]*-/ {print $2}
')
if ((${#DATA_LINES[@]} == 0)); then
  mapfile -t DATA_LINES < <(printf "%s\n" "$SOLN_RAW" | sed -n 's/.*data:\s*\[\s*\([0-9-]\+\)\s*,\s*\([0-9-]\+\)\s*\].*/\1 \2/p')
fi

if ((${#DATA_LINES[@]} >= 2)); then
  A="${DATA_LINES[0]}"; B="${DATA_LINES[1]}"
else
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
