#!/usr/bin/env bash
set -euo pipefail

# Kill echo subscriber if running
if [[ -f .twosum_echo.pid ]]; then
  PID=$(cat .twosum_echo.pid)
  if kill -0 "$PID" 2>/dev/null; then
    echo "[INFO] Killing echo subscriber (PID=$PID)"
    kill "$PID" || true
    wait "$PID" 2>/dev/null || true
  fi
  rm -f .twosum_echo.pid
fi

# Kill node if running
if [[ -f .twosum_node.pid ]]; then
  PID=$(cat .twosum_node.pid)
  if kill -0 "$PID" 2>/dev/null; then
    echo "[INFO] Killing node (PID=$PID)"
    kill "$PID" || true
    wait "$PID" 2>/dev/null || true
  fi
  rm -f .twosum_node.pid
fi

echo "[INFO] All publishers/subscribers stopped."
