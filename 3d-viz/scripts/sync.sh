#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -f "$root/ws.repos" ]]; then
  mkdir -p "$root/ws/src"
  vcs validate < "$root/ws.repos"
  vcs import "$root/ws/src" < "$root/ws.repos"
  vcs pull "$root/ws/src"
else
  echo "No ws.repos found (single-repo or packages already present)."
fi
