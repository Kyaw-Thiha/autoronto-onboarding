#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -d "$root/ws/src" ]]; then
  echo "Nothing in ws/src to freeze. Run scripts/sync.sh first."
  exit 1
fi
vcs export --exact "$root/ws/src" > "$root/ws.repos.lock"
echo "Wrote $root/ws.repos.lock"
