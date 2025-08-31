#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
rm -rf "$root/ws/build" "$root/ws/install" "$root/ws/log"
echo "Cleaned build/install/log"
