#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "Compatibility alias: use 'bash nav-one.sh' in new instructions."
exec bash "$SCRIPT_DIR/nav-one.sh" "$@"
