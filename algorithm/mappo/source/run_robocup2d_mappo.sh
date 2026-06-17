#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
GITHUB_ROOT=$(cd "$SCRIPT_DIR/../../.." && pwd)
exec "$GITHUB_ROOT/scripts/run_mappo_demo.sh" "$@"
