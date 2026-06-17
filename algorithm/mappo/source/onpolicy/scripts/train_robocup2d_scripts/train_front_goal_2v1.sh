#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
GITHUB_ROOT=$(cd "$SCRIPT_DIR/../../../../../.." && pwd)
ENV_CONFIG=${ENV_CONFIG:-parallelr2drl_11vs11scenario_catalog_front-goal-2v1_opp-lv3_epv-on}
exec "$GITHUB_ROOT/scripts/run_mappo_demo.sh" "$@"
