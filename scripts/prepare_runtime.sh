#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
GITHUB_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
R2DRL_ROOT="$GITHUB_ROOT/R2DRL"

export LD_LIBRARY_PATH="$R2DRL_ROOT/librcsc/rcsc/.libs:$R2DRL_ROOT/runtime_libs:$R2DRL_ROOT/rcssserver/build/rcss/clang:$R2DRL_ROOT/rcssserver/build/rcss/conf:$R2DRL_ROOT/rcssserver/build/rcss/net:$R2DRL_ROOT/rcssserver/build/rcss/gzip:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$GITHUB_ROOT/algorithm/qmix/source:$R2DRL_ROOT:${PYTHONPATH:-}"
export PYTHONUNBUFFERED=1
export SACRED_CAPTURE=no

python3 "$SCRIPT_DIR/localize_paths.py"

missing=0
check_file() {
    if [ ! -e "$1" ]; then
        echo "[ERROR] Missing $1" >&2
        missing=1
    fi
}

check_exec() {
    if [ ! -x "$1" ]; then
        echo "[ERROR] Missing executable $1" >&2
        missing=1
    fi
}

check_exec "$R2DRL_ROOT/rcssserver/build/rcssserver"
check_exec "$R2DRL_ROOT/helios-base/src/player/sample_player"
check_exec "$R2DRL_ROOT/helios-base/src/coach/sample_coach"
check_exec "$R2DRL_ROOT/helios-base/src/trainer/sample_trainer"
check_file "$R2DRL_ROOT/helios-base/src/player.conf"
check_file "$R2DRL_ROOT/librcsc/rcsc/.libs/librcsc.so.19"
check_file "$R2DRL_ROOT/runtime_libs/libstdc++.so.6"
check_file "$R2DRL_ROOT/runtime_libs/libgcc_s.so.1"
check_file "$R2DRL_ROOT/runtime_libs/libz.so.1"
check_file "$R2DRL_ROOT/robocup2d/trajectories/scenarioes.npz"

if [ "$missing" -ne 0 ]; then
    exit 1
fi

echo "[INFO] Runtime paths are localized under $GITHUB_ROOT"
echo "[INFO] LD_LIBRARY_PATH=$LD_LIBRARY_PATH"
echo "[INFO] PYTHONPATH=$PYTHONPATH"
