#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
GITHUB_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
R2DRL_ROOT="$GITHUB_ROOT/R2DRL"
JOBS=${JOBS:-$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4)}

cmake_build() {
    local src=$1
    local build=$2
    shift 2
    cmake -S "$src" -B "$build" "$@"
    cmake --build "$build" -j "$JOBS"
}

cmake_build "$R2DRL_ROOT/rcssserver" "$R2DRL_ROOT/rcssserver/build"
cmake_build "$R2DRL_ROOT/librcsc" "$R2DRL_ROOT/librcsc/build"
cmake_build "$R2DRL_ROOT/helios-base" "$R2DRL_ROOT/helios-base/build" -DCMAKE_PREFIX_PATH="$R2DRL_ROOT/librcsc/build"

echo "[INFO] Build finished. The demo scripts use the bundled runtime paths under R2DRL/."
