#!/usr/bin/env bash
# Builds FreeType and OCCT for Linux or macOS into .deps/install/<rid>.
#   native/build-deps.sh [rid] [occt-source-dir]
# rid defaults to the host: linux-x64, linux-arm64, osx-x64 or osx-arm64.
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"

if [[ $# -ge 1 && -n "$1" ]]; then
  rid="$1"
else
  case "$(uname -s)-$(uname -m)" in
    Linux-x86_64)  rid=linux-x64 ;;
    Linux-aarch64) rid=linux-arm64 ;;
    Darwin-x86_64) rid=osx-x64 ;;
    Darwin-arm64)  rid=osx-arm64 ;;
    *) echo "unsupported host $(uname -s)-$(uname -m)" >&2; exit 1 ;;
  esac
fi

args=(-S "$root/native/occt" -B "$root/.deps/build/$rid" -G Ninja
      -DCMAKE_BUILD_TYPE=Release "-DCMAKE_INSTALL_PREFIX=$root/.deps/install/$rid")
case "$rid" in
  osx-x64)   args+=(-DCMAKE_OSX_ARCHITECTURES=x86_64 -DCMAKE_OSX_DEPLOYMENT_TARGET=11.0) ;;
  osx-arm64) args+=(-DCMAKE_OSX_ARCHITECTURES=arm64 -DCMAKE_OSX_DEPLOYMENT_TARGET=11.0) ;;
esac
if [[ $# -ge 2 ]]; then args+=("-DOCCT_SOURCE_DIR=$2"); fi

cmake "${args[@]}"
cmake --build "$root/.deps/build/$rid"
