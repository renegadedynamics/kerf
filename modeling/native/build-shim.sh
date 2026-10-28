#!/usr/bin/env bash
# Builds the b3docct shim against the OCCT in .deps/install/<rid> and installs it into native/out/<rid>.
#   native/build-shim.sh <rid>
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
rid="${1:?usage: native/build-shim.sh <rid>}"

cmake -S "$root/native/shim" -B "$root/.deps/build/shim-$rid" -G Ninja -DCMAKE_BUILD_TYPE=Release \
  "-DCMAKE_PREFIX_PATH=$root/.deps/install/$rid" "-DCMAKE_INSTALL_PREFIX=$root/native/out/$rid"
cmake --build "$root/.deps/build/shim-$rid" --target install
