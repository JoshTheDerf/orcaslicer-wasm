#!/usr/bin/env bash
# Build the OrcaSlicer engine for Cubby Slicer (ABI: cubby-slicer/docs/ENGINE-CONTRACT.md).
#
#   bash scripts/build-wasm.sh                  # release → build-wasm/
#   BUILD_VARIANT=debug bash scripts/build-wasm.sh   # SAFE_HEAP/ASSERTIONS → build-wasm-debug/
#   NPROC=3 ...                                 # parallel compile jobs (default 3; ~1-2 GB RAM each)
#
# Requires ../wasm-deps (shared toolchain + deps: `bash ../wasm-deps/build-deps.sh all`).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
# shellcheck disable=SC1091
source "$ROOT/../wasm-deps/env.sh"

VARIANT="${BUILD_VARIANT:-release}"
BUILD_DIR="build-wasm"; [[ "$VARIANT" == "debug" ]] && BUILD_DIR="build-wasm-debug"
NPROC="${NPROC:-3}"
ORCA_TAG="${ORCA_TAG:-v2.4.2}"

# 1) Orca source at the pinned tag with the WASM patch applied (idempotent).
if [[ ! -d orca/.git && ! -f orca/.git ]]; then
  git submodule update --init --depth 1 orca
fi
if ! git -C orca rev-parse -q --verify "refs/tags/$ORCA_TAG" >/dev/null; then
  git -C orca fetch --depth 1 origin tag "$ORCA_TAG"
fi
if [[ "$(git -C orca rev-parse HEAD)" != "$(git -C orca rev-parse "$ORCA_TAG^{commit}")" ]]; then
  echo "checking out orca $ORCA_TAG (discarding local changes in orca/)"
  git -C orca checkout -q -- . && git -C orca clean -fdq && git -C orca checkout -q "$ORCA_TAG"
fi
if git -C orca apply --reverse --check ../patches/orca-wasm.patch 2>/dev/null; then
  echo "orca-wasm.patch already applied"
else
  git -C orca apply ../patches/orca-wasm.patch
  echo "applied orca-wasm.patch"
fi

# 2) Configure + build.
emcmake cmake -S wasm -B "$BUILD_DIR" -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_VARIANT="$VARIANT"
cmake --build "$BUILD_DIR" --target slicer -j"$NPROC"

# 3) Schema + version sidecars.
node scripts/gen-schema.mjs "$BUILD_DIR"
ls -la "$BUILD_DIR"/slicer.{mjs,wasm,data} "$BUILD_DIR"/{schema,version}.json
