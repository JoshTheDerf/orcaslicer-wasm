#!/usr/bin/env bash
# Build the texture import module (texture.mjs / texture.wasm / texture.data) for Cubby Slicer.
# A separate, lazily loaded module next to the slicer engine (contract: cubby-slicer
# docs/ENGINE-CONTRACT.md "Textured and coloured model import"): OrcaSlicer 2.5's glTF/GLB/FBX import
# (Assimp) and texture -> colour painting (OpenCV), see wasm/texture.cmake.
#
#   bash scripts/build-texture.sh                        # release, in build-wasm/ (shares the slicer's libslic3r)
#   BUILD_VARIANT=debug bash scripts/build-texture.sh    # SAFE_HEAP/ASSERTIONS, in build-wasm-debug/
#   NPROC=3 ...                                          # parallel compile jobs (default 3)
#
# Needs the toolchain's Assimp + OpenCV: `bash ../wasm-deps/build-deps.sh texture`.
# Single-threaded only. Builds only the `texture` target (the slicer target is not rebuilt).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
if [[ -z "${WASM_DEPS_ENV:-}" ]]; then
  if [[ -f "$ROOT/../wasm-deps/env.sh" ]]; then WASM_DEPS_ENV="$ROOT/../wasm-deps/env.sh"; else WASM_DEPS_ENV="$ROOT/toolchain/env.sh"; fi
fi
unset WASM_THREADS
# shellcheck disable=SC1090
source "$WASM_DEPS_ENV"

VARIANT="${BUILD_VARIANT:-release}"
BUILD_DIR="build-wasm"; [[ "$VARIANT" == "debug" ]] && BUILD_DIR="build-wasm-debug"
NPROC="${NPROC:-3}"
ORCA_TAG="${ORCA_TAG:-v2.5.0}"

for f in "$WASM_DEPS_ROOT/assimp-5.4.3/lib/libassimp.a" "$WASM_DEPS_ROOT/opencv-4.14.0/lib/libopencv_imgcodecs.a"; do
  [[ -f "$f" ]] || { echo "missing $f: bash $WASM_DEPS_ROOT/build-deps.sh texture" >&2; exit 1; }
done

# Orca source at the pinned tag with the WASM patch applied (scripts/build-wasm.sh does the checkout).
if [[ "$(git -C orca rev-parse HEAD)" != "$(git -C orca rev-parse "$ORCA_TAG^{commit}" 2>/dev/null)" ]]; then
  echo "orca/ is not at $ORCA_TAG: run scripts/build-wasm.sh once (it checks out and patches orca/)" >&2
  exit 1
fi
git -C orca apply --reverse --check ../patches/orca-wasm.patch 2>/dev/null \
  || { echo "orca-wasm.patch is not applied in orca/: run scripts/build-wasm.sh once" >&2; exit 1; }

emcmake cmake -S wasm -B "$BUILD_DIR" -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_VARIANT="$VARIANT" -DORCA_WASM_THREADS=OFF
cmake --build "$BUILD_DIR" --target texture -j"$NPROC"

# version.json sidecar (tex_version()).
node --input-type=module -e "
import { readFileSync, writeFileSync } from 'node:fs';
const dir = process.argv[1];
const { default: f } = await import('file://' + dir + '/texture.mjs');
const m = await f({ wasmBinary: readFileSync(dir + '/texture.wasm'), locateFile: (p) => dir + '/' + p, print() {}, printErr() {} });
writeFileSync(dir + '/texture-version.json', m.UTF8ToString(m._tex_version()) + '\n');
" "$ROOT/$BUILD_DIR"
ls -la "$BUILD_DIR"/texture.{mjs,wasm,data} "$BUILD_DIR"/texture-version.json
cat "$BUILD_DIR"/texture-version.json
