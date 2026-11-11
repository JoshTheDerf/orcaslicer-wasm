#!/usr/bin/env bash
# Build the Design-tab CAD kernel module (cad.mjs / cad.wasm) for Cubby Slicer.
# Contract: cubby-slicer/docs/CAD-KERNEL-CONTRACT.md. A separate module from the slicer engine:
# it links real OpenCASCADE 8.0.1 (toolchain: `bash ../wasm-deps/build-deps.sh occt`).
#
#   bash scripts/build-cad.sh                        # release → build-cad/
#   BUILD_VARIANT=debug bash scripts/build-cad.sh    # SAFE_HEAP/ASSERTIONS → build-cad-debug/
#   NPROC=3 ...                                      # parallel compile jobs (default 3)
#
# Single-threaded only. The slicer's build dirs (build-wasm, build-wasm-mt) are not touched.
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
BUILD_DIR="build-cad"; [[ "$VARIANT" == "debug" ]] && BUILD_DIR="build-cad-debug"
NPROC="${NPROC:-3}"
ORCA_TAG="${ORCA_TAG:-v2.5.0}"

if [[ ! -f "$WASM_DEPS_ROOT/occt-8.0.1/lib/libTKDESTEP.a" ]]; then
  echo "OCCT 8.0.1 libraries missing: bash $WASM_DEPS_ROOT/build-deps.sh occt" >&2
  exit 1
fi

# Orca source at the pinned tag with the WASM patch applied (same as build-wasm.sh).
if [[ ! -d orca/.git && ! -f orca/.git ]]; then git submodule update --init --depth 1 orca; fi
if [[ "$(git -C orca rev-parse HEAD)" != "$(git -C orca rev-parse "$ORCA_TAG^{commit}" 2>/dev/null)" ]]; then
  echo "orca/ is not at $ORCA_TAG: run scripts/build-wasm.sh once (it checks out and patches orca/)" >&2
  exit 1
fi
git -C orca apply --reverse --check ../patches/orca-wasm.patch 2>/dev/null \
  || { echo "orca-wasm.patch is not applied in orca/: run scripts/build-wasm.sh once" >&2; exit 1; }
# CAD-only hooks (replay observer, failing feature, faces_made_by from a recorded replay): touches
# only libslic3r/CAD/CadDocument.*, which the slicer engine never compiles.
if git -C orca apply --reverse --check ../patches/orca-cad.patch 2>/dev/null; then
  echo "orca-cad.patch already applied"
else
  git -C orca apply ../patches/orca-cad.patch
  echo "applied orca-cad.patch"
fi

emcmake cmake -S wasm -B "$BUILD_DIR" -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_VARIANT="$VARIANT" \
  -DORCA_WASM_THREADS=OFF -DORCA_WASM_CAD=ON
cmake --build "$BUILD_DIR" --target cad -j"$NPROC"

# version.json sidecar (cad_version()).
node --input-type=module -e "
import { readFileSync, writeFileSync } from 'node:fs';
const dir = process.argv[1];
const { default: f } = await import('file://' + dir + '/cad.mjs');
const m = await f({ wasmBinary: readFileSync(dir + '/cad.wasm'), print() {}, printErr() {} });
writeFileSync(dir + '/version.json', m.UTF8ToString(m._cad_version()) + '\n');
" "$ROOT/$BUILD_DIR"
ls -la "$BUILD_DIR"/cad.{mjs,wasm} "$BUILD_DIR"/version.json
cat "$BUILD_DIR"/version.json
