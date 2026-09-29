#!/usr/bin/env bash
# Build the FullSpectrum engine (ratdoux/OrcaSlicer-FullSpectrum: Snapmaker
# Orca + mixed filaments / Local-Z colour blending) with this repo's bridge.
#
#   bash scripts/build-fullspectrum.sh        # → build-fs/ (engine id "fullspectrum")
#
# The fork is checked out at fullspectrum/ (FS_TAG) and patched with
# patches/fullspectrum-wasm.patch (orca-wasm.patch rebased onto the fork,
# plus: MQTT/installer off headless, OpenCV/Assimp optional, libigl
# DynamicSparseMatrix → triplets). Regenerate: git -C fullspectrum diff > patches/fullspectrum-wasm.patch
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
# Toolchain: a shared ../wasm-deps checkout when present, else ./toolchain (see toolchain/README.md).
if [[ -z "${WASM_DEPS_ENV:-}" ]]; then
  if [[ -f "$ROOT/../wasm-deps/env.sh" ]]; then WASM_DEPS_ENV="$ROOT/../wasm-deps/env.sh"; else WASM_DEPS_ENV="$ROOT/toolchain/env.sh"; fi
fi
# shellcheck disable=SC1090
source "$WASM_DEPS_ENV"
FS_TAG="${FS_TAG:-v0.9.14}"
NPROC="${NPROC:-3}"

if [[ ! -d fullspectrum/.git ]]; then
  git clone --depth 1 --branch "$FS_TAG" https://github.com/ratdoux/OrcaSlicer-FullSpectrum.git fullspectrum
fi
if git -C fullspectrum apply --reverse --check ../patches/fullspectrum-wasm.patch 2>/dev/null; then
  echo "fullspectrum-wasm.patch already applied"
else
  git -C fullspectrum apply ../patches/fullspectrum-wasm.patch
  echo "applied fullspectrum-wasm.patch"
fi

emcmake cmake -S wasm -B build-fs -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_VARIANT=release -DORCA_WASM_THREADS=OFF \
  -DORCA_SRC_DIR="$ROOT/fullspectrum" -DCS_ENGINE_ID=fullspectrum
cmake --build build-fs --target slicer -j"$NPROC"
node scripts/gen-schema.mjs build-fs
ls -la build-fs/slicer.{mjs,wasm,data} build-fs/{schema,version}.json
