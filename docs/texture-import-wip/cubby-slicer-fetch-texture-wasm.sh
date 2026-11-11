#!/usr/bin/env bash
# Stage the texture import module into public/wasm/texture/ (not committed).
#
# Source (first match wins):
#   $TEXTURE_WASM_BUILD                         explicit build dir
#   ../slicers/orcaslicer-wasm/build-wasm       (local build; that repo's scripts/build-texture.sh)
#
# Staged (contract: docs/ENGINE-CONTRACT.md "Textured and coloured model import"):
#   texture.mjs texture.wasm   OrcaSlicer 2.5 textured / coloured model import (Assimp glTF/GLB/FBX,
#                              OpenCV texture -> colour painting); lazy-loaded, never needed to slice
#   texture.data               preloaded resources/filament_mixing (CMYW / RYBW recipe tables)
#   version.json               tex_version()
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PORT="$REPO_ROOT/../slicers/orcaslicer-wasm"
BUILD="${TEXTURE_WASM_BUILD:-$PORT/build-wasm}"
DEST="$REPO_ROOT/public/wasm/texture"

for f in texture.mjs texture.wasm texture.data texture-version.json; do
  [[ -f "$BUILD/$f" ]] || { echo "error: $BUILD/$f missing (bash $PORT/scripts/build-texture.sh)" >&2; exit 1; }
done

python3 - "$BUILD/texture-version.json" <<'PY'
import json, sys
v = json.load(open(sys.argv[1]))
if v.get("module") != "orca-texture" or v.get("abi") != 1:
    sys.exit(f"error: {sys.argv[1]} is not an orca-texture ABI 1 module: {v}")
PY

mkdir -p "$DEST"
# Atomic per file (copy beside, then rename): a page loading the module meanwhile never reads a
# half-written file. The wasm and data go first, the glue and version last.
for f in texture.wasm texture.data texture.mjs; do cp -f "$BUILD/$f" "$DEST/.$f.staging"; done
cp -f "$BUILD/texture-version.json" "$DEST/.version.json.staging"
for f in texture.wasm texture.data texture.mjs version.json; do mv -f "$DEST/.$f.staging" "$DEST/$f"; done
echo "Staged $(python3 -c "import json;v=json.load(open('$DEST/version.json'));print(v['module'], 'orca', v['orca'], 'assimp', v['assimp'], 'opencv', v['opencv'])") into $DEST"
node -e '
const fs = require("fs"), z = require("zlib");
const b = fs.readFileSync(process.argv[1]);
const gz = z.gzipSync(b, { level: 9 }).length;
const br = z.brotliCompressSync(b, { params: { [z.constants.BROTLI_PARAM_QUALITY]: 11, [z.constants.BROTLI_PARAM_SIZE_HINT]: b.length } }).length;
console.log(`  texture.wasm ${b.length} B (gzip -9 ${gz} B, brotli 11 ${br} B), texture.data ${fs.statSync(process.argv[2]).size} B, texture.mjs ${fs.statSync(process.argv[3]).size} B`);
' "$DEST/texture.wasm" "$DEST/texture.data" "$DEST/texture.mjs"
