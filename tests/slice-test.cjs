#!/usr/bin/env node
// Baseline slicing test for orcaslicer-wasm.
//
// Loads ../build-wasm/slicer.js (or $ORCA_WASM_DIR/slicer.js), runs
// orc_slice on tests/fixtures/cylinder.stl, writes the produced G-code
// to a file, and exits 0 on success / non-zero on any failure.
//
// Usage:
//   node tests/slice-test.cjs                  # writes /tmp/orca-wasm-test.gcode
//   ORCA_WASM_OUT=/tmp/out.gcode node tests/slice-test.cjs
//   ORCA_WASM_DIR=../some/build node tests/slice-test.cjs
//
// Designed to be invoked from CI. Emits structured pass/fail lines on stdout
// so a downstream verifier (verify-gcode.cjs) can be chained.

const fs = require('fs');
const path = require('path');

const repoRoot   = path.resolve(__dirname, '..');
const wasmDir    = process.env.ORCA_WASM_DIR || path.join(repoRoot, 'build-wasm');
const stlPath    = path.join(__dirname, 'fixtures', 'cylinder.stl');
const outPath    = process.env.ORCA_WASM_OUT || '/tmp/orca-wasm-test.gcode';
const slicerJs   = path.join(wasmDir, 'slicer.js');

if (!fs.existsSync(slicerJs)) {
  console.error(`FAIL: slicer.js not found at ${slicerJs}`);
  console.error('Build it first: bash scripts/build-wasm.sh');
  process.exit(2);
}

const stl = fs.readFileSync(stlPath);
console.log(`stl_bytes ${stl.length}`);

// Load the slicer module via Node's CJS loader. slicer.js is UMD-style;
// the footer assigns to module.exports when require is in scope. We mirror
// to a .cjs tempfile so Node treats it as CJS regardless of the surrounding
// package.json "type" setting.
const os = require('os');
const tmpCjs = path.join(os.tmpdir(), `orca-slicer-${process.pid}.cjs`);
fs.copyFileSync(slicerJs, tmpCjs);
process.on('exit', () => { try { fs.unlinkSync(tmpCjs); } catch {} });

const factory = require(tmpCjs);
if (typeof factory !== 'function') {
  console.error('FAIL: OrcaModule factory not exported by slicer.js');
  process.exit(2);
}

const start = Date.now();

factory({
  locateFile: f => path.join(wasmDir, f),
  print:    () => {},
  printErr: m => process.stderr.write(`[wasm] ${m}\n`),
}).then(m => {
  const inPtr     = m._malloc(stl.length);
  const outPtrPtr = m._malloc(4);
  const outLenPtr = m._malloc(4);
  if (!inPtr || !outPtrPtr || !outLenPtr) {
    console.error('FAIL: malloc returned 0');
    process.exit(3);
  }
  m.HEAPU8.set(stl, inPtr);
  m.setValue(outPtrPtr, 0, 'i32');
  m.setValue(outLenPtr, 0, 'i32');

  // empty init payload — bridge falls back to bundled defaults
  m._orc_init(0, 0);

  let rc;
  try {
    rc = m._orc_slice(inPtr, stl.length, outPtrPtr, outLenPtr);
  } catch (e) {
    // Emscripten throws C++ exceptions as a numeric pointer into the wasm
    // heap. The bridge exports orc_decode_exception(ptr) -> char* which
    // pulls std::exception::what() off the pointer. ccall lets us pass the
    // raw number and get back a JS string.
    let detail = e && e.message ? e.message : String(e);
    if (typeof e === 'number') {
      try {
        const what = m.ccall('orc_decode_exception', 'string', ['number'], [e]);
        if (what) detail = `ptr=${e}  what="${what}"`;
      } catch (_) {}
    }
    console.error(`FAIL: orc_slice threw: ${detail}`);
    if (e && e.stack) console.error(e.stack);
    process.exit(4);
  }

  if (rc !== 0) {
    console.error(`FAIL: orc_slice returned non-zero rc=${rc}`);
    process.exit(5);
  }

  const gp = m.getValue(outPtrPtr, 'i32') >>> 0;
  const gl = m.getValue(outLenPtr, 'i32') >>> 0;
  if (!gp || !gl) {
    console.error(`FAIL: orc_slice returned empty gcode (ptr=${gp} len=${gl})`);
    process.exit(6);
  }

  const gcode = Buffer.from(m.HEAPU8.subarray(gp, gp + gl));
  fs.writeFileSync(outPath, gcode);

  m._orc_free(gp);
  m._free(inPtr);
  m._free(outPtrPtr);
  m._free(outLenPtr);

  const elapsedMs = Date.now() - start;
  console.log(`gcode_bytes ${gl}`);
  console.log(`gcode_path ${outPath}`);
  console.log(`elapsed_ms ${elapsedMs}`);
  console.log('PASS slice');
}).catch(e => {
  console.error(`FAIL: module load/run rejected: ${e && e.message || e}`);
  if (e && e.stack) console.error(e.stack);
  process.exit(7);
});
