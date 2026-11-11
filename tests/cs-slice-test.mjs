#!/usr/bin/env node
// End-to-end + robustness tests for the Cubby Slicer ABI (cs_*) Orca build.
//   node tests/cs-slice-test.mjs [build-dir]      (default build-wasm)
// Exercises: real vendor profiles (inheritance-resolved like the web app),
// several meshes incl. a ~200k-triangle one, multi-object transforms,
// back-to-back slices on one instance AND on fresh instances, malformed jobs
// (must return error reports, never trap), cs_eval_condition, schema sanity.
// Multi-threaded builds (build-wasm-mt: exports cs_slice_start) slice on the
// engine's own pthread and are awaited without blocking, like the web worker;
// CS_THREADS=<n> overrides the thread count, CS_SYNC=1 forces plain cs_slice.
import { readFileSync, existsSync, writeFileSync } from 'node:fs';
import { resolve, dirname, join } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

const here = dirname(fileURLToPath(import.meta.url));
const dir = resolve(process.argv[2] ?? join(here, '..', 'build-wasm'));
const profilesDir = join(here, '..', 'orca', 'resources', 'profiles');
const { default: factory } = await import(pathToFileURL(join(dir, 'slicer.mjs')).href);
const wasmBinary = readFileSync(join(dir, 'slicer.wasm'));
const enc = new TextEncoder(), dec = new TextDecoder();

let failures = 0;
const ok = (cond, msg) => { console.log(`${cond ? 'PASS' : 'FAIL'} ${msg}`); if (!cond) failures++; };

const instances = [];
async function instance() {
  const logs = [];
  const mod = await factory({
    wasmBinary, locateFile: (f) => join(dir, f),
    print: () => {}, printErr: (m) => logs.push(m),
    csProgress: () => {},
    ...(process.env.CS_THREADS ? { csThreads: +process.env.CS_THREADS } : {}),
  });
  mod.__logs = logs;
  instances.push(mod);
  return mod;
}
const heap = (mod) => { mod.csSyncHeap?.(); return mod; };

// ---- profiles (same algorithm as the web catalog) -------------------------
const vendorCache = new Map();
function vendor(id) {
  if (!vendorCache.has(id)) vendorCache.set(id, JSON.parse(readFileSync(join(profilesDir, `${id}.json`), 'utf8')));
  return vendorCache.get(id);
}
function findPreset(vendorId, list, name) {
  for (const vid of [vendorId, 'OrcaFilamentLibrary']) {
    const v = vendor(vid);
    const e = v[list]?.find((x) => x.name === name);
    if (e) return { vid, data: JSON.parse(readFileSync(join(profilesDir, vid, e.sub_path), 'utf8')) };
  }
  return null;
}
// PresetBundle::load_vendor_configs_from_json: the resolved parent, then each `include`d preset
// (resolved the same way), then the preset's own keys. Variant vectors are then sized to the
// preset's variant count (extend_default_config_length, see padVariants).
const STRUCTURAL = ['inherits', 'include', 'instantiation', 'from', 'setting_id', 'name', 'type', 'filament_id', 'version', 'description', 'renamed_from'];
function resolvePreset(vendorId, list, name, guard = 0) {
  if (guard > 32) throw new Error(`preset chain too deep: ${name}`);
  const p = findPreset(vendorId, list, name);
  if (!p) throw new Error(`preset not found: ${name}`);
  const out = p.data.inherits ? resolvePreset(p.vid, list, p.data.inherits, guard + 1) : {};
  for (const inc of [].concat(p.data.include ?? [])) Object.assign(out, resolvePreset(p.vid, list, inc, guard + 1));
  for (const [k, v] of Object.entries(p.data)) if (!STRUCTURAL.includes(k)) out[k] = v;
  return out;
}
// Pad (front value) / trim a filament preset's variant vectors to its filament_extruder_variant count.
function padVariants(fil) {
  const n = fil.filament_extruder_variant?.length ?? 1;
  for (const [k, v] of Object.entries(fil))
    if (schema?.options[k]?.variant === 'filament' && Array.isArray(v) && v.length !== n) fil[k] = Array.from({ length: n }, (_, i) => v[i] ?? v[0]);
  return fil;
}
function compatible(vendorId, list, machine) {
  return vendor(vendorId)[list].map((e) => e.name).filter((n) => {
    try {
      const p = findPreset(vendorId, list, n).data;
      return p.instantiation !== 'false' && (p.compatible_printers ?? []).includes(machine);
    } catch { return false; }
  });
}
function configFor(vendorId, machine, process, filament) {
  process ??= compatible(vendorId, 'process_list', machine).find((n) => /0\.20mm/.test(n));
  filament ??= compatible(vendorId, 'filament_list', machine).find((n) => /PLA/.test(n));
  if (!process || !filament) throw new Error(`no process/filament for ${machine}`);
  return { name: `${machine} | ${process} | ${filament}`,
           config: { ...resolvePreset(vendorId, 'machine_list', machine), ...resolvePreset(vendorId, 'process_list', process),
                     ...padVariants(resolvePreset(vendorId, 'filament_list', filament)) } };
}

// ---- meshes -------------------------------------------------------------------
function box(sx, sy, sz) {
  const p = new Float32Array([0,0,0, sx,0,0, sx,sy,0, 0,sy,0, 0,0,sz, sx,0,sz, sx,sy,sz, 0,sy,sz]);
  const i = new Uint32Array([0,2,1, 0,3,2, 4,5,6, 4,6,7, 0,1,5, 0,5,4, 1,2,6, 1,6,5, 2,3,7, 2,7,6, 3,0,4, 3,4,7]);
  return { positions: p, indices: i };
}
function cylinder(r, h, seg) {
  const pos = [], idx = [];
  for (let s = 0; s < seg; s++) { const a = (s / seg) * 2 * Math.PI; pos.push(r * Math.cos(a), r * Math.sin(a), 0, r * Math.cos(a), r * Math.sin(a), h); }
  const cb = pos.length / 3; pos.push(0, 0, 0); const ct = cb + 1; pos.push(0, 0, h);
  for (let s = 0; s < seg; s++) {
    const n = (s + 1) % seg, b0 = 2 * s, t0 = 2 * s + 1, b1 = 2 * n, t1 = 2 * n + 1;
    idx.push(b0, b1, t1, b0, t1, t0, cb, b1, b0, ct, t0, t1);
  }
  return { positions: new Float32Array(pos), indices: new Uint32Array(idx) };
}
function torus(R, r, nu, nv) { // nu*nv*2 triangles
  const pos = new Float32Array(nu * nv * 3), idx = new Uint32Array(nu * nv * 6);
  for (let u = 0; u < nu; u++) for (let v = 0; v < nv; v++) {
    const a = (u / nu) * 2 * Math.PI, b = (v / nv) * 2 * Math.PI, k = (u * nv + v) * 3;
    pos[k] = (R + r * Math.cos(b)) * Math.cos(a); pos[k + 1] = (R + r * Math.cos(b)) * Math.sin(a); pos[k + 2] = r * Math.sin(b) + r;
  }
  let t = 0;
  for (let u = 0; u < nu; u++) for (let v = 0; v < nv; v++) {
    const a = u * nv + v, b = ((u + 1) % nu) * nv + v, c = ((u + 1) % nu) * nv + (v + 1) % nv, d = u * nv + (v + 1) % nv;
    idx.set([a, b, c, a, c, d], t); t += 6;
  }
  return { positions: pos, indices: idx };
}
const translate = (x, y, z) => [1,0,0,0, 0,1,0,0, 0,0,1,0, x,y,z,1];
function rotZ(deg, x, y, z) { const c = Math.cos(deg * Math.PI / 180), s = Math.sin(deg * Math.PI / 180); return [c,s,0,0, -s,c,0,0, 0,0,1,0, x,y,z,1]; }

// ---- ABI helpers ---------------------------------------------------------------
function packJob(config, objects, options = {}) {
  let size = 0; const lay = objects.map((o) => { const v = size; size += o.positions.byteLength; const i = size; size += o.indices.byteLength; return { v, i }; });
  const blob = new Uint8Array(size);
  objects.forEach((o, n) => { blob.set(new Uint8Array(o.positions.buffer), lay[n].v); blob.set(new Uint8Array(o.indices.buffer), lay[n].i); });
  const job = { config, options, objects: objects.map((o, n) => ({ name: o.name ?? `obj${n}`, vertexOffset: lay[n].v, vertexCount: o.positions.length / 3,
    indexOffset: lay[n].i, triangleCount: o.indices.length / 3, transform: o.transform ?? translate(0, 0, 0), config: o.config ?? {} })) };
  return { json: enc.encode(JSON.stringify(job)), blob };
}
function put(mod, bytes) { const p = mod._malloc(Math.max(1, bytes.length)) >>> 0; heap(mod).HEAPU8.set(bytes, p); return p; }
// MT: run cs_slice on the engine's slicing pthread and wait without blocking
// this (main runtime) thread, which Emscripten needs to start TBB's workers.
async function sliceCall(mod, args, sync) {
  if (sync || process.env.CS_SYNC === '1' || typeof mod._cs_slice_start !== 'function') return mod._cs_slice(...args);
  const st = mod._malloc(8) >>> 0;
  heap(mod).HEAP32.fill(0, st >>> 2, (st >>> 2) + 2);
  if (mod._cs_slice_start(...args, st) !== 0) throw new Error('cs_slice_start failed');
  // Node: pooled pthread Workers are unref'd and a pending waitAsync doesn't
  // hold the event loop either, so keep the process alive while we wait.
  const keepAlive = setInterval(() => {}, 1000);
  try {
    for (;;) {
      const H = heap(mod).HEAP32;
      if (Atomics.load(H, st >>> 2) !== 0) break;
      const w = Atomics.waitAsync(H, st >>> 2, 0, 500);
      if (w.async) await w.value;
    }
  } finally { clearInterval(keepAlive); }
  const rc = heap(mod).HEAP32[(st >>> 2) + 1];
  mod._free(st);
  return rc;
}
async function rawSlice(mod, jsonBytes, blob, blobLenOverride, sync = false) {
  const jp = put(mod, jsonBytes), bp = put(mod, blob), o = mod._malloc(16) >>> 0;
  heap(mod).HEAPU32.fill(0, o >>> 2, (o >>> 2) + 4);
  const rc = await sliceCall(mod, [jp, jsonBytes.length, bp, blobLenOverride ?? blob.length, o, o + 4, o + 8, o + 12], sync);
  const H = heap(mod).HEAPU32, g = H[o >>> 2] >>> 0, gl = H[(o >>> 2) + 1] >>> 0, r = H[(o >>> 2) + 2] >>> 0, rl = H[(o >>> 2) + 3] >>> 0;
  const report = rl ? JSON.parse(dec.decode(mod.HEAPU8.slice(r, r + rl))) : null;
  const gcode = gl ? dec.decode(mod.HEAPU8.slice(g, g + gl)) : '';
  if (g) mod._cs_free(g); if (r) mod._cs_free(r);
  mod._free(jp); mod._free(bp); mod._free(o);
  return { rc, report, gcode };
}
const slice = (mod, config, objects, options, sync) => { const { json, blob } = packJob(config, objects, options); return rawSlice(mod, json, blob, undefined, sync); };

function checkGcode(label, res, { minLayers = 5, maxZ } = {}) {
  ok(res.rc === 0 && res.report?.ok, `${label}: rc=${res.rc} ${res.report?.error ?? ''}`);
  if (res.rc !== 0) { if (res.report?.error) console.log('   error:', res.report.error); return; }
  const g = res.gcode;
  const layers = (g.match(/^;\s*(LAYER_CHANGE|CHANGE_LAYER)\b/gm) ?? []).length;
  let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity, x = 0, y = 0;
  for (const m of g.matchAll(/^G[123] ([^;\n]*)/gm)) {
    const xm = m[1].match(/X(-?[\d.]+)/), ym = m[1].match(/Y(-?[\d.]+)/);
    if (xm) x = +xm[1]; if (ym) y = +ym[1];
    if (/E[\d.]+/.test(m[1]) && !/E-/.test(m[1])) { minX = Math.min(minX, x); maxX = Math.max(maxX, x); minY = Math.min(minY, y); maxY = Math.max(maxY, y); }
  }
  console.log(`   extrusion XY bounds: X ${minX}..${maxX}  Y ${minY}..${maxY}`);
  if (process.env.DUMP_DIR) writeFileSync(join(process.env.DUMP_DIR, label.replace(/[^\w]+/g, '_') + '.gcode'), g);
  const extrusions = (g.match(/^G1 [^;\n]*E[\d.]+/gm) ?? []).length;
  const st = res.report.stats;
  ok(layers >= minLayers, `${label}: ${layers} layer changes, ${extrusions} extrusion moves, ${(g.length / 1e6).toFixed(2)} MB`);
  ok(st && st.printTimeSec > 0 && st.filamentMm[0] > 0, `${label}: stats time=${st?.printTimeSec?.toFixed(0)}s filament=${st?.filamentMm?.[0]?.toFixed(0)}mm layers=${st?.layers} maxZ=${st?.maxZ}`);
  if (maxZ) ok(Math.abs(st.maxZ - maxZ) < 0.6, `${label}: maxZ ${st.maxZ} ≈ ${maxZ}`);
  if (res.report.warnings?.length) console.log('   warnings:', res.report.warnings.map((w) => w.message).join(' | ').slice(0, 300));
  if (res.report.timings) console.log(`   timings: process ${res.report.timings.processMs.toFixed(0)} ms, export ${res.report.timings.exportMs.toFixed(0)} ms`);
}

// ---- run ---------------------------------------------------------------------------
const t0 = Date.now();
const mod = await instance();
const version = JSON.parse(mod.UTF8ToString(mod._cs_version()));
ok(version.engine === 'orca' && version.version === '2.5.0', `version ${JSON.stringify(version)}`);
const MT = typeof mod._cs_slice_start === 'function';
console.log(MT ? `multi-threaded build: ${mod._cs_thread_count()} threads${process.env.CS_SYNC === '1' ? ' (CS_SYNC: serial cs_slice)' : ''}` : 'single-threaded build');

let schema;
{ // schema
  const o = mod._malloc(8) >>> 0;
  const rc = mod._cs_describe_config(o, o + 4);
  const p = mod.HEAPU32[o >>> 2] >>> 0, n = mod.HEAPU32[(o >>> 2) + 1] >>> 0;
  schema = JSON.parse(dec.decode(mod.HEAPU8.slice(p, p + n))); mod._cs_free(p); mod._free(o);
  const keys = Object.keys(schema.options);
  ok(rc === 0 && keys.length > 500, `schema: ${keys.length} options`);
  ok(schema.options.layer_height?.type === 'float' && schema.options.sparse_infill_pattern?.type === 'enum'
     && schema.options.sparse_infill_pattern.enumValues.includes('gyroid') && schema.options.layer_height.scope === 'print'
     && schema.options.nozzle_diameter?.scope === 'machine' && schema.options.filament_type?.scope === 'filament',
     'schema: types/scopes/enums sane');
  ok(schema.options.outer_wall_speed?.variant === 'print' && schema.options.filament_max_volumetric_speed?.variant === 'filament'
     && schema.options.slicing_pipeline_plugin?.pluginType === 'slicing-pipeline' && schema.options.slicing_pipeline_plugin.guiType === 'plugin_picker'
     && schema.options.filament_mixed_components?.type === 'strings' && schema.options.enable_mixed_color_sublayer?.type === 'bool',
     'schema: 2.5 variant flags, plugin picker, mixed filament keys');
}

const bbl = configFor('BBL', 'Bambu Lab P1S 0.4 nozzle');
const mk4 = configFor('Prusa', 'Prusa MK4 0.4 nozzle');
console.log('profiles:', bbl.name, '||', mk4.name);

checkGcode('cube 20mm (P1S)', await slice(mod, bbl.config, [{ ...box(20, 20, 20), transform: translate(118, 118, 0) }]), { minLayers: 50, maxZ: 20 });
checkGcode('cylinder (MK4)', await slice(mod, mk4.config, [{ ...cylinder(10, 15, 96), transform: translate(125, 105, 0) }]), { minLayers: 40, maxZ: 15 });
checkGcode('multi-object rotated + per-object override (P1S)', await slice(mod, bbl.config, [
  { name: 'a', ...box(15, 30, 10), transform: rotZ(30, 90, 100, 0) },
  { name: 'b', ...cylinder(8, 25, 64), transform: translate(170, 150, 0), config: { wall_loops: '5', sparse_infill_density: '40%' } },
]), { minLayers: 60, maxZ: 25 });
checkGcode('support + brim + gyroid (P1S)', await slice(mod, { ...bbl.config, enable_support: '1', brim_type: 'outer_only', brim_width: '5', sparse_infill_pattern: 'gyroid' },
  [{ ...torus(25, 8, 96, 48), transform: translate(128, 128, 0) }]), { minLayers: 40 });
{
  const big = torus(40, 12, 500, 200); // 200k triangles
  const t = Date.now();
  checkGcode(`large mesh ${big.indices.length / 3} tris (P1S)`, await slice(mod, bbl.config, [{ ...big, transform: translate(128, 128, 0) }]), { minLayers: 60 });
  console.log(`   large mesh slice took ${((Date.now() - t) / 1000).toFixed(1)}s, heap ${(heap(mod).HEAPU8.byteLength / 1048576).toFixed(0)} MB`);
}

// Back-to-back on one instance (the web app uses fresh instances, but the
// engine must not rely on that for correctness).
{
  let same = true, first = null;
  for (let i = 0; i < 5; i++) {
    const r = await slice(mod, bbl.config, [{ ...box(20, 20, 10), transform: translate(118, 118, 0) }]);
    if (r.rc !== 0) { same = false; console.log('   repeat error', r.report?.error); break; }
    // Object ids come from libslic3r's process-global ObjectID counter, so they
    // legitimately grow between slices on one instance; normalise them.
    const body = r.gcode.replace(/^; (generated|model printing time|total estimated time).*$/gm, '')
      .replace(/(label id: |M486 [AS]|M62[45] [SE]|EXCLUDE_OBJECT[_A-Z]* NAME=)\S+/g, '$1#')
      .replace(/(id)(\s*[:=]\s*)\d+/gi, '$1$2#');
    if (first === null) first = body;
    else if (body !== first) {
      same = false;
      const a = first.split('\n'), b = body.split('\n');
      const k = a.findIndex((l, n) => l !== b[n]);
      console.log(`   run ${i + 1} differs at line ${k + 1} (${a.length} vs ${b.length} lines):\n     - ${a[k]}\n     + ${b[k]}`);
      break;
    }
  }
  ok(same, '5 back-to-back slices on one instance succeed with stable output');
}

// Malformed jobs → clean error reports, never a trap.
{
  const good = packJob(bbl.config, [{ ...box(10, 10, 10), transform: translate(100, 100, 0) }]);
  const jobObj = JSON.parse(dec.decode(good.json));
  const variant = (f) => { const j = structuredClone(jobObj); f(j); return enc.encode(JSON.stringify(j)); };
  const cases = [
    ['invalid JSON', enc.encode('{"config":'), good.blob],
    ['vertex range past blob', variant((j) => { j.objects[0].vertexOffset = 1 << 20; }), good.blob],
    ['huge vertexCount', variant((j) => { j.objects[0].vertexCount = 2 ** 31; }), good.blob],
    ['index out of range', variant((j) => { j.objects[0].vertexCount = 3; }), good.blob],
    ['misaligned offset', variant((j) => { j.objects[0].indexOffset = 97; }), good.blob],
    ['NaN transform', enc.encode(dec.decode(variant((j) => { j.objects[0].transform[12] = 1e400; }))), good.blob],
    ['no objects', variant((j) => { j.objects = []; }), good.blob],
    ['blob shorter than claimed', good.json, good.blob, 10],
    ['config wrong type', variant((j) => { j.config = [1, 2]; }), good.blob],
    ['object outside bed', variant((j) => { j.objects[0].transform[12] = 5000; }), good.blob],
  ];
  const nanMesh = box(10, 10, 10); nanMesh.positions[3] = NaN;
  const nm = packJob(bbl.config, [nanMesh]);
  cases.push(['NaN vertex', nm.json, nm.blob]);
  for (const [label, j, b, len] of cases) {
    let res, threw = null;
    try { res = await rawSlice(mod, j, b, len); } catch (e) { threw = e; }
    ok(!threw && res.rc !== 0 && typeof res.report?.error === 'string',
       `malformed: ${label} → ${threw ? 'THREW ' + threw : res.report?.error?.slice(0, 90)}`);
  }
  checkGcode('slice after malformed jobs (same instance)', await slice(mod, bbl.config, [{ ...box(20, 20, 5), transform: translate(118, 118, 0) }]), { minLayers: 10 });
  // MT: the synchronous cs_slice on the main runtime thread must still work (serially).
  if (MT) checkGcode('synchronous cs_slice on the main thread (MT, serial)', await slice(mod, bbl.config, [{ ...box(20, 20, 5), transform: translate(118, 118, 0) }], {}, true), { minLayers: 10 });
}

// Fresh instances (what the web worker does).
for (let i = 0; i < 3; i++) {
  const m = await instance();
  checkGcode(`fresh instance #${i + 1}`, await slice(m, mk4.config, [{ ...box(20, 20, 5), transform: translate(125, 105, 0) }]), { minLayers: 10 });
}

{ // auto-orient (Orca AutoOrienter via cs_orient)
  const orient = (obj) => {
    const { json, blob } = packJob({}, [obj]);
    const jp = put(mod, json), bp = put(mod, blob), o = mod._malloc(8) >>> 0;
    mod.HEAPU32.fill(0, o >>> 2, (o >>> 2) + 2);
    const rc = mod._cs_orient(jp, json.length, bp, blob.length, o, o + 4);
    const p = heap(mod).HEAPU32[o >>> 2] >>> 0, n = mod.HEAPU32[(o >>> 2) + 1] >>> 0;
    const r = JSON.parse(dec.decode(mod.HEAPU8.slice(p, p + n)));
    mod._cs_free(p); mod._free(jp); mod._free(bp); mod._free(o);
    return { rc, ...r };
  };
  // Box 40x20x5 tilted 30deg about X: best orientation puts a big face down.
  const c = Math.cos(Math.PI / 6), s = Math.sin(Math.PI / 6);
  const tilt = [1,0,0,0, 0,c,s,0, 0,-s,c,0, 0,0,0,1]; // column-major
  const r = orient({ ...box(40, 20, 5), transform: tilt });
  ok(r.rc === 0 && r.ok && r.rotation?.length === 9, `cs_orient ok (angle ${r.angle?.toFixed?.(3)} rad)`);
  if (r.ok) {
    // Apply R (row-major) after the tilt to the box's local +Z face normal; it must end up ±Z.
    const n0 = [0, -s, c]; // tilt * (0,0,1)
    const R = r.rotation, n1 = [R[0]*n0[0]+R[1]*n0[1]+R[2]*n0[2], R[3]*n0[0]+R[4]*n0[1]+R[5]*n0[2], R[6]*n0[0]+R[7]*n0[1]+R[8]*n0[2]];
    ok(Math.abs(Math.abs(n1[2]) - 1) < 1e-3, `auto-orient lays the large face flat (normal z=${n1[2].toFixed(4)})`);
  }
  const bad = orient({ ...box(1, 1, 1), transform: tilt });
  ok(typeof bad.ok === 'boolean', 'cs_orient on a tiny mesh returns a report');
}

// ---- Orca 2.5: extruder variants (per-nozzle-type vectors) -----------------------------
{
  const h2s = configFor('BBL', 'Bambu Lab H2S 0.4 nozzle', '0.20mm Standard @BBL H2S', 'Bambu PLA Basic @BBL H2S');
  const wallSpeed = (g) => +(g.match(/^; outer_wall_speed = ([\d.]+)/m)?.[1] ?? NaN);
  for (const [nvt, want] of [['Standard', 200], ['High Flow', 500]]) {
    const r = await slice(mod, { ...h2s.config, nozzle_volume_type: [nvt] }, [{ ...box(20, 20, 5), transform: translate(160, 160, 0) }]);
    checkGcode(`H2S, ${nvt} nozzle`, r, { minLayers: 10 });
    ok(wallSpeed(r.gcode) === want, `H2S ${nvt}: outer_wall_speed resolves to ${wallSpeed(r.gcode)} (want ${want})`);
  }
  // Dual extruder: each extruder takes its own variant's element.
  const h2d = configFor('BBL', 'Bambu Lab H2D 0.4 nozzle', '0.20mm Standard @BBL H2D', 'Bambu PLA Basic @BBL H2D');
  const r = await slice(mod, { ...h2d.config, nozzle_volume_type: ['High Flow', 'Standard'] }, [{ ...box(20, 20, 5), transform: translate(160, 160, 0) }]);
  checkGcode('H2D, left High Flow / right Standard', r, { minLayers: 10 });
  const line = r.gcode.match(/^; outer_wall_speed = (.*)$/m)?.[1];
  ok(line === '500,200', `H2D: outer_wall_speed per extruder = ${line} (want 500,200)`);
}

// ---- Orca 2.5: Hybrid nozzle rack (H2C), Manual / Nozzle Manual grouping ----------------------
// Desktop's full_config(false, f_maps, f_volume_maps) shape: each filament's variant vectors
// concatenated (3 columns per H2C filament, from its `include` template), filament_volume_map per
// filament, and extruder_nozzle_stats (seeded by desktop, split per type for a Hybrid rack).
function multiFilament(cfg, colors) {
  const n = colors.length, out = { ...cfg };
  const nv = cfg.filament_extruder_variant?.length ?? 1;
  for (const [k, v] of Object.entries(cfg))
    if (schema.options[k]?.scope === 'filament' && Array.isArray(v) && k !== 'filament_self_index')
      out[k] = Array.from({ length: n }, () => v).flat();
  out.filament_self_index = Array.from({ length: n }, (_, i) => Array(nv).fill(String(i + 1))).flat();
  out.filament_colour = colors;
  return out;
}
async function toolCall(mod, op, args) {
  const req = enc.encode(JSON.stringify({ op, args }));
  const rp = put(mod, req), o = mod._malloc(16) >>> 0;
  heap(mod).HEAPU32.fill(0, o >>> 2, (o >>> 2) + 4);
  mod._cs_tool(rp, req.length, 0, 0, o, o + 4, o + 8, o + 12);
  const H = heap(mod).HEAPU32, p = H[o >>> 2] >>> 0, l = H[(o >>> 2) + 1] >>> 0, b = H[(o >>> 2) + 2] >>> 0;
  const r = JSON.parse(dec.decode(mod.HEAPU8.slice(p, p + l)));
  mod._cs_free(p); if (b) mod._cs_free(b); mod._free(rp); mod._free(o);
  return r;
}
{
  const h2c = configFor('BBL', 'Bambu Lab H2C 0.4 nozzle', '0.20mm Standard @BBL H2C', 'Bambu PLA Basic @BBL H2C');
  ok(h2c.config.filament_extruder_variant?.length === 3, `H2C filament: include template gives 3 variant columns (${h2c.config.filament_extruder_variant})`);
  const base = { ...multiFilament(h2c.config, ['#FF0000', '#00FF00', '#0000FF']),
    filament_map_mode: 'Manual', filament_map: ['1', '2', '2'], nozzle_volume_type: ['Standard', 'Hybrid'], filament_volume_map: ['0', '0', '1'],
    extruder_nozzle_stats: ['Standard#1', 'Standard#3|High Flow#3'],
    flush_multiplier: ['1', '1'], flush_volumes_matrix: Array(18).fill('100'), wipe_tower_x: ['200'], wipe_tower_y: ['220'] };
  const objs = [0, 1, 2].map((f) => ({ ...box(15, 15, 4), transform: translate(100 + f * 30, 150, 0), config: { extruder: String(f + 1) } }));
  const field = (g, k) => g.match(new RegExp(`^; ${k} = (.*)$`, 'm'))?.[1];

  const rv = await toolCall(mod, 'resolve_variants', { config: base });
  ok(rv.ok && rv.result.filament.filament_extruder_variant?.[2] === 'Direct Drive High Flow' && rv.result.filament.filament_max_volumetric_speed?.join() === '25,25,40',
     `H2C resolve_variants: filament 3 → High Flow column (${rv.result?.filament?.filament_extruder_variant}; ${rv.result?.filament?.filament_max_volumetric_speed}) ${rv.error ?? ''}`);

  let r = await slice(mod, base, objs, { validate: true });
  checkGcode('H2C Hybrid rack, Manual 1,2,2 / volumes 0,0,1', r, { minLayers: 10 });
  if (r.rc === 0) {
    ok(field(r.gcode, 'filament_volume_map') === '0,0,1', `H2C: filament_volume_map written back = ${field(r.gcode, 'filament_volume_map')} (want 0,0,1)`);
    const nmap = field(r.gcode, 'filament_nozzle_map')?.split(',').map(Number) ?? [];
    // nozzle list: extruder 1 Standard (0), extruder 2 Standard x3 (1..3) then High Flow x3 (4..6)
    ok(nmap.length === 3 && nmap[0] === 0 && nmap[1] >= 1 && nmap[1] <= 3 && nmap[2] >= 4 && nmap[2] <= 6, `H2C: filament_nozzle_map = ${nmap} (filament 3 on a High Flow nozzle)`);
    ok(field(r.gcode, 'filament_extruder_variant') === '"Direct Drive Standard";"Direct Drive Standard";"Direct Drive High Flow"',
       `H2C: filament_extruder_variant = ${field(r.gcode, 'filament_extruder_variant')}`);
    ok(field(r.gcode, 'filament_max_volumetric_speed') === '25,25,40', `H2C: filament_max_volumetric_speed = ${field(r.gcode, 'filament_max_volumetric_speed')} (want 25,25,40)`);
  }
  // Nozzle Manual: the host picks the physical nozzle per filament.
  r = await slice(mod, { ...base, filament_map_mode: 'Nozzle Manual', filament_nozzle_map: ['0', '1', '2'] }, objs, { validate: true });
  ok(r.rc === 0 && field(r.gcode, 'filament_volume_map') === '0,0,1' && (field(r.gcode, 'filament_nozzle_map')?.split(',').map(Number)[2] ?? -1) >= 4,
     `H2C Nozzle Manual: volumes ${field(r.gcode, 'filament_volume_map')}, nozzles ${field(r.gcode, 'filament_nozzle_map')} ${r.report?.error ?? ''}`);

  // Inputs the bridge rejects with a message naming what to send.
  const bad = async (label, cfg, re) => {
    const b = await slice(mod, cfg, objs, { validate: true });
    ok(b.rc !== 0 && re.test(b.report?.error ?? ''), `H2C rejects ${label}: ${b.report?.error?.slice(0, 160)}`);
  };
  const { extruder_nozzle_stats: _s, ...noStats } = base;
  await bad('Hybrid without extruder_nozzle_stats', noStats, /Hybrid.*extruder_nozzle_stats/);
  await bad('a malformed stats token', { ...base, extruder_nozzle_stats: ['Standard#1', 'Standard:3'] }, /bad token/);
  await bad('more nozzles than the rack holds', { ...base, extruder_nozzle_stats: ['Standard#1', 'Standard#4|High Flow#3'] }, /at most 6/);
  await bad('a mis-sized filament_volume_map', { ...base, filament_volume_map: ['0', '1'] }, /filament_volume_map must have one entry per filament/);
  await bad('Nozzle Manual without filament_nozzle_map', { ...base, filament_map_mode: 'Nozzle Manual' }, /filament_nozzle_map/);
  // The app's shape before it applied `include`: one "Direct Drive Standard" name per filament over 3 columns.
  await bad('variant vectors wider than filament_extruder_variant',
            { ...base, filament_extruder_variant: Array(3).fill('Direct Drive Standard'), filament_self_index: ['1', '2', '3'] }, /include/);
}

// ---- Orca 2.5: native mixed filaments (plain config, desktop project_settings shape) ------------
// Physical slots first, mixed slots at the tail; every per-filament vector has an entry for the
// mixed slot (desktop copies its first component's preset); flush matrix over all slots with zero
// rows/columns for the mixed slot.
function withMixed(cfg, colors, mixes) {
  const n = colors.length, all = n + mixes.length, out = { ...cfg };
  // Each slot gets the whole filament preset: its variant vectors (one value per variant) concatenated.
  const nv = cfg.filament_extruder_variant?.length ?? 1;
  for (const [k, v] of Object.entries(cfg))
    if (schema.options[k]?.scope === 'filament' && Array.isArray(v) && k !== 'filament_self_index' && (v.length === 1 || schema.options[k]?.variant === 'filament' || k === 'filament_extruder_variant'))
      out[k] = Array.from({ length: all }, () => v).flat();
  out.filament_self_index = Array.from({ length: all }, (_, i) => Array(nv).fill(String(i + 1))).flat();
  out.filament_colour = [...colors, ...mixes.map((m) => m.color)];
  out.filament_map = Array(all).fill('1');
  out.flush_volumes_matrix = Array.from({ length: all * all }, (_, q) => {
    const i = Math.floor(q / all), j = q % all;
    return i === j || i >= n || j >= n ? '0' : '200';
  });
  out.filament_is_mixed = [...colors.map(() => '0'), ...mixes.map(() => '1')];
  out.filament_mixed_components = [...colors.map(() => ''), ...mixes.map((m) => m.components)];
  out.filament_mixed_sublayer_ratios = [...colors.map(() => ''), ...mixes.map((m) => m.ratios)];
  out.filament_mixed_gradient = [...colors.map(() => '0'), ...mixes.map((m) => (m.range ? '1' : '0'))];
  out.filament_mixed_gradient_range = [...colors.map(() => ''), ...mixes.map((m) => m.range ?? '')];
  out.filament_mixed_gradient_curve = [...colors.map(() => ''), ...mixes.map(() => '')];
  out.filament_mixed_gradient_per_part = [...colors.map(() => '0'), ...mixes.map(() => '0')];
  return out;
}
{
  const cfg = { ...withMixed(bbl.config, ['#FF0000', '#0000FF'], [{ components: '1,2', ratios: '0.5000,0.5000', range: '0.1000,0.9000', color: '#7F007F' }]),
                enable_mixed_color_sublayer: '1', enable_prime_tower: '1' };
  const r = await slice(mod, cfg, [{ ...box(20, 20, 8), transform: translate(118, 118, 0), config: { extruder: '3' } }]);
  checkGcode('mixed filament 1+2 with Z gradient, object on slot 3 (P1S + AMS)', r, { minLayers: 20 });
  const fm = r.report?.stats?.filamentMm ?? [];
  ok(fm[0] > 0 && fm[1] > 0, `both physical filaments print the mixed slot (${fm.map((x) => x.toFixed(0)).join(', ')} mm)`);
  // A gradient: range "0.1,0.9" = filament 1's share rises from ~10% at the bottom to ~90% at the
  // top. Count the object's extrusions only (prime-tower purges are the same for every change).
  const lay = r.gcode.split(/^;\s*(?:LAYER_CHANGE|CHANGE_LAYER)\b/m), share = (txt) => {
    let t = -1, feat = '', e = [0, 0];
    for (const line of txt.split('\n')) {
      let m;
      if ((m = line.match(/^T(\d+)/))) t = +m[1];
      else if ((m = line.match(/^; FEATURE: (.*)/))) feat = m[1];
      else if ((m = line.match(/^G1 [^;]*E([\d.]+)/)) && t >= 0 && t < 2 && feat !== 'Prime tower' && feat !== 'Custom') e[t] += +m[1];
    }
    return e[0] + e[1] > 0 ? e[0] / (e[0] + e[1]) : NaN;
  };
  const lo = share(lay.slice(3, 8).join('')), hi = share(lay.slice(-8, -3).join(''));
  ok(hi > lo + 0.3, `gradient: filament 1 share ${lo.toFixed(2)} low → ${hi.toFixed(2)} high`);
  const bad = await slice(mod, { ...cfg, filament_mixed_components: ['', '', '1,9'] }, [{ ...box(20, 20, 8), transform: translate(118, 118, 0), config: { extruder: '3' } }]);
  ok(bad.rc !== 0 && /mixed filament/i.test(bad.report?.error ?? ''), `invalid mixed filament rejected: ${bad.report?.error}`);
}

// ---- Orca 2.5: Precise Seam modifier parts --------------------------------------------------
{
  const cube = box(20, 20, 10), seam = box(4, 4, 12);
  const size = cube.positions.byteLength + cube.indices.byteLength;
  const blob = new Uint8Array(size + seam.positions.byteLength + seam.indices.byteLength);
  blob.set(new Uint8Array(cube.positions.buffer), 0); blob.set(new Uint8Array(cube.indices.buffer), cube.positions.byteLength);
  blob.set(new Uint8Array(seam.positions.buffer), size); blob.set(new Uint8Array(seam.indices.buffer), size + seam.positions.byteLength);
  const part = (type) => ({ name: 'seam', type, vertexOffset: size, vertexCount: 8, indexOffset: size + seam.positions.byteLength, triangleCount: 12,
    transform: translate(136, 136, -1), config: {} });
  const jobFor = (type) => enc.encode(JSON.stringify({ config: bbl.config, objects: [{ name: 'cube', vertexOffset: 0, vertexCount: 8,
    indexOffset: cube.positions.byteLength, triangleCount: 12, transform: translate(118, 118, 0), parts: [part(type)] }] }));
  for (const t of ['precise_seam_center', 'precise_seam_enforced', 'precise_seam_blocked'])
    checkGcode(`precise seam part (${t})`, await rawSlice(mod, jobFor(t), blob), { minLayers: 20 });
  const bad = await rawSlice(mod, jobFor('precise_seam_sideways'), blob);
  ok(bad.rc !== 0 && /unknown part type/.test(bad.report?.error ?? ''), `unknown precise seam subtype rejected: ${bad.report?.error}`);
}

// ---- Orca 2.5: slicing-pipeline plugin hooks (Module.csHook + cs_hook_query) ------------------
function hookQuery(m, req, reqBlob = new Uint8Array(0)) {
  const j = enc.encode(JSON.stringify(req));
  const jp = put(m, j), bp = put(m, reqBlob), o = m._malloc(16) >>> 0;
  heap(m).HEAPU32.fill(0, o >>> 2, (o >>> 2) + 4);
  const rc = m._cs_hook_query(jp, j.length, bp, reqBlob.length, o, o + 4, o + 8, o + 12);
  const H = heap(m).HEAPU32, p = H[o >>> 2] >>> 0, n = H[(o >>> 2) + 1] >>> 0, b = H[(o >>> 2) + 2] >>> 0, bl = H[(o >>> 2) + 3] >>> 0;
  const rep = n ? JSON.parse(dec.decode(m.HEAPU8.slice(p, p + n))) : null;
  const bytes = bl ? m.HEAPU8.slice(b, b + bl) : new Uint8Array(0);
  if (p) m._cs_free(p); if (b) m._cs_free(b); m._free(jp); m._free(bp); m._free(o);
  return { rc, rep, bytes };
}
{
  const hm = await instance();
  ok(hookQuery(hm, { op: 'print.info' }).rc === -1, 'cs_hook_query outside a hook → -1');
  const steps = [];
  let shrunk = 0, mode = 'shrink';
  hm.csHook = (kind, reqJson) => {
    const begin = JSON.parse(reqJson);
    steps.push(begin.step);
    if (mode === 'fail' && begin.step === 'posPerimeters') return { status: 'FatalError', capability: 'boom', message: 'plugin said no' };
    if (begin.step !== 'posSlice' || mode !== 'shrink') return { status: 'Success', capability: 'shrink' };
    const id = begin.objectId;
    const layers = hookQuery(hm, { op: 'object.layers', objectId: id }).rep.results[0].layers;
    // 1) read every layer's region-0 slices, 2) offset them by -2 mm with libslic3r's Clipper,
    // 3) write them back (the geom results' blob is the write's request blob), 4) re-derive lslices.
    const read = hookQuery(hm, { batch: layers.map((_, l) => ({ op: 'layer.surfaces', objectId: id, layer: l, region: 0, which: 'slices' })) });
    const delta = -2 / begin.scalingFactor;
    const geo = hookQuery(hm, { batch: read.rep.results.map((r) => ({ op: 'geom.offset_ex', a: r.surfaces, delta, blobOffset: r.blobOffset })) }, read.bytes);
    const type = read.rep.results[0].surfaces[0]?.type ?? 0;
    const write = hookQuery(hm, { batch: geo.rep.results.flatMap((g, l) => [
      { op: 'surfaces.set', objectId: id, layer: l, region: 0, which: 'slices', type, expolygons: g.expolygons, blobOffset: g.blobOffset },
      { op: 'layer.make_slices', objectId: id, layer: l }]) }, geo.bytes);
    if (write.rc === 0) shrunk = layers.length;
    return { status: 'Success', capability: 'shrink', message: `shrunk ${shrunk} layers` };
  };
  // Width of the object's own extrusions (the start G-code's purge line sits far left of it).
  const bounds = (g) => { let lo = Infinity, hi = -Infinity, x = 0;
    for (const m of g.matchAll(/^G1 ([^;\n]*)/gm)) { const xm = m[1].match(/X(-?[\d.]+)/); if (xm) x = +xm[1]; if (/E[\d.]+/.test(m[1]) && x > 90 && x < 170) { lo = Math.min(lo, x); hi = Math.max(hi, x); } } return hi - lo; };
  const obj = [{ ...box(30, 30, 6), transform: translate(113, 113, 0) }];
  const plain = await slice(hm, { ...bbl.config, skirt_loops: '0', brim_type: 'no_brim' }, obj);
  ok(plain.rc === 0 && steps.length === 0, 'no hooks without slicing_pipeline_plugin (inert)');
  const hooked = await slice(hm, { ...bbl.config, skirt_loops: '0', brim_type: 'no_brim', slicing_pipeline_plugin: ['shrink'] }, obj);
  checkGcode('pipeline hook edits layer slices', hooked, { minLayers: 20 });
  const w0 = bounds(plain.gcode), w1 = bounds(hooked.gcode);
  ok(shrunk > 20 && Math.abs((w0 - w1) - 4) < 0.6, `posSlice hook shrank ${shrunk} layers: width ${w0.toFixed(2)} → ${w1.toFixed(2)} mm`);
  const want = ['posSlice', 'posPerimeters', 'posPrepareInfill', 'posInfill', 'psWipeTower', 'psSkirtBrim'];
  ok(want.every((s) => steps.includes(s)), `hook steps fired: ${[...new Set(steps)].join(', ')}`);
  ok(hooked.report.warnings.some((w) => w.code === 'plugin' && /shrunk/.test(w.message)), 'hook message reported as a plugin warning');
  const ev = (hooked.report.events ?? []).map((e) => e.event);
  ok(['SliceStarted', 'SliceGeometryFinished', 'GCodeExportStarted', 'GCodeExportFinished'].every((e) => ev.includes(e)), `lifecycle events: ${ev.join(', ')}`);
  mode = 'fail';
  const failed = await slice(hm, { ...bbl.config, slicing_pipeline_plugin: ['boom'] }, obj);
  ok(failed.rc !== 0 && /Slicing pipeline plugin 'boom' error: plugin said no/.test(failed.report?.error ?? ''), `FatalError → slice error: ${failed.report?.error}`);
  hm.csHook = () => { throw new Error('python blew up'); };
  const threw = await slice(hm, { ...bbl.config, slicing_pipeline_plugin: ['x'] }, obj);
  ok(threw.rc !== 0 && /python blew up/.test(threw.report?.error ?? ''), `csHook exception → slice error: ${threw.report?.error}`);
  mode = 'shrink';
  hm.csHook = undefined;
  checkGcode('slice after hook failures (same instance)', await slice(hm, bbl.config, obj), { minLayers: 20 });
}

{ // conditions
  const ev = (expr, cfg) => { const e = enc.encode(expr), c = enc.encode(JSON.stringify(cfg)); const ep = put(mod, e), cp = put(mod, c);
    const r = mod._cs_eval_condition(ep, e.length, cp, c.length); mod._free(ep); mod._free(cp); return r; };
  ok(ev('nozzle_diameter[0]==0.4', mk4.config) === 1, 'condition true');
  ok(ev('nozzle_diameter[0]==0.6', mk4.config) === 0, 'condition false');
  ok(ev('printer_notes=~/.*PRINTER_VENDOR_PRUSA3D.*/', { printer_notes: 'PRINTER_VENDOR_PRUSA3D' }) === 1, 'condition regex');
  ok(ev('this is ((( not valid', {}) < 0, 'condition parse error → <0');
}

console.log(`\n${failures ? `${failures} FAILED` : 'ALL PASSED'} in ${((Date.now() - t0) / 1000).toFixed(1)}s`);
for (const m of instances) m.csTerminateThreads?.();
process.exit(failures ? 1 : 0);
