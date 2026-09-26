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
function resolvePreset(vendorId, list, name) {
  const chain = [];
  let cur = name, vid = vendorId, guard = 0;
  while (cur && guard++ < 32) {
    const p = findPreset(vid, list, cur);
    if (!p) throw new Error(`preset not found: ${cur}`);
    chain.unshift(p.data); vid = p.vid;
    cur = p.data.inherits;
  }
  const out = {};
  for (const e of chain) for (const [k, v] of Object.entries(e)) {
    if (['inherits', 'instantiation', 'from', 'setting_id', 'name', 'type', 'filament_id', 'version', 'description', 'renamed_from'].includes(k)) continue;
    out[k] = v;
  }
  return out;
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
                     ...resolvePreset(vendorId, 'filament_list', filament) } };
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
ok(version.engine === 'orca' && version.version.startsWith('2.4'), `version ${JSON.stringify(version)}`);
const MT = typeof mod._cs_slice_start === 'function';
console.log(MT ? `multi-threaded build: ${mod._cs_thread_count()} threads${process.env.CS_SYNC === '1' ? ' (CS_SYNC: serial cs_slice)' : ''}` : 'single-threaded build');

{ // schema
  const o = mod._malloc(8) >>> 0;
  const rc = mod._cs_describe_config(o, o + 4);
  const p = mod.HEAPU32[o >>> 2] >>> 0, n = mod.HEAPU32[(o >>> 2) + 1] >>> 0;
  const schema = JSON.parse(dec.decode(mod.HEAPU8.slice(p, p + n))); mod._cs_free(p); mod._free(o);
  const keys = Object.keys(schema.options);
  ok(rc === 0 && keys.length > 500, `schema: ${keys.length} options`);
  ok(schema.options.layer_height?.type === 'float' && schema.options.sparse_infill_pattern?.type === 'enum'
     && schema.options.sparse_infill_pattern.enumValues.includes('gyroid') && schema.options.layer_height.scope === 'print'
     && schema.options.nozzle_diameter?.scope === 'machine' && schema.options.filament_type?.scope === 'filament',
     'schema: types/scopes/enums sane');
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
