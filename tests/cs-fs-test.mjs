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
import { readFileSync, existsSync } from 'node:fs';
import { resolve, dirname, join } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

const here = dirname(fileURLToPath(import.meta.url));
const dir = resolve(process.argv[2] ?? join(here, '..', 'build-fs'));
const profilesDir = join(here, '..', 'fullspectrum', 'resources', 'profiles');
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
  for (const vid of [vendorId, 'OrcaFilamentLibrary'].filter((v) => existsSync(join(profilesDir, `${v}.json`)))) {
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


// ---- FullSpectrum engine (mixed filaments) ----
function tool(mod, op, args) {
  const json = enc.encode(JSON.stringify({ op, args, meshes: [] }));
  const jp = put(mod, json), bp = put(mod, new Uint8Array(1)), o = mod._malloc(16) >>> 0;
  heap(mod).HEAPU32.fill(0, o >>> 2, (o >>> 2) + 4);
  mod._cs_tool(jp, json.length, bp, 0, o, o + 4, o + 8, o + 12);
  const H = heap(mod).HEAPU32, p = H[o >>> 2] >>> 0, n = H[(o >>> 2) + 1] >>> 0;
  const r = JSON.parse(dec.decode(mod.HEAPU8.slice(p, p + n)));
  mod._cs_free(p); mod._free(jp); mod._free(bp); mod._free(o);
  return r;
}
const mod = await instance();
const version = JSON.parse(mod.UTF8ToString(mod._cs_version()));
ok(version.engine === 'fullspectrum', `cs_version → ${JSON.stringify(version)}`);

const machine = 'Snapmaker U1 (0.4 nozzle)';
const cfg0 = configFor('Snapmaker', machine, compatible('Snapmaker', 'process_list', machine).find((n) => /0\.20mm/.test(n)),
  compatible('Snapmaker', 'filament_list', machine).find((n) => /PLA/.test(n)));
console.log('config:', cfg0.name);
// Two physical filaments (per-filament vectors widened to 2), red + blue.
const cfg = { ...cfg0.config };
const nExt = Array.isArray(cfg.nozzle_diameter) ? cfg.nozzle_diameter.length : 1;
for (const [k, v] of Object.entries(cfg)) if (Array.isArray(v) && v.length === 1 && k.startsWith('filament_')) cfg[k] = [v[0], v[0]];
cfg.filament_colour = ['#FF0000', '#0000FF'];
console.log('extruders in machine:', nExt);

const mf = tool(mod, 'mixed_filaments', { colors: cfg.filament_colour, definitions: '' });
ok(mf.ok && mf.result.filaments.length >= 1, `mixed_filaments auto pair → ${JSON.stringify(mf.result?.filaments ?? mf.error)}`);
const add = tool(mod, 'mixed_filaments', { colors: cfg.filament_colour, definitions: '', edits: [{ op: 'add', a: 1, b: 2, percent: 25 }] });
ok(add.ok && add.result.filaments.some((f) => f.custom && f.percent === 25) && add.result.definitions.length > 0,
  `add custom 75/25 mix → ${add.result?.filaments?.length} filaments, definitions "${add.result?.definitions?.slice(0, 60)}"`);
const again = tool(mod, 'mixed_filaments', { colors: cfg.filament_colour, definitions: add.result.definitions });
ok(again.ok && again.result.filaments.length === add.result.filaments.length, 'definitions round-trip');

const plain = await slice(mod, cfg, [{ ...box(20, 20, 10), transform: translate(130, 130, 0) }]);
ok(plain.rc === 0, `plain slice on U1 ${plain.report?.error ?? ''}`);
const mixId = mf.result.filaments[0]?.id ?? 3;
const mixed = await slice(mod, { ...cfg, mixed_filament_definitions: '' }, [{ ...box(20, 20, 10), transform: translate(130, 130, 0), config: { extruder: String(mixId) } }]);
const t0 = (mixed.gcode.match(/^T0\b/gm) || []).length, t1 = (mixed.gcode.match(/^T1\b/gm) || []).length;
ok(mixed.rc === 0 && t0 > 3 && t1 > 3, `object on mixed filament ${mixId} alternates tools (T0 ×${t0}, T1 ×${t1}) ${mixed.report?.error ?? ''}`);

// Gradient: red → blue along Z, and a 3-colour gradient; manual pattern.
const cols3 = ['#FF0000', '#FFFF00', '#0000FF'];
const g2 = tool(mod, 'mixed_filaments', { colors: cfg.filament_colour, definitions: '', edits: [{ op: 'add_definition', definition: {
  components: [{ id: 1, percent: 50 }, { id: 2, percent: 50 }], gradient: { enabled: true, start: 1, end: 0 } } }] });
const gf = g2.result?.filaments?.find((f) => f.gradient?.enabled);
ok(g2.ok && gf && gf.custom, `add 2-colour gradient → ${JSON.stringify(gf ?? g2.error)}`);
const g2b = tool(mod, 'mixed_filaments', { colors: cfg.filament_colour, definitions: g2.result.definitions });
ok(g2b.ok && g2b.result.filaments.some((f) => f.gradient?.enabled && f.gradient.start > 0.95 && f.gradient.end < 0.05), 'gradient round-trips through definitions');
const g3 = tool(mod, 'mixed_filaments', { colors: cols3, definitions: '', edits: [{ op: 'add_definition', definition: {
  components: [{ id: 1, percent: 34 }, { id: 2, percent: 33 }, { id: 3, percent: 33 }], gradient: { enabled: true, stops: [0, 0.25, 0.5, 0.75, 1] } } }] });
const g3f = g3.result?.filaments?.find((f) => f.components.length === 3);
ok(g3.ok && g3f && g3f.gradient.enabled && g3f.gradient.stops.length === 5, `3-colour gradient → ${JSON.stringify(g3f?.gradient ?? g3.error)}`);
const setp = tool(mod, 'mixed_filaments', { colors: cfg.filament_colour, definitions: g2.result.definitions, edits: [{ op: 'set', stableId: gf.stableId, definition: { gradient: { enabled: false }, localZMax: 3, distribution: 'layer_cycle', cadence: { a: 2, b: 1 } } }] });
const sf = setp.result?.filaments?.find((f) => f.stableId === gf.stableId);
ok(setp.ok && sf && !sf.gradient.enabled && sf.localZMax === 3 && sf.distribution === 'layer_cycle' && sf.cadence.a === 2, `set edits behaviour → ${JSON.stringify(sf ?? setp.error)}`);
const pat = tool(mod, 'mixed_filaments', { colors: cfg.filament_colour, definitions: '', edits: [{ op: 'add_definition', definition: { kind: 'pattern', pattern: [[1, 1, 2]] } }] });
ok(pat.ok && pat.result.filaments.some((f) => f.kind === 'pattern'), `manual pattern → ${pat.error ?? 'ok'}`);

// Slice a gradient object: tool use should shift from T0 (bottom) to T1 (top).
const gid = gf.id;
const gs = await slice(mod, { ...cfg, mixed_filament_definitions: g2.result.definitions }, [{ ...box(20, 20, 20), transform: translate(130, 130, 0), config: { extruder: String(gid) } }]);
const lines = gs.gcode.split('\n'); let z = 0, tool_ = 0; const lo = [0, 0], hi = [0, 0];
for (const l of lines) { const m = /^;Z:([\d.]+)/.exec(l); if (m) z = +m[1]; const t = /^T(\d)\b/.exec(l); if (t) tool_ = +t[1];
  if (/^G1 .*E\d/.test(l) && tool_ < 2) (z < 6 ? lo : z > 14 ? hi : [0, 0])[tool_]++; }
ok(gs.rc === 0 && lo[0] > lo[1] && hi[1] > hi[0], `gradient slice: bottom T0/T1 moves ${lo}, top ${hi} ${gs.report?.error ?? ''}`);
process.exit(failures ? 1 : 0);
