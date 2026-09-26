#!/usr/bin/env node
// ST vs MT benchmark (node). Interleaves the builds so machine noise hits both.
//   node tests/cs-bench.mjs [stDir=build-wasm] [mtDir=build-wasm-mt] [reps=2]
// Jobs: 4-object plate (P1S) and a 200k-triangle torus (P1S). MT runs through
// cs_slice_start (non-blocking, like the web worker). CS_THREADS=n for MT.
import { readFileSync } from 'node:fs';
import { resolve, dirname, join } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

const here = dirname(fileURLToPath(import.meta.url));
const [stDir, mtDir] = [process.argv[2] ?? 'build-wasm', process.argv[3] ?? 'build-wasm-mt'].map((d) => resolve(here, '..', d));
const reps = +(process.argv[4] ?? 2);
const profilesDir = join(here, '..', 'orca', 'resources', 'profiles');
const enc = new TextEncoder(), dec = new TextDecoder();

async function load(dir) {
  const { default: factory } = await import(pathToFileURL(join(dir, 'slicer.mjs')).href);
  const wasmBinary = readFileSync(join(dir, 'slicer.wasm'));
  const t = performance.now();
  const mod = await factory({ wasmBinary, locateFile: (f) => join(dir, f), print() {}, printErr() {},
    ...(process.env.CS_THREADS ? { csThreads: +process.env.CS_THREADS } : {}) });
  return { mod, initMs: performance.now() - t };
}

// ---- profiles (same resolution as the web catalog) ----
const vendor = (id) => JSON.parse(readFileSync(join(profilesDir, `${id}.json`), 'utf8'));
function resolvePreset(vid, list, name) {
  const chain = [];
  for (let cur = name, g = 0; cur && g < 32; g++) {
    let hit = null;
    for (const v of [vid, 'OrcaFilamentLibrary']) {
      const e = vendor(v)[list]?.find((x) => x.name === cur);
      if (e) { hit = { v, data: JSON.parse(readFileSync(join(profilesDir, v, e.sub_path), 'utf8')) }; break; }
    }
    if (!hit) throw new Error(`preset not found: ${cur}`);
    chain.unshift(hit.data); vid = hit.v; cur = hit.data.inherits;
  }
  const out = {};
  for (const e of chain) for (const [k, v] of Object.entries(e))
    if (!['inherits', 'instantiation', 'from', 'setting_id', 'name', 'type', 'filament_id', 'version', 'description', 'renamed_from'].includes(k)) out[k] = v;
  return out;
}
const config = { ...resolvePreset('BBL', 'machine_list', 'Bambu Lab P1S 0.4 nozzle'),
  ...resolvePreset('BBL', 'process_list', '0.20mm Standard @BBL X1C'), ...resolvePreset('BBL', 'filament_list', 'Bambu PLA Basic @BBL X1C') };

// ---- meshes ----
function box(sx, sy, sz) {
  return { positions: new Float32Array([0,0,0, sx,0,0, sx,sy,0, 0,sy,0, 0,0,sz, sx,0,sz, sx,sy,sz, 0,sy,sz]),
           indices: new Uint32Array([0,2,1, 0,3,2, 4,5,6, 4,6,7, 0,1,5, 0,5,4, 1,2,6, 1,6,5, 2,3,7, 2,7,6, 3,0,4, 3,4,7]) };
}
function torus(R, r, nu, nv) {
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
const T = (x, y) => [1,0,0,0, 0,1,0,0, 0,0,1,0, x,y,0,1];
const jobs = {
  'plate 4 objects': [{ ...box(25, 25, 30), transform: T(60, 60) }, { ...torus(18, 6, 120, 60), transform: T(180, 70) },
                      { ...box(40, 15, 20), transform: T(70, 170) }, { ...torus(15, 8, 96, 48), transform: T(180, 180) }],
  'torus 200k tris': [{ ...torus(40, 12, 500, 200), transform: T(128, 128) }],
};

function pack(objects) {
  let size = 0; const lay = objects.map((o) => { const v = size; size += o.positions.byteLength; const i = size; size += o.indices.byteLength; return { v, i }; });
  const blob = new Uint8Array(size);
  objects.forEach((o, n) => { blob.set(new Uint8Array(o.positions.buffer), lay[n].v); blob.set(new Uint8Array(o.indices.buffer), lay[n].i); });
  const job = { config, options: { validate: true, dropToBed: true }, objects: objects.map((o, n) => ({ name: `obj${n}`, vertexOffset: lay[n].v,
    vertexCount: o.positions.length / 3, indexOffset: lay[n].i, triangleCount: o.indices.length / 3, transform: o.transform, config: {} })) };
  return { json: enc.encode(JSON.stringify(job)), blob };
}

async function slice(mod, objects) {
  const { json, blob } = pack(objects);
  const put = (b) => { const p = mod._malloc(Math.max(1, b.length)) >>> 0; mod.csSyncHeap?.(); mod.HEAPU8.set(b, p); return p; };
  const jp = put(json), bp = put(blob), o = mod._malloc(16) >>> 0;
  mod.csSyncHeap?.(); mod.HEAPU32.fill(0, o >>> 2, (o >>> 2) + 4);
  const args = [jp, json.length, bp, blob.length, o, o + 4, o + 8, o + 12];
  const t = performance.now();
  let rc;
  if (typeof mod._cs_slice_start === 'function') {
    const st = mod._malloc(8) >>> 0; mod.csSyncHeap?.(); mod.HEAP32.fill(0, st >>> 2, (st >>> 2) + 2);
    if (mod._cs_slice_start(...args, st) !== 0) throw new Error('cs_slice_start failed');
    const keep = setInterval(() => {}, 1000);
    for (;;) { mod.csSyncHeap?.(); if (Atomics.load(mod.HEAP32, st >>> 2)) break; const w = Atomics.waitAsync(mod.HEAP32, st >>> 2, 0, 500); if (w.async) await w.value; }
    clearInterval(keep); rc = mod.HEAP32[(st >>> 2) + 1]; mod._free(st);
  } else rc = mod._cs_slice(...args);
  const wall = performance.now() - t;
  mod.csSyncHeap?.();
  const H = mod.HEAPU32, g = H[o >>> 2], r = H[(o >>> 2) + 2] >>> 0, rl = H[(o >>> 2) + 3] >>> 0;
  const rep = JSON.parse(dec.decode(mod.HEAPU8.slice(r, r + rl)));
  if (rc !== 0) throw new Error(rep.error);
  mod._cs_free(g); mod._cs_free(r); mod._free(jp); mod._free(bp); mod._free(o);
  return { wall, ...rep.timings };
}

const st = await load(stDir), mt = await load(mtDir);
console.log(`instance init: ST ${st.initMs.toFixed(0)} ms, MT ${mt.initMs.toFixed(0)} ms (${mt.mod._cs_thread_count()} threads)`);
const fmt = (r) => `wall ${(r.wall / 1000).toFixed(2)}s (process ${(r.processMs / 1000).toFixed(2)}s, export ${(r.exportMs / 1000).toFixed(2)}s)`;
for (const [name, objs] of Object.entries(jobs)) {
  const best = { ST: null, MT: null };
  for (let i = 0; i < reps; i++) for (const [label, m] of [['ST', st.mod], ['MT', mt.mod]]) {
    const r = await slice(m, objs);
    console.log(`${name} | ${label} #${i + 1}: ${fmt(r)}`);
    if (!best[label] || r.wall < best[label].wall) best[label] = r;
  }
  console.log(`${name} | best: ST ${fmt(best.ST)} | MT ${fmt(best.MT)} | speedup x${(best.ST.wall / best.MT.wall).toFixed(2)}`);
}
mt.mod.csTerminateThreads?.();
process.exit(0);
