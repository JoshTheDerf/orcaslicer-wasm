#!/usr/bin/env node
// Timing of cs_tool "arrange" on heavy meshes (3 × ~50k-triangle spheres).
//   node tests/cs-arrange-bench.mjs [build-dir]
import { readFileSync } from 'node:fs';
import { resolve, dirname, join } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
const here = dirname(fileURLToPath(import.meta.url));
const dir = resolve(process.argv[2] ?? join(here, '..', 'build-wasm'));
const { default: factory } = await import(pathToFileURL(join(dir, 'slicer.mjs')).href);
const mod = await factory({ wasmBinary: readFileSync(join(dir, 'slicer.wasm')), locateFile: (f) => join(dir, f), print: () => {}, printErr: () => {}, csProgress: () => {} });

function sphere(r, seg) {
  const p = [], idx = [];
  for (let i = 0; i <= seg; i++) for (let j = 0; j < 2 * seg; j++) {
    const th = Math.PI * i / seg, ph = Math.PI * j / seg;
    p.push(r * Math.sin(th) * Math.cos(ph), r * Math.sin(th) * Math.sin(ph), r + r * Math.cos(th));
  }
  const at = (i, j) => i * 2 * seg + (j % (2 * seg));
  for (let i = 0; i < seg; i++) for (let j = 0; j < 2 * seg; j++) { idx.push(at(i, j), at(i + 1, j), at(i + 1, j + 1), at(i, j), at(i + 1, j + 1), at(i, j + 1)); }
  return { p: new Float32Array(p), i: new Uint32Array(idx) };
}
const s = sphere(20, 112); // ~50k triangles
const objects = [0, 1, 2].map(() => ({ mesh: s, transform: [1,0,0,0, 0,1,0,0, 0,0,1,0, 128,128,0,1] }));
let size = 0; const offs = new Map();
const place = (a) => { if (!offs.has(a)) { offs.set(a, size); size += a.byteLength; } return offs.get(a); };
const objs = objects.map((o, k) => ({ name: `obj${k}`, vertexOffset: place(o.mesh.p), vertexCount: o.mesh.p.length / 3, indexOffset: place(o.mesh.i), triangleCount: o.mesh.i.length / 3, transform: o.transform, config: {} }));
const blob = new Uint8Array(size);
for (const [a, off] of offs) blob.set(new Uint8Array(a.buffer), off);
for (const mode of ['plate', 'all']) {
  const req = { op: 'arrange', config: { printable_area: ['0x0', '256x0', '256x256', '0x256'] }, objects: objs,
    args: { mode, currentPlate: 0, plates: [{ locked: false }], settings: { distance: 0, enableRotation: false }, items: objects.map(() => ({ plate: 0, printable: true, selected: false })) } };
  const json = new TextEncoder().encode(JSON.stringify(req));
  const w = (b) => { const ptr = mod._malloc(b.byteLength) >>> 0; mod.HEAPU8.set(b, ptr); return ptr; };
  const jp = w(json), bp = w(blob), outs = mod._malloc(16) >>> 0; mod.HEAPU32.fill(0, outs >>> 2, (outs >>> 2) + 4);
  const t0 = performance.now();
  mod._cs_tool(jp, json.byteLength, bp, blob.byteLength, outs, outs + 4, outs + 8, outs + 12);
  const ms = performance.now() - t0;
  const H = mod.HEAPU32; const pj = H[outs >>> 2], lj = H[(outs + 4) >>> 2];
  const r = JSON.parse(new TextDecoder().decode(mod.HEAPU8.slice(pj, pj + lj)));
  mod._cs_free(pj); mod._free(jp); mod._free(bp); mod._free(outs);
  console.log(`${mode}: ${ms.toFixed(0)} ms, ${s.i.length / 3 * 3} triangles, ok=${r.ok} ${r.error ?? ''}`, JSON.stringify(r.result?.timing ?? {}, (k, v) => typeof v === 'number' ? Math.round(v) : v));
}
