#!/usr/bin/env node
// cs_tool "arrange" (OrcaSlicer ArrangeJob): spacing, auto rotation, plates.
//   node tests/cs-arrange-test.mjs [build-dir]
import { readFileSync } from 'node:fs';
import { resolve, dirname, join } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
const here = dirname(fileURLToPath(import.meta.url));
const dir = resolve(process.argv[2] ?? join(here, '..', 'build-wasm'));
const { default: factory } = await import(pathToFileURL(join(dir, 'slicer.mjs')).href);
const mod = await factory({ wasmBinary: readFileSync(join(dir, 'slicer.wasm')), locateFile: (f) => join(dir, f), print: () => {}, printErr: () => {}, csProgress: () => {} });
let failures = 0;
const ok = (c, m) => { console.log(`${c ? 'PASS' : 'FAIL'} ${m}`); if (!c) failures++; };

function box(sx, sy, sz) {
  const x = sx / 2, y = sy / 2;
  return { p: new Float32Array([-x,-y,0, x,-y,0, x,y,0, -x,y,0, -x,-y,sz, x,-y,sz, x,y,sz, -x,y,sz]), i: new Uint32Array([0,2,1,0,3,2,4,5,6,4,6,7,0,1,5,0,5,4,1,2,6,1,6,5,2,3,7,2,7,6,3,0,4,3,4,7]) };
}
const T = (x, y, deg = 0) => { const a = deg * Math.PI / 180, c = Math.cos(a), s = Math.sin(a); return [c, s, 0, 0, -s, c, 0, 0, 0, 0, 1, 0, x, y, 0, 1]; };

/** objects: [{ mesh, transform, plate?, printable?, selected?, config? }] */
function arrange(objects, args, config = {}) {
  let size = 0; const offs = new Map();
  const place = (a) => { if (!offs.has(a)) { offs.set(a, size); size += a.byteLength; } return offs.get(a); };
  const objs = objects.map((o, k) => ({ name: `obj${k}`, vertexOffset: place(o.mesh.p), vertexCount: o.mesh.p.length / 3, indexOffset: place(o.mesh.i), triangleCount: o.mesh.i.length / 3, transform: o.transform, config: o.config ?? {}, ...(o.paint ? { paint: o.paint } : {}) }));
  const blob = new Uint8Array(Math.max(1, size));
  for (const [a, off] of offs) blob.set(new Uint8Array(a.buffer), off);
  const req = { op: 'arrange', config: { printable_area: ['0x0', '256x0', '256x256', '0x256'], ...config }, objects: objs,
    args: { mode: 'all', currentPlate: 0, plates: [{ locked: false }], settings: { distance: 0, enableRotation: false }, ...args,
      items: objects.map((o) => ({ plate: o.plate ?? 0, printable: o.printable ?? true, selected: o.selected ?? false })) } };
  const json = new TextEncoder().encode(JSON.stringify(req));
  const w = (b) => { const p = mod._malloc(b.byteLength) >>> 0; mod.HEAPU8.set(b, p); return p; };
  const jp = w(json), bp = w(blob), outs = mod._malloc(16) >>> 0; mod.HEAPU32.fill(0, outs >>> 2, (outs >>> 2) + 4);
  mod._cs_tool(jp, json.byteLength, bp, blob.byteLength, outs, outs + 4, outs + 8, outs + 12);
  const H = mod.HEAPU32; const pj = H[outs >>> 2], lj = H[(outs + 4) >>> 2];
  const r = JSON.parse(new TextDecoder().decode(mod.HEAPU8.slice(pj, pj + lj)));
  mod._cs_free(pj); mod._free(jp); mod._free(bp); mod._free(outs);
  return r;
}
// Axis-aligned footprint of a box after the result (boxes are centred on their origin).
const rect = (it, w, h) => {
  const a = Math.abs(it.dRot % Math.PI), c = Math.abs(Math.cos(a)), s = Math.abs(Math.sin(a));
  const W = w * c + h * s, H = w * s + h * c;
  return { x0: it.x - W / 2, x1: it.x + W / 2, y0: it.y - H / 2, y1: it.y + H / 2 };
};
const gap = (a, b) => Math.max(b.x0 - a.x1, a.x0 - b.x1, b.y0 - a.y1, a.y0 - b.y1);
const minGap = (rs) => { let g = Infinity; for (let i = 0; i < rs.length; i++) for (let j = i + 1; j < rs.length; j++) g = Math.min(g, gap(rs[i], rs[j])); return g; };
const inBed = (r) => r.x0 >= -1e-6 && r.y0 >= -1e-6 && r.x1 <= 256 + 1e-6 && r.y1 <= 256 + 1e-6;

const b50 = box(50, 50, 10);
const four = [0, 1, 2, 3].map(() => ({ mesh: b50, transform: T(128, 128) }));

// Auto spacing (0): each object inflated by its brim width (1 mm) → 2 mm gaps.
let r = arrange(four, {});
ok(r.ok, `arrange ok ${r.error ?? ''}`);
let rs = r.result.items.map((it) => rect(it, 50, 50));
ok(r.result.items.every((it) => it.moved && it.plate === 0), 'all four on plate 0');
ok(rs.every(inBed), 'inside the bed');
ok(Math.abs(minGap(rs) - 2) < 0.05, `auto spacing → ${minGap(rs).toFixed(2)} mm gaps (brim 1 mm each side)`);

// Explicit spacing.
r = arrange(four, { settings: { distance: 10, enableRotation: false } });
rs = r.result.items.map((it) => rect(it, 50, 50));
ok(Math.abs(minGap(rs) - 10) < 0.05, `spacing 10 → ${minGap(rs).toFixed(2)} mm gaps`);
r = arrange(four, { settings: { distance: 25, enableRotation: false } });
rs = r.result.items.map((it) => rect(it, 50, 50));
ok(minGap(rs) >= 25 - 0.05, `spacing 25 → ${minGap(rs).toFixed(2)} mm gaps`);

// Normal support: brim width 6 → 12 mm auto gaps.
r = arrange(four, {}, { enable_support: '1', support_type: 'normal(auto)' });
rs = r.result.items.map((it) => rect(it, 50, 50));
ok(Math.abs(minGap(rs) - 12) < 0.05, `auto spacing with normal support → ${minGap(rs).toFixed(2)} mm`);

// Rotation off keeps a rotated object's angle; on lets Orca pick one.
const long = box(200, 20, 10);
const tilted = [{ mesh: long, transform: T(128, 128, 30) }, { mesh: long, transform: T(128, 128, 30) }];
r = arrange(tilted, {});
ok(r.result.items.every((it) => Math.abs(it.dRot) < 1e-9), 'rotation off: no rotation change');
r = arrange(tilted, { settings: { distance: 0, enableRotation: true } });
// (Orca's _arrange sets each item's rotation to the min-area angle of its
// already-rotated outline rather than adding it, so a tilted object need not
// end up axis-aligned; check the placement is valid and the angle changed.)
const rot = (it) => ({ ...it, dRot: 30 * Math.PI / 180 + it.dRot });
rs = r.result.items.map((it) => rect(rot(it), 200, 20));
// Same final angle: separation along the bars' own axes.
const [p1, p2] = r.result.items.map(rot), th = p1.dRot;
const dx = p2.x - p1.x, dy = p2.y - p1.y;
const sep = Math.max(Math.abs(dx * Math.cos(th) + dy * Math.sin(th)) - 200, Math.abs(-dx * Math.sin(th) + dy * Math.cos(th)) - 20);
ok(r.result.items.every((it) => Math.abs(it.dRot) > 1e-3) && Math.abs(p1.dRot - p2.dRot) < 1e-9 && rs.every(inBed) && sep >= 2 - 0.05,
  `rotation on: rotated (dRot ${r.result.items.map((it) => (it.dRot * 180 / Math.PI).toFixed(1)).join(', ')}°), inside the bed, ${sep.toFixed(2)} mm apart`);
// A long part that only fits diagonally is placed when rotation is allowed.
const diag = [{ mesh: box(300, 20, 10), transform: T(128, 128) }];
ok(arrange(diag, {}).result.unplaced.length === 1, '300 mm bar does not fit unrotated');
r = arrange(diag, { settings: { distance: 0, enableRotation: true } });
ok(r.result.unplaced.length === 0 && r.result.items[0].plate === 0, `300 mm bar fits rotated (dRot ${(r.result.items[0].dRot * 180 / Math.PI).toFixed(1)}°)`);

// Overflow: 9 × 100 mm boxes → 4 per plate, new plates appended.
const nine = Array.from({ length: 9 }, () => ({ mesh: box(100, 100, 10), transform: T(128, 128) }));
r = arrange(nine, {});
const per = r.result.items.reduce((m, it) => (m[it.plate] = (m[it.plate] ?? 0) + 1, m), {});
ok(r.result.plates === 3 && per[0] === 4 && per[1] === 4 && per[2] === 1, `overflow → plates ${r.result.plates}, per plate ${JSON.stringify(per)}`);

// Locked plate 0: its objects stay; the rest skip it.
r = arrange([{ mesh: b50, transform: T(30, 30), plate: 0 }, { mesh: b50, transform: T(128, 128), plate: 1 }, { mesh: b50, transform: T(128, 128), plate: 1 }],
  { plates: [{ locked: true }, { locked: false }] });
ok(!r.result.items[0].moved && r.result.items[1].plate === 1 && r.result.items[2].plate === 1, `locked plate untouched, others on plate 1 (${r.result.items.map((i) => i.plate).join(',')})`);

// Plate mode: only the current plate's objects move.
r = arrange([{ mesh: b50, transform: T(30, 30), plate: 0 }, { mesh: b50, transform: T(30, 30), plate: 1 }, { mesh: b50, transform: T(30, 30), plate: 1 }],
  { mode: 'plate', currentPlate: 1, plates: [{ locked: false }, { locked: false }] });
ok(!r.result.items[0].moved && r.result.items[1].moved && r.result.items[1].plate === 1 && r.result.items[2].plate === 1, 'plate mode moves only that plate');

// Unprintable objects go to the plate after the last one used.
r = arrange([{ mesh: b50, transform: T(30, 30) }, { mesh: b50, transform: T(60, 60), printable: false }], {});
ok(r.result.items[0].plate === 0 && r.result.items[1].plate === 1, `unprintable → plate ${r.result.items[1].plate}`);

// Too big for any plate: parked on the plate after the last.
r = arrange([{ mesh: box(300, 300, 10), transform: T(128, 128) }], {});
ok(r.result.items[0].plate === 1 && r.result.unplaced.length === 1, `oversize → plate ${r.result.items[0].plate}, unplaced ${JSON.stringify(r.result.unplaced)}`);

// The engine hands Orca a vertex subset with the same outline / extremes; a
// (no-op) colour paint entry keeps the whole mesh. Both must arrange the same.
function sphere(r, seg) {
  const p = [], idx = [];
  for (let i = 0; i <= seg; i++) for (let j = 0; j < 2 * seg; j++) {
    const th = Math.PI * i / seg, ph = Math.PI * j / seg;
    p.push(r * Math.sin(th) * Math.cos(ph) * 1.6, r * Math.sin(th) * Math.sin(ph), r + r * Math.cos(th) * 0.7);
  }
  const at = (i, j) => i * 2 * seg + (j % (2 * seg));
  for (let i = 0; i < seg; i++) for (let j = 0; j < 2 * seg; j++) idx.push(at(i, j), at(i + 1, j), at(i + 1, j + 1), at(i, j), at(i + 1, j + 1), at(i, j + 1));
  return { p: new Float32Array(p), i: new Uint32Array(idx) };
}
const tiltXZ = (x, y, ax, az) => { // Rz(az) * Rx(ax), column-major
  const cx = Math.cos(ax), sx = Math.sin(ax), cz = Math.cos(az), sz = Math.sin(az);
  return [cz, sz, 0, 0, -sz * cx, cz * cx, sx, 0, sz * sx, -cz * sx, cx, 0, x, y, 0, 1];
};
const egg = sphere(25, 40);
const mix = [egg, box(120, 18, 30), egg, box(60, 60, 8)];
const scene = (paint) => mix.map((mesh, k) => ({ mesh, transform: tiltXZ(128, 128, 0.35 * k, 0.4 + 0.3 * k), ...(paint ? { paint: { color: [[0, '']] } } : {}) }));
for (const settings of [{ distance: 0, enableRotation: false }, { distance: 4, enableRotation: true }, { distance: 0, enableRotation: false, alignToYAxis: true }]) {
  const a = arrange(scene(false), { settings }), b = arrange(scene(true), { settings });
  const same = a.ok && b.ok && a.result.items.every((it, k) => {
    const o = b.result.items[k];
    return it.plate === o.plate && Math.abs(it.x - o.x) < 1e-6 && Math.abs(it.y - o.y) < 1e-6 && Math.abs(it.dRot - o.dRot) < 1e-9;
  });
  ok(same, `vertex subset arranges like the whole mesh (${JSON.stringify(settings)}) ${a.error ?? ''}${b.error ?? ''}`);
}

// Bad request is an error, not a trap.
r = arrange(four, { mode: 'sideways' });
ok(!r.ok && /mode/.test(r.error), 'unknown mode rejected');

console.log(failures ? `\n${failures} FAILED` : '\nALL PASSED');
process.exit(failures ? 1 : 0);
