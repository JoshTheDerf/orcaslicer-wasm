#!/usr/bin/env node
// cs_tool tests: painting sessions, paint_from_states, layer profiles, cut (+ connectors, groove).
//   node tests/cs-tool-test.mjs [build-dir]
import { readFileSync } from 'node:fs';
import { resolve, dirname, join } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
const here = dirname(fileURLToPath(import.meta.url));
const dir = resolve(process.argv[2] ?? join(here, '..', 'build-wasm'));
const { default: factory } = await import(pathToFileURL(join(dir, 'slicer.mjs')).href);
const mod = await factory({ wasmBinary: readFileSync(join(dir, 'slicer.wasm')), locateFile: (f) => join(dir, f), print: () => {}, printErr: () => {}, csProgress: () => {} });
let failures = 0;
const ok = (c, m) => { console.log(`${c ? 'PASS' : 'FAIL'} ${m}`); if (!c) failures++; };

function tool(op, args, meshes = []) {
  let size = 0; const ents = meshes.map((m) => { const v = size; size += m.p.byteLength; const i = size; size += m.i.byteLength; return { vertexOffset: v, vertexCount: m.p.length / 3, indexOffset: i, triangleCount: m.i.length / 3 }; });
  const blob = new Uint8Array(Math.max(1, size));
  meshes.forEach((m, k) => { blob.set(new Uint8Array(m.p.buffer), ents[k].vertexOffset); blob.set(new Uint8Array(m.i.buffer), ents[k].indexOffset); });
  const json = new TextEncoder().encode(JSON.stringify({ op, args, meshes: ents }));
  const w = (b) => { const p = mod._malloc(b.byteLength) >>> 0; mod.HEAPU8.set(b, p); return p; };
  const jp = w(json), bp = w(blob), outs = mod._malloc(16) >>> 0; mod.HEAPU32.fill(0, outs >>> 2, (outs >>> 2) + 4);
  mod._cs_tool(jp, json.byteLength, bp, blob.byteLength, outs, outs + 4, outs + 8, outs + 12);
  const H = mod.HEAPU32; const pj = H[outs >>> 2], lj = H[(outs + 4) >>> 2], pb = H[(outs + 8) >>> 2], lb = H[(outs + 12) >>> 2];
  const r = JSON.parse(new TextDecoder().decode(mod.HEAPU8.slice(pj, pj + lj)));
  const bytes = pb ? mod.HEAPU8.slice(pb, pb + lb) : new Uint8Array(0);
  r.meshData = (r.meshes ?? []).map((m) => ({ p: new Float32Array(bytes.buffer.slice(m.vertexOffset, m.vertexOffset + m.vertexCount * 12)), i: new Uint32Array(bytes.buffer.slice(m.indexOffset, m.indexOffset + m.triangleCount * 12)) }));
  if (pj) mod._cs_free(pj); if (pb) mod._cs_free(pb); mod._free(jp); mod._free(bp); mod._free(outs);
  return r;
}
function box(sx, sy, sz) {
  const x = sx / 2, y = sy / 2, z = sz / 2;
  return { p: new Float32Array([-x,-y,-z, x,-y,-z, x,y,-z, -x,y,-z, -x,-y,z, x,-y,z, x,y,z, -x,y,z]), i: new Uint32Array([0,2,1,0,3,2,4,5,6,4,6,7,0,1,5,0,5,4,1,2,6,1,6,5,2,3,7,2,7,6,3,0,4,3,4,7]) };
}
const I = [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1];
const cube = box(20, 20, 20);

// painting session: top face (z=+10) triangles are 2,3 → paint a circle there
const open = tool('paint_open', { maxState: 2 }, [cube]);
ok(open.ok && open.result.handle > 0 && open.result.states.length === 0, 'paint_open');
const h = open.result.handle;
const ap = tool('paint_apply', { handle: h, kind: 'circle', facet: 2, point: [0, 0, 10], camera: [0, 0, 100], radius: 4, state: 1, trafo: I }, []);
ok(ap.ok && ap.result.states[0] === 1 && ap.meshData[0].i.length > 3, `circle stroke paints subdivided facets (${ap.meshData[0]?.i.length / 3} tris)`);
const st2 = tool('paint_apply', { handle: h, kind: 'circle', facet: 2, point: [5, 5, 10], prev: [0, 0, 10], camera: [0, 0, 100], radius: 2, state: 2, trafo: I }, []);
ok(st2.ok && st2.result.states.includes(2), 'stroke with prev point (blocker)');
const fill = tool('paint_apply', { handle: h, kind: 'fill', facet: 8, point: [0, -10, 0], camera: [0, -100, 0], state: 1, angle: 30, trafo: I }, []);
ok(fill.ok, 'smart fill');
const got = tool('paint_get', { handle: h }, []);
ok(got.ok && got.result.paint.length >= 2 && got.result.paint.every(([t, s]) => Number.isInteger(t) && /^[0-9A-F]+$/i.test(s)), `paint_get → ${got.result.paint.length} triangle strings`);
tool('paint_close', { handle: h }, []);
const reopen = tool('paint_open', { paint: got.result.paint, maxState: 2 }, [cube]);
ok(reopen.ok && reopen.result.states.length === 2, 'paint round-trips through strings');
tool('paint_close', { handle: reopen.result.handle }, []);
const fs = tool('paint_from_states', { states: [[0, 3], [1, 3], [4, 2]] }, [cube]);
ok(fs.ok && fs.result.paint.length === 3, 'paint_from_states');
const bad = tool('paint_apply', { handle: 9999, kind: 'circle' }, []);
ok(!bad.ok && /session/.test(bad.error), 'unknown session is an error, not a trap');

// layer profiles on a tall cylinder-ish shape (a box works: adaptive gives max height on vertical walls)
const tall = box(20, 20, 40);
const cfg = { layer_height: '0.2', initial_layer_print_height: '0.2', min_layer_height: ['0.07'], max_layer_height: ['0.28'], nozzle_diameter: ['0.4'] };
const ad = tool('layer_profile_adaptive', { transform: I, config: cfg, quality: 0.5 }, [tall]);
ok(ad.ok && ad.result.profile.length >= 4 && ad.result.profile.length % 2 === 0, `adaptive profile (${ad.result.profile?.length / 2} points) ${ad.error ?? ''}`);
const sm = tool('layer_profile_smooth', { transform: I, config: cfg, profile: ad.result.profile ?? [0, 0.2, 40, 0.2], radius: 5, keepMin: false }, [tall]);
ok(sm.ok && sm.result.profile.length >= 4, `smooth profile ${sm.error ?? ''}`);

// cut the cube in half at z=0 (object space), keep both, with one plug connector
const cutArgs = (extra = {}) => ({ name: 'Cube', offset: [100, 100, 10], volumes: [{ name: 'Cube', type: 'part', transform: I }],
  cut: { matrix: I, keepUpper: true, keepLower: true, placeOnCutUpper: true, ...extra } });
const c1 = tool('cut', cutArgs(), [cube]);
ok(c1.ok && c1.result.objects.length === 2, `plane cut → ${c1.result.objects?.length} objects ${c1.error ?? ''}`);
const c2 = tool('cut', cutArgs({ connectors: [{ pos: [0, 0, 0], radius: 3, height: 6, type: 'plug', style: 'prism', shape: 'circle', heightTolerance: 0.1 }] }), [cube]);
const vols = c2.result.objects?.flatMap((o) => o.volumes.map((v) => v.type)) ?? [];
ok(c2.ok && c2.result.objects.length === 2 && vols.includes('negative') , `cut with plug connector → volumes ${JSON.stringify(vols)} ${c2.error ?? ''}`);
const c3 = tool('cut', cutArgs({ connectors: [{ pos: [0, 0, 0], radius: 3, height: 6, type: 'dowel', style: 'prism', shape: 'circle' }] }), [cube]);
ok(c3.ok && c3.result.objects.length === 3, `cut with dowel → ${c3.result.objects?.length} objects (2 halves + dowel) ${c3.error ?? ''}`);
const c4 = tool('cut', cutArgs({ mode: 'groove', groove: { depth: 4, width: 6, flapsAngle: 1.0, angle: 0, radius: 30 } }), [cube]);
ok(c4.ok && c4.result.objects.length === 2, `tongue & groove cut ${c4.error ?? ''}`);

console.log(failures ? `\n${failures} FAILED` : '\nALL PASSED');
process.exit(failures ? 1 : 0);
