#!/usr/bin/env node
// CAD kernel module: one call of every method cad-test.mjs does not cover, on small documents.
//   node tests/cad-smoke.mjs [build-dir]
// A method that throws at the ABI level (rc 1 other than a deliberate refusal) FAILs; a geometry
// result with ok:false is reported (and FAILs only where the call is expected to build).
// Contract: cubby-slicer/docs/CAD-KERNEL-CONTRACT.md.
import { readFileSync } from 'node:fs';
import { resolve, dirname, join } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

const here = dirname(fileURLToPath(import.meta.url));
const dir = resolve(process.argv.slice(2).find((a) => !a.startsWith('--')) ?? join(here, '..', 'build-cad'));
const { default: factory } = await import(pathToFileURL(join(dir, 'cad.mjs')).href);
const mod = await factory({ wasmBinary: readFileSync(join(dir, 'cad.wasm')), print: () => {}, printErr: () => {} });
const enc = new TextEncoder(), dec = new TextDecoder();
function call(method, params = {}, blob = null) {
  const w = (b) => { const p = mod._malloc(Math.max(1, b.byteLength)) >>> 0; mod.HEAPU8.set(b, p); return p; };
  const m = enc.encode(method), j = enc.encode(JSON.stringify(params)), b = blob ?? new Uint8Array(0);
  const mp = w(m), jp = w(j), bp = w(b), outs = mod._malloc(16) >>> 0;
  mod.HEAPU32.fill(0, outs >>> 2, (outs >>> 2) + 4);
  const rc = mod._cad_call(mp, m.byteLength, jp, j.byteLength, bp, b.byteLength, outs, outs + 4, outs + 8, outs + 12);
  const H = mod.HEAPU32;
  const pj = H[outs >>> 2], lj = H[(outs + 4) >>> 2], pb = H[(outs + 8) >>> 2], lb = H[(outs + 12) >>> 2];
  const r = JSON.parse(dec.decode(mod.HEAPU8.slice(pj, pj + lj)));
  r.rc = rc;
  r.blob = pb ? mod.HEAPU8.slice(pb, pb + lb) : new Uint8Array(0);
  mod._cad_free(pj); if (pb) mod._cad_free(pb);
  mod._free(mp); mod._free(jp); mod._free(bp); mod._free(outs);
  return r;
}
let failures = 0;
const ok = (c, m) => { console.log(`${c ? 'PASS' : 'FAIL'} ${m}`); if (!c) failures++; };
// S: call, print, return result (null on rc 1). expect: 'ok' (must build), 'any' (report only)
function S(label, method, params, expect = 'ok', blob = null) {
  const t = performance.now();
  const r = call(method, params, blob);
  const ms = Math.round(performance.now() - t);
  if (!r.ok) { ok(false, `${label}: rc ${r.rc} ${r.error.code} ${r.error.message}`); return null; }
  const res = r.result; res.blob = r.blob;
  const built = res.ok !== false;
  if (expect === 'ok') ok(built, `${label}${built ? '' : `: ${res.error}`} (${ms} ms)`);
  else console.log(`INFO ${label}: ${built ? 'ok' : `ok:false ${res.error}`} (${ms} ms)`);
  return res;
}
const XY = { origin: [0, 0, 0], normal: [0, 0, 1], x_axis: [1, 0, 0], y_axis: [0, 1, 0] };
const line = (p0, p1) => ({ type: 'line', p0, p1 });

// ---- solid verbs on a 20 x 20 x 10 box --------------------------------------------------------
const { doc } = S('doc_new', 'doc_new', { modeling_origin: [0, 0, 0] });
S('extrude box', 'extrude', { doc, width: 20, height: 20, distance: 10 });
const vol = () => S('mass', 'mass_properties', { doc, body: 0 }, 'any').volume;
const v0 = vol();
S('chamfer Top 1', 'chamfer', { doc, face_group: 'Top', distance: 1 });
S('hole_styled countersink', 'hole_styled', { doc, diameter: 4, style: 2, csink_diameter: 8, x: 4, y: 4, plane: 'XY' }, 'any');
S('hole_styled counterbore', 'hole_styled', { doc, diameter: 3, style: 1, cbore_diameter: 6, cbore_depth: 2, x: -4, y: -4, plane: 'XY' }, 'any');
let topo = S('query_topology', 'query_topology', { doc, body: 0 });
const side = topo.faces.find((f) => f.kind === 'planar' && Math.abs(f.normal[0] - 1) < 1e-6);
const bottom = topo.faces.find((f) => f.kind === 'planar' && f.normal[2] < -0.99);
S('plane_of_face', 'plane_of_face', { doc, body: 0, face: side.id, generation: topo.generation });
S('measure face-face', 'measure', { doc, body: 0, a: { face: side.id }, b: { face: bottom.id } });
S('draft side 3°', 'draft', { doc, face: side.id, angle: 3, generation: topo.generation }, 'any');
S('mirror YZ new', 'mirror', { doc, plane: 'YZ', mode: 'new', keep_original: true }, 'any');
const sc = S('describe_scene', 'describe_scene', { doc });
S('transform body 1', 'transform', { doc, body: sc.bodies.length - 1, dx: 40, angle: 15 }, 'any');
S('set_body name/colour', 'set_body', { doc, body: 0, name: 'Box', color: [1, 0, 0, 1] });
S('check_interference', 'check_interference', { doc, min_volume: 1e-6 });
S('validate_against body', 'validate_against', { doc, body: 0, reference: { body: 0 } }, 'any');
S('cut XY @5 keep lower', 'cut', { doc, plane: 'XY', offset: 5, keep_upper: false, keep_lower: true, body: 0 }, 'any');
topo = S('query_topology', 'query_topology', { doc, body: 0 });
const top = topo.faces.find((f) => f.kind === 'planar' && f.normal[2] > 0.99);
S('thicken top face 2', 'thicken', { doc, body: 0, face: top.id, thickness: 2, generation: topo.generation }, 'any');
topo = S('query_topology', 'query_topology', { doc, body: 0 });
const cyl = topo.faces.find((f) => f.kind === 'cylindrical');
if (cyl) S('delete_face (hole wall)', 'delete_face', { doc, body: 0, faces: [cyl.id], generation: topo.generation }, 'any');
topo = S('query_topology', 'query_topology', { doc, body: 0 });
const anyPlanar = topo.faces.find((f) => f.kind === 'planar' && f.normal[2] > 0.99);
S('split by face', 'split', { doc, body: 0, face_body: 0, face: anyPlanar.id, keep_upper: true, keep_lower: true, generation: topo.generation }, 'any');
topo = S('query_topology', 'query_topology', { doc, body: 0 });
S('project top face → XY', 'project', { doc, source_body: 0, face: topo.faces.find((f) => f.kind === 'planar' && f.normal[2] > 0.99).id, plane: 'XY', generation: topo.generation }, 'any');
S('axis two points', 'axis', { doc, type: 'two_points', p1: [0, 0, 0], p2: [0, 0, 10] });
const csA = S('coordsys A', 'coordsys', { doc, type: 'point_world', point: [0, 0, 10] });
const csB = S('coordsys B', 'coordsys', { doc, type: 'point_world', point: [30, 0, 0] });
S('mate_options', 'mate_options', { doc, cs_a: csA.coordsys_index, cs_b: csB.coordsys_index });
S('mate fastened', 'mate', { doc, kind: 0, cs_a: csA.coordsys_index, cs_b: csB.coordsys_index }, 'any');
S('helix', 'helix', { doc, radius: 5, pitch: 3, height: 9, plane: { ...XY, origin: [60, 0, 0] } });
S('plane_datum offset 15', 'plane_datum', { doc, base: 0, offset: 15 });
S('datums', 'datums', { doc });
S('set_document auto_close', 'set_document', { doc, auto_close_loops: true });
S('doc_list', 'doc_list', {});

// ---- sketch-driven verbs --------------------------------------------------------------------------
const { doc: d2 } = S('doc_new 2', 'doc_new', { modeling_origin: [0, 0, 0] });
// revolve: 4x10 rectangle 5 mm off the plane's Y axis, about plane Y (axis 1)
S('sketch_begin XZ', 'sketch_begin', { doc: d2, plane: 'XZ' });
S('sketch_add rect', 'sketch_add', { doc: d2, rect: [5, 0, 9, 10] });
const rsk = S('sketch_commit', 'sketch_commit', { doc: d2 });
const rv = S('revolve 360 about Y', 'revolve', { doc: d2, sketch: rsk.sketch_index, angle: 360, axis: 1 });
if (rv) ok(Math.abs(S('mass', 'mass_properties', { doc: d2, body: 0 }).volume - Math.PI * (81 - 25) * 10) < 0.5, 'revolve volume = π(9²−5²)·10');
// sweep: circle r2 at (40,0) on XY along a 30 mm line in +Z
S('sketch_begin XY', 'sketch_begin', { doc: d2, plane: 'XY' });
S('sketch_add circle', 'sketch_add', { doc: d2, type: 'circle', center: [40, 0], radius: 2 });
const prof = S('sketch_commit', 'sketch_commit', { doc: d2 });
S('sketch_begin XZ path', 'sketch_begin', { doc: d2, plane: 'XZ' });
S('sketch_add path', 'sketch_add', { doc: d2, entities: [line([40, 0], [40, 30])] });
const path = S('sketch_commit', 'sketch_commit', { doc: d2 });
const sw = S('sweep', 'sweep', { doc: d2, profile: prof.sketch_index, path: path.sketch_index, boolean: 'new' });
// loft: square on XY at x=-40 to a circle 20 mm up
S('sketch_begin loft 0', 'sketch_begin', { doc: d2, plane: { ...XY, origin: [-40, 0, 0] } });
S('sketch_add square', 'sketch_add', { doc: d2, rect: [-5, -5, 5, 5] });
const l0 = S('sketch_commit', 'sketch_commit', { doc: d2 });
S('sketch_begin loft 1', 'sketch_begin', { doc: d2, plane: { ...XY, origin: [-40, 0, 20] } });
S('sketch_add circle', 'sketch_add', { doc: d2, type: 'circle', center: [0, 0], radius: 3 });
const l1 = S('sketch_commit', 'sketch_commit', { doc: d2 });
S('loft', 'loft', { doc: d2, profiles: [l0.sketch_index, l1.sketch_index], boolean: 'new' });
// surfaces: an open polyline extruded to a sheet, thickened
S('sketch_begin surface', 'sketch_begin', { doc: d2, plane: { ...XY, origin: [0, 40, 0] } });
S('sketch_add line', 'sketch_add', { doc: d2, entities: [line([0, 0], [10, 0])] });
const ssk = S('sketch_commit', 'sketch_commit', { doc: d2 });
const se = S('surface_extrude', 'surface_extrude', { doc: d2, sketch: ssk.sketch_index, distance: 10 });
const sc2 = S('describe_scene', 'describe_scene', { doc: d2 });
S('thicken_surface', 'thicken_surface', { doc: d2, body: sc2.bodies.length - 1, thickness: 1 }, 'any');
S('surface_offset', 'surface_offset', { doc: d2, body: sc2.bodies.length - 1, offset: 2 }, 'any');
S('surface_revolve', 'surface_revolve', { doc: d2, sketch: ssk.sketch_index, angle: 90, axis: 1 }, 'any');
S('surface_loft', 'surface_loft', { doc: d2, profiles: [l0.sketch_index, l1.sketch_index] }, 'any');
S('surface_fill', 'surface_fill', { doc: d2, sketch: l1.sketch_index }, 'any');
S('body_touching_sketch', 'body_touching_sketch', { doc: d2, sketch: prof.sketch_index });
// rib + pattern_on_curve + bridge on a shelled box
const { doc: d3 } = S('doc_new 3', 'doc_new', { modeling_origin: [0, 0, 0] });
S('extrude 40x20x10', 'extrude', { doc: d3, width: 40, height: 20, distance: 10 });
let t3 = S('query_topology', 'query_topology', { doc: d3, body: 0 });
S('shell open top', 'shell', { doc: d3, thickness: 1.5, face: t3.faces.find((f) => f.normal[2] > 0.99).id, generation: t3.generation });
S('sketch_begin rib', 'sketch_begin', { doc: d3, plane: { ...XY, origin: [0, 0, 1.5] } });
S('sketch_add rib line', 'sketch_add', { doc: d3, entities: [line([-19, 0], [19, 0])] });
const rib = S('sketch_commit', 'sketch_commit', { doc: d3 });
S('rib', 'rib', { doc: d3, sketch: rib.sketch_index, entity: 0, thickness: 1.5, depth: 6 }, 'any');
S('sketch_begin curve', 'sketch_begin', { doc: d3, plane: { ...XY, origin: [0, 40, 0] } });
S('sketch_add arc', 'sketch_add', { doc: d3, type: 'arc', center: [0, 0], radius: 15, start_angle: 0, end_angle: Math.PI });
const crv = S('sketch_commit', 'sketch_commit', { doc: d3 });
S('extrude boss', 'extrude', { doc: d3, plane: { ...XY, origin: [15, 40, 0] }, width: 3, height: 3, distance: 3, boolean: 'new' });
S('pattern_on_curve ×4', 'pattern_on_curve', { doc: d3, count: 4, sketch: crv.sketch_index, entity: 0, body: 1 }, 'any');
S('bridge two lines', 'bridge', { doc: d3, sketch: rib.sketch_index, ent_a: 0, end_a: 1, ent_b: 0, end_b: 0 }, 'any');
// import_mesh: a 10 mm cube (8 vertices, 12 triangles)
{
  const P = new Float32Array([0, 0, 0, 10, 0, 0, 10, 10, 0, 0, 10, 0, 0, 0, 10, 10, 0, 10, 10, 10, 10, 0, 10, 10]);
  const I = new Uint32Array([0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5, 2, 3, 7, 2, 7, 6, 3, 0, 4, 3, 4, 7]);
  const blob = new Uint8Array(P.byteLength + I.byteLength);
  blob.set(new Uint8Array(P.buffer), 0); blob.set(new Uint8Array(I.buffer), P.byteLength);
  const { doc: d4 } = S('doc_new 4', 'doc_new', {});
  const im = S('import_mesh cube', 'import_mesh', { doc: d4, positions: { offset: 0, count: P.length }, indices: { offset: P.byteLength, count: I.length } }, 'ok', blob);
  if (im) ok(Math.abs(S('mass', 'mass_properties', { doc: d4, body: 0 }).volume - 1000) < 1e-6, `import_mesh: ${JSON.stringify({ faces: im.faces_final, solid: im.is_solid })} volume 1000`);
}

// ---- sketch session verbs -------------------------------------------------------------------------
const { doc: d5 } = S('doc_new 5', 'doc_new', { modeling_origin: [0, 0, 0] });
S('sketch_begin', 'sketch_begin', { doc: d5, plane: 'XY' });
S('sketch_add rect', 'sketch_add', { doc: d5, rect: [0, 0, 20, 10] });
S('sketch_add line', 'sketch_add', { doc: d5, entities: [line([30, 0], [40, 0]), line([40, 0], [40, 10])] });
S('sketch_infer', 'sketch_infer', { doc: d5, entity: 4 });
S('sketch_select', 'sketch_select', { doc: d5, entities: [4] });
S('sketch_construction', 'sketch_construction', { doc: d5 });
S('sketch_offset', 'sketch_offset', { doc: d5, entities: [0, 1, 2, 3], distance: 2 }, 'any');
S('sketch_mirror', 'sketch_mirror', { doc: d5, entities: [5], axis_a: [0, 0], axis_b: [0, 1] }, 'any');
const dd = S('sketch_describe', 'sketch_describe', { doc: d5 });
S('sketch_set_entity', 'sketch_set_entity', { doc: d5, index: 5, entity: line([40, 0], [40, 12]) }, 'any');
S('sketch_drag', 'sketch_drag', { doc: d5, entity: 0, role: 'p1', to: [22, 0] }, 'any');
S('sketch_solve', 'sketch_solve', { doc: d5 });
if (dd.constraints.length) S('sketch_delete_constraint', 'sketch_delete_constraint', { doc: d5, index: dd.constraints.length - 1 }, 'any');
S('sketch_ops fillet', 'sketch_ops', { doc: d5, op: 'fillet', a: 0, b: 1, radius: 2 }, 'any');
S('sketch_ops array', 'sketch_ops', { doc: d5, op: 'array', entities: [4], count: 3, step: [0, 15] }, 'any');
S('sketch_ops transform', 'sketch_ops', { doc: d5, op: 'transform', entities: [5], move: [1, 0], angle: 0, scale: 1 }, 'any');
S('sketch_ops trim', 'sketch_ops', { doc: d5, op: 'trim', entity: 4, pick: [35, 0] }, 'any');
S('sketch_undo', 'sketch_undo', { doc: d5 });
S('sketch_redo', 'sketch_redo', { doc: d5 });
S('sketch_delete', 'sketch_delete', { doc: d5, entities: [4] });
S('sketch_validate', 'sketch_validate', { doc: d5 });
S('sketch_heal', 'sketch_heal', { doc: d5 }, 'any');
S('sketch_cancel', 'sketch_cancel', { doc: d5 });
const noSk = call('sketch_describe', { doc: d5 });
ok(!noSk.ok && noSk.error.code === -32022, 'no open sketch → -32022');
ok(!call('describe_scene', { doc: 999 }).ok, 'unknown doc handle refused');

for (const dd of [doc, d2, d3, d5]) call('doc_close', { doc: dd });
console.log(`  wasm memory (peak): ${(mod.HEAPU8.length / 1048576).toFixed(0)} MB`);
console.log(failures ? `\n${failures} FAILED` : '\nall passed');
process.exit(failures ? 1 : 0);
