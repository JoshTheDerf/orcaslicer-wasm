#!/usr/bin/env node
// CAD kernel module (cad.mjs) tests — contract: cubby-slicer/docs/CAD-KERNEL-CONTRACT.md.
//   node tests/cad-test.mjs [build-dir]            # default build-cad/
//   node tests/cad-test.mjs build-cad --update     # rewrite tests/cad-golden.json from this build
//
// Covers: Orca's recipe fixtures (open, rebuild, tessellate, byte-compatible round trip, refusal of
// v3, read-only above v6, unknown bytes kept), a design built from scratch through the API
// (sketch → extrude → fillet → shell → pattern → boolean → hole → thread), save/reopen with stable
// face/edge ids, a dimension edit, undo/redo, STEP export/import, and the OCCT 7.6 BRep fixture.
// Golden numbers (volumes, face/edge counts) live in tests/cad-golden.json so a regression — or
// drift between this wasm build and a native Orca 2.5.0 — shows up as a diff.
import { readFileSync, writeFileSync, existsSync } from 'node:fs';
import { resolve, dirname, join } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

const here = dirname(fileURLToPath(import.meta.url));
const args = process.argv.slice(2);
const update = args.includes('--update');
const dir = resolve(args.find((a) => !a.startsWith('--')) ?? join(here, '..', 'build-cad'));
const fixtures = join(here, '..', 'orca', 'tests', 'data');
const goldenPath = join(here, 'cad-golden.json');
const golden = existsSync(goldenPath) ? JSON.parse(readFileSync(goldenPath, 'utf8')) : {};
const record = {};

const t0 = performance.now();
const { default: factory } = await import(pathToFileURL(join(dir, 'cad.mjs')).href);
const wasmBinary = readFileSync(join(dir, 'cad.wasm'));
const tImport = performance.now();
const mod = await factory({ wasmBinary, print: () => {}, printErr: () => {} });
const tInst = performance.now();

let failures = 0;
const ok = (c, m) => { console.log(`${c ? 'PASS' : 'FAIL'} ${m}`); if (!c) failures++; };
const near = (a, b, rel = 1e-6) => Math.abs(a - b) <= rel * Math.max(1, Math.abs(b));

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
  if (pj) mod._cad_free(pj); if (pb) mod._cad_free(pb);
  mod._free(mp); mod._free(jp); mod._free(bp); mod._free(outs);
  return r;
}
// call that must succeed at the ABI level
function C(method, params = {}, blob = null) {
  const r = call(method, params, blob);
  if (!r.ok) throw new Error(`${method}: ${JSON.stringify(r.error)}`);
  return Object.assign(r.result, { blob: r.blob });
}
// wasm linear memory never shrinks, so its size is the peak heap so far
const memMB = () => (mod.HEAPU8.length / 1048576).toFixed(0);
const f32 = (blob, a) => new Float32Array(blob.buffer, blob.byteOffset + a.offset, a.count);
const u32 = (blob, a) => new Uint32Array(blob.buffer, blob.byteOffset + a.offset, a.count);
const i32 = (blob, a) => new Int32Array(blob.buffer, blob.byteOffset + a.offset, a.count);

// Welded plate mesh: count edges not shared by exactly two triangles (0 = watertight).
function meshStats(blob, m) {
  const P = f32(blob, m.positions), I = u32(blob, m.indices);
  const edges = new Map();
  let vol = 0;
  for (let t = 0; t < I.length; t += 3) {
    const a = I[t], b = I[t + 1], c = I[t + 2];
    for (const [u, v] of [[a, b], [b, c], [c, a]]) { const k = u < v ? `${u},${v}` : `${v},${u}`; edges.set(k, (edges.get(k) ?? 0) + 1); }
    const ax = P[3 * a], ay = P[3 * a + 1], az = P[3 * a + 2], bx = P[3 * b], by = P[3 * b + 1], bz = P[3 * b + 2], cx = P[3 * c], cy = P[3 * c + 1], cz = P[3 * c + 2];
    vol += (ax * (by * cz - bz * cy) - ay * (bx * cz - bz * cx) + az * (bx * cy - by * cx)) / 6;
  }
  let open = 0;
  for (const n of edges.values()) if (n !== 2) open++;
  return { triangles: I.length / 3, vertices: P.length / 3, openEdges: open, edges: edges.size, volume: vol };
}
// Exact topology signature of a document: per body, every face (surface type, centroid, normal)
// and edge (end points) by id, the face/edge counts and the B-rep volume.
function topoSig(doc) {
  const sc = C('describe_scene', { doc });
  return JSON.stringify(sc.bodies.map((b) => {
    const t = C('query_topology', { doc, body: b.index });
    return { n: [t.face_count, t.edge_count], v: C('mass_properties', { doc, body: b.index }).volume,
             f: t.faces.map((f) => [f.surface, f.centroid, f.normal]), e: t.edges.map((e) => [e.p0, e.p1]) };
  }));
}
function checkGolden(key, value, tol = 1e-6) {
  record[key] = value;
  if (update || !(key in golden)) return;
  const g = golden[key];
  const same = typeof value === 'number' ? near(value, g, tol) : JSON.stringify(value) === JSON.stringify(g);
  ok(same, `golden ${key} = ${JSON.stringify(value)}${same ? '' : ` (expected ${JSON.stringify(g)})`}`);
}

// ---------------------------------------------------------------------------------------------
const ver = JSON.parse(mod.UTF8ToString(mod._cad_version()));
ok(ver.module === 'orca-cad' && ver.recipeVersion === 6 && /^8\.0\.1/.test(ver.occt), `cad_version ${JSON.stringify(ver)}`);
console.log(`  import ${(tImport - t0).toFixed(0)} ms, instantiate ${(tInst - tImport).toFixed(0)} ms (wasm ${(wasmBinary.length / 1048576).toFixed(2)} MB)`);

const unknown = call('no_such_method', {});
ok(!unknown.ok && unknown.rc === 1 && unknown.error.code === -32601, 'unknown method → -32601');
const tools = C('describe_tools');
ok(tools.methods.length > 80 && tools.feature_types.length === 32 && tools.fields.length > 140, `describe_tools: ${tools.methods.length} methods, ${tools.fields.length} fields`);
ok(C('thread_standards').threads.some((t) => t.name === 'M6'), 'thread_standards has M6');
ok(C('hole_standards').holes.length === 12, 'hole_standards');

// ---- fixtures ---------------------------------------------------------------------------------
const v3 = call('doc_open', {}, readFileSync(join(fixtures, 'cad_recipe_v3.bin')));
ok(!v3.ok && v3.error.code === -32030 && /older version/.test(v3.error.message), `v3 refused: ${v3.error?.message}`);

for (const v of [4, 5]) {
  const bytes = readFileSync(join(fixtures, `cad_recipe_v${v}.bin`));
  const tOpen = performance.now();
  const o = C('doc_open', {}, bytes);
  const openMs = performance.now() - tOpen;
  ok(o.version === v && o.features === 34 && !o.read_only, `v${v}: opens, ${o.features} features, rebuild ok=${o.ok} ${o.error} (${openMs.toFixed(0)} ms)`);
  record[`v${v}.open_ms`] = Math.round(openMs);
  checkGolden(`v${v}.rebuild_ok`, o.ok); checkGolden(`v${v}.rebuild_error`, o.error);
  const doc = o.doc;
  const names0 = C('list_features', { doc }).features;
  const sc0 = C('describe_scene', { doc });
  ok(o.failed_feature === 2 && o.failed_feature_name === names0[2].name && sc0.failed_feature === 2 && sc0.failed_feature_name === names0[2].name && sc0.rebuild_ok === false,
     `v${v}: doc_open and describe_scene name the failing feature: ${o.failed_feature} "${o.failed_feature_name}"`);
  // byte-compatible round trip (before any edit)
  const same = C('doc_save', { doc });
  ok(same.verbatim && Buffer.compare(Buffer.from(same.blob), bytes) === 0, `v${v}: unchanged document saves byte-for-byte`);
  const canon = C('doc_save', { doc, canonical: true });
  const orcaW = C('doc_save', { doc, writer: 'orca' });
  ok(Buffer.compare(Buffer.from(canon.blob), Buffer.from(orcaW.blob)) === 0, `v${v}: our writer = Orca's serialize_recipe (${canon.bytes} B, v${canon.version})`);
  const re = C('doc_open', {}, canon.blob);
  const canon2 = C('doc_save', { doc: re.doc, canonical: true });
  ok(re.features === o.features && Buffer.compare(Buffer.from(canon.blob), Buffer.from(canon2.blob)) === 0, `v${v}: v6 re-serialisation is stable`);
  C('doc_close', { doc: re.doc });
  if (v === 5) {
    // every v5 frame is a byte prefix of its v6 frame (fields appended since, none reordered)
    const rd = (b) => { const dv = new DataView(b.buffer, b.byteOffset, b.byteLength); let p = 8; const fr = []; for (let i = 0; i < dv.getUint32(4, true); i++) { const n = dv.getUint32(p, true); fr.push(b.subarray(p + 4, p + 4 + n)); p += 4 + n; } return fr; };
    const a = rd(new Uint8Array(bytes)), b = rd(canon.blob);
    ok(a.length === b.length && a.every((f, i) => Buffer.compare(Buffer.from(f), Buffer.from(b[i].subarray(0, f.length))) === 0), 'v5 frames are prefixes of the v6 frames');
  }
  // Orca's golden tree is a serialisation-coverage history that does not rebuild as a whole (its own
  // test: "the only failure allowed is the golden tree's own geometry" — dress-ups too large). Find
  // the features that fail: suppress everything (force: the history is transactional, and a
  // failing document refuses any edit that does not fix it), then re-enable them in order, keeping
  // each one only if the rebuild still succeeds.
  let failing = [];
  if (!o.ok) {
    const tRep = performance.now();
    const known = golden[`v${v}.failing_features`];
    if (known) {
      // the set found by the search below, applied as one batch (one rebuild)
      failing = known;
      if (v === 5) {
        // a late failure: all but feature 29 repaired, the replay runs to 29 and names it
        const late = C('suppress_feature', { doc, indices: known.filter((i) => i !== 29), suppressed: true, force: true });
        ok(!late.ok && late.kept === true && late.failed_feature === 29 && late.failed_feature_name === names0[29].name && C('describe_scene', { doc }).failed_feature === 29,
           `v5: forced suppress keeps the toggle (kept, ok:false) and names the next failing feature: ${late.failed_feature} "${late.failed_feature_name}" (${(performance.now() - tRep).toFixed(0)} ms)`);
      }
      const r = C('suppress_feature', { doc, indices: known, suppressed: true });
      ok(r.ok && r.kept === true && r.failed_feature === -1, `v${v}: rebuilds with features ${JSON.stringify(known)} suppressed (${(performance.now() - tRep).toFixed(0)} ms, replayed from ${r.replayed_from}) ${r.error}`);
    } else {
      // search (slow: one full replay per feature, 10+ min; only when cad-golden.json has no list): suppress all but the base, re-enable in order
      const n = o.features;
      C('suppress_feature', { doc, indices: Array.from({ length: n }, (_, i) => i), suppressed: true, force: true });
      for (let i = 0; i < n; i++) if (!C('suppress_feature', { doc, index: i, suppressed: false }).ok) failing.push(i);
      ok(failing.length > 0, `v${v}: search: rebuilds with ${JSON.stringify(failing)} suppressed (${(performance.now() - tRep).toFixed(0)} ms)`);
    }
    const names = C('list_features', { doc }).features;
    console.log(`  v${v}: suppressed ${failing.map((i) => `${i} ${names[i].type} ${names[i].name}`).join(', ')}`);
  }
  checkGolden(`v${v}.failing_features`, failing);
  const scene = C('describe_scene', { doc });
  ok(scene.bodies.length > 0 && !scene.error, `v${v}: ${scene.bodies.length} bodies ${scene.error}`);
  // A body can legitimately end up with no faces (Orca only rejects a NULL shape): with features
  // suppressed, body indices shift and the tree's own plane Cut (GoldenSplit) keeps an empty side.
  const empty = scene.bodies.filter((b) => b.face_count === 0).map((b) => b.index);
  checkGolden(`v${v}.empty_bodies`, empty);
  // display meshes (get_mesh: no rebuild); one timed regenerate (full replay + display mesh) for v5
  const gm = C('get_mesh', { doc });
  let allDisplay = true;
  for (const m of gm.meshes) {
    if (empty.includes(m.body)) continue;
    const tri = u32(gm.blob, m.indices).length / 3, tf = i32(gm.blob, m.triFace);
    if (!(tri > 0 && tf.length === tri && new Set(tf).size === m.faces && m.edges > 0)) allDisplay = false;
  }
  ok(allDisplay && gm.meshes.length === scene.bodies.length, `v${v}: display meshes for ${gm.meshes.length - empty.length} non-empty bodies (every face has triangles, edge polylines; ${gm.blob.length} B)`);
  if (v === 5) {
    // faces_made_by from the rebuild's record = Orca's scratch replay (two cheap features)
    for (const i of [1, 7]) {
      const a = C('faces_made_by', { doc, index: i }), b = C('faces_made_by', { doc, index: i, replay: true });
      ok(!a.replayed && b.replayed && a.faces.length > 0 && JSON.stringify(a.faces) === JSON.stringify(b.faces), `v5: faces_made_by(${i}) from the record = Orca's replay (${a.faces.length} faces)`);
    }
    // an edit of the last feature replays only it; the result equals a full replay exactly
    const last = names0.length - 1, g0 = C('get_feature', { doc, index: last }).feature;
    const sigBefore = topoSig(doc);
    const tEd = performance.now();
    const ed = C('set_feature', { doc, index: last, fields: { mate_offset: g0.mate_offset + 2 } });
    const edMs = performance.now() - tEd;
    record['v5.cached_edit_ms'] = Math.round(edMs);
    ok(ed.ok && ed.replayed_from === last, `v5: edit of feature ${last} replays from ${ed.replayed_from} (${edMs.toFixed(0)} ms)`);
    const sigCached = topoSig(doc);
    const tReg = performance.now();
    const reg = C('regenerate', { doc, mesh: false, full: true });
    console.log(`  v${v}: regenerate (full replay) ${(performance.now() - tReg).toFixed(0)} ms (kernel ${reg.ms.toFixed(0)} ms)`);
    record[`v${v}.regenerate_ms`] = Math.round(reg.ms);
    ok(reg.ok && reg.generation > o.generation && reg.replayed_from === 0, `v${v}: regenerate {full} ok, generation ${reg.generation}`);
    ok(sigCached === topoSig(doc) && sigCached !== sigBefore, 'v5: the cached rebuild equals the full replay (every face and edge, volumes)');
    const u = C('undo', { doc });
    ok(u.ok && u.replayed_from === names0.length && topoSig(doc) === sigBefore, `v5: undo comes back from the replay cache (replayed from ${u.replayed_from}) to the same topology`);
    const fast = C('regenerate', { doc, mesh: false });
    ok(fast.ok && fast.replayed_from === names0.length && topoSig(doc) === sigBefore, `v5: regenerate without full: from the cache (${fast.ms.toFixed(0)} ms)`);
  }
  const plate = C('export_mesh', { doc });
  for (const m of plate.meshes) {
    if (empty.includes(m.body)) continue;
    const st = meshStats(plate.blob, m);
    const mp = C('mass_properties', { doc, body: m.body });
    ok(st.triangles > 0, `v${v} body ${m.body}: ${st.triangles} tris, ${st.openEdges} open edges, mesh vol ${st.volume.toFixed(3)}, B-rep vol ${mp.volume?.toFixed(3)}`);
    if (mp.is_solid) ok(st.openEdges === 0 || st.openEdges / st.edges < 0.001, `v${v} body ${m.body}: watertight-ish (${st.openEdges}/${st.edges})`);
    checkGolden(`v${v}.body${m.body}.volume`, mp.volume ?? 0, 1e-6);
    checkGolden(`v${v}.body${m.body}.faces`, scene.bodies[m.body].face_count);
    checkGolden(`v${v}.body${m.body}.edges`, scene.bodies[m.body].edge_count);
  }
  // the repaired (edited) document saves as v6 and reopens to the same bodies, ids and geometry
  const ed = C('doc_save', { doc });
  const re2 = C('doc_open', {}, ed.blob);
  const s2 = C('describe_scene', { doc: re2.doc });
  ok(!ed.verbatim && ed.version === 6 && re2.ok && s2.bodies.length === scene.bodies.length && s2.bodies.every((b, i) => b.face_count === scene.bodies[i].face_count && b.edge_count === scene.bodies[i].edge_count),
     `v${v}: edited document saves (v6) and reopens to the same bodies / face and edge counts`);
  let idsSame = true;
  for (const b of scene.bodies) {
    const a = C('query_topology', { doc, body: b.index }), c = C('query_topology', { doc: re2.doc, body: b.index });
    idsSame &&= a.faces.length === c.faces.length && a.faces.every((f, i) => f.surface === c.faces[i].surface && f.centroid.every((x, k) => Math.abs(x - c.faces[i].centroid[k]) < 1e-9));
  }
  ok(idsSame, `v${v}: face ids (surface type + centroid per id) identical after save/reopen`);
  C('doc_close', { doc }); C('doc_close', { doc: re2.doc });
  console.log(`  wasm memory after v${v}: ${memMB()} MB`);
}

// ---- unknown bytes and read-only ----------------------------------------------------------------
{
  const base = C('doc_open', {}, readFileSync(join(fixtures, 'cad_recipe_v5.bin')));
  const v6 = C('doc_save', { doc: base.doc, canonical: true }).blob;
  C('doc_close', { doc: base.doc });
  // append 6 unknown bytes to frame 1's tail, 3 to the extras block, 5 trailing
  const dv = new DataView(v6.buffer, v6.byteOffset, v6.byteLength);
  const n = dv.getUint32(4, true);
  const parts = []; let p = 8; const frames = [];
  for (let i = 0; i < n; i++) { const len = dv.getUint32(p, true); frames.push(v6.subarray(p + 4, p + 4 + len)); p += 4 + len; }
  const blocks = []; for (let k = 0; k < 3; k++) { const len = dv.getUint32(p, true); blocks.push(v6.subarray(p + 4, p + 4 + len)); p += 4 + len; }
  const tail1 = Uint8Array.from([0xde, 0xad, 0xbe, 0xef, 0x01, 0x02]), tailX = Uint8Array.from([7, 8, 9]), trail = Uint8Array.from([1, 2, 3, 4, 5]);
  const u32le = (x) => { const b = new Uint8Array(4); new DataView(b.buffer).setUint32(0, x, true); return b; };
  parts.push(v6.subarray(0, 8));
  frames.forEach((f, i) => { const g = i === 1 ? Buffer.concat([f, tail1]) : f; parts.push(u32le(g.length), g); });
  blocks.forEach((b, k) => { const g = k === 2 ? Buffer.concat([b, tailX]) : b; parts.push(u32le(g.length), g); });
  parts.push(trail);
  const withTails = Buffer.concat(parts);
  const o = C('doc_open', {}, withTails);
  ok(o.unknown.feature_tail_bytes === 6 && o.unknown.block_tail_bytes === 3 && o.unknown.trailing_bytes === 5, `unknown bytes detected ${JSON.stringify(o.unknown)}`);
  C('rename_feature', { doc: o.doc, index: 0, name: 'Renamed_Base' });   // edit → canonical write with tails
  C('move_feature', { doc: o.doc, index: 1, to: 1 });
  const saved = C('doc_save', { doc: o.doc }).blob;
  const has = (hay, needle) => Buffer.from(hay).indexOf(Buffer.from(needle)) >= 0;
  ok(!C('doc_save', { doc: o.doc }).verbatim && has(saved, tail1) && has(saved, tailX) && Buffer.compare(Buffer.from(saved.subarray(saved.length - 5)), Buffer.from(trail)) === 0,
     'edited document re-emits the unknown feature tail, block tail and trailing bytes');
  const ro = Buffer.from(withTails); ro.writeUInt32LE(7, 0);
  const r7 = C('doc_open', {}, ro);
  ok(r7.read_only && r7.features === 34, `v7 opens read-only: ${r7.read_only_reason}`);
  const denied = call('set_variable', { doc: r7.doc, name: 'x', expr: '1' });
  ok(!denied.ok && denied.error.code === -32020, 'read-only document refuses edits (-32020)');
  ok(Buffer.compare(Buffer.from(C('doc_save', { doc: r7.doc }).blob), ro) === 0, 'read-only document saves its original bytes');
  C('doc_close', { doc: o.doc }); C('doc_close', { doc: r7.doc });
}

// ---- OCCT 7.6 BRep fixture (Import feature) ----------------------------------------------------
{
  const brep = readFileSync(join(fixtures, 'cad_brep_occt76.brep'), 'utf8');
  const { doc } = C('doc_new');
  const a = C('add_feature', { doc, type: 'Import', name: 'Plate', fields: { brep } });
  const mp = C('mass_properties', { doc, body: 0 });
  const r = 3, fr = 0.5, ca = fr * fr * (1 - Math.PI / 4), cc = r + fr * (10 - 3 * Math.PI) / (3 * (4 - Math.PI));
  const expected = 30 * 30 * 5 - 9 * Math.PI * r * r * 5 - 9 * ca * 2 * Math.PI * cc;
  const topo = C('query_topology', { doc, body: 0 });
  ok(a.ok && topo.face_count === 24 && near(mp.volume, expected, 1e-4), `OCCT 7.6 BRep imports: ${topo.face_count} faces, volume ${mp.volume.toFixed(4)} (Pappus ${expected.toFixed(4)})`);
  const saved = C('doc_save', { doc }).blob;
  const re = C('doc_open', {}, saved);
  ok(re.ok && near(C('mass_properties', { doc: re.doc, body: 0 }).volume, mp.volume, 1e-9), 'Import feature round-trips through the recipe (embedded BRep)');
  C('doc_close', { doc }); C('doc_close', { doc: re.doc });
}

// ---- a design from scratch ----------------------------------------------------------------------
const timings = {};
const timed = (name, f) => { const t = performance.now(); const r = f(); timings[name] = Math.round(performance.now() - t); return r; };
const { doc } = C('doc_new', { modeling_origin: [0, 0, 0] });
// 1. sketch: 60 x 40 rectangle, fully dimensioned
C('sketch_begin', { doc, plane: 'XY' });
const add = C('sketch_add', { doc, rect: [-30, -20, 30, 20] });
ok(add.added === 4 && add.constraints >= 8, `sketch_add rect: ${add.constraints} inferred constraints (coincident + H/V), dof ${add.dof}`);
C('sketch_select', { doc, entities: [0] });
const sv = C('sketch_set_value', { doc, value: 60 });
C('sketch_select', { doc, entities: [1] });
C('sketch_set_value', { doc, value: 40 });
// fix the lower-left corner at the origin offset (-30,-20)
const fx = C('sketch_constrain', { doc, constraints: [{ type: 'distance_x', ea: -2, ra: 'p0', eb: 0, rb: 'p0', value: -30 }, { type: 'distance_y', ea: -2, ra: 'p0', eb: 0, rb: 'p0', value: -20 }] });
const desc = C('sketch_describe', { doc });
ok(sv.kind === 'length' && desc.buildable && desc.closed_loops.length === 1 && near(Math.abs(desc.closed_loops[0].area), 2400, 1e-9),
   `sketch: buildable, area ${desc.closed_loops[0]?.area}, dof ${desc.dof}, solve_ok ${desc.solve_ok} (fix ok=${fx.ok})`);
const bad = C('sketch_constrain', { doc, constraint: { type: 'distance', ea: 0, ra: 'p0', eb: 0, rb: 'p1', value: 10 } });
ok(bad.ok === false && bad.bad.length > 0, `conflicting dimension refused (bad ${JSON.stringify(bad.bad)})`);
const plan = C('sketch_plan', { doc, type: 'parallel', entities: [0, 2] });
ok(['apply', 'reject', 'ask_value'].includes(plan.kind), `sketch_plan parallel → ${plan.kind}/${plan.reason}`);
const cm = C('sketch_commit', { doc, name: 'Base sketch' });
ok(cm.ok && cm.sketch_index === 0, 'sketch_commit → feature 0');
// 2. extrude 15
const ex = timed('extrude', () => C('extrude', { doc, sketch: 0, distance: 15 }));
ok(ex.ok && ex.extrude_index === 1 && ex.bodies === 1, `extrude existing sketch: ${JSON.stringify({ ok: ex.ok, err: ex.error })}`);
let topo = C('query_topology', { doc, body: 0 });
ok(topo.face_count === 6 && topo.edge_count === 12 && near(C('mass_properties', { doc }).volume, 60 * 40 * 15), 'box: 6 faces, 12 edges, 36000 mm³');
// 3. fillet the four vertical edges
const vertical = topo.edges.filter((e) => e.kind === 'line' && Math.abs(e.p0[2] - e.p1[2]) > 14).map((e) => e.id);
const fl = timed('fillet', () => C('fillet', { doc, edge: vertical, radius: 4, generation: topo.generation }));
ok(fl.ok && vertical.length === 4, `fillet 4 vertical edges r4: ${fl.error || 'ok'}`);
const stale = call('fillet', { doc, edge: [0], radius: 1, generation: topo.generation });
ok(!stale.ok && stale.error.code === -32010, 'stale generation refused (-32010)');
// 4. shell, top face open
topo = C('query_topology', { doc, body: 0 });
const top = topo.faces.find((f) => f.kind === 'planar' && f.normal[2] > 0.99 && Math.abs(f.centroid[2] - 15) < 1e-6);
const sh = timed('shell', () => C('shell', { doc, thickness: 2, face: top.id, generation: topo.generation }));
ok(sh.ok, `shell 2 mm, top open: ${sh.error || 'ok'}`);
// 5. a boss on a second body, patterned, then unioned
const ex2 = C('extrude', { doc, plane: 'XY', profile: [[-25, -15], [-21, -15], [-21, -11], [-25, -11]], distance: 15, boolean: 'new' });
ok(ex2.ok && ex2.bodies === 2, `second body: ${ex2.error || 'ok'}`);
const pt = timed('pattern', () => C('pattern', { doc, count: 3, spacing: 23, dir: 0, body: 1 }));
ok(pt.ok, `linear pattern ×3: ${pt.error || 'ok'}`);
const bo = timed('boolean', () => C('boolean', { doc, op: 'union', target: 0, tool: 1 }));
ok(bo.ok && bo.bodies === 1, `boolean union: ${bo.error || 'ok'}`);
// 6. holes: a plain through hole and an M6 counterbore through the floor
const ho = timed('hole', () => C('hole', { doc, diameter: 5, through: true, x: 12, y: 0, plane: 'XY' }));
ok(ho.ok, `hole Ø5 through: ${ho.error || 'ok'}`);
const hs = C('hole_standard', { doc, designation: 'M6', style: 1, through: true, x: -12, y: 4, plane: 'XY' });
ok(hs.ok, `hole_standard M6 counterbore: ${hs.error || 'ok'}`);
// 7. external thread on a new rod body
// (a true cylinder: a 48-gon prism makes the thread boolean ~20x slower — 67 s at 0.05 mm)
C('sketch_begin', { doc, plane: { origin: [0, 60, 0], normal: [0, 0, 1], x_axis: [1, 0, 0], y_axis: [0, 1, 0] } });
C('sketch_add', { doc, type: 'circle', center: [0, 0], radius: 4 });
const rodSk = C('sketch_commit', { doc, name: 'Rod sketch' });
const rod = C('extrude', { doc, sketch: rodSk.sketch_index, distance: 12, boolean: 'new' });
ok(rod.ok, `rod body: ${rod.error || 'ok'}`);
const th = timed('thread', () => C('thread', { doc, designation: 'M8', height: 10, plane: { origin: [0, 60, 0], normal: [0, 0, 1], x_axis: [1, 0, 0], y_axis: [0, 1, 0] }, body: 1 }));
ok(th.ok, `M8 external thread on the rod: ${th.error || 'ok'}`);
const scene = C('describe_scene', { doc });
ok(scene.bodies.length === 2 && scene.features.length >= 10, `design: ${scene.features.length} features, ${scene.bodies.length} bodies`);
const vols = scene.bodies.map((b) => C('mass_properties', { doc, body: b.index }).volume);
scene.bodies.forEach((b, i) => { checkGolden(`design.body${i}.volume`, vols[i], 1e-6); checkGolden(`design.body${i}.faces`, b.face_count); checkGolden(`design.body${i}.edges`, b.edge_count); });
const plate = C('export_mesh', { doc });
for (const m of plate.meshes) {
  const st = meshStats(plate.blob, m);
  ok(st.openEdges === 0 && near(st.volume, vols[m.body], 2e-3), `design body ${m.body}: plate mesh watertight (${st.triangles} tris), mesh vol ${st.volume.toFixed(2)} ≈ ${vols[m.body].toFixed(2)}`);
}
// variables + expressions
const sv1 = C('set_variable', { doc, name: 'h', expr: '18' });
const se = C('set_feature_expr', { doc, feature: 1, field: 'distance', expr: 'h' });
ok(sv1.ok && se.ok && C('get_feature', { doc, index: 1 }).feature.distance === 18, 'variable drives the extrude distance');
{
  // set_variable: a variable no feature reads does not rebuild; one that is read does
  const g1 = C('describe_scene', { doc }).generation;
  const uv = C('set_variable', { doc, name: 'unused', expr: 'h*2 + 1' });
  ok(uv.ok && uv.rebuilt === false && uv.generation === g1 && uv.replayed_from === -1, `set_variable of an unused variable: no rebuild (rebuilt ${uv.rebuilt}, generation ${uv.generation})`);
  const used = C('set_variable', { doc, name: 'h', expr: '17' });
  ok(used.ok && used.rebuilt === true && used.generation > g1 && used.replayed_from === 1, `set_variable of a variable feature 1 reads: rebuilds, from feature ${used.replayed_from}`);
  const sc = C('describe_scene', { doc });
  ok(sc.variable_values.h === 17 && sc.variable_values.unused === 35 && Object.keys(sc.variable_errors).length === 0 && sc.variables.unused === 'h*2 + 1',
     `describe_scene variable values ${JSON.stringify(sc.variable_values)}`);
  const badv = C('set_variable', { doc, name: 'unused', expr: 'nope + 1' });
  ok(!badv.ok && badv.rebuilt === true && /unknown identifier/.test(badv.error) && badv.failed_feature === -1 && C('describe_scene', { doc }).variables.unused === 'h*2 + 1',
     `an unused variable that does not evaluate is still refused (rolled back): ${badv.error}`);
  ok(C('set_variable', { doc, name: 'unused', expr: '' }).rebuilt === false, 'deleting an unused variable: no rebuild');
}
C('set_feature_expr', { doc, feature: 1, field: 'distance', expr: '' });
C('set_feature', { doc, index: 1, fields: { distance: 15 } });
// faces made by, face/edge info, plane of face
const fmb = C('faces_made_by', { doc, index: 1 });
ok(fmb.faces.length > 0, `faces_made_by(extrude) → ${fmb.faces.length} faces`);
{
  // every feature: the rebuild's record gives Orca's own answer, without replaying
  const nf = C('list_features', { doc }).features.length;
  let same = true, any = 0;
  const t = performance.now();
  for (let i = 0; i < nf; i++) {
    const a = C('faces_made_by', { doc, index: i });
    if (a.replayed) same = false;
    any += a.faces.length;
  }
  const recMs = performance.now() - t;
  for (let i = 0; i < nf && same; i++) same = JSON.stringify(C('faces_made_by', { doc, index: i }).faces) === JSON.stringify(C('faces_made_by', { doc, index: i, replay: true }).faces);
  ok(same && any > 0, `faces_made_by of all ${nf} features from the record (${recMs.toFixed(0)} ms) = Orca's replays`);
}
const t2 = C('query_topology', { doc, body: 0 });
const floor = t2.faces.find((f) => f.kind === 'planar' && f.normal[2] < -0.99);
const fi = C('face_info', { doc, body: 0, face: floor.id, generation: t2.generation });
ok(fi.surface === 'plane' && fi.plane && fi.edges.length > 0, `face_info: ${fi.surface}, area ${fi.area.toFixed(1)}, ${fi.edges.length} edges, plane normal ${fi.plane?.normal}`);
const ei = C('edge_info', { doc, body: 0, edge: fi.edges[0], generation: t2.generation });
ok(ei.faces.includes(floor.id) && ei.length > 0, `edge_info: ${ei.curve}, length ${ei.length.toFixed(2)}, faces ${ei.faces}`);
// display mesh: every face id present
const gm = C('get_mesh', { doc, bodies: [0] });
const tf = i32(gm.blob, gm.meshes[0].triFace);
ok(new Set(tf).size === t2.face_count && gm.meshes[0].edges === t2.edge_count, `display mesh: ${tf.length} tris over ${new Set(tf).size}/${t2.face_count} faces, ${gm.meshes[0].edges} edge polylines`);

// ---- save, reopen: ids stable; edit a dimension ----------------------------------------------
const recipe = C('doc_save', { doc }).blob;
const re = C('doc_open', {}, recipe);
ok(re.ok && re.features === scene.features.length, `reopen: ${re.features} features, ok=${re.ok} (${re.ms.toFixed(0)} ms)`);
const t3 = C('query_topology', { doc: re.doc, body: 0 });
const sameIds = t2.faces.length === t3.faces.length && t2.faces.every((f, i) => f.id === t3.faces[i].id && f.centroid.every((x, k) => Math.abs(x - t3.faces[i].centroid[k]) < 1e-9))
  && t2.edges.every((e, i) => t3.edges[i] && e.p0.every((x, k) => Math.abs(x - t3.edges[i].p0[k]) < 1e-9));
ok(sameIds, 'face and edge ids (and their geometry) are identical after save/reopen');
// edit the sketch's width dimension 60 → 70
C('sketch_begin', { doc: re.doc, feature: 0 });
const kd = C('sketch_describe', { doc: re.doc });
const widthIdx = kd.constraints.findIndex((c) => c.type === 'distance' && c.ea === 0 && c.eb === 0);
const sc = C('sketch_set_constraint', { doc: re.doc, index: widthIdx, value: 70 });
ok(sc.ok && sc.solve_ok, `sketch dimension edited 60 → 70 (dof ${sc.dof})`);
const cm2 = C('sketch_commit', { doc: re.doc });
const vNew = C('mass_properties', { doc: re.doc, body: 0 }).volume;
ok(cm2.ok && vNew > vols[0], `regenerated after the edit: volume ${vols[0].toFixed(1)} → ${vNew.toFixed(1)} (${cm2.error || 'ok'})`);
checkGolden('design.edited.body0.volume', vNew, 1e-6);
// undo / redo
const u = C('undo', { doc: re.doc });
ok(u.ok && near(C('mass_properties', { doc: re.doc, body: 0 }).volume, vols[0], 1e-9), 'undo restores the previous volume');
const rd = C('redo', { doc: re.doc });
ok(rd.ok && near(C('mass_properties', { doc: re.doc, body: 0 }).volume, vNew, 1e-9), 'redo re-applies the edit');
// history edits: suppress, move, delete
const sp = C('suppress_feature', { doc: re.doc, index: 3, suppressed: true });
ok(sp.ok && C('list_features', { doc: re.doc }).features[3].enabled === false, `suppress shell: ${sp.error || 'ok'}`);
C('suppress_feature', { doc: re.doc, index: 3, suppressed: false });
const mv = C('move_feature', { doc: re.doc, index: 1, to: 0 });
ok(!mv.ok && /cannot come before/.test(mv.error), `move consumer above its sketch refused: ${mv.error}`);
{
  const { doc: d3 } = C('doc_new');
  C('extrude', { doc: d3, width: 10, height: 10, distance: 5 });
  C('extrude', { doc: d3, plane: { origin: [20, 0, 0], normal: [0, 0, 1], x_axis: [1, 0, 0], y_axis: [0, 1, 0] }, width: 6, height: 6, distance: 5 });
  const del = C('delete_feature', { doc: d3, index: 0 });
  const left = C('list_features', { doc: d3, full: true }).features;
  ok(del.ok && JSON.stringify(del.removed) === '[0,1]' && left.length === 2 && left[1].sketch_ref === 0 && del.bodies === 1,
     `delete a sketch cascades to its extrude; later refs remapped (removed ${JSON.stringify(del.removed)}) ${del.error}`);
  C('undo', { doc: d3 });
  ok(C('describe_scene', { doc: d3 }).features.length === 4, 'undo brings the deleted features back');
  C('doc_close', { doc: d3 });
}

// ---- failing feature, suppress `kept`, cancel, progress, replay cache ---------------------------
{
  const d = re.doc;
  const nf = C('list_features', { doc: d }).features.length;
  const sig0 = topoSig(d), gen0 = C('describe_scene', { doc: d }).generation;
  // a failing edit names the feature (here the new one, which does not stay) and leaves the
  // document exactly as it was, generation included (no replay to restore it)
  const big = C('fillet', { doc: d, edge: [0], radius: 500, body: 0 });
  const scB = C('describe_scene', { doc: d });
  ok(!big.ok && big.failed_feature === nf && big.failed_feature_name !== '' && big.generation === gen0 && scB.rebuild_ok && scB.failed_feature === -1 && scB.features.length === nf && topoSig(d) === sig0,
     `failing edit: failed_feature ${big.failed_feature} "${big.failed_feature_name}", rolled back without a replay (generation ${big.generation})`);
  // suppress: kept = the toggle is in the document
  const s1 = C('suppress_feature', { doc: d, index: 3, suppressed: true });
  const s2 = C('suppress_feature', { doc: d, indices: [3], suppressed: false });
  const s3 = C('suppress_feature', { doc: d, index: 1, suppressed: true });
  const s4 = C('suppress_feature', { doc: d, indices: [1], suppressed: true });
  const s5 = C('suppress_feature', { doc: d, indices: [1], suppressed: true, force: true });
  ok(s1.ok && s1.kept && s2.ok && s2.kept && !s3.ok && s3.kept === false && !s4.ok && s4.kept === false && !s5.ok && s5.kept === true && s5.failed_feature >= 0,
     `suppress kept: ok→true, refused→false, forced→true with ok:false (failed_feature ${s5.failed_feature} "${s5.failed_feature_name}")`);
  C('undo', { doc: d });
  ok(topoSig(d) === sig0, 'undo of the forced suppress restores the design');
  // progress: one call per replayed feature, then featureIndex == featureCount
  const calls = [];
  mod.cadProgress = (i, n, name, can) => { calls.push([i, n, name, can]); return false; };
  const rg = C('regenerate', { doc: d, mesh: false, full: true });
  ok(rg.ok && calls.length === nf + 1 && calls.every((c, k) => c[0] === k && c[1] === nf && c[3] === true) && calls[nf][2] === '' && calls[0][2] !== '',
     `cadProgress: ${calls.length} calls (${calls.slice(0, 3).map((c) => c.slice(0, 3).join('/')).join(', ')} …)`);
  // cancel a regenerate at feature 3: nothing changes
  const genR = rg.generation;
  mod.cadProgress = (i) => i === 3;
  const cr = C('regenerate', { doc: d, mesh: false, full: true });
  ok(!cr.ok && cr.cancelled && cr.generation === genR && cr.error === 'rebuild cancelled' && topoSig(d) === sig0 && C('describe_scene', { doc: d }).rebuild_ok,
     'cancelled regenerate: document unchanged (same generation and topology)');
  // cancel an edit: rolled back, no undo step
  const canUndo = C('describe_scene', { doc: d }).can_undo, d1 = C('get_feature', { doc: d, index: 1 }).feature.distance;
  calls.length = 0;
  mod.cadProgress = (i, n, name, can) => { calls.push([i, can]); return i >= 2; };
  const ce = C('set_feature', { doc: d, index: 1, fields: { distance: d1 + 1 } });
  ok(!ce.ok && ce.cancelled && ce.failed_feature === -1 && C('get_feature', { doc: d, index: 1 }).feature.distance === d1 && ce.generation === genR && topoSig(d) === sig0 && C('describe_scene', { doc: d }).can_undo === canUndo,
     `cancelled edit: rolled back (${calls.length} progress calls, from feature ${calls[0]?.[0]})`);
  // an Orca transactional edit restores by replaying: that restore is never cancellable
  calls.length = 0;
  mod.cadProgress = (i, n, name, can) => { calls.push([i, can]); return true; };
  const cs = C('suppress_feature', { doc: d, index: 3, suppressed: true });
  ok(!cs.ok && cs.cancelled && cs.kept === false && calls[0][1] === true && calls.some((c) => c[1] === false) && topoSig(d) === sig0,
     `cancelled suppress: rolled back; the restore ran uncancellable (${calls.filter((c) => !c[1]).length} calls)`);
  // cancel while opening: the document is open, with no bodies, and rebuilds later
  mod.cadProgress = (i) => i === 2;
  const co = C('doc_open', {}, C('doc_save', { doc: d }).blob);
  mod.cadProgress = undefined;
  const rr = C('regenerate', { doc: co.doc, mesh: false });
  ok(!co.ok && co.cancelled && co.bodies === 0 && rr.ok && topoSig(co.doc) === sig0, 'cancelled doc_open: open with no bodies; regenerate then gives the design');
  C('doc_close', { doc: co.doc });
  // replay cache: an edit of a late feature replays from it, and equals a full replay
  const late = C('list_features', { doc: d }).features.findLastIndex((f) => f.type === 'Hole'), lf = C('get_feature', { doc: d, index: late }).feature;
  const t = performance.now();
  const le = C('set_feature', { doc: d, index: late, fields: { hole_x: lf.hole_x - 1 } });
  const leMs = performance.now() - t;
  const sigC = topoSig(d);
  const fr = C('regenerate', { doc: d, mesh: false, full: true });
  ok(le.ok && le.replayed_from === late && fr.ok && sigC === topoSig(d) && sigC !== sig0,
     `edit of feature ${late} (${lf.type}) replays from ${le.replayed_from} (${leMs.toFixed(0)} ms, full replay ${fr.ms.toFixed(0)} ms) and equals the full replay`);
  // saving after cached rebuilds writes the same recipe as a fresh document would
  const reo = C('doc_open', {}, C('doc_save', { doc: d }).blob);
  ok(reo.ok && topoSig(reo.doc) === sigC && Buffer.compare(Buffer.from(C('doc_save', { doc: reo.doc, canonical: true }).blob), Buffer.from(C('doc_save', { doc: d, canonical: true }).blob)) === 0,
     'cached document saves and reopens to the same recipe and topology');
  C('doc_close', { doc: reo.doc });
  C('undo', { doc: d });
}
// not a recipe
for (const junk of [Uint8Array.from([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]), Uint8Array.from([6, 0, 0, 0, 255, 255, 255, 255])]) {
  const r = call('doc_open', {}, junk);
  ok(!r.ok && r.error.code === -32031 && /not a Design-tab recipe/.test(r.error.message), `foreign bytes → -32031 ${r.error?.message}`);
}

// ---- STEP round trip --------------------------------------------------------------------------
const step = timed('export_step', () => C('export_step', { doc, bodies: [0] }));
ok(step.bytes > 1000 && /ISO-10303-21/.test(Buffer.from(step.blob.subarray(0, 64)).toString()), `export_step: ${step.bytes} B`);
const { doc: d2 } = C('doc_new');
const imp = timed('import_step', () => C('import_step', { doc: d2 }, step.blob));
const vImp = imp.ok ? C('mass_properties', { doc: d2, body: 0 }).volume : NaN;
const tImp = imp.ok ? C('query_topology', { doc: d2, body: 0 }) : { face_count: -1 };
ok(imp.ok && near(vImp, vols[0], 1e-6) && tImp.face_count === t2.face_count, `STEP round trip: volume ${vImp?.toFixed?.(3)}, ${tImp.face_count} faces`);
{
  const m0 = Buffer.from(C('export_mesh', { doc: d2 }).blob);
  const re = C('doc_open', {}, C('doc_save', { doc: d2, canonical: true }).blob);
  const m1 = Buffer.from(C('export_mesh', { doc: re.doc }).blob);
  ok(Buffer.compare(m0, m1) === 0, `imported STEP body: plate mesh identical after save → reopen (${m0.length} B)`);
  C('doc_close', { doc: re.doc });
}
const sm = C('step_to_mesh', {}, step.blob);
ok(sm.solids === 1 && meshStats(sm.blob, sm.meshes[0]).openEdges === 0, 'step_to_mesh (stateless) → watertight mesh');

// ---- v1.2: no-op shells refused, generation on add/set_feature, primitives ---------------------
{
  const vol = (d, body = 0) => C('mass_properties', { doc: d, body }).volume;
  // a 30 x 30 x 30 box whose side face 5 has its own edges filleted: OCCT "succeeds" and changes nothing
  const { doc: d } = C('doc_new', { modeling_origin: [0, 0, 0] });
  C('extrude', { doc: d, width: 30, height: 30, distance: 30 });
  const g0 = C('query_topology', { doc: d, body: 0 }).generation;
  C('fillet', { doc: d, edge: C('face_info', { doc: d, body: 0, face: 5 }).edges, radius: 2, generation: g0 });
  const tq = C('query_topology', { doc: d, body: 0 });
  const nf = C('list_features', { doc: d }).features.length, v0 = vol(d);
  const noop = C('shell', { doc: d, thickness: 1.5, face: 5, generation: tq.generation });
  ok(!noop.ok && /changed nothing/.test(noop.error) && noop.failed_feature === nf && noop.generation === tq.generation &&
     C('list_features', { doc: d }).features.length === nf && near(vol(d), v0) && !C('describe_scene', { doc: d }).can_redo,
     `shell that leaves the body unchanged → ok:false, rolled back: ${noop.error}`);
  const noop2 = C('add_feature', { doc: d, type: 'Shell', fields: { shell_face: 5, shell_thickness: 1.5 }, generation: tq.generation });
  ok(!noop2.ok && /changed nothing/.test(noop2.error) && C('list_features', { doc: d }).features.length === nf, 'add_feature Shell no-op → ok:false');
  const top = tq.faces.find((f) => f.kind === 'planar' && f.normal[2] > 0.99);
  const good = C('shell', { doc: d, thickness: 1.5, face: top.id, generation: tq.generation });
  const vShell = vol(d);
  ok(good.ok && vShell < v0 * 0.5, `shell with the filleted top open hollows it (${v0.toFixed(0)} → ${vShell.toFixed(0)})`);
  checkGolden('shell_filleted_top.volume', vShell);
  const g1 = C('describe_scene', { doc: d }).generation;
  const edit = C('set_feature', { doc: d, index: good.shell_index, fields: { shell_face: 5 }, generation: g1 });
  ok(!edit.ok && /changed nothing/.test(edit.error) && C('get_feature', { doc: d, index: good.shell_index }).feature.shell_face === top.id && near(vol(d), vShell),
     'set_feature to a no-op shell face → ok:false, rolled back');
  // closed hollow: Orca's Shell without an open face returns only the inward offset → refused
  const { doc: h } = C('doc_new', { modeling_origin: [0, 0, 0] });
  C('extrude', { doc: h, width: 20, height: 20, distance: 10 });
  const closed = C('shell', { doc: h, thickness: 1.5, face: -1 });
  ok(!closed.ok && /closed hollow/.test(closed.error) && near(vol(h), 4000), `shell {face:-1} → ok:false: ${closed.error}`);
  C('fillet', { doc: h, face_group: 'All', radius: 2 });
  const closed2 = C('shell', { doc: h, thickness: 1.5, face: -1 });
  ok(!closed2.ok && /sheet|closed hollow/.test(closed2.error), `shell {face:-1} on a filleted box (OCCT gives a sheet) → ok:false: ${closed2.error}`);
  C('undo', { doc: h });
  // a solid with an inner shell (cavity) measures as outer minus cavity: 20x20x10 − 17x17x7 = 1977
  C('extrude', { doc: h, profile: [[-8.5, -8.5], [8.5, -8.5], [8.5, 8.5], [-8.5, 8.5]], distance: 7, plane: { origin: [0, 0, 1.5], normal: [0, 0, 1], x_axis: [1, 0, 0], y_axis: [0, 1, 0] }, boolean: 'new' });
  const cav = C('boolean', { doc: h, op: 'subtract', target: 0, tool: 1 });
  const mpc = C('mass_properties', { doc: h, body: 0 });
  ok(cav.ok && mpc.is_solid && near(mpc.volume, 1977, 1e-9) && C('query_topology', { doc: h, body: 0 }).face_count === 12,
     `mass_properties of a solid with a cavity: ${mpc.volume} (expected 1977)`);
  checkGolden('cavity_box.volume', mpc.volume);
  // generation on add_feature / set_feature: checked only when the fields carry face / edge ids
  const gNow = C('describe_scene', { doc: d }).generation;
  const stale1 = call('add_feature', { doc: d, type: 'Fillet', fields: { dressup_edges: [0], dressup_size: 0.5 }, generation: gNow - 1 });
  ok(!stale1.ok && stale1.error.code === -32010, `add_feature with edge ids and a stale generation → -32010 (${stale1.error?.message})`);
  const stale2 = call('set_feature', { doc: d, index: good.shell_index, fields: { shell_face: top.id }, generation: gNow - 1 });
  ok(!stale2.ok && stale2.error.code === -32010, 'set_feature with a face id and a stale generation → -32010');
  const noIds = C('set_feature', { doc: d, index: good.shell_index, fields: { shell_thickness: 1.2 }, generation: gNow - 1 });
  ok(noIds.ok, `set_feature without ids ignores a stale generation: ${noIds.error || 'ok'}`);
  const gCur = noIds.generation;
  const cur = C('add_feature', { doc: d, type: 'Chamfer', fields: { face_group: 'Bottom', dressup_size: 0.5 }, generation: gCur });
  ok(cur.ok, `add_feature (no ids) with the current generation: ${cur.error || 'ok'}`);
  const noGen = C('set_feature', { doc: d, index: good.shell_index, fields: { shell_face: top.id, shell_thickness: 1.5 } });
  ok(noGen.ok, `set_feature with ids and no generation still works: ${noGen.error || 'ok'}`);
  const badGen = call('add_feature', { doc: d, type: 'Fillet', fields: { dressup_edges: [0] }, generation: 'x' });
  ok(!badGen.ok && badGen.error.code === -32602, 'ill-typed generation → -32602');
  C('doc_close', { doc: d });
  C('doc_close', { doc: h });

  // primitives: one Import feature each (rigid B-rep, desktop-Orca readable)
  const { doc: p } = C('doc_new', { modeling_origin: [0, 0, 0] });
  const PI = Math.PI;
  const prims = [
    ['box', { width: 30, depth: 20, height: 10 }, 6000, 6],
    ['cylinder', { radius: 5, height: 12, x: 40 }, PI * 25 * 12, 3],
    ['cone', { radius1: 6, radius2: 2, height: 9, x: 80 }, PI * 9 * (36 + 12 + 4) / 3, 3],
    ['sphere', { radius: 7, y: 40 }, 4 / 3 * PI * 343, 1],
    ['torus', { major_radius: 10, minor_radius: 3, x: 40, y: 40 }, 2 * PI * PI * 10 * 9, 1],
  ];
  let allOk = true;
  for (const [m, args, v, faces] of prims) {
    const n0 = C('list_features', { doc: p }).features.length;
    const r = C(m, { doc: p, ...args });
    const fs = C('list_features', { doc: p }).features;
    const b = C('describe_scene', { doc: p }).bodies.at(-1);
    const vv = r.ok ? vol(p, r.body) : NaN;
    const onPlane = b && Math.abs(b.bbox.min[2]) < 0.02;   // describe_scene bbox: OCCT's, enlarged on curved faces
    const good1 = r.ok && fs.length === n0 + 1 && fs.at(-1).type === 'Import' && r.primitive_index === n0 && near(vv, v, 1e-6) && b.face_count === faces && onPlane;
    ok(good1, `${m}: one Import feature, volume ${vv.toFixed?.(3)} (expected ${v.toFixed(3)}), ${b?.face_count} faces, base on the plane ${onPlane}`);
    allOk &&= good1;
    checkGolden(`primitive.${m}.volume`, vv);
  }
  const before = C('describe_scene', { doc: p }).bodies.map((b) => [b.face_count, b.edge_count]);
  const vols0 = before.map((_, i) => vol(p, i));
  const u = C('undo', { doc: p });
  ok(u.ok && C('list_features', { doc: p }).features.length === 4, 'undo removes a primitive in one step');
  C('redo', { doc: p });
  // round trip through the bridge's writer and through Orca's own serialize/deserialize
  for (const writer of [undefined, 'orca']) {
    const bytes = C('doc_save', { doc: p, ...(writer ? { writer } : {}) }).blob;
    const re = C('doc_open', {}, bytes);
    const after = C('describe_scene', { doc: re.doc }).bodies.map((b) => [b.face_count, b.edge_count]);
    const vols1 = after.map((_, i) => vol(re.doc, i));
    ok(re.ok && JSON.stringify(after) === JSON.stringify(before) && vols1.every((x, i) => near(x, vols0[i], 1e-9)),
       `primitives round-trip through ${writer ?? 'the bridge'} writer (${bytes.length} B): same faces, edges, volumes`);
    // the bridge saves Import B-reps without the display tessellation the rebuild put on them
    if (!writer) ok(bytes.length < 200_000, `five primitives save in ${bytes.length} B (no tessellation in the recipe)`);
    C('doc_close', { doc: re.doc });
  }
  // the plate mesh of an Import body is the same after save → reopen (linked plate objects keep their mesh)
  {
    const m0 = Buffer.from(C('export_mesh', { doc: p }).blob);
    const re = C('doc_open', {}, C('doc_save', { doc: p, canonical: true }).blob);
    const m1 = Buffer.from(C('export_mesh', { doc: re.doc }).blob);
    ok(Buffer.compare(m0, m1) === 0, `primitives: plate meshes identical after save → reopen (${m0.length} B)`);
    C('doc_close', { doc: re.doc });
  }
  const refused = call('cone', { doc: p, radius1: 3, radius2: 3, height: 5 });
  ok(!refused.ok && refused.error.code === -32602, 'cone with equal radii → -32602');
  // placement on a plane frame: a box on the XZ plane grows along +Y (Orca's XZ normal)
  const bx = C('box', { doc: p, plane: 'XZ', width: 10, depth: 10, height: 4 });
  const bb = C('describe_scene', { doc: p }).bodies.at(-1).bbox;
  ok(bx.ok && near(bb.min[1], 0, 1e-6) && near(bb.max[1], 4, 1e-6), `box on XZ: y ${bb.min[1].toFixed(3)}..${bb.max[1].toFixed(3)}`);
  C('doc_close', { doc: p });
}

// ---- v1.2 sketch fixes (K1–K8 from the sketcher slice) -----------------------------------------
{
  const { doc: d } = C('doc_new', { modeling_origin: [0, 0, 0] });
  const fresh = (ents) => { call('sketch_cancel', { doc: d }); C('sketch_begin', { doc: d, plane: 'XY' }); if (ents) C('sketch_add', { doc: d, entities: ents }); };
  const L = (p0, p1) => ({ type: 'line', p0, p1 });
  const has = (p, q) => Math.hypot(p[0] - q[0], p[1] - q[1]) < 1e-6;
  const dirDeg = (e) => Math.atan2(e.p1[1] - e.p0[1], e.p1[0] - e.p0[0]) * 180 / Math.PI;
  // K3/K7: kind mismatches are -32602, never a trap
  fresh([L([0, 0], [10, 0]), L([0, 5], [10, 8]), { type: 'circle', center: [20, 0], radius: 3 }]);
  let allRefused = true;
  for (const con of [{ type: 'radius', ea: 0, value: 5 }, { type: 'diameter', ea: 0, value: 5 }, { type: 'equal_radius', ea: 0, eb: 1 },
    { type: 'tangent', ea: 0, eb: 1 }, { type: 'concentric', ea: 0, eb: 1 }, { type: 'parallel', ea: 0, eb: 2 }, { type: 'angle', ea: 2, eb: 0, value: 30 },
    { type: 'radius', ea: -3, value: 2 }, { type: 'equal_radius', ea: 2, eb: 1 }, { type: 'equal_length', ea: 2, eb: 2 }]) {
    const r = call('sketch_constrain', { doc: d, constraint: con });
    if (r.ok || r.error.code !== -32602) { allRefused = false; console.log('   not refused:', JSON.stringify(con), JSON.stringify(r.error ?? r.result).slice(0, 120)); }
  }
  ok(allRefused, 'radius / diameter / equal_radius on lines, tangent / concentric between lines, parallel / angle / equal_length with circles → -32602 (no trap)');
  ok(C('sketch_constrain', { doc: d, constraint: { type: 'radius', ea: 2, value: 4 } }).ok, 'radius on a circle still works');
  const se = call('sketch_set_entity', { doc: d, index: 2, entity: L([20, 0], [25, 0]) });
  ok(se.ok && Array.isArray(se.result.skipped) && se.result.skipped.length >= 1, `sketch_set_entity circle → line under its radius constraint: solved without it, no trap (skipped ${JSON.stringify(se.result?.skipped)})`);
  // K4: angle in degrees on the wire
  fresh([L([0, 0], [10, 0]), L([0, 0], [10, 3])]);
  C('sketch_constrain', { doc: d, constraints: [{ type: 'fix', ea: 0, ra: 'p0' }, { type: 'fix', ea: 0, ra: 'p1' }] });
  const an = C('sketch_constrain', { doc: d, constraint: { type: 'angle', ea: 0, eb: 1, value: 60 } });
  let ds = C('sketch_describe', { doc: d });
  ok(an.ok && Math.abs(Math.abs(dirDeg(ds.entities[1])) - 60) < 1e-6 && Math.abs(ds.constraints.at(-1).value - 60) < 1e-9,
     `angle value 60 → lines ${dirDeg(ds.entities[1]).toFixed(3)}° apart, read back as ${ds.constraints.at(-1).value}`);
  const ci = ds.constraints.findIndex((c) => c.type === 'angle');
  const sc = C('sketch_set_constraint', { doc: d, index: ci, value: 30 });
  ds = C('sketch_describe', { doc: d });
  ok(sc.ok && Math.abs(Math.abs(dirDeg(ds.entities[1])) - 30) < 1e-6, `sketch_set_constraint angle 30 (degrees): ${dirDeg(ds.entities[1]).toFixed(3)}°`);
  fresh([L([0, 0], [10, 0]), L([0, 0], [10, 3])]);
  C('sketch_select', { doc: d, entities: [0, 1] });
  const sv = C('sketch_set_value', { doc: d, value: 45 });
  ds = C('sketch_describe', { doc: d });
  const sva = ds.constraints.find((c) => c.type === 'angle');
  ok(sv.ok && Math.abs(Math.abs(dirDeg(ds.entities[1]) - dirDeg(ds.entities[0])) - 45) < 1e-6 && Math.abs(sva?.value - 45) < 1e-9,
     `sketch_set_value 45 on two lines: before ${Number(sv.before).toFixed(3)}°, now ${(dirDeg(ds.entities[1]) - dirDeg(ds.entities[0])).toFixed(3)}°, constraint ${sva?.value}`);
  // K1: fillet of a constrained rectangle corner
  fresh(null);
  C('sketch_add', { doc: d, rect: [0, 0, 20, 10] });
  let sd = C('sketch_describe', { doc: d });
  const isH = (e, y) => e.type === 'line' && Math.abs(e.p0[1] - y) < 1e-9 && Math.abs(e.p1[1] - y) < 1e-9;
  const isV = (e, x) => e.type === 'line' && Math.abs(e.p0[0] - x) < 1e-9 && Math.abs(e.p1[0] - x) < 1e-9;
  const bottom = sd.entities.findIndex((e) => isH(e, 0)), right = sd.entities.findIndex((e) => isV(e, 20));
  const fo = C('sketch_ops', { doc: d, op: 'fillet', a: bottom, b: right, radius: 2 });
  sd = C('sketch_describe', { doc: d });
  const arc = sd.entities[fo.first_index], lb = sd.entities[bottom], lr = sd.entities[right];
  const tangents = sd.constraints.filter((c) => c.type === 'tangent' && c.ea === fo.first_index).length;
  const cornerGone = !sd.constraints.some((c) => c.type === 'coincident' && [c.ea, c.eb].includes(bottom) && [c.ea, c.eb].includes(right));
  ok(fo.ok && arc?.type === 'arc' && Math.abs(arc.radius - 2) < 1e-6 && has(arc.center, [18, 2]) && (has(lb.p0, [18, 0]) || has(lb.p1, [18, 0])) && (has(lr.p0, [20, 2]) || has(lr.p1, [20, 2])) && tangents === 2 && cornerGone && sd.solve_ok !== false,
     `fillet r2 at a constrained corner: arc r=${arc?.radius?.toFixed?.(3)} at ${arc?.center?.map((v) => v.toFixed(3))}, legs ${JSON.stringify([lb.p0, lb.p1, lr.p0, lr.p1])}, ${tangents} tangents, corner coincident ${cornerGone ? 'replaced' : 'kept'}`);
  // K2: trim with constraints on the moving end
  fresh([L([0, 0], [20, 0]), L([10, -5], [10, 5])]);
  C('sketch_constrain', { doc: d, constraints: [{ type: 'fix', ea: 0, ra: 'p1' }, { type: 'fix', ea: 0, ra: 'p0' }] });
  const tr = C('sketch_ops', { doc: d, op: 'trim', entity: 0, pick: [16, 0] });
  const tds = C('sketch_describe', { doc: d }), t0 = tds.entities[0];
  const len = Math.hypot(t0.p1[0] - t0.p0[0], t0.p1[1] - t0.p0[1]);
  const left = tds.constraints.filter((c) => [c.ea, c.eb, c.ec].includes(0)).length;
  ok(tr.ok && Math.abs(len - 10) < 1e-6 && tr.released >= 1 && left === 0, `trim with both ends fixed: the subject's ${tr.released} constraint(s) dropped (${left} left), length ${len.toFixed(3)}`);
  // K5: the conflicting set
  fresh(null);
  C('sketch_add', { doc: d, entities: [L([0, 0], [10, 0])], infer: false });
  C('sketch_constrain', { doc: d, constraints: [{ type: 'fix', ea: 0, ra: 'p0' }, { type: 'horizontal', ea: 0, ra: 'p0', eb: 0, rb: 'p1' }, { type: 'distance', ea: 0, ra: 'p0', eb: 0, rb: 'p1', value: 10 }] });
  const bad = C('sketch_constrain', { doc: d, constraint: { type: 'distance', ea: 0, ra: 'p0', eb: 0, rb: 'p1', value: 12 } });
  ok(!bad.ok && bad.conflicts?.includes(2) && bad.bad.includes(3) && bad.bad.includes(2), `refused distance: conflicting set ${JSON.stringify(bad.conflicts)}, bad ${JSON.stringify(bad.bad)} (solver alone: ${JSON.stringify(bad.solver_bad)})`);
  // K6: ellipse_arc ends derived
  fresh([{ type: 'ellipse_arc', center: [5, 5], radius: 4, rminor: 2, rotation: 0, start_angle: 0, end_angle: Math.PI / 2 }]);
  const ea = C('sketch_describe', { doc: d }).entities[0];
  ok(has(ea.p0, [9, 5]) && has(ea.p1, [5, 7]), `ellipse_arc ends derived: ${ea.p0} → ${ea.p1}`);
  // K8: an empty sketch is not committed (desktop Orca: "Sketch empty — nothing committed")
  fresh(null);
  const ec = call('sketch_commit', { doc: d });
  ok(!ec.ok && ec.error.code === -32602 && C('list_features', { doc: d }).features.length === 0, `empty sketch_commit refused: ${ec.error?.message}`);
  // …but a session with only imported regions (Text / SVG) commits them in one call (`fields`)
  const rc = C('sketch_commit', { doc: d, name: 'Text', fields: { imported_regions: [[[[0, 0], [10, 0], [10, 10], [0, 10]]]] } });
  const rx = rc.ok ? C('extrude', { doc: d, sketch: rc.sketch_index, distance: 2 }) : { ok: false };
  ok(rc.ok && rc.fields_applied && rx.ok && near(C('mass_properties', { doc: d, body: 0 }).volume, 200, 1e-9), `sketch_commit {fields: imported_regions} of an entity-less session → extrudes 200 mm³`);
  C('sketch_begin', { doc: d, plane: 'XY' });
  ok(call('sketch_commit', { doc: d, fields: { entities: [] } }).error?.code === -32602, "sketch_commit 'fields' cannot set the session's own fields");
  call('sketch_cancel', { doc: d });
  C('doc_close', { doc: d });
}

// ---- misc --------------------------------------------------------------------------------------
const pv =C('feature_preview', { doc, type: 'Fillet', fields: { dressup_size: 0.5, face_group: 'Top', target_body: 0 } });
ok(pv.ok && pv.meshes.length === 2, `feature_preview (ghost): ${pv.error || 'ok'}`);
const sl = C('slice_body', { doc, body: 0, plane: 'XY', offset: 7 });
ok(sl.closed_count >= 2, `slice_body at z=7: ${sl.contour_count} contours`);
const dt = C('datums', { doc });
ok(dt.base_planes.length === 3, 'datums');
console.log('  timings (ms):', JSON.stringify(timings));
console.log(`  wasm memory (peak): ${memMB()} MB`);
record.timings = timings;

if (update) { writeFileSync(goldenPath, JSON.stringify(record, null, 2) + '\n'); console.log(`wrote ${goldenPath}`); }
console.log(failures ? `\n${failures} FAILED` : '\nall passed');
process.exit(failures ? 1 : 0);
