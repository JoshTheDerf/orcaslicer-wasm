#!/usr/bin/env node
// CAD kernel module: timings of the calls a Design UI makes repeatedly, on Orca's golden fixture
// repaired (cad_recipe_v5.bin with features 2 3 5 6 29 suppressed: 29 features, 6 bodies).
//   node tests/cad-perf.mjs [build-dir] [--quick]
// Works against older builds too (fields they lack print as "-"). Prints one line per measurement
// and a JSON summary at the end. Not part of the pass/fail suites: it asserts only that the calls
// give the same answers as a full replay where both are available.
import { readFileSync } from 'node:fs';
import { resolve, dirname, join } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

const here = dirname(fileURLToPath(import.meta.url));
const args = process.argv.slice(2);
const dir = resolve(args.find((a) => !a.startsWith('--')) ?? join(here, '..', 'build-cad'));
const quick = args.includes('--quick');
const { default: factory } = await import(pathToFileURL(join(dir, 'cad.mjs')).href);
const mod = await factory({ wasmBinary: readFileSync(join(dir, 'cad.wasm')), print: () => {}, printErr: () => {} });
const enc = new TextEncoder(), dec = new TextDecoder();
function call(method, params = {}, blob = null) {
  const w = (b) => { const p = mod._malloc(Math.max(1, b.byteLength)) >>> 0; mod.HEAPU8.set(b, p); return p; };
  const m = enc.encode(method), j = enc.encode(JSON.stringify(params)), b = blob ?? new Uint8Array(0);
  const mp = w(m), jp = w(j), bp = w(b), outs = mod._malloc(16) >>> 0;
  mod.HEAPU32.fill(0, outs >>> 2, (outs >>> 2) + 4);
  mod._cad_call(mp, m.byteLength, jp, j.byteLength, bp, b.byteLength, outs, outs + 4, outs + 8, outs + 12);
  const H = mod.HEAPU32;
  const pj = H[outs >>> 2], lj = H[(outs + 4) >>> 2], pb = H[(outs + 8) >>> 2], lb = H[(outs + 12) >>> 2];
  const r = JSON.parse(dec.decode(mod.HEAPU8.slice(pj, pj + lj)));
  r.blob = pb ? mod.HEAPU8.slice(pb, pb + lb) : new Uint8Array(0);
  if (pj) mod._cad_free(pj); if (pb) mod._cad_free(pb);
  mod._free(mp); mod._free(jp); mod._free(bp); mod._free(outs);
  if (!r.ok) throw new Error(`${method}: ${JSON.stringify(r.error)}`);
  return Object.assign(r.result, { blob: r.blob });
}
const out = {};
function T(key, f) {
  const t = performance.now();
  const r = f();
  out[key] = Math.round(performance.now() - t);
  return r;
}
const show = (v) => (v === undefined ? '-' : JSON.stringify(v));

const fixture = readFileSync(join(here, '..', 'orca', 'tests', 'data', 'cad_recipe_v5.bin'));
const o = T('open_fixture', () => C0('doc_open', {}, fixture));
function C0(m, p, b) { return call(m, p, b); }
console.log(`open fixture: ${out.open_fixture} ms, ok=${o.ok}, failed_feature=${show(o.failed_feature)} ${show(o.failed_feature_name)}`);
const doc = o.doc;
const names = call('list_features', { doc }).features;

// A late failure: everything but feature 29 (Project) repaired, so the replay runs to 29 and fails.
const late = T('late_failure_rebuild', () => call('suppress_feature', { doc, indices: [2, 3, 5, 6], suppressed: true, force: true }));
console.log(`late failure (force-suppress 2 3 5 6): ${out.late_failure_rebuild} ms, ok=${late.ok}, failed_feature=${show(late.failed_feature)} ${show(late.failed_feature_name)}`);
const lateBytes = call('doc_save', { doc }).blob;

// What the UI does without failed_feature: binary search over history prefixes on a scratch copy
// (each probe disables the features after `mid` and rebuilds).
if (!quick) {
  const t = performance.now();
  const sc = call('doc_open', {}, lateBytes).doc;
  const base = call('list_features', { doc: sc }).features.map((f) => f.enabled);
  let lo = 0, hi = names.length - 1, probes = 0;   // invariant: prefix [0..hi] fails
  let enabled = base.slice();
  const setPrefix = (mid) => {
    const want = base.map((e, i) => e && i <= mid);
    const off = [], on = [];
    want.forEach((w, i) => { if (w !== enabled[i]) (w ? on : off).push(i); });
    let r = null;
    if (off.length) r = call('suppress_feature', { doc: sc, indices: off, suppressed: true, force: true });
    if (on.length) r = call('suppress_feature', { doc: sc, indices: on, suppressed: false, force: true });
    enabled = want;
    probes++;
    return r ? r.ok : true;
  };
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (setPrefix(mid)) lo = mid + 1; else hi = mid;
  }
  out.binary_search = Math.round(performance.now() - t);
  console.log(`binary search for the failing feature (scratch copy, ${probes} probes): ${out.binary_search} ms → feature ${lo} ${names[lo].name}`);
  call('doc_close', { doc: sc });
}

// The repaired fixture.
const rep = T('repair_rebuild', () => call('suppress_feature', { doc, indices: [29], suppressed: true, force: true }));
console.log(`repaired fixture (29 suppressed too): ${out.repair_rebuild} ms, ok=${rep.ok}, bodies ${rep.bodies}`);

const full = T('regenerate_full', () => call('regenerate', { doc, mesh: false, full: true }));
console.log(`regenerate full replay: ${out.regenerate_full} ms (kernel ${full.ms?.toFixed(0)} ms)`);
const fast = T('regenerate_default', () => call('regenerate', { doc, mesh: false }));
console.log(`regenerate (default): ${out.regenerate_default} ms (kernel ${fast.ms?.toFixed(0)} ms, replayed_from ${show(fast.replayed_from)})`);

const fmb1 = T('faces_made_by_21_first', () => call('faces_made_by', { doc, index: 21 }));
const fmb2 = T('faces_made_by_21_again', () => call('faces_made_by', { doc, index: 21 }));
console.log(`faces_made_by 21 (${names[21].type} ${names[21].name}): ${out.faces_made_by_21_first} ms, again ${out.faces_made_by_21_again} ms, ${fmb1.faces.length} faces`);
const fmbAll = T('faces_made_by_all', () => names.map((_, i) => call('faces_made_by', { doc, index: i }).faces.length));
console.log(`faces_made_by every feature: ${out.faces_made_by_all} ms → ${JSON.stringify(fmbAll)}`);

const sv = T('set_variable_unused', () => call('set_variable', { doc, name: 'unused_var', expr: '12' }));
console.log(`set_variable (no feature uses it): ${out.set_variable_unused} ms, ok=${sv.ok}, rebuilt=${show(sv.rebuilt)}`);
const sv2 = T('set_variable_unused_delete', () => call('set_variable', { doc, name: 'unused_var', expr: '' }));
console.log(`set_variable delete (unused): ${out.set_variable_unused_delete} ms, rebuilt=${show(sv2.rebuilt)}`);

// Edits late and mid history (item 9: replay from the first changed feature).
const last = names.length - 1;
const vol = () => call('describe_scene', { doc }).bodies.map((b) => [b.face_count ?? 0, b.edge_count ?? 0]);
const before = vol();
const g0 = call('get_feature', { doc, index: last }).feature;
const e1 = T('edit_last_feature', () => call('rename_feature', { doc, index: last, name: g0.name }) && call('set_feature', { doc, index: last, fields: { mate_offset: g0.mate_offset + 1 } }));
console.log(`set_feature on feature ${last} (${g0.type} ${g0.name}): ${out.edit_last_feature} ms, ok=${e1.ok} ${e1.error}, replayed_from ${show(e1.replayed_from)}`);
const e2 = T('edit_last_feature_back', () => call('set_feature', { doc, index: last, fields: { mate_offset: g0.mate_offset } }));
console.log(`  and back: ${out.edit_last_feature_back} ms, replayed_from ${show(e2.replayed_from)}`);
const u = T('undo', () => call('undo', { doc }));
console.log(`undo: ${out.undo} ms, replayed_from ${show(u.replayed_from)}`);
const r = T('redo', () => call('redo', { doc }));
console.log(`redo: ${out.redo} ms, replayed_from ${show(r.replayed_from)}`);
const after = vol();
console.log(`  topology unchanged after edit + revert: ${JSON.stringify(before) === JSON.stringify(after)}`);
const failEdit = T('failing_edit', () => call('set_feature', { doc, index: 27, fields: { thicken_thickness: 0 } }));
console.log(`failing edit of feature 27 (${names[27].type} ${names[27].name}): ${out.failing_edit} ms, ok=${failEdit.ok} ${failEdit.error}, failed_feature=${show(failEdit.failed_feature)}`);
console.log(JSON.stringify(out));
