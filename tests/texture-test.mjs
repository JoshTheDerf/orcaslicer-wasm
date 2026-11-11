#!/usr/bin/env node
// Texture import module (texture.mjs): load → analyze → match (auto-merge, CMYW / RYBW auto-mix) →
// add_mixed → apply, on small real files:
//   orca/tests/data/texture_import/two_color_png.glb, two_color_jpeg.glb   Orca's own fixtures (GLB, embedded PNG / JPEG)
//   orca/tests/data/texture_import/two_color_sphere/*.obj                  Orca's own (MTL Kd colours; map_Kd JPEG)
//   tests/fixtures/texture/vertex_color_cube.obj                          written for this suite (OBJ vertex colours)
//   tests/fixtures/texture/maxPbrMaterial_metalRough.fbx, box_orphant_embedded_texture.fbx, phong_cube.fbx
//                                                                         Assimp 5.4.3's test models (BSD-3; the first
//                                                                         references Textures\albedo.png, generated here)
// The resulting paint strings are checked against the slicer engine's own TriangleSelector (cs_tool
// paint_open) when a slicer build is next to the module.
//   node tests/texture-test.mjs [build-dir]
import { readFileSync, existsSync } from 'node:fs';
import { resolve, dirname, join, basename } from 'node:path';
import zlib from 'node:zlib';
import { fileURLToPath, pathToFileURL } from 'node:url';
const here = dirname(fileURLToPath(import.meta.url));
const dir = resolve(process.argv[2] ?? join(here, '..', 'build-wasm'));
const { default: factory } = await import(pathToFileURL(join(dir, 'texture.mjs')).href);
const progress = [];
const mod = await factory({ wasmBinary: readFileSync(join(dir, 'texture.wasm')), locateFile: (f) => join(dir, f), print: () => {}, printErr: () => {},
  csProgress: (p, m) => progress.push([p, m]) });
let failures = 0;
const ok = (c, m) => { console.log(`${c ? 'PASS' : 'FAIL'} ${m}`); if (!c) failures++; };
const orcaData = join(here, '..', 'orca', 'tests', 'data', 'texture_import');
const fx = join(here, 'fixtures', 'texture');

function call(op, args = {}, files = []) {
  // files: [{ name, bytes }] → args.files entries pointing into the blob
  let size = 0;
  const ents = files.map((f) => { const e = { name: f.name, offset: size, length: f.bytes.length }; size += (f.bytes.length + 3) & ~3; return e; });
  if (files.length) args = { ...args, files: ents };
  const blob = new Uint8Array(Math.max(1, size));
  files.forEach((f, k) => blob.set(f.bytes, ents[k].offset));
  const json = new TextEncoder().encode(JSON.stringify({ op, args }));
  const w = (b) => { const p = mod._malloc(b.byteLength) >>> 0; mod.HEAPU8.set(b, p); return p; };
  const jp = w(json), bp = w(blob), outs = mod._malloc(16) >>> 0; mod.HEAPU8.fill(0, outs, outs + 16);
  mod._tex_call(jp, json.byteLength, bp, blob.byteLength, outs, outs + 4, outs + 8, outs + 12);
  const H = new Uint32Array(mod.HEAPU8.buffer, outs, 4); const [pj, lj, pb, lb] = H;
  const r = JSON.parse(new TextDecoder().decode(mod.HEAPU8.slice(pj, pj + lj)));
  const bytes = pb ? mod.HEAPU8.slice(pb, pb + lb) : new Uint8Array(0);
  r.blobData = Object.fromEntries(Object.entries(r.blobs ?? {}).map(([k, b]) => [k, bytes.slice(b.offset, b.offset + b.length)]));
  r.meshData = (r.meshes ?? []).map((m) => ({ p: new Float32Array(bytes.buffer.slice(m.vertexOffset, m.vertexOffset + m.vertexCount * 12)), i: new Uint32Array(bytes.buffer.slice(m.indexOffset, m.indexOffset + m.triangleCount * 12)) }));
  if (pj) mod._tex_free(pj); if (pb) mod._tex_free(pb); mod._free(jp); mod._free(bp); mod._free(outs);
  return r;
}
const file = (p, name = basename(p)) => ({ name, bytes: new Uint8Array(readFileSync(p)) });

// Slicer engine (optional): validates the paint strings the way a slice job reads them.
let slicer = null;
if (existsSync(join(dir, 'slicer.mjs'))) {
  const { default: sf } = await import(pathToFileURL(join(dir, 'slicer.mjs')).href);
  slicer = await sf({ wasmBinary: readFileSync(join(dir, 'slicer.wasm')), locateFile: (f) => join(dir, f), print: () => {}, printErr: () => {}, csProgress: () => {} });
}
function paintStates(mesh, paint) {
  if (!slicer) return null;
  const blob = new Uint8Array(mesh.p.byteLength + mesh.i.byteLength); blob.set(new Uint8Array(mesh.p.buffer)); blob.set(new Uint8Array(mesh.i.buffer), mesh.p.byteLength);
  const req = (op, args, meshes) => {
    const json = new TextEncoder().encode(JSON.stringify({ op, args, meshes }));
    const w = (b) => { const p = slicer._malloc(b.byteLength) >>> 0; slicer.HEAPU8.set(b, p); return p; };
    const jp = w(json), bp = w(blob), outs = slicer._malloc(16) >>> 0; slicer.HEAPU8.fill(0, outs, outs + 16);
    slicer._cs_tool(jp, json.byteLength, bp, blob.byteLength, outs, outs + 4, outs + 8, outs + 12);
    const [pj, lj, pb] = new Uint32Array(slicer.HEAPU8.buffer, outs, 4);
    const r = JSON.parse(new TextDecoder().decode(slicer.HEAPU8.slice(pj, pj + lj)));
    if (pj) slicer._cs_free(pj); if (pb) slicer._cs_free(pb); slicer._free(jp); slicer._free(bp); slicer._free(outs);
    return r;
  };
  const meshes = [{ vertexOffset: 0, vertexCount: mesh.p.length / 3, indexOffset: mesh.p.byteLength, triangleCount: mesh.i.length / 3 }];
  const o = req('paint_open', { paint, maxState: 32 }, meshes);
  if (o.ok) req('paint_close', { handle: o.result.handle }, []);
  return o.ok ? o.result.states : null;
}

const version = JSON.parse(mod.UTF8ToString(mod._tex_version()));
ok(version.module === 'orca-texture' && version.abi === 1 && version.assimp === '5.4.3' && version.opencv === '4.14.0', `version ${JSON.stringify(version)}`);

const red = (h) => { const n = parseInt(h.slice(1), 16); return ((n >> 16) & 255) > (n & 255); };
const blue = (h) => { const n = parseInt(h.slice(1), 16); return (n & 255) > ((n >> 16) & 255); };
const project = [{ color: '#FF0000', name: 'Red PLA', type: 'PLA' }, { color: '#0000FF', name: 'Blue PLA', type: 'PLA' }];

// The Orca core test's check (test_texture_import.cpp), through the module's ops, then the dialog's
// matching and the plater's apply.
function twoColour(label, files) {
  const ld = call('load', {}, files);
  ok(ld.ok && ld.result.importable && ld.meshData[0]?.i.length > 0, `${label}: load (${ld.result?.format}, ${ld.result?.source}, ${ld.result?.objectTriangles} object tris) ${ld.error ?? ''}`);
  if (!ld.ok) return null;
  const h = ld.result.handle;
  const oc = ld.blobData.originalColors;
  ok(oc && oc.length === (ld.meshData[1]?.i.length ?? 0), `${label}: original colours, one RGB per textured triangle (${oc?.length / 3})`);
  progress.length = 0;
  const an = call('analyze', { handle: h, colorCount: 0, smooth: 5, repair: 'none' });
  const cl = an.result?.clusters ?? [];
  ok(an.ok && cl.some(red) && cl.some(blue) && an.blobData.faceCluster?.length === an.meshData[0]?.i.length / 3,
     `${label}: analyze (auto colour count) → clusters ${cl.join(' ')}, ${an.result?.triangles} painted tris ${an.error ?? ''}`);
  // Desktop's auto-merge: a cluster within ΔE00 5 of a project filament takes it, else a new filament.
  const own = cl.map((c, k) => ({ color: c, name: `F${k + 1}`, type: 'PLA' })).reverse();
  const mt = call('match', { handle: h, filaments: own });
  const fils = (mt.result?.matches ?? []).map((m) => m.filament);
  ok(mt.ok && mt.result.entries.length === cl.length && mt.result.matches.every((m) => own[m.filament]?.color === m.color && m.deltaE < 1e-6),
     `${label}: match against filaments of the same colours → existing ${fils}, nothing new`);
  const pr = call('match', { handle: h, filaments: project });
  const tgt = (m) => pr.result.entries[m.filament]?.color ?? '#808080';
  ok(pr.ok && pr.result.matches.every((m) => (red(m.color) ? red(tgt(m)) : blue(tgt(m)))),
     `${label}: against pure red / blue: ${pr.result?.matches?.map((m) => `${m.color}→${tgt(m)}${m.filament >= 2 ? '(new)' : ''}`).join(' ')}`);
  const ap = call('apply', { handle: h, entries: mt.result?.entries, matches: mt.result?.matches });
  const states = ap.ok ? paintStates(ap.meshData[0], ap.result.paint.color) : null;
  ok(ap.ok && ap.result.newFilaments.length === 0 && ap.result.extruder === 1 && ap.result.paint.color.length > 0 &&
     ap.blobData.faceFilament?.length === ap.meshData[0].i.length / 3 && (states === null || states.join() === '1,2'),
     `${label}: apply → ${ap.result?.paint?.color?.length} paint strings, extruder ${ap.result?.extruder}, engine reads states ${states} ${ap.error ?? ''}`);
  if (ld.result.hasModelUv) ok(ap.blobData.modelUv?.length === ap.meshData[0].i.length * 8, `${label}: model UV channel resampled onto the painted mesh`);
  return h;
}
const hPng = twoColour('PNG GLB', [file(join(orcaData, 'two_color_png.glb'))]);
ok(progress.length > 0, `progress reported through Module.csProgress (${progress.length} calls)`);
twoColour('JPEG GLB', [file(join(orcaData, 'two_color_jpeg.glb'))]);
twoColour('OBJ + MTL colours', [file(join(orcaData, 'two_color_sphere', 'test_two_color_sphere.obj')), file(join(orcaData, 'two_color_sphere', 'test_two_color_sphere.mtl'))]);
const hTexObj = twoColour('OBJ + MTL + external JPEG', [file(join(orcaData, 'two_color_sphere', 'test_two_color_sphere_textured.obj')),
  file(join(orcaData, 'two_color_sphere', 'test_two_color_sphere_textured.mtl')), file(join(orcaData, 'two_color_sphere', 'textures', 'two_color.jpg'), 'textures/two_color.jpg')]);
twoColour('OBJ vertex colours', [file(join(fx, 'vertex_color_cube.obj'))]);

// Textured preview data: decoded pixels + per-face UVs / texture ids.
{
  const ld = call('load', {}, [file(join(orcaData, 'two_color_png.glb'))]);
  const t = ld.result?.textures?.[0];
  const px = t && ld.blobData[t.pixels];
  const nf = ld.meshData[1]?.i.length / 3;
  ok(ld.ok && t.width === 8 && t.height === 8 && px?.length === 8 * 8 * 4 && ld.blobData.uv?.length === nf * 24 && ld.blobData.faceTexture?.length === nf * 4,
     `textured preview: ${t?.width}×${t?.height} RGBA, uv + faceTexture for ${nf} faces`);
  ok(ld.ok && ld.result.materials.length >= 1 && ld.result.materials.some((m) => m.texture === 0), `materials: ${JSON.stringify(ld.result?.materials)}`);
  const noPx = call('load', { texturePixels: false }, [file(join(orcaData, 'two_color_png.glb'))]);
  ok(noPx.ok && !noPx.blobData.texture0 && noPx.result.textures[0].width === 8, 'texturePixels:false skips the pixel blobs');
  call('close', { handle: ld.result?.handle }); call('close', { handle: noPx.result?.handle });
}

// Without its external texture, the textured OBJ falls back to the white Kd colours (nothing to import).
{
  const ld = call('load', {}, [file(join(orcaData, 'two_color_sphere', 'test_two_color_sphere_textured.obj')),
    file(join(orcaData, 'two_color_sphere', 'test_two_color_sphere_textured.mtl'))]);
  const an = ld.ok && ld.result.importable ? call('analyze', { handle: ld.result.handle, repair: 'none' }) : null;
  ok(ld.ok && ld.result.hasModelUv && ld.blobData.modelUv?.length === ld.result.objectTriangles * 24 && (!an || !an.ok || an.result.clusters.length === 1),
     `OBJ without its texture: loads, model UV kept, ${an ? (an.ok ? an.result.clusters : an.error) : 'not importable'}`);
  if (ld.ok) call('close', { handle: ld.result.handle });
}

// FBX (Assimp's test models). An external PNG texture (generated: left half red, right half blue);
// an embedded TGA (OpenCV, like desktop's, has no TGA decoder: the analysis fails and desktop imports the
// geometry only); material colours only (not importable: geometry only, no dialog).
function pngRGB(w, h, f) {
  const crcT = Array.from({ length: 256 }, (_, n) => { let c = n; for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1; return c >>> 0; });
  const crc = (b) => { let c = 0xffffffff; for (const x of b) c = crcT[(c ^ x) & 255] ^ (c >>> 8); return (c ^ 0xffffffff) >>> 0; };
  const chunk = (type, data) => { const out = Buffer.alloc(12 + data.length); out.writeUInt32BE(data.length, 0); out.write(type, 4, 'ascii'); data.copy(out, 8); out.writeUInt32BE(crc(out.subarray(4, 8 + data.length)), 8 + data.length); return out; };
  const raw = Buffer.alloc((w * 3 + 1) * h); for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) raw.set(f(x, y), y * (w * 3 + 1) + 1 + x * 3);
  const ihdr = Buffer.alloc(13); ihdr.writeUInt32BE(w, 0); ihdr.writeUInt32BE(h, 4); ihdr[8] = 8; ihdr[9] = 2;
  return new Uint8Array(Buffer.concat([Buffer.from([137, 80, 78, 71, 13, 10, 26, 10]), chunk('IHDR', ihdr), chunk('IDAT', zlib.deflateSync(raw)), chunk('IEND', Buffer.alloc(0))]));
}
{
  const albedo = pngRGB(16, 16, (x) => (x < 8 ? [220, 20, 20] : [20, 20, 220]));
  const fb = call('load', {}, [file(join(fx, 'maxPbrMaterial_metalRough.fbx')), { name: 'Textures/albedo.png', bytes: albedo }]);
  ok(fb.ok && fb.result.format === 'fbx' && fb.result.importable && fb.result.textures?.[0]?.width === 16,
     `FBX + external PNG: ${fb.result?.objectTriangles} tris, textures ${JSON.stringify(fb.result?.textures)} ${fb.error ?? ''}`);
  const an = fb.ok ? call('analyze', { handle: fb.result.handle, colorCount: 2, repair: 'none' }) : null;
  ok(an?.ok && an.result.clusters.some(red) && an.result.clusters.some(blue), `FBX analyze → ${an?.result?.clusters ?? an?.error}`);
  if (fb.ok) call('close', { handle: fb.result.handle });
  const tga = call('load', {}, [file(join(fx, 'box_orphant_embedded_texture.fbx'))]);
  const tan = tga.ok ? call('analyze', { handle: tga.result.handle, repair: 'none' }) : null;
  ok(tga.ok && tga.meshData[0]?.i.length / 3 === 12 && tga.result.textures[0].width === 0 && tan && !tan.ok && /Computation failed/.test(tan.error),
     `FBX embedded TGA: loads (12 tris), texture undecodable, analyze fails like desktop (${tan?.error})`);
  const ph = call('load', {}, [file(join(fx, 'phong_cube.fbx'))]);
  ok(ph.ok && ph.meshData[0]?.i.length / 3 === 12 && ph.result.importable === false && ph.result.materials.length >= 1,
     `FBX phong cube: geometry only, materials ${JSON.stringify(ph.result?.materials)} ${ph.error ?? ''}`);
  if (tga.ok) call('close', { handle: tga.result.handle }); if (ph.ok) call('close', { handle: ph.result.handle });
}

// Dialog behaviour: auto-merge off adds a filament per colour; colour count; filaments far from the
// clusters become new filaments (ΔE > 5); CMYW / RYBW auto-mix builds mixed filaments.
if (hPng) {
  const two = call('analyze', { handle: hPng, colorCount: 2, smooth: 0, repair: 'none' });
  ok(two.ok && two.result.colorCount === 2, `colorCount 2 → ${two.result?.colorCount} clusters`);
  const nm = call('match', { handle: hPng, filaments: project, autoMerge: false, defaultPresetName: 'Bambu PLA Basic @BBL X1C' });
  const kinds = nm.result?.entries?.map((e) => e.kind) ?? [];
  ok(nm.ok && kinds.filter((k) => k === 'new').length === 2 && nm.result.matches.every((m) => m.filament >= 2),
     `auto-merge off: one new filament per colour (${kinds}) preset ${nm.result?.entries?.[2]?.presetName}`);
  const far = call('match', { handle: hPng, filaments: [{ color: '#00FF00', name: 'Green' }] });
  ok(far.ok && far.result.entries.length === 3 && far.result.entries.slice(1).every((e) => e.kind === 'new' && e.name === 'Bambu PLA Basic'),
     `ΔE > 5 from every filament → new filaments (${far.result?.entries?.map((e) => e.color)})`);
  for (const mode of ['cmyw', 'rybw']) {
    const mx = call('match', { handle: hPng, filaments: project, autoMix: mode });
    const mixed = mx.result?.entries?.filter((e) => e.kind === 'newMixed') ?? [];
    ok(mx.ok && mx.result.autoMixApplied && mixed.length > 0 && mixed.every((e) => e.components.length >= 2 && e.ratios.reduce((a, b) => a + b, 0) > 0) &&
       mx.result.matches.every((m) => mx.result.entries[m.filament]?.kind === 'newMixed'),
       `auto-mix ${mode}: ${mixed.length} mixed (${mixed.map((e) => `${e.color}=${e.components}:${e.ratios}`).join(' ')}), bases ${mx.result?.entries?.filter((e) => e.kind === 'new').map((e) => e.color)}`);
    const ap = call('apply', { handle: hPng, entries: mx.result?.entries, matches: mx.result?.matches });
    const np = ap.result?.newFilaments ?? [], nmx = ap.result?.newMixedFilaments ?? [];
    ok(ap.ok && nmx.length === mixed.length && np.every((f, k) => f.index === 2 + k) && nmx.every((m, k) => m.index === 2 + np.length + k) &&
       nmx.every((m) => m.components.every((c) => c >= 1 && c <= 2 + np.length)) && ap.result.matches.every((m) => m.filament >= 2 + np.length),
       `apply ${mode}: new physical ${np.map((f) => f.index + ':' + f.color)}, new mixed ${nmx.map((m) => m.index + ':' + m.components)} ${ap.error ?? ''}`);
  }
  // Existing mixed slots move up by the number of new physical filaments (physical first, mixed at the tail).
  const withMix = [...project, { color: '#800080', mixed: true, components: '1,2', ratios: '0.5000,0.5000' }];
  const em = call('match', { handle: hPng, filaments: withMix, autoMerge: false });
  const emApply = call('apply', { handle: hPng, entries: em.result?.entries, matches: em.result?.matches });
  ok(emApply.ok && emApply.result.remap[2] === 2 + emApply.result.newFilaments.length && emApply.result.newFilaments.every((f, k) => f.index === 2 + k),
     `existing mixed slot remapped to ${emApply.result?.remap?.[2]} after ${emApply.result?.newFilaments?.length} new physical ${emApply.error ?? ''}`);
  // add_mixed (the row's "decompose into a mix" from a recipe): existing red + a new white.
  const ownCl = two.result?.clusters ?? [];
  const base = call('match', { handle: hPng, filaments: [{ color: red(ownCl[0]) ? ownCl[0] : ownCl[1] }, { color: red(ownCl[0]) ? ownCl[1] : ownCl[0] }] });
  const am = call('add_mixed', { entries: base.result?.entries, matches: base.result?.matches, row: 0,
    components: [{ filament: 0 }, { color: '#FFFFFF' }], ratios: [60, 40], color: '#FF6666' });
  const row0 = base.result?.matches?.[0]?.filament;
  const e = am.result?.entries ?? [];
  ok(am.ok && e.length === 4 && e[2].kind === 'new' && e[2].color === '#FFFFFF' && e[3].kind === 'newMixed' && e[3].components.join() === '1,3' &&
     am.result.matches[0].filament === 3 && row0 !== 3, `add_mixed: ${e.map((x) => x.kind + x.color).join(' ')} ${am.error ?? ''}`);
  // Unused virtual filaments are pruned on apply (compact_used_virtual_filaments).
  const pruned = call('apply', { handle: hPng, entries: am.result?.entries, matches: base.result?.matches });
  ok(pruned.ok && pruned.result.newFilaments.length === 0 && pruned.result.newMixedFilaments.length === 0, 'apply prunes filaments no colour uses');
  const bad = call('apply', { handle: hPng, entries: base.result?.entries, matches: [{ cluster: 0, color: '#FF0000', filament: 9 }] });
  ok(!bad.ok && /out of range/.test(bad.error), `apply rejects a bad filament index: ${bad.error}`);
  call('close', { handle: hPng });
  const gone = call('analyze', { handle: hPng });
  ok(!gone.ok && /unknown model handle/.test(gone.error), 'closed handle');
}
if (hTexObj) call('close', { handle: hTexObj });

// Robustness: bad input never traps.
{
  const r1 = call('load', {}, [{ name: 'x.glb', bytes: new Uint8Array([1, 2, 3, 4]) }]);
  ok(!r1.ok && typeof r1.error === 'string', `garbage GLB: ${r1.error}`);
  const r2 = call('load', {}, [{ name: '../x.obj', bytes: new Uint8Array([1]) }]);
  ok(!r2.ok && /bad file name/.test(r2.error), 'path traversal rejected');
  const r3 = call('load', {}, [{ name: 'x.stl', bytes: new Uint8Array([1]) }]);
  ok(!r3.ok && /unsupported/.test(r3.error), 'unsupported extension');
  const r4 = call('nope');
  ok(!r4.ok && /unknown op/.test(r4.error), 'unknown op');
  const r5 = call('match', { clusters: ['#FF0000'], filaments: [] });
  ok(!r5.ok && /filaments/.test(r5.error), 'match without filaments');
}

console.log(failures ? `${failures} FAILED` : 'ALL PASSED');
process.exit(failures ? 1 : 0);
