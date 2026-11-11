#!/usr/bin/env node
// cs_tool tests: painting sessions, paint_from_states, layer profiles, cut (+ connectors, groove).
//   node tests/cs-tool-test.mjs [build-dir]
import { readFileSync } from 'node:fs';
import zlib from 'node:zlib';
import { resolve, dirname, join } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
const here = dirname(fileURLToPath(import.meta.url));
const dir = resolve(process.argv[2] ?? join(here, '..', 'build-wasm'));
const { default: factory } = await import(pathToFileURL(join(dir, 'slicer.mjs')).href);
const mod = await factory({ wasmBinary: readFileSync(join(dir, 'slicer.wasm')), locateFile: (f) => join(dir, f), print: () => {}, printErr: () => {}, csProgress: () => {} });
let failures = 0;
const ok = (c, m) => { console.log(`${c ? 'PASS' : 'FAIL'} ${m}`); if (!c) failures++; };

// `extra` bytes go after the meshes; `args` may be a function of their offset.
function tool(op, args, meshes = [], extra = null) {
  let size = 0; const ents = meshes.map((m) => { const v = size; size += m.p.byteLength; const i = size; size += m.i.byteLength; return { vertexOffset: v, vertexCount: m.p.length / 3, indexOffset: i, triangleCount: m.i.length / 3 }; });
  const extraOffset = size; if (extra) size += extra.length;
  if (typeof args === 'function') args = args(extraOffset);
  const blob = new Uint8Array(Math.max(1, size));
  meshes.forEach((m, k) => { blob.set(new Uint8Array(m.p.buffer), ents[k].vertexOffset); blob.set(new Uint8Array(m.i.buffer), ents[k].indexOffset); });
  if (extra) blob.set(extra, extraOffset);
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
const o32 = tool('paint_open', { maxState: 32 }, [cube]);
const s32 = tool('paint_apply', { handle: o32.result?.handle, kind: 'circle', facet: 2, point: [0, 0, 10], camera: [0, 0, 100], radius: 4, state: 32, trafo: I }, []);
ok(o32.ok && s32.ok && s32.result.states.includes(32), `paint state 32 (2.5 raised colour states to 32) ${s32.error ?? ''}`);
tool('paint_close', { handle: o32.result?.handle }, []);
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

// ---- Orca 2.5: mixed filament helpers -------------------------------------------------------
const pv = tool('mixed_filament', { action: 'preview', colors: ['#FF0000', '#0000FF'], components: [1, 2], ratios: [50, 50] });
ok(pv.ok && /^#[0-9A-F]{6}$/i.test(pv.result.color) && pv.result.color.toUpperCase() !== '#FF0000', `mixed preview colour ${pv.result?.color} ${pv.error ?? ''}`);
const rc1 = tool('mixed_filament', { action: 'recommend', target: '#00A000', mode: 'cmyw' });
ok(rc1.ok && rc1.result.mode === 'cmyw' && Array.isArray(rc1.result.components), `CMYW recipe: valid=${rc1.result?.valid} ${JSON.stringify(rc1.result?.components?.map((c) => [c.baseColor, c.ratio]))} ${rc1.error ?? ''}`);
const rc2 = tool('mixed_filament', { action: 'recommend', target: '#800080', filaments: [{ color: '#FF0000', type: 'PLA' }, { color: '#0000FF', type: 'PLA' }] });
ok(rc2.ok && (!rc2.result.valid || rc2.result.components.every((c) => c.filament >= 1)), `recipe from loaded filaments: valid=${rc2.result?.valid} ${rc2.error ?? ''}`);
const va = tool('mixed_filament', { action: 'validate', config: { filament_is_mixed: ['0', '0', '1'], filament_mixed_components: ['', '', '1'], filament_mixed_sublayer_ratios: ['', '', '1'] } });
ok(va.ok && Object.keys(va.result.errors).length > 0, `validate flags a 1-component mix: ${JSON.stringify(va.result?.errors)?.slice(0, 120)}`);
const cv = tool('mixed_filament', { action: 'curve', curve: '0,0.15|0.5,0.5|1,0.85', samples: 3 });
ok(cv.ok && cv.result.points.length === 3 && Math.abs(cv.result.samples[1][1] - 0.5) < 1e-6, `gradient curve sampled ${JSON.stringify(cv.result?.samples)}`);
const ex = tool('mixed_filament', { action: 'expand', config: { filament_diameter: ['1.75', '1.75'], filament_colour: ['#FF0000', '#0000FF'], nozzle_temperature: ['200', '230'] },
  mixedFilaments: [{ components: [2, 1], ratios: [70, 30] }] });
ok(ex.ok && ex.result.slots[0].slot === 3 && ex.result.config.filament_colour?.length === 3 && ex.result.config.nozzle_temperature?.[2] === '230'
   && ex.result.config.filament_mixed_components?.[2] === '2,1', `expand → slot 3, copies filament 2's settings ${ex.error ?? ''}`);

// ---- Orca 2.5: extruder variant resolution ----------------------------------------------------
const variants = (printerVariants, nvt) => ({ nozzle_diameter: ['0.4'], extruder_type: ['Direct Drive'], nozzle_volume_type: [nvt],
  extruder_variant_list: [printerVariants.join(',')], printer_extruder_variant: printerVariants, printer_extruder_id: printerVariants.map(() => '1'),
  print_extruder_variant: ['Direct Drive Standard', 'Direct Drive High Flow'], print_extruder_id: ['1', '1'], outer_wall_speed: ['200', '500'] });
const rv1 = tool('resolve_variants', { config: variants(['Direct Drive Standard', 'Direct Drive High Flow'], 'High Flow') });
ok(rv1.ok && rv1.result.resolvedBy === 'print_apply' && rv1.result.print.outer_wall_speed?.[0] === '500', `two-variant printer, High Flow → ${rv1.result?.print?.outer_wall_speed} via ${rv1.result?.resolvedBy} ${rv1.error ?? ''}`);
const rv2 = tool('resolve_variants', { config: variants(['Direct Drive High Flow'], 'High Flow') });
ok(rv2.ok && rv2.result.resolvedBy === 'bridge' && rv2.result.print.outer_wall_speed?.[0] === '500', `single-variant High Flow printer → ${rv2.result?.print?.outer_wall_speed} via ${rv2.result?.resolvedBy} ${rv2.error ?? ''}`);

// ---- wipe tower footprint (estimate_wipe_tower_footprint + desktop placement) ---------------------
{
  const base = { printable_area: ['0x0', '256x0', '256x256', '0x256'], filament_diameter: ['1.75', '1.75'], filament_colour: ['#FF0000', '#0000FF'],
    enable_prime_tower: '1', prime_tower_width: '35', prime_volume: '45', layer_height: '0.2', initial_layer_print_height: '0.2',
    prime_tower_brim_width: '3', wipe_tower_x: ['250'], wipe_tower_y: ['250'] };
  const inside = (fp, x, y, area) => fp.every(([px, py]) => area.some((poly) => {
    const xs = poly.map((p) => p[0]), ys = poly.map((p) => p[1]);
    return px + x >= Math.min(...xs) - 1e-6 && px + x <= Math.max(...xs) + 1e-6 && py + y >= Math.min(...ys) - 1e-6 && py + y <= Math.max(...ys) + 1e-6;
  }));
  let wt = tool('wipe_tower_footprint', { config: base, filaments: [1, 2], maxHeight: 30 });
  const R = wt.result;
  ok(wt.ok && R.printed && R.width > 5 && R.depth > 5 && R.brim >= 3 && R.brim < 4 && R.footprint.length >= 4,
     `footprint: ${R?.towerType} ${R?.width?.toFixed?.(1)} x ${R?.depth?.toFixed?.(1)} mm, brim ${R?.brim}, printed ${R?.printed} ${wt.error ?? ''}`);
  ok(wt.ok && R.placement.moved && R.placement.fits && R.placement.x + R.width + R.brim <= 256 && inside(R.footprint, R.placement.x, R.placement.y, R.printableArea),
     `placement: (250,250) → (${R?.placement?.x?.toFixed?.(1)}, ${R?.placement?.y?.toFixed?.(1)}) inside the bed`);
  ok(wt.ok && R.defaultPosition.fits, `default position (${R?.defaultPosition?.x?.toFixed?.(1)}, ${R?.defaultPosition?.y?.toFixed?.(1)})`);
  // More filaments, taller → deeper tower.
  const big = tool('wipe_tower_footprint', { config: { ...base, filament_diameter: Array(4).fill('1.75'), filament_colour: ['#F00', '#0F0', '#00F', '#FFF'] }, maxHeight: 30 });
  ok(big.ok && big.result.depth > R.depth, `4 filaments: depth ${big.result?.depth?.toFixed?.(1)} > ${R?.depth?.toFixed?.(1)}`);
  // Dual extruder: the tower must sit in both extruders' areas (x 20..240).
  const dual = { ...base, nozzle_diameter: ['0.4', '0.4'], extruder_printable_area: ['0x0,240x0,240x256,0x256', '20x0,256x0,256x256,20x256'] };
  wt = tool('wipe_tower_footprint', { config: dual, filaments: [1, 2], maxHeight: 30, position: [2, 100] });
  const D = wt.result;
  const xs = D?.printableArea?.[0]?.map((p) => p[0]) ?? [];
  ok(wt.ok && Math.min(...xs) === 20 && Math.max(...xs) === 240 && D.placement.fits && D.placement.x - D.brim >= 20 && inside(D.footprint, D.placement.x, D.placement.y, D.printableArea),
     `dual extruder: shared area x ${Math.min(...xs)}..${Math.max(...xs)}, (2,100) → (${D?.placement?.x?.toFixed?.(1)}, ${D?.placement?.y?.toFixed?.(1)}) ${wt.error ?? ''}`);
  wt = tool('wipe_tower_footprint', { config: dual, filaments: [1, 2], maxHeight: 30, position: [230, 100] });
  ok(wt.ok && wt.result.placement.fits && wt.result.placement.x + wt.result.width + wt.result.brim <= 240 + 1e-6, `dual extruder: (230,100) → x ${wt.result?.placement?.x?.toFixed?.(1)} (right edge ≤ 240)`);
  const one = tool('wipe_tower_footprint', { config: base, filaments: [1], maxHeight: 30 });
  ok(one.ok && !one.result.printed, `one filament: no tower printed (depth ${one.result?.depth})`);
}

// ---- mesh repair (desktop "Fix model" / "Fix All": CGAL repair_mesh) -----------------------------
{
  const holed = { p: cube.p, i: cube.i.slice(0, 30) }; // one side face (2 triangles) missing
  const fx = tool('repair_mesh', {}, [holed]);
  const m = fx.meshData?.[0];
  ok(fx.ok && fx.result.repaired && fx.result.modified && fx.result.stats.filledHoles >= 1 && m && m.i.length / 3 >= 12,
     `repair fills a hole: ${JSON.stringify(fx.result?.stats)}, ${m ? m.i.length / 3 : 0} tris ${fx.error ?? ''}`);
  const same = tool('repair_mesh', {}, [cube]);
  ok(same.ok && same.result.repaired && !same.result.modified && same.meshData[0].i.length === cube.i.length, 'closed cube: unchanged');
  const flipped = { p: cube.p, i: Uint32Array.from(cube.i, (_, k) => cube.i[k - (k % 3) + [0, 2, 1][k % 3]]) };
  const fl = tool('repair_mesh', {}, [flipped]);
  ok(fl.ok && fl.result.modified && fl.result.stats.reversedShells === 1, `inside-out cube: ${fl.result?.stats?.reversedShells} shell reversed`);
  const colour = tool('paint_from_states', { states: [[4, 1], [5, 1]] }, [cube]).result.paint;
  const kp = tool('repair_mesh', { paint: { color: colour } }, [holed]);
  ok(kp.ok && kp.result.modified && kp.result.paint?.color?.length > 0, `painting kept through repair (${kp.result?.paint?.color?.length} painted triangles)`);
  const bad = tool('repair_mesh', {}, []);
  ok(!bad.ok && /exactly one mesh/.test(bad.error), 'repair_mesh needs one mesh');
}

// ---- inspect_3mf: texture displacement, CAD recipe, auto_drop -----------------------------------
{
  const file = new Uint8Array(readFileSync(join(here, 'fixtures', 'features-roundtrip.3mf')));
  const r3 = tool('inspect_3mf', {}, [], file);
  const vols = r3.result?.objects?.flatMap((o) => o.volumes) ?? [];
  const td = vols.find((v) => v.textureDisplacement)?.textureDisplacement;
  ok(r3.ok && r3.result.cadRecipe?.present === true && r3.result.cadRecipe.bytes === 14, `orca_cad.bin: ${JSON.stringify(r3.result?.cadRecipe)} ${r3.error ?? ''}`);
  ok(r3.ok && r3.result.objects[0].autoDrop === false && r3.result.objects[1].autoDrop === true, `auto_drop per object: ${r3.result?.objects?.map((o) => o.autoDrop)}`);
  ok(td && td.layers?.layers?.[0]?.slot === 0 && td.images[0].pathIn3mf === 'Metadata/texture_displacement/7_0.png' && td.images[0].imageBytes === 12,
     `texture displacement layer: ${JSON.stringify(td?.images)}`);
}

// ---- Orca 2.5: texture displacement bake -------------------------------------------------------
function png(w, h, f) { // 8-bit greyscale
  const crcT = Array.from({ length: 256 }, (_, n) => { let c = n; for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1; return c >>> 0; });
  const crc = (b) => { let c = 0xffffffff; for (const x of b) c = crcT[(c ^ x) & 255] ^ (c >>> 8); return (c ^ 0xffffffff) >>> 0; };
  const chunk = (type, data) => { const out = Buffer.alloc(12 + data.length); out.writeUInt32BE(data.length, 0); out.write(type, 4, 'ascii'); data.copy(out, 8); out.writeUInt32BE(crc(out.subarray(4, 8 + data.length)), 8 + data.length); return out; };
  const raw = Buffer.alloc((w + 1) * h); for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) raw[y * (w + 1) + 1 + x] = f(x, y);
  const ihdr = Buffer.alloc(13); ihdr.writeUInt32BE(w, 0); ihdr.writeUInt32BE(h, 4); ihdr[8] = 8; ihdr[9] = 0;
  return new Uint8Array(Buffer.concat([Buffer.from([137, 80, 78, 71, 13, 10, 26, 10]), chunk('IHDR', ihdr), chunk('IDAT', zlib.deflateSync(raw)), chunk('IEND', Buffer.alloc(0))]));
}
{
  const img = png(64, 64, (x, y) => ((x >> 3) + (y >> 3)) % 2 ? 255 : 0);
  const all = tool('paint_from_states', { states: Array.from({ length: 12 }, (_, t) => [t, 1]) }, [cube]).result.paint;
  const td = tool('texture_displacement', (off) => ({ layers: { version: 1, options: {}, layers: [{ slot: 0, depth_mm: 1, tiling_scale: 8 }] },
    images: { 0: { offset: off, length: img.length } }, paint: { 0: all } }), [cube], img);
  const out = td.meshData?.[0];
  let maxX = -Infinity; if (out) for (let k = 0; k < out.p.length; k += 3) maxX = Math.max(maxX, out.p[k]);
  ok(td.ok && out && out.i.length / 3 > 12 && maxX > 10.05 && maxX < 11.5, `texture displacement bake: ${out ? out.i.length / 3 : 0} tris, max X ${maxX.toFixed(2)} (cube 10) ${td.error ?? ''}`);
  const bad = tool('texture_displacement', { layers: '{not json' }, [cube]);
  ok(!bad.ok && /layers/.test(bad.error), 'texture_displacement rejects bad layers JSON');
}

// ---- Orca 2.5: Design-tab sketch solver (SolveSpace) / inference / SVG import --------------
{
  const ents = [{ type: 'line', p0: [0, 0], p1: [10, 1.5] }, { type: 'line', p0: [10.4, 1.2], p1: [10, 8] }];
  const cons = [
    { type: 'fix', ea: 0, ra: 'p0' },
    { type: 'horizontal', ea: 0, ra: 'p0', eb: 0, rb: 'p1' },
    { type: 'distance', ea: 0, ra: 'p0', eb: 0, rb: 'p1', value: 12 },
    { type: 'coincident', ea: 0, ra: 'p1', eb: 1, rb: 'p0' },
  ];
  const sv = tool('sketch', { action: 'solve', entities: ents, constraints: cons });
  const e = sv.result?.entities;
  ok(sv.ok && sv.result.ok && e && Math.abs(e[0].p1[1]) < 1e-6 && Math.abs(Math.hypot(e[0].p1[0], e[0].p1[1]) - 12) < 1e-6
     && Math.hypot(e[1].p0[0] - e[0].p1[0], e[1].p0[1] - e[0].p1[1]) < 1e-6, `sketch solve: line0 end ${JSON.stringify(e?.[0]?.p1)}, dof ${sv.result?.dof} ${sv.error ?? ''}`);
  const over = tool('sketch', { action: 'solve', entities: ents, constraints: [...cons, { type: 'distance', ea: 0, ra: 'p0', eb: 0, rb: 'p1', value: 5 }] });
  ok(over.ok && !over.result.ok && over.result.bad.length > 0, `over-constrained sketch reports bad constraints ${JSON.stringify(over.result?.bad)}`);
  // Desktop only relates connected lines: a line drawn off the end of another at ~90° gets Perpendicular.
  const inf = tool('sketch', { action: 'infer', entities: [{ type: 'line', p0: [0, 0], p1: [10, 0] }, { type: 'line', p0: [10, 0], p1: [10.2, 8] }], newEntity: 1 });
  ok(inf.ok && inf.result.constraints.some((c) => c.type === 'perpendicular' && c.ea === 0 && c.eb === 1), `inference proposes perpendicular to the joined line: ${JSON.stringify(inf.result?.constraints?.map((c) => c.type))}`);
  const ax = tool('sketch', { action: 'axis', anchor: [0, 0], tip: [10, 0.05] });
  ok(ax.ok && ax.result.type === 'horizontal', `axis inference: ${ax.result?.type}`);
  const svg = tool('sketch', { action: 'svg', svg: '<svg xmlns="http://www.w3.org/2000/svg" width="20mm" height="10mm" viewBox="0 0 20 10"><rect x="0" y="0" width="20" height="10"/></svg>' });
  ok(svg.ok && svg.result.regions.length === 1 && svg.result.regions[0][0].length >= 4, `SVG → ${svg.result?.regions?.length} sketch region(s) ${svg.error ?? ''}`);
}

console.log(failures ? `\n${failures} FAILED` : '\nALL PASSED');
process.exit(failures ? 1 : 0);
