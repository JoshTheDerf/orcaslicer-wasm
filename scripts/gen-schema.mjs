#!/usr/bin/env node
// Generate schema.json + version.json next to the built engine by calling
// cs_describe_config / cs_version in Node.
//   node scripts/gen-schema.mjs [build-dir]
import { readFileSync, writeFileSync } from 'node:fs';
import { resolve } from 'node:path';
import { pathToFileURL } from 'node:url';

const dir = resolve(process.argv[2] ?? 'build-wasm');
const { default: factory } = await import(pathToFileURL(resolve(dir, 'slicer.mjs')).href);
const mod = await factory({
  wasmBinary: readFileSync(resolve(dir, 'slicer.wasm')),
  locateFile: (f) => resolve(dir, f),
  print: () => {}, printErr: () => {},
});
const version = JSON.parse(mod.UTF8ToString(mod._cs_version()));
const outs = mod._malloc(8) >>> 0;
mod.HEAPU32[outs >>> 2] = 0; mod.HEAPU32[(outs >>> 2) + 1] = 0;
if (mod._cs_describe_config(outs, outs + 4) !== 0) throw new Error('cs_describe_config failed');
const p = mod.HEAPU32[outs >>> 2] >>> 0, n = mod.HEAPU32[(outs >>> 2) + 1] >>> 0;
const schema = JSON.parse(new TextDecoder().decode(mod.HEAPU8.subarray(p, p + n)));
mod._cs_free(p);
writeFileSync(resolve(dir, 'schema.json'), JSON.stringify(schema));
writeFileSync(resolve(dir, 'version.json'), JSON.stringify(version));
const types = {};
for (const o of Object.values(schema.options)) types[o.type] = (types[o.type] ?? 0) + 1;
console.log(`engine ${version.engine} ${version.version} (${version.build}); ${Object.keys(schema.options).length} options`, types);
