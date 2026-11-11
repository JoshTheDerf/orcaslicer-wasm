# Architecture

How OrcaSlicer v2.5.0's `libslic3r` becomes the Cubby Slicer engine
(`slicer.mjs` + `slicer.wasm`). The ABI, job format and build invariants are
specified in `cubby-slicer/docs/ENGINE-CONTRACT.md`; this file explains where
each piece lives.

## Layers

```
host (Cubby Slicer engine Worker)
  │  cs_* exports, Module.csProgress / Module.csHook imports
  ▼
bridge/cs_bridge.cpp      job JSON + mesh blob → Model + DynamicPrintConfig → Print::process →
                          export_gcode → G-code + report; cs_describe_config; cs_tool ops
bridge/cs_hooks.cpp       slicing-pipeline plugin hooks (→ Module.csHook), cs_hook_query,
                          lifecycle events (→ report.events)
bridge/common/            engine-neutral job validation (shared with preflight-wasm)
  ▼
orca/src/libslic3r        upstream, plus the small patches/orca-wasm.patch
  ▼
toolchain prefix          Boost, CGAL, GMP/MPFR, Eigen, cereal, NLopt, libnoise, qhull,
                          expat, libslvs (+ oneTBB for the MT build), OCCT headers
```

## Build

`wasm/CMakeLists.txt` is a superbuild: it `add_subdirectory()`s the upstream
tree with the GUI, CAD kernel, PCH and warnings-as-errors off, and makes the
desktop dependencies resolvable:

* `wasm/cmake/Find*.cmake` — real libraries from the toolchain prefix (Boost,
  CGAL, GMP, MPFR, NLopt, libnoise, SLVS, oneTBB in MT) or Emscripten ports
  (zlib, libpng, libjpeg); empty "found" packages for libraries that are not
  linked (OCCT, OpenCV, Draco, Assimp, OpenVDB, Python, CURL, OpenSSL, OpenGL,
  glfw).
* `wasm/stubs/*.cpp` — the libslic3r sources that need one of those unlinked
  libraries are removed from the `libslic3r` target and replaced by failing
  stand-ins with the same symbols (STEP/SVG import, Draco, Assimp, texture →
  colour painting, SLA hollowing), so the rest compiles unpatched.
* `wasm/wasm_shims/` — header shims for the single-threaded build only:
  sequential `tbb::`, single-thread `boost::thread`, and an MD5 for
  `<openssl/md5.h>`. None shadows a linked library.
* libslic3r/CAD: the OCCT-free sketch sources (SolveSpace solver, inference,
  text/SVG import) are added back; they compile against OCCT's headers only.

Two variants share the sources: `build-wasm/` (single-threaded) and
`build-wasm-mt/` (Emscripten pthreads + oneTBB); `scripts/build-wasm.sh`
builds either and writes `schema.json` / `version.json` (`scripts/gen-schema.mjs`).

## Upstream patch

`patches/orca-wasm.patch` is the whole diff of `orca/` against the pinned tag
(`git -C orca diff > patches/orca-wasm.patch`). It holds runtime fixes that
upstream would want (Arachne wasm32 overflow, JPEG thumbnail buffer,
regex-free Klipper parsing) and the few build-gating edits that can't be done
from outside (see README "Fixes carried in the patch").
