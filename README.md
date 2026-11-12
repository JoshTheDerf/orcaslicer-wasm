# OrcaSlicer → WebAssembly (Cubby Slicer engine)

OrcaSlicer **v2.5.0**'s `libslic3r` compiled to WebAssembly for
[Cubby Slicer](https://github.com/JoshTheDerf/cubbyslicer), the browser slicer that goes with the [CubbyCAD editor](https://cubbycad.com/editor). The engine exposes the Cubby Slicer ABI v1
(`cs_version`, `cs_describe_config`, `cs_slice`, `cs_eval_condition`,
`cs_orient`, `cs_tool`, `cs_hook_query`, `cs_free`), specified in `cubby-slicer/docs/ENGINE-CONTRACT.md`.

## Layout

```
orca/                       submodule, checked out at v2.5.0 + patches/orca-wasm.patch
patches/orca-wasm.patch     everything changed in orca/ (regenerate: git -C orca diff > patches/orca-wasm.patch)
bridge/cs_bridge.cpp        ABI implementation (job → Model/Print → G-code + report, cs_tool ops)
bridge/cs_hooks.cpp         plugin pipeline hooks → Module.csHook, cs_hook_query, lifecycle events
bridge/common/              engine-neutral, bounds-checked job parsing + fast strtod (same files in preflight-wasm)
toolchain/                  bootstrap (emsdk/CMake/Ninja/m4) + dependency build, see toolchain/README.md
wasm/CMakeLists.txt         superbuild (emcmake); ORCA_SRC_DIR / CS_ENGINE_ID select the tree
wasm/cmake/                 find-modules: real deps from the toolchain prefix, stubs only for unlinked libs
wasm/wasm_shims/            sequential TBB, single-thread Boost.Thread, MD5 (headers; ST build)
wasm/stubs/                 failing stand-ins for the libslic3r sources that need unlinked libraries
                            (STEP/SVG-OCCT, Draco, Assimp, OpenCV texture painting, OpenVDB hollowing)
wasm/mt_pre.js, mt_post.js  pthreads build: thread count, csTerminateThreads, csSyncHeap
scripts/build-wasm.sh       checkout + patch + build + schema
scripts/gen-schema.mjs      writes schema.json / version.json next to the build
tests/cs-slice-test.mjs     end-to-end + robustness suite (node); 2.5: extruder variants (presets with `include`),
                            H2C nozzle rack / Hybrid (Manual, Nozzle Manual), mixed filaments, plugin hooks,
                            stats.sliceInfo (sliced-3MF plate data) and stats.mass (centre-of-mass sums)
tests/cs-tool-test.mjs      cs_tool: painting, layer profiles, cut + connectors / groove, mixed_filament,
                            resolve_variants, wipe_tower_footprint, repair_mesh, inspect_3mf,
                            texture_displacement, sketch
tests/cs-arrange-test.mjs   cs_tool arrange (Orca ArrangeJob): spacing, auto rotate, plates, locked / unprintable,
                            IDEX/IQEX zones, belt printers
tests/cs-arrange-bench.mjs  arrange timing on heavy meshes, per phase
wasm/tests/md5_selftest.cpp native check of the MD5 shim against md5sum
```

## Build

```bash
bash toolchain/bootstrap.sh                  # once: emsdk 6.0.10, CMake, Ninja, m4 (skip with a shared ../wasm-deps)
bash toolchain/build-deps.sh all             # once: Boost, GMP, MPFR, CGAL 6.2.1, Eigen, libslvs, OCCT headers, … (same flags for every archive)
bash scripts/build-wasm.sh                   # → build-wasm/slicer.{mjs,wasm,data} + schema.json + version.json
node tests/cs-slice-test.mjs build-wasm      # must print ALL PASSED
```

Multi-threaded variant (Emscripten pthreads + real oneTBB 2022.3.0):

```bash
WASM_THREADS=1 bash toolchain/build-deps.sh all-mt      # once: -pthread deps → toolchain/install-mt
WASM_THREADS=1 bash scripts/build-wasm.sh               # → build-wasm-mt/ (same outputs)
node tests/cs-slice-test.mjs build-wasm-mt              # ALL PASSED; CS_THREADS=n / CS_SYNC=1 knobs
```

Design-tab CAD kernel (separate module, `build-cad/cad.{mjs,wasm}`; contract:
cubby-slicer `docs/CAD-KERNEL-CONTRACT.md`): Orca's `libslic3r/CAD` (CadDocument,
GeometryEngine, SketchEngine, libslvs) on real OpenCASCADE 8.0.1, driven by one stateful
dispatcher `cad_call(method, json, blob)` in `bridge/cad_bridge.cpp`.

```bash
bash toolchain/build-deps.sh occt            # once: OCCT 8.0.1 static toolkits → ../wasm-deps/occt-8.0.1 (1–3 h)
bash scripts/build-cad.sh                    # → build-cad/cad.{mjs,wasm} + version.json (wasm/cad.cmake, -DORCA_WASM_CAD=ON)
node tests/cad-test.mjs build-cad            # fixtures, build-from-scratch design, STEP, golden numbers (tests/cad-golden.json); ~5 min
node tests/cad-smoke.mjs build-cad           # every other method once; ~1 min
```

OCCT is built without `OCC_CONVERT_SIGNALS` (setjmp inside `try` is miscompiled by this LLVM's
WebAssembly backend; see `build_occt` in `toolchain/build-deps.sh`), and every OCCT object is
checked with `wasm-validate`.

`ORCA_WASM_CAD=ON` cuts libslic3r down to the members the kernel links (see `wasm/cad.cmake`),
so the slicer target cannot be built from `build-cad/`; the slicer configuration with the option
OFF (default) is unchanged.

## Multi-threaded build (`build-wasm-mt/`)

* Same ABI plus `cs_slice_start(…same args…, int32_t* state)` and
  `cs_thread_count()`. `cs_slice_start` runs `cs_slice` on its own pthread
  (64 MB stack) and returns at once; `state[0]` flips to 1 (futex wake) when
  done, `state[1]` = return code. The host awaits it with `Atomics.waitAsync`
  and must not block, because Emscripten only starts a pthread requested by
  another pthread (TBB's workers spawn each other) once the main runtime
  thread is back in its event loop. Plain `cs_slice` / `cs_orient` still work when
  called on the main runtime thread, but run inside a 1-slot
  `tbb::task_arena` (serial). Otherwise Orca's TBB warm-up barrier in
  `Print::process` would deadlock.
* Threads: `csThreadCount()` = `Module.csThreads` or
  `navigator.hardwareConcurrency`, clamped to 2..16. The pthread pool
  (`-sPTHREAD_POOL_SIZE=csThreadCount()`) is created during instantiation and
  TBB is capped to the same number (`tbb::global_control`), so every TBB
  worker lands on a pre-loaded Worker.
* Stacks: slicing thread 64 MB, TBB workers 16 MB (oneTBB's Emscripten
  default is 64 KB, patched in build-deps.sh), `STACK_OVERFLOW_CHECK=2`.
* `Module.csSyncHeap()` refreshes `HEAP*` views after another thread grew
  memory; `Module.csTerminateThreads()` kills the instance's Workers (no
  teardown otherwise with `EXIT_RUNTIME=0`).
* Progress from non-main threads is proxied to the main runtime thread
  (`cs_common.hpp`); the Print status callback is mutex-guarded.
* Needs a cross-origin-isolated page (COOP `same-origin` + COEP) for
  SharedArrayBuffer. Deps: separate prefix `../wasm-deps/install-mt`
  (Boost `threading=multi` + Boost.Thread, thread-safe MPFR, oneTBB with
  its `-fexceptions` dropped so native Wasm EH is the only model).

Diagnostics (relink-only unless noted):

```bash
cmake -S wasm -B build-wasm -DORCA_WASM_NAMES=ON        # function names in wasm stack traces
cmake -S wasm -B build-wasm -DORCA_WASM_SAFE_HEAP=ON    # SAFE_HEAP + ASSERTIONS=2
cmake -S wasm -B build-wasm "-DORCA_WASM_ASSERT_DIRS=Arachne;Fill"   # recompile those dirs with assert()
BUILD_VARIANT=debug bash scripts/build-wasm.sh          # full -O1 -g2 SAFE_HEAP build in build-wasm-debug/
```

## Memory-safety invariants

* One exception model everywhere: `-fwasm-exceptions -sSUPPORT_LONGJMP=wasm`
  (deps included). No `EMULATE_FUNCTION_POINTER_CASTS`, no allocator overrides.
* Single-threaded (`build-wasm/`): no `-pthread` objects are linked
  (Boost.Atomic/Locale, which b2 forces to threading=multi, are not linked).
  The pthreads build (`build-wasm-mt/`) links ONLY `-pthread` objects from
  `install-mt`; the two prefixes are never mixed (CMake checks WASM_THREADS).
* No shim shadows a library that is linked. Real: Boost 1.84, CGAL 6.2.1,
  GMP/MPFR, Eigen 5.0.1, cereal, NLopt, libnoise, qhull, expat, libslvs
  (SolveSpace), zlib/libpng/libjpeg (Emscripten ports). OCCT 8.0.1 headers
  (no code) for the CAD sketch sources only.
* `-sSTACK_SIZE=64MB -sSTACK_OVERFLOW_CHECK=2`: a deep recursion traps instead
  of overwriting static data. `-sMAXIMUM_MEMORY=4GB` (hosts treat pointers as unsigned).
* Every job input is validated by `cs_common.hpp` before the engine sees it.

## Fixes carried in the patch (beyond build gating)

* `Arachne/SkeletalTrapezoidation.cpp`: `interpolate()` computed
  `left.toolpath_locations.size() - 1` into a 64-bit `coord_t`; with wasm32's
  32-bit `size_t` an empty vector gives 4294967295 → out-of-bounds read (the
  "memory access out of bounds" crash seen in the browser on concentric infill).
* `GCode/Thumbnails.cpp`: plain libjpeg has no RGBA input (convert), and the
  JPEG buffer was copied from the wrong pointer when `jpeg_mem_dest` reallocated.
* `GCode/GCodeProcessor.cpp`: `SET_VELOCITY_LIMIT` parsed without `std::regex`
  (milliseconds per line under WebAssembly; dominated export on Klipper printers).
* `utils.cpp`: Boost.Log single-thread sink types; Boost.Locale NFC normalisation
  (a no-op without ICU) skipped.
* Build gating only: `Format/STEP.hpp` (compiles without OCCT), `Platform.cpp`
  (Emscripten branch), `CMakeLists.txt` (pybind11 path as a subproject),
  `src/CMakeLists.txt` (no ffmpeg copy without the GUI).
* Removed from the old v2.3.2 port: stubbed CGAL Voronoi planarity check
  (always "valid" → invalid diagrams reached Arachne), hand-rolled G-code layer
  loop (now the upstream pipeline runs on a fixed sequential TBB shim),
  Boost.Log replacement, fake version header.

## Disabled / not available

OCCT: STEP/SVG-as-model import and the Design tab's B-rep kernel (CadDocument,
GeometryEngine, SketchEngine); the sketch solver/inference/import are in (cs_tool
`sketch`). OpenCV: texture → colour painting of textured OBJ/GLB imports and the
texture-displacement colour output. Assimp (FBX/GLB/DAE import), Draco, OpenVDB
(SLA hollowing), Python (plugins run in the host; their libslic3r hook points are
bridged, see ENGINE-CONTRACT.md "Plugin hooks"), networking/printer connectivity,
multithreading in the ST build (see build-wasm-mt). Thumbnails are not generated
(no renderer).

