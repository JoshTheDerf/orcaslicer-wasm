# OrcaSlicer → WebAssembly (Cubby Slicer engine)

OrcaSlicer **v2.4.2**'s `libslic3r` compiled to WebAssembly for
[Cubby Slicer](https://github.com/JoshTheDerf/cubby-slicer). The engine exposes the Cubby Slicer ABI v1
(`cs_version`, `cs_describe_config`, `cs_slice`, `cs_eval_condition`,
`cs_orient`, `cs_tool`, `cs_free`), specified in `cubby-slicer/docs/ENGINE-CONTRACT.md`.

The same bridge and CMake also build **[Full Spectrum](#full-spectrum)**, the
Snapmaker Orca fork with mixed filaments.

## Layout

```
orca/                       submodule, checked out at v2.4.2 + patches/orca-wasm.patch
patches/orca-wasm.patch     everything changed in orca/ (regenerate: git -C orca diff > patches/orca-wasm.patch)
bridge/cs_bridge.cpp        ABI implementation (job → Model/Print → G-code + report)
bridge/common/              engine-neutral, bounds-checked job parsing + fast strtod (same files in preflight-wasm)
toolchain/                  bootstrap (emsdk/CMake/Ninja/m4) + dependency build, see toolchain/README.md
wasm/CMakeLists.txt         superbuild (emcmake); ORCA_SRC_DIR / CS_ENGINE_ID select the tree
wasm/cmake/                 find-modules: real deps from the toolchain prefix, stubs only for unlinked libs
wasm/wasm_shims/            sequential TBB, single-thread Boost.Thread, MD5, OpenVDB stub (ST build)
wasm/mt_pre.js, mt_post.js  pthreads build: thread count, csTerminateThreads, csSyncHeap
scripts/build-wasm.sh       checkout + patch + build + schema
scripts/build-fullspectrum.sh  the same for the FullSpectrum fork (fullspectrum/, patches/fullspectrum-wasm.patch)
scripts/gen-schema.mjs      writes schema.json / version.json next to the build
tests/cs-slice-test.mjs     end-to-end + robustness suite (node)
tests/cs-tool-test.mjs      cs_tool: painting, layer profiles, cut + connectors / groove
tests/cs-fs-test.mjs        FullSpectrum: U1 profile, mixed filaments, gradients, patterns
wasm/tests/md5_selftest.cpp native check of the MD5 shim against md5sum
```

## Build

```bash
bash toolchain/bootstrap.sh                  # once: emsdk 6.0.10, CMake, Ninja, m4 (skip with a shared ../wasm-deps)
bash toolchain/build-deps.sh all             # once: Boost, GMP, MPFR, CGAL, Eigen, … (same flags for every archive)
bash scripts/build-wasm.sh                   # → build-wasm/slicer.{mjs,wasm,data} + schema.json + version.json
node tests/cs-slice-test.mjs build-wasm      # must print ALL PASSED
```

Multi-threaded variant (Emscripten pthreads + real oneTBB 2022.3.0):

```bash
WASM_THREADS=1 bash toolchain/build-deps.sh all-mt      # once: -pthread deps → toolchain/install-mt
WASM_THREADS=1 bash scripts/build-wasm.sh               # → build-wasm-mt/ (same outputs)
node tests/cs-slice-test.mjs build-wasm-mt              # ALL PASSED; CS_THREADS=n / CS_SYNC=1 knobs
```

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
* No shim shadows a library that is linked. Real: Boost 1.84, CGAL 5.6.3,
  GMP/MPFR, Eigen 5.0.1, cereal, NLopt, libnoise, qhull, expat, zlib/libpng/
  libjpeg/freetype (Emscripten ports).
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
* `utils.cpp`: Boost.Log single-thread sink type; Boost.Locale NFC normalisation
  (a no-op without ICU) skipped.
* Removed from the old v2.3.2 port: stubbed CGAL Voronoi planarity check
  (always "valid" → invalid diagrams reached Arachne), hand-rolled G-code layer
  loop (now the upstream pipeline runs on a fixed sequential TBB shim),
  Boost.Log replacement, fake version header.

## Disabled / not available

OCCT (STEP import), OpenVDB (SLA hollowing), OpenCV, Draco, networking/printer
connectivity, multithreading in the ST build (see build-wasm-mt). Thumbnails are not generated (no renderer).

## Full Spectrum

[Full Spectrum](https://github.com/ratdoux/OrcaSlicer-FullSpectrum) is a fork of Snapmaker Orca (2.3.6) that adds mixed filaments. It makes new colours by alternating very thin layers of the physical filaments, and supports gradients along Z. It shares enough of libslic3r with OrcaSlicer that the same bridge and CMake build it, with `ORCA_SRC_DIR` pointed at the fork:

```bash
bash scripts/build-fullspectrum.sh    # clones v0.9.14 into fullspectrum/, patches it, builds build-fs/
node tests/cs-fs-test.mjs build-fs
```

`patches/fullspectrum-wasm.patch` is `orca-wasm.patch` rebased onto the fork, plus a few fork-specific changes:

- MQTT and the Windows installer bits are skipped in headless builds.
- OpenCV and Assimp are optional. Without them the photo calibration reader and the extra mesh formats are stubbed out.
- libigl still used `Eigen::DynamicSparseMatrix`, which current Eigen removed. It builds from triplets now.
- `ConfigOptionEnumsGeneric` initialised `keys_map` from itself (undefined behaviour, which crashed `cs_describe_config`). Fixed the same way Orca 2.4 fixed it.
- A couple of missing includes the desktop build got from its precompiled header.

The fork's libslic3r API is a bit older, so the bridge checks for it (`CS_ORCA_LEGACY_API`). It also adds a `cs_tool` op, `mixed_filaments`, that lists and edits virtual filaments through the fork's own `MixedFilamentManager` (blends, gradients, patterns, Local-Z caps and surface bias).
