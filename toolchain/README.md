# Toolchain

Shared, consistently-flagged WebAssembly toolchain + dependency prefix
(the same files are vendored in orcaslicer-wasm and preflight-wasm).

```bash
bash toolchain/bootstrap.sh          # emsdk 6.0.10, CMake 3.31.8, Ninja, m4 → toolchain/
bash toolchain/build-deps.sh all     # Boost, GMP, MPFR, CGAL, Eigen, libslvs, OCCT headers, … → toolchain/install/
bash toolchain/build-deps.sh preflight-extras   # (preflight-wasm) Qhull, expat, heatshrink, nlohmann_json, nanosvg
WASM_THREADS=1 bash toolchain/build-deps.sh all-mt   # (orcaslicer-wasm, optional) pthreads prefix
bash toolchain/build-deps.sh texture  # (orcaslicer-wasm texture module) Assimp 5.4.3 + OpenCV 4.14.0 → assimp-5.4.3/, opencv-4.14.0/
                                      #   (WASM_THREADS=1: -mt prefixes; nothing links those today)
```

The build scripts use `../wasm-deps/env.sh` when a shared sibling checkout
exists (one toolchain for both ports), else this directory. Override with
`WASM_DEPS_ENV=/path/to/env.sh`.
