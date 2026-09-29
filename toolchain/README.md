# Toolchain

Shared, consistently-flagged WebAssembly toolchain + dependency prefix
(the same files are vendored in orcaslicer-wasm and preflight-wasm).

```bash
bash toolchain/bootstrap.sh          # emsdk 6.0.10, CMake 3.31.8, Ninja, m4 → toolchain/
bash toolchain/build-deps.sh all     # Boost, GMP, MPFR, CGAL, Eigen, … → toolchain/install/
bash toolchain/build-deps.sh preflight-extras   # (preflight-wasm) Qhull, expat, heatshrink, nlohmann_json, nanosvg
WASM_THREADS=1 bash toolchain/build-deps.sh all-mt   # (orcaslicer-wasm, optional) pthreads prefix
```

The build scripts use `../wasm-deps/env.sh` when a shared sibling checkout
exists (one toolchain for both ports), else this directory. Override with
`WASM_DEPS_ENV=/path/to/env.sh`.
