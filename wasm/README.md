# wasm/

The CMake superbuild that turns `../orca` into the engine module. See the
repository README for the build commands and ARCHITECTURE.md for the layout.

| path | what |
|---|---|
| `CMakeLists.txt` | superbuild: options for the upstream tree, libslic3r source substitutions, link flags, exports |
| `cmake/Find*.cmake` | real dependencies from the toolchain prefix, or empty packages for libraries that are not linked |
| `stubs/` | failing stand-ins for libslic3r sources that need an unlinked library |
| `wasm_shims/` | single-threaded header shims: sequential TBB, `boost::thread`, MD5 |
| `mt_pre.js`, `mt_post.js` | pthreads build: thread count, `csSyncHeap`, `csTerminateThreads` |
| `tests/md5_selftest.cpp` | native check of the MD5 shim |

## Not in the engine

| upstream feature | needs | engine behaviour |
|---|---|---|
| STEP / SVG-as-model import, Design-tab B-rep kernel | OCCT | import fails cleanly; sketch solver/inference/import are in |
| FBX/GLB/DAE/… import (`Format/AssimpImport`) | Assimp | fails cleanly (host loads meshes) |
| Draco mesh files | Draco | fails cleanly |
| texture → colour painting of textured imports | OpenCV | fails cleanly |
| SLA hollowing / drain holes | OpenVDB | no-op (FFF only) |
| Python plugins | CPython | hooks bridged to the host (`Module.csHook`) |
| multithreading (single-threaded build) | pthreads | sequential TBB shim; use `build-wasm-mt` |
