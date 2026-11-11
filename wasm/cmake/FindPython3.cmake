# Not linked in the WASM build. Orca 2.5's top-level CMakeLists finds an embedded
# CPython unconditionally for its plugin layer (src/slic3r/plugin, GUI only);
# libslic3r has no Python dependency (plugin hooks are plain std::function seams,
# driven from JS by the bridge — see bridge/cs_hooks.cpp).
include(${CMAKE_CURRENT_LIST_DIR}/WasmStub.cmake)
wasm_stub_package(Python3 Python3::Python Python3::Module)
set(Python3_VERSION "${Python3_FIND_VERSION}")
set(Python3_EXECUTABLE "")
set(Python3_Interpreter_FOUND TRUE)
set(Python3_Development.Embed_FOUND TRUE)
