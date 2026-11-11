# Not linked in the WASM build (Format/DRC.cpp is replaced by wasm/stubs/formats.cpp).
include(${CMAKE_CURRENT_LIST_DIR}/WasmStub.cmake)
wasm_stub_package(Draco draco::draco)
