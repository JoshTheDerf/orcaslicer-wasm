# Not linked in the WASM build (Format/AssimpImport.cpp is replaced by
# wasm/stubs/formats.cpp; the host loads meshes itself).
include(${CMAKE_CURRENT_LIST_DIR}/WasmStub.cmake)
wasm_stub_package(assimp assimp::assimp)
