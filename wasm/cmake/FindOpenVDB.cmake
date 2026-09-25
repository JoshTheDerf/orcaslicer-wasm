# Not linked in the WASM build (code paths compiled out).
include(${CMAKE_CURRENT_LIST_DIR}/WasmStub.cmake)
wasm_stub_package(OpenVDB OpenVDB::openvdb)
set(OpenVDB_VERSION "10.0.0")
