# Not linked in the WASM build (the OpenCV users, ObjColorUtils and the texture →
# colour importer, are replaced by wasm/stubs/texture_painting.cpp).
include(${CMAKE_CURRENT_LIST_DIR}/WasmStub.cmake)
wasm_stub_package(OpenCV OpenCV::opencv opencv_core opencv_world)
set(OpenCV_LIBS "")
set(OpenCV_VERSION "4.0.0")
