# Not linked in the WASM build (code paths compiled out).
include(${CMAKE_CURRENT_LIST_DIR}/WasmStub.cmake)
wasm_stub_package(OpenCV OpenCV::opencv opencv_core)
set(OpenCV_LIBS "")
set(OpenCV_VERSION "4.0.0")
