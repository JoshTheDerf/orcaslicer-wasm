# Not linked in the WASM build (code paths compiled out). libslic3r links the
# OCCT toolkits by bare name (OCCT_LIBS), so each one is an empty target here.
include(${CMAKE_CURRENT_LIST_DIR}/WasmStub.cmake)
wasm_stub_package(OpenCASCADE
  TKDESTEP TKDE TKXCAF TKXSBase TKVCAF TKCAF TKLCAF TKCDF TKV3d TKService TKMesh TKBO
  TKPrim TKHLR TKShHealing TKTopAlgo TKGeomAlgo TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel
  TKFillet TKOffset TKBool)
set(OpenCASCADE_INCLUDE_DIR "")
