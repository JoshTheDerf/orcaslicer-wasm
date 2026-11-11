# Design-tab CAD kernel module (cad.mjs / cad.wasm): OrcaSlicer's libslic3r/CAD on real
# OpenCASCADE 8.0.1, as a SEPARATE module from the slicer engine (which stays OCCT-free).
# Included by CMakeLists.txt only with -DORCA_WASM_CAD=ON (build dir build-cad/, see
# scripts/build-cad.sh); the slicer targets and their flags are untouched.
#
# Contract: cubby-slicer/docs/CAD-KERNEL-CONTRACT.md.
#
# What links:
#   cad_bridge.cpp                         the stateful JSON dispatcher (cad_call)
#   CAD/CadDocument, GeometryEngine,       the OCCT kernel half of libslic3r/CAD (SLIC3R_CAD=OFF
#   SketchEngine                           leaves them out of libslic3r; compiled here)
#   libslic3r (archive)                    only the members those reference are pulled in:
#                                          TriangleMesh, Point, the OCCT-free CAD half (SketchSolver,
#                                          SketchInference, SketchConstraints, ThreadStandards), …
#   libslvs, OCCT toolkits (static)        wasm-deps/occt-8.0.1 (toolchain/build-deps.sh occt)
#
# libslic3r's own stubs/cad_sketch.cpp defines sketch_join_tol()/set_sketch_auto_close(), which
# the real SketchEngine.cpp also defines. SketchEngine's object is linked directly, so the
# archive member is never pulled (and the link fails loudly on a duplicate if that ever changes).
if(ORCA_WASM_THREADS)
  message(FATAL_ERROR "ORCA_WASM_CAD: the CAD module is single-threaded only")
endif()

# In a CAD configuration (build-cad/) libslic3r is cut down to the members the kernel actually
# links: the list below is exactly what wasm-ld pulls out of the full slicer libslic3r archive for
# cad_bridge + CadDocument + GeometryEngine + SketchEngine (`-Wl,--why-extract`; mostly
# TriangleMesh's closure, which reaches PrintConfig through ClipperUtils). Same target, so every
# compile flag, define and include stays libslic3r's own. The slicer target is NOT buildable in a
# CAD build dir (build it in build-wasm/ as always).
set(_cad_slic3r_members
  AppConfig ArcFitter BoundingBox Circle Geometry/Circle ClipperUtils Color ColorDecomposeRecipe Config
  Geometry/ConvexHull Emboss ExPolygon ExPolygonsIndex ExtrusionEntity FilamentMixer Flow Geometry
  InstanceLock Line LocalesUtils MaterialType Geometry/MedialAxis miniz_extension MultiPoint NSVGUtils
  GCode/OrderingStrategies PlaceholderParser Point Polygon Polyline PresetBundle PresetCacheFormat Preset
  PrintConfig PublishSettings ShortestPath CAD/SketchConstraints CAD/SketchImport CAD/SketchInference
  CAD/SketchSolver Tesselate Thread CAD/ThreadStandards GCode/Thumbnails Time TriangleMesh
  TriangleMeshSlicer utils Geometry/Voronoi Geometry/VoronoiOffset Geometry/VoronoiUtilsCgal
  Geometry/VoronoiUtils Zipper)
set(_cad_slic3r_src ${CMAKE_CURRENT_LIST_DIR}/stubs/nanosvg.cpp)
foreach(_m ${_cad_slic3r_members})
  if(NOT EXISTS "${ORCA_LIBSLIC3R}/${_m}.cpp")
    message(FATAL_ERROR "cad.cmake: libslic3r/${_m}.cpp not found (Orca layout changed? re-derive the member list)")
  endif()
  list(APPEND _cad_slic3r_src "${ORCA_LIBSLIC3R}/${_m}.cpp")
endforeach()
set_property(TARGET libslic3r PROPERTY SOURCES ${_cad_slic3r_src})
set_property(TARGET libslic3r_cgal PROPERTY SOURCES
  ${ORCA_LIBSLIC3R}/IntersectionPoints.cpp ${ORCA_LIBSLIC3R}/Triangulation.cpp)
set_target_properties(slicer PROPERTIES EXCLUDE_FROM_ALL TRUE)

set(WASM_OCCT_PREFIX "$ENV{WASM_DEPS_ROOT}/occt-8.0.1" CACHE PATH "OCCT 8.0.1 static install (build-deps.sh occt)" FORCE)
if(NOT EXISTS "${WASM_OCCT_PREFIX}/lib/libTKDESTEP.a")
  # Configuring (and building libslic3r) works without it; compiling the kernel does not.
  message(WARNING "OCCT libraries missing in ${WASM_OCCT_PREFIX} — bash wasm-deps/build-deps.sh occt")
endif()
set(WASM_OCCT_INCLUDE "${WASM_OCCT_PREFIX}/include/occt")

# Link order: dependents before their dependencies (static archives, single pass).
set(_occt_tk TKDESTEP TKXCAF TKVCAF TKCAF TKLCAF TKCDF TKV3d TKHLR TKService TKDE TKXSBase
             TKFillet TKOffset TKBool TKBO TKPrim TKMesh TKShHealing TKTopAlgo TKGeomAlgo TKBRep
             TKGeomBase TKG3d TKG2d TKMath TKernel)
set(_occt_libs "")
foreach(_tk ${_occt_tk})
  if(EXISTS "${WASM_OCCT_PREFIX}/lib/lib${_tk}.a")
    list(APPEND _occt_libs "${WASM_OCCT_PREFIX}/lib/lib${_tk}.a")
  endif()
endforeach()

add_executable(cad
  ../bridge/cad_bridge.cpp
  ${ORCA_LIBSLIC3R}/CAD/CadDocument.cpp
  ${ORCA_LIBSLIC3R}/CAD/GeometryEngine.cpp
  ${ORCA_LIBSLIC3R}/CAD/SketchEngine.cpp)
set_target_properties(cad PROPERTIES OUTPUT_NAME "cad" SUFFIX ".mjs")
target_include_directories(cad PRIVATE
  ${ORCA_SRC_DIR}/src
  ${ORCA_SRC_DIR}/deps_src
  ${CMAKE_BINARY_DIR}/orca-build/src/libslic3r
  ${CMAKE_CURRENT_LIST_DIR}/../bridge/common)
target_include_directories(cad SYSTEM PRIVATE ${WASM_OCCT_INCLUDE})
# The (whole-program) archives; -Wl,--start-group lets OCCT's cross-toolkit references resolve
# whatever order the members are needed in.
target_link_libraries(cad PRIVATE libslic3r ${Boost_LIBRARIES}
  -Wl,--start-group ${_occt_libs} -Wl,--end-group)

set(_cad_link
  -sALLOW_MEMORY_GROWTH=1
  -sINITIAL_MEMORY=128MB
  -sMAXIMUM_MEMORY=4GB
  -sSTACK_SIZE=64MB
  -sABORTING_MALLOC=0
  -sMALLOC=dlmalloc
  -sMODULARIZE=1
  -sEXPORT_ES6=1
  -sEXPORT_NAME=OrcaCadModule
  -sENVIRONMENT=web,worker,node
  -sUSE_ZLIB=1
  -sFILESYSTEM=1
  -sEXIT_RUNTIME=0
  "-sEXPORTED_RUNTIME_METHODS=['HEAPU8','HEAPU32','UTF8ToString']"
  "-sINCOMING_MODULE_JS_API=['locateFile','print','printErr','instantiateWasm','wasmBinary','onAbort']"
  "-sEXPORTED_FUNCTIONS=['_cad_version','_cad_call','_cad_free','_malloc','_free']")
if(BUILD_VARIANT STREQUAL "debug")
  list(APPEND _cad_link -O1 -g2 -sASSERTIONS=2 -sSAFE_HEAP=1 -sSTACK_OVERFLOW_CHECK=2)
else()
  set(ORCA_CAD_LINK_OPT "-O3" CACHE STRING "Link-time (wasm-opt) optimisation level of cad.wasm")
  option(ORCA_CAD_NAMES "Keep function names in cad.wasm (readable stack traces)" OFF)
  if(ORCA_CAD_NAMES)
    list(APPEND _cad_link --profiling-funcs)
  endif()
  list(APPEND _cad_link ${ORCA_CAD_LINK_OPT} -sSTACK_OVERFLOW_CHECK=1)
endif()
target_link_options(cad PRIVATE ${_cad_link})
