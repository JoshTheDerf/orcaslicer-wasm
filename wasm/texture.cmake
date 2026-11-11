# Texture import module (texture.mjs / texture.wasm): OrcaSlicer 2.5's import of textured and coloured
# models (glTF/GLB/FBX through Assimp, texture -> colour painting through OpenCV), as a SEPARATE module
# from the slicer engine, loaded only when a user imports such a model. The slicer stays Assimp- and
# OpenCV-free (wasm/stubs/assimp_import.cpp, wasm/stubs/texture_painting.cpp).
#
# Built in the single-threaded slicer build dir (build-wasm/, scripts/build-texture.sh) and linked against
# the SAME libslic3r archive as slicer.wasm (identical flags; only the members the module references are
# pulled in). The upstream sources that need the two libraries are compiled here as objects, so they take
# precedence over libslic3r's stand-ins (separate archive members, never pulled once these define them):
#   Format/AssimpImport.cpp                        Assimp 5.4.3 (wasm-deps/assimp-5.4.3)
#   TexturePainting.cpp, TextureToColor/*.cpp      OpenCV 4.14.0 core/imgproc/imgcodecs (wasm-deps/opencv-4.14.0)
# Libraries come from their own prefixes (toolchain/build-deps.sh texture), never from the slicer's.
# Single-threaded only (like the CAD module): the import runs once per model, on the host's worker.
if(ORCA_WASM_THREADS)
  return()
endif()

set(WASM_ASSIMP_PREFIX "$ENV{WASM_DEPS_ROOT}/assimp-5.4.3" CACHE PATH "Assimp 5.4.3 static install (build-deps.sh assimp)" FORCE)
set(WASM_OPENCV_PREFIX "$ENV{WASM_DEPS_ROOT}/opencv-4.14.0" CACHE PATH "OpenCV 4.14.0 static install (build-deps.sh opencv)" FORCE)
if(NOT EXISTS "${WASM_ASSIMP_PREFIX}/lib/libassimp.a" OR NOT EXISTS "${WASM_OPENCV_PREFIX}/lib/libopencv_imgcodecs.a")
  message(STATUS "texture module: Assimp / OpenCV not built (bash wasm-deps/build-deps.sh texture); target 'texture' not defined")
  return()
endif()

set(_tex_upstream
  ${ORCA_LIBSLIC3R}/Format/AssimpImport.cpp
  ${ORCA_LIBSLIC3R}/TexturePainting.cpp
  ${ORCA_LIBSLIC3R}/TextureToColor/TextureToColor.cpp
  ${ORCA_LIBSLIC3R}/TextureToColor/ColorUtils.cpp)
# Compiled exactly as libslic3r compiles its own members (its include dirs, definitions and options).
add_library(texture_upstream OBJECT ${_tex_upstream})
target_include_directories(texture_upstream PRIVATE
  $<TARGET_PROPERTY:libslic3r,INCLUDE_DIRECTORIES>
  ${CMAKE_BINARY_DIR}/orca-build/src/libslic3r)
target_include_directories(texture_upstream SYSTEM PRIVATE
  "${WASM_OPENCV_PREFIX}/include/opencv4" "${WASM_ASSIMP_PREFIX}/include" "${WASM_CGAL_PREFIX}/include")
target_compile_definitions(texture_upstream PRIVATE $<TARGET_PROPERTY:libslic3r,COMPILE_DEFINITIONS>)
target_compile_options(texture_upstream PRIVATE $<TARGET_PROPERTY:libslic3r,COMPILE_OPTIONS>)
# TextureToColor's mesh repair runs CGAL Polygon_mesh_processing: CGAL's strict-rounding flags, as libslic3r_cgal.
set_source_files_properties(${ORCA_LIBSLIC3R}/TextureToColor/TextureToColor.cpp PROPERTIES
  COMPILE_OPTIONS "-frounding-math;-fno-fast-math;-ffp-contract=off;-DCGAL_DISABLE_ROUNDING_MATH_CHECK;-DBOOST_NUMERIC_INTERVAL_HW_ROUNDING_HPP")

add_executable(texture EXCLUDE_FROM_ALL ../bridge/texture_bridge.cpp $<TARGET_OBJECTS:texture_upstream>)
set_target_properties(texture PROPERTIES OUTPUT_NAME "texture" SUFFIX ".mjs")
target_include_directories(texture PRIVATE
  ${ORCA_SRC_DIR}/src
  ${ORCA_SRC_DIR}/deps_src
  ${CMAKE_BINARY_DIR}/orca-build/src/libslic3r
  ${CMAKE_CURRENT_LIST_DIR}/../bridge/common)
target_include_directories(texture SYSTEM PRIVATE "${WASM_OPENCV_PREFIX}/include/opencv4" "${WASM_ASSIMP_PREFIX}/include")
target_link_libraries(texture PRIVATE libslic3r libslic3r_cgal ${Boost_LIBRARIES}
  "${WASM_ASSIMP_PREFIX}/lib/libassimp.a"
  "${WASM_OPENCV_PREFIX}/lib/libopencv_imgcodecs.a"
  "${WASM_OPENCV_PREFIX}/lib/libopencv_imgproc.a"
  "${WASM_OPENCV_PREFIX}/lib/libopencv_core.a"
  "${WASM_DEPS_PREFIX}/lib/libmpfr.a" "${WASM_DEPS_PREFIX}/lib/libgmp.a")

set(_tex_link
  -Wl,--wrap=strtod -Wl,--wrap=strtof
  -sALLOW_MEMORY_GROWTH=1
  -sINITIAL_MEMORY=128MB
  -sMAXIMUM_MEMORY=4GB
  -sSTACK_SIZE=64MB
  -sABORTING_MALLOC=0
  -sMALLOC=dlmalloc
  -sMODULARIZE=1
  -sEXPORT_ES6=1
  -sEXPORT_NAME=OrcaTextureModule
  -sENVIRONMENT=web,worker,node
  -sUSE_ZLIB=1 -sUSE_LIBPNG=1 -sUSE_LIBJPEG=1
  -sFILESYSTEM=1
  -sEXIT_RUNTIME=0
  "-sEXPORTED_RUNTIME_METHODS=['HEAPU8','UTF8ToString']"
  "-sINCOMING_MODULE_JS_API=['locateFile','print','printErr','instantiateWasm','wasmBinary','getPreloadedPackage','onAbort','monitorRunDependencies','setStatus']"
  "-sEXPORTED_FUNCTIONS=['_tex_version','_tex_call','_tex_free','_malloc','_free']"
  # CMYW / RYBW auto-mix recipe tables (ColorDecomposeRecipe), 250 KB.
  "--preload-file=${ORCA_SRC_DIR}/resources/filament_mixing@/resources/filament_mixing")
target_sources(texture PRIVATE ../bridge/common/cs_fast_strtod.cpp)
if(BUILD_VARIANT STREQUAL "debug")
  list(APPEND _tex_link -O1 -g2 -sASSERTIONS=2 -sSAFE_HEAP=1 -sSTACK_OVERFLOW_CHECK=2)
else()
  list(APPEND _tex_link -O3 -sSTACK_OVERFLOW_CHECK=2)
endif()
target_link_options(texture PRIVATE ${_tex_link})
