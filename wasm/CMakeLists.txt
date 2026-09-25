# OrcaSlicer → WebAssembly superbuild (Cubby Slicer engine ABI v1).
#
#   source ../wasm-deps/env.sh
#   emcmake cmake -S wasm -B build-wasm -G Ninja -DCMAKE_BUILD_TYPE=Release
#   cmake --build build-wasm --target slicer
#
# See cubby-slicer/docs/ENGINE-CONTRACT.md for the build invariants this file
# enforces (single exception model, no function-pointer-cast emulation, no
# shims shadowing linked libraries).
cmake_minimum_required(VERSION 3.22)
project(orca_wasm CXX C)

if(NOT EMSCRIPTEN)
  message(FATAL_ERROR "Configure with emcmake (source wasm-deps/env.sh first).")
endif()

set(BUILD_VARIANT "release" CACHE STRING "release | debug (SAFE_HEAP + assertions)")
set_property(CACHE BUILD_VARIANT PROPERTY STRINGS release debug)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

# ---- shared deps (wasm-deps) ------------------------------------------------
if(NOT DEFINED ENV{WASM_DEPS_PREFIX})
  message(FATAL_ERROR "WASM_DEPS_PREFIX not set — source wasm-deps/env.sh")
endif()
set(WASM_DEPS_PREFIX "$ENV{WASM_DEPS_PREFIX}" CACHE PATH "" FORCE)
set(WASM_CGAL_PREFIX "$ENV{WASM_DEPS_ROOT}/cgal-5.6.3" CACHE PATH "" FORCE)   # orca pins CGAL 5.6.3
set(WASM_SHIMS_DIR "${CMAKE_CURRENT_LIST_DIR}/wasm_shims" CACHE PATH "" FORCE)
set(EM_EH "$ENV{WASM_EH_FLAGS}")
if(NOT EM_EH)
  message(FATAL_ERROR "WASM_EH_FLAGS not set — source wasm-deps/env.sh")
endif()

list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(PREPEND CMAKE_PREFIX_PATH "${WASM_DEPS_PREFIX}" "${WASM_CGAL_PREFIX}")
set(Eigen3_DIR "${WASM_DEPS_PREFIX}/share/eigen3/cmake" CACHE PATH "" FORCE)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY BOTH)

# ---- one exception/longjmp model for every TU ------------------------------
separate_arguments(_eh UNIX_COMMAND "${EM_EH}")
add_compile_options(${_eh} -Wno-unknown-warning-option -Wno-deprecated-declarations
                    -Wno-unused-command-line-argument -Wno-missing-template-arg-list-after-template-kw)
add_link_options(${_eh})
if(BUILD_VARIANT STREQUAL "debug")
  add_compile_options(-g2)
endif()

# ccache if available
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
  set(CMAKE_C_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
  set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
endif()

# Shims: TBB (sequential), Boost.Thread (single-thread), unlinked stubs only.
include_directories(BEFORE SYSTEM
  "${WASM_SHIMS_DIR}"
  "${WASM_SHIMS_DIR}/boost_runtime")
include_directories(SYSTEM "${WASM_DEPS_PREFIX}/include" "${WASM_DEPS_PREFIX}/include/eigen3")

add_compile_definitions(
  WASM_BUILD=1
  SLIC3R_NO_TBB=1
  BOOST_LOG_NO_THREADS
  BOOST_LOG_STATIC_LINK
  BOOST_ALL_NO_LIB
  BOOST_NO_CXX98_FUNCTION_BASE
  ORCA_DISABLE_OPENVDB=1
  _LIBCPP_ENABLE_CXX17_REMOVED_FEATURES=1
  _LIBCPP_DISABLE_DEPRECATION_WARNINGS=1)

# ---- orca configuration ------------------------------------------------------
set(SLIC3R_STATIC ON CACHE BOOL "" FORCE)
set(SLIC3R_GUI OFF CACHE BOOL "" FORCE)
set(SLIC3R_PCH OFF CACHE BOOL "" FORCE)
set(SLIC3R_NLS OFF CACHE BOOL "" FORCE)
set(SLIC3R_ENC_CHECK OFF CACHE BOOL "" FORCE)
set(SLIC3R_WITH_OCCT OFF CACHE BOOL "" FORCE)
set(SLIC3R_WITH_OPENCV OFF CACHE BOOL "" FORCE)
set(SLIC3R_WITH_DRACO OFF CACHE BOOL "" FORCE)
set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(ORCA_TOOLS OFF CACHE BOOL "" FORCE)
set(ORCA_CHECK_GCODE_PLACEHOLDERS "0" CACHE STRING "" FORCE)
set(CMAKE_DISABLE_FIND_PACKAGE_TBB ON)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/FindTBB.cmake)  # header-only TBB::* targets

add_subdirectory(../orca ${CMAKE_BINARY_DIR}/orca-build EXCLUDE_FROM_ALL)

# VoronoiUtilsCgal.cpp lives in libslic3r but needs the (real) CGAL headers;
# scope the include to that one file so the rest of libslic3r is unaffected.
set_source_files_properties(
  ${CMAKE_CURRENT_LIST_DIR}/../orca/src/libslic3r/Geometry/VoronoiUtilsCgal.cpp
  DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/../orca/src/libslic3r
  PROPERTIES COMPILE_OPTIONS "-isystem;${WASM_CGAL_PREFIX}/include;-frounding-math")

set(ORCA_WASM_ASSERT_DIRS "" CACHE STRING "libslic3r subdirs (e.g. Arachne) compiled with assert() enabled")
foreach(_d ${ORCA_WASM_ASSERT_DIRS})
  file(GLOB _asrc ${CMAKE_CURRENT_LIST_DIR}/../orca/src/libslic3r/${_d}/*.cpp)
  set_source_files_properties(${_asrc} DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/../orca/src/libslic3r
    PROPERTIES COMPILE_OPTIONS "-UNDEBUG;-g")
endforeach()

if(TARGET libslic3r_cgal)
  # CGAL exact predicates need strict IEEE rounding.
  target_compile_options(libslic3r_cgal PRIVATE -frounding-math -fno-fast-math -ffp-contract=off)
  target_compile_definitions(libslic3r_cgal PRIVATE CGAL_DISABLE_ROUNDING_MATH_CHECK)
endif()
if(TARGET mcut)
  target_compile_options(mcut PRIVATE -Wno-deprecated-non-prototype)
endif()

# ---- engine module ------------------------------------------------------------
add_executable(slicer ../bridge/cs_bridge.cpp)
set_target_properties(slicer PROPERTIES OUTPUT_NAME "slicer" SUFFIX ".mjs")
target_include_directories(slicer PRIVATE
  ../orca/src
  ../orca/deps_src
  ${CMAKE_BINARY_DIR}/orca-build/src/libslic3r
  ${CMAKE_CURRENT_LIST_DIR}/../../wasm-bridge)
target_link_libraries(slicer PRIVATE libslic3r libslic3r_cgal ${Boost_LIBRARIES}
  "${WASM_DEPS_PREFIX}/lib/libmpfr.a" "${WASM_DEPS_PREFIX}/lib/libgmp.a")

set(_link
  -sALLOW_MEMORY_GROWTH=1
  -sINITIAL_MEMORY=128MB
  -sMAXIMUM_MEMORY=4GB
  -sSTACK_SIZE=64MB
  -sABORTING_MALLOC=0
  -sMALLOC=dlmalloc
  -sMODULARIZE=1
  -sEXPORT_ES6=1
  -sEXPORT_NAME=OrcaModule
  -sENVIRONMENT=web,worker,node
  -sUSE_ZLIB=1 -sUSE_LIBPNG=1 -sUSE_LIBJPEG=1 -sUSE_FREETYPE=1
  -sFILESYSTEM=1
  # Module hooks the host passes in (anything not listed is silently ignored in
  # release builds): the web worker uses instantiateWasm + getPreloadedPackage,
  # node tests use wasmBinary. csProgress is read at call time via Module[].
  "-sINCOMING_MODULE_JS_API=['locateFile','print','printErr','instantiateWasm','wasmBinary','getPreloadedPackage','onAbort','monitorRunDependencies','setStatus']"
  -sEXIT_RUNTIME=0
  "-sEXPORTED_FUNCTIONS=['_cs_version','_cs_describe_config','_cs_slice','_cs_eval_condition','_cs_orient','_cs_free','_malloc','_free']"
  "-sEXPORTED_RUNTIME_METHODS=['HEAPU8','HEAP32','HEAPU32','UTF8ToString','stringToUTF8','lengthBytesUTF8','FS']"
  # libslic3r reads only resources/info (nozzle tables) and resources/flush
  # (MM flush matrices) at slice time — ~40 KB. Profiles are resolved in JS.
  "--preload-file=${CMAKE_CURRENT_LIST_DIR}/../orca/resources/info@/resources/info"
  "--preload-file=${CMAKE_CURRENT_LIST_DIR}/../orca/resources/flush@/resources/flush"
)
if(BUILD_VARIANT STREQUAL "debug")
  list(APPEND _link -O1 -g2 -sASSERTIONS=2 -sSAFE_HEAP=1 -sSTACK_OVERFLOW_CHECK=2)
else()
  set(ORCA_WASM_LINK_OPT "-O3" CACHE STRING "Link-time (wasm-opt) optimisation level")
  option(ORCA_WASM_NAMES "Keep function names (readable wasm stack traces)" OFF)
  option(ORCA_WASM_SAFE_HEAP "Instrument every load/store (diagnostics only; slow)" OFF)
  if(ORCA_WASM_SAFE_HEAP)
    list(APPEND _link -sSAFE_HEAP=1 -sASSERTIONS=2)
  endif()
  if(ORCA_WASM_NAMES)
    list(APPEND _link --profiling-funcs)
  endif()
  # STACK_OVERFLOW_CHECK=2 bounds-checks every stack-pointer update, so a deep
  # recursion traps deterministically instead of silently overwriting the
  # static data that sits below the stack (emscripten's default layout).
  list(APPEND _link ${ORCA_WASM_LINK_OPT} -sSTACK_OVERFLOW_CHECK=2)
endif()
target_link_options(slicer PRIVATE ${_link})
