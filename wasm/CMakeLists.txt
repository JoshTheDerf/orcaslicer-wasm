cmake_minimum_required(VERSION 3.22)
project(slicer_wasm)

# Release by default
set(CMAKE_BUILD_TYPE Release CACHE STRING "")

# --- Make our custom find-modules visible (Boost/TBB/OpenSSL shims) ---
list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

# --- Threading model ---
# For the MVP, build single-threaded to avoid SharedArrayBuffer / shared-memory constraints in browsers
# and to simplify linking (no atomics/bulk-memory requirements).
set(EM_PTHREAD_FLAGS "")

# --- Point to locally built Boost (headers + static libs) ---
set(BOOST_PREFIX "${CMAKE_SOURCE_DIR}/../deps/boost-wasm/install")
set(BOOST_INC    "${BOOST_PREFIX}/include")
set(BOOST_LIB    "${BOOST_PREFIX}/lib")

# Export to subprojects (orca)
set(BOOST_ROOT         "${BOOST_PREFIX}" CACHE PATH "Boost root"             FORCE)
set(Boost_ROOT         "${BOOST_PREFIX}" CACHE PATH "Boost root (CamelCase)" FORCE)
set(BOOST_INCLUDEDIR   "${BOOST_INC}"    CACHE PATH "Boost include dir"      FORCE)
set(BOOST_LIBRARYDIR   "${BOOST_LIB}"    CACHE PATH "Boost library dir"      FORCE)
set(Boost_NO_SYSTEM_PATHS ON             CACHE BOOL ""                       FORCE)
set(Boost_USE_STATIC_LIBS ON             CACHE BOOL ""                       FORCE)
set(Boost_USE_MULTITHREADED ON           CACHE BOOL ""                       FORCE)

# --- Staged GMP/MPFR/CGAL toolchain for WASM ---
set(WASM_MATH_PREFIX "${CMAKE_SOURCE_DIR}/../deps/toolchain-wasm/install")
set(WASM_MATH_PREFIX "${WASM_MATH_PREFIX}" CACHE PATH "Prefix with GMP/MPFR/CGAL for WASM" FORCE)
set(ENV{WASM_MATH_PREFIX} "${WASM_MATH_PREFIX}")
list(PREPEND CMAKE_PREFIX_PATH "${WASM_MATH_PREFIX}")

# --- Create WASM shims for complex dependencies ---
add_library(wasm_shims INTERFACE)
target_include_directories(wasm_shims INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/wasm_shims
)
target_compile_definitions(wasm_shims INTERFACE
  WASM_BUILD=1
  TBB_MOCK=1
  SLIC3R_NO_TBB=1
)

# --- Prefer our shims for cereal/libigl but keep real Boost visible ---
include_directories(BEFORE
  ${CMAKE_CURRENT_SOURCE_DIR}/wasm_shims
  ${CMAKE_CURRENT_SOURCE_DIR}/wasm_shims/boost_runtime
  ${CMAKE_CURRENT_SOURCE_DIR}/wasm_shims/libslic3r
  ${BOOST_INC}
)

# --- Eigen include (try common in-tree layouts) ---
# Use the dir that contains the "Eigen/" folder.
set(_eigen_candidates)

if(EIGEN3_INCLUDE_DIR)
  list(APPEND _eigen_candidates "${EIGEN3_INCLUDE_DIR}")
endif()

list(APPEND _eigen_candidates
  "${PROJECT_SOURCE_DIR}/deps_src/eigen3"
  "${PROJECT_SOURCE_DIR}/deps_src/eigen"
  "${PROJECT_SOURCE_DIR}/../orca/deps_src/eigen3"
  "${PROJECT_SOURCE_DIR}/../orca/deps_src/eigen"
)

set(_eigen_resolved "")
foreach(_candidate IN LISTS _eigen_candidates)
  if(EXISTS "${_candidate}/Eigen/Core")
    set(_eigen_resolved "${_candidate}")
    break()
  endif()
endforeach()

if(_eigen_resolved)
  include_directories("${_eigen_resolved}")
  set(EIGEN3_INCLUDE_DIR "${_eigen_resolved}" CACHE PATH "Eigen include dir for WASM build" FORCE)
  set(EIGEN3_INCLUDE_DIRS "${_eigen_resolved}" CACHE PATH "Eigen include dirs for WASM build" FORCE)
else()
  message(WARNING "Eigen headers not found – CGAL components may fail to configure")
endif()

# Eigen runtime-sane flags for wasm
add_compile_definitions(
  EIGEN_DONT_ALIGN_STATICALLY=1
  EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT=1
  EIGEN_HAS_STD_RESULT_OF=0
  _LIBCPP_ENABLE_CXX17_REMOVED_FEATURES=1
  _LIBCPP_DISABLE_DEPRECATION_WARNINGS=1
)

# --- Disable TBB usage (shim will satisfy REQUIRED without linking) ---
set(SLIC3R_TBB OFF CACHE BOOL "Disable TBB in WASM build" FORCE)
set(SLIC3R_USE_TBB OFF CACHE BOOL "" FORCE)

# OCCT-based functionality is not available in the WASM toolchain
set(SLIC3R_WITH_OCCT OFF CACHE BOOL "Disable OCCT for WASM" FORCE)
set(SLIC3R_WITH_OPENCV OFF CACHE BOOL "Disable OpenCV for WASM" FORCE)
# Draco isn't built for wasm32; load_drc() falls back to a no-op stub.
set(SLIC3R_WITH_DRACO OFF CACHE BOOL "Disable Draco for WASM" FORCE)

# --- Tell Orca to avoid GUI and NLS (no Boost.Locale required) ---
set(SLIC3R_GUI OFF CACHE BOOL "Disable GUI for WASM" FORCE)
set(SLIC3R_NLS OFF CACHE BOOL "Disable translations for WASM" FORCE)
set(SLIC3R_ENC_CHECK OFF CACHE BOOL "Disable encoding check for WASM" FORCE)

# --- Emscripten-specific compile options ---
if(EMSCRIPTEN)
  # Silence unknown warning option spam
  add_compile_options(-Wno-unknown-warning-option)

  # Remove problematic warning flags that Emscripten doesn't support
  string(REPLACE "-Werror=enum-constexpr-conversion" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  string(REPLACE "-Werror=enum-constexpr-conversion" "" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")

  # Disable PCH for WASM builds to avoid include ordering issues
  set(SLIC3R_PCH OFF CACHE BOOL "Disable PCH for WASM" FORCE)

endif()

# --- Subprojects ---

# Provide Threads::Threads interface target for Emscripten (no real pthread lib needed)
if(NOT TARGET Threads::Threads)
  add_library(Threads::Threads INTERFACE IMPORTED)
endif()

if(NOT EMSCRIPTEN)
  target_compile_options(Threads::Threads INTERFACE -pthread)
  target_link_options(Threads::Threads INTERFACE -pthread)
endif()

# Provide TBB shim targets early so subprojects can link even if find_package(TBB) is disabled
if(EMSCRIPTEN)
  if(NOT TARGET TBB::tbb)
    add_library(TBB::tbb INTERFACE IMPORTED)
    # Point includes to our wasm shims; they provide minimal containers/APIs
    set_target_properties(TBB::tbb PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_LIST_DIR}/wasm_shims;${CMAKE_CURRENT_LIST_DIR}/wasm_shims/tbb;${CMAKE_CURRENT_LIST_DIR}/wasm_shims/oneapi"
    )
  endif()
  if(NOT TARGET TBB::tbbmalloc)
    add_library(TBB::tbbmalloc INTERFACE IMPORTED)
    set_target_properties(TBB::tbbmalloc PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_LIST_DIR}/wasm_shims;${CMAKE_CURRENT_LIST_DIR}/wasm_shims/tbb;${CMAKE_CURRENT_LIST_DIR}/wasm_shims/oneapi"
    )
  endif()
endif()

add_subdirectory(../bridge ${CMAKE_BINARY_DIR}/bridge-build)
add_subdirectory(../orca   ${CMAKE_BINARY_DIR}/orca-build  EXCLUDE_FROM_ALL)

# --- Finalise bridge wiring now that targets exist ---
if (TARGET orca_wasm_bridge)
  if (EXISTS "${PROJECT_SOURCE_DIR}/deps_src/semver/include/semver/semver.h")
    target_include_directories(orca_wasm_bridge BEFORE PRIVATE
      ${PROJECT_SOURCE_DIR}/deps_src/semver/include
    )
  elseif (EXISTS "${PROJECT_SOURCE_DIR}/deps_src/semver/semver/semver.h")
    target_include_directories(orca_wasm_bridge BEFORE PRIVATE
      ${PROJECT_SOURCE_DIR}/deps_src/semver
    )
  elseif (EXISTS "${PROJECT_SOURCE_DIR}/../orca/deps_src/semver/include/semver/semver.h")
    target_include_directories(orca_wasm_bridge BEFORE PRIVATE
      ${PROJECT_SOURCE_DIR}/../orca/deps_src/semver/include
    )
  elseif (EXISTS "${PROJECT_SOURCE_DIR}/../orca/deps_src/semver/semver/semver.h")
    target_include_directories(orca_wasm_bridge BEFORE PRIVATE
      ${PROJECT_SOURCE_DIR}/../orca/deps_src/semver
    )
  endif()

  if(NOT SLIC3R_WITH_OCCT)
    target_compile_definitions(orca_wasm_bridge PUBLIC SLIC3R_NO_OCCT)
  endif()
endif()

if (NOT SLIC3R_WITH_OCCT)
  if (TARGET libslic3r)
    target_compile_definitions(libslic3r PUBLIC SLIC3R_NO_OCCT)
    target_include_directories(libslic3r BEFORE PUBLIC
      ${PROJECT_SOURCE_DIR}/wasm_shims
      ${PROJECT_SOURCE_DIR}/wasm_shims/boost_runtime
    )
  endif()
  if (TARGET libslic3r_cgal)
    target_compile_definitions(libslic3r_cgal PUBLIC SLIC3R_NO_OCCT)
    target_include_directories(libslic3r_cgal BEFORE PUBLIC
      ${PROJECT_SOURCE_DIR}/wasm_shims
      ${PROJECT_SOURCE_DIR}/wasm_shims/boost_runtime
    )
  endif()
endif()

# --- Suppress third-party warnings for Emscripten ---
if(EMSCRIPTEN AND TARGET mcut)
  target_compile_options(mcut PRIVATE -Wno-deprecated-non-prototype)
endif()

# Eigen settings for libslic3r
if(EMSCRIPTEN AND TARGET libslic3r)
  target_compile_definitions(libslic3r PRIVATE
    EIGEN_DONT_ALIGN_STATICALLY=1
    EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT=1
    ORCA_DISABLE_OPENVDB=1
  )
endif()

if(EMSCRIPTEN AND TARGET libslic3r_cgal)
  target_compile_options(libslic3r_cgal PRIVATE
    -frounding-math
    -fno-fast-math
    -ffp-contract=off
  )
  target_compile_definitions(libslic3r_cgal PRIVATE CGAL_DISABLE_ROUNDING_MATH_CHECK)
endif()

# --- Executable (bridge that links to Orca slicer) ---
add_executable(slicer ../bridge/wasm_wrap.cpp)

target_include_directories(slicer PRIVATE
  # Ensure our WASM shims take precedence over system/Boost includes
  ${CMAKE_CURRENT_SOURCE_DIR}/wasm_shims
  ${CMAKE_CURRENT_SOURCE_DIR}/wasm_shims/libslic3r
  ${CMAKE_CURRENT_SOURCE_DIR}/wasm_shims/boost_runtime
  # Project includes
  ../bridge
  ../orca/src
  ${CMAKE_BINARY_DIR}/orca-build/src/libslic3r
  # Finally, real Boost headers if needed
  ${BOOST_INC}
)

if(NOT SLIC3R_WITH_OCCT)
  target_compile_definitions(slicer PRIVATE SLIC3R_NO_OCCT)
endif()

# Link Orca slicer libraries and our shims
target_link_libraries(slicer PRIVATE
  libslic3r
  libslic3r_cgal
  wasm_shims
)

# Link Boost static archives that we know exist
# Link Boost static archives that are required at link time.
# Avoid Boost.Thread for single-threaded WASM build; Boost.Log will still function in single-threaded mode.
target_link_libraries(slicer PRIVATE
  "${BOOST_LIB}/libboost_system.a"
  "${BOOST_LIB}/libboost_filesystem.a"
  "${BOOST_LIB}/libboost_regex.a"
  "${BOOST_LIB}/libboost_chrono.a"
  "${BOOST_LIB}/libboost_atomic.a"
  "${BOOST_LIB}/libboost_date_time.a"
  "${BOOST_LIB}/libboost_iostreams.a"
  "${BOOST_LIB}/libboost_program_options.a"
  "${BOOST_LIB}/libboost_log.a"
  "${BOOST_LIB}/libboost_log_setup.a"
)

# --- Emscripten link flags ---
set(EM_FLAGS
  "-O3"
  "-sMODULARIZE=1"
  "-sEXPORT_ES6=1"
  "-sWASM_BIGINT=1"
  "-sALLOW_MEMORY_GROWTH=1"
  "-sSTACK_SIZE=16777216"
  "-sABORTING_MALLOC=0"
  "-sENVIRONMENT=web,worker"
  "-sDISABLE_EXCEPTION_CATCHING=0"
  "-fexceptions"
  ${EM_PTHREAD_FLAGS}
)


# --- Fix em++ options: use target_link_options instead of LINK_FLAGS ---
# First, clear any previous LINK_FLAGS we set earlier:

# --- Emscripten link options ---
# Link-time -O2 (not -O3): -O3 + --fpcast-emu (needed for our function-pointer
# casts) triggers a wasm-opt assertion in binaryen's directize pass on the
# orca binary. -O2 dodges that pass without affecting runtime correctness;
# size delta is negligible because libslic3r object files are already -O3.
target_link_options(slicer PRIVATE
  -O2
  -sALLOW_MEMORY_GROWTH=1
  -sMALLOC=emmalloc
  -sSTACK_SIZE=16777216
  -sINITIAL_MEMORY=268435456
  -sMAXIMUM_MEMORY=4294967296
  -sABORTING_MALLOC=0
  -sENVIRONMENT=web,worker,node
  -sMODULARIZE=1
  -sEXPORT_NAME=OrcaModule
  -sDISABLE_EXCEPTION_CATCHING=0
  # EMULATE_FUNCTION_POINTER_CASTS triggers wasm-opt's --fpcast-emu pass
  # which hits a binaryen assertion on the orca binary (interacts badly with
  # the directize-initial-contents-immutable pass). Disabled until either
  # emsdk ships a fixed binaryen or we identify the indirect call that
  # genuinely needs it. The slice path doesn't appear to rely on cross-
  # signature function pointer casts.
  # -sEMULATE_FUNCTION_POINTER_CASTS=1
  -fexceptions
  # Resource preload: orca's full resources/ tree is ~230 MB on disk
  # (profiles 70M, hms 63M, images 35M, fonts 35M, web 19M, …) — almost all
  # of it is UI / BBL telemetry / preset previews that headless slicing
  # never touches. Audit of src/libslic3r shows only these subdirs are
  # actually read from at slice time:
  #   info/   — nozzle_info.json, nozzle_incompatibles.json (~16 KB)
  #   flush/  — multi-material flush volume tables (~24 KB)
  #
  # Profiles are split out and served separately by the host. The host
  # writes the specific vendor JSON(s) it needs into MEMFS at
  # /resources/profiles/ before slicing, so we don't pack them either —
  # see cad-project/scripts/fetch-orca-wasm.sh and OrcaBackend's loader.
  --preload-file=../orca/resources/info@/resources/info
  --preload-file=../orca/resources/flush@/resources/flush
  "-sEXPORTED_FUNCTIONS=['_orc_init','_orc_slice','_malloc','_free','_orc_free','_orc_decode_exception']"
  "-sEXPORTED_RUNTIME_METHODS=['ccall','cwrap','getValue','setValue','UTF8ToString','stringToUTF8','lengthBytesUTF8','HEAP8','HEAPU8','HEAP32','HEAPU32']"
)
