if(ORCA_WASM_THREADS)
  # Pthreads build: real oneTBB (static, -pthread) from wasm-deps/install-mt.
    foreach(_l tbb tbbmalloc)
    if(NOT EXISTS "${WASM_DEPS_PREFIX}/lib/lib${_l}.a")
      message(FATAL_ERROR "lib${_l}.a missing in ${WASM_DEPS_PREFIX} — WASM_THREADS=1 bash wasm-deps/build-deps.sh all-mt")
    endif()
  endforeach()
  set(TBB_FOUND TRUE)
  set(TBB_VERSION_MAJOR 2022)
  foreach(_t tbb tbbmalloc)
    if(NOT TARGET TBB::${_t})
      add_library(TBB::${_t} STATIC IMPORTED GLOBAL)
      set_target_properties(TBB::${_t} PROPERTIES
        IMPORTED_LOCATION "${WASM_DEPS_PREFIX}/lib/lib${_t}.a"
        INTERFACE_INCLUDE_DIRECTORIES "${WASM_DEPS_PREFIX}/include"
        INTERFACE_LINK_OPTIONS "-pthread")
    endif()
  endforeach()
  if(NOT TARGET TBB::tbbmalloc_proxy)
    add_library(TBB::tbbmalloc_proxy INTERFACE IMPORTED GLOBAL)  # never used: no malloc replacement
  endif()
  return()
endif()

# Sequential TBB replacement (header-only, wasm_shims/tbb + oneapi/tbb). No
# TBB library is linked: the TBB::* targets are empty INTERFACE targets that
# only put the shim on the include path, so orca's find_package(TBB REQUIRED)
# and target_link_libraries(... TBB::tbb) work unpatched.
set(TBB_FOUND TRUE)
set(TBB_VERSION_MAJOR 2022)
foreach(_t TBB::tbb TBB::tbbmalloc TBB::tbbmalloc_proxy)
  if(NOT TARGET ${_t})
    add_library(${_t} INTERFACE IMPORTED GLOBAL)
    set_target_properties(${_t} PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${WASM_SHIMS_DIR}")
  endif()
endforeach()
