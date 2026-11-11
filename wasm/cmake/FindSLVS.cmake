# Real libslvs (SolveSpace solver) from the toolchain prefix (build-deps.sh slvs).
if(NOT EXISTS "${WASM_DEPS_PREFIX}/lib/libslvs.a")
  message(FATAL_ERROR "libslvs.a missing in ${WASM_DEPS_PREFIX} — bash wasm-deps/build-deps.sh slvs")
endif()
set(SLVS_FOUND TRUE)
if(NOT TARGET SLVS::slvs)
  add_library(SLVS::slvs STATIC IMPORTED GLOBAL)
  set_target_properties(SLVS::slvs PROPERTIES
    IMPORTED_LOCATION "${WASM_DEPS_PREFIX}/lib/libslvs.a"
    INTERFACE_INCLUDE_DIRECTORIES "${WASM_DEPS_PREFIX}/include")
endif()
