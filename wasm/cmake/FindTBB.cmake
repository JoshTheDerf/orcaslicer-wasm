# Sequential TBB replacement (header-only, wasm_shims/tbb + oneapi/tbb). No
# TBB library is linked. TBB_FOUND stays FALSE so orca compiles with
# SLIC3R_USE_TBB=0; the TBB::* targets exist for target_link_libraries().
set(TBB_FOUND FALSE)
foreach(_t TBB::tbb TBB::tbbmalloc TBB::tbbmalloc_proxy)
  if(NOT TARGET ${_t})
    add_library(${_t} INTERFACE IMPORTED GLOBAL)
    set_target_properties(${_t} PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${WASM_SHIMS_DIR}")
  endif()
endforeach()
