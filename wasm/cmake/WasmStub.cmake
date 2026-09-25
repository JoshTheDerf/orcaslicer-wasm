# Helpers for the WASM superbuild find-modules.
#
# wasm_stub_package(<Pkg> <targets...>): declare <Pkg> "found" with empty
#   INTERFACE targets. ONLY for libraries that are not linked into the engine
#   at all (their code paths are compiled out by the orca patch), so no
#   header/ABI mismatch is possible.
# wasm_port_target(<target> <-sUSE_...=1 flags>): an INTERFACE target that
#   pulls in an Emscripten port (real library) at compile + link time.
include_guard(GLOBAL)
macro(wasm_stub_package pkg)
  set(${pkg}_FOUND TRUE)
  string(TOUPPER "${pkg}" _up)
  set(${_up}_FOUND TRUE)
  set(${pkg}_INCLUDE_DIRS "")
  set(${pkg}_LIBRARIES "")
  set(${_up}_INCLUDE_DIRS "")
  set(${_up}_LIBRARIES "")
  foreach(_t ${ARGN})
    if(NOT TARGET ${_t})
      add_library(${_t} INTERFACE IMPORTED GLOBAL)
    endif()
  endforeach()
endmacro()
function(wasm_port_target tgt)
  if(NOT TARGET ${tgt})
    add_library(${tgt} INTERFACE IMPORTED GLOBAL)
    set_target_properties(${tgt} PROPERTIES
      INTERFACE_COMPILE_OPTIONS "${ARGN}"
      INTERFACE_LINK_OPTIONS "${ARGN}")
  endif()
endfunction()
