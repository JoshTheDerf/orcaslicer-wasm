if(NOT TARGET NLopt::nlopt)
  add_library(NLopt::nlopt STATIC IMPORTED GLOBAL)
  set_target_properties(NLopt::nlopt PROPERTIES
    IMPORTED_LOCATION "${WASM_DEPS_PREFIX}/lib/libnlopt.a"
    INTERFACE_INCLUDE_DIRECTORIES "${WASM_DEPS_PREFIX}/include")
endif()
set(NLopt_FOUND TRUE)
set(NLOPT_FOUND TRUE)
set(NLopt_VERSION "2.9.1")
set(NLopt_LIBS NLopt::nlopt)
set(NLOPT_LIBRARIES NLopt::nlopt)
set(NLopt_INCLUDE_DIR "${WASM_DEPS_PREFIX}/include")
