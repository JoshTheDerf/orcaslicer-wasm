if(NOT TARGET noise::noise)
  add_library(noise::noise STATIC IMPORTED GLOBAL)
  set_target_properties(noise::noise PROPERTIES
    IMPORTED_LOCATION "${WASM_DEPS_PREFIX}/lib/liblibnoise_static.a"
    INTERFACE_INCLUDE_DIRECTORIES "${WASM_DEPS_PREFIX}/include")
endif()
set(libnoise_FOUND TRUE)
set(LIBNOISE_FOUND TRUE)
