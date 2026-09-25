# Real CGAL (header-only) from wasm-deps/cgal-<ver>, with GMP/MPFR from the prefix.
set(CGAL_DIR "${WASM_CGAL_PREFIX}/lib/cmake/CGAL")
set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)
find_package(CGAL CONFIG REQUIRED PATHS "${WASM_CGAL_PREFIX}" NO_DEFAULT_PATH)
