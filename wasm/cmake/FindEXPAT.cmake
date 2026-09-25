# Use orca's bundled deps_src/expat (real library, built with the engine's
# flags by deps_src/CMakeLists.txt as target `expat`). Report it as found so
# the root CMakeLists doesn't define a second `expat` target.
set(EXPAT_FOUND TRUE)
set(EXPAT_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../orca/deps_src/expat")
set(EXPAT_INCLUDE_DIR "${EXPAT_INCLUDE_DIRS}")
set(EXPAT_LIBRARIES expat)
