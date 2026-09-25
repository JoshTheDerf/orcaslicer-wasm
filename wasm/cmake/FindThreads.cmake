# Single-threaded WASM build: Threads::Threads is an empty target (no -pthread).
if(NOT TARGET Threads::Threads)
  add_library(Threads::Threads INTERFACE IMPORTED GLOBAL)
endif()
set(Threads_FOUND TRUE)
set(CMAKE_THREAD_LIBS_INIT "")
