// pthreads build only: Emscripten's pthreads libc declares pthread_setname_np but has
// no definition (the single-threaded libc stubs it). libslic3r/Thread.cpp names TBB
// worker threads with it; forward to Emscripten's own thread naming (shown in
// debuggers / thread profilers). Thread names are cosmetic.
#define _GNU_SOURCE
#include <pthread.h>
#include <emscripten/threading.h>

int pthread_setname_np(pthread_t thread, const char *name)
{
    emscripten_set_thread_name(thread, name);
    return 0;
}
