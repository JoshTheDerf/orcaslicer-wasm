#pragma once

#include <functional>

namespace oneapi { namespace tbb {

class task_group {
public:
    task_group() = default;

    template <typename Func>
    void run(Func&& func) {
        func();
    }

    template <typename Func>
    void run_and_wait(Func&& func) {
        func();
    }

    void wait() {}
    void cancel() {}
    bool is_canceling() const { return false; }
};

// Sequential shim: no task group is ever cancelled (cancel() above is a no-op).
inline bool is_current_task_group_canceling() { return false; }

// (oneTBB declares this in task_arena.h; its only user here, ParallelResolve.hpp,
// includes both.) Nothing else can run on one thread, so isolation is just the call.
namespace this_task_arena {
template <typename F> auto isolate(const F& f) -> decltype(f()) { return f(); }
} // namespace this_task_arena

}} // namespace oneapi::tbb

#ifndef ORCA_WASM_TBB_ALIAS_DEFINED
#define ORCA_WASM_TBB_ALIAS_DEFINED
namespace tbb = oneapi::tbb;
#endif
