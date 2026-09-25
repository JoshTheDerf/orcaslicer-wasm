#pragma once

#include <atomic>

namespace oneapi { namespace tbb {

class spin_mutex {
public:
    spin_mutex() : m_flag(ATOMIC_FLAG_INIT) {}

    // Single-threaded build: nobody else can release the lock, so spinning on
    // a held lock would hang forever. Re-locking is a logic error; abort loudly.
    void lock() {
        if (m_flag.test_and_set(std::memory_order_acquire)) __builtin_trap();
    }
    bool try_lock() { return !m_flag.test_and_set(std::memory_order_acquire); }

    void unlock() {
        m_flag.clear(std::memory_order_release);
    }

    class scoped_lock {
    public:
        scoped_lock() : m_mutex(nullptr) {}
        explicit scoped_lock(spin_mutex& mutex) { acquire(mutex); }
        scoped_lock(const scoped_lock&) = delete;
        scoped_lock& operator=(const scoped_lock&) = delete;
        ~scoped_lock() { release(); }

        void acquire(spin_mutex& mutex) {
            release();
            m_mutex = &mutex;
            m_mutex->lock();
        }

        void release() {
            if (m_mutex) {
                m_mutex->unlock();
                m_mutex = nullptr;
            }
        }

    private:
        spin_mutex* m_mutex;
    };

private:
    std::atomic_flag m_flag;
};

}} // namespace oneapi::tbb

#ifndef ORCA_WASM_TBB_ALIAS_DEFINED
#define ORCA_WASM_TBB_ALIAS_DEFINED
namespace tbb = oneapi::tbb;
#endif
