#pragma once

#include <cstddef>
#include <cstdlib>
#include <limits>
#include <new>
#include <utility>

// tbbmalloc's C entry points (Clipper2's node allocator uses them directly).
inline void* scalable_malloc(std::size_t size) { return std::malloc(size); }
inline void  scalable_free(void* p) { std::free(p); }

namespace oneapi { namespace tbb {

template<class T>
struct scalable_allocator {
    using value_type = T;
    using pointer = T*;
    using const_pointer = const T*;
    using reference = T&;
    using const_reference = const T&;
    using size_type = std::size_t;
    using difference_type = std::ptrdiff_t;

    scalable_allocator() noexcept = default;
    scalable_allocator(const scalable_allocator&) noexcept = default;

    template<class U>
    scalable_allocator(const scalable_allocator<U>&) noexcept {}

    template<class U>
    struct rebind {
        using other = scalable_allocator<U>;
    };

    T* allocate(std::size_t n) {
        if (n > max_size()) {
            throw std::bad_alloc();
        }
        return static_cast<T*>(::operator new(n * sizeof(T)));
    }

    void deallocate(T* p, std::size_t) noexcept {
        ::operator delete(p);
    }

    size_type max_size() const noexcept {
        return std::numeric_limits<size_type>::max() / sizeof(T);
    }

    template<class U, class... Args>
    void construct(U* p, Args&&... args) {
        new(p) U(std::forward<Args>(args)...);
    }

    template<class U>
    void destroy(U* p) {
        p->~U();
    }
};

template<class T, class U>
constexpr bool operator==(const scalable_allocator<T>&, const scalable_allocator<U>&) noexcept {
    return true;
}

template<class T, class U>
constexpr bool operator!=(const scalable_allocator<T>&, const scalable_allocator<U>&) noexcept {
    return false;
}

}} // namespace oneapi::tbb

#ifndef ORCA_WASM_TBB_ALIAS_DEFINED
#define ORCA_WASM_TBB_ALIAS_DEFINED
namespace tbb = oneapi::tbb;
#endif