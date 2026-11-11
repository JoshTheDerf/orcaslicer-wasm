#pragma once
// Sequential tbb::parallel_sort for the single-threaded WASM build.
#include <algorithm>
#include <functional>
#include <iterator>

namespace oneapi { namespace tbb {
template<class It> void parallel_sort(It first, It last) { std::sort(first, last); }
template<class It, class Cmp> void parallel_sort(It first, It last, const Cmp& cmp) { std::sort(first, last, cmp); }
template<class Range> void parallel_sort(Range&& r) { std::sort(std::begin(r), std::end(r)); }
template<class Range, class Cmp> void parallel_sort(Range&& r, const Cmp& cmp) { std::sort(std::begin(r), std::end(r), cmp); }
}} // namespace oneapi::tbb

#ifndef ORCA_WASM_TBB_ALIAS_DEFINED
#define ORCA_WASM_TBB_ALIAS_DEFINED
namespace tbb = oneapi::tbb;
#endif
