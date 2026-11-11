#pragma once
// Sequential tbb::parallel_invoke for the single-threaded WASM build.
namespace oneapi { namespace tbb {
template<class... Fs> void parallel_invoke(Fs&&... fs) { (fs(), ...); }
}} // namespace oneapi::tbb

#ifndef ORCA_WASM_TBB_ALIAS_DEFINED
#define ORCA_WASM_TBB_ALIAS_DEFINED
namespace tbb = oneapi::tbb;
#endif
