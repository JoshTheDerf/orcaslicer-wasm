# Source me. Shared toolchain + flag definitions for every WASM slicer build.
_WASM_DEPS_HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if ! command -v emcc >/dev/null 2>&1 || [[ "${EMSDK:-}" != "$_WASM_DEPS_HERE/emsdk" ]]; then
  # shellcheck disable=SC1091
  EMSDK_QUIET=1 source "$_WASM_DEPS_HERE/emsdk/emsdk_env.sh" >/dev/null
fi

export WASM_DEPS_ROOT="$_WASM_DEPS_HERE"

# Threading model. Default: single-threaded (prefix install/). WASM_THREADS=1
# selects the Emscripten-pthreads variant: every archive compiled with -pthread
# (atomics + bulk-memory, shared memory) into a SEPARATE prefix install-mt/.
# The two prefixes must never be mixed in one link.
if [[ "${WASM_THREADS:-0}" == "1" ]]; then
  export WASM_DEPS_PREFIX="$_WASM_DEPS_HERE/install-mt"
  export WASM_THREAD_FLAGS="-pthread"
else
  export WASM_DEPS_PREFIX="$_WASM_DEPS_HERE/install"
  export WASM_THREAD_FLAGS=""
fi

# Exception + longjmp model. Must be identical for every object and at link.
export WASM_EH_FLAGS="-fwasm-exceptions -sSUPPORT_LONGJMP=wasm"
export WASM_CFLAGS="-O3 $WASM_EH_FLAGS${WASM_THREAD_FLAGS:+ $WASM_THREAD_FLAGS}"
export WASM_CXXFLAGS="-O3 $WASM_EH_FLAGS${WASM_THREAD_FLAGS:+ $WASM_THREAD_FLAGS}"

# Local build tools (no system cmake/ninja/m4 on this host).
export PATH="$_WASM_DEPS_HERE/cmake/bin:$_WASM_DEPS_HERE/tools/bin:$PATH"
export CCACHE_DIR="$_WASM_DEPS_HERE/ccache-cache"
export CCACHE_MAXSIZE=20G
