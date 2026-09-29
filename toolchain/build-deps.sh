#!/usr/bin/env bash
# Shared, consistently-flagged WebAssembly dependency prefix for the
# orcaslicer-wasm and preflight-wasm ports.
#
# Every archive in $PREFIX is compiled with the SAME exception / longjmp /
# threading model as the slicer engines that link it:
#
#   -fwasm-exceptions -sSUPPORT_LONGJMP=wasm   (native Wasm EH, no JS emulation)
#   single-threaded                            (no -pthread, Boost threading=single)
#
# WASM_THREADS=1 builds the Emscripten-pthreads variant instead: the same
# libraries plus Boost.Thread and oneTBB, every object compiled with -pthread,
# into install-mt/ (sources/logs under src-mt/ + logs-mt/). The single-thread
# prefix is never touched by an MT build and vice versa.
#
# Mixing models (e.g. a -pthread Boost with a non-pthread engine, or
# JS-emulated exceptions in one archive and none in another) is what produced
# the signature-mismatch LinkErrors and "null function" / out-of-bounds traps
# in the previous builds, which were papered over with
# -sEMULATE_FUNCTION_POINTER_CASTS. Keep this file the single source of truth.
#
# Usage:  bash build-deps.sh [boost|gmp|mpfr|cgal|eigen|...|preflight-extras|all]
#         WASM_THREADS=1 bash build-deps.sh all-mt      # pthreads prefix (orca MT engine)
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$HERE/env.sh"

PREFIX="$WASM_DEPS_PREFIX"
DL="$HERE/downloads"
NPROC="${NPROC:-$(nproc)}"
if [[ "${WASM_THREADS:-0}" == "1" ]]; then
  MT=1; SRC="$HERE/src-mt"; LOGS="$HERE/logs-mt"
else
  MT=0; SRC="$HERE/src"; LOGS="$HERE"
fi
mkdir -p "$PREFIX" "$SRC" "$DL" "$LOGS"

fetch() { # name url [sha256]
  local out="$DL/$1"
  [[ -f "$out" ]] && return
  echo "⬇  $1"
  curl -L --fail --retry 3 --connect-timeout 20 -o "$out.partial" "$2"
  mv "$out.partial" "$out"
}

build_boost() {
  local ver=1.84.0 dir="boost_1_84_0"
  [[ -f "$PREFIX/lib/libboost_log.a" ]] && { echo "boost: up to date"; return; }
  # Boost is built in-tree (bin.v2); MT uses its own extracted tree (src-mt).
  fetch "$dir.tar.gz" "https://archives.boost.io/release/$ver/source/$dir.tar.gz"
  [[ -d "$SRC/$dir" ]] || tar xf "$DL/$dir.tar.gz" -C "$SRC"
  pushd "$SRC/$dir" >/dev/null
  [[ -x ./b2 ]] || ./bootstrap.sh >/dev/null
  cat > user-config.jam <<JAM
using clang : emscripten : $EMSDK/upstream/emscripten/em++ :
  <archiver>"$EMSDK/upstream/emscripten/emar"
  <ranlib>"$EMSDK/upstream/emscripten/emranlib" ;
JAM
  # Boost.Log sink backends that need OS facilities are compiled out; the
  # same defines are re-applied by the engines (BOOST_LOG_* in env.sh).
  # MT: threading=multi, real Boost.Thread, thread-safe Boost.Log.
  local threading=single extra=(define=BOOST_LOG_NO_THREADS)
  if [[ $MT == 1 ]]; then threading=multi; extra=(--with-thread linkflags=-pthread); fi
  ./b2 --user-config=user-config.jam \
    toolset=clang-emscripten \
    link=static runtime-link=static threading=$threading variant=release \
    cxxflags="$WASM_CXXFLAGS -sUSE_ZLIB=1 -Wno-unused-command-line-argument" \
    cflags="$WASM_CFLAGS -sUSE_ZLIB=1" \
    define=BOOST_LOG_WITHOUT_SYSLOG define=BOOST_LOG_WITHOUT_EVENT_LOG \
    define=BOOST_LOG_WITHOUT_DEBUG_OUTPUT define=BOOST_LOG_WITHOUT_IPC \
    define=BOOST_FILESYSTEM_NO_CXX20_ATOMIC_REF "${extra[@]}" \
    -sNO_BZIP2=1 -sNO_LZMA=1 -sNO_ZSTD=1 \
    boost.locale.icu=off boost.locale.iconv=off boost.locale.posix=on \
    --with-system --with-filesystem --with-regex --with-chrono --with-atomic \
    --with-date_time --with-iostreams --with-program_options --with-log \
    --with-nowide --with-locale \
    -j"$NPROC" install --prefix="$PREFIX" > "$LOGS/boost-build.log" 2>&1 \
    || { tail -40 "$LOGS/boost-build.log"; exit 1; }
  popd >/dev/null
  echo "boost: installed"
}

build_gmp() {
  local ver=6.3.0
  [[ -f "$PREFIX/lib/libgmpxx.a" ]] && { echo "gmp: up to date"; return; }
  fetch "gmp-$ver.tar.xz" "https://ftp.gnu.org/gnu/gmp/gmp-$ver.tar.xz"
  rm -rf "$SRC/gmp-$ver"; tar xf "$DL/gmp-$ver.tar.xz" -C "$SRC"
  pushd "$SRC/gmp-$ver" >/dev/null
  CC_FOR_BUILD=gcc CXX_FOR_BUILD=g++ CFLAGS="$WASM_CFLAGS" CXXFLAGS="$WASM_CXXFLAGS" \
    emconfigure ./configure --host=wasm32-unknown-emscripten --build="$(./config.guess)" \
      --disable-shared --enable-static --disable-assembly --enable-cxx \
      --prefix="$PREFIX" > "$LOGS/gmp-configure.log" 2>&1 \
    || { tail -30 "$LOGS/gmp-configure.log"; exit 1; }
  emmake make -j"$NPROC" > "$LOGS/gmp-build.log" 2>&1 && emmake make install >> "$LOGS/gmp-build.log" 2>&1 \
    || { tail -30 "$LOGS/gmp-build.log"; exit 1; }
  popd >/dev/null
  echo "gmp: installed"
}

build_mpfr() {
  local ver=4.2.2
  [[ -f "$PREFIX/lib/libmpfr.a" ]] && { echo "mpfr: up to date"; return; }
  fetch "mpfr-$ver.tar.xz" "https://ftp.gnu.org/gnu/mpfr/mpfr-$ver.tar.xz"
  rm -rf "$SRC/mpfr-$ver"; tar xf "$DL/mpfr-$ver.tar.xz" -C "$SRC"
  pushd "$SRC/mpfr-$ver" >/dev/null
  CFLAGS="$WASM_CFLAGS" emconfigure ./configure --host=wasm32-unknown-emscripten --build="$(./config.guess)" \
      --disable-shared --enable-static --with-gmp="$PREFIX" --prefix="$PREFIX" \
      $([[ $MT == 1 ]] && echo --enable-thread-safe || echo --disable-thread-safe) > "$LOGS/mpfr-configure.log" 2>&1 \
    || { tail -30 "$LOGS/mpfr-configure.log"; exit 1; }
  emmake make -j"$NPROC" > "$LOGS/mpfr-build.log" 2>&1 && emmake make install >> "$LOGS/mpfr-build.log" 2>&1 \
    || { tail -30 "$LOGS/mpfr-build.log"; exit 1; }
  popd >/dev/null
  echo "mpfr: installed"
}

# CGAL is header-only. Orca pins 5.6.3; preFlight needs 6.1 (AABB_traits_3,
# std::optional property maps). Each gets its own prefix so the engines can't
# pick up the wrong one.
install_cgal() { # version subdir
  local ver="$1" dest="$HERE/cgal-$1"
  [[ -f "$dest/include/CGAL/version.h" ]] && { echo "cgal $ver: up to date"; return; }
  fetch "CGAL-$ver.tar.xz" "https://github.com/CGAL/cgal/releases/download/v$ver/CGAL-$ver.tar.xz"
  rm -rf "$SRC/CGAL-$ver"; tar xf "$DL/CGAL-$ver.tar.xz" -C "$SRC"
  cmake -S "$SRC/CGAL-$ver" -B "$SRC/CGAL-$ver/build" -DCMAKE_INSTALL_PREFIX="$dest" \
    -DCGAL_HEADER_ONLY=ON -DWITH_CGAL_Qt5=OFF -DWITH_CGAL_ImageIO=OFF > /dev/null
  cmake --install "$SRC/CGAL-$ver/build" > /dev/null
  echo "cgal $ver: installed"
}

install_eigen() {
  local ver=5.0.1
  [[ -f "$PREFIX/include/eigen3/Eigen/Core" ]] && { echo "eigen: up to date"; return; }
  fetch "eigen-$ver.tar.gz" "https://gitlab.com/libeigen/eigen/-/archive/$ver/eigen-$ver.tar.gz"
  rm -rf "$SRC/eigen-$ver"; tar xf "$DL/eigen-$ver.tar.gz" -C "$SRC"
  cmake -S "$SRC/eigen-$ver" -B "$SRC/eigen-$ver/build" -DCMAKE_INSTALL_PREFIX="$PREFIX" \
    -DBUILD_TESTING=OFF -DEIGEN_BUILD_BLAS=OFF -DEIGEN_BUILD_LAPACK=OFF -DEIGEN_BUILD_DOC=OFF -DEIGEN_BUILD_PKGCONFIG=OFF > /dev/null
  cmake --install "$SRC/eigen-$ver/build" > /dev/null
  echo "eigen: installed"
}

em_cmake_lib() { # name srcdir [cmake args...]
  local name="$1" src="$2"; shift 2
  emcmake cmake -S "$src" -B "$src/build-wasm" -G Ninja -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$PREFIX" -DBUILD_SHARED_LIBS=OFF \
    -DCMAKE_C_FLAGS="$WASM_CFLAGS" -DCMAKE_CXX_FLAGS="$WASM_CXXFLAGS" \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 "$@" > "$LOGS/$name-configure.log" 2>&1 \
    || { tail -30 "$LOGS/$name-configure.log"; exit 1; }
  cmake --build "$src/build-wasm" -j"$NPROC" > "$LOGS/$name-build.log" 2>&1 \
    && cmake --install "$src/build-wasm" >> "$LOGS/$name-build.log" 2>&1 \
    || { tail -30 "$LOGS/$name-build.log"; exit 1; }
  echo "$name: installed"
}

# Orca's fuzzy-skin noise types need the real libnoise (a stub silently
# produced flat "noise").
build_libnoise() {
  [[ -f "$PREFIX/lib/liblibnoise_static.a" ]] && { echo "libnoise: up to date"; return; }
  fetch "libnoise-1.0.zip" "https://github.com/SoftFever/Orca-deps-libnoise/archive/refs/tags/1.0.zip"
  rm -rf "$SRC/Orca-deps-libnoise-1.0"
  python3 -c "import zipfile,sys;zipfile.ZipFile(sys.argv[1]).extractall(sys.argv[2])" "$DL/libnoise-1.0.zip" "$SRC"
  em_cmake_lib libnoise "$SRC/Orca-deps-libnoise-1.0" -DBUILD_SHARED_LIBS=OFF -DLIBNOISE_BUILD_SHARED=OFF
}

build_nlopt() {
  [[ -f "$PREFIX/lib/libnlopt.a" ]] && { echo "nlopt: up to date"; return; }
  fetch "nlopt-2.9.1.tar.gz" "https://github.com/stevengj/nlopt/archive/v2.9.1.tar.gz"
  rm -rf "$SRC/nlopt-2.9.1"; tar xf "$DL/nlopt-2.9.1.tar.gz" -C "$SRC"
  em_cmake_lib nlopt "$SRC/nlopt-2.9.1" -DNLOPT_PYTHON=OFF -DNLOPT_OCTAVE=OFF -DNLOPT_MATLAB=OFF \
    -DNLOPT_GUILE=OFF -DNLOPT_SWIG=OFF -DNLOPT_TESTS=OFF -DNLOPT_FORTRAN=OFF
}

install_cereal() {
  [[ -f "$PREFIX/include/cereal/cereal.hpp" ]] && { echo "cereal: up to date"; return; }
  fetch "cereal-1.3.2.tar.gz" "https://github.com/USCiLab/cereal/archive/refs/tags/v1.3.2.tar.gz"
  rm -rf "$SRC/cereal-1.3.2"; tar xf "$DL/cereal-1.3.2.tar.gz" -C "$SRC"
  cmake -S "$SRC/cereal-1.3.2" -B "$SRC/cereal-1.3.2/build" -DCMAKE_INSTALL_PREFIX="$PREFIX" \
    -DJUST_INSTALL_CEREAL=ON > /dev/null && cmake --install "$SRC/cereal-1.3.2/build" > /dev/null
  echo "cereal: installed"
}

# ---- preFlight extras (PrusaSlicer-family deps; same flags as everything else) ----
unzip_to() { python3 -c "import zipfile,sys;zipfile.ZipFile(sys.argv[1]).extractall(sys.argv[2])" "$1" "$2"; }

# Qhull: TriangleMesh::convex_hull_3d (ModelVolume convex hulls). Same tag preFlight pins.
build_qhull() {
  [[ -f "$PREFIX/lib/libqhullstatic_r.a" ]] && { echo "qhull: up to date"; return; }
  fetch "qhull-8.1-alpha3.zip" "https://github.com/qhull/qhull/archive/refs/tags/v8.1-alpha3.zip"
  rm -rf "$SRC/qhull-8.1-alpha3"; unzip_to "$DL/qhull-8.1-alpha3.zip" "$SRC"
  em_cmake_lib qhull "$SRC/qhull-8.1-alpha3" -DBUILD_APPLICATIONS=OFF -DBUILD_STATIC_LIBS=ON \
    -DQHULL_ENABLE_TESTING=OFF -DINCLUDE_INSTALL_DIR=include -DLINK_APPS_SHARED=OFF
}

build_expat() {
  [[ -f "$PREFIX/lib/libexpat.a" ]] && { echo "expat: up to date"; return; }
  fetch "expat-R_2_6_4.zip" "https://github.com/libexpat/libexpat/archive/refs/tags/R_2_6_4.zip"
  rm -rf "$SRC/libexpat-R_2_6_4"; unzip_to "$DL/expat-R_2_6_4.zip" "$SRC"
  em_cmake_lib expat "$SRC/libexpat-R_2_6_4/expat" -DEXPAT_BUILD_TOOLS=OFF -DEXPAT_BUILD_EXAMPLES=OFF \
    -DEXPAT_BUILD_TESTS=OFF -DEXPAT_BUILD_DOCS=OFF -DEXPAT_BUILD_PKGCONFIG=OFF -DEXPAT_SHARED_LIBS=OFF
}

# heatshrink has no build system; use the CMakeLists PrusaSlicer/preFlight ship for it.
build_heatshrink() {
  [[ -f "$PREFIX/lib/libheatshrink_dynalloc.a" ]] && { echo "heatshrink: up to date"; return; }
  local pf="${PREFLIGHT_SRC:-$HERE/../preflight}/deps/+heatshrink"
  [[ -f "$pf/CMakeLists.txt" ]] || { echo "heatshrink: need preFlight checkout at $pf" >&2; exit 1; }
  fetch "heatshrink-0.4.1.zip" "https://github.com/atomicobject/heatshrink/archive/refs/tags/v0.4.1.zip"
  rm -rf "$SRC/heatshrink-0.4.1"; unzip_to "$DL/heatshrink-0.4.1.zip" "$SRC"
  cp "$pf/CMakeLists.txt" "$pf/Config.cmake.in" "$SRC/heatshrink-0.4.1/"
  em_cmake_lib heatshrink "$SRC/heatshrink-0.4.1"
}

install_json() {
  [[ -f "$PREFIX/include/nlohmann/json.hpp" ]] && { echo "nlohmann_json: up to date"; return; }
  fetch "json-3.12.0.tar.gz" "https://github.com/nlohmann/json/archive/refs/tags/v3.12.0.tar.gz"
  rm -rf "$SRC/json-3.12.0"; tar xf "$DL/json-3.12.0.tar.gz" -C "$SRC"
  cmake -S "$SRC/json-3.12.0" -B "$SRC/json-3.12.0/build" -DCMAKE_INSTALL_PREFIX="$PREFIX" \
    -DJSON_BuildTests=OFF -DJSON_ImplicitConversions=OFF -DJSON_Install=ON > /dev/null
  cmake --install "$SRC/json-3.12.0/build" > /dev/null
  echo "nlohmann_json: installed"
}

# fltk fork of nanosvg (nsvgRasterizeXY), pinned to the commit preFlight uses. Header-only here.
install_nanosvg() {
  [[ -f "$PREFIX/include/nanosvg/nanosvg.h" ]] && { echo "nanosvg: up to date"; return; }
  local rev=abcd277ea45e9098bed752cf9c6875b533c0892f
  fetch "nanosvg-$rev.zip" "https://github.com/fltk/nanosvg/archive/$rev.zip"
  rm -rf "$SRC/nanosvg-$rev"; unzip_to "$DL/nanosvg-$rev.zip" "$SRC"
  mkdir -p "$PREFIX/include/nanosvg"
  cp "$SRC/nanosvg-$rev/src/nanosvg.h" "$SRC/nanosvg-$rev/src/nanosvgrast.h" "$PREFIX/include/nanosvg/"
  echo "nanosvg: installed"
}

# oneTBB (MT variant only; the single-thread engine uses a sequential header
# shim). Orca 2.4.2 pins 2021.5.0 on desktop; 2022.x is API/ABI-compatible with
# that code (TBB_VERSION_MAJOR >= 2021 paths) and is the first line with
# upstream Emscripten+pthreads support.
build_onetbb() {
  local ver=2022.3.0
  [[ $MT == 1 ]] || { echo "onetbb: only built for WASM_THREADS=1" >&2; exit 1; }
  [[ -f "$PREFIX/lib/libtbb.a" ]] && { echo "onetbb: up to date"; return; }
  fetch "oneTBB-$ver.tar.gz" "https://github.com/uxlfoundation/oneTBB/archive/refs/tags/v$ver.tar.gz"
  rm -rf "$SRC/oneTBB-$ver"; tar xf "$DL/oneTBB-$ver.tar.gz" -C "$SRC"
  # Upstream's Emscripten profile (cmake/compilers/Clang.cmake) adds -fexceptions
  # (JS-emulated EH: must not be mixed with our -fwasm-exceptions) and gives
  # every TBB worker a 64 KB stack (Arachne/CGAL recursion needs far more).
  # Worker stacks: 16 MB, bounds-checked by STACK_OVERFLOW_CHECK=2 at link.
  sed -i -e 's/ -fexceptions)$/)/' \
         -e 's/set(TBB_EMSCRIPTEN_STACK_SIZE 65536)/set(TBB_EMSCRIPTEN_STACK_SIZE 16777216)/' \
    "$SRC/oneTBB-$ver/cmake/compilers/Clang.cmake"
  grep -q 'TBB_EMSCRIPTEN_STACK_SIZE 16777216' "$SRC/oneTBB-$ver/cmake/compilers/Clang.cmake" \
    && ! grep -q 'COMPILE_FLAGS} -fexceptions' "$SRC/oneTBB-$ver/cmake/compilers/Clang.cmake" \
    || { echo "onetbb: Clang.cmake patch did not apply" >&2; exit 1; }
  em_cmake_lib onetbb "$SRC/oneTBB-$ver" -DTBB_TEST=OFF -DTBB_EXAMPLES=OFF -DTBB_STRICT=OFF \
    -DTBB_DISABLE_HWLOC_AUTOMATIC_SEARCH=ON -DTBBMALLOC_BUILD=ON -DTBBMALLOC_PROXY_BUILD=OFF \
    -DCMAKE_EXE_LINKER_FLAGS="-pthread"
}

what="${1:-all}"
case "$what" in
  boost) build_boost ;;
  gmp)   build_gmp ;;
  mpfr)  build_gmp; build_mpfr ;;
  cgal)  install_cgal 5.6.3; install_cgal 6.1 ;;
  eigen) install_eigen ;;
  libnoise) build_libnoise ;;
  nlopt) build_nlopt ;;
  cereal) install_cereal ;;
  qhull) build_qhull ;;
  expat) build_expat ;;
  heatshrink) build_heatshrink ;;
  json) install_json ;;
  nanosvg) install_nanosvg ;;
  onetbb) build_onetbb ;;
  preflight-extras) build_qhull; build_expat; build_heatshrink; install_json; install_nanosvg ;;
  all)   install_eigen; install_cgal 5.6.3; install_cgal 6.1; install_cereal; build_gmp; build_mpfr;
         build_libnoise; build_nlopt; build_boost ;;
  all-mt) [[ $MT == 1 ]] || { echo "all-mt needs WASM_THREADS=1" >&2; exit 1; }
         install_eigen; install_cereal; build_gmp; build_mpfr; build_libnoise; build_nlopt;
         build_onetbb; build_boost ;;
  *) echo "unknown target $what" >&2; exit 1 ;;
esac
