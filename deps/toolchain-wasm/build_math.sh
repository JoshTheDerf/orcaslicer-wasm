#!/usr/bin/env bash
# Build GMP, MPFR, and CGAL for the Emscripten toolchain used by the WASM slicer.
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
REPO_DIR=$(cd "${ROOT_DIR}/.." && pwd)
PREFIX="${ROOT_DIR}/toolchain-wasm/install"
BUILD_DIR="${ROOT_DIR}/toolchain-wasm/build"
SRC_DIR="${ROOT_DIR}/toolchain-wasm/src"
DL_DIR="${ROOT_DIR}/toolchain-wasm/downloads"
BOOST_PREFIX="${ROOT_DIR}/boost-wasm/install"

mkdir -p "${PREFIX}" "${BUILD_DIR}" "${SRC_DIR}" "${DL_DIR}"

# Resolve emscripten helper commands for Windows (.bat) vs POSIX
resolve_em_tools() {
  if command -v emcc >/dev/null 2>&1; then
    local emcc_path
    emcc_path="$(command -v emcc)"
    local em_dir
    em_dir="$(cd "$(dirname "${emcc_path}")" && pwd)"
    # On Windows Git Bash emcc may be emcc.bat; prefer .bat siblings when present
    if [[ -f "${em_dir}/emconfigure.bat" ]]; then
      EMCONFIGURE="${em_dir}/emconfigure.bat"
    else
      EMCONFIGURE=emconfigure
    fi
    if [[ -f "${em_dir}/emmake.bat" ]]; then
      EMMAKE="${em_dir}/emmake.bat"
    else
      EMMAKE=emmake
    fi
    if [[ -f "${em_dir}/emcmake.bat" ]]; then
      EMCMAKE="${em_dir}/emcmake.bat"
    else
      EMCMAKE=emcmake
    fi
    EM_TOOLCHAIN="${em_dir}/cmake/Modules/Platform/Emscripten.cmake"
  else
    EMCONFIGURE=emconfigure
    EMMAKE=emmake
    EMCMAKE=emcmake
    EM_TOOLCHAIN=""
  fi
}
resolve_em_tools

# Load Emscripten environment. Prefer a native (WSL/Linux) emsdk over repo-local Windows emsdk when under WSL.
if grep -qi microsoft /proc/version 2>/dev/null; then
  # Running under WSL
  if [[ -f "$HOME/emsdk/emsdk_env.sh" ]]; then
    # shellcheck disable=SC1091
    source "$HOME/emsdk/emsdk_env.sh"
  elif [[ -f "/opt/emsdk/emsdk_env.sh" ]]; then
    # shellcheck disable=SC1091
    source "/opt/emsdk/emsdk_env.sh"
  elif command -v emsdk_env.sh >/dev/null 2>&1; then
    # shellcheck disable=SC1091
    source "$(command -v emsdk_env.sh)"
  elif [[ -f "${REPO_DIR}/wasm/toolchain/emsdk.env" ]]; then
    # Fallback to repo-local env as last resort
    # shellcheck disable=SC1090
    source "${REPO_DIR}/wasm/toolchain/emsdk.env"
  fi
else
  # Non-WSL: allow repo-local env
  if [[ -f "${REPO_DIR}/wasm/toolchain/emsdk.env" ]]; then
    # shellcheck disable=SC1090
    source "${REPO_DIR}/wasm/toolchain/emsdk.env"
  elif command -v emsdk_env.sh >/dev/null 2>&1; then
    # shellcheck disable=SC1091
    source "$(command -v emsdk_env.sh)"
  fi
fi

if ! command -v emcc >/dev/null 2>&1; then
  echo "ERROR: Emscripten environment not found (missing emcc)" >&2
  exit 1
fi

# Re-resolve after sourcing env
resolve_em_tools

NPROC=${NPROC:-$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)}

fetch() {
  local name="$1"
  shift
  local dest="${DL_DIR}/${name}"
  if [[ -f "${dest}" ]]; then
    return
  fi

  mkdir -p "${DL_DIR}"
  local url success=0
  for url in "$@"; do
    if [[ -z "${url}" ]]; then
      continue
    fi
  echo "Downloading ${name} from ${url}"
    if curl -L --fail --retry 3 --connect-timeout 20 --retry-connrefused \
        --output "${dest}.partial" "${url}"; then
      mv "${dest}.partial" "${dest}"
      success=1
      break
    else
  echo "WARN: Failed to download ${name} from ${url}" >&2
      rm -f "${dest}.partial"
    fi
  done

  if [[ ${success} -ne 1 ]]; then
  echo "ERROR: Unable to download ${name} from provided mirrors" >&2
    exit 1
  fi
}

extract() {
  local archive="$1" target="$2"
  if [[ ! -d "${target}" ]]; then
  echo "Extracting $(basename "${archive}")"
    tar xf "${archive}" -C "${SRC_DIR}"
  fi
}

build_gmp() {
  local version="6.2.1"
  local tarball="gmp-${version}.tar.xz"
  local src="${SRC_DIR}/gmp-${version}"
  fetch "${tarball}" \
    "https://gmplib.org/download/gmp/${tarball}" \
    "https://ftp.gnu.org/gnu/gmp/${tarball}" \
    "https://mirrors.kernel.org/gnu/gmp/${tarball}"
  extract "${DL_DIR}/${tarball}" "${src}"
  if [[ ! -f "${PREFIX}/lib/libgmp.a" || ! -f "${PREFIX}/lib/libgmpxx.a" ]]; then
    pushd "${src}" >/dev/null
    "${EMMAKE}" make distclean >/dev/null 2>&1 || true
    if command -v "${EMCONFIGURE}" >/dev/null 2>&1 && [[ "${EMCONFIGURE}" != *".bat" ]]; then
      CC_FOR_BUILD=gcc CXX_FOR_BUILD=g++ \
      "${EMCONFIGURE}" ./configure \
        --host=wasm32-unknown-emscripten \
        --disable-shared \
        --disable-assembly \
        --with-pic \
        --enable-cxx \
        --prefix="${PREFIX}"
    else
      CC_FOR_BUILD=gcc CXX_FOR_BUILD=g++ \
      ac_cv_prog_cc_works=yes ac_cv_prog_cxx_works=yes ac_cv_c_bigendian=no \
      CC=emcc CXX=em++ AR=emar RANLIB=emranlib NM=llvm-nm \
      ./configure \
        --host=wasm32-unknown-emscripten \
        --disable-shared \
        --disable-assembly \
        --with-pic \
        --enable-cxx \
        --prefix="${PREFIX}"
    fi
    "${EMMAKE}" make -j "${NPROC}"
    "${EMMAKE}" make install
    popd >/dev/null
  fi
}

build_mpfr() {
  local version="4.2.1"
  local tarball="mpfr-${version}.tar.xz"
  local src="${SRC_DIR}/mpfr-${version}"
  fetch "${tarball}" \
    "https://www.mpfr.org/mpfr-${version}/${tarball}" \
    "https://ftp.gnu.org/gnu/mpfr/${tarball}" \
    "https://mirrors.kernel.org/gnu/mpfr/${tarball}"
  extract "${DL_DIR}/${tarball}" "${src}"
  if [[ ! -f "${PREFIX}/lib/libmpfr.a" ]]; then
    pushd "${src}" >/dev/null
    CPPFLAGS="-I${PREFIX}/include" LDFLAGS="-L${PREFIX}/lib" \
    CC_FOR_BUILD=gcc CXX_FOR_BUILD=g++ \
    ac_cv_prog_cc_works=yes ac_cv_prog_cxx_works=yes ac_cv_c_bigendian=no \
    CC=emcc CXX=em++ AR=emar RANLIB=emranlib NM=llvm-nm \
    ./configure \
      --host=wasm32-unknown-emscripten \
      --disable-shared \
      --with-gmp="${PREFIX}" \
      --prefix="${PREFIX}"
    "${EMMAKE}" make -j "${NPROC}"
    "${EMMAKE}" make install
    popd >/dev/null
  fi
}

build_cgal() {
  # Match the CGAL version that upstream OrcaSlicer pins in
  # orca/deps/CGAL/CGAL.cmake (v5.6.3). Bumping past 5.6.x breaks
  # orca's libslic3r_cgal code (CGAL 6.0 removed `iter.base()` on
  # Halfedge_around_*_iterator; CGAL 5.5 lacked `parameters::default_values`).
  local version="5.6.3"
  local tarball="CGAL-${version}.tar.xz"
  local src="${SRC_DIR}/CGAL-${version}"
  fetch "${tarball}" \
    "https://github.com/CGAL/cgal/releases/download/v${version}/${tarball}" \
    "https://github.com/CGAL/cgal/archive/refs/tags/v${version}.tar.gz"
  extract "${DL_DIR}/${tarball}" "${src}"
  local build_path="${BUILD_DIR}/cgal"
  if [[ ! -f "${PREFIX}/lib/libCGAL.a" ]]; then
    rm -rf "${build_path}"
    cmake -S "${src}" -B "${build_path}" \
      -DCMAKE_TOOLCHAIN_FILE="${EM_TOOLCHAIN}" \
      -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_SHARED_LIBS=OFF \
      -DWITH_CGAL_ImageIO=OFF \
      -DWITH_CGAL_Qt5=OFF \
      -DGMP_INCLUDE_DIR="${PREFIX}/include" \
      -DGMP_LIBRARIES="${PREFIX}/lib/libgmp.a" \
      -DMPFR_INCLUDE_DIR="${PREFIX}/include" \
      -DMPFR_LIBRARIES="${PREFIX}/lib/libmpfr.a" \
      -DBoost_NO_SYSTEM_PATHS=ON \
      -DBOOST_ROOT="${BOOST_PREFIX}" \
      -DBoost_USE_STATIC_LIBS=ON
    cmake --build "${build_path}" -j "${NPROC}"
    cmake --install "${build_path}"
  fi
}

build_gmp
build_mpfr
build_cgal

echo "DONE: GMP/MPFR/CGAL staged under ${PREFIX}"
