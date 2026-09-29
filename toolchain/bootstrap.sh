#!/usr/bin/env bash
# Install the pinned build tools next to this script (no system packages needed
# beyond git, curl, python3, a C compiler for m4, and xz/unzip):
#   emsdk 6.0.10 · CMake 3.31.8 · Ninja 1.13.1 · GNU m4 1.4.19
# Then build the dependency prefix:  bash build-deps.sh all
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"
EMSDK_VERSION="${EMSDK_VERSION:-6.0.10}"
CMAKE_VERSION="${CMAKE_VERSION:-3.31.8}"
NINJA_VERSION="${NINJA_VERSION:-1.13.1}"
M4_VERSION="${M4_VERSION:-1.4.19}"
mkdir -p downloads tools/bin

if [[ ! -x emsdk/emsdk ]]; then git clone --depth 1 https://github.com/emscripten-core/emsdk.git emsdk; fi
( cd emsdk && ./emsdk install "$EMSDK_VERSION" && ./emsdk activate "$EMSDK_VERSION" )

arch="$(uname -m)"; os="$(uname -s)"
if [[ ! -x cmake/bin/cmake ]]; then
  case "$os-$arch" in
    Linux-x86_64) c="cmake-$CMAKE_VERSION-linux-x86_64.tar.gz" ;;
    Linux-aarch64) c="cmake-$CMAKE_VERSION-linux-aarch64.tar.gz" ;;
    Darwin-*) c="cmake-$CMAKE_VERSION-macos-universal.tar.gz" ;;
    *) echo "unsupported host $os-$arch for the CMake download; put cmake >= 3.28 in cmake/bin" >&2; exit 1 ;;
  esac
  curl -fL -o downloads/cmake.tgz "https://github.com/Kitware/CMake/releases/download/v$CMAKE_VERSION/$c"
  mkdir -p cmake && tar -xzf downloads/cmake.tgz -C cmake --strip-components=1
  [[ "$os" == Darwin ]] && { mv cmake cmake.app && ln -s cmake.app/CMake.app/Contents cmake; }
fi
if [[ ! -x tools/bin/ninja ]]; then
  case "$os-$arch" in
    Linux-x86_64) n=ninja-linux.zip ;; Linux-aarch64) n=ninja-linux-aarch64.zip ;; Darwin-*) n=ninja-mac.zip ;;
  esac
  curl -fL -o downloads/ninja.zip "https://github.com/ninja-build/ninja/releases/download/v$NINJA_VERSION/$n"
  (cd tools/bin && unzip -o ../../downloads/ninja.zip >/dev/null && chmod +x ninja)
fi
if [[ ! -x tools/bin/m4 ]] && ! command -v m4 >/dev/null; then
  curl -fL -o downloads/m4.tar.xz "https://ftp.gnu.org/gnu/m4/m4-$M4_VERSION.tar.xz"
  rm -rf m4-src && mkdir m4-src && tar -xJf downloads/m4.tar.xz -C m4-src --strip-components=1
  (cd m4-src && ./configure --prefix="$HERE/tools" >/dev/null && make -j"$(nproc 2>/dev/null || echo 4)" >/dev/null && make install >/dev/null)
  rm -rf m4-src
fi
# Emscripten ports the engines use (headers + libs land in emsdk's cache).
emsdk/upstream/emscripten/embuilder build zlib libpng libjpeg freetype
echo "toolchain ready: $(emsdk/upstream/emscripten/emcc --version | head -1)"
