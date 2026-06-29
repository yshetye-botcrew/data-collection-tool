#!/bin/bash
set -e
cd "$(dirname "$0")"

BUILD_TYPE="RelWithDebInfo"
JOBS="$(nproc)"
PREFIX="${BOTCREW_PREFIX:-/opt/botcrew}"
CLEAN=0

for arg in "$@"; do
    case $arg in
        Debug|Release|RelWithDebInfo|MinSizeRel) BUILD_TYPE="$arg" ;;
        --clean) CLEAN=1 ;;
    esac
done

if [ $CLEAN -eq 1 ] && [ -d build ]; then
    rm -rf build
fi

mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE="$BUILD_TYPE" -DCMAKE_PREFIX_PATH="$PREFIX" ..
make -j"$JOBS"

echo "Build complete: $BUILD_TYPE"
echo "  binaries: build/dds_recorder  build/dds_player"
