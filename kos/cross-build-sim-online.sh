#!/bin/bash

SCRIPT_DIR="$(dirname "$(realpath "${0}")")"
BUILD="${SCRIPT_DIR}/build_sim_online"

export LANG=C
export TARGET="aarch64-kos"
export PKG_CONFIG=""
export SDK_PREFIX="/opt/KasperskyOS-Local-Edition"
export INSTALL_PREFIX="$BUILD/../install"
export PATH="$SDK_PREFIX/toolchain/bin:$PATH"

export BUILD_WITH_CLANG=
export BUILD_WITH_GCC=

set -eu

function help
{
    cat <<EOF

  Usage: $0 [--help] [-s <SDK path>]

  Compile and link precompiled_vfs project with respect to specified arguments.

  Optional arguments:
    -s, --sdk-path,
             Path to KasperskyOS Community Edition SDK
             Default: ${SDK_PREFIX}

  Examples:
      bash cross-build.sh -s /opt/KasperskyOS-Community-Edition-1.1.1.13

EOF
}

# Main
while [[ $# > 0 ]];
do
    key="$1"
    case $key in
        --help|-h)
            help
            exit 0
            ;;
        --sdk-path|-s)
            SDK_PREFIX=$2
            ;;
        -*)
            echo "Invalid option: $key"
            exit 1
            ;;
        esac
    shift
done

TOOLCHAIN_SUFFIX=""

if [ "$BUILD_WITH_CLANG" == "y" ];then
    TOOLCHAIN_SUFFIX="-clang"
fi

if [ "$BUILD_WITH_GCC" == "y" ];then
    TOOLCHAIN_SUFFIX="-gcc"
fi

"$SDK_PREFIX/toolchain/bin/cmake" -G "Unix Makefiles" -B "$BUILD" \
      -D SIMULATION="TRUE" \
      -D SERVER="TRUE" \
      -D BOARD_ID="2" \
      -D SIMULATOR_IP="172.28.65.87" \
      -D CMAKE_BUILD_TYPE:STRING=Debug \
      -D CMAKE_INSTALL_PREFIX:STRING="$INSTALL_PREFIX" \
      -D CMAKE_FIND_ROOT_PATH="${SDK_PREFIX}/sysroot-$TARGET" \
      -D CMAKE_TOOLCHAIN_FILE="$SDK_PREFIX/toolchain/share/toolchain-$TARGET$TOOLCHAIN_SUFFIX.cmake" \
      "$SCRIPT_DIR/" && "$SDK_PREFIX/toolchain/bin/cmake" --build "$BUILD" --target sim
