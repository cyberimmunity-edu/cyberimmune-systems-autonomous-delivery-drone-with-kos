#!/bin/bash

SCRIPT_DIR="$(dirname "$(realpath "${0}")")"
BUILD="${SCRIPT_DIR}/build"

export LANG=C
export TARGET="aarch64-kos"
export PKG_CONFIG=""
export SDK_PREFIX="/opt/KasperskyOS-Community-Edition-1.2.0.89"
export INSTALL_PREFIX="$BUILD/../install"
export PATH="$SDK_PREFIX/toolchain/bin:$PATH"

export BUILD_WITH_CLANG=
export BUILD_WITH_GCC=

SIMULATION=""
SERVER=""
KOS_TARGET=""
BOARD_ID=""
UNIT_TESTS=""
PAL_TESTS=""
SIMULATOR_IP="10.0.2.2"
SERVER_IP="10.0.2.2"
USE_DEMO_CODE="FALSE"

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
    --board-id,
             User-defined board ID to use instead of MAC-address
    --simulator-ip,
             User-defined IP of SITL
    --server-ip,
             User-defined IP of the AFCS server
    --target,
             Build target: hardware (real), simulation (sim), unit-tests (unit) or pal-tests (pal)
    --mode,
             Connection mode: online or offline

  Examples:
      bash cross-build.sh -s /opt/KasperskyOS-Community-Edition-1.2.0.89

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
        --simulator-ip)
            SIMULATOR_IP=$2
            ;;
        --server-ip)
            SERVER_IP=$2
            ;;
        --board-id)
            BOARD_ID=$2
            ;;
        --target)
            if [ "$2" == "hardware" ] || [ "$2" == "real" ]; then
                SIMULATION="FALSE"
                UNIT_TESTS="FALSE"
                PAL_TESTS="FALSE"
                KOS_TARGET="kos-image"
            elif [ "$2" == "simulation" ] || [ "$2" == "sim" ]; then
                SIMULATION="TRUE"
                UNIT_TESTS="FALSE"
                PAL_TESTS="FALSE"
                KOS_TARGET="sim"
            elif [ "$2" == "unit-tests" ] || [ "$2" == "unit" ]; then
                SIMULATION="TRUE"
                UNIT_TESTS="TRUE"
                PAL_TESTS="FALSE"
                KOS_TARGET="sim"
            elif [ "$2" == "pal-tests" ] || [ "$2" == "pal" ]; then
                SIMULATION="TRUE"
                UNIT_TESTS="FALSE"
                PAL_TESTS="TRUE"
                KOS_TARGET="pal-test0"
            else
                echo "Unknown target '$2'"
                exit 1
            fi
            ;;
        --mode)
            if [ "$2" == "online" ]; then
                SERVER="TRUE"
            elif [ "$2" == "offline" ]; then
                SERVER="FALSE"
            else
                echo "Unknown mode '$2'"
                exit 1
            fi
            ;;
        --demo)
            USE_DEMO_CODE="TRUE"
            ;;
        -*)
            echo "Invalid option: $key"
            exit 1
            ;;
        esac
    shift
done

if [ "$SIMULATION" == "" ]; then
    echo "Build target is not set"
    exit 1
fi

if [ "$SERVER" == "" ] && [ "$UNIT_TESTS" != "TRUE" ] && [ "$PAL_TESTS" != "TRUE" ]; then
    echo "Build mode is not set"
    exit 1
fi

TOOLCHAIN_SUFFIX=""

if [ "$BUILD_WITH_CLANG" == "y" ]; then
    TOOLCHAIN_SUFFIX="-clang"
fi

if [ "$BUILD_WITH_GCC" == "y" ]; then
    TOOLCHAIN_SUFFIX="-gcc"
fi

"$SDK_PREFIX/toolchain/bin/cmake" -G "Unix Makefiles" -B "$BUILD" \
      -D SIMULATION=$SIMULATION \
      -D SERVER=$SERVER \
      -D UNIT_TESTS="$UNIT_TESTS" \
      -D PAL_TESTS="$PAL_TESTS" \
      -D BOARD_ID="$BOARD_ID" \
      -D SIMULATOR_IP=$SIMULATOR_IP \
      -D SERVER_IP=$SERVER_IP \
      -D USE_DEMO_CODE=$USE_DEMO_CODE \
      -D CMAKE_BUILD_TYPE:STRING=Debug \
      -D CMAKE_INSTALL_PREFIX:STRING="$INSTALL_PREFIX" \
      -D CMAKE_FIND_ROOT_PATH="${SDK_PREFIX}/sysroot-$TARGET" \
      -D CMAKE_TOOLCHAIN_FILE="$SDK_PREFIX/toolchain/share/toolchain-$TARGET$TOOLCHAIN_SUFFIX.cmake" \
      "$SCRIPT_DIR/" && "$SDK_PREFIX/toolchain/bin/cmake" --build "$BUILD" --target "$KOS_TARGET"
