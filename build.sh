#!/bin/bash
###############################################################################
###
### @file       build.sh
###
### @project    EM7189
###
### @brief
###
### @classification  Confidential
###
###############################################################################
###
###############################################################################
###
### @copyright Copyright (C) 2018 EM Microelectronic
### @cond
###
### All rights reserved.
###
### Redistribution and use in source and binary forms, with or without
### modification, are permitted provided that the following conditions are met:
### 1. Redistributions of source code must retain the above copyright notice,
### this list of conditions and the following disclaimer.
### 2. Redistributions in binary form must reproduce the above copyright notice,
### this list of conditions and the following disclaimer in the documentation
### and/or other materials provided with the distribution.
###
###############################################################################
###
### THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
### AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
### IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
### ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
### LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
### CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
### SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
### INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
### CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
### ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
### POSSIBILITY OF SUCH DAMAGE.
### @endcond
###############################################################################

VERSION=`cat version`
USE_GCC=$1
TARGET=$2
DEFINES="${@:3}" # All remaining definitions
BUILD_CLEAN=$1

if [ "$BUILD_CLEAN" == "clean" ]
then
	echo "Remove build and release folder"
	rm -rf build release
	exit -1
fi

if [[ "$USE_GCC" != "USE_GCC" ]]
then
    USE_GCC=""
    TARGET=$1
    DEFINES="${@:2}" # All remaining definitions
else
    USE_GCC="-DUSE_GCC=1"
fi

if [[ "$TARGET" != "" ]]
then
    #ensure target(s) exists, else treat as define
    OLDIFS="$IFS"
    IFS=';'
    TARGETS=""
    for target_board in $TARGET
    do
        if [ ! -e boards/${target_board}.cfg ]
        then
            echo "Board config ${target_board}.cfg not found, this block is skipped!"
        else
            echo "Building board ${target_board}.cfg"
            TARGETS="$TARGETS;$target_board"
        fi
    done
    IFS=$OLDIFS

    if [[ "$TARGETS" == "" ]]
    then
        echo "No valid boards found in '${TARGET}', treating parameter as define!"
        DEFINES="${@:1}" # All parameters are treated as definitions
    fi
else
    DEFINES="${@:1}" # All parameters are treated as definitions
fi

CMAKE=`which cmake3 2>/dev/null`
if [[ "${CMAKE}" == "" ]]
then
    CMAKE=`which cmake 2>/dev/null`
fi

CPACK=`which cpack3 2>/dev/null`
if [[ "${CPACK}" == "" ]]
then
    CPACK=`which cpack 2>/dev/null`
fi

if [[ "${CMAKE}" == "" ]]
then
    echo "Unable to locate cmake executable."
    exit -1
fi

if [[ "${GENERATOR}" == "" ]]
then
    NINJA=`which ninja 2>/dev/null`
    if [[ "${NINJA}" != "" ]]
    then
        GENERATOR="-GNinja"
    else
        NINJA=`which ninja-build 2>/dev/null`
        if [[ "${NINJA}" != "" ]]
        then
            GENERATOR="-GNinja"
        else
            # Fallback to make if ninja isn't found.
            GENERATOR="-GUnix Makefiles"
        fi
    fi
else
    GENERATOR="-G${GENERATOR}"
    echo "Using CMake generator from the environment: ${GENERATOR}"
fi

# Error out if any command below here fail.
set -e

echo "Attempting to build SDK ${VERSION}"

BUILD_DIR=`pwd`/build/
RELEASE_DIR=`pwd`/release/

if [[ -d "$BUILD_DIR" ]]
then
	rm -rf "$BUILD_DIR"
	echo "rm $BUILD_DIR"
fi

if [[  -d "$RELEASE_DIR" ]]
then
	rm -rf "$RELEASE_DIR"
	echo "rm $RELEASE_DIR"
fi

mkdir build release
cd build

# if TARGETS is set, modify BOARDS variable to contain only specified target(s)
if [[ "$TARGETS" != "" ]]
then
    ${CMAKE} "${GENERATOR}" -DCMAKE_INSTALL_PREFIX="${RELEASE_DIR}" "${USE_GCC}" -DBOARDS="${TARGETS}" "${DEFINES}" ..
else
    ${CMAKE} "${GENERATOR}" -DCMAKE_INSTALL_PREFIX="${RELEASE_DIR}" "${USE_GCC}" "${DEFINES}" ..
fi
${CMAKE} --build .
${CMAKE} --build . --target install

# Print the export disclaimer
FILE=../DISCLAIMER.txt
if test -f "$FILE"; then
    cat "$FILE"
fi
