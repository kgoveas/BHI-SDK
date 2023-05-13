#!/bin/bash
###############################################################################
###
### @file       utils/isign/build.sh
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
### Disclosure to third parties or reproduction in any form what-
### soever, without prior written consent, is strictly forbidden
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

GENERATOR=""

CMAKE=`which cmake3 2>/dev/null`
if [[ "${CMAKE}" == "" ]]
then
    CMAKE=`which cmake 2>/dev/null`
fi

if [[ "${CMAKE}" == "" ]]
then
    echo "Unable to locate cmake executable."
    exit -1
fi

NINJA=`which ninja 2>/dev/null`
if [[ "${NINJA}" != "" ]]
then
    GENERATOR="-G Ninja"
fi

echo "Attempting to build isign"

rm -rf build
mkdir build
cd build
${CMAKE} ${GENERATOR} ..
${CMAKE} --build .
