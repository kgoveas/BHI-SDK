################################################################################
###
### @file       common/cpack.cmake
###
### @project    EM7189
###
### @brief
###
### @classification  Confidential
###
################################################################################
###
################################################################################
###
### @copyright Copyright (C) 2019 EM Microelectronic
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
################################################################################
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
################################################################################
##### CPack Configuration #####
SET(CPACK_GENERATOR "TGZ;ZIP")
SET(CPACK_PACKAGE_VERSION "${RELEASE_MAJOR}.${RELEASE_MINOR}.${SVN_REVISION}")
SET(CPACK_PACKAGE_VERSION_MAJOR "${RELEASE_MAJOR}")
SET(CPACK_PACKAGE_VERSION_MINOR "${RELEASE_MINOR}")
SET(CPACK_PACKAGE_VERSION_PATCH "${SVN_REVISION}")


SET(CPACK_PACKAGE_VENDOR "EM Microelectronic-US Inc")

SET(CPACK_SYSTEM_NAME "SDK")
SET(CPACK_TOPLEVEL_TAG "SDK")

SET(CPACK_PACKAGE_NAME "${DIST_TYPE}")
SET(CPACK_PACKAGING_INSTALL_PREFIX "/${DIST_TYPE}-${CPACK_PACKAGE_VERSION}-SDK")
SET(CPACK_PACKAGE_FILE_NAME "${DIST_TYPE}-${CPACK_PACKAGE_VERSION}")

SET(CPACK_ARCHIVE_COMPONENT_INSTALL ON)
set(CPACK_COMPONENTS_ALL ${INSTALL_COMPONENTS})

INCLUDE(CPack)
