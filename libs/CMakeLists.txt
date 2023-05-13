################################################################################
###
### @file       libs/CMakeLists.txt
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
#project(libs C ASM)

ADD_C_FLAGS( ${LIB_C_FLAGS} )
ADD_ASM_FLAGS( ${LIB_ASM_FLAGS} )

include_directories( ../common/includes/
                     Config/includes/
                     Time/includes/
                     SensorInterface/includes/
                     Hooks/includes/
                     HostInterface/includes/
                     openrtos/include/
                     openrtos/portable/Metaware/EMx_MPU/
                     Ram/includes/
                     DMA/includes/
                     OTPInterface/includes/
                   )

# Ensure Libraries are in at most one of EXPORT_LIB_SOURCE, EXPORT_LIB_BINARIES
SET(EXPORT_DUP_LIBS
    ${EXPORT_LIB_SOURCE}
    ${EXPORT_LIB_BINARIES}
)
SET(EXPORT_LIBS_ALL ${EXPORT_DUP_LIBS})
list(REMOVE_DUPLICATES EXPORT_DUP_LIBS)
IF(NOT EXPORT_DUP_LIBS STREQUAL EXPORT_LIBS_ALL)
    MESSAGE("EXPORT_LIBS_ALL: ${EXPORT_LIBS_ALL}")
    MESSAGE("EXPORT_DUP_LIBS: ${EXPORT_DUP_LIBS}")
    MESSAGE(FATAL_ERROR "Duplicate library found in one of EXPORT_LIB_SOURCE or EXPORT_LIB_BINARIES")
ENDIF()

# Ensure Libraries are in at most one of EXPORT_LIB_SOURCE, EXPORT_LIB_HEADERS
SET(EXPORT_DUP_LIBS
    ${EXPORT_LIB_SOURCE}
    ${EXPORT_LIB_HEADERS}
)
SET(EXPORT_LIBS_ALL ${EXPORT_DUP_LIBS})
list(REMOVE_DUPLICATES EXPORT_DUP_LIBS)
IF(NOT EXPORT_DUP_LIBS STREQUAL EXPORT_LIBS_ALL)
    MESSAGE("EXPORT_LIBS_ALL: ${EXPORT_LIBS_ALL}")
    MESSAGE("EXPORT_DUP_LIBS: ${EXPORT_DUP_LIBS}")
    MESSAGE(FATAL_ERROR "Duplicate library found in one of EXPORT_LIB_SOURCE or EXPORT_LIB_HEADERS")
ENDIF()




SET(SUBDIRS
    ${EXPORT_LIB_SOURCE}    # Libraries that will be expoted in source form
    ${EXPORT_LIB_BINARIES}  # LIbraries that will be exported in binary and header form
    ${EXPORT_LIB_HEADERS}   # Libraries that will be exported in header form with a dummy binary
    ${BUILD_LIBS}           # All Libraries that need to be built, even if not exported
    ${EXTRA_LIBS}           # Any additional libraries needed that don't fall into the above categories
)
list(REMOVE_DUPLICATES SUBDIRS)
IF(REMOVED_LIBS)
    LIST(REMOVE_ITEM SUBDIRS ${REMOVED_LIBS})
ENDIF()
FOREACH(subdir ${SUBDIRS})
    if(EXISTS ${CMAKE_CURRENT_LIST_DIR}/${subdir}/CMakeLists.txt)
        add_subdirectory(${subdir})
    else()
        message(WARNING "library ${subdir} is not cmake-enabled")
    endif()
ENDFOREACH()

IF(NOT SDK)
    install(FILES CMakeLists.txt DESTINATION libs)
ENDIF()

IF(GENERATES_SDK)
    install(FILES CMakeLists.no-source.in DESTINATION libs)
    install(FILES CMakeLists.txt.sdk DESTINATION libs)
ENDIF()
