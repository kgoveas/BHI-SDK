################################################################################
###
### @file       CMakeLists.txt
###
### @project    7189
###
### @brief      Device driver CMake file.
###
### @classification  Confidential
###
################################################################################
###
################################################################################
###
### @copyright Copyright (C) 2017-2019 EM Microelectronic
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
### and#or other materials provided with the distribution.
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

project(drivers_custom C)

ADD_C_FLAGS( ${DRIVER_C_FLAGS} )
ADD_ASM_FLAGS( ${DRIVER_ASM_FLAGS} )

include_directories( ../common/includes
                     ../libs/Hooks/includes
                     ../libs/SensorInterface/includes
                     ../libs/I2CInterface/includes
                     ../libs/SPIInterface/includes
                     ../libs/Time/includes
                     ../libs/HostInterface/includes
                     ../libs/HostInterface${HOST_INTERFACE}/includes/
                     ../libs/openrtos/include
                     ../libs/openrtos/portable/Metaware/EMx_MPU
                    )

FOREACH(drv ${DRIVERS})
    IF(EXISTS ${CMAKE_CURRENT_LIST_DIR}/${drv})
        ADD_SUBDIRECTORY(${drv})
    ENDIF()
ENDFOREACH(drv)

IF(NOT SDK)
    install(FILES CMakeLists.txt DESTINATION drivers_custom)
    install(DIRECTORY templates DESTINATION drivers_custom)
    install(DIRECTORY VirtAltitude DESTINATION drivers_custom)
    install(DIRECTORY VirtBSXCustomAccelDataSource DESTINATION drivers_custom)
    install(DIRECTORY VirtBSXLeanDeviceOrientation DESTINATION drivers_custom)
ENDIF()

IF(GENERATES_SDK)
    install(FILES CMakeLists.no-source.in DESTINATION drivers_custom)
    install(FILES CMakeLists.txt.sdk DESTINATION drivers_custom)
ENDIF()
