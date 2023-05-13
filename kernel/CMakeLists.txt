################################################################################
###
### @file       kernel/CMakeLists.txt
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
file(GLOB CONFIG_FILES "${CMAKE_CURRENT_LIST_DIR}/*.sdk.cmake")
foreach(f ${CONFIG_FILES})
  include(${f})
endforeach()

if(SUPPORT_ENCRYPTED_FW)
    set_property(TARGET libkernel-aes PROPERTY USER_LINKER_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/kernel-aes-linker.cmd)
    add_custom_command(
        OUTPUT kernel-aes.${KERNEL_SUFFIX}
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_LIST_DIR}/kernel-aes.${KERNEL_SUFFIX} ${CMAKE_BINARY_DIR}/kernel/kernel-aes.${KERNEL_SUFFIX}
        VERBATIM)
    add_custom_target(copy-kernel-aes.${KERNEL_SUFFIX} DEPENDS kernel-aes.${KERNEL_SUFFIX})
    add_dependencies(libkernel-aes copy-kernel-aes.${KERNEL_SUFFIX})
endif()

if(BUILD_FLASH)
    set_property(TARGET libkernel-flash PROPERTY USER_LINKER_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/kernel-flash-linker.cmd)
    add_custom_command(
        OUTPUT kernel-flash.${KERNEL_SUFFIX}
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_LIST_DIR}/kernel-flash.${KERNEL_SUFFIX} ${CMAKE_BINARY_DIR}/kernel/kernel-flash.${KERNEL_SUFFIX}
        VERBATIM)
    add_custom_target(copy-kernel-flash.${KERNEL_SUFFIX} DEPENDS kernel-flash.${KERNEL_SUFFIX})
    add_dependencies(libkernel-flash copy-kernel-flash.${KERNEL_SUFFIX})
endif()

set_property(TARGET libkernel PROPERTY USER_LINKER_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/kernel-linker.cmd)
add_custom_command(
    OUTPUT kernel.${KERNEL_SUFFIX}
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_LIST_DIR}/kernel.${KERNEL_SUFFIX} ${CMAKE_BINARY_DIR}/kernel/kernel.${KERNEL_SUFFIX}
    VERBATIM)
add_custom_target(copy-kernel.${KERNEL_SUFFIX} DEPENDS kernel.${KERNEL_SUFFIX})
add_dependencies(libkernel copy-kernel.${KERNEL_SUFFIX})
