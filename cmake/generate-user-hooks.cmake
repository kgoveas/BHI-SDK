################################################################################
###
### @file       generate-user-hooks.cmake
###
### @project    7189
###
### @brief      CMake script use to generate the user mode hook support.
###
### @classification  Confidential
###
################################################################################
###
################################################################################
###
### @copyright Copyright (C) 2017 EM Microelectronic
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

SET(list ${CMAKE_ARGV3})
SET(output ${CMAKE_ARGV4})
SET(isuser ${CMAKE_ARGV5})

SET(HEADER "////////////////////////////////////////////////////////////////////////////////
//
//
// Project       : 718x
//
// Description   : AUTO-GENERATED DO NOT MODIFY
//
////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////
//                          Copyright
////////////////////////////////////////////////////////////////////////////////
//
//         Copyright (C) EM Microelectronic US Inc.
//
// Disclosure to third parties or reproduction in any form what-
// soever, without prior written consent, is strictly forbidden
//
////////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <SensorAPI.h>
#include <hooks_support.h>

extern bool hookExecute(void* sensor, void* param, structHook** hookItems, structHook** hookItemEnd, int hook_type, void* output);

#define HOOK_START(name)    CONCAT(_fhook_,name)
#define HOOK_END(name)      CONCAT(_ehook_,name)

#define DEF_HOOK_SYMBOLS(name)  \\
    extern structHook * __attribute__((weak)) HOOK_START(name)[]; \\
    extern structHook * __attribute__((weak)) HOOK_END(name)[]

#define DEF_HOOK_ADDRS(name) \\
    {HOOK_START(name), HOOK_END(name)}

struct sectionAddress {
    structHook** addStart;
    structHook** addEnd;
};

")

SET(FOOTER "
")

message(STATUS "generating ${output} from ${list}")

include(${CMAKE_CURRENT_LIST_DIR}/../common/macros.cmake)

FILE(STRINGS ${list} hooklist)

set(s )
set(hooks )
FOREACH(line ${hooklist})
    separate_arguments(elems UNIX_COMMAND ${line})
    list(LENGTH elems len)

    if(${len} LESS 2)
        continue()
    endif()

    list(GET elems 1 return_type)
    list(GET elems 0 hook_function)

    SET(hooks ${hooks} ${hook_function})
ENDFOREACH()

SET(s "

void RECLAIM_TEXT user_init_hooks(void)
{
    extern void hookSortStructuresByPriority(structHook *structArray[], int numItems);
    SInt32 numHookItems;

    // Sort all hooks;
    for(unsigned int i = 0; i < (sizeof(gHookSectionAddress)/sizeof(gHookSectionAddress[0])); i++)
    {
        numHookItems = gHookSectionAddress[i].addEnd - gHookSectionAddress[i].addStart;
        if (1 < numHookItems )
            hookSortStructuresByPriority(gHookSectionAddress[i].addStart, numHookItems);
    }
}

void user_nonvoid_hook(void* arg0, void* arg1, UInt8 hookid, UInt8 hook_type, void* output)
{
    (void)hookExecute(arg0, arg1, gHookSectionAddress[hookid].addStart, gHookSectionAddress[hookid].addEnd, hook_type, output);
}

void user_void_hook(void* arg0, void* arg1, UInt8 hookid, UInt8 hook_type)
{
    (void)callNArgHook(arg0, arg1, gHookSectionAddress[hookid].addStart, gHookSectionAddress[hookid].addEnd, hook_type);
}
")

SET(DATA )
FOREACH(hook ${hooks})
    SET(DATA "${DATA}DEF_HOOK_SYMBOLS(${hook});\n")
ENDFOREACH()

SET(DATA "${DATA}\nconst struct sectionAddress gHookSectionAddress[] = {\n")
FOREACH(hook ${hooks})
    SET(DATA "${DATA}    DEF_HOOK_ADDRS(${hook}),\n")
ENDFOREACH()
SET(DATA "${DATA}};\n\n")


FILE(WRITE ${output} "${HEADER}${DATA}${s}${FOOTER}")
#MESSAGE("${HEADER}${DATA}${s}${FOOTER}")
