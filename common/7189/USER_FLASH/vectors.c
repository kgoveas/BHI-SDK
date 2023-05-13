////////////////////////////////////////////////////////////////////////////////
///
/// @file       common/7189/USER_FLASH/vectors.c
///
/// @project    EM7189
///
/// @brief      Default entr points for Usermode RAM code.
///
/// @classification  Confidential
///
////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
///
/// @copyright Copyright (C) 2018 EM Microelectronic
/// @cond
///
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions are met:
/// 1. Redistributions of source code must retain the above copyright notice,
/// this list of conditions and the following disclaimer.
/// 2. Redistributions in binary form must reproduce the above copyright notice,
/// this list of conditions and the following disclaimer in the documentation
/// and/or other materials provided with the distribution.
///
////////////////////////////////////////////////////////////////////////////////
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
/// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
/// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
/// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
/// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
/// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
/// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
/// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
/// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
/// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
/// POSSIBILITY OF SUCH DAMAGE.
/// @endcond
////////////////////////////////////////////////////////////////////////////////

#include <user.h>
#include <SensorAPI.h>
#include <linker.h>
/**
 * @brief Interrupt vector table.
 */
__attribute__((section(".vectors")))
const user_entries_t gUserEntries =
{
    .entries = {
        user_init,              // 0x00 Early Initialization
        user_thread,            // 0x01 Main User Thread
        user_void_hook,         // 0x02 Execute user hook with void return.
        user_nonvoid_hook,      // 0x03 Execute user hook with a return value.
    },

    .config   = &gEM7189Config,
    .initdata = &_initdat,

    .fphys_sensor_descriptors = _fphys_sensor_descriptors,
    .ephys_sensor_descriptors = _ephys_sensor_descriptors,
    .fvirt_sensor_descriptors = _fvirt_sensor_descriptors,
    .evirt_sensor_descriptors = _evirt_sensor_descriptors,
    .ftimer_sensor_descriptors = _ftimer_sensor_descriptors,
    .etimer_sensor_descriptors = _etimer_sensor_descriptors,

    .ffreecoderam = _ffreecoderam,
    .efreecoderam = _efreecoderam,
    .ffreedataram = _ffreedataram,
    .efreedataram = _efreedataram,

    .user_hash = {.hash = BUILD_HASH},
};
