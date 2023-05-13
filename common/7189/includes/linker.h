////////////////////////////////////////////////////////////////////////////////
///
/// @file       common/7189/includes/linker.h
///
/// @project    EM7189
///
/// @brief      Various linker defined symbols.
///
/// @classification  Confidential
///
////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
///
/// @copyright Copyright (C) 2016-2018 EM Microelectronic
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

#ifndef _LINKER_H
#define _LINKER_H
#include <SensorAPI.h>

/**
 * @brief Code to execute to initialize early routines.
 * This is defined in the crt1.s
 */
int user_init(void);

/**
 * @brief Main usermode thread to execute.
 */
int user_thread(void);

/**
 * @brief Main usermode hook entry points.
 */
int user_void_hook(void);
int user_nonvoid_hook(void);

extern void* _initdat;
extern PhysicalSensorDescriptor  _fphys_sensor_descriptors[],  _ephys_sensor_descriptors[]; /* defined by the linker */
extern VirtualSensorDescriptor   _fvirt_sensor_descriptors[],  _evirt_sensor_descriptors[]; /* defined by the linker */
extern TimerSensorDescriptor    _ftimer_sensor_descriptors[], _etimer_sensor_descriptors[]; /* defined by the linker */

extern UInt8 _ffreedataram[];                   // Start of free data ram
extern UInt8 _efreedataram[];                   // End of free data ram + 1

extern UInt8 _ffreecoderam[];                   // Start of free program ram
extern UInt8 _efreecoderam[];                   // End of free program ram + 1

extern char _fstack[];                          // Lowest stack address.
extern char _estack[];                          // Highest stack address + 1.


#endif /* _LINKER_H */
