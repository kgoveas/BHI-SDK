////////////////////////////////////////////////////////////////////////////////
///
/// @file       libs/Outerloop/includes/support_ram.h
///
/// @project    EM7189
///
/// @brief
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
#ifndef SUPPORT_RAM_H
#define SUPPORT_RAM_H

#include <types.h>
#include <SensorAPI.h>
#include <EM718xConfig.h>

//#define USE_RTOS
extern bool initOpenRTOS(void);
extern UInt32 getAndClearRTOSEvents(UInt32 mask);
extern UInt32 getRTOSEvents(void);
extern void hostTask(void *pvParameters);
extern void sensorTask(void *pvParameters);
extern void initHostInterfaceTask(void);
extern void initSensorManagerTask(void);

#define NUM_RESETS_BEFORE_ERROR 2   /* Error out on the 3rd reset */
#define TIMEOUT_MULTIPLIER      3

#define CONFIG_T    EM7189Config

extern SensorStatus checkSensors(PhysicalSensorDescriptor* phys, UInt32 numPhys, VirtualSensorDescriptor* virt, UInt32 numVirt);

extern bool RECLAIM_TEXT initHardware(const CONFIG_T*  em718xConfig);
extern bool RECLAIM_TEXT initSensorInterfaces(const CONFIG_T*  em718xConfig, PhysicalSensorDescriptor* phys, UInt32 numPhys, VirtualSensorDescriptor* virt, UInt32 numVirt);
extern bool RECLAIM_TEXT startupHardware(void);

extern UInt8 initSensors(PhysicalSensorDescriptor* phys, UInt32 numPhys, VirtualSensorDescriptor* virt, UInt32 numVirt);

extern void teardown(PhysicalSensorDescriptor* phys, UInt32 numPhys, VirtualSensorDescriptor* virt, UInt32 numVirt);

extern void haltOnError(void);

extern void hookInitPrintf(void);

#endif /* SUPPORT_RAM_H */
