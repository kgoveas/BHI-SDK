////////////////////////////////////////////////////////////////////////////////
///
/// @file       libs/OuterloopROM/includes/support.h
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
#ifndef SUPPORT_H
#define SUPPORT_H

#include <types.h>
#include <SensorAPI.h>
#include <EM718xConfig.h>

#if defined(__U7180__)
#define CONFIG_T    EM718XConfig
#elif defined(__U7183__)
#define CONFIG_T    EM7183Config
#elif defined(_FRAMEWORK)
#define CONFIG_T    EM7183Config
#endif

extern bool gWatchdogEnabled; // if TRUE, watchdog is enabled
extern bool gWatchdogIntEnabled; // if TRUE, watchdog interrupts are enabled (will not reset)
extern bool gHasWakeSource; // if TRUE, hang detector timer is on


typedef enum RTOSEvent
{
   // host interface task events
   RTOSEV_HIF_START = 0,  //lint !e849

   RTOSEV_UPLOAD_CHANNEL = 0,    //lint !e849
   RTOSEV_COMMAND_MAIN,          // 1
   RTOSEV_HIF_EVENT,             // 2
   RTOSEV_WK_EVENT,              // 3
   RTOSEV_NW_EVENT,              // 4
   RTOSEV_DBG_EVENT,             // 5
   RTOSEV_WK_LATENCY,            // 6
   RTOSEV_NW_LATENCY,            // 7
   RTOSEV_DBG_LATENCY,           // 8
   RTOSEV_WK_WATERMARK,          // 9
   RTOSEV_NW_WATERMARK,          // 10
   RTOSEV_DBG_WATERMARK,         // 11
   RTOSEV_WK_FLUSH,              // 12
   RTOSEV_NW_FLUSH,              // 13
   RTOSEV_DBG_FLUSH,             // 14
   RTOSEV_HIF_END = 15,          // 15

   // sensor manager task events
   RTOSEV_SENSOR_START = 16,     //lint !e849

   RTOSEV_SENSOR_RECONFIG = 16,  //lint !e849
   RTOSEV_SENSOR_HUNG,           // 17
   RTOSEV_SENSORS_TRIGGERED,     // 18
                                 //
   RTOSEV_SENSOR_END,            // 23

   // sensor fusion task events
   RTOSEV_FUSION_START = 24,     //lint !e849
   RTOSEV_FUSION_TRIGGER = 24,   //lint !e849
   RTOSEV_FUSION_END = 27,

   // fusion calibration task events
   RTOSEV_CALIB_START = 28,      //lint !e849
   RTOSEV_CALIB_TRIGGER = 28,    //lint !e849
   RTOSEV_CALIB_END = 31
} RTOSEvent;

#define RTOSEV_BIT(_ev_) ((UInt32)(1u << (_ev_)))
#define RTOSEV_HIF_MASK (0x0000ffffu)
#define RTOSEV_SNS_MASK (0x00ff0000u)
#define RTOSEV_FUS_MASK (0x0f000000u)
#define RTOSEV_CAL_MASK (0xf0000000u)

// don't use JLI macro -- must be JLI always
extern void __attribute__((jli_always)) triggerRTOSEvent(RTOSEvent event);
extern void __attribute__((jli_always)) triggerRTOSEvents(UInt32 events);

extern void __attribute__((jli_always)) triggerRTOSEventFromTask(RTOSEvent event);
extern void __attribute__((jli_always)) triggerRTOSEventsFromTask(UInt32 events);

extern void __attribute__((jli_always)) triggerRTOSEventFromISR(RTOSEvent event);
extern void __attribute__((jli_always)) triggerRTOSEventsFromISR(UInt32 events);

extern SensorStatus JLI checkSensors(PhysicalSensorDescriptor* phys, UInt32 numPhys, VirtualSensorDescriptor* virt, UInt32 numVirt);
extern UInt8 JLI initSensors(PhysicalSensorDescriptor* phys, UInt32 numPhys, VirtualSensorDescriptor* virt, UInt32 numVirt);
extern void JLI teardown(PhysicalSensorDescriptor* phys, UInt32 numPhys, VirtualSensorDescriptor* virt, UInt32 numVirt);
extern void gpioInterruptHandler(UInt32 event);

extern void JLI __attribute__((noreturn)) haltOnError(void);
extern void JLI checkChpDiagnostic(void);

extern void JLI initSimpleHeap(const char *ffreecoderam, const char *efreecoderam, const char *ffreedataram, const char *efreedataram);
extern UInt8 JLI *getEndofCodeRAM(void);
extern UInt8 JLI *getEndofDataRAM(void);
extern void JLI setEndofDataRAM(UInt8 *end);
extern UInt32 JLI numCodeRAMBytesFree(void);
extern UInt32 JLI numDataRAMBytesFree(void);
extern UInt32 JLI numRAMBytesFree(void);
extern UInt8 JLI *allocDataRAM(UInt32 num_bytes);
extern UInt8 JLI *allocCodeRAM(UInt32 num_bytes);
extern UInt8 JLI *allocRAM(UInt32 num_bytes);
extern UInt32 JLI powerDataRAM(UInt16 wordsreq);

#endif /* SUPPORT_H */
