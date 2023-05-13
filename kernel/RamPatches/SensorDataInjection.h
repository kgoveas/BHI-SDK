////////////////////////////////////////////////////////////////////////////////
///
/// @file       kernel/RamPatches/SensorDataInjection.h
///
/// @project    EM7189
///
/// @brief      Sensor Data Injection support
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
#ifndef _SENSOR_DATA_INJECTION_H
#define _SENSOR_DATA_INJECTION_H

extern SystemTime_t getSystemTime_bosch_rom();
extern void patch_jli(unsigned jli_index, void *func_addr);

// Host commands
void SDI_setMode(const COMMAND_HEADER *cmd, STATUS_HEADER *stat);
void SDI_injectData(const COMMAND_HEADER *cmd, STATUS_HEADER *stat);

typedef struct _sdi_entry_type_
{
    PhysicalSensorDescriptor * sensor;
    UInt8 *dataBuffer;
    UInt8  packetSize;
    struct _sdi_entry_type_ *next;
} sdi_entry_t;

// Sensor driver calls
SensorStatus SDI_initializeSensor(sdi_entry_t *entry);

#endif
