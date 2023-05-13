////////////////////////////////////////////////////////////////////////////////
///
/// @file       libs/Outerloop/includes/user.h
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
#ifndef USER_H
#define USER_H

#include <fwHeader.h>
#include <EM718xConfig.h>

#define USER_EARLY_INIT         (0u)
#define USER_MAIN_THREAD        (1u)
#define USER_CALL_VOID_HOOK     (2u)
#define USER_CALL_NONVOID_HOOK  (3u)
#define USER_NUM_ENTRIES        (4u)

/* Forward declaration. */
struct PhysicalSensorDescriptor;
typedef struct PhysicalSensorDescriptor PhysicalSensorDescriptor;
struct VirtualSensorDescriptor;
typedef struct VirtualSensorDescriptor VirtualSensorDescriptor;
typedef struct VirtualSensorDescriptor TimerSensorDescriptor;

typedef int(*user_entry_t)();

typedef struct UserHash
{
    UInt8 hash[6];
} UserHash;

typedef struct {
    EM7189Config*               config;
    void*                       initdata;
    PhysicalSensorDescriptor*   fphys_sensor_descriptors;
    PhysicalSensorDescriptor*   ephys_sensor_descriptors;
    VirtualSensorDescriptor*    fvirt_sensor_descriptors;
    VirtualSensorDescriptor*    evirt_sensor_descriptors;
    TimerSensorDescriptor*      ftimer_sensor_descriptors;
    TimerSensorDescriptor*      etimer_sensor_descriptors;
    UInt8*                      ffreecoderam;
    UInt8*                      efreecoderam;
    UInt8*                      ffreedataram;
    UInt8*                      efreedataram;

    user_entry_t entries[USER_NUM_ENTRIES];

    UserHash                    user_hash;
} user_entries_t;

typedef struct {
    FWHeader_t      header;
    UInt32          header_copy_offset;
    FWHeader_t      header_copy;
    user_entries_t  user_entries;
} user_firmware_t;

user_firmware_t* get_user_firmware(void);
UInt8 verifyFirmwareHeader(const user_firmware_t* fw);
bool checkEncryption(const FWHeader_t *kernelFwHdr, const FWHeader_t *userFwHdr, UInt8 *fwBuffer, UInt32 encrOffset);

void RECLAIM_TEXT initUserMode(void);

void user_register_configuration(void);
void user_register_descriptors(void);
void user_register_ram(void);
void user_init_ram(void);

int call_user_entry(UInt8 entry);

void user_call_void_hook(void* arg0, void* arg1, UInt8 hook, UInt8 num_args);
void user_call_nonvoid_hook(void* arg0, void* arg1, UInt8 hook, UInt8 num_args, void* output);

static int ALWAYS_INLINE user_early_init(void)
{
    return call_user_entry(USER_EARLY_INIT);
}

static int ALWAYS_INLINE user_main_thread(void)
{
    return call_user_entry(USER_MAIN_THREAD);
}

#endif /* !USER_H */
