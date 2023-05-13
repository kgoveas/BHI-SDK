////////////////////////////////////////////////////////////////////////////////
///
/// @file       common/7189/includes/em7189_host_regs.h
///
/// @project    EM7189
///
/// @brief      <DESCRIPTION>
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

#ifndef EM7189_HOST_REGS_H
#define EM7189_HOST_REGS_H

#include <types.h>

#define HREG_INPUT_CHANNEL0 ((UInt8)0x0)         /* This bit-field holds the data to be transferred to CCM via channel 0. A write to this register initiates a transfer (i.e. the host throttles the upload). */
#define HREG_OUTPUT_CHANNEL1 ((UInt8)0x1)        /* This bit-field holds the data to be transferred from CCM via channel 1. A read from this register initiates a transfer (i.e. the host throttles the download). */
#define HREG_OUTPUT_CHANNEL2 ((UInt8)0x2)        /* This bit-field holds the data to be transferred from CCM via channel 2. A read from this register initiates a transfer (i.e. the host throttles the download). */
#define HREG_OUTPUT_CHANNEL3 ((UInt8)0x3)        /* This bit-field holds the data to be transferred from CCM via channel 3. A read from this register initiates a transfer (i.e. the host throttles the download). */
#define HREG_RESERVED_0 ((UInt8)0x4)             /* General purpose registers for communicating information between host and sensor hub FW. */
#define HREG_CHIP_CONTROL ((UInt8)0x5)           /* Host writeable register to start us running after boot. */
#define HREG_HOST_INTERFACE_CONTROL ((UInt8)0x6) /* General purpose registers for communicating information between host and sensor hub FW. */
#define HREG_HOST_INTERRUPT_CONTROL ((UInt8)0x7) /* General purpose registers for communicating information between host and sensor hub FW. */
#define HREG_GP1_B0 ((UInt8)0x8)                 /* General purpose registers for communicating information between host and sensor hub FW. */
#define HREG_GP1_B1 ((UInt8)0x9)                 /* General purpose registers for communicating information between host and sensor hub FW. */
#define HREG_GP1_B2 ((UInt8)0xa)                 /* General purpose registers for communicating information between host and sensor hub FW. */
#define HREG_GP1_B3 ((UInt8)0xb)                 /* General purpose registers for communicating information between host and sensor hub FW. */
#define HREG_GP2_B0 ((UInt8)0xc)                 /* General purpose registers for communicating information between host and sensor hub FW. */
#define HREG_GP2_B1 ((UInt8)0xd)                 /* General purpose registers for communicating information between host and sensor hub FW. */
#define HREG_GP2_B2 ((UInt8)0xe)                 /* General purpose registers for communicating information between host and sensor hub FW. */
#define HREG_GP2_B3 ((UInt8)0xf)                 /* General purpose registers for communicating information between host and sensor hub FW. */
#define HREG_GP3_B0 ((UInt8)0x10)                /* General purpose registers for communicating information between host and sensor hub FW. */
#define HREG_GP3_B1 ((UInt8)0x11)                /* General purpose registers for communicating information between host and sensor hub FW. */
#define HREG_GP3_B2 ((UInt8)0x12)                /* General purpose registers for communicating information between host and sensor hub FW. */
#define HREG_GP3_B3 ((UInt8)0x13)                /* General purpose registers for communicating information between host and sensor hub FW. */
#define HREG_RESETREQ ((UInt8)0x14)              /*  */
#define HREG_EVIRQREQ ((UInt8)0x15)              /*  */
#define HREG_HOST_CONTROL ((UInt8)0x16)          /*  */
#define HREG_HOST_STATUS ((UInt8)0x17)           /*  */
#define HREG_CRC0 ((UInt8)0x18)                  /* This bit-field is part of the 32-bit CRC calculated over the bytes transferred via the current down-load channel in use. The 32-bit CRC initializes to 0xFFFF_FFFF (initial CRC seed) every time a different channel is accessed. CRC0 holds the lower byte of the 32-bit CRC and CRC3 the upper byte. */
#define HREG_CRC1 ((UInt8)0x19)                  /* This bit-field is part of the 32-bit CRC calculated over the bytes transferred via the current down-load channel in use. The 32-bit CRC initializes to 0xFFFF_FFFF (initial CRC seed) every time a different channel is accessed. CRC0 holds the lower byte of the 32-bit CRC and CRC3 the upper byte. */
#define HREG_CRC2 ((UInt8)0x1a)                  /* This bit-field is part of the 32-bit CRC calculated over the bytes transferred via the current down-load channel in use. The 32-bit CRC initializes to 0xFFFF_FFFF (initial CRC seed) every time a different channel is accessed. CRC0 holds the lower byte of the 32-bit CRC and CRC3 the upper byte. */
#define HREG_CRC3 ((UInt8)0x1b)                  /* This bit-field is part of the 32-bit CRC calculated over the bytes transferred via the current down-load channel in use. The 32-bit CRC initializes to 0xFFFF_FFFF (initial CRC seed) every time a different channel is accessed. CRC0 holds the lower byte of the 32-bit CRC and CRC3 the upper byte. */
#define PREG_PRODUCT_ID ((UInt8)0x1c)            /*  */
#define PREG_REVISION_ID ((UInt8)0x1d)           /*  */
#define PREG_ROM_VERSION_0 ((UInt8)0x1e)         /* General purpose registers for communicating information between sensor hub FW and host. */
#define PREG_ROM_VERSION_1 ((UInt8)0x1f)         /* General purpose registers for communicating information between sensor hub FW and host. */
#define PREG_RAM_VERSION_0 ((UInt8)0x20)         /* General purpose registers for communicating information between sensor hub FW and host. */
#define PREG_RAM_VERSION_1 ((UInt8)0x21)         /* General purpose registers for communicating information between sensor hub FW and host. */
#define PREG_FLASH_VERSION_0 ((UInt8)0x22)       /* General purpose registers for communicating information between sensor hub FW and host. */
#define PREG_FLASH_VERSION_1 ((UInt8)0x23)       /* General purpose registers for communicating information between sensor hub FW and host. */
#define PREG_FEATURE_STATUS ((UInt8)0x24)        /* General purpose registers for communicating information between sensor hub FW and host. */
#define PREG_BOOT_STATUS ((UInt8)0x25)           /* General purpose registers for communicating information between sensor hub FW and host. */
#define PREG_HOST_IRQ_TIMESTAMP_0 ((UInt8)0x26)  /* General purpose registers for communicating information between sensor hub FW and host. */
#define PREG_HOST_IRQ_TIMESTAMP_1 ((UInt8)0x27)  /* General purpose registers for communicating information between sensor hub FW and host. */
#define PREG_HOST_IRQ_TIMESTAMP_2 ((UInt8)0x28)  /* General purpose registers for communicating information between sensor hub FW and host. */
#define PREG_HOST_IRQ_TIMESTAMP_3 ((UInt8)0x29)  /* General purpose registers for communicating information between sensor hub FW and host. */
#define PREG_HOST_IRQ_TIMESTAMP_4 ((UInt8)0x2a)  /* General purpose registers for communicating information between sensor hub FW and host. */
#define PREG_RESERVED_1 ((UInt8)0x2b)            /* General purpose registers for communicating information between sensor hub FW and host. */
#define PREG_RESERVED_2 ((UInt8)0x2c)            /* General purpose registers for communicating information between sensor hub FW and host. */
#define PREG_INTERRUPT_STATUS ((UInt8)0x2d)      /* General purpose registers for communicating information between sensor hub FW and host. */
#define PREG_ERROR ((UInt8)0x2e)                 /* Firmware error code. */
#define PREG_INTERRUPT_STATE ((UInt8)0x2f)       /* Debug-related interrupt state. */
#define PREG_DEBUG_VALUE ((UInt8)0x30)           /* Debug value. */
#define PREG_DEBUG_STATE ((UInt8)0x31)           /* Debug state. */
#define PREG_GP5_B0 ((UInt8)0x32)                /* General purpose registers for communicating information between sensor hub FW and host. */
#define PREG_GP5_B1 ((UInt8)0x33)                /* General purpose registers for communicating information between sensor hub FW and host. */
#define PREG_GP5_B2 ((UInt8)0x34)                /* General purpose registers for communicating information between sensor hub FW and host. */
#define PREG_GP5_B3 ((UInt8)0x35)                /* General purpose registers for communicating information between sensor hub FW and host. */
#define PREG_GP6_B0 ((UInt8)0x36)                /* General purpose registers for communicating information between sensor hub FW and host. */
#define PREG_GP6_B1 ((UInt8)0x37)                /* General purpose registers for communicating information between sensor hub FW and host. */
#define PREG_GP6_B2 ((UInt8)0x38)                /* General purpose registers for communicating information between sensor hub FW and host. */
#define PREG_GP6_B3 ((UInt8)0x39)                /* General purpose registers for communicating information between sensor hub FW and host. */
#define PREG_GP7_B0 ((UInt8)0x3a)                /* General purpose registers for communicating information between sensor hub FW and host. */
#define PREG_GP7_B1 ((UInt8)0x3b)                /* General purpose registers for communicating information between sensor hub FW and host. */
#define PREG_GP7_B2 ((UInt8)0x3c)                /* General purpose registers for communicating information between sensor hub FW and host. */
#define PREG_GP7_B3 ((UInt8)0x3d)                /* General purpose registers for communicating information between sensor hub FW and host. */
#define HREG_TEST ((UInt8)0x3e)                  /*  */

#endif /* !EM7189_HOST_REGS_H */
