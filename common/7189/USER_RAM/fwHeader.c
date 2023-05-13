////////////////////////////////////////////////////////////////////////////////
///
/// @file       common/7189/USER_RAM/fwHeader.c
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
#include <types.h>
#include "fwHeader.h"
#include "Ram.h"

extern UInt8 KERNEL_VERSION[];

__attribute__((section(".fw_header_orig")))
const FWHeader_t gFwHeader =
{
    .Magic = FW_MAGIC_VALUE,
    .Version = VERSION,
    .ExpVer = (UInt32)KERNEL_VERSION,
    .ImageFlags =
    {
        .bits =
        {
            .NoExec = 0,
            .EndOfImgChain = 0,
            .NextImgUnsigned = 0,
            .ImgTypeFlash = 0,
            .SSBL = 0,
            .PairWithSSBL = 0,
            .EncryptionSupported = 0,
            .Encrypted = 0,
            .Reserved = 0,
        },
    },
    .maxFwSizeKB = MAX_RAM_FW_SIZE_KB,
}; //lint !e785

extern UInt8 _fw_header_copy_offset[];

__attribute__((section(".fw_header_offset")))
const UInt32 header_copy_offset = (UInt32)_fw_header_copy_offset;

__attribute__((section(".fw_header_copy")))
const FWHeader_t gFwHeader_copy =
{
    .Magic = FW_MAGIC_VALUE,
    .Version = VERSION,
    .ExpVer = (UInt32)KERNEL_VERSION,
    .ImageFlags =
    {
        .bits =
        {
            .NoExec = 0,
            .EndOfImgChain = 0,
            .NextImgUnsigned = 0,
            .ImgTypeFlash = 0,
            .SSBL = 0,
            .PairWithSSBL = 0,
            .EncryptionSupported = 0,
            .Encrypted = 0,
            .Reserved = 0,
        },
    },
    .maxFwSizeKB = MAX_RAM_FW_SIZE_KB,
}; //lint !e785
