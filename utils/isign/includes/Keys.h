////////////////////////////////////////////////////////////////////////////////
///
/// @file       libs/Security/includes/Keys.h
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
#ifndef KEYS_H
#define KEYS_H

#include "ECC_Typedef.h"

#define KEY_MAGIC 0xce42
#define MAX_PUB_KEYS 2

PREPACK
typedef struct
{
    POINT    key; // 320 bit public key.
    uint32_t crc;
} MIDPACK Key_t;
POSTPACK

PREPACK
typedef struct
{
    uint16_t keyMagic;
    uint8_t  numKeys;
    uint8_t  reserved;
} MIDPACK KeyListHdr_t;
POSTPACK

PREPACK
typedef struct
{
    KeyListHdr_t hdr;
    Key_t        keys[MAX_PUB_KEYS];
} MIDPACK KeyList_t;
POSTPACK

extern KeyList_t gRomKeys;

#endif
