////////////////////////////////////////////////////////////////////////////////
///
/// @file       utils/isign/includes/Typedef.h
///
/// @project    EM7189
///
/// @brief      Generic typedef
///
/// @classification  Confidential
///
////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
///
/// @copyright Copyright (C) 2016-2019 EM Microelectronic
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
#ifndef TYPEDEF_H
#define TYPEDEF_H

#include <stdint.h>

/**
 * @defgroup TYPEDEF Generic Typedef
 */
#ifndef PREPACK
    #if defined(_MSC_VER)
    #define PACK
    #define PREPACK __pragma(pack(push, 1))
    #define MIDPACK
    #define POSTPACK __pragma(pack(pop))
    #else
    #define PREPACK
    #define MIDPACK __attribute__((packed))
    #define POSTPACK
    #endif
#endif

/** 8-bit type
* \ingroup TYPEDEF
*/
typedef uint8_t u8;
typedef uint8_t UInt8;

/** 16-bit type
* \ingroup TYPEDEF
*/
typedef uint16_t u16;
typedef uint16_t UInt16;

/** 32-bit type
* \ingroup TYPEDEF
*/
typedef uint32_t UInt32;

/** 64-bit type
* \ingroup TYPEDEF
*/
typedef uint64_t u64;

#endif
