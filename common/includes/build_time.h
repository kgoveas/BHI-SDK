////////////////////////////////////////////////////////////////////////////////
///
/// @file       common/includes/build_time.h
///
/// @project    EM7189
///
/// @brief      Build time of the firmware.
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

#ifndef _BUILD_TIME_H
#define _BUILD_TIME_H

/** @cond */
extern const char _BUILD_YEAR[];
extern const char _BUILD_MONTH[];
extern const char _BUILD_DAY[];
extern const char _BUILD_HOUR[];
extern const char _BUILD_MINUTE[];
extern const char _BUILD_SECOND[];
/** @endcond */

#define BUILD_YEAR      ((UInt16)((UInt32)_BUILD_YEAR))
#define BUILD_MONTH     ((UInt8) ((UInt32) _BUILD_MONTH))
#define BUILD_DAY       ((UInt8) ((UInt32) _BUILD_DAY))
#define BUILD_HOUR      ((UInt8) ((UInt32) _BUILD_HOUR))
#define BUILD_MINUTE    ((UInt8) ((UInt32) _BUILD_MINUTE))
#define BUILD_SECOND    ((UInt8) ((UInt32) _BUILD_SECOND))

#endif
