////////////////////////////////////////////////////////////////////////////////
///
/// @file       common/7189/includes/aux_registers.h
///
/// @project    EM7189
///
/// @brief      Bit fields for AUX registers.
///
/// @classification  Confidential
///
////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
///
/// @copyright Copyright (C) 2013-2018 EM Microelectronic
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

#ifndef AUX_REGISTERS_H
#define AUX_REGISTERS_H

#include <types.h>
#include <arc_reg.h>
#include "eiaextensions.h"

#if _ARCVER /* Must be compiling for the arc */

/**
 * @fn      static UInt32 ALWAYS_INLINE readAUX_unsafe(void* reg)
 * @brief   Used to read registers in the Aux register space. Does not
 *              double-read if needed
 *
 * This function is a wrapper around _lr.
 * Unless absolutely needed, use readAUX instead. This does not guarantee host
 *      writable registers are read properly.
 *
 * @param   reg     The register address to read
 * @returns         The value in the specified register.
 */
static UInt32 ALWAYS_INLINE readAUX_unsafe(void* reg)
{
#if defined(__CCAC__)
    return (UInt32)_lr((UInt32)reg);
#elif defined(__GNUC__) || defined(__GNUG__)
    return __builtin_arc_lr((UInt32)reg);
#else
#error Unknown compiler.
#endif
}

/**
 * @fn      static UInt32 ALWAYS_INLINE readAUX_safe(void* reg)
 * @brief   Used to read registers in the Aux register space. This always
 *              double-reads, even if not needed.
 *
 * This function is a wrapper around _lr.
 * Only use *if* reg is not constant. Always double reads the register, even if
 *      not needed.
 *
 * @param   reg     The register address to read
 * @returns         The value in the specified register.
 */
static UInt32 ALWAYS_INLINE readAUX_safe(void* reg)
{
    UInt32 firstRead;
    UInt32 secondRead;
    do
    {
        // These registers have a possibility of instability (not on same clock domain).
        // We don't have any protection on these, so we need to read them out twice and verify
#if defined(__CCAC__)
        firstRead = (UInt32)_lr((UInt32)reg);
        secondRead = (UInt32)_lr((UInt32)reg);
#elif defined(__GNUC__) || defined(__GNUG__)
        firstRead = __builtin_arc_lr((UInt32)reg);
        secondRead = __builtin_arc_lr((UInt32)reg);
#else
#error Unknown compiler.
#endif
    } while(firstRead != secondRead);

    return firstRead;
}

/**
 * @fn      static UInt32 ALWAYS_INLINE readAUX(void* reg)
 * @brief   Used to read registers in the Aux register space.
 *
 * This function is a wrapper around _lr.
 * If the register is a host writable register, we read twice to ensure validity
 *
 * @param   reg     The register address to read
 * @returns         The value in the specified register.
 */
static UInt32 ALWAYS_INLINE readAUX(void* reg)
{
    return readAUX_unsafe(reg);
}

/**
 * @fn      static void ALWAYS_INLINE writeAUX(UInt32 value, void* reg)
 * @brief   Used to write registers in the Aux register space.
 *
 * This function is a wrapper around _sr.
 *
 * @param   value   The value to write to the register.
 * @param   reg     The register address to write to.
 */
static void ALWAYS_INLINE writeAUX(UInt32 value, void* reg)
{
#if defined(__CCAC__)
    _sr((int)value, (UInt32)reg);
#elif defined(__GNUC__) || defined(__GNUG__)
    __builtin_arc_sr(value, (UInt32)reg);
#else
#error Unknown compiler.
#endif
}

#endif /* _ARCVER */

#endif /* AUX_REGISTERS_H */
