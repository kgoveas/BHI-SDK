////////////////////////////////////////////////////////////////////////////////
///
/// @file       common/7189/includes/debug.h
///
/// @project    EM7189
///
/// @brief      Debug routines.
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

#ifndef _DEBUG_H
#define _DEBUG_H

#include "hif.h"

/**
 * @fn static ALWAYS_INLINE void DebugState(UInt8 state)
 *
 * @param   state Write the specified state to the debug register
 */
//lint -sem( DebugState, 1n < 16) // only 4 bits valid.
//lint -sem( DebugState, 2n < 16) // only 4 bits valid.
static ALWAYS_INLINE void DebugState(UInt8 module, UInt8 state)
{
    PROC.Debug.bits.debug_state = ((module & 0xf) << 4) | (state & 0xf);
}

/**
 * @fn static ALWAYS_INLINE UInt8 GetErrorReason()
 *
 * @brief Reads the specified error reason from the host accessible error register.
 *
 * @returns   The error reason in the error register
 */
static ALWAYS_INLINE UInt8 GetDebugState(void)
{
    return PROC.Debug.bits.debug_state;
}


/**
 * @fn static ALWAYS_INLINE void ErrorReason(UInt8 reason)
 *
 * @brief Writes the specified error reason to the host accessible error register.
 *
 * @param   reason   Write the specified error to the error register
 */
static ALWAYS_INLINE void ErrorReason(UInt8 reason)
{
    PROC.Debug.bits.error = reason;
}


/**
 * @fn static ALWAYS_INLINE UInt8 GetErrorReason()
 *
 * @brief Reads the specified error reason from the host accessible error register.
 *
 * @returns   The error reason in the error register
 */
static ALWAYS_INLINE UInt8 GetErrorReason(void)
{
    return PROC.Debug.bits.error;
}


/**
 * @fn static ALWAYS_INLINE void DebugValue(UInt32 value)
 *
 * @brief Writes the specified debug value into the debug value register. Note that only
 *       the last 8 bits will be readable by the host.
 *
 * @param   value Write the specified value to the debug value register, MSB first
 */
static ALWAYS_INLINE void DebugValue(UInt32 value)
{
    PROC.Debug.bits.debug_value = (UInt8)value;
}


/**
 * @fn static ALWAYS_INLINE UInt8 GetDebugValue()
 *
 * @brief   Retrieves the current debug value
 *
 * @returns The current debug value
 */
static ALWAYS_INLINE UInt8 GetDebugValue(void)
{
    return PROC.Debug.bits.debug_value;
}


/**
 * @fn static ALWAYS_INLINE void InterruptState(UInt8 state)
 *
 * @brief Sets the current interrupt routine state register.
 *
 * @param   state Write the specified state to the interrupt state register
 */
static ALWAYS_INLINE void InterruptState(UInt8 state)
{
    PROC.Debug.bits.interrupt_state = state;
}


/**
 * @fn static ALWAYS_INLINE UInt8 GetInterruptState()
 *
 * @brief   Retrieves the current interrupt routine state
 *
 * @returns The current interrupt state
 */
static ALWAYS_INLINE UInt8 GetInterruptState(void)
{
    return PROC.Debug.bits.interrupt_state;
}


/**
 * @fn static ALWAYS_INLINE UInt8 GetDebugWord
 *
 * @brief   Retrieves the full debug word
 *
 * @returns The current debug word
 */
static ALWAYS_INLINE UInt32 GetDebugWord(void)
{
    return PROC.Debug.r32;
}


#endif /* _DEBUG_H */
