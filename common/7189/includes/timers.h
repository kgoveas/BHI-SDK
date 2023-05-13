////////////////////////////////////////////////////////////////////////////////
///
/// @file       common/7189/includes/timers.h
///
/// @project    EM7189
///
/// @brief      Macros for timer access.
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

#ifndef _TIMER_H
#define _TIMER_H


#include <aux_registers.h>
#include <arc.h>
#include <eiaextensions.h>  /* added by vtu */

#define ITMR_INDEX      (2u)    /**< @brief The interval timer id. */
#define UTMR_INDEX      (4u)    /**< @brief The universal timer id. */

#if defined(_ARCVER) || defined(_FRAMEWORK)

#define SYSOSC_FREQUENCY_LONGRUN_KHZ        (20000)
#define SYSOSC_FREQUENCY_TURBO_KHZ          (50000)

/**
 * @fn static UInt32 ALWAYS_INLINE getSYSOSCFrequency(void);
 *
 * @brief Determine the frequency, in kHz, of the system oscillator.
 *
 * @returns The frequency of the osc in kHz.
 */
static UInt32 ALWAYS_INLINE getSYSOSCFrequency(void)
{
    if(CHP.Powerconfig.bits.run_lvl == CHP_POWERCONFIG_RUN_LVL_50MHZ_AT_1_1V)
    {
        return SYSOSC_FREQUENCY_TURBO_KHZ;
    }
    else
    {
        return SYSOSC_FREQUENCY_LONGRUN_KHZ;
    }
}

/**
 * @fn static UInt32 ALWAYS_INLINE getSYSOSCTime(void)
 *
 * @brief Returns the current count of the system oscillator timer.
 *
 * Note: This timer is may not count when in sleep mode.
 * Note: The timer wrap point may be undefined.
 * Note: getSystemTime should be used for timing whenever possible.
 *
 * @returns A 32 bit count.
 */
static UInt32 ALWAYS_INLINE DEPRECATED("The getSystemTime function should be used instead.") getSYSOSCTime(void)
{
    return readAUX((void*)REG_COUNT0);
}

/**
 * @fn static UInt32 ALWAYS_INLINE getTIMOSCFrequency(void);
 *
 * @brief Determine the frequency, in kHz, of the timer oscillator.
 *
 * @returns The frequency of the osc in kHz. This always returns 64KHz.
 */
static UInt32 ALWAYS_INLINE getTIMOSCFrequency(void)
{
    return 64;
}


/**
 * @fn static void ALWAYS_INLINE setTIMOSCFrequency(UInt8 khz);
 *
 * @brief Set the frequency, in kHz, of the timer oscillator.
 *
 * @param   khz The frequency, in kHz, of the oscillator. Unused.
 */
static void ALWAYS_INLINE setTIMOSCFrequency(UInt8 khz)
{
    UNUSED(khz);
}


/**
 * @fn static void ALWAYS_INLINE setTimerLimit(UInt8 timer, UInt32 limit);
 *
 * @brief Set the timer limit, in counts, of the specified timer.
 *
 * @param   timer   The hardware timer (2 or 4) to modify.
 * @param   limit   The new limit for the specified timer.
 */
static void ALWAYS_INLINE setTimerLimit(UInt8 timer, UInt32 limit)
{
    switch(timer)
    {
        case ITMR_INDEX:
            ITMR.Limit.r32 = limit;
            break;

        case UTMR_INDEX:
            UTMR.Limit.r32 = limit;
            break;

        default:
            break;
    }
}


/**
 * @fn static UInt32 ALWAYS_INLINE getTimerLimit(UInt8 timer);
 *
 * @brief Get the timer limit, in counts, of the specified timer.
 *
 * @param   timer   The hardware timer (2 or 4) to read.
 *
 * @returns The current limit of the specified timer.
 */
static UInt32 ALWAYS_INLINE getTimerLimit(UInt8 timer)
{
    UInt32 limit;
    switch(timer)
    {
        case ITMR_INDEX:
            limit = ITMR.Limit.r32;
            break;

        case UTMR_INDEX:
            limit = UTMR.Limit.r32;
            break;

        default:
            /** Unknown Timer **/
            limit = 0;
            break;
    }

    return limit;
}


/**
 * @fn static UInt32 ALWAYS_INLINE getTimerCount(UInt8 timer);
 *
 * @brief Get the current count of the specified timer.
 *
 * @param   timer   The hardware timer (2 or 4) to read.
 *
 * @returns The current count of the specified timer.
 */
static UInt32 ALWAYS_INLINE getTimerCount(UInt8 timer)
{
    UInt32 count;

    switch(timer)
    {
        case ITMR_INDEX:
            count = safeRead32(&ITMR.Value.r32);
            break;

        case UTMR_INDEX:
            count = safeRead32(&UTMR.Value.r32); //NOTE: this is presently 16bit.
            break;

        default:
            return 0;
    }

    return count;
}


/**
 * @fn static void ALWAYS_INLINE clearTimer(UInt8 timer);
 *
 * @brief Reset the specified timer's count.
 *
 * @param   timer   The hardware timer (2 or 4) to read.
 */
static void ALWAYS_INLINE clearTimer(UInt8 timer)
{
    switch(timer)
    {
        case ITMR_INDEX:
            ITMR.Control.bits.itmr_clr = 1;
            break;

        case UTMR_INDEX:
            /*
            RegTimer4Ctrl ctrl4;
            ctrl4.reg = readAUX(AR_TIMER4_CTRL);
            ctrl4.bits.Clear = 1;
            writeAUX(ctrl4.reg, AR_TIMER4_CTRL);
            */
            break;

        default:
            break;
    }
}

/**
 * @fn static void ALWAYS_INLINE enableTimer(UInt8 timer);
 *
 * @brief Enable the specified timer.
 *
 * @param   timer   The hardware timer (2 or 4) to enable.
 */
static void ALWAYS_INLINE enableTimer(UInt8 timer)
{
    switch(timer)
    {
        case ITMR_INDEX:
            ITMR.Control.bits.itmr_en = 1;
            break;

        case UTMR_INDEX:
            UTMR.Control.bits.utmr_clk_src = 1;
            UTMR.Control.bits.utmr_en = 1;
            UTMR.Control.bits.utmr_sw_start = 1;
            break;

        default:
            break;
    }
}

/**
 * @fn static bool ALWAYS_INLINE enabledTimer(UInt8 timer);
 *
 * @brief Determine if the specified timer has been enabled.
 *
 * @param   timer   The hardware timer (2 or 4) to enable.
 */
static bool ALWAYS_INLINE enabledTimer(UInt8 timer)
{
    switch(timer)
    {
        case ITMR_INDEX:
            return ITMR.Control.bits.itmr_en == 1;

        case UTMR_INDEX:
            return UTMR.Control.bits.utmr_en == 1;

        default:
            return 0;
    }
}


/**
 * @fn static void ALWAYS_INLINE disableTimer(UInt8 timer);
 *
 * @brief Disable the specified timer.
 *
 * @param   timer   The hardware timer (2 or 4) to disable.
 */
static void ALWAYS_INLINE disableTimer(UInt8 timer)
{
    switch(timer)
    {
        case ITMR_INDEX:
            ITMR.Control.bits.itmr_en = 0;
            break;

        case UTMR_INDEX:
            UTMR.Control.bits.utmr_en = 0;
            break;

        default:
            break;
    }
}

#endif /* _ARCVER || _FRAMEWORK */
#endif /* _TIMER_H */
