////////////////////////////////////////////////////////////////////////////////
///
/// @file       common/7189/includes/arc.h
///
/// @project    EM7189
///
/// @brief      Main arc include file, standard macros for
///             manipulating interrupts, registers, and cpu states.
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

#ifndef _ARC_H
#define _ARC_H

#if __ASM__
.include "../7189/includes/i2cm.s"
.include "../7189/includes/spim.s"
.include "../7189/includes/hif.s"
.include "../7189/includes/peripherals.s"

.if 0
#endif

#include <hw_versions.h>
#include <types.h>

#include "i2cm.h"
#include "spim.h"
#include "hif.h"
#include "dma.h"
#include "peripherals.h"
#include "em7189_host_regs.h"

#include <aux_registers.h>
#include <eiaextensions.h>

/** @cond */
#define SLEEP_MODE_LSLP     (0x03u)
#define SLEEP_MODE_SLP      (0x05u)
#define SLEEP_MODE_DSLP     (0x07u)
#define SLEEP_MODE_BITSHIFT (5u)
/** @endcond */

#if _ARCVER /* Only valid when compiling for the arc. */
/**
 * @fn      void HaltCPU();
 * @brief   Cause the CPU to enter a HALT state.
 *
 * NOTE: The ARC will ignore the halt instruction if the CPURunReq bit is set.
 */
static void ALWAYS_INLINE HaltCPU(void)
{
    /* Set the CPU Halt bit */
#if _ARCVER == ARCV2_EM4_CORE2
    __asm__("kflag  1");
#else
#error Unknown CPU version _ARCVER
#endif
}


/**
 * @fn      void SleepCPU();
 * @brief   Cause the CPU to enter the sleep state.
 * SYSOSC is disabled, but TMROSC remains in its previous state.
 *
 * NOTE: This function does not modify interrupt enables.
 */
static void ALWAYS_INLINE SleepCPU(void)
{
    /* Issue a sleep instruction */
#if _ARCVER == ARCV2_EM4_CORE2

#include "dma.h"
    if(2 == PROC.Idinfo.bits.rev_id)
    {
        // DI02
        __asm__("clri\n\t\n" \
                "sync"             // serializing instruction, flush the pipeline before continuing (ensure interrupts are really disabled)
                ::: "memory");
        // We need to use light slepe instead of regular sleep if the DMA is enabled.
        UInt32 cenb;

        #if defined(__CCAC__)
            cenb = AR_DMACENB;
        #else
            cenb = readAUX((void*)DMACENB);
        #endif

        if ((cenb & 0x0e) == 0)
        {
            // DMA is disabled, use regular sleep mode.
            // Don't change interrupt enables.
            __asm__("sleep %0" : : "c"(SLEEP_MODE_SLP << SLEEP_MODE_BITSHIFT));
        }
        else
        {
            // DMA is enabled, use light sleep mode.
            __asm__("sleep %0" : : "c"(SLEEP_MODE_LSLP << SLEEP_MODE_BITSHIFT));
        }
    }
    else
    {
        // DI03, use regular sleep.
        // Don't change interrupt enables.
        __asm__("sleep %0" : : "c"(SLEEP_MODE_SLP << SLEEP_MODE_BITSHIFT));
    }

#else
#error Unknown CPU version _ARCVER
#endif
}

/**
 * @fn      void LightSleepCPU();
 * @brief   Cause the CPU to enter the light sleep state.
 * SYSOSC and TMROSC remain running.
 *
 * NOTE: This function does not modify interrupt enables.
 */
static void ALWAYS_INLINE LightSleepCPU(void)
{
    /* Issue a sleep instruction */
#if _ARCVER == ARCV2_EM4_CORE2
    // allow interrupt levels 1 and 2 to wake us up.
    //__asm__("sleep    0x12");

    // Don't change interrupt enables.
    __asm__("sleep %0" : : "c"(SLEEP_MODE_LSLP << SLEEP_MODE_BITSHIFT));
#else
#error Unknown CPU version _ARCVER
#endif
}


/**
 * @fn      void DeepSleepCPU();
 * @brief   Cause the CPU to enter the deep sleep state.
 * SYSOSC and TMROSC are disabled.  A qualified interrupt will
 * re-enable SYSOSC and then resume execution.
 *
 * NOTE: This function does not modify interrupt enables.
 */
static void ALWAYS_INLINE DeepSleepCPU(void)
{
    /* Issue a sleep instruction */
#if _ARCVER == ARCV2_EM4_CORE2
    // allow interrupt levels 1 and 2 to wake us up.
    //__asm__("sleep    0x12");

    // Don't change interrupt enables.
    __asm__("sleep %0" : : "c"(SLEEP_MODE_DSLP << SLEEP_MODE_BITSHIFT));
#else
#error Unknown CPU version _ARCVER
#endif
}

/**
 * @fn      void SleepAndEnableInterrupts();
 * @brief   Cause the CPU to enter a sleep state.
 *
 * NOTE: This function enables all interrupts.
 */
static void ALWAYS_INLINE SleepAndEnableInterrupts(void)
{
#if defined(DEBUG_TIME_SPENT_WITH_INTERRUPTS_OFF)
    PAD.Gpiooutd.r32 = (1 << 4);
    PAD.Gpiodout.r32 |= (1 << 4);
#endif
#if (_ARCVER == ARCV2_EM4_CORE2)
#include "dma.h"
    if(2 == PROC.Idinfo.bits.rev_id)
    {
        // DI02
        __asm__("clri\n\t\n" \
                "sync"             // serializing instruction, flush the pipeline before continuing (ensure interrupts are really disabled)
                ::: "memory");
        // DMA/sleep fix -- if at least one of channels 1-3 is enabled, don't sleep
        UInt32 cenb;

        #if defined(__CCAC__)
            cenb = AR_DMACENB;
        #else
            cenb = readAUX((void*)DMACENB);
        #endif

        if ((cenb & 0x0e) == 0)
        {
            // DMA is disabled, use regular sleep
            __asm__("sleep  %0" :: "c" (0x17 | SLEEP_MODE_SLP << SLEEP_MODE_BITSHIFT));
        }
        else
        {
            // DMA is enabled, use light sleep.
            __asm__("sleep  %0" :: "c" (0x17 | SLEEP_MODE_LSLP << SLEEP_MODE_BITSHIFT));
        }
    }
    else
    {
        // DI03
        // allow all interrupt levels
        __asm__("sleep  %0" :: "c" (0x17 | SLEEP_MODE_SLP << SLEEP_MODE_BITSHIFT));
    }
#else
#error Unknown CPU version _ARCVER
#endif
}



/**
 * @fn      void hasStandbyRequest();
 * @brief   Determines if the host has requested that we
 *        enter the standby state.
 *
 * NOTE: This function does not modify interrupt enables.
 */
static bool ALWAYS_INLINE hasStandbyRequest(void)
{
    return 0;
}

/**
 * @fn      UInt32 safeRead32(volatile UInt32* reg);
 * @brief   Safely reads host-write reigsters that have the possability of metastability.
 *
 * @returns The read out value.
 */
static UInt32 ALWAYS_INLINE safeRead32(volatile UInt32* reg)
{
    // TODO/FIXME: ensure HIF code properly calls this instead of reading registers directly.
    UInt32 val1;
    UInt32 val2;
    do
    {
        val1 = *reg;
        val2 = *reg;
        _nop();
    } while(val1 != val2);

    return val1;
}

/**
 * @fn      UInt16 safeRead16(volatile UInt16* reg);
 * @brief   Safely reads host-write reigsters that have the possability of metastability.
 *
 * @returns The read out value.
 */
static UInt16 ALWAYS_INLINE safeRead16(volatile UInt16* reg)
{
    // TODO/FIXME: ensure HIF code properly calls this instead of reading registers directly.
    UInt16 val1;
    UInt16 val2;
    do
    {
        val1 = *reg;
        val2 = *reg;
        _nop();
    } while(val1 != val2);

    return val1;
}

/**
 * @fn      UInt8 safeRead8(volatile UInt8* reg);
 * @brief   Safely reads host-write reigsters that have the possability of metastability.
 *
 * @returns The read out value.
 */
static UInt8 ALWAYS_INLINE safeRead8(volatile UInt8* reg)
{
    // TODO/FIXME: ensure HIF code properly calls this instead of reading registers directly.
    UInt8 val1;
    UInt8 val2;
    do
    {
        val1 = *reg;
        val2 = *reg;
        _nop();
    } while(val1 != val2);

    return val1;
}


/**
 * @fn      void trap(UInt8 trap_id);
 * @brief   Issue a trap request to the kernel.
 */
static void ALWAYS_INLINE trap(UInt8 trap_id)
{
    /* Issue a trap instruction */
#if _ARCVER == ARCV2_EM4_CORE2

    // Don't change interrupt enables.
    __asm__("trap_s %0" : : "c"(trap_id));
#else
#error Unknown CPU version _ARCVER
#endif
}

#include "watchdog.h"

#endif /* _ARCVER */

#include <timers.h>
#include <gpio.h>
#include <interrupts.h>
#include <debug.h>

#if __ASM__
.endif
#endif
#endif /* _ARC_H */
