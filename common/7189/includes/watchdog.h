////////////////////////////////////////////////////////////////////////////////
///
/// @file       common/7189/includes/watchdog.h
///
/// @project    EM7189
///
/// @brief      Routines for accessing the watchdog.
///
/// @classification  Confidential
///
////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
///
/// @copyright Copyright (C) 2015-2018 EM Microelectronic
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

#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <arc.h>
#include <arc_reg.h>


#if defined(__CCAC__)
/**
 * @brief The watchdog password register is used to access the control and
 *          period registers.
 *
 * Three passwords are defined. Two of the passwords unlocks the respective
 *      watchdog timer register for write access.
 *
 * Attempting to read from the register generates an illegal instruction
 *      exception.
 */
static volatile __aux(WDT_PASSWD) UInt32 AR_WDT_PASSWD;
#endif

/**
 * @brief Password that lets you automatically load the value in the
 *      @ref AR_WDT_PERIOD register to the watchdog timer counter.
 */
#define WDT_PASSWORD_KICK       (0x5A)

/**
 * @brief Password to unlock @ref AR_WDT_PERIOD for write access.
 */
#define WDT_PASSWORD_PERIOD     (0x55)

/**
 * @brief Password to unlock @ref AR_WDT_CTRL for write access.
 */
#define WDT_PASSWORD_CTRL       (0xAA)

/**
 * @brief The watchdog control register is used to configure the timer.
 */
typedef union
{
    struct {
        /**
         * @brief Enable watchdog timer
         */
        UInt32 E:1;

        /** @brief Specifies the action the watchdog timer takes when the
         *          counter reaches zero.
         *
         *  1: Raise an interrupt (irq number 18).
         *  2: Trigger a system reset.
         */
        UInt32 EVENT:2;

        /**
         * @brief Indicates if a time out has occurred.
         *
         *  1: the watchdog internal counter has reached the value 0 (timed out).
         *      Can be written to clear the flag.
         *  0: For an interrupt configured event, writing a 0 to this bit clears
         *      the interrupt
         */
        UInt32 F:1;

        UInt32 reserved:28;
    };

    UInt32 r32;
} WDTControl_t;

#if defined(__CCAC__)
/**
 * @brief The watchdog control register is used to configure the timer.
 */
static volatile __aux(WDT_CTRL) UInt32 AR_WDT_CTRL;


/**
 * @brief This register stores the start value of the watchdog-timer counter.
 */
static volatile __aux(WDT_PERIOD) UInt32 AR_WDT_PERIOD;

/**
 * @brief The watchdog timer count register holds the current value of the
 *      internal counter.
 *
 * Attempting to write to the register generates an illegal instruction
 *      exception.
 */
static volatile __aux(WDT_COUNT) UInt32 AR_WDT_COUNT;
#endif

/**
 * @fn      void ClearWatchdog();
 * @brief   Resets the watchdog counter.
 */
static void ALWAYS_INLINE ClearWatchdog(void)
{
#if defined(__CCAC__)
    AR_WDT_PASSWD = WDT_PASSWORD_KICK;
#else
    writeAUX(WDT_PASSWORD_KICK, (void*)WDT_PASSWD);
#endif
}

/**
 * @fn      void EnableWatchdog();
 * @brief   Enables the system watchdog.
 */
static void ALWAYS_INLINE EnableWatchdog(void)
{
    WDTControl_t ctrl;
    ctrl.E = 1;     // Enable
    ctrl.EVENT = 2; // Output reset req to reset manager.
    ctrl.F = 0;     // Reload
#if defined(__CCAC__)
    AR_WDT_PASSWD = WDT_PASSWORD_CTRL;
    AR_WDT_CTRL = ctrl.r32;
#else
    writeAUX(WDT_PASSWORD_CTRL, (void*)WDT_PASSWD);
    writeAUX(ctrl.r32, (void*)WDT_CTRL);
#endif
}

/**
 * @fn      void EnableWatchdogInterrupt();
 * @brief   Enables the system watchdog so that it generates an interrupt, and does not reset.
 */
static void ALWAYS_INLINE EnableWatchdogInterrupt(void)
{
    WDTControl_t ctrl;
    ctrl.E = 1;     // Enable
    ctrl.EVENT = 1; // raise IRQ18.
    ctrl.F = 0;     // Reload

#if defined(__CCAC__)
    AR_WDT_PASSWD = WDT_PASSWORD_CTRL;
    AR_WDT_CTRL = ctrl.r32;
#else
    writeAUX(WDT_PASSWORD_CTRL, (void*)WDT_PASSWD);
    writeAUX(ctrl.r32, (void*)WDT_CTRL);
#endif
}

/**
 * @fn      void DisableWatchdog();
 * @brief   Disables the watchdog counter.
 */
static void ALWAYS_INLINE DisableWatchdog(void)
{
    WDTControl_t ctrl;
    ctrl.r32 = 0;
    //ctrl.E = 0;     // Disable

#if defined(__CCAC__)
    AR_WDT_PASSWD = WDT_PASSWORD_CTRL;
    AR_WDT_CTRL = ctrl.r32;
#else
    writeAUX(WDT_PASSWORD_CTRL, (void*)WDT_PASSWD);
    writeAUX(ctrl.r32, (void*)WDT_CTRL);
#endif
}

/**
 * @fn      void SetWatchdogLimit(UInt16 limit);
 * @brief   Updates the watchdog limit. This should only be called with a disabled watchdog.
 *
 */
static void ALWAYS_INLINE SetWatchdogLimit(UInt32 limit)
{
#if defined(__CCAC__)
    AR_WDT_PASSWD = WDT_PASSWORD_PERIOD;
    AR_WDT_PERIOD = limit;
#else
    writeAUX(WDT_PASSWORD_PERIOD, (void*)WDT_PASSWD);
    writeAUX(limit, (void*)WDT_PERIOD);
#endif
}

#endif /* ! WATCHDOG_H */
