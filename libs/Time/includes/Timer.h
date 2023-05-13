////////////////////////////////////////////////////////////////////////////////
///
/// @file       libs/Time/includes/Timer.h
///
/// @project    EM7189
///
/// @brief      Macros for timer access
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
#ifndef _SI_TIMER_H
#define _SI_TIMER_H

#include <SensorAPI.h>
#include <hw_versions.h>
#include <types.h>

#define TIMER_ENTRY_SIZE (24u)

/**
 * @fn void timer_initialize_queue(void* buffer, SInt8 depth)
 *
 * @brief Initializes the timer queue.
 *
 * @param buffer                The buffer to use as a timer queue
 * @param depth                 The number of entries available in the timer queue
 */
//lint -function( fopen(1), timer_initialize_queue(1) ) // arg cannot be null
void timer_initialize_queue(void *buffer, SInt8 depth);


/**
 * @fn SInt8 timer_schedule_polling(float frequency, void (*callback)(SensorStatus, void*), void* data);
 *
 * @brief This function is used to schedule a callback function to be executed at a specified frequency.
 *
 * This functions use should not be used unless a sensor does not support automatic measurements. By
 * scheduling a polling function, a timer must be enabled and current consumption will increase.
 *
 * @param frequency     Specifies the frequency, in Hz, to run the callback function
 * @param callback      The function to call at the specified rate.
 * @param data          Additional data to pass to the callback function.
 *
 * @returns The timer polling index used to access all other timer functions.
 */
SInt8 JLI timer_schedule_polling(float frequency, void (*callback)(SensorStatus, void *), void *data);

/**
 * @fn SInt8 timer_schedule_polling_immediately(float frequency, void (*callback)(SensorStatus, void*), void* data);
 *
 * @brief This function is used to schedule a callback function to be executed at a specified frequency.
 *
 * This functions use should not be used unless a sensor does not support automatic measurements. By
 * scheduling a polling function, a timer must be enabled and current consumption will increase.
 *
 * This function causes the timer to being counting immediately, and will not coalesce with other
 * timers running at the same frequency. The timer_schedule_polling function should be used instead,
 * as it allows for lower power consumption.
 *
 * @param frequency     Specifies the frequency, in Hz, to run the callback function
 * @param callback      The function to call at the specified rate.
 * @param data          Additional data to pass to the callback function.
 *
 * @returns The timer polling index used to access all other timer functions.
 */
SInt8 JLI timer_schedule_polling_immediately(float frequency, void (*callback)(SensorStatus, void *), void *data);

/**
 * @fn void timer_unschedule_polling(SInt8 pollIndex);
 *
 * @brief Removes a previously scheduled callback function.
 *
 * @param pollIndex     The timer index returned from the timer_schedule_polling function.
 */
void JLI timer_unschedule_polling(SInt8 pollIndex);

/**
 * @fn UInt8 timer_update_polling(SInt8 pollIndex, float frequency);
 *
 * @brief This function is used to modify the poll interval of a timer.
 *
 * @param   frequency           Specifies the frequency, in Hz, to run the callback function
 * @param   pollIndex           The already-scheduled timer index to modify.
 *
 * @returns The timer polling index used to access all other timer functions.
 *              If the frequency requested is unsupported, -1 will be returned and
 *              the frequency will not be updated.
 */
SInt8 JLI timer_update_polling(SInt8 pollIndex, float frequency);

/**
 * @fn float timer_get_polling(SInt8 pollIndex);
 *
 * @brief This function is used to retrieve the poll interval of a timer.
 *
 * @param   pollIndex           The already-scheduled timer index to modify.
 *
 * @returns The timer polling rate configured, or 0 if no timer found.
 */
float JLI timer_get_polling(SInt8 pollIndex);

/**
 * @fn void timer_enable_interrupt(SInt8 pollIndex);
 *
 * @brief Begins counting the timer at the specified rate.
 *
 * @param pollIndex     The timer index returned from the timer_schedule_polling function.
 */
void JLI timer_enable_interrupt(SInt8 pollIndex);

/**
 * @fn void timer_disable_interrupt(SInt8 pollIndex);
 *
 * @brief Stops the timer from counting
 *
 * @param pollIndex     The timer index returned from the timer_schedule_polling function.
 */
void JLI timer_disable_interrupt(SInt8 pollIndex);

/**
 * @fn void timer_enabled(SInt8 pollIndex);
 *
 * @brief Determines if the timer is enabled
 *
 * @param pollIndex     The timer index returned from the timer_schedule_polling function.
 */
bool JLI timer_enabled(SInt8 pollIndex);

/**
 * @fn SInt8 timer_oneshot_delay(float milliseconds, void (*callback)(SensorStatus, void*), void* data);
 *
 * @brief This function is used to execute a callback function after the specified time.
 *
 *  Callback function is executed only once.
 *
 * @param   milliseconds        Specifies the delay, in ms, to run the callback function
 * @param   callback            The function to call at the specified rate.
 * @param   data                The data to be passed to the callback function.
 *
 * @retval  -1                  The timer could not be scheduled
 * @retval  Others              The timer was scheduled.
 */
SInt8 JLI timer_oneshot_delay(float milliseconds, void (*callback)(SensorStatus, void *), void *data);

/**
 * @fn void timer_add_delay_coalesced(SInt8 pollIndex, SInt32 delay);
 *
 * @brief Add a one time delay the specified timer by delay timosc ticks.
 *          The specified delay is also added to any coalesced timers.
 *
 * @param pollIndex     The timer index returned from the timer_schedule_polling function.
 * @param delay         The one time delay to add, in timosc ticks.
 */
void JLI timer_add_delay_coalesced(SInt8 pollIndex, SInt32 delay);

/**
 * @fn void timer_add_delay(SInt8 pollIndex, SInt32 delay);
 *
 * @brief Add a one time delay the specified timer by delay timosc ticks.
 *
 * @param pollIndex     The timer index returned from the timer_schedule_polling function.
 * @param delay         The one time delay to add, in timosc ticks.
 */
void JLI timer_add_delay(SInt8 pollIndex, SInt32 delay);

/**
 * @fn float delayms(float milliseconds)
 *
 * @brief This function is used to delay execution for the specified time.
 *
 *  Callback function is executed only once.
 *
 * @deprecated                  The @ref timer_oneshot_delay function should be used instead.
 * @param   milliseconds        Specifies the delay, in ms, before returning.
 *
 * @returns                     The actual amount of time delayed.
 */
float JLI delayms(float milliseconds) DEPRECATED("The timer_oneshot_delay function should be used instead.");

/**
 * @fn float delayums(SInt32 milliseconds)
 *
 * @brief This function is used to delay execution for the specified time.
 *
 *  Callback function is executed only once.
 *
 * @deprecated                  The @ref timer_oneshot_delay function should be used instead.
 * @param   milliseconds        Specifies the delay, in ms, before returning.
 *
 * @returns                     The actual amount of time delayed.
 */
float JLI delayums(SInt32 milliseconds) DEPRECATED("The timer_oneshot_delay function should be used instead.");

/**
 * @fn UInt32 getSystemTime(void);
 *
 * @brief Determines the current system time when the function was called.
 *
 * @returns    The current system time.
 */
SystemTime_t JLI getSystemTime(void);


/**
 * @fn SystemTime_t timestampToSystemTime(TimestampTime_t timestamp);
 *
 * @brief Converts from the timestamp counter time into the full timestamp time. For safety, you may need to call this with interrupts disabled.
 *
 *  Note: This function <b>must</b> be called within 2 seconds of the timestamp being generated.
 *
 * @param      timestamp    The hardware generated timestamp
 *
 * @returns    The system time at the specified timestamp.
 */
SystemTime_t JLI timestampToSystemTime(TimestampTime_t timestamp);

/**
 * @fn SystemTime_t timestampToSystemTime_safe(TimestampTime_t timestamp, SystemTime_t sysTime);
 *
 * @brief Converts from the timestamp counter time into the full timestamp time. This function is safe to call even with interrupts enabled.
 *
 *  Note: This function <b>must</b> be called within 2 seconds of the timestamp being generated.
 *
 * @param      timestamp    The hardware generated timestamp
 * @param      sysTime      The system time to translate to.
 *
 * @returns    The system time at the specified timestamp.
 */
SystemTime_t JLI timestampToSystemTime_safe(TimestampTime_t timestamp, SystemTime_t sysTime);


/**
 * @fn void getBuildTime(UInt16* year, UInt8* month, UInt8* day, UInt8* hour, UInt8* minute, UInt8* second);
 *
 * @brief Returns the build time of the firmware.
 *
 * @param[out]  year    The address to store the year in, or NULL if not needed.
 * @param[out]  month   The address to store the month in, or NULL if not needed.
 * @param[out]  day     The address to store the day in, or NULL if not needed.
 * @param[out]  hour    The address to store the hour in, or NULL if not needed.
 * @param[out]  minute  The address to store the minute in, or NULL if not needed.
 * @param[out]  second  The address to store the second in, or NULL if not needed.
 */
void JLI getBuildTime(UInt16 *year, UInt8 *month, UInt8 *day,
                      UInt8 *hour, UInt8 *minute, UInt8 *second);


/**
 * @brief
 * @return UInt8
 */
UInt8 numTimersEnabled(void);

/**
 * @brief
 * @return UInt8
 */
SInt8 timer_get_free_index(void);


/**
 * @brief
 * @return SInt8
 */
SInt8 timer_get_num_entries(void);


#endif /* _SI_TIMER_H */
