////////////////////////////////////////////////////////////////////////////////
///
/// @file       libs/Hooks/includes/hooks_support.h
///
/// @project    EM7189
///
/// @brief      Hook function prototypes
///
/// @classification  Confidential
///
////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
///
/// @copyright Copyright (C) 2018-2019 EM Microelectronic
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
#ifndef _HOOK_SUPPORT_H
#define _HOOK_SUPPORT_H

#include <types.h>

// Hook priority definition
#define HOOK_PRIORITY_MAX 1  /**< @brief The highest priority for a hook - ensure it runs before built in clients. */
#define HOOK_PRIORITY_ROM 5  /**< @brief The default priority for built in clients. */
#define HOOK_PRIORITY_MIN 10 /**< @brief The lowest priority for a hook - ensure it runs after built in clients. */

/**
 * @cond
 */

#define HOOK_0ARG 4
#define HOOK_1ARG 6
#define HOOK_2ARG 0


// Support definition
#define _CONCAT(__a__, __b__) __a__##__b__
#define CONCAT(__a__, __b__)  _CONCAT(__a__, __b__)

#define _CONCAT_UNDERSCORE(__a__, __b__) __a__##_##__b__
#define CONCAT_UNDERSCORE(__a__, __b__)  _CONCAT_UNDERSCORE(__a__, __b__)

#ifndef STRINGIFY
#define _STRINGIFY(__x__) #__x__
#define STRINGIFY(__x__)  _STRINGIFY(__x__)
#endif

#define ROM_FCNT_NAME(__function__) CONCAT_UNDERSCORE(__function__, ROM_NAME)


// Select proper function header according to number of input arguments of hook function (__VA_ARGS__)
#define HEADER_NAME_N(__name__, n0, n1, n2, n3, __count__, ...) \
    CONCAT(__name__, __count__)

#define HOOK_FCNT_HEADER_NAME(__name__, __hook__, __function__, __return__, ...) \
    HEADER_NAME_N(__name__, ##__VA_ARGS__, _ARG, _ARG, _ARG, _ARG, _ARG_NULL)    \
    (__hook__, __function__, __return__, __VA_ARGS__)

#define HOOK_FCNT_HEADER(__hook__, __function__, __return__, ...) \
    HOOK_FCNT_HEADER_NAME(HOOK_FCNT_HEADER, __hook__, __function__, __return__, ##__VA_ARGS__)


// Header is selected from these two types
#define HOOK_FCNT_HEADER_ARG_NULL(__hook__, __function__, __return__, ...) \
    __return__ __function__(bool *stopHookExecution)

#define HOOK_FCNT_HEADER_ARG(__hook__, __function__, __return__, ...) \
    __return__ __function__(__VA_ARGS__, bool *stopHookExecution)


// Define a hook structure and its pointer
// clang-format off
#define INSTALL_HOOK_STRUCT(__hook__, __function__, __priority__)     \
    structHook CONCAT(structHookItem_, __function__) =                \
    {                                                                 \
        .priority     = __priority__,                                 \
        .hook_pointer = (UInt32(*)(void*, void*, bool*))__function__, \
    }
// clang-format on

#define INSTALL_STRUCT_POINTER(__hook__, __function__) \
    structHook *CONCAT(ptrStructHookItem_, __function__) __attribute__((unused, section(STRINGIFY(CONCAT(.hook_, __hook__))))) = &CONCAT(structHookItem_, __function__)

#define INSTALL_STRUCT_POINTER_AUTO(__hook__, __function__) \
    structHook *CONCAT(ptrStructHookItem_, __function__) __attribute__((unused, section(STRINGIFY(CONCAT(.hookauto_, __hook__))))) = &CONCAT(structHookItem_, __function__)


// Type definition of hook structure
typedef struct
{
    UInt8 priority;
    UInt32 (*hook_pointer)(void *, void *, bool *);
} structHook;

bool callNArgHook(void *arg0, void *arg1, structHook **hookItems, structHook **hookItemEnd, int hook_type);
bool call0ArgHook(void *arg0, void *arg1, structHook **hookItems, structHook **hookItemEnd);
bool call1ArgHook(void *arg0, void *arg1, structHook **hookItems, structHook **hookItemEnd);
bool call2ArgHook(void *arg0, void *arg1, structHook **hookItems, structHook **hookItemEnd);

/**
 * @endcond
 */


/**
 * @def HOOK(__hook__,__function__,__priority__, __return__,...)
 * @brief Define a new hook function and place it in the hook structure
 *
 * @param __hook__      The hook type to register
 * @param __function__  The function to declare and register as a hook client.
 * @param __priority__  The priority of the hook. Smaller values fun first, lower run last. See @ref HOOK_PRIORITY_MAX and @ref HOOK_PRIORITY_MIN
 * @param __return__    The return type of the hook. Please see the documentation on the hook type.
 * @param ...           Remaining arguments for the hook. Please see the documentation on the hook type.
 */
/*lint ++d"HOOK(__hook__,__function__,__priority__,__return__, __args__...)=__return__ __function__(bool* stopHookExecution, ##__args__)" */
#define HOOK(__hook__, __function__, __priority__, __return__, ...)            \
    HOOK_DEF(__hook__, __function__, __priority__, __return__, ##__VA_ARGS__); \
    HOOK_REF(__hook__, __function__, __priority__, __return__, ##__VA_ARGS__); \
    HOOK_DEF(__hook__, __function__, __priority__, __return__, ##__VA_ARGS__)

/**
 * @def HOOK_AUTO(__hook__,__function__,__priority__, __return__,...)
 * @brief Define a new hook function and place it in the hook structure.
 *          This hook is only included in the final binary if it is referenced.
 *          @ref HOOK should be used in most cases instead of HOOK_AUTO.
 *
 * @param __hook__      The hook type to register
 * @param __function__  The function to declare and register as a hook client.
 * @param __priority__  The priority of the hook. Smaller values fun first, lower run last. See @ref HOOK_PRIORITY_MAX and @ref HOOK_PRIORITY_MIN
 * @param __return__    The return type of the hook. Please see the documentation on the hook type.
 * @param ...           Remaining arguments for the hook. Please see the documentation on the hook type.
 */
/*lint ++d"HOOK_AUTO(__hook__,__function__,__priority__,__return__, __args__...)=__return__ __function__(bool* stopHookExecution, ##__args__)" */
#define HOOK_AUTO(__hook__, __function__, __priority__, __return__, ...)            \
    HOOK_DEF(__hook__, __function__, __priority__, __return__, ##__VA_ARGS__);      \
    HOOK_REF_AUTO(__hook__, __function__, __priority__, __return__, ##__VA_ARGS__); \
    HOOK_DEF(__hook__, __function__, __priority__, __return__, ##__VA_ARGS__)

/**
 * @def HOOK_DEF(__hook__,__function__,__priority__,__return__,...)
 * @brief Define a new hook function only
 *
 * @param __hook__      The hook type to register
 * @param __function__  The function to declare and register as a hook client.
 * @param __priority__  The priority of the hook. Smaller values fun first, lower run last. See @ref HOOK_PRIORITY_MAX and @ref HOOK_PRIORITY_MIN
 * @param __return__    The return type of the hook. Please see the documentation on the hook type.
 * @param ...           Remaining arguments for the hook. Please see the documentation on the hook type.
 */
//lint ++d"HOOK_DEF(__hook__,__function__,__priority__,__return__, __args__...)=__return__ __function__(bool* stopHookExecution, ##__args__)"
#define HOOK_DEF(__hook__, __function__, __priority__, __return__, ...) \
    HOOK_FCNT_HEADER(__hook__, __function__, __return__, ##__VA_ARGS__)

/**
 * @def HOOK_REF(__hook__,__function__,__priority__,__return__,...)
 * @brief Define a structure including a reference to a hook function
 *           (e.g. already defined in ROM or other place in RAM)
 *
 * @param __hook__      The hook type to register
 * @param __function__  The function to declare and register as a hook client.
 * @param __priority__  The priority of the hook. Smaller values fun first, lower run last. See @ref HOOK_PRIORITY_MAX and @ref HOOK_PRIORITY_MIN
 * @param __return__    The return type of the hook. Please see the documentation on the hook type.
 * @param ...           Remaining arguments for the hook. Please see the documentation on the hook type.
 */
//lint ++d"HOOK_REF(__hook__,__function__,__priority__,__return__, __args__...)=/*lint -e19 */"
#define HOOK_REF(__hook__, __function__, __priority__, __return__, ...) \
    INSTALL_HOOK_STRUCT(__hook__, __function__, __priority__);          \
    INSTALL_STRUCT_POINTER(__hook__, __function__)


/**
 * @def HOOK_REF_AUTO(__hook__,__function__,__priority__,__return__,...)
 * @brief Define a structure including a reference to a hook function
 *          (e.g. already defined in ROM or other place in RAM)
 *          This hook is only included in the final binary if it is referenced.
 *          @ref HOOK_REF should be used in most cases instead of HOOK_REF_AUTO.
 *
 * @param __hook__      The hook type to register
 * @param __function__  The function to declare and register as a hook client.
 * @param __priority__  The priority of the hook. Smaller values fun first, lower run last. See @ref HOOK_PRIORITY_MAX and @ref HOOK_PRIORITY_MIN
 * @param __return__    The return type of the hook. Please see the documentation on the hook type.
 * @param ...           Remaining arguments for the hook. Please see the documentation on the hook type.
 */
//lint ++d"HOOK_REF(__hook__,__function__,__priority__,__return__, __args__...)=/*lint -e19 */"
#define HOOK_REF_AUTO(__hook__, __function__, __priority__, __return__, ...) \
    INSTALL_HOOK_STRUCT(__hook__, __function__, __priority__);               \
    INSTALL_STRUCT_POINTER_AUTO(__hook__, __function__)
/* ------------------------------------- */
/* Example of using multi-hooks facility */
/* ------------------------------------- */

/**
 * @def HOOK(__hook__,__function__,__priority__, __return__,...)
 * Hook \b PhysicalRate:
 *
 * @par
 * This function is called any time a rate for a physical sensor driver is required. In the event that
 *      the hook implementer needs a different physical sensor rate, it can be overwritten here.
 *
 * @par
 * Usage:
 *      HOOK(PhysicalRate, newHookPhysicalRate, HOOK_PRIORITY_ROM, void, PhysicalSensorDescriptor* phys, float* rate)
 * @param[in]       phys                @ref PhysicalSensorDescriptor*    The physical sensor causing the hook to execute
 * @param[in,out]   rate                     float*                       The current rate selected
 * @param[out]      stopHookExecution   @ref bool*                        If set, stops further hook execution
 * @returns         void
 */

/**
 * @def HOOK(__hook__,__function__,__priority__, __return__,...)
 * Hook \b TimerRate:
 *
 * @par
 * This function is called any time a rate for a physical sensor driver is required. In the event that
 *      the hook implementer needs a different physical sensor rate, it can be overwritten here.
 *
 * @par
 * Usage:
 *      HOOK(TimerRate, newHookTimerRate, HOOK_PRIORITY_ROM, void, PhysicalSensorDescriptor* timer, float* rate)
 * @param[in]       timer                @ref VirtualSensorDescriptor*   The timer sensor causing the hook to execute
 * @param[in,out]   rate                      float*                     The current rate selected
 * @param[out]      stopHookExecution    @ref bool*                      If set, stops further hook execution
 * @returns         void
 */

/**
 * @def HOOK(__hook__,__function__,__priority__, __return__,...)
 * Hook \b PhysicalRateChanged:
 *
 * @par
 * This function is called any time a rate for a physical sensor driver has changed.
 *
 * @par
 * Usage:
 *      HOOK(PhysicalRateChanged, newHookPhysicalRateChanged, HOOK_PRIORITY_ROM, void, PhysicalSensorDescriptor* phys, float* rate)
 * @param[in]       phys                 @ref PhysicalSensorDescriptor*  The physical sensor causing the hook to execute
 * @param[in]       rate                      float                      The current rate of the physical sensor
 * @param[out]      stopHookExecution    @ref bool*                      If set, stops further hook execution
 * @returns         void
 */

/**
 * @def HOOK(__hook__,__function__,__priority__, __return__,...)
 * Hook \b PhysicalRangeChanged:
 *
 * @par
 * This function is called any time a range for a physical sensor driver has changed.
 *
 * @par
 * Usage:
 *      HOOK(PhysicalRangeChanged, newHookPhysicalRangeChanged, HOOK_PRIORITY_ROM, void, PhysicalSensorDescriptor* phys, UInt16 range)
 * @param[in]       phys                 @ref PhysicalSensorDescriptor*  The physical sensor causing the hook to execute
 * @param[in]       range                @ref UInt16                     The current range of the physical sensor
 * @param[out]      stopHookExecution    @ref bool*                      If set, stops further hook execution
 * @returns         void
 */

/**
 * @def HOOK(__hook__,__function__,__priority__, __return__,...)
 * Hook \b VirtualSensorsDetermined:
 *
 * @par
 * Function called to notify listeners that the host requested virtual drivers
 *  have changed. A client may override a request from the host by disabling a
 *  sensor using @ref disableSensor or may enable a needed sensor by
 *  calling @ref enableSensor.
 *
 * @par
 * Usage:
 *      HOOK(VirtualSensorsDetermined, newHookVirtualSensorsDetermined, HOOK_PRIORITY_ROM, void)
 * @param[out]      stopHookExecution    @ref bool*                      If set, stops further hook execution
 * @returns         void
 */

/**
 * @def HOOK(__hook__,__function__,__priority__, __return__,...)
 * Hook \b OverrideMaxRate:
 *
 * @par
 * This function is called any time a max rate for a physical sensor driver is required. In the event that
 *      the hook implementer needs a different maximal physical sensor rate, it can be overwritten here.
 *
 * @par
 * Usage:
 *      HOOK(OverrideMaxRate, newHookOverrideMaxRate, HOOK_PRIORITY_ROM, void, SensorDescriptorHeader* sensor, UInt16* rate)
 * @param[in]       sensor               @ref PhysicalSensorDescriptor*  The physical sensor causing the hook to execute
 * @param[in,out]   rate                 @ref UInt16*                    The current range of the physical sensor
 * @param[out]      stopHookExecution    @ref bool*                      If set, stops further hook execution
 * @returns         void
 */

/**
 * @def HOOK(__hook__,__function__,__priority__, __return__,...)
 * Hook \b updatePhysicalState:
 *
 * @par
 * This function is called any time a state of physical sensor driver is changed. In the event that
 *      the hook implementer needs a different state of physical sensor rate, it can be done here.
 *      This may be called even if no actual transitions occurred.
 *
 * @par
 * Usage:
 *      HOOK(updatePhysicalState, newHookUpdatePhysicalState, HOOK_PRIORITY_ROM, void, PhysicalSensorDescriptor* phys)
 * @param[in]       phys                 @ref PhysicalSensorDescriptor*  The physical sensor causing the hook to execute
 * @param[out]      stopHookExecution    @ref bool*                      If set, stops further hook execution
 * @returns         void
 */


/**
 * @def HOOK(__hook__,__function__,__priority__, __return__,...)
 * Hook \b determinePowerState:
 *
 * @par
 * This function is called any time a power state of a physical sensor driver will be updated.
 *   This hook allows for a client to specify the correct power state to transition to.
 *   This, for example, can allow a client to use a lower power, but higher noise, mode
 *   when requesting sample data. The return value of this hook will be passed directly
 *   to the sensor driver if it's different from the current state.
 *
 * @par
 * Usage:
 *      HOOK(determinePowerState, newHookDeterminePowerState, HOOK_PRIORITY_ROM, SensorPowerMode, PhysicalSensorDescriptor* phys)
 * @param[in]       phys                 @ref PhysicalSensorDescriptor*  The physical sensor causing the hook to execute
 * @param[out]      stopHookExecution    @ref bool*                     If set, stops further hook execution
 * @returns         @ref SensorPowerMode                                The power mode requested for this sensor driver
 */

/**
 * @def HOOK(__hook__,__function__,__priority__, __return__,...)
 * Hook \b initOnce:
 *
 * @par
 * This function is called only after the system has begun execution.
 *
 * @par
 * Usage:
 *      HOOK(initOnce, newHookInitOnce, HOOK_PRIORITY_ROM, void)
 * @param[out]      stopHookExecution    @ref bool*                     If set, stops further hook execution
 */

/**
 * @def HOOK(__hook__,__function__,__priority__, __return__,...)
 * Hook \b initialize:
 *
 * @par
 * Any time the system successfully begins execution of the main algorithm,
 *  this function is run. The function runs at the main execution priority level.
 *  Custom code here should be used to initialize items such as timer callbacks,
 *  sensor drivers, and data structures needed.
 *
 * @par
 * Usage:
 *      HOOK(initialize, newHookInitialize, HOOK_PRIORITY_ROM, void)
 * @param[out]      stopHookExecution    @ref bool*                     If set, stops further hook execution
 */

/**
 * @def HOOK(__hook__,__function__,__priority__, __return__,...)
 * Hook \b exitShutdown:
 *
 * @par
 * Any time the system successfully begins execution of the main algorithm,
 *  this function is run. The function runs at the main execution priority level.
 *  Custom code here should be used to initialize items such as timer callbacks,
 *  sensor drivers, and data structures needed.
 *
 * @par
 * Usage:
 *      HOOK(exitShutdown, newHookExitShutdown, HOOK_PRIORITY_ROM, void)
 * @param[out]      stopHookExecution    @ref bool*                     If set, stops further hook execution
 */

/**
 * @def HOOK(__hook__,__function__,__priority__, __return__,...)
 * Hook \b teardown:
 *
 * @par
 * Any time the system decides that the algorithm needs to stop,
 *  this function is run. This can be after an unsuccessful initialization
 *  (NOTE: \b initialize will not have been called), if the host has
 *  requested a shutdown or a halt, or if an internal error occurred. This
 *  function should be written in a way that disables timers, sensors, and
 *  allows the system to enter a low power shutdown state.
 *
 * @par
 * Usage:
 *      HOOK(teardown, newHookTeardown, HOOK_PRIORITY_ROM, void)
 * @param[out]      stopHookExecution    @ref bool*                     If set, stops further hook execution
 */

/** -------------------------------------- **/


#endif /* _HOOK_SUPPORT_H */
