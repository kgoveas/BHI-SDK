////////////////////////////////////////////////////////////////////////////////
///
/// @file       libs/OuterloopROM/includes/hooks.h
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
#ifndef _HOOKS_H
#define _HOOKS_H

#include <types.h>

/**
 * @fn void hook_initOnce(void);
 *
 * @brief This function is used to initialize user specific code and drivers.
 *
 * This function is called only after the system has begun execution.
 */
void hook_initOnce(void);

/**
 * @fn void hook_initialize(void);
 *
 * @brief This function is used to initialize user specific code and drivers.
 *
 * Any time the system successfully begins execution of the main algorithm,
 *  this function is run. The function runs at the main execution priority level.
 *  Custom code here should be used to initialize items such as timer callbacks,
 *  sensor drivers, and data structures needed.
 */
void hook_initialize(void);

/**
 * @fn void hook_exitShutdown(void);
 *
 * @brief This function is used to initialize user specific code and drivers.
 *
 * Any time the system successfully begins execution of the main algorithm,
 *  this function is run. The function runs at the main execution priority level.
 *  Custom code here should be used to initialize items such as timer callbacks,
 *  sensor drivers, and data structures needed.
 */
void hook_exitShutdown(void);

/**
 * @fn void hook_teardown(void);
 *
 * @brief This function is used to shutdown user specific code and drivers.
 *
 * Any time the system decides that the algorithm needs to stop,
 *  this function is run. This can be after an unsuccessful initialization
 *  (NOTE: hook_initialize() will not have been called), if the host has
 *  requested a shutdown or a halt, or if an internal error occurred. This
 *  function should be written in a way that disables timers, sensors, and
 *  allows the system to enter a low power shutdown state.
 */
void hook_teardown(void);

#endif /* _HOOKS_H */
