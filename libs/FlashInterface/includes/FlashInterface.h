////////////////////////////////////////////////////////////////////////////////
///
/// @file       libs/FlashInterface/includes/FlashInterface.h
///
/// @project    EM7189
///
/// @brief      QSPI Flash driver.
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

/** @addtogroup FLASH_INT
 * @{
 */

#ifndef FLASH_INTERFACE_H
#define FLASH_INTERFACE_H

#include <types.h>
#include <FlashDescriptor.h>

/**
 * @fn void Flash_invalidateCache(void);
 * @brief  Invalidates the flash cache. This function must not be called when the cache controller is enabled.
 */
void Flash_invalidateCache(void);

/**
 * @fn void Flash_enableCacheController(FlashDescriptor_t* desc);
 * @brief  Initializes the flash device as specified in the descriptor and enables the cache controller.
 *
 * Note: This function does not explicitly invalidate the flash cache. If the cache must be invalidated
 *          due to an erase or a write, @ref Flash_invalidateCache must be called before enabling the cache.
 *
 * @param [in] desc     The flash descriptor for the attached device.
 */
void Flash_enableCacheController(FlashDescriptor_t* desc);


/**
 * @fn void Flash_disableCacheController(bool turn_off_ram);
 * @brief Disables the flash cache controller and turns of associated ram if requested..
 *
 * @param turn_off_ram  Request to turn off tha FCC ram bank or not.
 */
void Flash_disableCacheController(bool turn_off_ram);

/**
 * @fn void Flash_readDescriptor(FlashDescriptor_t* desc);
 * @brief  Reads the flash descriptor from the attached device.
 *
 * @param [out] desc    The flash descriptor for the attached device.
 */
void Flash_readDescriptor(FlashDescriptor_t* desc);

/**
 * @fn bool Flash_hasValidDescriptor(void);
 * @brief  Determines if the firmware has loaded a valid flash descriptor.
 *
 * @returns TRUE if a valid descriptor has been loaded. FALSE otherwise.
 */
bool Flash_hasValidDescriptor(void);

/**
 * @fn UInt32 Flash_getSize(void);
 * @brief Determines the size of the attached flash device based on the flash descriptor.
 *
 * @returns The size, in bytes, if the flash descriptor is valid. 0 otherwise.
 */
UInt32 Flash_getSize(void);

/**
 * @fn UInt8 Flash_probe(void);
 * @brief Resets the part into SSPI mode and reads out the manufacturer id.
 *
 * @returns The manufacturer ID if valid, 0 otherwise.
 */
UInt8 Flash_probe(void);

/**
 * @fn UInt32 Flash_eraseSize(void);
 * @brief Determines the erase size of the attached device, as determined by the flash descriptor.
 *
 * @returns The size of the sector size to be erased in bytes.
 */
UInt32 Flash_eraseSize(void);

/**
 * @fn UInt32 Flash_writeSize(void);
 * @brief Determines the write size of the attached device, as determined by the flash descriptor.
 *
 * @returns The maximum number of bytes that can be written to the device before wrapping around.
 */
UInt32 Flash_writeSize(void);

/**
 * @fn bool Flash_TurboAllowed(void);
 * @brief Determines is the flash descriptor allows enabling of turbo mode.
 *
 * @returns TRUE if allowed, FALSE otherwise.
 */
bool Flash_TurboAllowed(void);

#include "QSPIInterface.h"

#endif /* !FLASH_INTERFACE_H */

/** @} */
