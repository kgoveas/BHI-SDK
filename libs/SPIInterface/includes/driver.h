////////////////////////////////////////////////////////////////////////////////
///
/// @file       libs/SPIInterface/includes/driver.h
///
/// @project    EM7189
///
/// @brief      SPI Driver support datastructures.
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

#ifndef DRIVER_H
#define DRIVER_H

#include <stdint.h>
#include <types.h>
#include <string.h>

/////////////// Typedefs
typedef struct SPIM_Device_t {
    UInt16 maxClock;            /**< @brief Select the max SPI clock speed allowed for the device in kHz. */

    UInt8  csPin;               /**< @brief Select which GPIO pin should be monitored for interrupts. */
    UInt8  csLevel;             /**< @brief The value of CS needed to select the chip: (1) Active High or (0) Active Low. */
    UInt8  cpol:1;              /**< @brief The clock polarity. */
    UInt8  cpha:1;              /**< @brief The clock phase. */
    UInt8  en3wire:1;           /**< @brief When set, 3-write SPI is used instead of 4-wire */
    UInt8  lsb_first:1;         /**< @brief When set, the least significant bit is sent out first instead of the most significant. */
    UInt8  resvd1:4;

    UInt8  reg_shift:3;         /**< @brief Specifies the number of bits to shift the register addr by to genreate teh cmd byte. */
    UInt8  read_pol:1;          /**< @brief Specify if a read is signified by a 1 or a 0 in the cmd byte. */
    UInt8  read_bit:3;          /**< @brief Specify the bit position for the read/write bit. This is often bit 7 or 0. */
    UInt8  resvd2:1;
} SPIM_Device_t;

/** @cond */
typedef enum SensorStatus Driver_Status_t;
/** @endcond */

typedef void(*Driver_Callback_t)(enum SensorStatus, void*); /**< @brief The function type to call after a transaction completes. */
#include <SensorAPI.h>


#endif /* !DRIVER_H */
