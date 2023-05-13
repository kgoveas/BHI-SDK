////////////////////////////////////////////////////////////////////////////////
///
/// @file       libs/Ram/includes/Ram.h
///
/// @project    EM7189
///
/// @brief      Routines for configuring RAM.
///
/// @classification  Confidential
///
////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
///
/// @copyright Copyright (C) 2015-2019 EM Microelectronic
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

/** @defgroup RAM_INT    Ram Interface */
/** @addtogroup RAM_INT
 * @{
 */

#ifndef RAM_H
#define RAM_H

#include <types.h>

#define RAM_BANKS_TOTAL         (9u)                                   /**< @brief The total number of RAM banks in the system. */
#define RAM_BANKS_OPTIONAL      (7u)                                   /**< @brief The number of RAM banks that can be turned on or off. */
#define RAM_BANKS_FIXED         (RAM_BANKS_TOTAL - RAM_BANKS_OPTIONAL) /**< @brief The number of banks always turned on. */
#define RAM_BANKS_DCCM_POSSIBLE (RAM_BANKS_TOTAL - 1)                  /**< @brief The total number of RAM banks that can be configured for DCCM */
#define RAM_BANKS_ICCM_POSSIBLE (RAM_BANKS_TOTAL - 1)                  /**< @brief The total number of RAM banks that can be configured for ICCM */
#define RAM_BANK_DCCM_OFFSET    (RAM_BANKS_TOTAL)                      /**< @brief The FW bank offset used to signal DCCM vs ICCM. */
#define RAM_BANKS_FCC           (1u)                                   /**< @brief The number of RAM banks dedicated for the flash cache controller when enabled. */
#define RAM_BANK_FCC_SHIFT      (6u)                                   /**< @brief The hardware RAM bank dedicated for the flash cache controller when enabled. */
#define RAM_INITIAL_SIZE        (16u * 1024u)                          /**< @brief The size (in bytes) of the initial (non-optional) RAM banks. */
#define RAM_BANK_SIZE           (32u * 1024u)                          /**< @brief The size (in bytes) of the optional RAM banks. */
#define INVALID_BANK            (0xffu)                                /**< @brief the FW bank ID for invalid banks. */
#define MAX_RAM_FW_SIZE_KB      ((RAM_INITIAL_SIZE + (RAM_BANKS_OPTIONAL_MAX * RAM_BANK_SIZE)) / 1024u)

/**
 * @brief   Determines the start of a specified FW bank.
 * @param   bank    The FW bank to determine the start of.
 * @return  The first byte address of the specified bank.
 */
extern UInt8 *RAM_start(UInt8 bank);

/**
 * @brief   Determines the start of a specified FW bank.
 * @param   bank    The FW bank to determine the end of.
 * @return  The last byte address of the specified bank.
 */
extern UInt8 *RAM_end(UInt8 bank);

/**
 * @brief   This function is used to retrieve the last hardware bank available in the part.
 * @return  The last hardware bank availale on the part.
 */
extern UInt8 RAM_lastHardwareBank(void);

/**
 * @brief   This function is used to determine th maximum number of DCCM banks available.
 * @return  The number of DCCM banks available.
 */
extern UInt8 RAM_lastDCCMBank(void);

/**
 * @brief   This function is used to determine th maximum number of ICCM banks available.
 * @return  The number of ICCM banks available.
 */
extern UInt8 RAM_lastICCMBank(void);

/**
 * @brief   This function is used to determine the bank, if possible, of a specified address.
 * @param   addr    The address to determine the bank of.
 * @returns  The firmware bank ID if the address is a valid RAM bank.
 * @returns  INVALID_BANK if the specified address cannot be mapped into any known bank.
 */
extern UInt8 RAM_getBank(UInt8 *addr);

/**
 * @brief   Turns on the specified number of DCCM and ICCM ram banks. All other banks are turned off.
 *
 *  If more banks are requested on than are available in the system (@ref RAM_BANKS_TOTAL), the behaviour of this function is undefined.
 *
 * @param   num_iccm_banks  The number of ICCM banks requested. num_iccm_banks cannot be larger than @ref RAM_BANKS_ICCM_POSSIBLE.
 * @param   num_dccm_banks  The number of DCCM banks requested. num_dccm_banks cannot be larger than @ref RAM_BANKS_DCCM_POSSIBLE.
 */
extern void RAM_powerOn(UInt8 num_iccm_banks, UInt8 num_dccm_banks);

/**
 * @brief   Specifies the number of banks to allocated for DCCM.
 * @param   num_banks   The number of banks to be reserved for DCCM.
 */
extern void RAM_setDCCMBanks(UInt8 num_banks);

/**
 * @brief   Determine the number of banks currently configured for DCCM
 * @returns The number of banks configured for DCCM
 */
extern UInt8 RAM_numDCCMBanks(void);

/**
 * @brief   Determine the number of banks currenty configured for ICCM.
 * @returns The number of banks configured for ICCM.
 */
extern UInt8 RAM_numICCMBanks(void);

/**
 * @brief   Determine if the specified bank is in DCCM or not.
 * @param   bank_num    The FW bank in question.
 * @return  TRUE        if the bank is a DCCM bank
 * @return  FALSE       if the bank is not a DCCM bank
 */
extern bool RAM_isDCCM(UInt8 bank_num);

/**
 * @brief   Determine if the specified bank is in ICCM or not.
 * @param   bank_num    The FW bank in question.
 * @return  TRUE        if the bank is a ICCM bank
 * @return  FALSE       if the bank is not a ICCM bank
 */
extern bool RAM_isICCM(UInt8 bank_num);

/**
 * @brief   Determine the total number of ram banks powered on.
 * @returns The number of banks powered on.
 */
extern UInt8 RAM_numBanksPowered(void);

/**
 * @brief   Determine the number of ICCM banks powered on.
 * @returns The number of ICCM banks powered on.
 */
extern UInt8 RAM_numICCMBanksPowered(void);

/**
 * @brief   Determine the number of DCCM banks powered on.
 * @returns The number of DCCM banks powered on.
 */
extern UInt8 RAM_numDCCMBanksPowered(void);

/**
 * @brief   Determine if a specified bank is powered on.
 * @param   bank_num    The FW bank in question.
 * @return  TRUE        if the bank is powered on.
 * @return  FALSE       if the bank is not powered on.
 */
extern bool RAM_isPowered(UInt8 bank_num);

/**
 * @brief   Determine if a specified bank is valid with the current hrdware configuration.
 * @param   bank_num    The FW bank in question.
 * @return  TRUE        if the bank is valid.
 * @return  FALSE       if the bank is not valid.
 */
extern bool RAM_bankIsValid(UInt8 bank_num);

/**
 * @brief   Requests that the next bank after currAddr, but no further than endAddr is powered on.
 * @param   currAddr    The current address in use.
 * @param   endAddr     The last address that will be used.
 */
extern void RAM_checkAndEnableBank(UInt8 *currAddr, UInt8 *endAddr);

/**
 * @brief   Determines the end address of the currently powered ICCM.
 * @returns The last byte address of currently powered ICCM.
 */
extern UInt8 *RAM_ICCMEndAddr(void);

/**
 * @brief   Determines the end address of the currently powered DCCM.
 * @returns The last byte address of currently powered DCCM.
 */
extern UInt8 *RAM_DCCMEndAddr(void);

#endif /*!RAM_H */

/** @} */
