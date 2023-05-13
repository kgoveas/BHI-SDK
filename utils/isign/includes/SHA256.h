////////////////////////////////////////////////////////////////////////////////
///
/// @file       utils/isign/includes/sha256.h
///
/// @project    EM7189
///
/// @brief      SHA256
///
/// @classification  Confidential
///
////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
///
/// @copyright Copyright (C) 2016-2019 EM Microelectronic
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
#ifndef SHA256_H
#define SHA256_H

#include "Typedef.h"

/**
 * @defgroup SHA256 SHA256 algorithm
 */


/** Digest size for SHA256 in bytes
* \ingroup SHA256
*/
#define SHA256_DIGEST_SIZE      32              //32 bytes = 256 bits

/** Block size for SHA256 in bytes
* \ingroup SHA256
*/
#define SHA256_BLOCK_SIZE       64              //Block size is 64 bytes

/**
* \fn void SHA256(uint8_t *pbInput, uint8_t *pbResult, uint64_t MessageLength)
* \brief Performs full SHA256
* \param[in] pbInput Pointer on message of MessageLength
* \param[out] pbResult Pointer on the 32 byte digest
* \param[in] MessageLength Length of the message to hash expressed in byte
\ingroup SHA256
*/
extern void SHA256(uint8_t *pbInput, uint8_t *pbResult, uint64_t MessageLength);

/**
* \fn void SHA256_Init(void)
* \brief  Initializes the SHA-256
\ingroup SHA256
*/
extern void SHA256_Init(void);

/**
* \fn void SHA256_Update(uint8_t *pbInput)
* \brief Updates the SHA-256
* \param[in] pbInput Pointer on a block of 64 bytes
\ingroup SHA256
*/
extern void SHA256_Update(uint8_t *pbInput);

/**
* \fn void SHA256_Final(uint8_t *pbInput, uint8_t LastBlockLength, uint8_t *pbResult)
* \brief Finalizes the SHA-256 computation
* \param[in] pbInput Pointer on the last block
* \param[in] LastBlockLength Length in bytes of the last block. It must be in the set [0..63]
* \param[out] pbResult Pointer on the 32 byte digest.
*
*
\ingroup SHA256
*/
extern void SHA256_Final(uint8_t *pbInput, uint8_t LastBlockLength, uint8_t *pbResult);

#endif
