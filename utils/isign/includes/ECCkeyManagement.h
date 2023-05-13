////////////////////////////////////////////////////////////////////////////////
///
/// @file       utils/isign/includes/ECCkeyManagement.h
///
/// @project    EM7189
///
/// @brief      ECC Key management functions
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
#ifndef ECCKEYMANAGEMENT_H
#define ECCKEYMANAGEMENT_H

#include "ECC_Typedef.h"

/**
 * @defgroup ECC_KEY_MANAGEMENT ECC Key Management
 */

/**
* \fn uint8_t ECC_GeneratePrivateKey(uint32_t *PrivateKey)
* \brief Generates an ECC Private key
* \param[out] PrivateKey Pointer on buffer of the scalar size
* \return SW_OK: OK,the private key was correctly generated
* \return SW_RDRAND_NOT_SUPPORTED: NOK the TRNG rdrand is not supported by this platform. No key generated
* \return SW_RDRAND_FAILED: NOK,the TRNF rndand failed in the random generation. No key generated.
\ingroup ECC_KEY_MANAGEMENT
*/
extern uint8_t ECC_GeneratePrivateKey(uint32_t *PrivateKey);

/**
* \fn void ECC_ReducePrivateKey(uint32_t *PrivateKeyIN, uint32_t *PrivateKeyOUT)
* \brief Reduces a private key modulo N (where N is the order of the curve)
* \param[in] PrivateKeyIN Pointer on the private key before reduction
* \param[out] PrivateKeyOUT Pointer on the private key after reduction
\ingroup ECC_KEY_MANAGEMENT
*/
extern void ECC_ReducePrivateKey(uint32_t *PrivateKeyIN, uint32_t *PrivateKeyOUT);

/**
* \fn uint8_t ECC_ComputePubKey(uint32_t *PrivateKey, POINT *PubKey)
* \brief Generates a public key from a private key
* \param[in] PrivateKey Pointer on the private key
* \param[out] PubKey Pointer on the public key
* \return SW_OK: OK,the public key was correctly generated
* \return SW_REDUCED_PRIVATE_KEY OK: the public key was generated but the private key was previously reduced modulo N
* \return SW_NOK: NOK,the private key is null. The public key was not generated. Another private key must be selected.
\ingroup ECC_KEY_MANAGEMENT
*/
extern uint8_t ECC_ComputePubKey(uint32_t *PrivateKey, POINT *PubKey);

#endif
