////////////////////////////////////////////////////////////////////////////////
///
/// @file       utils/isign/includes/ECDSA_Sign.h
///
/// @project    EM7189
///
/// @brief      ECDSA signature
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
#ifndef ECDSA_SIGN_H
#define ECDSA_SIGN_H

#include "ECC_Typedef.h"

/**
 * @defgroup ECDSA_SIGN ECDSA Signature
 */

/**
* \fn uint8_t ECDSA_Sign(uint32_t *SecretKey, uint8_t *Digest, PointCertificate *Signature)
* \brief Computes the signature with ECDSA
* \param[in] SecretKey Pointer on the secret key
* \param[in] Digest Pointer on the digest to sign
* \param[out] Signature Pointer on the signature
* \return SW_OK OK, the signature was performed
* \return SW_NOK: NOK: k is bigger than N. It should never happen
* \return SW_NOK_X_NULL: NOK : the X of the signature is null
* \return SW_NOK_Y_NULL: NOK : the Y of the signature is null
* \return SW_RDRAND_NOT_SUPPORTED: NOK the TRNG rdrand is not supported by this platform. No signature generated
* \return SW_RDRAND_FAILED: NOK,the TRNF rndand failed in the random generation. No signature generated
\ingroup ECDSA_SIGN
*/
extern uint8_t ECDSA_Sign(uint32_t *SecretKey, uint8_t *Digest, PointCertificate *Signature);

#endif
