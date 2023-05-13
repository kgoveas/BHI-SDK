////////////////////////////////////////////////////////////////////////////////
///
/// @file       libs/Security/includes/ECDSA_Verify.h
///
/// @project    EM7189
///
/// @brief      ECDSA in verification mode
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

#ifndef ECDSA_VERIFY_H
#define ECDSA_VERIFY_H

#include "ECC_Typedef.h"

/**
 * @defgroup ECC ECC
 */
/**
* \fn uint8_t ECDSA_Verify_FUNC(PointCertificate *Certificate, POINT *PubKey, uint8_t *Digest)
* \brief Computes the signature with ECDSA
* \param[in] Certificate Pointer on the certificate to verify
* \param[in] PubKey Pointer on the the public key
* \param[in] Digest Digest of the message
* \return SW_OK: Valid certificate
* \return SW_NOK: Invalid certificate
* \return SW_NOK_X: NOK, X coordinate bigger than N. Incorrect certificate
* \return SW_NOK_Y: NOK, Y coordinate bigger than N. Incorrect certificate
* \return SW_NOK_X_NULL NOK:  X coordinate is null. Incorrect certificate.
* \return SW_NOK_Y_NULL NOK: Y coordinate is null. Incorrect certificate.
* \ingroup ECC
*/
extern uint8_t ECDSA_Verify_FUNC(PointCertificate *Certificate, POINT *PubKey, uint8_t *Digest);

/** Local RAM requirement in bytes
* \ingroup ECC
*/
#define ECC_RAM_REQUIREMENT   568

/** Pointer on local RAM
* \ingroup ECC
*/
extern uint8_t *ptrCryptoRAM;
#endif
