////////////////////////////////////////////////////////////////////////////////
///
/// @file       libs/Bootloader/includes/fwUpload.h
///
/// @project    EM7189
///
/// @brief      RAM and Flash firmware verification
///
/// @classification  Confidential
///
////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
///
/// @copyright Copyright (C) 2017-2018 EM Microelectronic
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
#ifndef FW_UPLOAD_H
#define FW_UPLOAD_H

#include <types.h>
#include <fwHeader.h>

typedef enum
{
    FWU_TYPE_INVALID,
    FWU_TYPE_RAM,
    FWU_TYPE_FLASH,
} FWU_TYPE;

/**
 * \brief Set up a new RAM firmware upload and verification or flash firmware verification
 * \param hdr - pointer to the firmware header
 * \param startAddr - starting address of firmware image
 * \param type - RAM or FLASH firmware
 */
void FWU_init(FWHeader_t* hdr, UInt8* startAddr, FWU_TYPE type);

/**
 * \brief Set the pointer to the end of the current DMA upload from the host
 * \param addr - the address to the last byte of RAM or FLASH that was uploaded
 * \param xferComplete - the transfer has finished
 */
void FWU_setUploadEnd(UInt8 *addr, bool xferComplete);

/**
 * \brief Check to see if there is pending work to complete for firmware verification.
 */
bool FWU_pending(void);

/**
 * \brief Process any new data that has been uploaded.  This includes updating the running CRC and updating the SHA.
 */
void FWU_process(void);

/**
 * \brief Verify flash firmware by calculating the SHA and verifying the ECDSA signature
 */
void FWU_verifyFlash(bool autoExec);


/**
 * \brief Calculate the CRC of a buffer and verify the result.
 * \param InBuf - The input buffer
 * \param BufLen - The number of bytes in the buffer
 * \param CrcVal - The CRC to check against.
 * \return TRUE if the buffer CRC matches the passed in CRC.
 */
bool FWU_verifyCrc(const UInt32 *InBuf, UInt8 BufLen, UInt32 CrcVal);

/**
 * \brief Check if it is ok to boot from flash
 * \param checkAutoExec - if TRUE, also check the NoExec bit
 * \return TRUE if flash firmware is present, is executable, and has been verified
 */
bool FWU_flashIsExecutable(bool checkAutoExec);

/**
 * \brief Request the flash firmware vector location
 * \return Pointer to the flash firmware vector location
 */
UInt8* FWU_getFlashVector(void);

/**
 * \brief Determine if the system has OTP keys.
 * \return TRUE     The system has at least one OTP key present.
 * \return FALSE    The system has no OTP keys present.
 */
bool FWU_OTPKeyPresent(void);

#endif /* !FW_UPLOAD_H */
