////////////////////////////////////////////////////////////////////////////////
///
/// @file       common/includes/fwHeader.h
///
/// @project    EM7189
///
/// @brief      Firmware image header.
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
#ifndef FW_IMAGE_H
#define FW_IMAGE_H

#include <types.h>
#include <ECC_Typedef.h>
#include <SHA256.h>

#define FW_MAGIC_2B (0x662B)
#define FW_MAGIC_2C (0x662C)

#ifdef ROM_VERSION
#if (ROM_VERSION == ROM_7189_DI01) || (ROM_VERSION == ROM_7189_DI02)
#define FW_MAGIC_VALUE  FW_MAGIC_2B
#else
#define FW_MAGIC_VALUE  FW_MAGIC_2C
#endif
#endif

/**
 * @brief Firmware Image flags
 */
typedef union
{
    /**
     * @brief Direct access to all flags
     */
    UInt16 value;
    struct {
        /** @brief Do not execute the image immediately after upload */
        UInt16  NoExec:1;
        /** @brief This is the last image in the chain */
        UInt16  EndOfImgChain:1;
        /** @brief Allow next payload in chain to be unsigned */
        UInt16  NextImgUnsigned:1;
        /** @brief This is a FLASH image */
        UInt16  ImgTypeFlash:1;
        /** @brief This is a second-stage bootloader */
        UInt16  SSBL:1;
        /** @brief This image should be paired with a SSBL */
        UInt16  PairWithSSBL:1;
        /** @brief The kernel image supports encryption. This flag is valid for kernel image only. */
        UInt16  EncryptionSupported:1;
        /** @brief The entire user image is encrypted. This flag is valid for user images only. */
        UInt16  Encrypted:1;
        /** @brief Reserved */
        UInt16  Reserved:8;
    } bits;

} ImageFlags_t;

#define KEY_LOCATION_OTP    0x0
#define KEY_LOCATION_ROM    0x1
#define KEY_LOCATION_RAM    0x2
#define KEY_LOCATION_FLASH  0x3

#define KEY_TYPE_ECDSA      0x0

#define KEY_NO_AUTHENTICATION   0xF
#define KEY_MAX_LOCATION        0xF

/**
 * @brief Information about the key used to verify the image
 */
typedef union
{
    /**
     * @brief Direct access to all flags
     */
    UInt16 value;
    struct
    {
        /** @brief Key location */
        UInt16  Location:4;
        /** @brief Key index into the key location */
        UInt16  Index:4;
        /** @brief Key type */
        UInt16  Type:2;
        /** @brief Reserved */
        UInt16  Reserved:6;
    } bits;
} KeyFlags_t;

// Lengths in bytes
#define CERT_LEN    (48)

/**
 * @brief Signature used to verify the image
 */
typedef union
{
    /**
     * @brief Direct access certificate
     */
    UInt8   value[CERT_LEN];
    /** @brief ECDSA certificate value */
    PointCertificate    ecdsaCert;
} Cert_t;

/**
 * @brief Fields used by a second-stage bootloader
 */
typedef struct
{
    /** @brief Offset into flash where the shadow firmware image is located */
    UInt32 shadowOffset;
} SSBL_t;

/**
 * @brief Firmware header for image signing
 */
typedef struct
{
    /** @brief The firmware magic number */
    UInt16  Magic;           // 0x00-0x01
    /** @brief Flags used to notify and control the boot process */
    ImageFlags_t ImageFlags; // 0x02-0x03
    /** @brief Key flags used to control verifying the firmware */
    KeyFlags_t KeyFlags;     // 0x04-0x05
    /** @brief The Version for the image. */
    UInt16  Version;          // 0x06-0x07
    /** @brief SHA-256 of the uploaded image */
    UInt8   Sha[SHA256_DIGEST_SIZE]; // 0x08-0x27
    /** @brief Image certificate used to verify the uploaded image */
    Cert_t  Certificate;     // 0x28-0x57
    /** @brief Uploaded Image Length */
    UInt32  ImageLen;        // 0x58-0x5B
    /** @brief Image CRC */
    UInt32  ImageCrc;        // 0x5C-0x5F
    /** @brief Offset of the public keys in the uploaded image, if 0 then no key */
    UInt32  KeyOffset;       // 0x60-0x63
    /** @brief Expected ROM/RAM version to be run with */
    UInt32  ExpVer;          // 0x64-0x67
    union
    {
        /** @brief Information for the second-stage bootloader */
        SSBL_t ssblInfo;     // 0x68 - 0x6B
        /** @brief Reserved value */
        UInt8  Reserved2[4]; // 0x68 - 0x6B
    } SSBL;
    /** @brief Maximum firmware image size (KB) */
    UInt32 maxFwSizeKB;      // 0x6C - 0x6F
    /** @brief Reserved value */
    // Padding to make the header an even 124 bytes + 4 bytes for HOST command (Handy for DMA)
    UInt8   Reserved1[8];    // 0x70-0x77
    /** @brief CRC calculated over all fields prior to this */
    UInt32  HdrCrc;          // 0x78-0x7c
} FWHeader_t;
#endif // FW_IMAGE_H
