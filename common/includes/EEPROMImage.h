////////////////////////////////////////////////////////////////////////////////
///
/// @file       common/includes/EEPROMImage.h
///
/// @project    EM7189
///
/// @brief      EEPROM firmware image.
///
/// @classification  Confidential
///
////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
///
/// @copyright Copyright (C) 2013-2018 EM Microelectronic
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

#ifndef EEPROM_IMAGE_H
#define EEPROM_IMAGE_H

/* EEPROM Image File format
 * 2bytes   MagicNumber
 * 2bytes   Flags
 * 4bytes   Text CRC32-CCITT
 * 4bytes   Data CRC32-CCITT
 * 2bytes   Text Length
 * 2bytes   Data Length
 * Nbytes   Text
 * Nbytes   Data
 */


typedef     UInt16      EEPROMMagic;
#define EEPROM_MAGIC_VALUE  0x652A

/**
 * @brief EEPROM boot flags.
 */
typedef union {
    /**
     * @brief Direct access to all flags.
     */
    UInt16 value;
    struct {
        /** @brief Do not execute the EEPROM image immediately after upload */
        UInt16  EEPROMNoExec:1;
        /** @brief Reserved */
        UInt16  Reserved:7;
        /** @brief The clock speed to upload the firmware at */
        UInt16  I2CClockSpeed:3;
        /** @brief The Expected Rom Version for the image. */
        UInt16  ROMVerExp:4;
        /** @brief Reserved */
        UInt16  reserved1:1;
    } bits;

} EEPROMFlags;

#define EXP_ROM_VERSION_ANY     0x00
#define EXP_ROM_VERSION_DI01    0x01
#define EXP_ROM_VERSION_DI02    0x02

typedef     UInt32      EEPROMTextCRC;
typedef     UInt32      EEPROMDataCRC;

typedef     UInt16      EEPROMTextLength;
typedef     UInt16      EEPROMDataLength;

typedef     UInt8*      EEPROMText;
typedef     UInt8*      EEPROMData;


/**
 * @brief EEPROM header format
 *
 * NOTE: a ROM version may also be useful to ensure that an incorrect ram binary is not used.
 * This is currently not implemented, however the RAM / EEPROM start code can double check this before it starts if needed.
 */
typedef struct {
    /** @brief The firmware magic number */
    UInt16  Magic; // Already read
    /** @brief Flags used to notify and control the boot process */
    UInt16  EEPROMFlags;
    /** @brief The CRC32-CCITT of the firmware text segment */
    UInt32  EEPROMTextCRC; // CRC32-CCITT
    /** @brief The CRC32-CCITT of the firmware data segment */
    UInt32  EEPROMDataCRC; // CRC32-CCITT
    /** @brief The number of program bytes to upload */
    UInt16  EEPROMTextLength;
    /** @brief The number of data bytes to upload */
    UInt16  EEPROMDataLength;
} EEPROMHeader;









#endif /* EEPROM_IMAGE_H */
