////////////////////////////////////////////////////////////////////////////////
///
/// @file       libs/Config/includes/EM718xConfig.h
///
/// @project    EM7189
///
/// @brief      Firmware configuration structure
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
#ifndef EM718X_CONFIG_H
#define EM718X_CONFIG_H

#include <types.h>

#define EM7189_CONFIG_MAGIC_NUMBER      0xC99B /**< @brief Unique number to identify the firmware configuration structure. */

#define BOOT_PROTOCOL_FLASH             0x00    /**< @brief Boot from the eeprom has been selected. */

#define FLAGS_INVALIDATE_FIRMWARE       0x01    /**< @brief When Set, force the bootloader to treat the firmware as invalid. */
#define FLAGS_USE_TURBO_MODE            0x02    /**< @brief When set, causes the firmware to transition from LongRun mode into Turbo mode. */

#define GPIO_PULL_DISABLED              0x00    /**< @brief Disable the 718x built-in GPIO pulls. */
#define GPIO_PULL_ENABLED               0x01    /**< @brief Enable the 718x built-in GPIO pull down. */

#define GPIO_MODE_HIZ                   0x00    /**< @brief Default mode - HiZ pin (disabled). */
#define GPIO_MODE_INPUT                 0x01    /**< @brief Configure the pin for input. */
#define GPIO_MODE_OUTPUT_0              0x02    /**< @brief Configure the pin as an output with the value as 0. */
#define GPIO_MODE_OUTPUT_1              0x03    /**< @brief Configure the pin as an output with the value as 1. */
#define GPIO_MODE_BITS                  (2)     /**< @brief The number of bits per entry in the structure. */
#define GET_GPIO_MODE(__mode__, __pin__)     ((__mode__ >> (__pin__ * GPIO_MODE_BITS)) & 0x03)
#define EM7189_CONFIG_VERSION           5       /**< @brief Current version of the EM7183 configuration structure (7189 based hw). */
/**
 * @brief   Configuration structure located at the end of a given firmware image.
 *
 *  The configuration structure contains information used to configure non-sensor specific settings.
 *
 */
PREPACK
typedef struct {
    /* Bootloader / Structure Information */
    UInt16  MagicNumber;        /**< @brief Config data magic number: 0xC82B */
    UInt16  Flags;              /**< @brief Configuration Flags, Bit 0 = Invalidate firmware, other = RFU. */
    UInt32  DriverCount:8;      /**< @brief The number of drivers (sensor descriptors) located immediately before the EM718xConfig data structure. */
    UInt32  VirtualCount:8;     /**< @brief The number of virtual (sensor descriptors) located in the firmware. */
    UInt32  CustomVersion:16;   /**< @brief Customer Specific version of the firmware image. */
    UInt16  RamVersion;         /**< @brief Ram version of the firmware image. */
    UInt8   Version;            /**< @brief Current version: 4 */
    UInt8   BootProtocol;       /**< @brief Protocol used for firmware loading. */

    /* GPIO Configuration */
    union {
        UInt64  PinSelection;       /**< @brief Direct access to all of the PinSelection bits. */
        struct {
            UInt64  EvCfg:12;       /**< @brief GPIO source selection for each event interrupt. */
            UInt64  HostIRQ:6;      /**< @brief The GPIO pin used for the Host Interrupt. A value > 24 corresponds to no pin being selected. */
            UInt64  HIFDisable:1;   /**< @brief If set, causes the host interface pins to be treated as GPIO, disabling the host interface. */
            UInt64  RFU:45;         /**< @brief Reserved for future use */
        } bits;
    } PinSelection;

    union {
        UInt32 PullSelection;       /**< @brief Direct access to all of the configuration data bits. */
        struct {
            UInt8  GPIO0PullSelection:1;    /**< @brief Pull selection for GPIO00. 0 = disable pull up or down; 1 = enable **/
            UInt8  GPIO1PullSelection:1;    /**< @brief Pull selection for GPIO01. 0 = disable pull up or down; 1 = enable **/
            UInt8  GPIO2PullSelection:1;    /**< @brief Pull selection for GPIO02. 0 = disable pull up or down; 1 = enable **/
            UInt8  GPIO3PullSelection:1;    /**< @brief Pull selection for GPIO03. 0 = disable pull up or down; 1 = enable **/
            UInt8  GPIO4PullSelection:1;    /**< @brief Pull selection for GPIO04. 0 = disable pull up or down; 1 = enable **/
            UInt8  GPIO5PullSelection:1;    /**< @brief Pull selection for GPIO05. 0 = disable pull up or down; 1 = enable **/
            UInt8  GPIO6PullSelection:1;    /**< @brief Pull selection for GPIO06. 0 = disable pull up or down; 1 = enable **/
            UInt8  GPIO7PullSelection:1;    /**< @brief Pull selection for GPIO07. 0 = disable pull up or down; 1 = enable **/

            UInt8  GPIO8PullSelection:1;    /**< @brief Pull selection for GPIO08. 0 = disable pull up or down; 1 = enable **/
            UInt8  GPIO9PullSelection:1;    /**< @brief Pull selection for GPIO09. 0 = disable pull up or down; 1 = enable **/
            UInt8  GPIO10PullSelection:1;   /**< @brief Pull selection for GPIO10. 0 = disable pull up or down; 1 = enable **/
            UInt8  GPIO11PullSelection:1;   /**< @brief Pull selection for GPIO11. 0 = disable pull up or down; 1 = enable **/
            UInt8  GPIO12PullSelection:1;   /**< @brief Pull selection for GPIO12. 0 = disable pull up or down; 1 = enable **/
            UInt8  GPIO13PullSelection:1;   /**< @brief Pull selection for GPIO13. 0 = disable pull up or down; 1 = enable **/
            UInt8  GPIO14PullSelection:1;   /**< @brief Pull selection for GPIO14. 0 = disable pull up or down; 1 = enable **/
            UInt8  GPIO15PullSelection:1;   /**< @brief Pull selection for GPIO15. 0 = disable pull up or down; 1 = enable **/

            UInt8  GPIO16PullSelection:1;   /**< @brief Pull selection for GPIO16. 0 = disable pull up or down; 1 = enable **/
            UInt8  GPIO17PullSelection:1;   /**< @brief Pull selection for GPIO17. 0 = disable pull up or down; 1 = enable **/
            UInt8  GPIO18PullSelection:1;   /**< @brief Pull selection for GPIO18. 0 = disable pull up or down; 1 = enable **/
            UInt8  GPIO19PullSelection:1;   /**< @brief Pull selection for GPIO19. 0 = disable pull up or down; 1 = enable **/
            UInt8  GPIO20PullSelection:1;   /**< @brief Pull selection for GPIO20. 0 = disable pull up or down; 1 = enable **/
            UInt8  GPIO21PullSelection:1;   /**< @brief Pull selection for GPIO21. 0 = disable pull up or down; 1 = enable **/
            UInt8  GPIO22PullSelection:1;   /**< @brief Pull selection for GPIO22. 0 = disable pull up or down; 1 = enable **/
            UInt8  GPIO23PullSelection:1;   /**< @brief Pull selection for GPIO23. 0 = disable pull up or down; 1 = enable **/

            UInt8  GPIO24PullSelection:1;   /**< @brief Pull selection for GPIO24. 0 = disable pull up or down; 1 = enable **/
            UInt8  SIF1CLKPullSelection:1;  /**< @brief Pull selection for SIF1CLK. 0 = disable pull up or down; 1 = enable **/
            UInt8  SIF1DINPullSelection:1;  /**< @brief Pull selection for SIF1DIN. 0 = disable pull up or down; 1 = enable **/
            UInt8  SIF1DIOPullSelection:1;  /**< @brief Pull selection for SIF1DIO. 0 = disable pull up or down; 1 = enable **/
            UInt8  RFU:4;
        } bits;
    } PullSelection;

    union {
        UInt64 PinMode;       /**< @brief Direct access to all of the configuration data bits. */
        UInt32 PinModeU32[2];
        struct {
            UInt8  GPIO0Mode:2;    /**< @brief Pin mode for GPIO00. 0 = disable, 1 = input, 2 = out(0), 3 = out(1) **/
            UInt8  GPIO1Mode:2;    /**< @brief Pin mode for GPIO01. 0 = disable, 1 = input, 2 = out(0), 3 = out(1) **/
            UInt8  GPIO2Mode:2;    /**< @brief Pin mode for GPIO02. 0 = disable, 1 = input, 2 = out(0), 3 = out(1) **/
            UInt8  GPIO3Mode:2;    /**< @brief Pin mode for GPIO03. 0 = disable, 1 = input, 2 = out(0), 3 = out(1) **/

            UInt8  GPIO4Mode:2;    /**< @brief Pin mode for GPIO04. 0 = disable, 1 = input, 2 = out(0), 3 = out(1) **/
            UInt8  GPIO5Mode:2;    /**< @brief Pin mode for GPIO05. 0 = disable, 1 = input, 2 = out(0), 3 = out(1) **/
            UInt8  GPIO6Mode:2;    /**< @brief Pin mode for GPIO06. 0 = disable, 1 = input, 2 = out(0), 3 = out(1) **/
            UInt8  GPIO7Mode:2;    /**< @brief Pin mode for GPIO07. 0 = disable, 1 = input, 2 = out(0), 3 = out(1) **/

            UInt8  GPIO8Mode:2;    /**< @brief Pin mode for GPIO08. 0 = disable, 1 = input, 2 = out(0), 3 = out(1) **/
            UInt8  GPIO9Mode:2;    /**< @brief Pin mode for GPIO09. 0 = disable, 1 = input, 2 = out(0), 3 = out(1) **/
            UInt8  GPIO10Mode:2;   /**< @brief Pin mode for GPIO10. 0 = disable, 1 = input, 2 = out(0), 3 = out(1) **/
            UInt8  GPIO11Mode:2;   /**< @brief Pin mode for GPIO11. 0 = disable, 1 = input, 2 = out(0), 3 = out(1) **/

            UInt8  GPIO12Mode:2;   /**< @brief Pin mode for GPIO12. 0 = disable, 1 = input, 2 = out(0), 3 = out(1) **/
            UInt8  GPIO13Mode:2;   /**< @brief Pin mode for GPIO13. 0 = disable, 1 = input, 2 = out(0), 3 = out(1) **/
            UInt8  GPIO14Mode:2;   /**< @brief Pin mode for GPIO14. 0 = disable, 1 = input, 2 = out(0), 3 = out(1) **/
            UInt8  GPIO15Mode:2;   /**< @brief Pin mode for GPIO15. 0 = disable, 1 = input, 2 = out(0), 3 = out(1) **/

            UInt8  GPIO16Mode:2;   /**< @brief Pin mode for GPIO16. 0 = disable, 1 = input, 2 = out(0), 3 = out(1) **/
            UInt8  GPIO17Mode:2;   /**< @brief Pin mode for GPIO17. 0 = disable, 1 = input, 2 = out(0), 3 = out(1) **/
            UInt8  GPIO18Mode:2;   /**< @brief Pin mode for GPIO18. 0 = disable, 1 = input, 2 = out(0), 3 = out(1) **/
            UInt8  GPIO19Mode:2;   /**< @brief Pin mode for GPIO19. 0 = disable, 1 = input, 2 = out(0), 3 = out(1) **/

            UInt8  GPIO20Mode:2;   /**< @brief Pin mode for GPIO20. 0 = disable, 1 = input, 2 = out(0), 3 = out(1) **/
            UInt8  GPIO21Mode:2;   /**< @brief Pin mode for GPIO21. 0 = disable, 1 = input, 2 = out(0), 3 = out(1) **/
            UInt8  GPIO22Mode:2;   /**< @brief Pin mode for GPIO22. 0 = disable, 1 = input, 2 = out(0), 3 = out(1) **/
            UInt8  GPIO23Mode:2;   /**< @brief Pin mode for GPIO23. 0 = disable, 1 = input, 2 = out(0), 3 = out(1) **/

            UInt8  GPIO24Mode:2;   /**< @brief Pin mode for GPIO24. 0 = disable, 1 = input, 2 = out(0), 3 = out(1) **/
            UInt8  SIF1CLKMode:2;  /**< @brief Pin mode for SIF1CLK. 0 = disable, 1 = input, 2 = out(0), 3 = out(1) **/
            UInt8  SIF1DINMode:2;  /**< @brief Pin mode for SIF1DIN. 0 = disable, 1 = input, 2 = out(0), 3 = out(1) **/
            UInt8  SIF1DIOMode:2;  /**< @brief Pin mode for SIF1DIO. 0 = disable, 1 = input, 2 = out(0), 3 = out(1) **/

            UInt8  RFU:8;
        } bits;
    } PinMode;

    char    reserved[12];           /**< @brief reserved bytes. */

    SInt8   FIFOAllocationRatio;    /**< @brief Ratio used to select amount of FIFO bytes in fifos. 0 = 50/50 split. -100 = 100% non-wakeup, +100 = 100% wakeup. +50 = 75% wakeup. */
    UInt8   SIFSelection;           /**< @brief The SIF mode used to select the i2c/spi interfaces on sif1, sif2 and sif3. */
    UInt16  FIFOWordsRequired;      /**< @brief 0 = only use free data RAM in already enabled RAM hardware blocks; non-zero = number of 32 bit words required -- additional hardware RAM blocks will be powered on as needed. */
} MIDPACK EM7189Config;
POSTPACK

#ifdef __clang__
_Static_assert(ELEMENT_OFFSET(EM7189Config, FIFOAllocationRatio) == 44, "Unexpected offset for FIFOAllocationRatio");
_Static_assert(ELEMENT_OFFSET(EM7189Config, SIFSelection) == 45, "Unexpected offset for SIFSelection");
_Static_assert(ELEMENT_OFFSET(EM7189Config, FIFOWordsRequired) == 46, "Unexpected offset for FIFOWordsRequired");
_Static_assert(sizeof(EM7189Config) == 48, "Unexpected size for EM7189Config");
#endif /* __clang__ */

#define EM7189_DESCRIPTOR   __attribute__ ((section (".em7189_descriptor")))
extern EM7189Config gEM7189Config;

#endif /* EM718X_CONFIG_H */
