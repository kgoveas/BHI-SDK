////////////////////////////////////////////////////////////////////////////////
///
/// @file       common/7189/includes/peripherals.h
///
/// @project    EM7189
///
/// @brief      peripherals registers
///
/// @classification  Confidential
///
////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
///
/// @copyright Copyright (c) 2015-2019, EM Microelectronic
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

/** @defgroup PERIPHERALS_H    peripherals registers */
/** @addtogroup PERIPHERALS_H
 * @{
 */
#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#include <types.h>

#define REG_CHP_TRIM0 ((volatile UInt32*)0xf00100) /*  */
#define     CHP_TRIM0_REG_TRIM_LO_SHIFT 0u
#define     CHP_TRIM0_REG_TRIM_LO_MASK  0xfu
#define GET_CHP_TRIM0_REG_TRIM_LO(__reg__)  (((__reg__) & 0xf) >> 0u)
#define SET_CHP_TRIM0_REG_TRIM_LO(__val__)  (((__val__) << 0u) & 0xfu)
#define     CHP_TRIM0_RESERVED_7_4_SHIFT 4u
#define     CHP_TRIM0_RESERVED_7_4_MASK  0xf0u
#define GET_CHP_TRIM0_RESERVED_7_4(__reg__)  (((__reg__) & 0xf0) >> 4u)
#define SET_CHP_TRIM0_RESERVED_7_4(__val__)  (((__val__) << 4u) & 0xf0u)
#define     CHP_TRIM0_REG_TRIM_HI_SHIFT 8u
#define     CHP_TRIM0_REG_TRIM_HI_MASK  0xf00u
#define GET_CHP_TRIM0_REG_TRIM_HI(__reg__)  (((__reg__) & 0xf00) >> 8u)
#define SET_CHP_TRIM0_REG_TRIM_HI(__val__)  (((__val__) << 8u) & 0xf00u)
#define     CHP_TRIM0_RESERVED_15_12_SHIFT 12u
#define     CHP_TRIM0_RESERVED_15_12_MASK  0xf000u
#define GET_CHP_TRIM0_RESERVED_15_12(__reg__)  (((__reg__) & 0xf000) >> 12u)
#define SET_CHP_TRIM0_RESERVED_15_12(__val__)  (((__val__) << 12u) & 0xf000u)
#define     CHP_TRIM0_POR_TRIM_LO_SHIFT 16u
#define     CHP_TRIM0_POR_TRIM_LO_MASK  0xf0000u
#define GET_CHP_TRIM0_POR_TRIM_LO(__reg__)  (((__reg__) & 0xf0000) >> 16u)
#define SET_CHP_TRIM0_POR_TRIM_LO(__val__)  (((__val__) << 16u) & 0xf0000u)
#define     CHP_TRIM0_RESERVED_23_20_SHIFT 20u
#define     CHP_TRIM0_RESERVED_23_20_MASK  0xf00000u
#define GET_CHP_TRIM0_RESERVED_23_20(__reg__)  (((__reg__) & 0xf00000) >> 20u)
#define SET_CHP_TRIM0_RESERVED_23_20(__val__)  (((__val__) << 20u) & 0xf00000u)
#define     CHP_TRIM0_POR_TRIM_HI_SHIFT 24u
#define     CHP_TRIM0_POR_TRIM_HI_MASK  0xf000000u
#define GET_CHP_TRIM0_POR_TRIM_HI(__reg__)  (((__reg__) & 0xf000000) >> 24u)
#define SET_CHP_TRIM0_POR_TRIM_HI(__val__)  (((__val__) << 24u) & 0xf000000u)
#define     CHP_TRIM0_RESERVED_31_28_SHIFT 28u
#define     CHP_TRIM0_RESERVED_31_28_MASK  0xf0000000u
#define GET_CHP_TRIM0_RESERVED_31_28(__reg__)  (((__reg__) & 0xf0000000) >> 28u)
#define SET_CHP_TRIM0_RESERVED_31_28(__val__)  (((__val__) << 28u) & 0xf0000000u)

/** @brief Register definition for @ref CHP_t.Trim0. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt8 reg_trim_lo:4;
        /** @brief Reserved bits 7 to 4. Reads as 0. */
        UInt8 reserved_7_4:4;
        /** @brief  */
        UInt8 reg_trim_hi:4;
        /** @brief Reserved bits 15 to 12. Reads as 0. */
        UInt8 reserved_15_12:4;
        /** @brief  */
        UInt8 por_trim_lo:4;
        /** @brief Reserved bits 23 to 20. Reads as 0. */
        UInt8 reserved_23_20:4;
        /** @brief  */
        UInt8 por_trim_hi:4;
        /** @brief Reserved bits 31 to 28. Reads as 0. */
        UInt8 reserved_31_28:4;
    } bits;
} RegCHPTrim0_t;

#define REG_CHP_TRIM1 ((volatile UInt32*)0xf00104) /*  */
#define     CHP_TRIM1_SOSC_TRIM_LO_SHIFT 0u
#define     CHP_TRIM1_SOSC_TRIM_LO_MASK  0xffu
#define GET_CHP_TRIM1_SOSC_TRIM_LO(__reg__)  (((__reg__) & 0xff) >> 0u)
#define SET_CHP_TRIM1_SOSC_TRIM_LO(__val__)  (((__val__) << 0u) & 0xffu)
#define     CHP_TRIM1_SOSC_TRIM_HI_SHIFT 8u
#define     CHP_TRIM1_SOSC_TRIM_HI_MASK  0xff00u
#define GET_CHP_TRIM1_SOSC_TRIM_HI(__reg__)  (((__reg__) & 0xff00) >> 8u)
#define SET_CHP_TRIM1_SOSC_TRIM_HI(__val__)  (((__val__) << 8u) & 0xff00u)
#define     CHP_TRIM1_TOSC_TRIM_SHIFT 16u
#define     CHP_TRIM1_TOSC_TRIM_MASK  0xff0000u
#define GET_CHP_TRIM1_TOSC_TRIM(__reg__)  (((__reg__) & 0xff0000) >> 16u)
#define SET_CHP_TRIM1_TOSC_TRIM(__val__)  (((__val__) << 16u) & 0xff0000u)
#define     CHP_TRIM1_GEN_CTRL_SHIFT 24u
#define     CHP_TRIM1_GEN_CTRL_MASK  0xf000000u
#define GET_CHP_TRIM1_GEN_CTRL(__reg__)  (((__reg__) & 0xf000000) >> 24u)
#define SET_CHP_TRIM1_GEN_CTRL(__val__)  (((__val__) << 24u) & 0xf000000u)
#define     CHP_TRIM1_RESERVED_31_28_SHIFT 28u
#define     CHP_TRIM1_RESERVED_31_28_MASK  0xf0000000u
#define GET_CHP_TRIM1_RESERVED_31_28(__reg__)  (((__reg__) & 0xf0000000) >> 28u)
#define SET_CHP_TRIM1_RESERVED_31_28(__val__)  (((__val__) << 28u) & 0xf0000000u)

/** @brief Register definition for @ref CHP_t.Trim1. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt8 sosc_trim_lo;
        /** @brief  */
        UInt8 sosc_trim_hi;
        /** @brief  */
        UInt8 tosc_trim;
        /** @brief  */
        UInt8 gen_ctrl:4;
        /** @brief Reserved bits 31 to 28. Reads as 0. */
        UInt8 reserved_31_28:4;
    } bits;
} RegCHPTrim1_t;

#define REG_CHP_POWERCONFIG ((volatile UInt32*)0xf00108) /*  */
#define     CHP_POWERCONFIG_RUN_LVL_SHIFT 0u
#define     CHP_POWERCONFIG_RUN_LVL_MASK  0x3u
#define GET_CHP_POWERCONFIG_RUN_LVL(__reg__)  (((__reg__) & 0x3) >> 0u)
#define SET_CHP_POWERCONFIG_RUN_LVL(__val__)  (((__val__) << 0u) & 0x3u)
#define     CHP_POWERCONFIG_RUN_LVL_20MHZ_AT_1_1V 0x0u
#define     CHP_POWERCONFIG_RUN_LVL_20MHZ_AT_0_86V 0x1u
#define     CHP_POWERCONFIG_RUN_LVL_50MHZ_AT_1_1V 0x2u
#define     CHP_POWERCONFIG_RUN_LVL_OTP_PROGRAMMING 0x3u

#define     CHP_POWERCONFIG_RESERVED_7_2_SHIFT 2u
#define     CHP_POWERCONFIG_RESERVED_7_2_MASK  0xfcu
#define GET_CHP_POWERCONFIG_RESERVED_7_2(__reg__)  (((__reg__) & 0xfc) >> 2u)
#define SET_CHP_POWERCONFIG_RESERVED_7_2(__val__)  (((__val__) << 2u) & 0xfcu)
#define     CHP_POWERCONFIG_RAM_PWR_SHIFT 8u
#define     CHP_POWERCONFIG_RAM_PWR_MASK  0x7f00u
#define GET_CHP_POWERCONFIG_RAM_PWR(__reg__)  (((__reg__) & 0x7f00) >> 8u)
#define SET_CHP_POWERCONFIG_RAM_PWR(__val__)  (((__val__) << 8u) & 0x7f00u)
#define     CHP_POWERCONFIG_RESERVED_15_15_SHIFT 15u
#define     CHP_POWERCONFIG_RESERVED_15_15_MASK  0x8000u
#define GET_CHP_POWERCONFIG_RESERVED_15_15(__reg__)  (((__reg__) & 0x8000) >> 15u)
#define SET_CHP_POWERCONFIG_RESERVED_15_15(__val__)  (((__val__) << 15u) & 0x8000u)
#define     CHP_POWERCONFIG_RAM_RST_N_SHIFT 16u
#define     CHP_POWERCONFIG_RAM_RST_N_MASK  0x7f0000u
#define GET_CHP_POWERCONFIG_RAM_RST_N(__reg__)  (((__reg__) & 0x7f0000) >> 16u)
#define SET_CHP_POWERCONFIG_RAM_RST_N(__val__)  (((__val__) << 16u) & 0x7f0000u)
#define     CHP_POWERCONFIG_RESERVED_31_23_SHIFT 23u
#define     CHP_POWERCONFIG_RESERVED_31_23_MASK  0xff800000u
#define GET_CHP_POWERCONFIG_RESERVED_31_23(__reg__)  (((__reg__) & 0xff800000) >> 23u)
#define SET_CHP_POWERCONFIG_RESERVED_31_23(__val__)  (((__val__) << 23u) & 0xff800000u)

/** @brief Register definition for @ref CHP_t.Powerconfig. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt8 run_lvl:2;
        /** @brief Reserved bits 7 to 2. Reads as 0. */
        UInt8 reserved_7_2:6;
        /** @brief  */
        UInt8 ram_pwr:7;
        /** @brief Reserved bit 15. Reads as 0. */
        UInt8 reserved_15_15:1;
        /** @brief  */
        UInt16 ram_rst_n:7;
        /** @brief Reserved bits 31 to 23. Reads as 0. */
        UInt16 reserved_31_23:9;
    } bits;
} RegCHPPowerconfig_t;

#define REG_CHP_MEMCONFIG ((volatile UInt32*)0xf0010c) /*  */
#define     CHP_MEMCONFIG_FLASH_SIZE_SHIFT 0u
#define     CHP_MEMCONFIG_FLASH_SIZE_MASK  0x3u
#define GET_CHP_MEMCONFIG_FLASH_SIZE(__reg__)  (((__reg__) & 0x3) >> 0u)
#define SET_CHP_MEMCONFIG_FLASH_SIZE(__val__)  (((__val__) << 0u) & 0x3u)
#define     CHP_MEMCONFIG_FLASH_SIZE_1M_EFLASH 0x0u
#define     CHP_MEMCONFIG_FLASH_SIZE_2M_EFLASH 0x1u
#define     CHP_MEMCONFIG_FLASH_SIZE_4M_EFLASH 0x2u
#define     CHP_MEMCONFIG_FLASH_SIZE_8M_EFLASH 0x3u

#define     CHP_MEMCONFIG_RESERVED_3_2_SHIFT 2u
#define     CHP_MEMCONFIG_RESERVED_3_2_MASK  0xcu
#define GET_CHP_MEMCONFIG_RESERVED_3_2(__reg__)  (((__reg__) & 0xc) >> 2u)
#define SET_CHP_MEMCONFIG_RESERVED_3_2(__val__)  (((__val__) << 2u) & 0xcu)
#define     CHP_MEMCONFIG_FLASH_CFG_SHIFT 4u
#define     CHP_MEMCONFIG_FLASH_CFG_MASK  0x10u
#define GET_CHP_MEMCONFIG_FLASH_CFG(__reg__)  (((__reg__) & 0x10) >> 4u)
#define SET_CHP_MEMCONFIG_FLASH_CFG(__val__)  (((__val__) << 4u) & 0x10u)
#define     CHP_MEMCONFIG_RESERVED_7_5_SHIFT 5u
#define     CHP_MEMCONFIG_RESERVED_7_5_MASK  0xe0u
#define GET_CHP_MEMCONFIG_RESERVED_7_5(__reg__)  (((__reg__) & 0xe0) >> 5u)
#define SET_CHP_MEMCONFIG_RESERVED_7_5(__val__)  (((__val__) << 5u) & 0xe0u)
#define     CHP_MEMCONFIG_DCCM_NUM_SHIFT 8u
#define     CHP_MEMCONFIG_DCCM_NUM_MASK  0x700u
#define GET_CHP_MEMCONFIG_DCCM_NUM(__reg__)  (((__reg__) & 0x700) >> 8u)
#define SET_CHP_MEMCONFIG_DCCM_NUM(__val__)  (((__val__) << 8u) & 0x700u)
#define     CHP_MEMCONFIG_RESERVED_31_11_SHIFT 11u
#define     CHP_MEMCONFIG_RESERVED_31_11_MASK  0xfffff800u
#define GET_CHP_MEMCONFIG_RESERVED_31_11(__reg__)  (((__reg__) & 0xfffff800) >> 11u)
#define SET_CHP_MEMCONFIG_RESERVED_31_11(__val__)  (((__val__) << 11u) & 0xfffff800u)

/** @brief Register definition for @ref CHP_t.Memconfig. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief This bit-field configures the address map on the system bus for accesses to the external Flash. */
        UInt8 flash_size:2;
        /** @brief Reserved bits 3 to 2. Reads as 0. */
        UInt8 reserved_3_2:2;
        /** @brief  */
        UInt8 flash_cfg:1;
        /** @brief Reserved bits 7 to 5. Reads as 0. */
        UInt8 reserved_7_5:3;
        /** @brief Via this bit-field, FW can allocate memory to DCCM in 32kB-blocks from the RAM pool. DCCM has X additional bank */
        UInt8 dccm_num:3;
        /** @brief Reserved bits 31 to 11. Reads as 0. */
        UInt8 reserved_15_11:5;
        /** @brief Reserved bits 31 to 11. Reads as 0. */
        UInt8 reserved_23_16;
        /** @brief Reserved bits 31 to 11. Reads as 0. */
        UInt8 reserved_31_24;
    } bits;
} RegCHPMemconfig_t;

#define REG_CHP_CONTROL ((volatile UInt32*)0xf00110) /*  */
#define     CHP_CONTROL_TOSC_EN_SHIFT 0u
#define     CHP_CONTROL_TOSC_EN_MASK  0x1u
#define GET_CHP_CONTROL_TOSC_EN(__reg__)  (((__reg__) & 0x1) >> 0u)
#define SET_CHP_CONTROL_TOSC_EN(__val__)  (((__val__) << 0u) & 0x1u)
#define     CHP_CONTROL_QSPI_SPEED_SHIFT 1u
#define     CHP_CONTROL_QSPI_SPEED_MASK  0x2u
#define GET_CHP_CONTROL_QSPI_SPEED(__reg__)  (((__reg__) & 0x2) >> 1u)
#define SET_CHP_CONTROL_QSPI_SPEED(__val__)  (((__val__) << 1u) & 0x2u)
#define     CHP_CONTROL_RESERVED_7_2_SHIFT 2u
#define     CHP_CONTROL_RESERVED_7_2_MASK  0xfcu
#define GET_CHP_CONTROL_RESERVED_7_2(__reg__)  (((__reg__) & 0xfc) >> 2u)
#define SET_CHP_CONTROL_RESERVED_7_2(__val__)  (((__val__) << 2u) & 0xfcu)
#define     CHP_CONTROL_RAM_BIST_EN_SHIFT 8u
#define     CHP_CONTROL_RAM_BIST_EN_MASK  0x100u
#define GET_CHP_CONTROL_RAM_BIST_EN(__reg__)  (((__reg__) & 0x100) >> 8u)
#define SET_CHP_CONTROL_RAM_BIST_EN(__val__)  (((__val__) << 8u) & 0x100u)
#define     CHP_CONTROL_RAM_BIST_LCK_SHIFT 9u
#define     CHP_CONTROL_RAM_BIST_LCK_MASK  0x200u
#define GET_CHP_CONTROL_RAM_BIST_LCK(__reg__)  (((__reg__) & 0x200) >> 9u)
#define SET_CHP_CONTROL_RAM_BIST_LCK(__val__)  (((__val__) << 9u) & 0x200u)
#define     CHP_CONTROL_RESERVED_15_10_SHIFT 10u
#define     CHP_CONTROL_RESERVED_15_10_MASK  0xfc00u
#define GET_CHP_CONTROL_RESERVED_15_10(__reg__)  (((__reg__) & 0xfc00) >> 10u)
#define SET_CHP_CONTROL_RESERVED_15_10(__val__)  (((__val__) << 10u) & 0xfc00u)
#define     CHP_CONTROL_OTP_PG_DIS_SHIFT 16u
#define     CHP_CONTROL_OTP_PG_DIS_MASK  0x10000u
#define GET_CHP_CONTROL_OTP_PG_DIS(__reg__)  (((__reg__) & 0x10000) >> 16u)
#define SET_CHP_CONTROL_OTP_PG_DIS(__val__)  (((__val__) << 16u) & 0x10000u)
#define     CHP_CONTROL_OTP_PG_LCK_SHIFT 17u
#define     CHP_CONTROL_OTP_PG_LCK_MASK  0x20000u
#define GET_CHP_CONTROL_OTP_PG_LCK(__reg__)  (((__reg__) & 0x20000) >> 17u)
#define SET_CHP_CONTROL_OTP_PG_LCK(__val__)  (((__val__) << 17u) & 0x20000u)
#define     CHP_CONTROL_RESERVED_23_18_SHIFT 18u
#define     CHP_CONTROL_RESERVED_23_18_MASK  0xfc0000u
#define GET_CHP_CONTROL_RESERVED_23_18(__reg__)  (((__reg__) & 0xfc0000) >> 18u)
#define SET_CHP_CONTROL_RESERVED_23_18(__val__)  (((__val__) << 18u) & 0xfc0000u)
#define     CHP_CONTROL_OTP_ACC_DIS_SHIFT 24u
#define     CHP_CONTROL_OTP_ACC_DIS_MASK  0x1000000u
#define GET_CHP_CONTROL_OTP_ACC_DIS(__reg__)  (((__reg__) & 0x1000000) >> 24u)
#define SET_CHP_CONTROL_OTP_ACC_DIS(__val__)  (((__val__) << 24u) & 0x1000000u)
#define     CHP_CONTROL_OTP_ACC_LCK_SHIFT 25u
#define     CHP_CONTROL_OTP_ACC_LCK_MASK  0x2000000u
#define GET_CHP_CONTROL_OTP_ACC_LCK(__reg__)  (((__reg__) & 0x2000000) >> 25u)
#define SET_CHP_CONTROL_OTP_ACC_LCK(__val__)  (((__val__) << 25u) & 0x2000000u)
#define     CHP_CONTROL_RESERVED_31_26_SHIFT 26u
#define     CHP_CONTROL_RESERVED_31_26_MASK  0xfc000000u
#define GET_CHP_CONTROL_RESERVED_31_26(__reg__)  (((__reg__) & 0xfc000000) >> 26u)
#define SET_CHP_CONTROL_RESERVED_31_26(__val__)  (((__val__) << 26u) & 0xfc000000u)

/** @brief Register definition for @ref CHP_t.Control. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief When asserted, enabled the low speed timer oscillator. */
        UInt8 tosc_en:1;
        /** @brief  */
        UInt8 qspi_speed:1;
        /** @brief Reserved bits 7 to 2. Reads as 0. */
        UInt8 reserved_7_2:6;
        /** @brief  */
        UInt8 ram_bist_en:1;
        /** @brief  */
        UInt8 ram_bist_lck:1;
        /** @brief Reserved bits 15 to 10. Reads as 0. */
        UInt8 reserved_15_10:6;
        /** @brief  */
        UInt8 otp_pg_dis:1;
        /** @brief  */
        UInt8 otp_pg_lck:1;
        /** @brief Reserved bits 23 to 18. Reads as 0. */
        UInt8 reserved_23_18:6;
        /** @brief  */
        UInt8 otp_acc_dis:1;
        /** @brief  */
        UInt8 otp_acc_lck:1;
        /** @brief Reserved bits 31 to 26. Reads as 0. */
        UInt8 reserved_31_26:6;
    } bits;
} RegCHPControl_t;

#define REG_CHP_DIAGNOSTIC ((volatile UInt32*)0xf00114) /*  */
#define     CHP_DIAGNOSTIC_PM_STATE_SHIFT 0u
#define     CHP_DIAGNOSTIC_PM_STATE_MASK  0xfu
#define GET_CHP_DIAGNOSTIC_PM_STATE(__reg__)  (((__reg__) & 0xf) >> 0u)
#define SET_CHP_DIAGNOSTIC_PM_STATE(__val__)  (((__val__) << 0u) & 0xfu)
#define     CHP_DIAGNOSTIC_PM_STATE_PWRUP_1 0x0u
#define     CHP_DIAGNOSTIC_PM_STATE_PWRUP_2 0x1u
#define     CHP_DIAGNOSTIC_PM_STATE_ACTV_1 0x2u
#define     CHP_DIAGNOSTIC_PM_STATE_ACTV_2 0x3u
#define     CHP_DIAGNOSTIC_PM_STATE_LSLP_1 0x4u
#define     CHP_DIAGNOSTIC_PM_STATE_LSLP_2 0x5u
#define     CHP_DIAGNOSTIC_PM_STATE_RAMP 0x6u
#define     CHP_DIAGNOSTIC_PM_STATE_SLP 0x8u
#define     CHP_DIAGNOSTIC_PM_STATE_DSLP 0x9u
#define     CHP_DIAGNOSTIC_PM_STATE_TEST_MODE 0xfu

#define     CHP_DIAGNOSTIC_ROM_ACC_ERR_SHIFT 4u
#define     CHP_DIAGNOSTIC_ROM_ACC_ERR_MASK  0x10u
#define GET_CHP_DIAGNOSTIC_ROM_ACC_ERR(__reg__)  (((__reg__) & 0x10) >> 4u)
#define SET_CHP_DIAGNOSTIC_ROM_ACC_ERR(__val__)  (((__val__) << 4u) & 0x10u)
#define     CHP_DIAGNOSTIC_ICCM_ACC_ERR_SHIFT 5u
#define     CHP_DIAGNOSTIC_ICCM_ACC_ERR_MASK  0x20u
#define GET_CHP_DIAGNOSTIC_ICCM_ACC_ERR(__reg__)  (((__reg__) & 0x20) >> 5u)
#define SET_CHP_DIAGNOSTIC_ICCM_ACC_ERR(__val__)  (((__val__) << 5u) & 0x20u)
#define     CHP_DIAGNOSTIC_DCCM_ACC_ERR_SHIFT 6u
#define     CHP_DIAGNOSTIC_DCCM_ACC_ERR_MASK  0x40u
#define GET_CHP_DIAGNOSTIC_DCCM_ACC_ERR(__reg__)  (((__reg__) & 0x40) >> 6u)
#define SET_CHP_DIAGNOSTIC_DCCM_ACC_ERR(__val__)  (((__val__) << 6u) & 0x40u)
#define     CHP_DIAGNOSTIC_RAM_BIST_ERR_SHIFT 7u
#define     CHP_DIAGNOSTIC_RAM_BIST_ERR_MASK  0x80u
#define GET_CHP_DIAGNOSTIC_RAM_BIST_ERR(__reg__)  (((__reg__) & 0x80) >> 7u)
#define SET_CHP_DIAGNOSTIC_RAM_BIST_ERR(__val__)  (((__val__) << 7u) & 0x80u)
#define     CHP_DIAGNOSTIC_RST_SRC_SHIFT 8u
#define     CHP_DIAGNOSTIC_RST_SRC_MASK  0x700u
#define GET_CHP_DIAGNOSTIC_RST_SRC(__reg__)  (((__reg__) & 0x700) >> 8u)
#define SET_CHP_DIAGNOSTIC_RST_SRC(__val__)  (((__val__) << 8u) & 0x700u)
#define     CHP_DIAGNOSTIC_RST_SRC_EXTERNAL_RESET 0x1u
#define     CHP_DIAGNOSTIC_RST_SRC_HOST_RESET 0x2u
#define     CHP_DIAGNOSTIC_RST_SRC_WATCHDOG_RESET 0x4u

#define     CHP_DIAGNOSTIC_RESERVED_15_11_SHIFT 11u
#define     CHP_DIAGNOSTIC_RESERVED_15_11_MASK  0xf800u
#define GET_CHP_DIAGNOSTIC_RESERVED_15_11(__reg__)  (((__reg__) & 0xf800) >> 11u)
#define SET_CHP_DIAGNOSTIC_RESERVED_15_11(__val__)  (((__val__) << 11u) & 0xf800u)
#define     CHP_DIAGNOSTIC_QSPI_ACC_ERR_SHIFT 16u
#define     CHP_DIAGNOSTIC_QSPI_ACC_ERR_MASK  0x10000u
#define GET_CHP_DIAGNOSTIC_QSPI_ACC_ERR(__reg__)  (((__reg__) & 0x10000) >> 16u)
#define SET_CHP_DIAGNOSTIC_QSPI_ACC_ERR(__val__)  (((__val__) << 16u) & 0x10000u)
#define     CHP_DIAGNOSTIC_RESERVED_31_17_SHIFT 17u
#define     CHP_DIAGNOSTIC_RESERVED_31_17_MASK  0xfffe0000u
#define GET_CHP_DIAGNOSTIC_RESERVED_31_17(__reg__)  (((__reg__) & 0xfffe0000) >> 17u)
#define SET_CHP_DIAGNOSTIC_RESERVED_31_17(__val__)  (((__val__) << 17u) & 0xfffe0000u)

/** @brief Register definition for @ref CHP_t.Diagnostic. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief This bit-field reflects the current state of the power manager. */
        UInt8 pm_state:4;
        /** @brief When '1', this bit indicates that a write access was attempted to a ROM location since the last clearing of the bit. The bit can be cleared by writing a '1' to it. */
        UInt8 rom_acc_err:1;
        /** @brief When '1', this bit indicates that a reserved location in the ICCM memory region has been accessed since the last clearing of the bit. The bit can be cleared by writing a '1' to it. */
        UInt8 iccm_acc_err:1;
        /** @brief When '1', this bit indicates that a reserved location in the DCCM memory region has been accessed since the last clearing of the bit. The bit can be cleared by writing a '1' to it. */
        UInt8 dccm_acc_err:1;
        /** @brief  */
        UInt8 ram_bist_err:1;
        /** @brief This bit-field relays the source of a reset. It is cleared on a POR. A bit in this bit-field can be cleared by writing a '1' to it. */
        UInt8 rst_src:3;
        /** @brief Reserved bits 15 to 11. Reads as 0. */
        UInt8 reserved_15_11:5;
        /** @brief  */
        UInt16 qspi_acc_err:1;
        /** @brief Reserved bits 31 to 17. Reads as 0. */
        UInt16 reserved_31_17:15;
    } bits;
} RegCHPDiagnostic_t;

/** @brief Component definition for @ref CHP. */
typedef struct {
    /** @brief  */
    RegCHPTrim0_t Trim0;

    /** @brief  */
    RegCHPTrim1_t Trim1;

    /** @brief  */
    RegCHPPowerconfig_t Powerconfig;

    /** @brief  */
    RegCHPMemconfig_t Memconfig;

    /** @brief  */
    RegCHPControl_t Control;

    /** @brief  */
    RegCHPDiagnostic_t Diagnostic;

} CHP_t;

/** @brief  */
extern volatile CHP_t CHP;


#define REG_FCC_CONTROL ((volatile UInt32*)0xf00190) /*  */
#define     FCC_CONTROL_FCC_EN_SHIFT 0u
#define     FCC_CONTROL_FCC_EN_MASK  0x1u
#define GET_FCC_CONTROL_FCC_EN(__reg__)  (((__reg__) & 0x1) >> 0u)
#define SET_FCC_CONTROL_FCC_EN(__val__)  (((__val__) << 0u) & 0x1u)
#define     FCC_CONTROL_RESERVED_31_1_SHIFT 1u
#define     FCC_CONTROL_RESERVED_31_1_MASK  0xfffffffeu
#define GET_FCC_CONTROL_RESERVED_31_1(__reg__)  (((__reg__) & 0xfffffffe) >> 1u)
#define SET_FCC_CONTROL_RESERVED_31_1(__val__)  (((__val__) << 1u) & 0xfffffffeu)

/** @brief Register definition for @ref FCC_t.Control. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief When asserted '1', this bit enables the eFlash cache controller. */
        UInt32 fcc_en:1;
        /** @brief Reserved bits 31 to 1. Reads as 0. */
        UInt32 reserved_31_1:31;
    } bits;
} RegFCCControl_t;

/** @brief Component definition for @ref FCC. */
typedef struct {
    /** @brief  */
    RegFCCControl_t Control;

} FCC_t;

/** @brief  */
extern volatile FCC_t FCC;


#define REG_FPGA_SENSOR_CONFIG ((volatile UInt32*)0xf002fc) /* This bit-field shows the current value of the real time counter. */
#define     FPGA_SENSOR_CONFIG_MAG_SHIFT 0u
#define     FPGA_SENSOR_CONFIG_MAG_MASK  0x1u
#define GET_FPGA_SENSOR_CONFIG_MAG(__reg__)  (((__reg__) & 0x1) >> 0u)
#define SET_FPGA_SENSOR_CONFIG_MAG(__val__)  (((__val__) << 0u) & 0x1u)
#define     FPGA_SENSOR_CONFIG_MAG_BMM150 0x0u
#define     FPGA_SENSOR_CONFIG_MAG_AKM09915 0x1u

#define     FPGA_SENSOR_CONFIG_ACCEL_SHIFT 1u
#define     FPGA_SENSOR_CONFIG_ACCEL_MASK  0x2u
#define GET_FPGA_SENSOR_CONFIG_ACCEL(__reg__)  (((__reg__) & 0x2) >> 1u)
#define SET_FPGA_SENSOR_CONFIG_ACCEL(__val__)  (((__val__) << 1u) & 0x2u)
#define     FPGA_SENSOR_CONFIG_ACCEL_BMI160 0x0u
#define     FPGA_SENSOR_CONFIG_ACCEL_BMA2XY 0x1u


/** @brief Register definition for @ref FPGA_t.SensorConfig. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief The magnetometer selection for use with the FPGA envorinment. NOP on ASIC. */
        UInt8 mag:1;
        /** @brief The accelerometer/gyroscope selection for use with the FPGA envorinment. NOP on ASIC */
        UInt8 accel:1;
    } bits;
} RegFPGASensorConfig_t;

/** @brief Component definition for @ref FPGA. */
typedef struct {
    /** @brief This bit-field shows the current value of the real time counter. */
    RegFPGASensorConfig_t SensorConfig;

} FPGA_t;

/** @brief FPGA specific configuration bits, no available on the ASIC. */
extern volatile FPGA_t FPGA;


#define REG_ITMR_CONTROL ((volatile UInt32*)0xf001a0) /*  */
#define     ITMR_CONTROL_ITMR_EN_SHIFT 0u
#define     ITMR_CONTROL_ITMR_EN_MASK  0x1u
#define GET_ITMR_CONTROL_ITMR_EN(__reg__)  (((__reg__) & 0x1) >> 0u)
#define SET_ITMR_CONTROL_ITMR_EN(__val__)  (((__val__) << 0u) & 0x1u)
#define     ITMR_CONTROL_ITMR_CLR_SHIFT 1u
#define     ITMR_CONTROL_ITMR_CLR_MASK  0x2u
#define GET_ITMR_CONTROL_ITMR_CLR(__reg__)  (((__reg__) & 0x2) >> 1u)
#define SET_ITMR_CONTROL_ITMR_CLR(__val__)  (((__val__) << 1u) & 0x2u)
#define     ITMR_CONTROL_RESERVED_31_2_SHIFT 2u
#define     ITMR_CONTROL_RESERVED_31_2_MASK  0xfffffffcu
#define GET_ITMR_CONTROL_RESERVED_31_2(__reg__)  (((__reg__) & 0xfffffffc) >> 2u)
#define SET_ITMR_CONTROL_RESERVED_31_2(__val__)  (((__val__) << 2u) & 0xfffffffcu)

/** @brief Register definition for @ref ITMR_t.Control. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief When asserted '1', this bit enables the interval timer. */
        UInt32 itmr_en:1;
        /** @brief When asserted '1', this bit clears the interval timer to zero. The bit automatically de-asserts, when the clearing of the interval timer is carried out. */
        UInt32 itmr_clr:1;
        /** @brief Reserved bits 31 to 2. Reads as 0. */
        UInt32 reserved_31_2:30;
    } bits;
} RegITMRControl_t;

#define REG_ITMR_LIMIT ((volatile UInt32*)0xf001a4) /*  */
#define     ITMR_LIMIT_ITMR_LIMIT_SHIFT 0u
#define     ITMR_LIMIT_ITMR_LIMIT_MASK  0xffffffffu
#define GET_ITMR_LIMIT_ITMR_LIMIT(__reg__)  (((__reg__) & 0xffffffff) >> 0u)
#define SET_ITMR_LIMIT_ITMR_LIMIT(__val__)  (((__val__) << 0u) & 0xffffffffu)

/** @brief Register definition for @ref ITMR_t.Limit. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt32 itmr_limit;
    } bits;
} RegITMRLimit_t;

#define REG_ITMR_VALUE ((volatile UInt32*)0xf001a8) /*  */
#define     ITMR_VALUE_ITMR_VALUE_SHIFT 0u
#define     ITMR_VALUE_ITMR_VALUE_MASK  0xffffffffu
#define GET_ITMR_VALUE_ITMR_VALUE(__reg__)  (((__reg__) & 0xffffffff) >> 0u)
#define SET_ITMR_VALUE_ITMR_VALUE(__val__)  (((__val__) << 0u) & 0xffffffffu)

/** @brief Register definition for @ref ITMR_t.Value. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt32 itmr_value;
    } bits;
} RegITMRValue_t;

/** @brief Component definition for @ref ITMR. */
typedef struct {
    /** @brief  */
    RegITMRControl_t Control;

    /** @brief  */
    RegITMRLimit_t Limit;

    /** @brief  */
    RegITMRValue_t Value;

} ITMR_t;

/** @brief  */
extern volatile ITMR_t ITMR;


#define REG_OTP_CONTROL ((volatile UInt32*)0xf00200) /*  */
#define     OTP_CONTROL_OTP_VDD_EN_Q_SHIFT 0u
#define     OTP_CONTROL_OTP_VDD_EN_Q_MASK  0x1u
#define GET_OTP_CONTROL_OTP_VDD_EN_Q(__reg__)  (((__reg__) & 0x1) >> 0u)
#define SET_OTP_CONTROL_OTP_VDD_EN_Q(__val__)  (((__val__) << 0u) & 0x1u)
#define     OTP_CONTROL_OTP_VDDQ_EN_Q_SHIFT 1u
#define     OTP_CONTROL_OTP_VDDQ_EN_Q_MASK  0x2u
#define GET_OTP_CONTROL_OTP_VDDQ_EN_Q(__reg__)  (((__reg__) & 0x2) >> 1u)
#define SET_OTP_CONTROL_OTP_VDDQ_EN_Q(__val__)  (((__val__) << 1u) & 0x2u)
#define     OTP_CONTROL_OTP_PGENB_Q_SHIFT 2u
#define     OTP_CONTROL_OTP_PGENB_Q_MASK  0x4u
#define GET_OTP_CONTROL_OTP_PGENB_Q(__reg__)  (((__reg__) & 0x4) >> 2u)
#define SET_OTP_CONTROL_OTP_PGENB_Q(__val__)  (((__val__) << 2u) & 0x4u)
#define     OTP_CONTROL_OTP_CSB_Q_SHIFT 3u
#define     OTP_CONTROL_OTP_CSB_Q_MASK  0x8u
#define GET_OTP_CONTROL_OTP_CSB_Q(__reg__)  (((__reg__) & 0x8) >> 3u)
#define SET_OTP_CONTROL_OTP_CSB_Q(__val__)  (((__val__) << 3u) & 0x8u)
#define     OTP_CONTROL_OTP_LOAD_Q_SHIFT 4u
#define     OTP_CONTROL_OTP_LOAD_Q_MASK  0x10u
#define GET_OTP_CONTROL_OTP_LOAD_Q(__reg__)  (((__reg__) & 0x10) >> 4u)
#define SET_OTP_CONTROL_OTP_LOAD_Q(__val__)  (((__val__) << 4u) & 0x10u)
#define     OTP_CONTROL_RESERVED_15_5_SHIFT 5u
#define     OTP_CONTROL_RESERVED_15_5_MASK  0xffe0u
#define GET_OTP_CONTROL_RESERVED_15_5(__reg__)  (((__reg__) & 0xffe0) >> 5u)
#define SET_OTP_CONTROL_RESERVED_15_5(__val__)  (((__val__) << 5u) & 0xffe0u)
#define     OTP_CONTROL_OTP_ADDR_SHIFT 16u
#define     OTP_CONTROL_OTP_ADDR_MASK  0x3ff0000u
#define GET_OTP_CONTROL_OTP_ADDR(__reg__)  (((__reg__) & 0x3ff0000) >> 16u)
#define SET_OTP_CONTROL_OTP_ADDR(__val__)  (((__val__) << 16u) & 0x3ff0000u)
#define     OTP_CONTROL_RESERVED_31_26_SHIFT 26u
#define     OTP_CONTROL_RESERVED_31_26_MASK  0xfc000000u
#define GET_OTP_CONTROL_RESERVED_31_26(__reg__)  (((__reg__) & 0xfc000000) >> 26u)
#define SET_OTP_CONTROL_RESERVED_31_26(__val__)  (((__val__) << 26u) & 0xfc000000u)

/** @brief Register definition for @ref OTP_t.Control. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt16 otp_vdd_en_q:1;
        /** @brief  */
        UInt16 otp_vddq_en_q:1;
        /** @brief  */
        UInt16 otp_pgenb_q:1;
        /** @brief  */
        UInt16 otp_csb_q:1;
        /** @brief  */
        UInt16 otp_load_q:1;
        /** @brief Reserved bits 15 to 5. Reads as 0. */
        UInt16 reserved_15_5:11;
        /** @brief  */
        UInt16 otp_addr:10;
        /** @brief Reserved bits 31 to 26. Reads as 0. */
        UInt16 reserved_31_26:6;
    } bits;
} RegOTPControl_t;

#define REG_OTP_STROBE ((volatile UInt32*)0xf00204) /*  */
#define     OTP_STROBE_OTP_STROBE_Q_SHIFT 0u
#define     OTP_STROBE_OTP_STROBE_Q_MASK  0x1u
#define GET_OTP_STROBE_OTP_STROBE_Q(__reg__)  (((__reg__) & 0x1) >> 0u)
#define SET_OTP_STROBE_OTP_STROBE_Q(__val__)  (((__val__) << 0u) & 0x1u)
#define     OTP_STROBE_RESERVED_31_1_SHIFT 1u
#define     OTP_STROBE_RESERVED_31_1_MASK  0xfffffffeu
#define GET_OTP_STROBE_RESERVED_31_1(__reg__)  (((__reg__) & 0xfffffffe) >> 1u)
#define SET_OTP_STROBE_RESERVED_31_1(__val__)  (((__val__) << 1u) & 0xfffffffeu)

/** @brief Register definition for @ref OTP_t.Strobe. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt32 otp_strobe_q:1;
        /** @brief Reserved bits 31 to 1. Reads as 0. */
        UInt32 reserved_31_1:31;
    } bits;
} RegOTPStrobe_t;

#define REG_OTP_DATA ((volatile UInt32*)0xf00208) /*  */
#define     OTP_DATA_OTP_RD_DATA_SHIFT 0u
#define     OTP_DATA_OTP_RD_DATA_MASK  0xffu
#define GET_OTP_DATA_OTP_RD_DATA(__reg__)  (((__reg__) & 0xff) >> 0u)
#define SET_OTP_DATA_OTP_RD_DATA(__val__)  (((__val__) << 0u) & 0xffu)
#define     OTP_DATA_RESERVED_15_8_SHIFT 8u
#define     OTP_DATA_RESERVED_15_8_MASK  0xff00u
#define GET_OTP_DATA_RESERVED_15_8(__reg__)  (((__reg__) & 0xff00) >> 8u)
#define SET_OTP_DATA_RESERVED_15_8(__val__)  (((__val__) << 8u) & 0xff00u)
#define     OTP_DATA_OTP_VDDQ_SW_ON_SYNC_SHIFT 16u
#define     OTP_DATA_OTP_VDDQ_SW_ON_SYNC_MASK  0x10000u
#define GET_OTP_DATA_OTP_VDDQ_SW_ON_SYNC(__reg__)  (((__reg__) & 0x10000) >> 16u)
#define SET_OTP_DATA_OTP_VDDQ_SW_ON_SYNC(__val__)  (((__val__) << 16u) & 0x10000u)
#define     OTP_DATA_RESERVED_31_17_SHIFT 17u
#define     OTP_DATA_RESERVED_31_17_MASK  0xfffe0000u
#define GET_OTP_DATA_RESERVED_31_17(__reg__)  (((__reg__) & 0xfffe0000) >> 17u)
#define SET_OTP_DATA_RESERVED_31_17(__val__)  (((__val__) << 17u) & 0xfffe0000u)

/** @brief Register definition for @ref OTP_t.Data. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt8 otp_rd_data;
        /** @brief Reserved bits 15 to 8. Reads as 0. */
        UInt8 reserved_15_8;
        /** @brief  */
        UInt16 otp_vddq_sw_on_sync:1;
        /** @brief Reserved bits 31 to 17. Reads as 0. */
        UInt16 reserved_31_17:15;
    } bits;
} RegOTPData_t;

/** @brief Component definition for @ref OTP. */
typedef struct {
    /** @brief  */
    RegOTPControl_t Control;

    /** @brief  */
    RegOTPStrobe_t Strobe;

    /** @brief  */
    RegOTPData_t Data;

} OTP_t;

/** @brief  */
extern volatile OTP_t OTP;


#define REG_PAD_CONTROL0 ((volatile UInt32*)0xf00240) /*  */
#define     PAD_CONTROL0_RESERVED_7_0_SHIFT 0u
#define     PAD_CONTROL0_RESERVED_7_0_MASK  0xffu
#define GET_PAD_CONTROL0_RESERVED_7_0(__reg__)  (((__reg__) & 0xff) >> 0u)
#define SET_PAD_CONTROL0_RESERVED_7_0(__val__)  (((__val__) << 0u) & 0xffu)
#define     PAD_CONTROL0_PAD_MFP_FUNC_SHIFT 8u
#define     PAD_CONTROL0_PAD_MFP_FUNC_MASK  0x7ff00u
#define GET_PAD_CONTROL0_PAD_MFP_FUNC(__reg__)  (((__reg__) & 0x7ff00) >> 8u)
#define SET_PAD_CONTROL0_PAD_MFP_FUNC(__val__)  (((__val__) << 8u) & 0x7ff00u)
#define     PAD_CONTROL0_PAD_MFP_FUNC_GPIO 0x0u
#define     PAD_CONTROL0_PAD_MFP_FUNC_QSPI_CLK 0x1u
#define     PAD_CONTROL0_PAD_MFP_FUNC_QSPI_CSN 0x2u
#define     PAD_CONTROL0_PAD_MFP_FUNC_QSPI_IO0 0x4u
#define     PAD_CONTROL0_PAD_MFP_FUNC_QSPI_IO1 0x8u
#define     PAD_CONTROL0_PAD_MFP_FUNC_QSPI_IO2 0x10u
#define     PAD_CONTROL0_PAD_MFP_FUNC_QSPI_IO3 0x20u
#define     PAD_CONTROL0_PAD_MFP_FUNC_SIF2_CLK 0x40u
#define     PAD_CONTROL0_PAD_MFP_FUNC_SIF2_DIO 0x80u
#define     PAD_CONTROL0_PAD_MFP_FUNC_SIF2_DIN 0x100u
#define     PAD_CONTROL0_PAD_MFP_FUNC_SIF3_SCL 0x200u
#define     PAD_CONTROL0_PAD_MFP_FUNC_SIF3_SDA 0x400u

#define     PAD_CONTROL0_RESERVED_31_19_SHIFT 19u
#define     PAD_CONTROL0_RESERVED_31_19_MASK  0xfff80000u
#define GET_PAD_CONTROL0_RESERVED_31_19(__reg__)  (((__reg__) & 0xfff80000) >> 19u)
#define SET_PAD_CONTROL0_RESERVED_31_19(__val__)  (((__val__) << 19u) & 0xfff80000u)

/** @brief Register definition for @ref PAD_t.Control0. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief Reserved bits 7 to 0. Reads as 0. */
        UInt32 reserved_7_0:8;
        /** @brief Each bit determines whether the respective general-function pad carries out function 1 or function 2 */
        UInt32 pad_mfp_func:11;
        /** @brief Reserved bits 31 to 19. Reads as 0. */
        UInt32 reserved_31_19:13;
    } bits;
} RegPADControl0_t;

#define REG_PAD_CONTROL1 ((volatile UInt32*)0xf00244) /*  */
#define     PAD_CONTROL1_PAD_JTAG_EN_SHIFT 0u
#define     PAD_CONTROL1_PAD_JTAG_EN_MASK  0x1u
#define GET_PAD_CONTROL1_PAD_JTAG_EN(__reg__)  (((__reg__) & 0x1) >> 0u)
#define SET_PAD_CONTROL1_PAD_JTAG_EN(__val__)  (((__val__) << 0u) & 0x1u)
#define     PAD_CONTROL1_PAD_JTAG_LCK_SHIFT 1u
#define     PAD_CONTROL1_PAD_JTAG_LCK_MASK  0x2u
#define GET_PAD_CONTROL1_PAD_JTAG_LCK(__reg__)  (((__reg__) & 0x2) >> 1u)
#define SET_PAD_CONTROL1_PAD_JTAG_LCK(__val__)  (((__val__) << 1u) & 0x2u)
#define     PAD_CONTROL1_RESERVED_31_2_SHIFT 2u
#define     PAD_CONTROL1_RESERVED_31_2_MASK  0xfffffffcu
#define GET_PAD_CONTROL1_RESERVED_31_2(__reg__)  (((__reg__) & 0xfffffffc) >> 2u)
#define SET_PAD_CONTROL1_RESERVED_31_2(__val__)  (((__val__) << 2u) & 0xfffffffcu)

/** @brief Register definition for @ref PAD_t.Control1. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief Asserting this bit '1', enables the JTAG interface. The bit can be written only once (either to '0' or '1'); thereafter, it is locked and can be unlocked only via a power-on event or reset. */
        UInt32 pad_jtag_en:1;
        /** @brief This bit asserts '1' upon the first write to this register. It indicates that the pad_jtag_en bit has been locked and can no longer be modified. */
        UInt32 pad_jtag_lck:1;
        /** @brief Reserved bits 31 to 2. Reads as 0. */
        UInt32 reserved_31_2:30;
    } bits;
} RegPADControl1_t;

#define REG_PAD_CONTROL2 ((volatile UInt32*)0xf00248) /*  */
#define     PAD_CONTROL2_PAD_HIF_DIS_SHIFT 0u
#define     PAD_CONTROL2_PAD_HIF_DIS_MASK  0x1u
#define GET_PAD_CONTROL2_PAD_HIF_DIS(__reg__)  (((__reg__) & 0x1) >> 0u)
#define SET_PAD_CONTROL2_PAD_HIF_DIS(__val__)  (((__val__) << 0u) & 0x1u)
#define     PAD_CONTROL2_PAD_HIF_LCK_SHIFT 1u
#define     PAD_CONTROL2_PAD_HIF_LCK_MASK  0x2u
#define GET_PAD_CONTROL2_PAD_HIF_LCK(__reg__)  (((__reg__) & 0x2) >> 1u)
#define SET_PAD_CONTROL2_PAD_HIF_LCK(__val__)  (((__val__) << 1u) & 0x2u)
#define     PAD_CONTROL2_RESERVED_31_2_SHIFT 2u
#define     PAD_CONTROL2_RESERVED_31_2_MASK  0xfffffffcu
#define GET_PAD_CONTROL2_RESERVED_31_2(__reg__)  (((__reg__) & 0xfffffffc) >> 2u)
#define SET_PAD_CONTROL2_RESERVED_31_2(__val__)  (((__val__) << 2u) & 0xfffffffcu)

/** @brief Register definition for @ref PAD_t.Control2. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief Asserting this bit '1', disables the host interface (HCLK, HCSN, HDIO0 and HDI01 - they become GPIO's). The bit can be written only once (either to '0' or '1'); thereafter, it is locked and can be unlocked only via a power-on event or reset. HIRQ never gets locked. */
        UInt32 pad_hif_dis:1;
        /** @brief This bit asserts '1' upon the first write to this register. It indicates that the pad_hif_dis bit has been locked and can no longer be modified. */
        UInt32 pad_hif_lck:1;
        /** @brief Reserved bits 31 to 2. Reads as 0. */
        UInt32 reserved_31_2:30;
    } bits;
} RegPADControl2_t;

#define REG_PAD_DRIVECONTROL ((volatile UInt32*)0xf0024c) /*  */
#define     PAD_DRIVECONTROL_MFPAD_DRIVE_SHIFT 0u
#define     PAD_DRIVECONTROL_MFPAD_DRIVE_MASK  0x1ffffffu
#define GET_PAD_DRIVECONTROL_MFPAD_DRIVE(__reg__)  (((__reg__) & 0x1ffffff) >> 0u)
#define SET_PAD_DRIVECONTROL_MFPAD_DRIVE(__val__)  (((__val__) << 0u) & 0x1ffffffu)
#define     PAD_DRIVECONTROL_SIF1CLK_DRIVE_SHIFT 25u
#define     PAD_DRIVECONTROL_SIF1CLK_DRIVE_MASK  0x2000000u
#define GET_PAD_DRIVECONTROL_SIF1CLK_DRIVE(__reg__)  (((__reg__) & 0x2000000) >> 25u)
#define SET_PAD_DRIVECONTROL_SIF1CLK_DRIVE(__val__)  (((__val__) << 25u) & 0x2000000u)
#define     PAD_DRIVECONTROL_SIF1CSN_DRIVE_SHIFT 26u
#define     PAD_DRIVECONTROL_SIF1CSN_DRIVE_MASK  0x4000000u
#define GET_PAD_DRIVECONTROL_SIF1CSN_DRIVE(__reg__)  (((__reg__) & 0x4000000) >> 26u)
#define SET_PAD_DRIVECONTROL_SIF1CSN_DRIVE(__val__)  (((__val__) << 26u) & 0x4000000u)
#define     PAD_DRIVECONTROL_SIF1DIO_DRIVE_SHIFT 27u
#define     PAD_DRIVECONTROL_SIF1DIO_DRIVE_MASK  0x8000000u
#define GET_PAD_DRIVECONTROL_SIF1DIO_DRIVE(__reg__)  (((__reg__) & 0x8000000) >> 27u)
#define SET_PAD_DRIVECONTROL_SIF1DIO_DRIVE(__val__)  (((__val__) << 27u) & 0x8000000u)
#define     PAD_DRIVECONTROL_RESERVED_31_28_SHIFT 28u
#define     PAD_DRIVECONTROL_RESERVED_31_28_MASK  0xf0000000u
#define GET_PAD_DRIVECONTROL_RESERVED_31_28(__reg__)  (((__reg__) & 0xf0000000) >> 28u)
#define SET_PAD_DRIVECONTROL_RESERVED_31_28(__val__)  (((__val__) << 28u) & 0xf0000000u)

/** @brief Register definition for @ref PAD_t.Drivecontrol. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt32 mfpad_drive:25;
        /** @brief  */
        UInt32 sif1clk_drive:1;
        /** @brief  */
        UInt32 sif1csn_drive:1;
        /** @brief  */
        UInt32 sif1dio_drive:1;
        /** @brief Reserved bits 31 to 28. Reads as 0. */
        UInt32 reserved_31_28:4;
    } bits;
} RegPADDrivecontrol_t;

#define REG_PAD_PULLENABLE ((volatile UInt32*)0xf00250) /*  */
#define     PAD_PULLENABLE_MFPAD_PEN_SHIFT 0u
#define     PAD_PULLENABLE_MFPAD_PEN_MASK  0x1ffffffu
#define GET_PAD_PULLENABLE_MFPAD_PEN(__reg__)  (((__reg__) & 0x1ffffff) >> 0u)
#define SET_PAD_PULLENABLE_MFPAD_PEN(__val__)  (((__val__) << 0u) & 0x1ffffffu)
#define     PAD_PULLENABLE_SIF1CLK_PEN_SHIFT 25u
#define     PAD_PULLENABLE_SIF1CLK_PEN_MASK  0x2000000u
#define GET_PAD_PULLENABLE_SIF1CLK_PEN(__reg__)  (((__reg__) & 0x2000000) >> 25u)
#define SET_PAD_PULLENABLE_SIF1CLK_PEN(__val__)  (((__val__) << 25u) & 0x2000000u)
#define     PAD_PULLENABLE_SIF1DIN_PEN_SHIFT 26u
#define     PAD_PULLENABLE_SIF1DIN_PEN_MASK  0x4000000u
#define GET_PAD_PULLENABLE_SIF1DIN_PEN(__reg__)  (((__reg__) & 0x4000000) >> 26u)
#define SET_PAD_PULLENABLE_SIF1DIN_PEN(__val__)  (((__val__) << 26u) & 0x4000000u)
#define     PAD_PULLENABLE_SIF1DIO_PEN_SHIFT 27u
#define     PAD_PULLENABLE_SIF1DIO_PEN_MASK  0x8000000u
#define GET_PAD_PULLENABLE_SIF1DIO_PEN(__reg__)  (((__reg__) & 0x8000000) >> 27u)
#define SET_PAD_PULLENABLE_SIF1DIO_PEN(__val__)  (((__val__) << 27u) & 0x8000000u)
#define     PAD_PULLENABLE_RESERVED_31_28_SHIFT 28u
#define     PAD_PULLENABLE_RESERVED_31_28_MASK  0xf0000000u
#define GET_PAD_PULLENABLE_RESERVED_31_28(__reg__)  (((__reg__) & 0xf0000000) >> 28u)
#define SET_PAD_PULLENABLE_RESERVED_31_28(__val__)  (((__val__) << 28u) & 0xf0000000u)

/** @brief Register definition for @ref PAD_t.Pullenable. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt32 mfpad_pen:25;
        /** @brief  */
        UInt32 sif1clk_pen:1;
        /** @brief  */
        UInt32 sif1din_pen:1;
        /** @brief  */
        UInt32 sif1dio_pen:1;
        /** @brief Reserved bits 31 to 28. Reads as 0. */
        UInt32 reserved_31_28:4;
    } bits;
} RegPADPullenable_t;

#define REG_PAD_GPIODOUT ((volatile UInt32*)0xf00254) /*  */
#define     PAD_GPIODOUT_GPIO_DOUT_SHIFT 0u
#define     PAD_GPIODOUT_GPIO_DOUT_MASK  0x1ffffffu
#define GET_PAD_GPIODOUT_GPIO_DOUT(__reg__)  (((__reg__) & 0x1ffffff) >> 0u)
#define SET_PAD_GPIODOUT_GPIO_DOUT(__val__)  (((__val__) << 0u) & 0x1ffffffu)
#define     PAD_GPIODOUT_SIF1CSN_ROUT_SHIFT 25u
#define     PAD_GPIODOUT_SIF1CSN_ROUT_MASK  0x2000000u
#define GET_PAD_GPIODOUT_SIF1CSN_ROUT(__reg__)  (((__reg__) & 0x2000000) >> 25u)
#define SET_PAD_GPIODOUT_SIF1CSN_ROUT(__val__)  (((__val__) << 25u) & 0x2000000u)
#define     PAD_GPIODOUT_RESERVED_31_26_SHIFT 26u
#define     PAD_GPIODOUT_RESERVED_31_26_MASK  0xfc000000u
#define GET_PAD_GPIODOUT_RESERVED_31_26(__reg__)  (((__reg__) & 0xfc000000) >> 26u)
#define SET_PAD_GPIODOUT_RESERVED_31_26(__val__)  (((__val__) << 26u) & 0xfc000000u)

/** @brief Register definition for @ref PAD_t.Gpiodout. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt32 gpio_dout:25;
        /** @brief  */
        UInt32 sif1csn_rout:1;
        /** @brief Reserved bits 31 to 26. Reads as 0. */
        UInt32 reserved_31_26:6;
    } bits;
} RegPADGpiodout_t;

#define REG_PAD_GPIOOUTD ((volatile UInt32*)0xf00258) /*  */
#define     PAD_GPIOOUTD_GPIO_CFG_OUT_SHIFT 0u
#define     PAD_GPIOOUTD_GPIO_CFG_OUT_MASK  0x1ffffffu
#define GET_PAD_GPIOOUTD_GPIO_CFG_OUT(__reg__)  (((__reg__) & 0x1ffffff) >> 0u)
#define SET_PAD_GPIOOUTD_GPIO_CFG_OUT(__val__)  (((__val__) << 0u) & 0x1ffffffu)
#define     PAD_GPIOOUTD_RESERVED_31_25_SHIFT 25u
#define     PAD_GPIOOUTD_RESERVED_31_25_MASK  0xfe000000u
#define GET_PAD_GPIOOUTD_RESERVED_31_25(__reg__)  (((__reg__) & 0xfe000000) >> 25u)
#define SET_PAD_GPIOOUTD_RESERVED_31_25(__val__)  (((__val__) << 25u) & 0xfe000000u)

/** @brief Register definition for @ref PAD_t.Gpiooutd. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt32 gpio_cfg_out:25;
        /** @brief Reserved bits 31 to 25. Reads as 0. */
        UInt32 reserved_31_25:7;
    } bits;
} RegPADGpiooutd_t;

#define REG_PAD_GPIOOUTZ ((volatile UInt32*)0xf0025c) /*  */
#define     PAD_GPIOOUTZ_GPIO_CFG_HIZ_SHIFT 0u
#define     PAD_GPIOOUTZ_GPIO_CFG_HIZ_MASK  0x1ffffffu
#define GET_PAD_GPIOOUTZ_GPIO_CFG_HIZ(__reg__)  (((__reg__) & 0x1ffffff) >> 0u)
#define SET_PAD_GPIOOUTZ_GPIO_CFG_HIZ(__val__)  (((__val__) << 0u) & 0x1ffffffu)
#define     PAD_GPIOOUTZ_RESERVED_31_25_SHIFT 25u
#define     PAD_GPIOOUTZ_RESERVED_31_25_MASK  0xfe000000u
#define GET_PAD_GPIOOUTZ_RESERVED_31_25(__reg__)  (((__reg__) & 0xfe000000) >> 25u)
#define SET_PAD_GPIOOUTZ_RESERVED_31_25(__val__)  (((__val__) << 25u) & 0xfe000000u)

/** @brief Register definition for @ref PAD_t.Gpiooutz. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt32 gpio_cfg_hiz:25;
        /** @brief Reserved bits 31 to 25. Reads as 0. */
        UInt32 reserved_31_25:7;
    } bits;
} RegPADGpiooutz_t;

#define REG_PAD_GPIOIN ((volatile UInt32*)0xf00260) /*  */
#define     PAD_GPIOIN_GPIO_CFG_IN_SHIFT 0u
#define     PAD_GPIOIN_GPIO_CFG_IN_MASK  0x1ffffffu
#define GET_PAD_GPIOIN_GPIO_CFG_IN(__reg__)  (((__reg__) & 0x1ffffff) >> 0u)
#define SET_PAD_GPIOIN_GPIO_CFG_IN(__val__)  (((__val__) << 0u) & 0x1ffffffu)
#define     PAD_GPIOIN_RESERVED_31_25_SHIFT 25u
#define     PAD_GPIOIN_RESERVED_31_25_MASK  0xfe000000u
#define GET_PAD_GPIOIN_RESERVED_31_25(__reg__)  (((__reg__) & 0xfe000000) >> 25u)
#define SET_PAD_GPIOIN_RESERVED_31_25(__val__)  (((__val__) << 25u) & 0xfe000000u)

/** @brief Register definition for @ref PAD_t.Gpioin. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt32 gpio_cfg_in:25;
        /** @brief Reserved bits 31 to 25. Reads as 0. */
        UInt32 reserved_31_25:7;
    } bits;
} RegPADGpioin_t;

#define REG_PAD_GPIOINOUT ((volatile UInt32*)0xf00264) /*  */
#define     PAD_GPIOINOUT_GPIO_CFG_IO_SHIFT 0u
#define     PAD_GPIOINOUT_GPIO_CFG_IO_MASK  0x1ffffffu
#define GET_PAD_GPIOINOUT_GPIO_CFG_IO(__reg__)  (((__reg__) & 0x1ffffff) >> 0u)
#define SET_PAD_GPIOINOUT_GPIO_CFG_IO(__val__)  (((__val__) << 0u) & 0x1ffffffu)
#define     PAD_GPIOINOUT_RESERVED_31_25_SHIFT 25u
#define     PAD_GPIOINOUT_RESERVED_31_25_MASK  0xfe000000u
#define GET_PAD_GPIOINOUT_RESERVED_31_25(__reg__)  (((__reg__) & 0xfe000000) >> 25u)
#define SET_PAD_GPIOINOUT_RESERVED_31_25(__val__)  (((__val__) << 25u) & 0xfe000000u)

/** @brief Register definition for @ref PAD_t.Gpioinout. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt32 gpio_cfg_io:25;
        /** @brief Reserved bits 31 to 25. Reads as 0. */
        UInt32 reserved_31_25:7;
    } bits;
} RegPADGpioinout_t;

#define REG_PAD_GPIOSETTING0 ((volatile UInt32*)0xf00268) /*  */
#define     PAD_GPIOSETTING0_MFPAD_IE_0_SHIFT 0u
#define     PAD_GPIOSETTING0_MFPAD_IE_0_MASK  0x1u
#define GET_PAD_GPIOSETTING0_MFPAD_IE_0(__reg__)  (((__reg__) & 0x1) >> 0u)
#define SET_PAD_GPIOSETTING0_MFPAD_IE_0(__val__)  (((__val__) << 0u) & 0x1u)
#define     PAD_GPIOSETTING0_MFPAD_OE_0_SHIFT 1u
#define     PAD_GPIOSETTING0_MFPAD_OE_0_MASK  0x2u
#define GET_PAD_GPIOSETTING0_MFPAD_OE_0(__reg__)  (((__reg__) & 0x2) >> 1u)
#define SET_PAD_GPIOSETTING0_MFPAD_OE_0(__val__)  (((__val__) << 1u) & 0x2u)
#define     PAD_GPIOSETTING0_MFPAD_IE_1_SHIFT 2u
#define     PAD_GPIOSETTING0_MFPAD_IE_1_MASK  0x4u
#define GET_PAD_GPIOSETTING0_MFPAD_IE_1(__reg__)  (((__reg__) & 0x4) >> 2u)
#define SET_PAD_GPIOSETTING0_MFPAD_IE_1(__val__)  (((__val__) << 2u) & 0x4u)
#define     PAD_GPIOSETTING0_MFPAD_OE_1_SHIFT 3u
#define     PAD_GPIOSETTING0_MFPAD_OE_1_MASK  0x8u
#define GET_PAD_GPIOSETTING0_MFPAD_OE_1(__reg__)  (((__reg__) & 0x8) >> 3u)
#define SET_PAD_GPIOSETTING0_MFPAD_OE_1(__val__)  (((__val__) << 3u) & 0x8u)
#define     PAD_GPIOSETTING0_MFPAD_IE_2_SHIFT 4u
#define     PAD_GPIOSETTING0_MFPAD_IE_2_MASK  0x10u
#define GET_PAD_GPIOSETTING0_MFPAD_IE_2(__reg__)  (((__reg__) & 0x10) >> 4u)
#define SET_PAD_GPIOSETTING0_MFPAD_IE_2(__val__)  (((__val__) << 4u) & 0x10u)
#define     PAD_GPIOSETTING0_MFPAD_OE_2_SHIFT 5u
#define     PAD_GPIOSETTING0_MFPAD_OE_2_MASK  0x20u
#define GET_PAD_GPIOSETTING0_MFPAD_OE_2(__reg__)  (((__reg__) & 0x20) >> 5u)
#define SET_PAD_GPIOSETTING0_MFPAD_OE_2(__val__)  (((__val__) << 5u) & 0x20u)
#define     PAD_GPIOSETTING0_MFPAD_IE_3_SHIFT 6u
#define     PAD_GPIOSETTING0_MFPAD_IE_3_MASK  0x40u
#define GET_PAD_GPIOSETTING0_MFPAD_IE_3(__reg__)  (((__reg__) & 0x40) >> 6u)
#define SET_PAD_GPIOSETTING0_MFPAD_IE_3(__val__)  (((__val__) << 6u) & 0x40u)
#define     PAD_GPIOSETTING0_MFPAD_OE_3_SHIFT 7u
#define     PAD_GPIOSETTING0_MFPAD_OE_3_MASK  0x80u
#define GET_PAD_GPIOSETTING0_MFPAD_OE_3(__reg__)  (((__reg__) & 0x80) >> 7u)
#define SET_PAD_GPIOSETTING0_MFPAD_OE_3(__val__)  (((__val__) << 7u) & 0x80u)
#define     PAD_GPIOSETTING0_MFPAD_IE_4_SHIFT 8u
#define     PAD_GPIOSETTING0_MFPAD_IE_4_MASK  0x100u
#define GET_PAD_GPIOSETTING0_MFPAD_IE_4(__reg__)  (((__reg__) & 0x100) >> 8u)
#define SET_PAD_GPIOSETTING0_MFPAD_IE_4(__val__)  (((__val__) << 8u) & 0x100u)
#define     PAD_GPIOSETTING0_MFPAD_OE_4_SHIFT 9u
#define     PAD_GPIOSETTING0_MFPAD_OE_4_MASK  0x200u
#define GET_PAD_GPIOSETTING0_MFPAD_OE_4(__reg__)  (((__reg__) & 0x200) >> 9u)
#define SET_PAD_GPIOSETTING0_MFPAD_OE_4(__val__)  (((__val__) << 9u) & 0x200u)
#define     PAD_GPIOSETTING0_MFPAD_IE_5_SHIFT 10u
#define     PAD_GPIOSETTING0_MFPAD_IE_5_MASK  0x400u
#define GET_PAD_GPIOSETTING0_MFPAD_IE_5(__reg__)  (((__reg__) & 0x400) >> 10u)
#define SET_PAD_GPIOSETTING0_MFPAD_IE_5(__val__)  (((__val__) << 10u) & 0x400u)
#define     PAD_GPIOSETTING0_MFPAD_OE_5_SHIFT 11u
#define     PAD_GPIOSETTING0_MFPAD_OE_5_MASK  0x800u
#define GET_PAD_GPIOSETTING0_MFPAD_OE_5(__reg__)  (((__reg__) & 0x800) >> 11u)
#define SET_PAD_GPIOSETTING0_MFPAD_OE_5(__val__)  (((__val__) << 11u) & 0x800u)
#define     PAD_GPIOSETTING0_MFPAD_IE_6_SHIFT 12u
#define     PAD_GPIOSETTING0_MFPAD_IE_6_MASK  0x1000u
#define GET_PAD_GPIOSETTING0_MFPAD_IE_6(__reg__)  (((__reg__) & 0x1000) >> 12u)
#define SET_PAD_GPIOSETTING0_MFPAD_IE_6(__val__)  (((__val__) << 12u) & 0x1000u)
#define     PAD_GPIOSETTING0_MFPAD_OE_6_SHIFT 13u
#define     PAD_GPIOSETTING0_MFPAD_OE_6_MASK  0x2000u
#define GET_PAD_GPIOSETTING0_MFPAD_OE_6(__reg__)  (((__reg__) & 0x2000) >> 13u)
#define SET_PAD_GPIOSETTING0_MFPAD_OE_6(__val__)  (((__val__) << 13u) & 0x2000u)
#define     PAD_GPIOSETTING0_MFPAD_IE_7_SHIFT 14u
#define     PAD_GPIOSETTING0_MFPAD_IE_7_MASK  0x4000u
#define GET_PAD_GPIOSETTING0_MFPAD_IE_7(__reg__)  (((__reg__) & 0x4000) >> 14u)
#define SET_PAD_GPIOSETTING0_MFPAD_IE_7(__val__)  (((__val__) << 14u) & 0x4000u)
#define     PAD_GPIOSETTING0_MFPAD_OE_7_SHIFT 15u
#define     PAD_GPIOSETTING0_MFPAD_OE_7_MASK  0x8000u
#define GET_PAD_GPIOSETTING0_MFPAD_OE_7(__reg__)  (((__reg__) & 0x8000) >> 15u)
#define SET_PAD_GPIOSETTING0_MFPAD_OE_7(__val__)  (((__val__) << 15u) & 0x8000u)
#define     PAD_GPIOSETTING0_MFPAD_IE_8_SHIFT 16u
#define     PAD_GPIOSETTING0_MFPAD_IE_8_MASK  0x10000u
#define GET_PAD_GPIOSETTING0_MFPAD_IE_8(__reg__)  (((__reg__) & 0x10000) >> 16u)
#define SET_PAD_GPIOSETTING0_MFPAD_IE_8(__val__)  (((__val__) << 16u) & 0x10000u)
#define     PAD_GPIOSETTING0_MFPAD_OE_8_SHIFT 17u
#define     PAD_GPIOSETTING0_MFPAD_OE_8_MASK  0x20000u
#define GET_PAD_GPIOSETTING0_MFPAD_OE_8(__reg__)  (((__reg__) & 0x20000) >> 17u)
#define SET_PAD_GPIOSETTING0_MFPAD_OE_8(__val__)  (((__val__) << 17u) & 0x20000u)
#define     PAD_GPIOSETTING0_MFPAD_IE_9_SHIFT 18u
#define     PAD_GPIOSETTING0_MFPAD_IE_9_MASK  0x40000u
#define GET_PAD_GPIOSETTING0_MFPAD_IE_9(__reg__)  (((__reg__) & 0x40000) >> 18u)
#define SET_PAD_GPIOSETTING0_MFPAD_IE_9(__val__)  (((__val__) << 18u) & 0x40000u)
#define     PAD_GPIOSETTING0_MFPAD_OE_9_SHIFT 19u
#define     PAD_GPIOSETTING0_MFPAD_OE_9_MASK  0x80000u
#define GET_PAD_GPIOSETTING0_MFPAD_OE_9(__reg__)  (((__reg__) & 0x80000) >> 19u)
#define SET_PAD_GPIOSETTING0_MFPAD_OE_9(__val__)  (((__val__) << 19u) & 0x80000u)
#define     PAD_GPIOSETTING0_MFPAD_IE_10_SHIFT 20u
#define     PAD_GPIOSETTING0_MFPAD_IE_10_MASK  0x100000u
#define GET_PAD_GPIOSETTING0_MFPAD_IE_10(__reg__)  (((__reg__) & 0x100000) >> 20u)
#define SET_PAD_GPIOSETTING0_MFPAD_IE_10(__val__)  (((__val__) << 20u) & 0x100000u)
#define     PAD_GPIOSETTING0_MFPAD_OE_10_SHIFT 21u
#define     PAD_GPIOSETTING0_MFPAD_OE_10_MASK  0x200000u
#define GET_PAD_GPIOSETTING0_MFPAD_OE_10(__reg__)  (((__reg__) & 0x200000) >> 21u)
#define SET_PAD_GPIOSETTING0_MFPAD_OE_10(__val__)  (((__val__) << 21u) & 0x200000u)
#define     PAD_GPIOSETTING0_MFPAD_IE_11_SHIFT 22u
#define     PAD_GPIOSETTING0_MFPAD_IE_11_MASK  0x400000u
#define GET_PAD_GPIOSETTING0_MFPAD_IE_11(__reg__)  (((__reg__) & 0x400000) >> 22u)
#define SET_PAD_GPIOSETTING0_MFPAD_IE_11(__val__)  (((__val__) << 22u) & 0x400000u)
#define     PAD_GPIOSETTING0_MFPAD_OE_11_SHIFT 23u
#define     PAD_GPIOSETTING0_MFPAD_OE_11_MASK  0x800000u
#define GET_PAD_GPIOSETTING0_MFPAD_OE_11(__reg__)  (((__reg__) & 0x800000) >> 23u)
#define SET_PAD_GPIOSETTING0_MFPAD_OE_11(__val__)  (((__val__) << 23u) & 0x800000u)
#define     PAD_GPIOSETTING0_MFPAD_IE_12_SHIFT 24u
#define     PAD_GPIOSETTING0_MFPAD_IE_12_MASK  0x1000000u
#define GET_PAD_GPIOSETTING0_MFPAD_IE_12(__reg__)  (((__reg__) & 0x1000000) >> 24u)
#define SET_PAD_GPIOSETTING0_MFPAD_IE_12(__val__)  (((__val__) << 24u) & 0x1000000u)
#define     PAD_GPIOSETTING0_MFPAD_OE_12_SHIFT 25u
#define     PAD_GPIOSETTING0_MFPAD_OE_12_MASK  0x2000000u
#define GET_PAD_GPIOSETTING0_MFPAD_OE_12(__reg__)  (((__reg__) & 0x2000000) >> 25u)
#define SET_PAD_GPIOSETTING0_MFPAD_OE_12(__val__)  (((__val__) << 25u) & 0x2000000u)
#define     PAD_GPIOSETTING0_MFPAD_IE_13_SHIFT 26u
#define     PAD_GPIOSETTING0_MFPAD_IE_13_MASK  0x4000000u
#define GET_PAD_GPIOSETTING0_MFPAD_IE_13(__reg__)  (((__reg__) & 0x4000000) >> 26u)
#define SET_PAD_GPIOSETTING0_MFPAD_IE_13(__val__)  (((__val__) << 26u) & 0x4000000u)
#define     PAD_GPIOSETTING0_MFPAD_OE_13_SHIFT 27u
#define     PAD_GPIOSETTING0_MFPAD_OE_13_MASK  0x8000000u
#define GET_PAD_GPIOSETTING0_MFPAD_OE_13(__reg__)  (((__reg__) & 0x8000000) >> 27u)
#define SET_PAD_GPIOSETTING0_MFPAD_OE_13(__val__)  (((__val__) << 27u) & 0x8000000u)
#define     PAD_GPIOSETTING0_MFPAD_IE_14_SHIFT 28u
#define     PAD_GPIOSETTING0_MFPAD_IE_14_MASK  0x10000000u
#define GET_PAD_GPIOSETTING0_MFPAD_IE_14(__reg__)  (((__reg__) & 0x10000000) >> 28u)
#define SET_PAD_GPIOSETTING0_MFPAD_IE_14(__val__)  (((__val__) << 28u) & 0x10000000u)
#define     PAD_GPIOSETTING0_MFPAD_OE_14_SHIFT 29u
#define     PAD_GPIOSETTING0_MFPAD_OE_14_MASK  0x20000000u
#define GET_PAD_GPIOSETTING0_MFPAD_OE_14(__reg__)  (((__reg__) & 0x20000000) >> 29u)
#define SET_PAD_GPIOSETTING0_MFPAD_OE_14(__val__)  (((__val__) << 29u) & 0x20000000u)
#define     PAD_GPIOSETTING0_MFPAD_IE_15_SHIFT 30u
#define     PAD_GPIOSETTING0_MFPAD_IE_15_MASK  0x40000000u
#define GET_PAD_GPIOSETTING0_MFPAD_IE_15(__reg__)  (((__reg__) & 0x40000000) >> 30u)
#define SET_PAD_GPIOSETTING0_MFPAD_IE_15(__val__)  (((__val__) << 30u) & 0x40000000u)
#define     PAD_GPIOSETTING0_MFPAD_OE_15_SHIFT 31u
#define     PAD_GPIOSETTING0_MFPAD_OE_15_MASK  0x80000000u
#define GET_PAD_GPIOSETTING0_MFPAD_OE_15(__reg__)  (((__reg__) & 0x80000000) >> 31u)
#define SET_PAD_GPIOSETTING0_MFPAD_OE_15(__val__)  (((__val__) << 31u) & 0x80000000u)

/** @brief Register definition for @ref PAD_t.Gpiosetting0. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt8 mfpad_ie_0:1;
        /** @brief  */
        UInt8 mfpad_oe_0:1;
        /** @brief  */
        UInt8 mfpad_ie_1:1;
        /** @brief  */
        UInt8 mfpad_oe_1:1;
        /** @brief  */
        UInt8 mfpad_ie_2:1;
        /** @brief  */
        UInt8 mfpad_oe_2:1;
        /** @brief  */
        UInt8 mfpad_ie_3:1;
        /** @brief  */
        UInt8 mfpad_oe_3:1;
        /** @brief  */
        UInt8 mfpad_ie_4:1;
        /** @brief  */
        UInt8 mfpad_oe_4:1;
        /** @brief  */
        UInt8 mfpad_ie_5:1;
        /** @brief  */
        UInt8 mfpad_oe_5:1;
        /** @brief  */
        UInt8 mfpad_ie_6:1;
        /** @brief  */
        UInt8 mfpad_oe_6:1;
        /** @brief  */
        UInt8 mfpad_ie_7:1;
        /** @brief  */
        UInt8 mfpad_oe_7:1;
        /** @brief  */
        UInt8 mfpad_ie_8:1;
        /** @brief  */
        UInt8 mfpad_oe_8:1;
        /** @brief  */
        UInt8 mfpad_ie_9:1;
        /** @brief  */
        UInt8 mfpad_oe_9:1;
        /** @brief  */
        UInt8 mfpad_ie_10:1;
        /** @brief  */
        UInt8 mfpad_oe_10:1;
        /** @brief  */
        UInt8 mfpad_ie_11:1;
        /** @brief  */
        UInt8 mfpad_oe_11:1;
        /** @brief  */
        UInt8 mfpad_ie_12:1;
        /** @brief  */
        UInt8 mfpad_oe_12:1;
        /** @brief  */
        UInt8 mfpad_ie_13:1;
        /** @brief  */
        UInt8 mfpad_oe_13:1;
        /** @brief  */
        UInt8 mfpad_ie_14:1;
        /** @brief  */
        UInt8 mfpad_oe_14:1;
        /** @brief  */
        UInt8 mfpad_ie_15:1;
        /** @brief  */
        UInt8 mfpad_oe_15:1;
    } bits;
} RegPADGpiosetting0_t;

#define REG_PAD_GPIOSETTING1 ((volatile UInt32*)0xf0026c) /*  */
#define     PAD_GPIOSETTING1_MFPAD_IE_16_SHIFT 0u
#define     PAD_GPIOSETTING1_MFPAD_IE_16_MASK  0x1u
#define GET_PAD_GPIOSETTING1_MFPAD_IE_16(__reg__)  (((__reg__) & 0x1) >> 0u)
#define SET_PAD_GPIOSETTING1_MFPAD_IE_16(__val__)  (((__val__) << 0u) & 0x1u)
#define     PAD_GPIOSETTING1_MFPAD_OE_16_SHIFT 1u
#define     PAD_GPIOSETTING1_MFPAD_OE_16_MASK  0x2u
#define GET_PAD_GPIOSETTING1_MFPAD_OE_16(__reg__)  (((__reg__) & 0x2) >> 1u)
#define SET_PAD_GPIOSETTING1_MFPAD_OE_16(__val__)  (((__val__) << 1u) & 0x2u)
#define     PAD_GPIOSETTING1_MFPAD_IE_17_SHIFT 2u
#define     PAD_GPIOSETTING1_MFPAD_IE_17_MASK  0x4u
#define GET_PAD_GPIOSETTING1_MFPAD_IE_17(__reg__)  (((__reg__) & 0x4) >> 2u)
#define SET_PAD_GPIOSETTING1_MFPAD_IE_17(__val__)  (((__val__) << 2u) & 0x4u)
#define     PAD_GPIOSETTING1_MFPAD_OE_17_SHIFT 3u
#define     PAD_GPIOSETTING1_MFPAD_OE_17_MASK  0x8u
#define GET_PAD_GPIOSETTING1_MFPAD_OE_17(__reg__)  (((__reg__) & 0x8) >> 3u)
#define SET_PAD_GPIOSETTING1_MFPAD_OE_17(__val__)  (((__val__) << 3u) & 0x8u)
#define     PAD_GPIOSETTING1_MFPAD_IE_18_SHIFT 4u
#define     PAD_GPIOSETTING1_MFPAD_IE_18_MASK  0x10u
#define GET_PAD_GPIOSETTING1_MFPAD_IE_18(__reg__)  (((__reg__) & 0x10) >> 4u)
#define SET_PAD_GPIOSETTING1_MFPAD_IE_18(__val__)  (((__val__) << 4u) & 0x10u)
#define     PAD_GPIOSETTING1_MFPAD_OE_18_SHIFT 5u
#define     PAD_GPIOSETTING1_MFPAD_OE_18_MASK  0x20u
#define GET_PAD_GPIOSETTING1_MFPAD_OE_18(__reg__)  (((__reg__) & 0x20) >> 5u)
#define SET_PAD_GPIOSETTING1_MFPAD_OE_18(__val__)  (((__val__) << 5u) & 0x20u)
#define     PAD_GPIOSETTING1_MFPAD_IE_19_SHIFT 6u
#define     PAD_GPIOSETTING1_MFPAD_IE_19_MASK  0x40u
#define GET_PAD_GPIOSETTING1_MFPAD_IE_19(__reg__)  (((__reg__) & 0x40) >> 6u)
#define SET_PAD_GPIOSETTING1_MFPAD_IE_19(__val__)  (((__val__) << 6u) & 0x40u)
#define     PAD_GPIOSETTING1_MFPAD_OE_19_SHIFT 7u
#define     PAD_GPIOSETTING1_MFPAD_OE_19_MASK  0x80u
#define GET_PAD_GPIOSETTING1_MFPAD_OE_19(__reg__)  (((__reg__) & 0x80) >> 7u)
#define SET_PAD_GPIOSETTING1_MFPAD_OE_19(__val__)  (((__val__) << 7u) & 0x80u)
#define     PAD_GPIOSETTING1_MFPAD_IE_20_SHIFT 8u
#define     PAD_GPIOSETTING1_MFPAD_IE_20_MASK  0x100u
#define GET_PAD_GPIOSETTING1_MFPAD_IE_20(__reg__)  (((__reg__) & 0x100) >> 8u)
#define SET_PAD_GPIOSETTING1_MFPAD_IE_20(__val__)  (((__val__) << 8u) & 0x100u)
#define     PAD_GPIOSETTING1_MFPAD_OE_20_SHIFT 9u
#define     PAD_GPIOSETTING1_MFPAD_OE_20_MASK  0x200u
#define GET_PAD_GPIOSETTING1_MFPAD_OE_20(__reg__)  (((__reg__) & 0x200) >> 9u)
#define SET_PAD_GPIOSETTING1_MFPAD_OE_20(__val__)  (((__val__) << 9u) & 0x200u)
#define     PAD_GPIOSETTING1_MFPAD_IE_21_SHIFT 10u
#define     PAD_GPIOSETTING1_MFPAD_IE_21_MASK  0x400u
#define GET_PAD_GPIOSETTING1_MFPAD_IE_21(__reg__)  (((__reg__) & 0x400) >> 10u)
#define SET_PAD_GPIOSETTING1_MFPAD_IE_21(__val__)  (((__val__) << 10u) & 0x400u)
#define     PAD_GPIOSETTING1_MFPAD_OE_21_SHIFT 11u
#define     PAD_GPIOSETTING1_MFPAD_OE_21_MASK  0x800u
#define GET_PAD_GPIOSETTING1_MFPAD_OE_21(__reg__)  (((__reg__) & 0x800) >> 11u)
#define SET_PAD_GPIOSETTING1_MFPAD_OE_21(__val__)  (((__val__) << 11u) & 0x800u)
#define     PAD_GPIOSETTING1_MFPAD_IE_22_SHIFT 12u
#define     PAD_GPIOSETTING1_MFPAD_IE_22_MASK  0x1000u
#define GET_PAD_GPIOSETTING1_MFPAD_IE_22(__reg__)  (((__reg__) & 0x1000) >> 12u)
#define SET_PAD_GPIOSETTING1_MFPAD_IE_22(__val__)  (((__val__) << 12u) & 0x1000u)
#define     PAD_GPIOSETTING1_MFPAD_OE_22_SHIFT 13u
#define     PAD_GPIOSETTING1_MFPAD_OE_22_MASK  0x2000u
#define GET_PAD_GPIOSETTING1_MFPAD_OE_22(__reg__)  (((__reg__) & 0x2000) >> 13u)
#define SET_PAD_GPIOSETTING1_MFPAD_OE_22(__val__)  (((__val__) << 13u) & 0x2000u)
#define     PAD_GPIOSETTING1_MFPAD_IE_23_SHIFT 14u
#define     PAD_GPIOSETTING1_MFPAD_IE_23_MASK  0x4000u
#define GET_PAD_GPIOSETTING1_MFPAD_IE_23(__reg__)  (((__reg__) & 0x4000) >> 14u)
#define SET_PAD_GPIOSETTING1_MFPAD_IE_23(__val__)  (((__val__) << 14u) & 0x4000u)
#define     PAD_GPIOSETTING1_MFPAD_OE_23_SHIFT 15u
#define     PAD_GPIOSETTING1_MFPAD_OE_23_MASK  0x8000u
#define GET_PAD_GPIOSETTING1_MFPAD_OE_23(__reg__)  (((__reg__) & 0x8000) >> 15u)
#define SET_PAD_GPIOSETTING1_MFPAD_OE_23(__val__)  (((__val__) << 15u) & 0x8000u)
#define     PAD_GPIOSETTING1_MFPAD_IE_24_SHIFT 16u
#define     PAD_GPIOSETTING1_MFPAD_IE_24_MASK  0x10000u
#define GET_PAD_GPIOSETTING1_MFPAD_IE_24(__reg__)  (((__reg__) & 0x10000) >> 16u)
#define SET_PAD_GPIOSETTING1_MFPAD_IE_24(__val__)  (((__val__) << 16u) & 0x10000u)
#define     PAD_GPIOSETTING1_MFPAD_OE_24_SHIFT 17u
#define     PAD_GPIOSETTING1_MFPAD_OE_24_MASK  0x20000u
#define GET_PAD_GPIOSETTING1_MFPAD_OE_24(__reg__)  (((__reg__) & 0x20000) >> 17u)
#define SET_PAD_GPIOSETTING1_MFPAD_OE_24(__val__)  (((__val__) << 17u) & 0x20000u)
#define     PAD_GPIOSETTING1_RESERVED_31_18_SHIFT 18u
#define     PAD_GPIOSETTING1_RESERVED_31_18_MASK  0xfffc0000u
#define GET_PAD_GPIOSETTING1_RESERVED_31_18(__reg__)  (((__reg__) & 0xfffc0000) >> 18u)
#define SET_PAD_GPIOSETTING1_RESERVED_31_18(__val__)  (((__val__) << 18u) & 0xfffc0000u)

/** @brief Register definition for @ref PAD_t.Gpiosetting1. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt8 mfpad_ie_16:1;
        /** @brief  */
        UInt8 mfpad_oe_16:1;
        /** @brief  */
        UInt8 mfpad_ie_17:1;
        /** @brief  */
        UInt8 mfpad_oe_17:1;
        /** @brief  */
        UInt8 mfpad_ie_18:1;
        /** @brief  */
        UInt8 mfpad_oe_18:1;
        /** @brief  */
        UInt8 mfpad_ie_19:1;
        /** @brief  */
        UInt8 mfpad_oe_19:1;
        /** @brief  */
        UInt8 mfpad_ie_20:1;
        /** @brief  */
        UInt8 mfpad_oe_20:1;
        /** @brief  */
        UInt8 mfpad_ie_21:1;
        /** @brief  */
        UInt8 mfpad_oe_21:1;
        /** @brief  */
        UInt8 mfpad_ie_22:1;
        /** @brief  */
        UInt8 mfpad_oe_22:1;
        /** @brief  */
        UInt8 mfpad_ie_23:1;
        /** @brief  */
        UInt8 mfpad_oe_23:1;
        /** @brief  */
        UInt16 mfpad_ie_24:1;
        /** @brief  */
        UInt16 mfpad_oe_24:1;
        /** @brief Reserved bits 31 to 18. Reads as 0. */
        UInt16 reserved_31_18:14;
    } bits;
} RegPADGpiosetting1_t;

#define REG_PAD_GPIOVALUE ((volatile UInt32*)0xf00270) /*  */
#define     PAD_GPIOVALUE_GPIO_VALUE_SHIFT 0u
#define     PAD_GPIOVALUE_GPIO_VALUE_MASK  0x1ffffffu
#define GET_PAD_GPIOVALUE_GPIO_VALUE(__reg__)  (((__reg__) & 0x1ffffff) >> 0u)
#define SET_PAD_GPIOVALUE_GPIO_VALUE(__val__)  (((__val__) << 0u) & 0x1ffffffu)
#define     PAD_GPIOVALUE_RESERVED_31_25_SHIFT 25u
#define     PAD_GPIOVALUE_RESERVED_31_25_MASK  0xfe000000u
#define GET_PAD_GPIOVALUE_RESERVED_31_25(__reg__)  (((__reg__) & 0xfe000000) >> 25u)
#define SET_PAD_GPIOVALUE_RESERVED_31_25(__val__)  (((__val__) << 25u) & 0xfe000000u)

/** @brief Register definition for @ref PAD_t.Gpiovalue. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt32 gpio_value:25;
        /** @brief Reserved bits 31 to 25. Reads as 0. */
        UInt32 reserved_31_25:7;
    } bits;
} RegPADGpiovalue_t;

#define REG_PAD_GPIOADDCONFIG ((volatile UInt32*)0xf00274) /*  */
#define     PAD_GPIOADDCONFIG_RESERVED_0_0_SHIFT 0u
#define     PAD_GPIOADDCONFIG_RESERVED_0_0_MASK  0x1u
#define GET_PAD_GPIOADDCONFIG_RESERVED_0_0(__reg__)  (((__reg__) & 0x1) >> 0u)
#define SET_PAD_GPIOADDCONFIG_RESERVED_0_0(__val__)  (((__val__) << 0u) & 0x1u)
#define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_SHIFT 1u
#define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_MASK  0xffeu
#define GET_PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG(__reg__)  (((__reg__) & 0xffe) >> 1u)
#define SET_PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG(__val__)  (((__val__) << 1u) & 0xffeu)
#define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_EVINT10_GPIO13 0x0u
#define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_EVINT8_GPIO11 0x0u
#define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_EVINT1_GPIO3 0x0u
#define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_EVINT2_GPIO4 0x0u
#define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_EVINT3_GPIO5 0x0u
#define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_EVINT4_GPIO6 0x0u
#define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_EVINT5_GPIO8 0x0u
#define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_EVINT6_GPIO9 0x0u
#define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_EVINT7_GPIO10 0x0u
#define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_EVINT9_GPIO12 0x0u
#define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_EVINT11_GPIO16 0x0u
#define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_EVINT1_GPIO1 0x1u
#define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_EVINT2_GPIO5 0x2u
#define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_EVINT3_GPIO4 0x4u
#define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_EVINT4_GPIO14 0x8u
#define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_EVINT5_GPIO9 0x10u
#define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_EVINT6_GPIO8 0x20u
#define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_EVINT7_GPIO15 0x40u
#define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_EVINT8_GPIO17 0x80u
#define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_EVINT9_GPIO18 0x100u
#define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_EVINT10_GPIO19 0x200u
#define     PAD_GPIOADDCONFIG_GPIO_EVIRQ_CFG_EVINT11_GPIO20 0x400u

#define     PAD_GPIOADDCONFIG_RESERVED_15_12_SHIFT 12u
#define     PAD_GPIOADDCONFIG_RESERVED_15_12_MASK  0xf000u
#define GET_PAD_GPIOADDCONFIG_RESERVED_15_12(__reg__)  (((__reg__) & 0xf000) >> 12u)
#define SET_PAD_GPIOADDCONFIG_RESERVED_15_12(__val__)  (((__val__) << 12u) & 0xf000u)
#define     PAD_GPIOADDCONFIG_GPIO_UTMR_CFG_SHIFT 16u
#define     PAD_GPIOADDCONFIG_GPIO_UTMR_CFG_MASK  0xf0000u
#define GET_PAD_GPIOADDCONFIG_GPIO_UTMR_CFG(__reg__)  (((__reg__) & 0xf0000) >> 16u)
#define SET_PAD_GPIOADDCONFIG_GPIO_UTMR_CFG(__val__)  (((__val__) << 16u) & 0xf0000u)
#define     PAD_GPIOADDCONFIG_RESERVED_23_20_SHIFT 20u
#define     PAD_GPIOADDCONFIG_RESERVED_23_20_MASK  0xf00000u
#define GET_PAD_GPIOADDCONFIG_RESERVED_23_20(__reg__)  (((__reg__) & 0xf00000) >> 20u)
#define SET_PAD_GPIOADDCONFIG_RESERVED_23_20(__val__)  (((__val__) << 20u) & 0xf00000u)
#define     PAD_GPIOADDCONFIG_PAD_SIF_CFG_SHIFT 24u
#define     PAD_GPIOADDCONFIG_PAD_SIF_CFG_MASK  0x3000000u
#define GET_PAD_GPIOADDCONFIG_PAD_SIF_CFG(__reg__)  (((__reg__) & 0x3000000) >> 24u)
#define SET_PAD_GPIOADDCONFIG_PAD_SIF_CFG(__val__)  (((__val__) << 24u) & 0x3000000u)
#define     PAD_GPIOADDCONFIG_RESERVED_31_26_SHIFT 26u
#define     PAD_GPIOADDCONFIG_RESERVED_31_26_MASK  0xfc000000u
#define GET_PAD_GPIOADDCONFIG_RESERVED_31_26(__reg__)  (((__reg__) & 0xfc000000) >> 26u)
#define SET_PAD_GPIOADDCONFIG_RESERVED_31_26(__val__)  (((__val__) << 26u) & 0xfc000000u)

/** @brief Register definition for @ref PAD_t.Gpioaddconfig. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief Reserved bit 0. Reads as 0. */
        UInt16 reserved_0_0:1;
        /** @brief Each bit determines whether the respective GPIO event interrupt gets its source per configuration 1 or configuration 2. */
        UInt16 gpio_evirq_cfg:11;
        /** @brief Reserved bits 15 to 12. Reads as 0. */
        UInt16 reserved_15_12:4;
        /** @brief This bit-field selects the configuration of the inputs / output for the universal timer according to. If a bit is 0, configuration 1 for the respective input/output is selected; otherwise configuration 2. The bit assignment is as follows: [0]: IN. [1]: STRT. [2]: CAPT. [3]: OUT */
        UInt8 gpio_utmr_cfg:4;
        /** @brief Reserved bits 23 to 20. Reads as 0. */
        UInt8 reserved_23_20:4;
        /** @brief This bit-field selects the protocol for SIF1 and SIF2. */
        UInt8 pad_sif_cfg:2;
        /** @brief Reserved bits 31 to 26. Reads as 0. */
        UInt8 reserved_31_26:6;
    } bits;
} RegPADGpioaddconfig_t;

#define REG_PAD_GPIOEVIRQPOLARITY ((volatile UInt32*)0xf00278) /*  */
#define     PAD_GPIOEVIRQPOLARITY_GPIO_EVIRQ_POL_SHIFT 0u
#define     PAD_GPIOEVIRQPOLARITY_GPIO_EVIRQ_POL_MASK  0xfffu
#define GET_PAD_GPIOEVIRQPOLARITY_GPIO_EVIRQ_POL(__reg__)  (((__reg__) & 0xfff) >> 0u)
#define SET_PAD_GPIOEVIRQPOLARITY_GPIO_EVIRQ_POL(__val__)  (((__val__) << 0u) & 0xfffu)
#define     PAD_GPIOEVIRQPOLARITY_RESERVED_31_12_SHIFT 12u
#define     PAD_GPIOEVIRQPOLARITY_RESERVED_31_12_MASK  0xfffff000u
#define GET_PAD_GPIOEVIRQPOLARITY_RESERVED_31_12(__reg__)  (((__reg__) & 0xfffff000) >> 12u)
#define SET_PAD_GPIOEVIRQPOLARITY_RESERVED_31_12(__val__)  (((__val__) << 12u) & 0xfffff000u)

/** @brief Register definition for @ref PAD_t.Gpioevirqpolarity. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief Each bit defines the active edge for the respective GPIO event interrupt. 0x0: negative edge constitutes an interrupt event. 0x1: positive edge constitutes an interrupt event. */
        UInt32 gpio_evirq_pol:12;
        /** @brief Reserved bits 31 to 12. Reads as 0. */
        UInt32 reserved_31_12:20;
    } bits;
} RegPADGpioevirqpolarity_t;

#define REG_PAD_GPIOEVIRQTIME0 ((volatile UInt32*)0xf0027c) /*  */
#define     PAD_GPIOEVIRQTIME0_GPIO_EVIRQ_TIME0_SHIFT 0u
#define     PAD_GPIOEVIRQTIME0_GPIO_EVIRQ_TIME0_MASK  0xffffffffu
#define GET_PAD_GPIOEVIRQTIME0_GPIO_EVIRQ_TIME0(__reg__)  (((__reg__) & 0xffffffff) >> 0u)
#define SET_PAD_GPIOEVIRQTIME0_GPIO_EVIRQ_TIME0(__val__)  (((__val__) << 0u) & 0xffffffffu)

/** @brief Register definition for @ref PAD_t.Gpioevirqtime0. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt32 gpio_evirq_time0;
    } bits;
} RegPADGpioevirqtime0_t;

#define REG_PAD_GPIOEVIRQTIME1 ((volatile UInt32*)0xf00280) /*  */
#define     PAD_GPIOEVIRQTIME1_GPIO_EVIRQ_TIME1_SHIFT 0u
#define     PAD_GPIOEVIRQTIME1_GPIO_EVIRQ_TIME1_MASK  0xffffffffu
#define GET_PAD_GPIOEVIRQTIME1_GPIO_EVIRQ_TIME1(__reg__)  (((__reg__) & 0xffffffff) >> 0u)
#define SET_PAD_GPIOEVIRQTIME1_GPIO_EVIRQ_TIME1(__val__)  (((__val__) << 0u) & 0xffffffffu)

/** @brief Register definition for @ref PAD_t.Gpioevirqtime1. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt32 gpio_evirq_time1;
    } bits;
} RegPADGpioevirqtime1_t;

#define REG_PAD_GPIOEVIRQTIME2 ((volatile UInt32*)0xf00284) /*  */
#define     PAD_GPIOEVIRQTIME2_GPIO_EVIRQ_TIME2_SHIFT 0u
#define     PAD_GPIOEVIRQTIME2_GPIO_EVIRQ_TIME2_MASK  0xffffffffu
#define GET_PAD_GPIOEVIRQTIME2_GPIO_EVIRQ_TIME2(__reg__)  (((__reg__) & 0xffffffff) >> 0u)
#define SET_PAD_GPIOEVIRQTIME2_GPIO_EVIRQ_TIME2(__val__)  (((__val__) << 0u) & 0xffffffffu)

/** @brief Register definition for @ref PAD_t.Gpioevirqtime2. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt32 gpio_evirq_time2;
    } bits;
} RegPADGpioevirqtime2_t;

#define REG_PAD_GPIOEVIRQTIME3 ((volatile UInt32*)0xf00288) /*  */
#define     PAD_GPIOEVIRQTIME3_GPIO_EVIRQ_TIME3_SHIFT 0u
#define     PAD_GPIOEVIRQTIME3_GPIO_EVIRQ_TIME3_MASK  0xffffffffu
#define GET_PAD_GPIOEVIRQTIME3_GPIO_EVIRQ_TIME3(__reg__)  (((__reg__) & 0xffffffff) >> 0u)
#define SET_PAD_GPIOEVIRQTIME3_GPIO_EVIRQ_TIME3(__val__)  (((__val__) << 0u) & 0xffffffffu)

/** @brief Register definition for @ref PAD_t.Gpioevirqtime3. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt32 gpio_evirq_time3;
    } bits;
} RegPADGpioevirqtime3_t;

#define REG_PAD_GPIOEVIRQTIME4 ((volatile UInt32*)0xf0028c) /*  */
#define     PAD_GPIOEVIRQTIME4_GPIO_EVIRQ_TIME4_SHIFT 0u
#define     PAD_GPIOEVIRQTIME4_GPIO_EVIRQ_TIME4_MASK  0xffffffffu
#define GET_PAD_GPIOEVIRQTIME4_GPIO_EVIRQ_TIME4(__reg__)  (((__reg__) & 0xffffffff) >> 0u)
#define SET_PAD_GPIOEVIRQTIME4_GPIO_EVIRQ_TIME4(__val__)  (((__val__) << 0u) & 0xffffffffu)

/** @brief Register definition for @ref PAD_t.Gpioevirqtime4. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt32 gpio_evirq_time4;
    } bits;
} RegPADGpioevirqtime4_t;

#define REG_PAD_GPIOEVIRQTIME5 ((volatile UInt32*)0xf00290) /*  */
#define     PAD_GPIOEVIRQTIME5_GPIO_EVIRQ_TIME5_SHIFT 0u
#define     PAD_GPIOEVIRQTIME5_GPIO_EVIRQ_TIME5_MASK  0xffffffffu
#define GET_PAD_GPIOEVIRQTIME5_GPIO_EVIRQ_TIME5(__reg__)  (((__reg__) & 0xffffffff) >> 0u)
#define SET_PAD_GPIOEVIRQTIME5_GPIO_EVIRQ_TIME5(__val__)  (((__val__) << 0u) & 0xffffffffu)

/** @brief Register definition for @ref PAD_t.Gpioevirqtime5. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt32 gpio_evirq_time5;
    } bits;
} RegPADGpioevirqtime5_t;

#define REG_PAD_GPIOEVIRQTIME6 ((volatile UInt32*)0xf00294) /*  */
#define     PAD_GPIOEVIRQTIME6_GPIO_EVIRQ_TIME6_SHIFT 0u
#define     PAD_GPIOEVIRQTIME6_GPIO_EVIRQ_TIME6_MASK  0xffffffffu
#define GET_PAD_GPIOEVIRQTIME6_GPIO_EVIRQ_TIME6(__reg__)  (((__reg__) & 0xffffffff) >> 0u)
#define SET_PAD_GPIOEVIRQTIME6_GPIO_EVIRQ_TIME6(__val__)  (((__val__) << 0u) & 0xffffffffu)

/** @brief Register definition for @ref PAD_t.Gpioevirqtime6. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt32 gpio_evirq_time6;
    } bits;
} RegPADGpioevirqtime6_t;

#define REG_PAD_GPIOEVIRQTIME7 ((volatile UInt32*)0xf00298) /*  */
#define     PAD_GPIOEVIRQTIME7_GPIO_EVIRQ_TIME7_SHIFT 0u
#define     PAD_GPIOEVIRQTIME7_GPIO_EVIRQ_TIME7_MASK  0xffffffffu
#define GET_PAD_GPIOEVIRQTIME7_GPIO_EVIRQ_TIME7(__reg__)  (((__reg__) & 0xffffffff) >> 0u)
#define SET_PAD_GPIOEVIRQTIME7_GPIO_EVIRQ_TIME7(__val__)  (((__val__) << 0u) & 0xffffffffu)

/** @brief Register definition for @ref PAD_t.Gpioevirqtime7. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt32 gpio_evirq_time7;
    } bits;
} RegPADGpioevirqtime7_t;

#define REG_PAD_GPIOEVIRQTIME8 ((volatile UInt32*)0xf0029c) /*  */
#define     PAD_GPIOEVIRQTIME8_GPIO_EVIRQ_TIME8_SHIFT 0u
#define     PAD_GPIOEVIRQTIME8_GPIO_EVIRQ_TIME8_MASK  0xffffffffu
#define GET_PAD_GPIOEVIRQTIME8_GPIO_EVIRQ_TIME8(__reg__)  (((__reg__) & 0xffffffff) >> 0u)
#define SET_PAD_GPIOEVIRQTIME8_GPIO_EVIRQ_TIME8(__val__)  (((__val__) << 0u) & 0xffffffffu)

/** @brief Register definition for @ref PAD_t.Gpioevirqtime8. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt32 gpio_evirq_time8;
    } bits;
} RegPADGpioevirqtime8_t;

#define REG_PAD_GPIOEVIRQTIME9 ((volatile UInt32*)0xf002a0) /*  */
#define     PAD_GPIOEVIRQTIME9_GPIO_EVIRQ_TIME9_SHIFT 0u
#define     PAD_GPIOEVIRQTIME9_GPIO_EVIRQ_TIME9_MASK  0xffffffffu
#define GET_PAD_GPIOEVIRQTIME9_GPIO_EVIRQ_TIME9(__reg__)  (((__reg__) & 0xffffffff) >> 0u)
#define SET_PAD_GPIOEVIRQTIME9_GPIO_EVIRQ_TIME9(__val__)  (((__val__) << 0u) & 0xffffffffu)

/** @brief Register definition for @ref PAD_t.Gpioevirqtime9. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt32 gpio_evirq_time9;
    } bits;
} RegPADGpioevirqtime9_t;

#define REG_PAD_GPIOEVIRQTIME10 ((volatile UInt32*)0xf002a4) /*  */
#define     PAD_GPIOEVIRQTIME10_GPIO_EVIRQ_TIME10_SHIFT 0u
#define     PAD_GPIOEVIRQTIME10_GPIO_EVIRQ_TIME10_MASK  0xffffffffu
#define GET_PAD_GPIOEVIRQTIME10_GPIO_EVIRQ_TIME10(__reg__)  (((__reg__) & 0xffffffff) >> 0u)
#define SET_PAD_GPIOEVIRQTIME10_GPIO_EVIRQ_TIME10(__val__)  (((__val__) << 0u) & 0xffffffffu)

/** @brief Register definition for @ref PAD_t.Gpioevirqtime10. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt32 gpio_evirq_time10;
    } bits;
} RegPADGpioevirqtime10_t;

#define REG_PAD_GPIOEVIRQTIME11 ((volatile UInt32*)0xf002a8) /*  */
#define     PAD_GPIOEVIRQTIME11_GPIO_EVIRQ_TIME11_SHIFT 0u
#define     PAD_GPIOEVIRQTIME11_GPIO_EVIRQ_TIME11_MASK  0xffffffffu
#define GET_PAD_GPIOEVIRQTIME11_GPIO_EVIRQ_TIME11(__reg__)  (((__reg__) & 0xffffffff) >> 0u)
#define SET_PAD_GPIOEVIRQTIME11_GPIO_EVIRQ_TIME11(__val__)  (((__val__) << 0u) & 0xffffffffu)

/** @brief Register definition for @ref PAD_t.Gpioevirqtime11. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt32 gpio_evirq_time11;
    } bits;
} RegPADGpioevirqtime11_t;

#define REG_PAD_HOSTEVIRQTIME ((volatile UInt32*)0xf002ac) /*  */
#define     PAD_HOSTEVIRQTIME_HOST_EVIRQ_TIME_SHIFT 0u
#define     PAD_HOSTEVIRQTIME_HOST_EVIRQ_TIME_MASK  0xffffffffu
#define GET_PAD_HOSTEVIRQTIME_HOST_EVIRQ_TIME(__reg__)  (((__reg__) & 0xffffffff) >> 0u)
#define SET_PAD_HOSTEVIRQTIME_HOST_EVIRQ_TIME(__val__)  (((__val__) << 0u) & 0xffffffffu)

/** @brief Register definition for @ref PAD_t.Hostevirqtime. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt32 host_evirq_time;
    } bits;
} RegPADHostevirqtime_t;

/** @brief Component definition for @ref PAD. */
typedef struct {
    /** @brief  */
    RegPADControl0_t Control0;

    /** @brief  */
    RegPADControl1_t Control1;

    /** @brief  */
    RegPADControl2_t Control2;

    /** @brief  */
    RegPADDrivecontrol_t Drivecontrol;

    /** @brief  */
    RegPADPullenable_t Pullenable;

    /** @brief  */
    RegPADGpiodout_t Gpiodout;

    /** @brief  */
    RegPADGpiooutd_t Gpiooutd;

    /** @brief  */
    RegPADGpiooutz_t Gpiooutz;

    /** @brief  */
    RegPADGpioin_t Gpioin;

    /** @brief  */
    RegPADGpioinout_t Gpioinout;

    /** @brief  */
    RegPADGpiosetting0_t Gpiosetting0;

    /** @brief  */
    RegPADGpiosetting1_t Gpiosetting1;

    /** @brief  */
    RegPADGpiovalue_t Gpiovalue;

    /** @brief  */
    RegPADGpioaddconfig_t Gpioaddconfig;

    /** @brief  */
    RegPADGpioevirqpolarity_t Gpioevirqpolarity;

    /** @brief  */
    RegPADGpioevirqtime0_t Gpioevirqtime0;

    /** @brief  */
    RegPADGpioevirqtime1_t Gpioevirqtime1;

    /** @brief  */
    RegPADGpioevirqtime2_t Gpioevirqtime2;

    /** @brief  */
    RegPADGpioevirqtime3_t Gpioevirqtime3;

    /** @brief  */
    RegPADGpioevirqtime4_t Gpioevirqtime4;

    /** @brief  */
    RegPADGpioevirqtime5_t Gpioevirqtime5;

    /** @brief  */
    RegPADGpioevirqtime6_t Gpioevirqtime6;

    /** @brief  */
    RegPADGpioevirqtime7_t Gpioevirqtime7;

    /** @brief  */
    RegPADGpioevirqtime8_t Gpioevirqtime8;

    /** @brief  */
    RegPADGpioevirqtime9_t Gpioevirqtime9;

    /** @brief  */
    RegPADGpioevirqtime10_t Gpioevirqtime10;

    /** @brief  */
    RegPADGpioevirqtime11_t Gpioevirqtime11;

    /** @brief  */
    RegPADHostevirqtime_t Hostevirqtime;

} PAD_t;

/** @brief  */
extern volatile PAD_t PAD;


#define REG_PIRQ_IRQENSET ((volatile UInt32*)0xf00120) /*  */
#define     PIRQ_IRQENSET_IRQ_EN_SET_SHIFT 0u
#define     PIRQ_IRQENSET_IRQ_EN_SET_MASK  0x7ffffffu
#define GET_PIRQ_IRQENSET_IRQ_EN_SET(__reg__)  (((__reg__) & 0x7ffffff) >> 0u)
#define SET_PIRQ_IRQENSET_IRQ_EN_SET(__val__)  (((__val__) << 0u) & 0x7ffffffu)
#define     PIRQ_IRQENSET_RESERVED_31_27_SHIFT 27u
#define     PIRQ_IRQENSET_RESERVED_31_27_MASK  0xf8000000u
#define GET_PIRQ_IRQENSET_RESERVED_31_27(__reg__)  (((__reg__) & 0xf8000000) >> 27u)
#define SET_PIRQ_IRQENSET_RESERVED_31_27(__val__)  (((__val__) << 27u) & 0xf8000000u)

/** @brief Register definition for @ref PIRQ_t.Irqenset. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt32 irq_en_set:27;
        /** @brief Reserved bits 31 to 27. Reads as 0. */
        UInt32 reserved_31_27:5;
    } bits;
} RegPIRQIrqenset_t;

#define REG_PIRQ_IRQENCLR ((volatile UInt32*)0xf00124) /*  */
#define     PIRQ_IRQENCLR_IRQ_EN_CLR_SHIFT 0u
#define     PIRQ_IRQENCLR_IRQ_EN_CLR_MASK  0x7ffffffu
#define GET_PIRQ_IRQENCLR_IRQ_EN_CLR(__reg__)  (((__reg__) & 0x7ffffff) >> 0u)
#define SET_PIRQ_IRQENCLR_IRQ_EN_CLR(__val__)  (((__val__) << 0u) & 0x7ffffffu)
#define     PIRQ_IRQENCLR_RESERVED_31_27_SHIFT 27u
#define     PIRQ_IRQENCLR_RESERVED_31_27_MASK  0xf8000000u
#define GET_PIRQ_IRQENCLR_RESERVED_31_27(__reg__)  (((__reg__) & 0xf8000000) >> 27u)
#define SET_PIRQ_IRQENCLR_RESERVED_31_27(__val__)  (((__val__) << 27u) & 0xf8000000u)

/** @brief Register definition for @ref PIRQ_t.Irqenclr. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt32 irq_en_clr:27;
        /** @brief Reserved bits 31 to 27. Reads as 0. */
        UInt32 reserved_31_27:5;
    } bits;
} RegPIRQIrqenclr_t;

#define REG_PIRQ_IRQENVALUE ((volatile UInt32*)0xf00128) /*  */
#define     PIRQ_IRQENVALUE_IRQ_EN_VAL_SHIFT 0u
#define     PIRQ_IRQENVALUE_IRQ_EN_VAL_MASK  0x7ffffffu
#define GET_PIRQ_IRQENVALUE_IRQ_EN_VAL(__reg__)  (((__reg__) & 0x7ffffff) >> 0u)
#define SET_PIRQ_IRQENVALUE_IRQ_EN_VAL(__val__)  (((__val__) << 0u) & 0x7ffffffu)
#define     PIRQ_IRQENVALUE_RESERVED_31_27_SHIFT 27u
#define     PIRQ_IRQENVALUE_RESERVED_31_27_MASK  0xf8000000u
#define GET_PIRQ_IRQENVALUE_RESERVED_31_27(__reg__)  (((__reg__) & 0xf8000000) >> 27u)
#define SET_PIRQ_IRQENVALUE_RESERVED_31_27(__val__)  (((__val__) << 27u) & 0xf8000000u)

/** @brief Register definition for @ref PIRQ_t.Irqenvalue. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief A bit in this register reflects the current value of the respective interrupt enable. */
        UInt32 irq_en_val:27;
        /** @brief Reserved bits 31 to 27. Reads as 0. */
        UInt32 reserved_31_27:5;
    } bits;
} RegPIRQIrqenvalue_t;

#define REG_PIRQ_STATUSCLR ((volatile UInt32*)0xf0012c) /*  */
#define     PIRQ_STATUSCLR_IRQ_STAT_CLR_SHIFT 0u
#define     PIRQ_STATUSCLR_IRQ_STAT_CLR_MASK  0x7ffffffu
#define GET_PIRQ_STATUSCLR_IRQ_STAT_CLR(__reg__)  (((__reg__) & 0x7ffffff) >> 0u)
#define SET_PIRQ_STATUSCLR_IRQ_STAT_CLR(__val__)  (((__val__) << 0u) & 0x7ffffffu)
#define     PIRQ_STATUSCLR_RESERVED_31_27_SHIFT 27u
#define     PIRQ_STATUSCLR_RESERVED_31_27_MASK  0xf8000000u
#define GET_PIRQ_STATUSCLR_RESERVED_31_27(__reg__)  (((__reg__) & 0xf8000000) >> 27u)
#define SET_PIRQ_STATUSCLR_RESERVED_31_27(__val__)  (((__val__) << 27u) & 0xf8000000u)

/** @brief Register definition for @ref PIRQ_t.Statusclr. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt32 irq_stat_clr:27;
        /** @brief Reserved bits 31 to 27. Reads as 0. */
        UInt32 reserved_31_27:5;
    } bits;
} RegPIRQStatusclr_t;

#define REG_PIRQ_STATUSVALUE ((volatile UInt32*)0xf00130) /*  */
#define     PIRQ_STATUSVALUE_IRQ_STAT_VAL_SHIFT 0u
#define     PIRQ_STATUSVALUE_IRQ_STAT_VAL_MASK  0x7ffffffu
#define GET_PIRQ_STATUSVALUE_IRQ_STAT_VAL(__reg__)  (((__reg__) & 0x7ffffff) >> 0u)
#define SET_PIRQ_STATUSVALUE_IRQ_STAT_VAL(__val__)  (((__val__) << 0u) & 0x7ffffffu)
#define     PIRQ_STATUSVALUE_RESERVED_31_27_SHIFT 27u
#define     PIRQ_STATUSVALUE_RESERVED_31_27_MASK  0xf8000000u
#define GET_PIRQ_STATUSVALUE_RESERVED_31_27(__reg__)  (((__reg__) & 0xf8000000) >> 27u)
#define SET_PIRQ_STATUSVALUE_RESERVED_31_27(__val__)  (((__val__) << 27u) & 0xf8000000u)

/** @brief Register definition for @ref PIRQ_t.Statusvalue. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief A bit in this register reflects the current value of the respective interrupt status bit. */
        UInt32 irq_stat_val:27;
        /** @brief Reserved bits 31 to 27. Reads as 0. */
        UInt32 reserved_31_27:5;
    } bits;
} RegPIRQStatusvalue_t;

#define REG_PIRQ_IRQMASKSET ((volatile UInt32*)0xf00134) /*  */
#define     PIRQ_IRQMASKSET_IRQ_MASK_SET_SHIFT 0u
#define     PIRQ_IRQMASKSET_IRQ_MASK_SET_MASK  0x1ffffffu
#define GET_PIRQ_IRQMASKSET_IRQ_MASK_SET(__reg__)  (((__reg__) & 0x1ffffff) >> 0u)
#define SET_PIRQ_IRQMASKSET_IRQ_MASK_SET(__val__)  (((__val__) << 0u) & 0x1ffffffu)
#define     PIRQ_IRQMASKSET_RESERVED_31_25_SHIFT 25u
#define     PIRQ_IRQMASKSET_RESERVED_31_25_MASK  0xfe000000u
#define GET_PIRQ_IRQMASKSET_RESERVED_31_25(__reg__)  (((__reg__) & 0xfe000000) >> 25u)
#define SET_PIRQ_IRQMASKSET_RESERVED_31_25(__val__)  (((__val__) << 25u) & 0xfe000000u)

/** @brief Register definition for @ref PIRQ_t.Irqmaskset. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt32 irq_mask_set:25;
        /** @brief Reserved bits 31 to 25. Reads as 0. */
        UInt32 reserved_31_25:7;
    } bits;
} RegPIRQIrqmaskset_t;

#define REG_PIRQ_IRQMASKCLR ((volatile UInt32*)0xf00138) /*  */
#define     PIRQ_IRQMASKCLR_IRQ_MASK_CLR_SHIFT 0u
#define     PIRQ_IRQMASKCLR_IRQ_MASK_CLR_MASK  0x1ffffffu
#define GET_PIRQ_IRQMASKCLR_IRQ_MASK_CLR(__reg__)  (((__reg__) & 0x1ffffff) >> 0u)
#define SET_PIRQ_IRQMASKCLR_IRQ_MASK_CLR(__val__)  (((__val__) << 0u) & 0x1ffffffu)
#define     PIRQ_IRQMASKCLR_RESERVED_31_25_SHIFT 25u
#define     PIRQ_IRQMASKCLR_RESERVED_31_25_MASK  0xfe000000u
#define GET_PIRQ_IRQMASKCLR_RESERVED_31_25(__reg__)  (((__reg__) & 0xfe000000) >> 25u)
#define SET_PIRQ_IRQMASKCLR_RESERVED_31_25(__val__)  (((__val__) << 25u) & 0xfe000000u)

/** @brief Register definition for @ref PIRQ_t.Irqmaskclr. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt32 irq_mask_clr:25;
        /** @brief Reserved bits 31 to 25. Reads as 0. */
        UInt32 reserved_31_25:7;
    } bits;
} RegPIRQIrqmaskclr_t;

#define REG_PIRQ_IRQMASKVALUE ((volatile UInt32*)0xf0013c) /*  */
#define     PIRQ_IRQMASKVALUE_IRQ_MASK_VAL_SHIFT 0u
#define     PIRQ_IRQMASKVALUE_IRQ_MASK_VAL_MASK  0x1ffffffu
#define GET_PIRQ_IRQMASKVALUE_IRQ_MASK_VAL(__reg__)  (((__reg__) & 0x1ffffff) >> 0u)
#define SET_PIRQ_IRQMASKVALUE_IRQ_MASK_VAL(__val__)  (((__val__) << 0u) & 0x1ffffffu)
#define     PIRQ_IRQMASKVALUE_RESERVED_31_25_SHIFT 25u
#define     PIRQ_IRQMASKVALUE_RESERVED_31_25_MASK  0xfe000000u
#define GET_PIRQ_IRQMASKVALUE_RESERVED_31_25(__reg__)  (((__reg__) & 0xfe000000) >> 25u)
#define SET_PIRQ_IRQMASKVALUE_RESERVED_31_25(__val__)  (((__val__) << 25u) & 0xfe000000u)

/** @brief Register definition for @ref PIRQ_t.Irqmaskvalue. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief A bit in this register reflects the current value of the respective interrupt mask. */
        UInt32 irq_mask_val:25;
        /** @brief Reserved bits 31 to 25. Reads as 0. */
        UInt32 reserved_31_25:7;
    } bits;
} RegPIRQIrqmaskvalue_t;

#define REG_PIRQ_SWIRQSET ((volatile UInt32*)0xf00140) /*  */
#define     PIRQ_SWIRQSET_SW_INT_SET_SHIFT 0u
#define     PIRQ_SWIRQSET_SW_INT_SET_MASK  0xfu
#define GET_PIRQ_SWIRQSET_SW_INT_SET(__reg__)  (((__reg__) & 0xf) >> 0u)
#define SET_PIRQ_SWIRQSET_SW_INT_SET(__val__)  (((__val__) << 0u) & 0xfu)
#define     PIRQ_SWIRQSET_RESERVED_31_4_SHIFT 4u
#define     PIRQ_SWIRQSET_RESERVED_31_4_MASK  0xfffffff0u
#define GET_PIRQ_SWIRQSET_RESERVED_31_4(__reg__)  (((__reg__) & 0xfffffff0) >> 4u)
#define SET_PIRQ_SWIRQSET_RESERVED_31_4(__val__)  (((__val__) << 4u) & 0xfffffff0u)

/** @brief Register definition for @ref PIRQ_t.Swirqset. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt32 sw_int_set:4;
        /** @brief Reserved bits 31 to 4. Reads as 0. */
        UInt32 reserved_31_4:28;
    } bits;
} RegPIRQSwirqset_t;

#define REG_PIRQ_SWIRQCLR ((volatile UInt32*)0xf00144) /*  */
#define     PIRQ_SWIRQCLR_SW_INT_CLR_SHIFT 0u
#define     PIRQ_SWIRQCLR_SW_INT_CLR_MASK  0xfu
#define GET_PIRQ_SWIRQCLR_SW_INT_CLR(__reg__)  (((__reg__) & 0xf) >> 0u)
#define SET_PIRQ_SWIRQCLR_SW_INT_CLR(__val__)  (((__val__) << 0u) & 0xfu)
#define     PIRQ_SWIRQCLR_RESERVED_31_4_SHIFT 4u
#define     PIRQ_SWIRQCLR_RESERVED_31_4_MASK  0xfffffff0u
#define GET_PIRQ_SWIRQCLR_RESERVED_31_4(__reg__)  (((__reg__) & 0xfffffff0) >> 4u)
#define SET_PIRQ_SWIRQCLR_RESERVED_31_4(__val__)  (((__val__) << 4u) & 0xfffffff0u)

/** @brief Register definition for @ref PIRQ_t.Swirqclr. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt32 sw_int_clr:4;
        /** @brief Reserved bits 31 to 4. Reads as 0. */
        UInt32 reserved_31_4:28;
    } bits;
} RegPIRQSwirqclr_t;

#define REG_PIRQ_SWIRQVALUE ((volatile UInt32*)0xf00148) /*  */
#define     PIRQ_SWIRQVALUE_SW_INT_SHIFT 0u
#define     PIRQ_SWIRQVALUE_SW_INT_MASK  0xfu
#define GET_PIRQ_SWIRQVALUE_SW_INT(__reg__)  (((__reg__) & 0xf) >> 0u)
#define SET_PIRQ_SWIRQVALUE_SW_INT(__val__)  (((__val__) << 0u) & 0xfu)
#define     PIRQ_SWIRQVALUE_RESERVED_31_4_SHIFT 4u
#define     PIRQ_SWIRQVALUE_RESERVED_31_4_MASK  0xfffffff0u
#define GET_PIRQ_SWIRQVALUE_RESERVED_31_4(__reg__)  (((__reg__) & 0xfffffff0) >> 4u)
#define SET_PIRQ_SWIRQVALUE_RESERVED_31_4(__val__)  (((__val__) << 4u) & 0xfffffff0u)

/** @brief Register definition for @ref PIRQ_t.Swirqvalue. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief A bit in this register reflects the current status of the respective software interrupt. */
        UInt32 sw_int:4;
        /** @brief Reserved bits 31 to 4. Reads as 0. */
        UInt32 reserved_31_4:28;
    } bits;
} RegPIRQSwirqvalue_t;

#define REG_PIRQ_HRWIRQENSET ((volatile UInt32*)0xf0014c) /*  */
#define     PIRQ_HRWIRQENSET_HRD_IRQ_EN_SET_SHIFT 0u
#define     PIRQ_HRWIRQENSET_HRD_IRQ_EN_SET_MASK  0xffffu
#define GET_PIRQ_HRWIRQENSET_HRD_IRQ_EN_SET(__reg__)  (((__reg__) & 0xffff) >> 0u)
#define SET_PIRQ_HRWIRQENSET_HRD_IRQ_EN_SET(__val__)  (((__val__) << 0u) & 0xffffu)
#define     PIRQ_HRWIRQENSET_HWR_IRQ_EN_SET_SHIFT 16u
#define     PIRQ_HRWIRQENSET_HWR_IRQ_EN_SET_MASK  0xffff0000u
#define GET_PIRQ_HRWIRQENSET_HWR_IRQ_EN_SET(__reg__)  (((__reg__) & 0xffff0000) >> 16u)
#define SET_PIRQ_HRWIRQENSET_HWR_IRQ_EN_SET(__val__)  (((__val__) << 16u) & 0xffff0000u)

/** @brief Register definition for @ref PIRQ_t.Hrwirqenset. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt16 hrd_irq_en_set;
        /** @brief  */
        UInt16 hwr_irq_en_set;
    } bits;
} RegPIRQHrwirqenset_t;

#define REG_PIRQ_HRWIRQENCLR ((volatile UInt32*)0xf00150) /*  */
#define     PIRQ_HRWIRQENCLR_HRD_IRQ_EN_CLR_SHIFT 0u
#define     PIRQ_HRWIRQENCLR_HRD_IRQ_EN_CLR_MASK  0xffffu
#define GET_PIRQ_HRWIRQENCLR_HRD_IRQ_EN_CLR(__reg__)  (((__reg__) & 0xffff) >> 0u)
#define SET_PIRQ_HRWIRQENCLR_HRD_IRQ_EN_CLR(__val__)  (((__val__) << 0u) & 0xffffu)
#define     PIRQ_HRWIRQENCLR_HWR_IRQ_EN_CLR_SHIFT 16u
#define     PIRQ_HRWIRQENCLR_HWR_IRQ_EN_CLR_MASK  0xffff0000u
#define GET_PIRQ_HRWIRQENCLR_HWR_IRQ_EN_CLR(__reg__)  (((__reg__) & 0xffff0000) >> 16u)
#define SET_PIRQ_HRWIRQENCLR_HWR_IRQ_EN_CLR(__val__)  (((__val__) << 16u) & 0xffff0000u)

/** @brief Register definition for @ref PIRQ_t.Hrwirqenclr. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt16 hrd_irq_en_clr;
        /** @brief  */
        UInt16 hwr_irq_en_clr;
    } bits;
} RegPIRQHrwirqenclr_t;

#define REG_PIRQ_HRWIRQENVALUE ((volatile UInt32*)0xf00154) /*  */
#define     PIRQ_HRWIRQENVALUE_HRD_IRQ_EN_VAL_SHIFT 0u
#define     PIRQ_HRWIRQENVALUE_HRD_IRQ_EN_VAL_MASK  0xffffu
#define GET_PIRQ_HRWIRQENVALUE_HRD_IRQ_EN_VAL(__reg__)  (((__reg__) & 0xffff) >> 0u)
#define SET_PIRQ_HRWIRQENVALUE_HRD_IRQ_EN_VAL(__val__)  (((__val__) << 0u) & 0xffffu)
#define     PIRQ_HRWIRQENVALUE_HWR_IRQ_EN_VAL_SHIFT 16u
#define     PIRQ_HRWIRQENVALUE_HWR_IRQ_EN_VAL_MASK  0xffff0000u
#define GET_PIRQ_HRWIRQENVALUE_HWR_IRQ_EN_VAL(__reg__)  (((__reg__) & 0xffff0000) >> 16u)
#define SET_PIRQ_HRWIRQENVALUE_HWR_IRQ_EN_VAL(__val__)  (((__val__) << 16u) & 0xffff0000u)

/** @brief Register definition for @ref PIRQ_t.Hrwirqenvalue. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt16 hrd_irq_en_val;
        /** @brief  */
        UInt16 hwr_irq_en_val;
    } bits;
} RegPIRQHrwirqenvalue_t;

#define REG_PIRQ_HRWSTATUSCLR ((volatile UInt32*)0xf00158) /*  */
#define     PIRQ_HRWSTATUSCLR_HRD_IRQ_STAT_CLR_SHIFT 0u
#define     PIRQ_HRWSTATUSCLR_HRD_IRQ_STAT_CLR_MASK  0xffffu
#define GET_PIRQ_HRWSTATUSCLR_HRD_IRQ_STAT_CLR(__reg__)  (((__reg__) & 0xffff) >> 0u)
#define SET_PIRQ_HRWSTATUSCLR_HRD_IRQ_STAT_CLR(__val__)  (((__val__) << 0u) & 0xffffu)
#define     PIRQ_HRWSTATUSCLR_HWR_IRQ_STAT_CLR_SHIFT 16u
#define     PIRQ_HRWSTATUSCLR_HWR_IRQ_STAT_CLR_MASK  0xffff0000u
#define GET_PIRQ_HRWSTATUSCLR_HWR_IRQ_STAT_CLR(__reg__)  (((__reg__) & 0xffff0000) >> 16u)
#define SET_PIRQ_HRWSTATUSCLR_HWR_IRQ_STAT_CLR(__val__)  (((__val__) << 16u) & 0xffff0000u)

/** @brief Register definition for @ref PIRQ_t.Hrwstatusclr. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt16 hrd_irq_stat_clr;
        /** @brief  */
        UInt16 hwr_irq_stat_clr;
    } bits;
} RegPIRQHrwstatusclr_t;

#define REG_PIRQ_HRWSTATUSVALUE ((volatile UInt32*)0xf0015c) /*  */
#define     PIRQ_HRWSTATUSVALUE_HRD_IRQ_STAT_VAL_SHIFT 0u
#define     PIRQ_HRWSTATUSVALUE_HRD_IRQ_STAT_VAL_MASK  0xffffu
#define GET_PIRQ_HRWSTATUSVALUE_HRD_IRQ_STAT_VAL(__reg__)  (((__reg__) & 0xffff) >> 0u)
#define SET_PIRQ_HRWSTATUSVALUE_HRD_IRQ_STAT_VAL(__val__)  (((__val__) << 0u) & 0xffffu)
#define     PIRQ_HRWSTATUSVALUE_HWR_IRQ_STAT_VAL_SHIFT 16u
#define     PIRQ_HRWSTATUSVALUE_HWR_IRQ_STAT_VAL_MASK  0xffff0000u
#define GET_PIRQ_HRWSTATUSVALUE_HWR_IRQ_STAT_VAL(__reg__)  (((__reg__) & 0xffff0000) >> 16u)
#define SET_PIRQ_HRWSTATUSVALUE_HWR_IRQ_STAT_VAL(__val__)  (((__val__) << 16u) & 0xffff0000u)

/** @brief Register definition for @ref PIRQ_t.Hrwstatusvalue. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt16 hrd_irq_stat_val;
        /** @brief  */
        UInt16 hwr_irq_stat_val;
    } bits;
} RegPIRQHrwstatusvalue_t;

/** @brief Component definition for @ref PIRQ. */
typedef struct {
    /** @brief  */
    RegPIRQIrqenset_t Irqenset;

    /** @brief  */
    RegPIRQIrqenclr_t Irqenclr;

    /** @brief  */
    RegPIRQIrqenvalue_t Irqenvalue;

    /** @brief  */
    RegPIRQStatusclr_t Statusclr;

    /** @brief  */
    RegPIRQStatusvalue_t Statusvalue;

    /** @brief  */
    RegPIRQIrqmaskset_t Irqmaskset;

    /** @brief  */
    RegPIRQIrqmaskclr_t Irqmaskclr;

    /** @brief  */
    RegPIRQIrqmaskvalue_t Irqmaskvalue;

    /** @brief  */
    RegPIRQSwirqset_t Swirqset;

    /** @brief  */
    RegPIRQSwirqclr_t Swirqclr;

    /** @brief  */
    RegPIRQSwirqvalue_t Swirqvalue;

    /** @brief  */
    RegPIRQHrwirqenset_t Hrwirqenset;

    /** @brief  */
    RegPIRQHrwirqenclr_t Hrwirqenclr;

    /** @brief  */
    RegPIRQHrwirqenvalue_t Hrwirqenvalue;

    /** @brief  */
    RegPIRQHrwstatusclr_t Hrwstatusclr;

    /** @brief  */
    RegPIRQHrwstatusvalue_t Hrwstatusvalue;

} PIRQ_t;

/** @brief  */
extern volatile PIRQ_t PIRQ;


#define REG_QSPI_CONTROL0 ((volatile UInt32*)0xf00170) /*  */
#define     QSPI_CONTROL0_QSPI_EN_SHIFT 0u
#define     QSPI_CONTROL0_QSPI_EN_MASK  0x1u
#define GET_QSPI_CONTROL0_QSPI_EN(__reg__)  (((__reg__) & 0x1) >> 0u)
#define SET_QSPI_CONTROL0_QSPI_EN(__val__)  (((__val__) << 0u) & 0x1u)
#define     QSPI_CONTROL0_QSPI_PROT_SHIFT 1u
#define     QSPI_CONTROL0_QSPI_PROT_MASK  0xeu
#define GET_QSPI_CONTROL0_QSPI_PROT(__reg__)  (((__reg__) & 0xe) >> 1u)
#define SET_QSPI_CONTROL0_QSPI_PROT(__val__)  (((__val__) << 1u) & 0xeu)
#define     QSPI_CONTROL0_QSPI_RW_SHIFT 4u
#define     QSPI_CONTROL0_QSPI_RW_MASK  0x10u
#define GET_QSPI_CONTROL0_QSPI_RW(__reg__)  (((__reg__) & 0x10) >> 4u)
#define SET_QSPI_CONTROL0_QSPI_RW(__val__)  (((__val__) << 4u) & 0x10u)
#define     QSPI_CONTROL0_QSPI_CMD_LNGTH_SHIFT 5u
#define     QSPI_CONTROL0_QSPI_CMD_LNGTH_MASK  0x20u
#define GET_QSPI_CONTROL0_QSPI_CMD_LNGTH(__reg__)  (((__reg__) & 0x20) >> 5u)
#define SET_QSPI_CONTROL0_QSPI_CMD_LNGTH(__val__)  (((__val__) << 5u) & 0x20u)
#define     QSPI_CONTROL0_QSPI_ADDR_LNGTH_SHIFT 6u
#define     QSPI_CONTROL0_QSPI_ADDR_LNGTH_MASK  0xc0u
#define GET_QSPI_CONTROL0_QSPI_ADDR_LNGTH(__reg__)  (((__reg__) & 0xc0) >> 6u)
#define SET_QSPI_CONTROL0_QSPI_ADDR_LNGTH(__val__)  (((__val__) << 6u) & 0xc0u)
#define     QSPI_CONTROL0_QSPI_CFG_LNGTH_SHIFT 8u
#define     QSPI_CONTROL0_QSPI_CFG_LNGTH_MASK  0xf00u
#define GET_QSPI_CONTROL0_QSPI_CFG_LNGTH(__reg__)  (((__reg__) & 0xf00) >> 8u)
#define SET_QSPI_CONTROL0_QSPI_CFG_LNGTH(__val__)  (((__val__) << 8u) & 0xf00u)
#define     QSPI_CONTROL0_QSPI_DMY_LNGTH_SHIFT 12u
#define     QSPI_CONTROL0_QSPI_DMY_LNGTH_MASK  0xf000u
#define GET_QSPI_CONTROL0_QSPI_DMY_LNGTH(__reg__)  (((__reg__) & 0xf000) >> 12u)
#define SET_QSPI_CONTROL0_QSPI_DMY_LNGTH(__val__)  (((__val__) << 12u) & 0xf000u)
#define     QSPI_CONTROL0_QSPI_DATA_LNGTH_SHIFT 16u
#define     QSPI_CONTROL0_QSPI_DATA_LNGTH_MASK  0x1ff0000u
#define GET_QSPI_CONTROL0_QSPI_DATA_LNGTH(__reg__)  (((__reg__) & 0x1ff0000) >> 16u)
#define SET_QSPI_CONTROL0_QSPI_DATA_LNGTH(__val__)  (((__val__) << 16u) & 0x1ff0000u)
#define     QSPI_CONTROL0_RESERVED_30_25_SHIFT 25u
#define     QSPI_CONTROL0_RESERVED_30_25_MASK  0x7e000000u
#define GET_QSPI_CONTROL0_RESERVED_30_25(__reg__)  (((__reg__) & 0x7e000000) >> 25u)
#define SET_QSPI_CONTROL0_RESERVED_30_25(__val__)  (((__val__) << 25u) & 0x7e000000u)
#define     QSPI_CONTROL0_QSPI_REQ_SHIFT 31u
#define     QSPI_CONTROL0_QSPI_REQ_MASK  0x80000000u
#define GET_QSPI_CONTROL0_QSPI_REQ(__reg__)  (((__reg__) & 0x80000000) >> 31u)
#define SET_QSPI_CONTROL0_QSPI_REQ(__val__)  (((__val__) << 31u) & 0x80000000u)

/** @brief Register definition for @ref QSPI_t.Control0. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief When asserted '1', this bit enables the QSPI interface to send instructions when a transaction request is issued. When this bit is de-asserted '0', any transaction currently in progress finishes before disabling the interface. */
        UInt8 qspi_en:1;
        /** @brief This bit-field defines the bus protocol. */
        UInt8 qspi_prot:3;
        /** @brief This bit indicates whether the instruction defined via the instruction registers is a read or write instruction. 0: Read instruction. 1: Write instruction. */
        UInt8 qspi_rw:1;
        /** @brief This bit indicates whether the instruction defined via the instruction registers has a command phase. */
        UInt8 qspi_cmd_lngth:1;
        /** @brief This bit-field indicates the length of the address phase of the instruction defined via the instruction registers. */
        UInt8 qspi_addr_lngth:2;
        /** @brief This bit-field indicates the length of the configuration phase of the instruction defined via the instruction registers. */
        UInt8 qspi_cfg_lngth:4;
        /** @brief This bit-field indicates the length of the dummy phase of the instruction defined via the instruction registers. */
        UInt8 qspi_dmy_lngth:4;
        /** @brief This bit-field indicates the length of the data phase of the instruction defined via the instruction registers. */
        UInt16 qspi_data_lngth:9;
        /** @brief Reserved bits 30 to 25. Reads as 0. */
        UInt16 reserved_30_25:6;
        /** @brief Reads as 0. */
        UInt16 qspi_req:1;
    } bits;
} RegQSPIControl0_t;

#define REG_QSPI_CONTROL1 ((volatile UInt32*)0xf00174) /*  */
#define     QSPI_CONTROL1_QSPI_RESET_SHIFT 0u
#define     QSPI_CONTROL1_QSPI_RESET_MASK  0x1u
#define GET_QSPI_CONTROL1_QSPI_RESET(__reg__)  (((__reg__) & 0x1) >> 0u)
#define SET_QSPI_CONTROL1_QSPI_RESET(__val__)  (((__val__) << 0u) & 0x1u)
#define     QSPI_CONTROL1_RESERVED_15_1_SHIFT 1u
#define     QSPI_CONTROL1_RESERVED_15_1_MASK  0xfffeu
#define GET_QSPI_CONTROL1_RESERVED_15_1(__reg__)  (((__reg__) & 0xfffe) >> 1u)
#define SET_QSPI_CONTROL1_RESERVED_15_1(__val__)  (((__val__) << 1u) & 0xfffeu)
#define     QSPI_CONTROL1_QSPI_PINS_SHIFT 16u
#define     QSPI_CONTROL1_QSPI_PINS_MASK  0x10000u
#define GET_QSPI_CONTROL1_QSPI_PINS(__reg__)  (((__reg__) & 0x10000) >> 16u)
#define SET_QSPI_CONTROL1_QSPI_PINS(__val__)  (((__val__) << 16u) & 0x10000u)
#define     QSPI_CONTROL1_QSPI_OUT0_SHIFT 17u
#define     QSPI_CONTROL1_QSPI_OUT0_MASK  0x20000u
#define GET_QSPI_CONTROL1_QSPI_OUT0(__reg__)  (((__reg__) & 0x20000) >> 17u)
#define SET_QSPI_CONTROL1_QSPI_OUT0(__val__)  (((__val__) << 17u) & 0x20000u)
#define     QSPI_CONTROL1_QSPI_OUT1_SHIFT 18u
#define     QSPI_CONTROL1_QSPI_OUT1_MASK  0x40000u
#define GET_QSPI_CONTROL1_QSPI_OUT1(__reg__)  (((__reg__) & 0x40000) >> 18u)
#define SET_QSPI_CONTROL1_QSPI_OUT1(__val__)  (((__val__) << 18u) & 0x40000u)
#define     QSPI_CONTROL1_RESERVED_31_19_SHIFT 19u
#define     QSPI_CONTROL1_RESERVED_31_19_MASK  0xfff80000u
#define GET_QSPI_CONTROL1_RESERVED_31_19(__reg__)  (((__reg__) & 0xfff80000) >> 19u)
#define SET_QSPI_CONTROL1_RESERVED_31_19(__val__)  (((__val__) << 19u) & 0xfff80000u)

/** @brief Register definition for @ref QSPI_t.Control1. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief Reads as 0. */
        UInt16 qspi_reset:1;
        /** @brief Reserved bits 15 to 1. Reads as 0. */
        UInt16 reserved_15_1:15;
        /** @brief This bit determines the function of qio[2] and qio[3]. */
        UInt16 qspi_pins:1;
        /** @brief This bit controls qio[2] if qspi_pins is 0x1 (for some Flash devices associated with write-protection control). */
        UInt16 qspi_out0:1;
        /** @brief This bit controls qio[3] if qspi_pins is 0x1 (for some Flash devices associated with reset or hold control). */
        UInt16 qspi_out1:1;
        /** @brief Reserved bits 31 to 19. Reads as 0. */
        UInt16 reserved_31_19:13;
    } bits;
} RegQSPIControl1_t;

#define REG_QSPI_INSTRUCTION0 ((volatile UInt32*)0xf00178) /*  */
#define     QSPI_INSTRUCTION0_QSPI_ADDR_SHIFT 0u
#define     QSPI_INSTRUCTION0_QSPI_ADDR_MASK  0xffffffu
#define GET_QSPI_INSTRUCTION0_QSPI_ADDR(__reg__)  (((__reg__) & 0xffffff) >> 0u)
#define SET_QSPI_INSTRUCTION0_QSPI_ADDR(__val__)  (((__val__) << 0u) & 0xffffffu)
#define     QSPI_INSTRUCTION0_QSPI_CMD_SHIFT 24u
#define     QSPI_INSTRUCTION0_QSPI_CMD_MASK  0xff000000u
#define GET_QSPI_INSTRUCTION0_QSPI_CMD(__reg__)  (((__reg__) & 0xff000000) >> 24u)
#define SET_QSPI_INSTRUCTION0_QSPI_CMD(__val__)  (((__val__) << 24u) & 0xff000000u)

/** @brief Register definition for @ref QSPI_t.Instruction0. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief This bit-field defines the address byte(s) in the instruction. */
        UInt32 qspi_addr:24;
        /** @brief This bit-field defines the command byte in the instruction. */
        UInt8 qspi_cmd;
    } bits;
} RegQSPIInstruction0_t;

#define REG_QSPI_INSTRUCTION1 ((volatile UInt32*)0xf0017c) /*  */
#define     QSPI_INSTRUCTION1_QSPI_CFG_SHIFT 0u
#define     QSPI_INSTRUCTION1_QSPI_CFG_MASK  0xffu
#define GET_QSPI_INSTRUCTION1_QSPI_CFG(__reg__)  (((__reg__) & 0xff) >> 0u)
#define SET_QSPI_INSTRUCTION1_QSPI_CFG(__val__)  (((__val__) << 0u) & 0xffu)
#define     QSPI_INSTRUCTION1_RESERVED_31_8_SHIFT 8u
#define     QSPI_INSTRUCTION1_RESERVED_31_8_MASK  0xffffff00u
#define GET_QSPI_INSTRUCTION1_RESERVED_31_8(__reg__)  (((__reg__) & 0xffffff00) >> 8u)
#define SET_QSPI_INSTRUCTION1_RESERVED_31_8(__val__)  (((__val__) << 8u) & 0xffffff00u)

/** @brief Register definition for @ref QSPI_t.Instruction1. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief This bit-field defines the configuration byte in the instruction. */
        UInt32 qspi_cfg:8;
        /** @brief Reserved bits 31 to 8. Reads as 0. */
        UInt32 reserved_31_8:24;
    } bits;
} RegQSPIInstruction1_t;

#define REG_QSPI_DATA ((volatile UInt32*)0xf00180) /* This register holds read data received during a read instruction or programming data to be transmitted during a write instruction. */
#define     QSPI_DATA_QSPI_RD_DATA_SHIFT 0u
#define     QSPI_DATA_QSPI_RD_DATA_MASK  0xffffffffu
#define GET_QSPI_DATA_QSPI_RD_DATA(__reg__)  (((__reg__) & 0xffffffff) >> 0u)
#define SET_QSPI_DATA_QSPI_RD_DATA(__val__)  (((__val__) << 0u) & 0xffffffffu)

/** @brief Register definition for @ref QSPI_t.Data. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt32 qspi_rd_data;
    } bits;
} RegQSPIData_t;

#define REG_QSPI_STATUS ((volatile UInt32*)0xf00184) /*  */
#define     QSPI_STATUS_QSPI_BUSY_SHIFT 0u
#define     QSPI_STATUS_QSPI_BUSY_MASK  0x1u
#define GET_QSPI_STATUS_QSPI_BUSY(__reg__)  (((__reg__) & 0x1) >> 0u)
#define SET_QSPI_STATUS_QSPI_BUSY(__val__)  (((__val__) << 0u) & 0x1u)
#define     QSPI_STATUS_RESERVED_31_1_SHIFT 1u
#define     QSPI_STATUS_RESERVED_31_1_MASK  0xfffffffeu
#define GET_QSPI_STATUS_RESERVED_31_1(__reg__)  (((__reg__) & 0xfffffffe) >> 1u)
#define SET_QSPI_STATUS_RESERVED_31_1(__val__)  (((__val__) << 1u) & 0xfffffffeu)

/** @brief Register definition for @ref QSPI_t.Status. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief When '1', this bit indicates that an instruction is being executed. */
        UInt32 qspi_busy:1;
        /** @brief Reserved bits 31 to 1. Reads as 0. */
        UInt32 reserved_31_1:31;
    } bits;
} RegQSPIStatus_t;

/** @brief Component definition for @ref QSPI. */
typedef struct {
    /** @brief  */
    RegQSPIControl0_t Control0;

    /** @brief  */
    RegQSPIControl1_t Control1;

    /** @brief  */
    RegQSPIInstruction0_t Instruction0;

    /** @brief  */
    RegQSPIInstruction1_t Instruction1;

    /** @brief This register holds read data received during a read instruction or programming data to be transmitted during a write instruction. */
    RegQSPIData_t Data;

    /** @brief  */
    RegQSPIStatus_t Status;

} QSPI_t;

/** @brief  */
extern volatile QSPI_t QSPI;


#define REG_RTC_VALUE ((volatile UInt32*)0xf001b0) /* This bit-field shows the current value of the real time counter. */
#define     RTC_VALUE_RTC_VALUE_SHIFT 0u
#define     RTC_VALUE_RTC_VALUE_MASK  0xffffffffu
#define GET_RTC_VALUE_RTC_VALUE(__reg__)  (((__reg__) & 0xffffffff) >> 0u)
#define SET_RTC_VALUE_RTC_VALUE(__val__)  (((__val__) << 0u) & 0xffffffffu)

/** @brief Register definition for @ref RTC_t.Value. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt32 rtc_value;
    } bits;
} RegRTCValue_t;

/** @brief Component definition for @ref RTC. */
typedef struct {
    /** @brief This bit-field shows the current value of the real time counter. */
    RegRTCValue_t Value;

} RTC_t;

/** @brief  */
extern volatile RTC_t RTC;


#define REG_UTMR_CONTROL ((volatile UInt32*)0xf002c0) /*  */
#define     UTMR_CONTROL_UTMR_EN_SHIFT 0u
#define     UTMR_CONTROL_UTMR_EN_MASK  0x1u
#define GET_UTMR_CONTROL_UTMR_EN(__reg__)  (((__reg__) & 0x1) >> 0u)
#define SET_UTMR_CONTROL_UTMR_EN(__val__)  (((__val__) << 0u) & 0x1u)
#define     UTMR_CONTROL_UTMR_IRQ_SEL_SHIFT 1u
#define     UTMR_CONTROL_UTMR_IRQ_SEL_MASK  0x6u
#define GET_UTMR_CONTROL_UTMR_IRQ_SEL(__reg__)  (((__reg__) & 0x6) >> 1u)
#define SET_UTMR_CONTROL_UTMR_IRQ_SEL(__val__)  (((__val__) << 1u) & 0x6u)
#define     UTMR_CONTROL_UTMR_CLR_SHIFT 3u
#define     UTMR_CONTROL_UTMR_CLR_MASK  0x8u
#define GET_UTMR_CONTROL_UTMR_CLR(__reg__)  (((__reg__) & 0x8) >> 3u)
#define SET_UTMR_CONTROL_UTMR_CLR(__val__)  (((__val__) << 3u) & 0x8u)
#define     UTMR_CONTROL_UTMR_CLK_SRC_SHIFT 4u
#define     UTMR_CONTROL_UTMR_CLK_SRC_MASK  0x30u
#define GET_UTMR_CONTROL_UTMR_CLK_SRC(__reg__)  (((__reg__) & 0x30) >> 4u)
#define SET_UTMR_CONTROL_UTMR_CLK_SRC(__val__)  (((__val__) << 4u) & 0x30u)
#define     UTMR_CONTROL_UTMR_CLK_DIV_SHIFT 6u
#define     UTMR_CONTROL_UTMR_CLK_DIV_MASK  0xc0u
#define GET_UTMR_CONTROL_UTMR_CLK_DIV(__reg__)  (((__reg__) & 0xc0) >> 6u)
#define SET_UTMR_CONTROL_UTMR_CLK_DIV(__val__)  (((__val__) << 6u) & 0xc0u)
#define     UTMR_CONTROL_UTMR_START_SEL_SHIFT 8u
#define     UTMR_CONTROL_UTMR_START_SEL_MASK  0x100u
#define GET_UTMR_CONTROL_UTMR_START_SEL(__reg__)  (((__reg__) & 0x100) >> 8u)
#define SET_UTMR_CONTROL_UTMR_START_SEL(__val__)  (((__val__) << 8u) & 0x100u)
#define     UTMR_CONTROL_UTMR_START_ACT_SHIFT 9u
#define     UTMR_CONTROL_UTMR_START_ACT_MASK  0x200u
#define GET_UTMR_CONTROL_UTMR_START_ACT(__reg__)  (((__reg__) & 0x200) >> 9u)
#define SET_UTMR_CONTROL_UTMR_START_ACT(__val__)  (((__val__) << 9u) & 0x200u)
#define     UTMR_CONTROL_UTMR_CAP_SEL_SHIFT 10u
#define     UTMR_CONTROL_UTMR_CAP_SEL_MASK  0x400u
#define GET_UTMR_CONTROL_UTMR_CAP_SEL(__reg__)  (((__reg__) & 0x400) >> 10u)
#define SET_UTMR_CONTROL_UTMR_CAP_SEL(__val__)  (((__val__) << 10u) & 0x400u)
#define     UTMR_CONTROL_RESERVED_13_11_SHIFT 11u
#define     UTMR_CONTROL_RESERVED_13_11_MASK  0x3800u
#define GET_UTMR_CONTROL_RESERVED_13_11(__reg__)  (((__reg__) & 0x3800) >> 11u)
#define SET_UTMR_CONTROL_RESERVED_13_11(__val__)  (((__val__) << 11u) & 0x3800u)
#define     UTMR_CONTROL_UTMR_CAP_POL_SHIFT 14u
#define     UTMR_CONTROL_UTMR_CAP_POL_MASK  0xc000u
#define GET_UTMR_CONTROL_UTMR_CAP_POL(__reg__)  (((__reg__) & 0xc000) >> 14u)
#define SET_UTMR_CONTROL_UTMR_CAP_POL(__val__)  (((__val__) << 14u) & 0xc000u)
#define     UTMR_CONTROL_UTMR_RSTRT_SEL_SHIFT 16u
#define     UTMR_CONTROL_UTMR_RSTRT_SEL_MASK  0x10000u
#define GET_UTMR_CONTROL_UTMR_RSTRT_SEL(__reg__)  (((__reg__) & 0x10000) >> 16u)
#define SET_UTMR_CONTROL_UTMR_RSTRT_SEL(__val__)  (((__val__) << 16u) & 0x10000u)
#define     UTMR_CONTROL_UTMR_LIM_ACT_SHIFT 17u
#define     UTMR_CONTROL_UTMR_LIM_ACT_MASK  0x60000u
#define GET_UTMR_CONTROL_UTMR_LIM_ACT(__reg__)  (((__reg__) & 0x60000) >> 17u)
#define SET_UTMR_CONTROL_UTMR_LIM_ACT(__val__)  (((__val__) << 17u) & 0x60000u)
#define     UTMR_CONTROL_UTMR_CMP_ACT_SHIFT 19u
#define     UTMR_CONTROL_UTMR_CMP_ACT_MASK  0x180000u
#define GET_UTMR_CONTROL_UTMR_CMP_ACT(__reg__)  (((__reg__) & 0x180000) >> 19u)
#define SET_UTMR_CONTROL_UTMR_CMP_ACT(__val__)  (((__val__) << 19u) & 0x180000u)
#define     UTMR_CONTROL_UTMR_OUT_EN_SHIFT 21u
#define     UTMR_CONTROL_UTMR_OUT_EN_MASK  0x200000u
#define GET_UTMR_CONTROL_UTMR_OUT_EN(__reg__)  (((__reg__) & 0x200000) >> 21u)
#define SET_UTMR_CONTROL_UTMR_OUT_EN(__val__)  (((__val__) << 21u) & 0x200000u)
#define     UTMR_CONTROL_RESERVED_29_22_SHIFT 22u
#define     UTMR_CONTROL_RESERVED_29_22_MASK  0x3fc00000u
#define GET_UTMR_CONTROL_RESERVED_29_22(__reg__)  (((__reg__) & 0x3fc00000) >> 22u)
#define SET_UTMR_CONTROL_RESERVED_29_22(__val__)  (((__val__) << 22u) & 0x3fc00000u)
#define     UTMR_CONTROL_UTMR_SW_START_SHIFT 30u
#define     UTMR_CONTROL_UTMR_SW_START_MASK  0x40000000u
#define GET_UTMR_CONTROL_UTMR_SW_START(__reg__)  (((__reg__) & 0x40000000) >> 30u)
#define SET_UTMR_CONTROL_UTMR_SW_START(__val__)  (((__val__) << 30u) & 0x40000000u)
#define     UTMR_CONTROL_UTMR_SW_CAP_SHIFT 31u
#define     UTMR_CONTROL_UTMR_SW_CAP_MASK  0x80000000u
#define GET_UTMR_CONTROL_UTMR_SW_CAP(__reg__)  (((__reg__) & 0x80000000) >> 31u)
#define SET_UTMR_CONTROL_UTMR_SW_CAP(__val__)  (((__val__) << 31u) & 0x80000000u)

/** @brief Register definition for @ref UTMR_t.Control. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt8 utmr_en:1;
        /** @brief  */
        UInt8 utmr_irq_sel:2;
        /** @brief Reads as 0. */
        UInt8 utmr_clr:1;
        /** @brief  */
        UInt8 utmr_clk_src:2;
        /** @brief  */
        UInt8 utmr_clk_div:2;
        /** @brief  */
        UInt8 utmr_start_sel:1;
        /** @brief  */
        UInt8 utmr_start_act:1;
        /** @brief  */
        UInt8 utmr_cap_sel:1;
        /** @brief Reserved bits 13 to 11. Reads as 0. */
        UInt8 reserved_13_11:3;
        /** @brief  */
        UInt8 utmr_cap_pol:2;
        /** @brief This bit enables / disabled auto-restart when the timer reaches the programmed limit. 0: The timer stops. 1: The timer restarts at '0' */
        UInt16 utmr_rstrt_sel:1;
        /** @brief  */
        UInt16 utmr_lim_act:2;
        /** @brief  */
        UInt16 utmr_cmp_act:2;
        /** @brief  */
        UInt16 utmr_out_en:1;
        /** @brief Reserved bits 29 to 22. Reads as 0. */
        UInt16 reserved_29_22:8;
        /** @brief This bit starts / stops the timer when utmr_start_sel is programmed to 0x0. */
        UInt16 utmr_sw_start:1;
        /** @brief This bit triggers a capture when utmr_cap_sel is programmed to 0x0. */
        UInt16 utmr_sw_cap:1;
    } bits;
} RegUTMRControl_t;

#define REG_UTMR_LIMIT ((volatile UInt32*)0xf002c4) /*  */
#define     UTMR_LIMIT_UTMR_LIMIT_SHIFT 0u
#define     UTMR_LIMIT_UTMR_LIMIT_MASK  0xffffu
#define GET_UTMR_LIMIT_UTMR_LIMIT(__reg__)  (((__reg__) & 0xffff) >> 0u)
#define SET_UTMR_LIMIT_UTMR_LIMIT(__val__)  (((__val__) << 0u) & 0xffffu)
#define     UTMR_LIMIT_RESERVED_31_16_SHIFT 16u
#define     UTMR_LIMIT_RESERVED_31_16_MASK  0xffff0000u
#define GET_UTMR_LIMIT_RESERVED_31_16(__reg__)  (((__reg__) & 0xffff0000) >> 16u)
#define SET_UTMR_LIMIT_RESERVED_31_16(__val__)  (((__val__) << 16u) & 0xffff0000u)

/** @brief Register definition for @ref UTMR_t.Limit. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt16 utmr_limit;
        /** @brief Reserved bits 31 to 16. Reads as 0. */
        UInt16 reserved_31_16;
    } bits;
} RegUTMRLimit_t;

#define REG_UTMR_COMPARE ((volatile UInt32*)0xf002c8) /*  */
#define     UTMR_COMPARE_UTMR_CMP_VAL_SHIFT 0u
#define     UTMR_COMPARE_UTMR_CMP_VAL_MASK  0xffffu
#define GET_UTMR_COMPARE_UTMR_CMP_VAL(__reg__)  (((__reg__) & 0xffff) >> 0u)
#define SET_UTMR_COMPARE_UTMR_CMP_VAL(__val__)  (((__val__) << 0u) & 0xffffu)
#define     UTMR_COMPARE_RESERVED_31_16_SHIFT 16u
#define     UTMR_COMPARE_RESERVED_31_16_MASK  0xffff0000u
#define GET_UTMR_COMPARE_RESERVED_31_16(__reg__)  (((__reg__) & 0xffff0000) >> 16u)
#define SET_UTMR_COMPARE_RESERVED_31_16(__val__)  (((__val__) << 16u) & 0xffff0000u)

/** @brief Register definition for @ref UTMR_t.Compare. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt16 utmr_cmp_val;
        /** @brief Reserved bits 31 to 16. Reads as 0. */
        UInt16 reserved_31_16;
    } bits;
} RegUTMRCompare_t;

#define REG_UTMR_CAPTURE ((volatile UInt32*)0xf002cc) /*  */
#define     UTMR_CAPTURE_UTMR_CAP_VAL_SHIFT 0u
#define     UTMR_CAPTURE_UTMR_CAP_VAL_MASK  0xffffu
#define GET_UTMR_CAPTURE_UTMR_CAP_VAL(__reg__)  (((__reg__) & 0xffff) >> 0u)
#define SET_UTMR_CAPTURE_UTMR_CAP_VAL(__val__)  (((__val__) << 0u) & 0xffffu)
#define     UTMR_CAPTURE_RESERVED_31_16_SHIFT 16u
#define     UTMR_CAPTURE_RESERVED_31_16_MASK  0xffff0000u
#define GET_UTMR_CAPTURE_RESERVED_31_16(__reg__)  (((__reg__) & 0xffff0000) >> 16u)
#define SET_UTMR_CAPTURE_RESERVED_31_16(__val__)  (((__val__) << 16u) & 0xffff0000u)

/** @brief Register definition for @ref UTMR_t.Capture. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt16 utmr_cap_val;
        /** @brief Reserved bits 31 to 16. Reads as 0. */
        UInt16 reserved_31_16;
    } bits;
} RegUTMRCapture_t;

#define REG_UTMR_VALUE ((volatile UInt32*)0xf002d0) /*  */
#define     UTMR_VALUE_UTMR_VALUE_SHIFT 0u
#define     UTMR_VALUE_UTMR_VALUE_MASK  0xffffu
#define GET_UTMR_VALUE_UTMR_VALUE(__reg__)  (((__reg__) & 0xffff) >> 0u)
#define SET_UTMR_VALUE_UTMR_VALUE(__val__)  (((__val__) << 0u) & 0xffffu)
#define     UTMR_VALUE_RESERVED_31_16_SHIFT 16u
#define     UTMR_VALUE_RESERVED_31_16_MASK  0xffff0000u
#define GET_UTMR_VALUE_RESERVED_31_16(__reg__)  (((__reg__) & 0xffff0000) >> 16u)
#define SET_UTMR_VALUE_RESERVED_31_16(__val__)  (((__val__) << 16u) & 0xffff0000u)

/** @brief Register definition for @ref UTMR_t.Value. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt16 utmr_value;
        /** @brief Reserved bits 31 to 16. Reads as 0. */
        UInt16 reserved_31_16;
    } bits;
} RegUTMRValue_t;

#define REG_UTMR_STATUS ((volatile UInt32*)0xf002d4) /*  */
#define     UTMR_STATUS_UTMR_BUSY_SHIFT 0u
#define     UTMR_STATUS_UTMR_BUSY_MASK  0x1u
#define GET_UTMR_STATUS_UTMR_BUSY(__reg__)  (((__reg__) & 0x1) >> 0u)
#define SET_UTMR_STATUS_UTMR_BUSY(__val__)  (((__val__) << 0u) & 0x1u)
#define     UTMR_STATUS_RESERVED_31_1_SHIFT 1u
#define     UTMR_STATUS_RESERVED_31_1_MASK  0xfffffffeu
#define GET_UTMR_STATUS_RESERVED_31_1(__reg__)  (((__reg__) & 0xfffffffe) >> 1u)
#define SET_UTMR_STATUS_RESERVED_31_1(__val__)  (((__val__) << 1u) & 0xfffffffeu)

/** @brief Register definition for @ref UTMR_t.Status. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt32 utmr_busy:1;
        /** @brief Reserved bits 31 to 1. Reads as 0. */
        UInt32 reserved_31_1:31;
    } bits;
} RegUTMRStatus_t;

/** @brief Component definition for @ref UTMR. */
typedef struct {
    /** @brief  */
    RegUTMRControl_t Control;

    /** @brief  */
    RegUTMRLimit_t Limit;

    /** @brief  */
    RegUTMRCompare_t Compare;

    /** @brief  */
    RegUTMRCapture_t Capture;

    /** @brief  */
    RegUTMRValue_t Value;

    /** @brief  */
    RegUTMRStatus_t Status;

} UTMR_t;

/** @brief  */
extern volatile UTMR_t UTMR;


#endif /* !PERIPHERALS_H */

/** @} */
