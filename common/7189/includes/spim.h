////////////////////////////////////////////////////////////////////////////////
///
/// @file       common/7189/includes/spim.h
///
/// @project    EM7189
///
/// @brief      spim registers
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

/** @defgroup SPIM_H    spim registers */
/** @addtogroup SPIM_H
 * @{
 */
#ifndef SPIM_H
#define SPIM_H

#include <types.h>

#define REG_SPIM0_CONFIG ((volatile UInt32*)0xf00300) /*  */
#define     SPIM0_CONFIG_SPIM_CLK_SEL_SHIFT 0u
#define     SPIM0_CONFIG_SPIM_CLK_SEL_MASK  0x7u
#define GET_SPIM0_CONFIG_SPIM_CLK_SEL(__reg__)  (((__reg__) & 0x7) >> 0u)
#define SET_SPIM0_CONFIG_SPIM_CLK_SEL(__val__)  (((__val__) << 0u) & 0x7u)
#define     SPIM0_CONFIG_SPIM_CLK_SEL_SYSOSC 0x0u
#define     SPIM0_CONFIG_SPIM_CLK_SEL_SYSOSC_DIV_2 0x1u
#define     SPIM0_CONFIG_SPIM_CLK_SEL_SYSOSC_DIV_3 0x2u
#define     SPIM0_CONFIG_SPIM_CLK_SEL_SYSOSC_DIV_4 0x3u
#define     SPIM0_CONFIG_SPIM_CLK_SEL_SYSOSC_DIV_6 0x4u
#define     SPIM0_CONFIG_SPIM_CLK_SEL_SYSOSC_DIV_8 0x5u
#define     SPIM0_CONFIG_SPIM_CLK_SEL_SYSOSC_DIV_16 0x6u
#define     SPIM0_CONFIG_SPIM_CLK_SEL_SYSOSC_DIV_32 0x7u

#define     SPIM0_CONFIG_SPIM_CPHA_SHIFT 3u
#define     SPIM0_CONFIG_SPIM_CPHA_MASK  0x8u
#define GET_SPIM0_CONFIG_SPIM_CPHA(__reg__)  (((__reg__) & 0x8) >> 3u)
#define SET_SPIM0_CONFIG_SPIM_CPHA(__val__)  (((__val__) << 3u) & 0x8u)
#define     SPIM0_CONFIG_SPIM_CPOL_SHIFT 4u
#define     SPIM0_CONFIG_SPIM_CPOL_MASK  0x10u
#define GET_SPIM0_CONFIG_SPIM_CPOL(__reg__)  (((__reg__) & 0x10) >> 4u)
#define SET_SPIM0_CONFIG_SPIM_CPOL(__val__)  (((__val__) << 4u) & 0x10u)
#define     SPIM0_CONFIG_SPIM_MSB_SHIFT 5u
#define     SPIM0_CONFIG_SPIM_MSB_MASK  0x20u
#define GET_SPIM0_CONFIG_SPIM_MSB(__reg__)  (((__reg__) & 0x20) >> 5u)
#define SET_SPIM0_CONFIG_SPIM_MSB(__val__)  (((__val__) << 5u) & 0x20u)
#define     SPIM0_CONFIG_SPIM_ICFG_SHIFT 6u
#define     SPIM0_CONFIG_SPIM_ICFG_MASK  0x40u
#define GET_SPIM0_CONFIG_SPIM_ICFG(__reg__)  (((__reg__) & 0x40) >> 6u)
#define SET_SPIM0_CONFIG_SPIM_ICFG(__val__)  (((__val__) << 6u) & 0x40u)
#define     SPIM0_CONFIG_SPIM_ICFG_4_WIRE 0x0u
#define     SPIM0_CONFIG_SPIM_ICFG_3_WIRE 0x1u

#define     SPIM0_CONFIG_RESERVED_7_7_SHIFT 7u
#define     SPIM0_CONFIG_RESERVED_7_7_MASK  0x80u
#define GET_SPIM0_CONFIG_RESERVED_7_7(__reg__)  (((__reg__) & 0x80) >> 7u)
#define SET_SPIM0_CONFIG_RESERVED_7_7(__val__)  (((__val__) << 7u) & 0x80u)
#define     SPIM0_CONFIG_SPIM_EN_SHIFT 8u
#define     SPIM0_CONFIG_SPIM_EN_MASK  0x100u
#define GET_SPIM0_CONFIG_SPIM_EN(__reg__)  (((__reg__) & 0x100) >> 8u)
#define SET_SPIM0_CONFIG_SPIM_EN(__val__)  (((__val__) << 8u) & 0x100u)
#define     SPIM0_CONFIG_RESERVED_15_9_SHIFT 9u
#define     SPIM0_CONFIG_RESERVED_15_9_MASK  0xfe00u
#define GET_SPIM0_CONFIG_RESERVED_15_9(__reg__)  (((__reg__) & 0xfe00) >> 9u)
#define SET_SPIM0_CONFIG_RESERVED_15_9(__val__)  (((__val__) << 9u) & 0xfe00u)
#define     SPIM0_CONFIG_SPIM_RW_SEL_SHIFT 16u
#define     SPIM0_CONFIG_SPIM_RW_SEL_MASK  0x10000u
#define GET_SPIM0_CONFIG_SPIM_RW_SEL(__reg__)  (((__reg__) & 0x10000) >> 16u)
#define SET_SPIM0_CONFIG_SPIM_RW_SEL(__val__)  (((__val__) << 16u) & 0x10000u)
#define     SPIM0_CONFIG_SPIM_RW_SEL_WRITE 0x0u
#define     SPIM0_CONFIG_SPIM_RW_SEL_READ 0x1u

#define     SPIM0_CONFIG_RESERVED_31_17_SHIFT 17u
#define     SPIM0_CONFIG_RESERVED_31_17_MASK  0xfffe0000u
#define GET_SPIM0_CONFIG_RESERVED_31_17(__reg__)  (((__reg__) & 0xfffe0000) >> 17u)
#define SET_SPIM0_CONFIG_RESERVED_31_17(__val__)  (((__val__) << 17u) & 0xfffe0000u)

/** @brief Register definition for @ref SPIM0_t.Config. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief Clock Selection */
        UInt8 spim_clk_sel:3;
        /** @brief This bit, together with spim_cpol, controls when output data is presented / input data is sampled with respect to the SPI clock. */
        UInt8 spim_cpha:1;
        /** @brief This bit controls the inactive level of the clock. 0: Inactive low. 1: Inactive high. */
        UInt8 spim_cpol:1;
        /** @brief This bit controls the bit-order in transmission and reception. 0: LSB First. 1: MSB Frst */
        UInt8 spim_msb:1;
        /** @brief This bit configures the SPI protocol. 0: 3 wire. 1: 4 wire. */
        UInt8 spim_icfg:1;
        /** @brief Reserved bit 7. Reads as 0. */
        UInt8 reserved_7_7:1;
        /** @brief When asserted '1', this bit enables the SPI master sensor interface. */
        UInt8 spim_en:1;
        /** @brief Reserved bits 15 to 9. Reads as 0. */
        UInt8 reserved_15_9:7;
        /** @brief This bit determines the transaction direction when the SPI sensor interface is configured as 3-wire SPI. In 4wire SPI, this bit is ignored. 0: write. 1: read. */
        UInt16 spim_rw_sel:1;
        /** @brief Reserved bits 31 to 17. Reads as 0. */
        UInt16 reserved_31_17:15;
    } bits;
} RegSPIM0Config_t;

#define REG_SPIM0_CONTROL ((volatile UInt32*)0xf00304) /*  */
#define     SPIM0_CONTROL_SPIM_DAT_LNGTH_SHIFT 0u
#define     SPIM0_CONTROL_SPIM_DAT_LNGTH_MASK  0x1fu
#define GET_SPIM0_CONTROL_SPIM_DAT_LNGTH(__reg__)  (((__reg__) & 0x1f) >> 0u)
#define SET_SPIM0_CONTROL_SPIM_DAT_LNGTH(__val__)  (((__val__) << 0u) & 0x1fu)
#define     SPIM0_CONTROL_RESERVED_7_5_SHIFT 5u
#define     SPIM0_CONTROL_RESERVED_7_5_MASK  0xe0u
#define GET_SPIM0_CONTROL_RESERVED_7_5(__reg__)  (((__reg__) & 0xe0) >> 5u)
#define SET_SPIM0_CONTROL_RESERVED_7_5(__val__)  (((__val__) << 5u) & 0xe0u)
#define     SPIM0_CONTROL_SPIM_CMD_LNGTH_SHIFT 8u
#define     SPIM0_CONTROL_SPIM_CMD_LNGTH_MASK  0x700u
#define GET_SPIM0_CONTROL_SPIM_CMD_LNGTH(__reg__)  (((__reg__) & 0x700) >> 8u)
#define SET_SPIM0_CONTROL_SPIM_CMD_LNGTH(__val__)  (((__val__) << 8u) & 0x700u)
#define     SPIM0_CONTROL_RESERVED_15_11_SHIFT 11u
#define     SPIM0_CONTROL_RESERVED_15_11_MASK  0xf800u
#define GET_SPIM0_CONTROL_RESERVED_15_11(__reg__)  (((__reg__) & 0xf800) >> 11u)
#define SET_SPIM0_CONTROL_RESERVED_15_11(__val__)  (((__val__) << 11u) & 0xf800u)
#define     SPIM0_CONTROL_SPIM_START_SHIFT 16u
#define     SPIM0_CONTROL_SPIM_START_MASK  0x10000u
#define GET_SPIM0_CONTROL_SPIM_START(__reg__)  (((__reg__) & 0x10000) >> 16u)
#define SET_SPIM0_CONTROL_SPIM_START(__val__)  (((__val__) << 16u) & 0x10000u)
#define     SPIM0_CONTROL_SPIM_STOP_SHIFT 17u
#define     SPIM0_CONTROL_SPIM_STOP_MASK  0x20000u
#define GET_SPIM0_CONTROL_SPIM_STOP(__reg__)  (((__reg__) & 0x20000) >> 17u)
#define SET_SPIM0_CONTROL_SPIM_STOP(__val__)  (((__val__) << 17u) & 0x20000u)
#define     SPIM0_CONTROL_RESERVED_30_18_SHIFT 18u
#define     SPIM0_CONTROL_RESERVED_30_18_MASK  0x7ffc0000u
#define GET_SPIM0_CONTROL_RESERVED_30_18(__reg__)  (((__reg__) & 0x7ffc0000) >> 18u)
#define SET_SPIM0_CONTROL_RESERVED_30_18(__val__)  (((__val__) << 18u) & 0x7ffc0000u)
#define     SPIM0_CONTROL_SPIM_RESET_SHIFT 31u
#define     SPIM0_CONTROL_SPIM_RESET_MASK  0x80000000u
#define GET_SPIM0_CONTROL_SPIM_RESET(__reg__)  (((__reg__) & 0x80000000) >> 31u)
#define SET_SPIM0_CONTROL_SPIM_RESET(__val__)  (((__val__) << 31u) & 0x80000000u)

/** @brief Register definition for @ref SPIM0_t.Control. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief This bit-field defines the data transfer length in bytes. */
        UInt8 spim_dat_lngth:5;
        /** @brief Reserved bits 7 to 5. Reads as 0. */
        UInt8 reserved_7_5:3;
        /** @brief This bit-field defines the command length in bytes. */
        UInt8 spim_cmd_lngth:3;
        /** @brief Reserved bits 15 to 11. Reads as 0. */
        UInt8 reserved_15_11:5;
        /** @brief When asserted '1', this bit starts a transaction. The bit automatically clears when the request is carried out. */
        UInt16 spim_start:1;
        /** @brief When asserted '1', this bit stops a transaction. The bit automatically clears when the request is carried out. */
        UInt16 spim_stop:1;
        /** @brief Reserved bits 30 to 18. Reads as 0. */
        UInt16 reserved_30_18:13;
        /** @brief When asserted '1', this bit resets the state machine and all registers. */
        UInt16 spim_reset:1;
    } bits;
} RegSPIM0Control_t;

#define REG_SPIM0_STATUS ((volatile UInt32*)0xf00308) /*  */
#define     SPIM0_STATUS_SPIM_BUSY_SHIFT 0u
#define     SPIM0_STATUS_SPIM_BUSY_MASK  0x1u
#define GET_SPIM0_STATUS_SPIM_BUSY(__reg__)  (((__reg__) & 0x1) >> 0u)
#define SET_SPIM0_STATUS_SPIM_BUSY(__val__)  (((__val__) << 0u) & 0x1u)
#define     SPIM0_STATUS_SPIM_CMD_SENT_SHIFT 1u
#define     SPIM0_STATUS_SPIM_CMD_SENT_MASK  0x2u
#define GET_SPIM0_STATUS_SPIM_CMD_SENT(__reg__)  (((__reg__) & 0x2) >> 1u)
#define SET_SPIM0_STATUS_SPIM_CMD_SENT(__val__)  (((__val__) << 1u) & 0x2u)
#define     SPIM0_STATUS_SPIM_DAT_SENT_SHIFT 2u
#define     SPIM0_STATUS_SPIM_DAT_SENT_MASK  0x4u
#define GET_SPIM0_STATUS_SPIM_DAT_SENT(__reg__)  (((__reg__) & 0x4) >> 2u)
#define SET_SPIM0_STATUS_SPIM_DAT_SENT(__val__)  (((__val__) << 2u) & 0x4u)
#define     SPIM0_STATUS_RESERVED_3_3_SHIFT 3u
#define     SPIM0_STATUS_RESERVED_3_3_MASK  0x8u
#define GET_SPIM0_STATUS_RESERVED_3_3(__reg__)  (((__reg__) & 0x8) >> 3u)
#define SET_SPIM0_STATUS_RESERVED_3_3(__val__)  (((__val__) << 3u) & 0x8u)
#define     SPIM0_STATUS_SPIM_STOP_STAT_SHIFT 4u
#define     SPIM0_STATUS_SPIM_STOP_STAT_MASK  0x10u
#define GET_SPIM0_STATUS_SPIM_STOP_STAT(__reg__)  (((__reg__) & 0x10) >> 4u)
#define SET_SPIM0_STATUS_SPIM_STOP_STAT(__val__)  (((__val__) << 4u) & 0x10u)
#define     SPIM0_STATUS_RESERVED_7_5_SHIFT 5u
#define     SPIM0_STATUS_RESERVED_7_5_MASK  0xe0u
#define GET_SPIM0_STATUS_RESERVED_7_5(__reg__)  (((__reg__) & 0xe0) >> 5u)
#define SET_SPIM0_STATUS_RESERVED_7_5(__val__)  (((__val__) << 5u) & 0xe0u)
#define     SPIM0_STATUS_SPIM_DATA_NUM_SHIFT 8u
#define     SPIM0_STATUS_SPIM_DATA_NUM_MASK  0x1f00u
#define GET_SPIM0_STATUS_SPIM_DATA_NUM(__reg__)  (((__reg__) & 0x1f00) >> 8u)
#define SET_SPIM0_STATUS_SPIM_DATA_NUM(__val__)  (((__val__) << 8u) & 0x1f00u)
#define     SPIM0_STATUS_RESERVED_15_13_SHIFT 13u
#define     SPIM0_STATUS_RESERVED_15_13_MASK  0xe000u
#define GET_SPIM0_STATUS_RESERVED_15_13(__reg__)  (((__reg__) & 0xe000) >> 13u)
#define SET_SPIM0_STATUS_RESERVED_15_13(__val__)  (((__val__) << 13u) & 0xe000u)
#define     SPIM0_STATUS_SPIM_CMD_NUM_SHIFT 16u
#define     SPIM0_STATUS_SPIM_CMD_NUM_MASK  0x70000u
#define GET_SPIM0_STATUS_SPIM_CMD_NUM(__reg__)  (((__reg__) & 0x70000) >> 16u)
#define SET_SPIM0_STATUS_SPIM_CMD_NUM(__val__)  (((__val__) << 16u) & 0x70000u)
#define     SPIM0_STATUS_RESERVED_31_19_SHIFT 19u
#define     SPIM0_STATUS_RESERVED_31_19_MASK  0xfff80000u
#define GET_SPIM0_STATUS_RESERVED_31_19(__reg__)  (((__reg__) & 0xfff80000) >> 19u)
#define SET_SPIM0_STATUS_RESERVED_31_19(__val__)  (((__val__) << 19u) & 0xfff80000u)

/** @brief Register definition for @ref SPIM0_t.Status. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief When '1', this bit indicates that a SPI transaction is in progress. */
        UInt8 spim_busy:1;
        /** @brief When '1', this bit indicates that all command bytes have been sent. */
        UInt8 spim_cmd_sent:1;
        /** @brief When '1', this bit indicates that all data bytes have been sent. */
        UInt8 spim_dat_sent:1;
        /** @brief Reserved bit 3. Reads as 0. */
        UInt8 reserved_3_3:1;
        /** @brief When '1', this bit indicates that a transaction has been stopped. */
        UInt8 spim_stop_stat:1;
        /** @brief Reserved bits 7 to 5. Reads as 0. */
        UInt8 reserved_7_5:3;
        /** @brief This bit-field shows the number of data bytes that have been transferred. */
        UInt8 spim_data_num:5;
        /** @brief Reserved bits 15 to 13. Reads as 0. */
        UInt8 reserved_15_13:3;
        /** @brief This bit-field shows the number of command bytes that have been transferred. */
        UInt16 spim_cmd_num:3;
        /** @brief Reserved bits 31 to 24. Reads as 0. */
        UInt16 reserved_31_19:13;
    } bits;
} RegSPIM0Status_t;

#define REG_SPIM0_TX_CMD ((volatile UInt32*)0xf0030c) /* This register holds the command to be transmitted and, optionally, a target address. */
/** @brief Register definition for @ref SPIM0_t.TxCmd. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];
} RegSPIM0TxCmd_t;

#define REG_SPIM0_TX_BUFFER0 ((volatile UInt32*)0xf00310) /* This register holds the data to be transmitted during write transactions. */
/** @brief Register definition for @ref SPIM0_t.TxBuffer0. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];
} RegSPIM0TxBuffer0_t;

#define REG_SPIM0_TX_BUFFER1 ((volatile UInt32*)0xf00314) /* This register holds the data to be transmitted during write transactions. */
/** @brief Register definition for @ref SPIM0_t.TxBuffer1. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];
} RegSPIM0TxBuffer1_t;

#define REG_SPIM0_TX_BUFFER2 ((volatile UInt32*)0xf00318) /* This register holds the data to be transmitted during write transactions. */
/** @brief Register definition for @ref SPIM0_t.TxBuffer2. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];
} RegSPIM0TxBuffer2_t;

#define REG_SPIM0_TX_BUFFER3 ((volatile UInt32*)0xf0031c) /* This register holds the data to be transmitted during write transactions. */
/** @brief Register definition for @ref SPIM0_t.TxBuffer3. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];
} RegSPIM0TxBuffer3_t;

#define REG_SPIM0_RX_CMD ((volatile UInt32*)0xf00320) /* This register holds the data received during command transmission. */
/** @brief Register definition for @ref SPIM0_t.RxCmd. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];
} RegSPIM0RxCmd_t;

#define REG_SPIM0_RX_BUFFER0 ((volatile UInt32*)0xf00324) /* This register holds the data received during read transactions. */
/** @brief Register definition for @ref SPIM0_t.RxBuffer0. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];
} RegSPIM0RxBuffer0_t;

#define REG_SPIM0_RX_BUFFER1 ((volatile UInt32*)0xf00328) /* This register holds the data received during read transactions. */
/** @brief Register definition for @ref SPIM0_t.RxBuffer1. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];
} RegSPIM0RxBuffer1_t;

#define REG_SPIM0_RX_BUFFER2 ((volatile UInt32*)0xf0032c) /* This register holds the data received during read transactions. */
/** @brief Register definition for @ref SPIM0_t.RxBuffer2. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];
} RegSPIM0RxBuffer2_t;

#define REG_SPIM0_RX_BUFFER3 ((volatile UInt32*)0xf00330) /* This register holds the data received during read transactions. */
/** @brief Register definition for @ref SPIM0_t.RxBuffer3. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];
} RegSPIM0RxBuffer3_t;

/** @brief Component definition for @ref SPIM0. */
typedef struct {
    /** @brief  */
    RegSPIM0Config_t Config;

    /** @brief  */
    RegSPIM0Control_t Control;

    /** @brief  */
    RegSPIM0Status_t Status;

    /** @brief This register holds the command to be transmitted and, optionally, a target address. */
    RegSPIM0TxCmd_t TxCmd;

    /** @brief This register holds the data to be transmitted during write transactions. */
    RegSPIM0TxBuffer0_t TxBuffer0;

    /** @brief This register holds the data to be transmitted during write transactions. */
    RegSPIM0TxBuffer1_t TxBuffer1;

    /** @brief This register holds the data to be transmitted during write transactions. */
    RegSPIM0TxBuffer2_t TxBuffer2;

    /** @brief This register holds the data to be transmitted during write transactions. */
    RegSPIM0TxBuffer3_t TxBuffer3;

    /** @brief This register holds the data received during command transmission. */
    RegSPIM0RxCmd_t RxCmd;

    /** @brief This register holds the data received during read transactions. */
    RegSPIM0RxBuffer0_t RxBuffer0;

    /** @brief This register holds the data received during read transactions. */
    RegSPIM0RxBuffer1_t RxBuffer1;

    /** @brief This register holds the data received during read transactions. */
    RegSPIM0RxBuffer2_t RxBuffer2;

    /** @brief This register holds the data received during read transactions. */
    RegSPIM0RxBuffer3_t RxBuffer3;

} SPIM0_t;

/** @brief  */
extern volatile SPIM0_t SPIM0;


#define REG_SPIM1_CONFIG ((volatile UInt32*)0xf00400) /*  */
#define     SPIM1_CONFIG_SPIM_CLK_SEL_SHIFT 0u
#define     SPIM1_CONFIG_SPIM_CLK_SEL_MASK  0x7u
#define GET_SPIM1_CONFIG_SPIM_CLK_SEL(__reg__)  (((__reg__) & 0x7) >> 0u)
#define SET_SPIM1_CONFIG_SPIM_CLK_SEL(__val__)  (((__val__) << 0u) & 0x7u)
#define     SPIM1_CONFIG_SPIM_CLK_SEL_SYSOSC 0x0u
#define     SPIM1_CONFIG_SPIM_CLK_SEL_SYSOSC_DIV_2 0x1u
#define     SPIM1_CONFIG_SPIM_CLK_SEL_SYSOSC_DIV_3 0x2u
#define     SPIM1_CONFIG_SPIM_CLK_SEL_SYSOSC_DIV_4 0x3u
#define     SPIM1_CONFIG_SPIM_CLK_SEL_SYSOSC_DIV_6 0x4u
#define     SPIM1_CONFIG_SPIM_CLK_SEL_SYSOSC_DIV_8 0x5u
#define     SPIM1_CONFIG_SPIM_CLK_SEL_SYSOSC_DIV_16 0x6u
#define     SPIM1_CONFIG_SPIM_CLK_SEL_SYSOSC_DIV_32 0x7u

#define     SPIM1_CONFIG_SPIM_CPHA_SHIFT 3u
#define     SPIM1_CONFIG_SPIM_CPHA_MASK  0x8u
#define GET_SPIM1_CONFIG_SPIM_CPHA(__reg__)  (((__reg__) & 0x8) >> 3u)
#define SET_SPIM1_CONFIG_SPIM_CPHA(__val__)  (((__val__) << 3u) & 0x8u)
#define     SPIM1_CONFIG_SPIM_CPOL_SHIFT 4u
#define     SPIM1_CONFIG_SPIM_CPOL_MASK  0x10u
#define GET_SPIM1_CONFIG_SPIM_CPOL(__reg__)  (((__reg__) & 0x10) >> 4u)
#define SET_SPIM1_CONFIG_SPIM_CPOL(__val__)  (((__val__) << 4u) & 0x10u)
#define     SPIM1_CONFIG_SPIM_MSB_SHIFT 5u
#define     SPIM1_CONFIG_SPIM_MSB_MASK  0x20u
#define GET_SPIM1_CONFIG_SPIM_MSB(__reg__)  (((__reg__) & 0x20) >> 5u)
#define SET_SPIM1_CONFIG_SPIM_MSB(__val__)  (((__val__) << 5u) & 0x20u)
#define     SPIM1_CONFIG_SPIM_ICFG_SHIFT 6u
#define     SPIM1_CONFIG_SPIM_ICFG_MASK  0x40u
#define GET_SPIM1_CONFIG_SPIM_ICFG(__reg__)  (((__reg__) & 0x40) >> 6u)
#define SET_SPIM1_CONFIG_SPIM_ICFG(__val__)  (((__val__) << 6u) & 0x40u)
#define     SPIM1_CONFIG_SPIM_ICFG_4_WIRE 0x0u
#define     SPIM1_CONFIG_SPIM_ICFG_3_WIRE 0x1u

#define     SPIM1_CONFIG_RESERVED_7_7_SHIFT 7u
#define     SPIM1_CONFIG_RESERVED_7_7_MASK  0x80u
#define GET_SPIM1_CONFIG_RESERVED_7_7(__reg__)  (((__reg__) & 0x80) >> 7u)
#define SET_SPIM1_CONFIG_RESERVED_7_7(__val__)  (((__val__) << 7u) & 0x80u)
#define     SPIM1_CONFIG_SPIM_EN_SHIFT 8u
#define     SPIM1_CONFIG_SPIM_EN_MASK  0x100u
#define GET_SPIM1_CONFIG_SPIM_EN(__reg__)  (((__reg__) & 0x100) >> 8u)
#define SET_SPIM1_CONFIG_SPIM_EN(__val__)  (((__val__) << 8u) & 0x100u)
#define     SPIM1_CONFIG_RESERVED_15_9_SHIFT 9u
#define     SPIM1_CONFIG_RESERVED_15_9_MASK  0xfe00u
#define GET_SPIM1_CONFIG_RESERVED_15_9(__reg__)  (((__reg__) & 0xfe00) >> 9u)
#define SET_SPIM1_CONFIG_RESERVED_15_9(__val__)  (((__val__) << 9u) & 0xfe00u)
#define     SPIM1_CONFIG_SPIM_RW_SEL_SHIFT 16u
#define     SPIM1_CONFIG_SPIM_RW_SEL_MASK  0x10000u
#define GET_SPIM1_CONFIG_SPIM_RW_SEL(__reg__)  (((__reg__) & 0x10000) >> 16u)
#define SET_SPIM1_CONFIG_SPIM_RW_SEL(__val__)  (((__val__) << 16u) & 0x10000u)
#define     SPIM1_CONFIG_SPIM_RW_SEL_WRITE 0x0u
#define     SPIM1_CONFIG_SPIM_RW_SEL_READ 0x1u

#define     SPIM1_CONFIG_RESERVED_31_17_SHIFT 17u
#define     SPIM1_CONFIG_RESERVED_31_17_MASK  0xfffe0000u
#define GET_SPIM1_CONFIG_RESERVED_31_17(__reg__)  (((__reg__) & 0xfffe0000) >> 17u)
#define SET_SPIM1_CONFIG_RESERVED_31_17(__val__)  (((__val__) << 17u) & 0xfffe0000u)

/** @brief Register definition for @ref SPIM1_t.Config. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief Clock Selection */
        UInt8 spim_clk_sel:3;
        /** @brief This bit, together with spim_cpol, controls when output data is presented / input data is sampled with respect to the SPI clock. */
        UInt8 spim_cpha:1;
        /** @brief This bit controls the inactive level of the clock. 0: Inactive low. 1: Inactive high. */
        UInt8 spim_cpol:1;
        /** @brief This bit controls the bit-order in transmission and reception. 0: LSB First. 1: MSB Frst */
        UInt8 spim_msb:1;
        /** @brief This bit configures the SPI protocol. 0: 3 wire. 1: 4 wire. */
        UInt8 spim_icfg:1;
        /** @brief Reserved bit 7. Reads as 0. */
        UInt8 reserved_7_7:1;
        /** @brief When asserted '1', this bit enables the SPI master sensor interface. */
        UInt8 spim_en:1;
        /** @brief Reserved bits 15 to 9. Reads as 0. */
        UInt8 reserved_15_9:7;
        /** @brief This bit determines the transaction direction when the SPI sensor interface is configured as 3-wire SPI. In 4wire SPI, this bit is ignored. 0: write. 1: read. */
        UInt16 spim_rw_sel:1;
        /** @brief Reserved bits 31 to 17. Reads as 0. */
        UInt16 reserved_31_17:15;
    } bits;
} RegSPIM1Config_t;

#define REG_SPIM1_CONTROL ((volatile UInt32*)0xf00404) /*  */
#define     SPIM1_CONTROL_SPIM_DAT_LNGTH_SHIFT 0u
#define     SPIM1_CONTROL_SPIM_DAT_LNGTH_MASK  0x1fu
#define GET_SPIM1_CONTROL_SPIM_DAT_LNGTH(__reg__)  (((__reg__) & 0x1f) >> 0u)
#define SET_SPIM1_CONTROL_SPIM_DAT_LNGTH(__val__)  (((__val__) << 0u) & 0x1fu)
#define     SPIM1_CONTROL_RESERVED_7_5_SHIFT 5u
#define     SPIM1_CONTROL_RESERVED_7_5_MASK  0xe0u
#define GET_SPIM1_CONTROL_RESERVED_7_5(__reg__)  (((__reg__) & 0xe0) >> 5u)
#define SET_SPIM1_CONTROL_RESERVED_7_5(__val__)  (((__val__) << 5u) & 0xe0u)
#define     SPIM1_CONTROL_SPIM_CMD_LNGTH_SHIFT 8u
#define     SPIM1_CONTROL_SPIM_CMD_LNGTH_MASK  0x700u
#define GET_SPIM1_CONTROL_SPIM_CMD_LNGTH(__reg__)  (((__reg__) & 0x700) >> 8u)
#define SET_SPIM1_CONTROL_SPIM_CMD_LNGTH(__val__)  (((__val__) << 8u) & 0x700u)
#define     SPIM1_CONTROL_RESERVED_15_11_SHIFT 11u
#define     SPIM1_CONTROL_RESERVED_15_11_MASK  0xf800u
#define GET_SPIM1_CONTROL_RESERVED_15_11(__reg__)  (((__reg__) & 0xf800) >> 11u)
#define SET_SPIM1_CONTROL_RESERVED_15_11(__val__)  (((__val__) << 11u) & 0xf800u)
#define     SPIM1_CONTROL_SPIM_START_SHIFT 16u
#define     SPIM1_CONTROL_SPIM_START_MASK  0x10000u
#define GET_SPIM1_CONTROL_SPIM_START(__reg__)  (((__reg__) & 0x10000) >> 16u)
#define SET_SPIM1_CONTROL_SPIM_START(__val__)  (((__val__) << 16u) & 0x10000u)
#define     SPIM1_CONTROL_SPIM_STOP_SHIFT 17u
#define     SPIM1_CONTROL_SPIM_STOP_MASK  0x20000u
#define GET_SPIM1_CONTROL_SPIM_STOP(__reg__)  (((__reg__) & 0x20000) >> 17u)
#define SET_SPIM1_CONTROL_SPIM_STOP(__val__)  (((__val__) << 17u) & 0x20000u)
#define     SPIM1_CONTROL_RESERVED_30_18_SHIFT 18u
#define     SPIM1_CONTROL_RESERVED_30_18_MASK  0x7ffc0000u
#define GET_SPIM1_CONTROL_RESERVED_30_18(__reg__)  (((__reg__) & 0x7ffc0000) >> 18u)
#define SET_SPIM1_CONTROL_RESERVED_30_18(__val__)  (((__val__) << 18u) & 0x7ffc0000u)
#define     SPIM1_CONTROL_SPIM_RESET_SHIFT 31u
#define     SPIM1_CONTROL_SPIM_RESET_MASK  0x80000000u
#define GET_SPIM1_CONTROL_SPIM_RESET(__reg__)  (((__reg__) & 0x80000000) >> 31u)
#define SET_SPIM1_CONTROL_SPIM_RESET(__val__)  (((__val__) << 31u) & 0x80000000u)

/** @brief Register definition for @ref SPIM1_t.Control. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief This bit-field defines the data transfer length in bytes. */
        UInt8 spim_dat_lngth:5;
        /** @brief Reserved bits 7 to 5. Reads as 0. */
        UInt8 reserved_7_5:3;
        /** @brief This bit-field defines the command length in bytes. */
        UInt8 spim_cmd_lngth:3;
        /** @brief Reserved bits 15 to 11. Reads as 0. */
        UInt8 reserved_15_11:5;
        /** @brief When asserted '1', this bit starts a transaction. The bit automatically clears when the request is carried out. */
        UInt16 spim_start:1;
        /** @brief When asserted '1', this bit stops a transaction. The bit automatically clears when the request is carried out. */
        UInt16 spim_stop:1;
        /** @brief Reserved bits 30 to 18. Reads as 0. */
        UInt16 reserved_30_18:13;
        /** @brief When asserted '1', this bit resets the state machine and all registers. */
        UInt16 spim_reset:1;
    } bits;
} RegSPIM1Control_t;

#define REG_SPIM1_STATUS ((volatile UInt32*)0xf00408) /*  */
#define     SPIM1_STATUS_SPIM_BUSY_SHIFT 0u
#define     SPIM1_STATUS_SPIM_BUSY_MASK  0x1u
#define GET_SPIM1_STATUS_SPIM_BUSY(__reg__)  (((__reg__) & 0x1) >> 0u)
#define SET_SPIM1_STATUS_SPIM_BUSY(__val__)  (((__val__) << 0u) & 0x1u)
#define     SPIM1_STATUS_SPIM_CMD_SENT_SHIFT 1u
#define     SPIM1_STATUS_SPIM_CMD_SENT_MASK  0x2u
#define GET_SPIM1_STATUS_SPIM_CMD_SENT(__reg__)  (((__reg__) & 0x2) >> 1u)
#define SET_SPIM1_STATUS_SPIM_CMD_SENT(__val__)  (((__val__) << 1u) & 0x2u)
#define     SPIM1_STATUS_SPIM_DAT_SENT_SHIFT 2u
#define     SPIM1_STATUS_SPIM_DAT_SENT_MASK  0x4u
#define GET_SPIM1_STATUS_SPIM_DAT_SENT(__reg__)  (((__reg__) & 0x4) >> 2u)
#define SET_SPIM1_STATUS_SPIM_DAT_SENT(__val__)  (((__val__) << 2u) & 0x4u)
#define     SPIM1_STATUS_RESERVED_3_3_SHIFT 3u
#define     SPIM1_STATUS_RESERVED_3_3_MASK  0x8u
#define GET_SPIM1_STATUS_RESERVED_3_3(__reg__)  (((__reg__) & 0x8) >> 3u)
#define SET_SPIM1_STATUS_RESERVED_3_3(__val__)  (((__val__) << 3u) & 0x8u)
#define     SPIM1_STATUS_SPIM_STOP_STAT_SHIFT 4u
#define     SPIM1_STATUS_SPIM_STOP_STAT_MASK  0x10u
#define GET_SPIM1_STATUS_SPIM_STOP_STAT(__reg__)  (((__reg__) & 0x10) >> 4u)
#define SET_SPIM1_STATUS_SPIM_STOP_STAT(__val__)  (((__val__) << 4u) & 0x10u)
#define     SPIM1_STATUS_RESERVED_7_5_SHIFT 5u
#define     SPIM1_STATUS_RESERVED_7_5_MASK  0xe0u
#define GET_SPIM1_STATUS_RESERVED_7_5(__reg__)  (((__reg__) & 0xe0) >> 5u)
#define SET_SPIM1_STATUS_RESERVED_7_5(__val__)  (((__val__) << 5u) & 0xe0u)
#define     SPIM1_STATUS_SPIM_DATA_NUM_SHIFT 8u
#define     SPIM1_STATUS_SPIM_DATA_NUM_MASK  0x1f00u
#define GET_SPIM1_STATUS_SPIM_DATA_NUM(__reg__)  (((__reg__) & 0x1f00) >> 8u)
#define SET_SPIM1_STATUS_SPIM_DATA_NUM(__val__)  (((__val__) << 8u) & 0x1f00u)
#define     SPIM1_STATUS_RESERVED_15_13_SHIFT 13u
#define     SPIM1_STATUS_RESERVED_15_13_MASK  0xe000u
#define GET_SPIM1_STATUS_RESERVED_15_13(__reg__)  (((__reg__) & 0xe000) >> 13u)
#define SET_SPIM1_STATUS_RESERVED_15_13(__val__)  (((__val__) << 13u) & 0xe000u)
#define     SPIM1_STATUS_SPIM_CMD_NUM_SHIFT 16u
#define     SPIM1_STATUS_SPIM_CMD_NUM_MASK  0x70000u
#define GET_SPIM1_STATUS_SPIM_CMD_NUM(__reg__)  (((__reg__) & 0x70000) >> 16u)
#define SET_SPIM1_STATUS_SPIM_CMD_NUM(__val__)  (((__val__) << 16u) & 0x70000u)
#define     SPIM1_STATUS_RESERVED_31_19_SHIFT 19u
#define     SPIM1_STATUS_RESERVED_31_19_MASK  0xfff80000u
#define GET_SPIM1_STATUS_RESERVED_31_19(__reg__)  (((__reg__) & 0xfff80000) >> 19u)
#define SET_SPIM1_STATUS_RESERVED_31_19(__val__)  (((__val__) << 19u) & 0xfff80000u)

/** @brief Register definition for @ref SPIM1_t.Status. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief When '1', this bit indicates that a SPI transaction is in progress. */
        UInt8 spim_busy:1;
        /** @brief When '1', this bit indicates that all command bytes have been sent. */
        UInt8 spim_cmd_sent:1;
        /** @brief When '1', this bit indicates that all data bytes have been sent. */
        UInt8 spim_dat_sent:1;
        /** @brief Reserved bit 3. Reads as 0. */
        UInt8 reserved_3_3:1;
        /** @brief When '1', this bit indicates that a transaction has been stopped. */
        UInt8 spim_stop_stat:1;
        /** @brief Reserved bits 7 to 5. Reads as 0. */
        UInt8 reserved_7_5:3;
        /** @brief This bit-field shows the number of data bytes that have been transferred. */
        UInt8 spim_data_num:5;
        /** @brief Reserved bits 15 to 13. Reads as 0. */
        UInt8 reserved_15_13:3;
        /** @brief This bit-field shows the number of command bytes that have been transferred. */
        UInt16 spim_cmd_num:3;
        /** @brief Reserved bits 31 to 24. Reads as 0. */
        UInt16 reserved_31_19:13;
    } bits;
} RegSPIM1Status_t;

#define REG_SPIM1_TX_CMD ((volatile UInt32*)0xf0040c) /* This register holds the command to be transmitted and, optionally, a target address. */
/** @brief Register definition for @ref SPIM1_t.TxCmd. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];
} RegSPIM1TxCmd_t;

#define REG_SPIM1_TX_BUFFER0 ((volatile UInt32*)0xf00410) /* This register holds the data to be transmitted during write transactions. */
/** @brief Register definition for @ref SPIM1_t.TxBuffer0. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];
} RegSPIM1TxBuffer0_t;

#define REG_SPIM1_TX_BUFFER1 ((volatile UInt32*)0xf00414) /* This register holds the data to be transmitted during write transactions. */
/** @brief Register definition for @ref SPIM1_t.TxBuffer1. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];
} RegSPIM1TxBuffer1_t;

#define REG_SPIM1_TX_BUFFER2 ((volatile UInt32*)0xf00418) /* This register holds the data to be transmitted during write transactions. */
/** @brief Register definition for @ref SPIM1_t.TxBuffer2. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];
} RegSPIM1TxBuffer2_t;

#define REG_SPIM1_TX_BUFFER3 ((volatile UInt32*)0xf0041c) /* This register holds the data to be transmitted during write transactions. */
/** @brief Register definition for @ref SPIM1_t.TxBuffer3. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];
} RegSPIM1TxBuffer3_t;

#define REG_SPIM1_RX_CMD ((volatile UInt32*)0xf00420) /* This register holds the data received during command transmission. */
/** @brief Register definition for @ref SPIM1_t.RxCmd. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];
} RegSPIM1RxCmd_t;

#define REG_SPIM1_RX_BUFFER0 ((volatile UInt32*)0xf00424) /* This register holds the data received during read transactions. */
/** @brief Register definition for @ref SPIM1_t.RxBuffer0. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];
} RegSPIM1RxBuffer0_t;

#define REG_SPIM1_RX_BUFFER1 ((volatile UInt32*)0xf00428) /* This register holds the data received during read transactions. */
/** @brief Register definition for @ref SPIM1_t.RxBuffer1. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];
} RegSPIM1RxBuffer1_t;

#define REG_SPIM1_RX_BUFFER2 ((volatile UInt32*)0xf0042c) /* This register holds the data received during read transactions. */
/** @brief Register definition for @ref SPIM1_t.RxBuffer2. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];
} RegSPIM1RxBuffer2_t;

#define REG_SPIM1_RX_BUFFER3 ((volatile UInt32*)0xf00430) /* This register holds the data received during read transactions. */
/** @brief Register definition for @ref SPIM1_t.RxBuffer3. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];
} RegSPIM1RxBuffer3_t;

/** @brief Component definition for @ref SPIM1. */
typedef struct {
    /** @brief  */
    RegSPIM1Config_t Config;

    /** @brief  */
    RegSPIM1Control_t Control;

    /** @brief  */
    RegSPIM1Status_t Status;

    /** @brief This register holds the command to be transmitted and, optionally, a target address. */
    RegSPIM1TxCmd_t TxCmd;

    /** @brief This register holds the data to be transmitted during write transactions. */
    RegSPIM1TxBuffer0_t TxBuffer0;

    /** @brief This register holds the data to be transmitted during write transactions. */
    RegSPIM1TxBuffer1_t TxBuffer1;

    /** @brief This register holds the data to be transmitted during write transactions. */
    RegSPIM1TxBuffer2_t TxBuffer2;

    /** @brief This register holds the data to be transmitted during write transactions. */
    RegSPIM1TxBuffer3_t TxBuffer3;

    /** @brief This register holds the data received during command transmission. */
    RegSPIM1RxCmd_t RxCmd;

    /** @brief This register holds the data received during read transactions. */
    RegSPIM1RxBuffer0_t RxBuffer0;

    /** @brief This register holds the data received during read transactions. */
    RegSPIM1RxBuffer1_t RxBuffer1;

    /** @brief This register holds the data received during read transactions. */
    RegSPIM1RxBuffer2_t RxBuffer2;

    /** @brief This register holds the data received during read transactions. */
    RegSPIM1RxBuffer3_t RxBuffer3;

} SPIM1_t;

/** @brief  */
extern volatile SPIM1_t SPIM1;


#endif /* !SPIM_H */

/** @} */
