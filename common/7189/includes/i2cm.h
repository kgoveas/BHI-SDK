////////////////////////////////////////////////////////////////////////////////
///
/// @file       common/7189/includes/i2cm.h
///
/// @project    EM7189
///
/// @brief      i2cm registers
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

/** @defgroup I2CM_H    i2cm registers */
/** @addtogroup I2CM_H
 * @{
 */
#ifndef I2CM_H
#define I2CM_H

#include <types.h>

#define REG_I2CM0_CONTROL ((volatile UInt32*)0xf00500) /*  */
#define     I2CM0_CONTROL_REG_ADDR_SHIFT 0u
#define     I2CM0_CONTROL_REG_ADDR_MASK  0xffu
#define GET_I2CM0_CONTROL_REG_ADDR(__reg__)  (((__reg__) & 0xff) >> 0u)
#define SET_I2CM0_CONTROL_REG_ADDR(__val__)  (((__val__) << 0u) & 0xffu)
#define     I2CM0_CONTROL_SLV_ADDR_SHIFT 8u
#define     I2CM0_CONTROL_SLV_ADDR_MASK  0x7f00u
#define GET_I2CM0_CONTROL_SLV_ADDR(__reg__)  (((__reg__) & 0x7f00) >> 8u)
#define SET_I2CM0_CONTROL_SLV_ADDR(__val__)  (((__val__) << 8u) & 0x7f00u)
#define     I2CM0_CONTROL_RW_SEL_SHIFT 15u
#define     I2CM0_CONTROL_RW_SEL_MASK  0x8000u
#define GET_I2CM0_CONTROL_RW_SEL(__reg__)  (((__reg__) & 0x8000) >> 15u)
#define SET_I2CM0_CONTROL_RW_SEL(__val__)  (((__val__) << 15u) & 0x8000u)
#define     I2CM0_CONTROL_TRANS_LENGTH_SHIFT 16u
#define     I2CM0_CONTROL_TRANS_LENGTH_MASK  0x1f0000u
#define GET_I2CM0_CONTROL_TRANS_LENGTH(__reg__)  (((__reg__) & 0x1f0000) >> 16u)
#define SET_I2CM0_CONTROL_TRANS_LENGTH(__val__)  (((__val__) << 16u) & 0x1f0000u)
#define     I2CM0_CONTROL_NO_STOP_SHIFT 21u
#define     I2CM0_CONTROL_NO_STOP_MASK  0x200000u
#define GET_I2CM0_CONTROL_NO_STOP(__reg__)  (((__reg__) & 0x200000) >> 21u)
#define SET_I2CM0_CONTROL_NO_STOP(__val__)  (((__val__) << 21u) & 0x200000u)
#define     I2CM0_CONTROL_RESERVED_22_SHIFT 22u
#define     I2CM0_CONTROL_RESERVED_22_MASK  0x400000u
#define GET_I2CM0_CONTROL_RESERVED_22(__reg__)  (((__reg__) & 0x400000) >> 22u)
#define SET_I2CM0_CONTROL_RESERVED_22(__val__)  (((__val__) << 22u) & 0x400000u)
#define     I2CM0_CONTROL_REG_ADDR_EN_SHIFT 23u
#define     I2CM0_CONTROL_REG_ADDR_EN_MASK  0x800000u
#define GET_I2CM0_CONTROL_REG_ADDR_EN(__reg__)  (((__reg__) & 0x800000) >> 23u)
#define SET_I2CM0_CONTROL_REG_ADDR_EN(__val__)  (((__val__) << 23u) & 0x800000u)
#define     I2CM0_CONTROL_START_TRANS_SHIFT 24u
#define     I2CM0_CONTROL_START_TRANS_MASK  0x1000000u
#define GET_I2CM0_CONTROL_START_TRANS(__reg__)  (((__reg__) & 0x1000000) >> 24u)
#define SET_I2CM0_CONTROL_START_TRANS(__val__)  (((__val__) << 24u) & 0x1000000u)
#define     I2CM0_CONTROL_STOP_TRANS_SHIFT 25u
#define     I2CM0_CONTROL_STOP_TRANS_MASK  0x2000000u
#define GET_I2CM0_CONTROL_STOP_TRANS(__reg__)  (((__reg__) & 0x2000000) >> 25u)
#define SET_I2CM0_CONTROL_STOP_TRANS(__val__)  (((__val__) << 25u) & 0x2000000u)
#define     I2CM0_CONTROL_RESERVED_30_26_SHIFT 26u
#define     I2CM0_CONTROL_RESERVED_30_26_MASK  0x7c000000u
#define GET_I2CM0_CONTROL_RESERVED_30_26(__reg__)  (((__reg__) & 0x7c000000) >> 26u)
#define SET_I2CM0_CONTROL_RESERVED_30_26(__val__)  (((__val__) << 26u) & 0x7c000000u)
#define     I2CM0_CONTROL_I2CM_RESET_SHIFT 31u
#define     I2CM0_CONTROL_I2CM_RESET_MASK  0x80000000u
#define GET_I2CM0_CONTROL_I2CM_RESET(__reg__)  (((__reg__) & 0x80000000) >> 31u)
#define SET_I2CM0_CONTROL_I2CM_RESET(__val__)  (((__val__) << 31u) & 0x80000000u)

/** @brief Register definition for @ref I2CM0_t.Control. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief This bit-field defines the register address to be transmitted during an I2C read or write transaction when register address transmission is enabled */
        UInt8 reg_addr;
        /** @brief This bit-field defines the slave device address to be transmitted during an I2C read or write transaction. The slave address is always transmitted immediately after a START condition has been issued. */
        UInt8 slv_addr:7;
        /** @brief This bit defines the I2C transaction type: 0: write. 1: I2C read. The transaction type is always transmitted immediately after the slave device address. */
        UInt8 rw_sel:1;
        /** @brief This bit-field defines the number of data bytes to be transacted during an I2C read or write. Valid values are 0x00 to 0x10 (16 bytes). */
        UInt8 trans_length:5;
        /** @brief Do not generate STOP at the end of transaction. */
        UInt8 no_stop:1;
        /** @brief Reserved bit 22. Reads as 0. */
        UInt8 reserved_22:1;
        /** @brief Setting this bit to '1', enables the transmission of the register address programmed in the reg_addr bit-field. */
        UInt8 reg_addr_en:1;
        /** @brief Writing '1' to this bit starts an I2C read or write transaction. */
        UInt8 start_trans:1;
        /** @brief Writing '1' to this bit forces a STOP condition. Setting this bit terminates the data portion of a multi-byte transfer. The current data byte in transfer at the time of bit assertion is completed before the STOP condition is issued. */
        UInt8 stop_trans:1;
        /** @brief Reserved bits 30 to 26. Reads as 0. */
        UInt8 reserved_30_26:5;
        /** @brief When asserted '1', this bit synchronously resets the I2C sensor interface state variables if the I2C sensor interface is enabled. The bit automatically de-asserts when the reset is carried out. */
        UInt8 i2cm_reset:1;
    } bits;
} RegI2CM0Control_t;

#define REG_I2CM0_TIMING_CONFIG0 ((volatile UInt32*)0xf00504) /*  */
#define     I2CM0_TIMING_CONFIG0_SCL_LOW_PER_SHIFT 0u
#define     I2CM0_TIMING_CONFIG0_SCL_LOW_PER_MASK  0xffu
#define GET_I2CM0_TIMING_CONFIG0_SCL_LOW_PER(__reg__)  (((__reg__) & 0xff) >> 0u)
#define SET_I2CM0_TIMING_CONFIG0_SCL_LOW_PER(__val__)  (((__val__) << 0u) & 0xffu)
#define     I2CM0_TIMING_CONFIG0_SCL_HIGH_PER_SHIFT 8u
#define     I2CM0_TIMING_CONFIG0_SCL_HIGH_PER_MASK  0xff00u
#define GET_I2CM0_TIMING_CONFIG0_SCL_HIGH_PER(__reg__)  (((__reg__) & 0xff00) >> 8u)
#define SET_I2CM0_TIMING_CONFIG0_SCL_HIGH_PER(__val__)  (((__val__) << 8u) & 0xff00u)
#define     I2CM0_TIMING_CONFIG0_SCL_STOP_PER_SHIFT 16u
#define     I2CM0_TIMING_CONFIG0_SCL_STOP_PER_MASK  0xff0000u
#define GET_I2CM0_TIMING_CONFIG0_SCL_STOP_PER(__reg__)  (((__reg__) & 0xff0000) >> 16u)
#define SET_I2CM0_TIMING_CONFIG0_SCL_STOP_PER(__val__)  (((__val__) << 16u) & 0xff0000u)
#define     I2CM0_TIMING_CONFIG0_SCL_START_PER_SHIFT 24u
#define     I2CM0_TIMING_CONFIG0_SCL_START_PER_MASK  0xff000000u
#define GET_I2CM0_TIMING_CONFIG0_SCL_START_PER(__reg__)  (((__reg__) & 0xff000000) >> 24u)
#define SET_I2CM0_TIMING_CONFIG0_SCL_START_PER(__val__)  (((__val__) << 24u) & 0xff000000u)

/** @brief Register definition for @ref I2CM0_t.TimingConfig0. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief SCL low period count. */
        UInt8 scl_low_per;
        /** @brief SCL high period count. */
        UInt8 scl_high_per;
        /** @brief SCL stop period count. */
        UInt8 scl_stop_per;
        /** @brief SCL start period count. */
        UInt8 scl_start_per;
    } bits;
} RegI2CM0TimingConfig0_t;

#define REG_I2CM0_TIMING_CONFIG1 ((volatile UInt32*)0xf00508) /*  */
#define     I2CM0_TIMING_CONFIG1_SDA_OUT_HOLD_SHIFT 0u
#define     I2CM0_TIMING_CONFIG1_SDA_OUT_HOLD_MASK  0xffu
#define GET_I2CM0_TIMING_CONFIG1_SDA_OUT_HOLD(__reg__)  (((__reg__) & 0xff) >> 0u)
#define SET_I2CM0_TIMING_CONFIG1_SDA_OUT_HOLD(__val__)  (((__val__) << 0u) & 0xffu)
#define     I2CM0_TIMING_CONFIG1_SDA_LOW_TIME_SHIFT 8u
#define     I2CM0_TIMING_CONFIG1_SDA_LOW_TIME_MASK  0xff00u
#define GET_I2CM0_TIMING_CONFIG1_SDA_LOW_TIME(__reg__)  (((__reg__) & 0xff00) >> 8u)
#define SET_I2CM0_TIMING_CONFIG1_SDA_LOW_TIME(__val__)  (((__val__) << 8u) & 0xff00u)
#define     I2CM0_TIMING_CONFIG1_SCL_RISE_TIME_SHIFT 16u
#define     I2CM0_TIMING_CONFIG1_SCL_RISE_TIME_MASK  0xff0000u
#define GET_I2CM0_TIMING_CONFIG1_SCL_RISE_TIME(__reg__)  (((__reg__) & 0xff0000) >> 16u)
#define SET_I2CM0_TIMING_CONFIG1_SCL_RISE_TIME(__val__)  (((__val__) << 16u) & 0xff0000u)
#define     I2CM0_TIMING_CONFIG1_RESERVED_31_24_SHIFT 24u
#define     I2CM0_TIMING_CONFIG1_RESERVED_31_24_MASK  0xff000000u
#define GET_I2CM0_TIMING_CONFIG1_RESERVED_31_24(__reg__)  (((__reg__) & 0xff000000) >> 24u)
#define SET_I2CM0_TIMING_CONFIG1_RESERVED_31_24(__val__)  (((__val__) << 24u) & 0xff000000u)

/** @brief Register definition for @ref I2CM0_t.TimingConfig1. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief SDA output hold time. */
        UInt8 sda_out_hold;
        /** @brief SDA input low time. */
        UInt8 sda_low_time;
        /** @brief SCL rise time count. */
        UInt8 scl_rise_time;
        /** @brief Reserved bits 31 to 24. Reads as 0. */
        UInt8 reserved_31_24;
    } bits;
} RegI2CM0TimingConfig1_t;

#define REG_I2CM0_CONFIG ((volatile UInt32*)0xf0050c) /*  */
#define     I2CM0_CONFIG_SDA_LOW_DET_DIS_SHIFT 0u
#define     I2CM0_CONFIG_SDA_LOW_DET_DIS_MASK  0x1u
#define GET_I2CM0_CONFIG_SDA_LOW_DET_DIS(__reg__)  (((__reg__) & 0x1) >> 0u)
#define SET_I2CM0_CONFIG_SDA_LOW_DET_DIS(__val__)  (((__val__) << 0u) & 0x1u)
#define     I2CM0_CONFIG_CK_STRETCH_EN_SHIFT 1u
#define     I2CM0_CONFIG_CK_STRETCH_EN_MASK  0x2u
#define GET_I2CM0_CONFIG_CK_STRETCH_EN(__reg__)  (((__reg__) & 0x2) >> 1u)
#define SET_I2CM0_CONFIG_CK_STRETCH_EN(__val__)  (((__val__) << 1u) & 0x2u)
#define     I2CM0_CONFIG_RESERVED_7_2_SHIFT 2u
#define     I2CM0_CONFIG_RESERVED_7_2_MASK  0xfcu
#define GET_I2CM0_CONFIG_RESERVED_7_2(__reg__)  (((__reg__) & 0xfc) >> 2u)
#define SET_I2CM0_CONFIG_RESERVED_7_2(__val__)  (((__val__) << 2u) & 0xfcu)
#define     I2CM0_CONFIG_I2CM_EN_SHIFT 8u
#define     I2CM0_CONFIG_I2CM_EN_MASK  0x100u
#define GET_I2CM0_CONFIG_I2CM_EN(__reg__)  (((__reg__) & 0x100) >> 8u)
#define SET_I2CM0_CONFIG_I2CM_EN(__val__)  (((__val__) << 8u) & 0x100u)
#define     I2CM0_CONFIG_RESERVED_31_9_SHIFT 9u
#define     I2CM0_CONFIG_RESERVED_31_9_MASK  0xfffffe00u
#define GET_I2CM0_CONFIG_RESERVED_31_9(__reg__)  (((__reg__) & 0xfffffe00) >> 9u)
#define SET_I2CM0_CONFIG_RESERVED_31_9(__val__)  (((__val__) << 9u) & 0xfffffe00u)

/** @brief Register definition for @ref I2CM0_t.Config. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief This bit enables recovery from an obstructed bus where the slave holds SDA low. 0: enable recovery. 1: disable recovery. */
        UInt8 sda_low_det_dis:1;
        /** @brief 0: clock stretching is disabled. 1: clock stretching is enabled. */
        UInt8 ck_stretch_en:1;
        /** @brief Reserved bits 7 to 2. Reads as 0. */
        UInt8 reserved_7_2:6;
        /** @brief 0: I2C master is disabled. 1: I2C master is enabled. */
        UInt8 i2cm_en:1;
        /** @brief Reserved bits 31 to 9. Reads as 0. */
        UInt8 reserved_15_9:7;
        /** @brief Reserved bits 31 to 9. Reads as 0. */
        UInt8 reserved_23_16;
        /** @brief Reserved bits 31 to 9. Reads as 0. */
        UInt8 reserved_31_24;
    } bits;
} RegI2CM0Config_t;

#define REG_I2CM0_STATUS ((volatile UInt32*)0xf00510) /*  */
#define     I2CM0_STATUS_BUSY_SHIFT 0u
#define     I2CM0_STATUS_BUSY_MASK  0x1u
#define GET_I2CM0_STATUS_BUSY(__reg__)  (((__reg__) & 0x1) >> 0u)
#define SET_I2CM0_STATUS_BUSY(__val__)  (((__val__) << 0u) & 0x1u)
#define     I2CM0_STATUS_CK_STRETCH_STAT_SHIFT 1u
#define     I2CM0_STATUS_CK_STRETCH_STAT_MASK  0x2u
#define GET_I2CM0_STATUS_CK_STRETCH_STAT(__reg__)  (((__reg__) & 0x2) >> 1u)
#define SET_I2CM0_STATUS_CK_STRETCH_STAT(__val__)  (((__val__) << 1u) & 0x2u)
#define     I2CM0_STATUS_NAK_SENT_SHIFT 2u
#define     I2CM0_STATUS_NAK_SENT_MASK  0x4u
#define GET_I2CM0_STATUS_NAK_SENT(__reg__)  (((__reg__) & 0x4) >> 2u)
#define SET_I2CM0_STATUS_NAK_SENT(__val__)  (((__val__) << 2u) & 0x4u)
#define     I2CM0_STATUS_DATA_TRANS_STAT_SHIFT 3u
#define     I2CM0_STATUS_DATA_TRANS_STAT_MASK  0x8u
#define GET_I2CM0_STATUS_DATA_TRANS_STAT(__reg__)  (((__reg__) & 0x8) >> 3u)
#define SET_I2CM0_STATUS_DATA_TRANS_STAT(__val__)  (((__val__) << 3u) & 0x8u)
#define     I2CM0_STATUS_STOP_STAT_SHIFT 4u
#define     I2CM0_STATUS_STOP_STAT_MASK  0x10u
#define GET_I2CM0_STATUS_STOP_STAT(__reg__)  (((__reg__) & 0x10) >> 4u)
#define SET_I2CM0_STATUS_STOP_STAT(__val__)  (((__val__) << 4u) & 0x10u)
#define     I2CM0_STATUS_NAK_RCVD_SHIFT 5u
#define     I2CM0_STATUS_NAK_RCVD_MASK  0x20u
#define GET_I2CM0_STATUS_NAK_RCVD(__reg__)  (((__reg__) & 0x20) >> 5u)
#define SET_I2CM0_STATUS_NAK_RCVD(__val__)  (((__val__) << 5u) & 0x20u)
#define     I2CM0_STATUS_REG_ADDR_SENT_SHIFT 6u
#define     I2CM0_STATUS_REG_ADDR_SENT_MASK  0x40u
#define GET_I2CM0_STATUS_REG_ADDR_SENT(__reg__)  (((__reg__) & 0x40) >> 6u)
#define SET_I2CM0_STATUS_REG_ADDR_SENT(__val__)  (((__val__) << 6u) & 0x40u)
#define     I2CM0_STATUS_RD_SEQ_ACT_SHIFT 7u
#define     I2CM0_STATUS_RD_SEQ_ACT_MASK  0x80u
#define GET_I2CM0_STATUS_RD_SEQ_ACT(__reg__)  (((__reg__) & 0x80) >> 7u)
#define SET_I2CM0_STATUS_RD_SEQ_ACT(__val__)  (((__val__) << 7u) & 0x80u)
#define     I2CM0_STATUS_WR_SEQ_ACT_SHIFT 8u
#define     I2CM0_STATUS_WR_SEQ_ACT_MASK  0x100u
#define GET_I2CM0_STATUS_WR_SEQ_ACT(__reg__)  (((__reg__) & 0x100) >> 8u)
#define SET_I2CM0_STATUS_WR_SEQ_ACT(__val__)  (((__val__) << 8u) & 0x100u)
#define     I2CM0_STATUS_START_SENT_SHIFT 9u
#define     I2CM0_STATUS_START_SENT_MASK  0x200u
#define GET_I2CM0_STATUS_START_SENT(__reg__)  (((__reg__) & 0x200) >> 9u)
#define SET_I2CM0_STATUS_START_SENT(__val__)  (((__val__) << 9u) & 0x200u)
#define     I2CM0_STATUS_SDA_LOW_STAT_SHIFT 10u
#define     I2CM0_STATUS_SDA_LOW_STAT_MASK  0x400u
#define GET_I2CM0_STATUS_SDA_LOW_STAT(__reg__)  (((__reg__) & 0x400) >> 10u)
#define SET_I2CM0_STATUS_SDA_LOW_STAT(__val__)  (((__val__) << 10u) & 0x400u)
#define     I2CM0_STATUS_START_TRANS_STAT_SHIFT 11u
#define     I2CM0_STATUS_START_TRANS_STAT_MASK  0x800u
#define GET_I2CM0_STATUS_START_TRANS_STAT(__reg__)  (((__reg__) & 0x800) >> 11u)
#define SET_I2CM0_STATUS_START_TRANS_STAT(__val__)  (((__val__) << 11u) & 0x800u)
#define     I2CM0_STATUS_STOP_TRANS_STAT_SHIFT 12u
#define     I2CM0_STATUS_STOP_TRANS_STAT_MASK  0x1000u
#define GET_I2CM0_STATUS_STOP_TRANS_STAT(__reg__)  (((__reg__) & 0x1000) >> 12u)
#define SET_I2CM0_STATUS_STOP_TRANS_STAT(__val__)  (((__val__) << 12u) & 0x1000u)
#define     I2CM0_STATUS_PAUSE_STAT_SHIFT 13u
#define     I2CM0_STATUS_PAUSE_STAT_MASK  0x2000u
#define GET_I2CM0_STATUS_PAUSE_STAT(__reg__)  (((__reg__) & 0x2000) >> 13u)
#define SET_I2CM0_STATUS_PAUSE_STAT(__val__)  (((__val__) << 13u) & 0x2000u)
#define     I2CM0_STATUS_RESERVED_15_14_SHIFT 14u
#define     I2CM0_STATUS_RESERVED_15_14_MASK  0xc000u
#define GET_I2CM0_STATUS_RESERVED_15_14(__reg__)  (((__reg__) & 0xc000) >> 14u)
#define SET_I2CM0_STATUS_RESERVED_15_14(__val__)  (((__val__) << 14u) & 0xc000u)
#define     I2CM0_STATUS_TRANS_LENGTH_SHIFT 16u
#define     I2CM0_STATUS_TRANS_LENGTH_MASK  0x1f0000u
#define GET_I2CM0_STATUS_TRANS_LENGTH(__reg__)  (((__reg__) & 0x1f0000) >> 16u)
#define SET_I2CM0_STATUS_TRANS_LENGTH(__val__)  (((__val__) << 16u) & 0x1f0000u)
#define     I2CM0_STATUS_RESERVED_31_21_SHIFT 21u
#define     I2CM0_STATUS_RESERVED_31_21_MASK  0xffe00000u
#define GET_I2CM0_STATUS_RESERVED_31_21(__reg__)  (((__reg__) & 0xffe00000) >> 21u)
#define SET_I2CM0_STATUS_RESERVED_31_21(__val__)  (((__val__) << 21u) & 0xffe00000u)

/** @brief Register definition for @ref I2CM0_t.Status. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief When '1', this bit indicates that an I2C transaction is in progress. This bit asserts when a START condition is issued and de-asserts when a STOP condition is issued. */
        UInt8 busy:1;
        /** @brief When '1', this bit indicates that clock stretching occurred during the I2C transaction. The bit is cleared every time an I2C transaction is started. */
        UInt8 ck_stretch_stat:1;
        /** @brief When '1', this bit indicates that a NAK has been issued to an attached slave. The bit is updated after each byte received from the slave and is cleared every time an I2C transaction is started. */
        UInt8 nak_sent:1;
        /** @brief 0: data transfer has not finished. 1: data transfer has finished. */
        UInt8 data_trans_stat:1;
        /** @brief When '1', this bit indicates that a STOP condition has been generated. */
        UInt8 stop_stat:1;
        /** @brief When '1', this bit indicates that a NAK has been received from the attached slave. The bit is updated after each byte transferred to the slave and is cleared every time an I2C transaction is started. */
        UInt8 nak_rcvd:1;
        /** @brief When '1', this bit indicates that the register address has been sent. */
        UInt8 reg_addr_sent:1;
        /** @brief When '1', this bit indicates that the slave device address has been sent and a read transaction has been activated. */
        UInt8 rd_seq_act:1;
        /** @brief When '1', this bit indicates that the slave device address has been sent and a write transaction has been activated. */
        UInt8 wr_seq_act:1;
        /** @brief When '1', this bit indicates that a START condition has been generated. */
        UInt8 start_sent:1;
        /** @brief When '1', this bit indicates that the SDA line is low before a START condition (i.e that the SDA line is obstructed by a slave on the I2C bus. */
        UInt8 sda_low_stat:1;
        /** @brief When '1', this bit indicates that a START condition has been issued. The bit is cleared every time an I2C transaction is started. */
        UInt8 start_trans_stat:1;
        /** @brief When '1', this bit indicates that a STOP condition has been issued. The bit is cleared every time an I2C transaction is started. */
        UInt8 stop_trans_stat:1;
        /** @brief Last transaction STOP status: 1: no STOP generated, transaction paused. */
        UInt8 pause_stat:1;
        /** @brief Reserved bits 15 to 14. Reads as 0. */
        UInt8 reserved_15_14:2;
        /** @brief This bit-field shows the number of bytes transferred during a read or write transaction. The bit-field is updated after each byte transacted and cleared every time an I2C transaction is started. */
        UInt16 trans_length:5;
        /** @brief Reserved bits 31 to 21. Reads as 0. */
        UInt16 reserved_31_21:11;
    } bits;
} RegI2CM0Status_t;

#define REG_I2CM0_TX_BUFFER0 ((volatile UInt32*)0xf00514) /* This register holds the data to be transmitted during write transactions. */
/** @brief Register definition for @ref I2CM0_t.TxBuffer0. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];
} RegI2CM0TxBuffer0_t;

#define REG_I2CM0_TX_BUFFER1 ((volatile UInt32*)0xf00518) /* This register holds the data to be transmitted during write transactions. */
/** @brief Register definition for @ref I2CM0_t.TxBuffer1. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];
} RegI2CM0TxBuffer1_t;

#define REG_I2CM0_TX_BUFFER2 ((volatile UInt32*)0xf0051c) /* This register holds the data to be transmitted during write transactions. */
/** @brief Register definition for @ref I2CM0_t.TxBuffer2. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];
} RegI2CM0TxBuffer2_t;

#define REG_I2CM0_TX_BUFFER3 ((volatile UInt32*)0xf00520) /* This register holds the data to be transmitted during write transactions. */
/** @brief Register definition for @ref I2CM0_t.TxBuffer3. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];
} RegI2CM0TxBuffer3_t;

#define REG_I2CM0_RX_BUFFER0 ((volatile UInt32*)0xf00524) /* This register holds the data received during read transactions. */
/** @brief Register definition for @ref I2CM0_t.RxBuffer0. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];
} RegI2CM0RxBuffer0_t;

#define REG_I2CM0_RX_BUFFER1 ((volatile UInt32*)0xf00528) /* This register holds the data received during read transactions. */
/** @brief Register definition for @ref I2CM0_t.RxBuffer1. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];
} RegI2CM0RxBuffer1_t;

#define REG_I2CM0_RX_BUFFER2 ((volatile UInt32*)0xf0052c) /* This register holds the data received during read transactions. */
/** @brief Register definition for @ref I2CM0_t.RxBuffer2. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];
} RegI2CM0RxBuffer2_t;

#define REG_I2CM0_RX_BUFFER3 ((volatile UInt32*)0xf00530) /* This register holds the data received during read transactions. */
/** @brief Register definition for @ref I2CM0_t.RxBuffer3. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];
} RegI2CM0RxBuffer3_t;

/** @brief Component definition for @ref I2CM0. */
typedef struct {
    /** @brief  */
    RegI2CM0Control_t Control;

    /** @brief  */
    RegI2CM0TimingConfig0_t TimingConfig0;

    /** @brief  */
    RegI2CM0TimingConfig1_t TimingConfig1;

    /** @brief  */
    RegI2CM0Config_t Config;

    /** @brief  */
    RegI2CM0Status_t Status;

    /** @brief This register holds the data to be transmitted during write transactions. */
    RegI2CM0TxBuffer0_t TxBuffer0;

    /** @brief This register holds the data to be transmitted during write transactions. */
    RegI2CM0TxBuffer1_t TxBuffer1;

    /** @brief This register holds the data to be transmitted during write transactions. */
    RegI2CM0TxBuffer2_t TxBuffer2;

    /** @brief This register holds the data to be transmitted during write transactions. */
    RegI2CM0TxBuffer3_t TxBuffer3;

    /** @brief This register holds the data received during read transactions. */
    RegI2CM0RxBuffer0_t RxBuffer0;

    /** @brief This register holds the data received during read transactions. */
    RegI2CM0RxBuffer1_t RxBuffer1;

    /** @brief This register holds the data received during read transactions. */
    RegI2CM0RxBuffer2_t RxBuffer2;

    /** @brief This register holds the data received during read transactions. */
    RegI2CM0RxBuffer3_t RxBuffer3;

} I2CM0_t;

/** @brief  */
extern volatile I2CM0_t I2CM0;


#define REG_I2CM1_CONTROL ((volatile UInt32*)0xf00600) /*  */
#define     I2CM1_CONTROL_REG_ADDR_SHIFT 0u
#define     I2CM1_CONTROL_REG_ADDR_MASK  0xffu
#define GET_I2CM1_CONTROL_REG_ADDR(__reg__)  (((__reg__) & 0xff) >> 0u)
#define SET_I2CM1_CONTROL_REG_ADDR(__val__)  (((__val__) << 0u) & 0xffu)
#define     I2CM1_CONTROL_SLV_ADDR_SHIFT 8u
#define     I2CM1_CONTROL_SLV_ADDR_MASK  0x7f00u
#define GET_I2CM1_CONTROL_SLV_ADDR(__reg__)  (((__reg__) & 0x7f00) >> 8u)
#define SET_I2CM1_CONTROL_SLV_ADDR(__val__)  (((__val__) << 8u) & 0x7f00u)
#define     I2CM1_CONTROL_RW_SEL_SHIFT 15u
#define     I2CM1_CONTROL_RW_SEL_MASK  0x8000u
#define GET_I2CM1_CONTROL_RW_SEL(__reg__)  (((__reg__) & 0x8000) >> 15u)
#define SET_I2CM1_CONTROL_RW_SEL(__val__)  (((__val__) << 15u) & 0x8000u)
#define     I2CM1_CONTROL_TRANS_LENGTH_SHIFT 16u
#define     I2CM1_CONTROL_TRANS_LENGTH_MASK  0x1f0000u
#define GET_I2CM1_CONTROL_TRANS_LENGTH(__reg__)  (((__reg__) & 0x1f0000) >> 16u)
#define SET_I2CM1_CONTROL_TRANS_LENGTH(__val__)  (((__val__) << 16u) & 0x1f0000u)
#define     I2CM1_CONTROL_NO_STOP_SHIFT 21u
#define     I2CM1_CONTROL_NO_STOP_MASK  0x200000u
#define GET_I2CM1_CONTROL_NO_STOP(__reg__)  (((__reg__) & 0x200000) >> 21u)
#define SET_I2CM1_CONTROL_NO_STOP(__val__)  (((__val__) << 21u) & 0x200000u)
#define     I2CM1_CONTROL_RESERVED_22_SHIFT 22u
#define     I2CM1_CONTROL_RESERVED_22_MASK  0x400000u
#define GET_I2CM1_CONTROL_RESERVED_22(__reg__)  (((__reg__) & 0x400000) >> 22u)
#define SET_I2CM1_CONTROL_RESERVED_22(__val__)  (((__val__) << 22u) & 0x400000u)
#define     I2CM1_CONTROL_REG_ADDR_EN_SHIFT 23u
#define     I2CM1_CONTROL_REG_ADDR_EN_MASK  0x800000u
#define GET_I2CM1_CONTROL_REG_ADDR_EN(__reg__)  (((__reg__) & 0x800000) >> 23u)
#define SET_I2CM1_CONTROL_REG_ADDR_EN(__val__)  (((__val__) << 23u) & 0x800000u)
#define     I2CM1_CONTROL_START_TRANS_SHIFT 24u
#define     I2CM1_CONTROL_START_TRANS_MASK  0x1000000u
#define GET_I2CM1_CONTROL_START_TRANS(__reg__)  (((__reg__) & 0x1000000) >> 24u)
#define SET_I2CM1_CONTROL_START_TRANS(__val__)  (((__val__) << 24u) & 0x1000000u)
#define     I2CM1_CONTROL_STOP_TRANS_SHIFT 25u
#define     I2CM1_CONTROL_STOP_TRANS_MASK  0x2000000u
#define GET_I2CM1_CONTROL_STOP_TRANS(__reg__)  (((__reg__) & 0x2000000) >> 25u)
#define SET_I2CM1_CONTROL_STOP_TRANS(__val__)  (((__val__) << 25u) & 0x2000000u)
#define     I2CM1_CONTROL_RESERVED_30_26_SHIFT 26u
#define     I2CM1_CONTROL_RESERVED_30_26_MASK  0x7c000000u
#define GET_I2CM1_CONTROL_RESERVED_30_26(__reg__)  (((__reg__) & 0x7c000000) >> 26u)
#define SET_I2CM1_CONTROL_RESERVED_30_26(__val__)  (((__val__) << 26u) & 0x7c000000u)
#define     I2CM1_CONTROL_I2CM_RESET_SHIFT 31u
#define     I2CM1_CONTROL_I2CM_RESET_MASK  0x80000000u
#define GET_I2CM1_CONTROL_I2CM_RESET(__reg__)  (((__reg__) & 0x80000000) >> 31u)
#define SET_I2CM1_CONTROL_I2CM_RESET(__val__)  (((__val__) << 31u) & 0x80000000u)

/** @brief Register definition for @ref I2CM1_t.Control. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief This bit-field defines the register address to be transmitted during an I2C read or write transaction when register address transmission is enabled */
        UInt8 reg_addr;
        /** @brief This bit-field defines the slave device address to be transmitted during an I2C read or write transaction. The slave address is always transmitted immediately after a START condition has been issued. */
        UInt8 slv_addr:7;
        /** @brief This bit defines the I2C transaction type: 0: write. 1: I2C read. The transaction type is always transmitted immediately after the slave device address. */
        UInt8 rw_sel:1;
        /** @brief This bit-field defines the number of data bytes to be transacted during an I2C read or write. Valid values are 0x00 to 0x10 (16 bytes). */
        UInt8 trans_length:5;
        /** @brief Do not generate STOP at the end of transaction. */
        UInt8 no_stop:1;
        /** @brief Reserved bit 22. Reads as 0. */
        UInt8 reserved_22:1;
        /** @brief Setting this bit to '1', enables the transmission of the register address programmed in the reg_addr bit-field. */
        UInt8 reg_addr_en:1;
        /** @brief Writing '1' to this bit starts an I2C read or write transaction. */
        UInt8 start_trans:1;
        /** @brief Writing '1' to this bit forces a STOP condition. Setting this bit terminates the data portion of a multi-byte transfer. The current data byte in transfer at the time of bit assertion is completed before the STOP condition is issued. */
        UInt8 stop_trans:1;
        /** @brief Reserved bits 30 to 26. Reads as 0. */
        UInt8 reserved_30_26:5;
        /** @brief When asserted '1', this bit synchronously resets the I2C sensor interface state variables if the I2C sensor interface is enabled. The bit automatically de-asserts when the reset is carried out. */
        UInt8 i2cm_reset:1;
    } bits;
} RegI2CM1Control_t;

#define REG_I2CM1_TIMING_CONFIG0 ((volatile UInt32*)0xf00604) /*  */
#define     I2CM1_TIMING_CONFIG0_SCL_LOW_PER_SHIFT 0u
#define     I2CM1_TIMING_CONFIG0_SCL_LOW_PER_MASK  0xffu
#define GET_I2CM1_TIMING_CONFIG0_SCL_LOW_PER(__reg__)  (((__reg__) & 0xff) >> 0u)
#define SET_I2CM1_TIMING_CONFIG0_SCL_LOW_PER(__val__)  (((__val__) << 0u) & 0xffu)
#define     I2CM1_TIMING_CONFIG0_SCL_HIGH_PER_SHIFT 8u
#define     I2CM1_TIMING_CONFIG0_SCL_HIGH_PER_MASK  0xff00u
#define GET_I2CM1_TIMING_CONFIG0_SCL_HIGH_PER(__reg__)  (((__reg__) & 0xff00) >> 8u)
#define SET_I2CM1_TIMING_CONFIG0_SCL_HIGH_PER(__val__)  (((__val__) << 8u) & 0xff00u)
#define     I2CM1_TIMING_CONFIG0_SCL_STOP_PER_SHIFT 16u
#define     I2CM1_TIMING_CONFIG0_SCL_STOP_PER_MASK  0xff0000u
#define GET_I2CM1_TIMING_CONFIG0_SCL_STOP_PER(__reg__)  (((__reg__) & 0xff0000) >> 16u)
#define SET_I2CM1_TIMING_CONFIG0_SCL_STOP_PER(__val__)  (((__val__) << 16u) & 0xff0000u)
#define     I2CM1_TIMING_CONFIG0_SCL_START_PER_SHIFT 24u
#define     I2CM1_TIMING_CONFIG0_SCL_START_PER_MASK  0xff000000u
#define GET_I2CM1_TIMING_CONFIG0_SCL_START_PER(__reg__)  (((__reg__) & 0xff000000) >> 24u)
#define SET_I2CM1_TIMING_CONFIG0_SCL_START_PER(__val__)  (((__val__) << 24u) & 0xff000000u)

/** @brief Register definition for @ref I2CM1_t.TimingConfig0. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief SCL low period count. */
        UInt8 scl_low_per;
        /** @brief SCL high period count. */
        UInt8 scl_high_per;
        /** @brief SCL stop period count. */
        UInt8 scl_stop_per;
        /** @brief SCL start period count. */
        UInt8 scl_start_per;
    } bits;
} RegI2CM1TimingConfig0_t;

#define REG_I2CM1_TIMING_CONFIG1 ((volatile UInt32*)0xf00608) /*  */
#define     I2CM1_TIMING_CONFIG1_SDA_OUT_HOLD_SHIFT 0u
#define     I2CM1_TIMING_CONFIG1_SDA_OUT_HOLD_MASK  0xffu
#define GET_I2CM1_TIMING_CONFIG1_SDA_OUT_HOLD(__reg__)  (((__reg__) & 0xff) >> 0u)
#define SET_I2CM1_TIMING_CONFIG1_SDA_OUT_HOLD(__val__)  (((__val__) << 0u) & 0xffu)
#define     I2CM1_TIMING_CONFIG1_SDA_LOW_TIME_SHIFT 8u
#define     I2CM1_TIMING_CONFIG1_SDA_LOW_TIME_MASK  0xff00u
#define GET_I2CM1_TIMING_CONFIG1_SDA_LOW_TIME(__reg__)  (((__reg__) & 0xff00) >> 8u)
#define SET_I2CM1_TIMING_CONFIG1_SDA_LOW_TIME(__val__)  (((__val__) << 8u) & 0xff00u)
#define     I2CM1_TIMING_CONFIG1_SCL_RISE_TIME_SHIFT 16u
#define     I2CM1_TIMING_CONFIG1_SCL_RISE_TIME_MASK  0xff0000u
#define GET_I2CM1_TIMING_CONFIG1_SCL_RISE_TIME(__reg__)  (((__reg__) & 0xff0000) >> 16u)
#define SET_I2CM1_TIMING_CONFIG1_SCL_RISE_TIME(__val__)  (((__val__) << 16u) & 0xff0000u)
#define     I2CM1_TIMING_CONFIG1_RESERVED_31_24_SHIFT 24u
#define     I2CM1_TIMING_CONFIG1_RESERVED_31_24_MASK  0xff000000u
#define GET_I2CM1_TIMING_CONFIG1_RESERVED_31_24(__reg__)  (((__reg__) & 0xff000000) >> 24u)
#define SET_I2CM1_TIMING_CONFIG1_RESERVED_31_24(__val__)  (((__val__) << 24u) & 0xff000000u)

/** @brief Register definition for @ref I2CM1_t.TimingConfig1. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief SDA output hold time. */
        UInt8 sda_out_hold;
        /** @brief SDA input low time. */
        UInt8 sda_low_time;
        /** @brief SCL rise time count. */
        UInt8 scl_rise_time;
        /** @brief Reserved bits 31 to 24. Reads as 0. */
        UInt8 reserved_31_24;
    } bits;
} RegI2CM1TimingConfig1_t;

#define REG_I2CM1_CONFIG ((volatile UInt32*)0xf0060c) /*  */
#define     I2CM1_CONFIG_SDA_LOW_DET_DIS_SHIFT 0u
#define     I2CM1_CONFIG_SDA_LOW_DET_DIS_MASK  0x1u
#define GET_I2CM1_CONFIG_SDA_LOW_DET_DIS(__reg__)  (((__reg__) & 0x1) >> 0u)
#define SET_I2CM1_CONFIG_SDA_LOW_DET_DIS(__val__)  (((__val__) << 0u) & 0x1u)
#define     I2CM1_CONFIG_CK_STRETCH_EN_SHIFT 1u
#define     I2CM1_CONFIG_CK_STRETCH_EN_MASK  0x2u
#define GET_I2CM1_CONFIG_CK_STRETCH_EN(__reg__)  (((__reg__) & 0x2) >> 1u)
#define SET_I2CM1_CONFIG_CK_STRETCH_EN(__val__)  (((__val__) << 1u) & 0x2u)
#define     I2CM1_CONFIG_RESERVED_7_2_SHIFT 2u
#define     I2CM1_CONFIG_RESERVED_7_2_MASK  0xfcu
#define GET_I2CM1_CONFIG_RESERVED_7_2(__reg__)  (((__reg__) & 0xfc) >> 2u)
#define SET_I2CM1_CONFIG_RESERVED_7_2(__val__)  (((__val__) << 2u) & 0xfcu)
#define     I2CM1_CONFIG_I2CM_EN_SHIFT 8u
#define     I2CM1_CONFIG_I2CM_EN_MASK  0x100u
#define GET_I2CM1_CONFIG_I2CM_EN(__reg__)  (((__reg__) & 0x100) >> 8u)
#define SET_I2CM1_CONFIG_I2CM_EN(__val__)  (((__val__) << 8u) & 0x100u)
#define     I2CM1_CONFIG_RESERVED_31_9_SHIFT 9u
#define     I2CM1_CONFIG_RESERVED_31_9_MASK  0xfffffe00u
#define GET_I2CM1_CONFIG_RESERVED_31_9(__reg__)  (((__reg__) & 0xfffffe00) >> 9u)
#define SET_I2CM1_CONFIG_RESERVED_31_9(__val__)  (((__val__) << 9u) & 0xfffffe00u)

/** @brief Register definition for @ref I2CM1_t.Config. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief This bit enables recovery from an obstructed bus where the slave holds SDA low. 0: enable recovery. 1: disable recovery. */
        UInt8 sda_low_det_dis:1;
        /** @brief 0: clock stretching is disabled. 1: clock stretching is enabled. */
        UInt8 ck_stretch_en:1;
        /** @brief Reserved bits 7 to 2. Reads as 0. */
        UInt8 reserved_7_2:6;
        /** @brief 0: I2C master is disabled. 1: I2C master is enabled. */
        UInt8 i2cm_en:1;
        /** @brief Reserved bits 31 to 9. Reads as 0. */
        UInt8 reserved_15_9:7;
        /** @brief Reserved bits 31 to 9. Reads as 0. */
        UInt8 reserved_23_16;
        /** @brief Reserved bits 31 to 9. Reads as 0. */
        UInt8 reserved_31_24;
    } bits;
} RegI2CM1Config_t;

#define REG_I2CM1_STATUS ((volatile UInt32*)0xf00610) /*  */
#define     I2CM1_STATUS_BUSY_SHIFT 0u
#define     I2CM1_STATUS_BUSY_MASK  0x1u
#define GET_I2CM1_STATUS_BUSY(__reg__)  (((__reg__) & 0x1) >> 0u)
#define SET_I2CM1_STATUS_BUSY(__val__)  (((__val__) << 0u) & 0x1u)
#define     I2CM1_STATUS_CK_STRETCH_STAT_SHIFT 1u
#define     I2CM1_STATUS_CK_STRETCH_STAT_MASK  0x2u
#define GET_I2CM1_STATUS_CK_STRETCH_STAT(__reg__)  (((__reg__) & 0x2) >> 1u)
#define SET_I2CM1_STATUS_CK_STRETCH_STAT(__val__)  (((__val__) << 1u) & 0x2u)
#define     I2CM1_STATUS_NAK_SENT_SHIFT 2u
#define     I2CM1_STATUS_NAK_SENT_MASK  0x4u
#define GET_I2CM1_STATUS_NAK_SENT(__reg__)  (((__reg__) & 0x4) >> 2u)
#define SET_I2CM1_STATUS_NAK_SENT(__val__)  (((__val__) << 2u) & 0x4u)
#define     I2CM1_STATUS_DATA_TRANS_STAT_SHIFT 3u
#define     I2CM1_STATUS_DATA_TRANS_STAT_MASK  0x8u
#define GET_I2CM1_STATUS_DATA_TRANS_STAT(__reg__)  (((__reg__) & 0x8) >> 3u)
#define SET_I2CM1_STATUS_DATA_TRANS_STAT(__val__)  (((__val__) << 3u) & 0x8u)
#define     I2CM1_STATUS_STOP_STAT_SHIFT 4u
#define     I2CM1_STATUS_STOP_STAT_MASK  0x10u
#define GET_I2CM1_STATUS_STOP_STAT(__reg__)  (((__reg__) & 0x10) >> 4u)
#define SET_I2CM1_STATUS_STOP_STAT(__val__)  (((__val__) << 4u) & 0x10u)
#define     I2CM1_STATUS_NAK_RCVD_SHIFT 5u
#define     I2CM1_STATUS_NAK_RCVD_MASK  0x20u
#define GET_I2CM1_STATUS_NAK_RCVD(__reg__)  (((__reg__) & 0x20) >> 5u)
#define SET_I2CM1_STATUS_NAK_RCVD(__val__)  (((__val__) << 5u) & 0x20u)
#define     I2CM1_STATUS_REG_ADDR_SENT_SHIFT 6u
#define     I2CM1_STATUS_REG_ADDR_SENT_MASK  0x40u
#define GET_I2CM1_STATUS_REG_ADDR_SENT(__reg__)  (((__reg__) & 0x40) >> 6u)
#define SET_I2CM1_STATUS_REG_ADDR_SENT(__val__)  (((__val__) << 6u) & 0x40u)
#define     I2CM1_STATUS_RD_SEQ_ACT_SHIFT 7u
#define     I2CM1_STATUS_RD_SEQ_ACT_MASK  0x80u
#define GET_I2CM1_STATUS_RD_SEQ_ACT(__reg__)  (((__reg__) & 0x80) >> 7u)
#define SET_I2CM1_STATUS_RD_SEQ_ACT(__val__)  (((__val__) << 7u) & 0x80u)
#define     I2CM1_STATUS_WR_SEQ_ACT_SHIFT 8u
#define     I2CM1_STATUS_WR_SEQ_ACT_MASK  0x100u
#define GET_I2CM1_STATUS_WR_SEQ_ACT(__reg__)  (((__reg__) & 0x100) >> 8u)
#define SET_I2CM1_STATUS_WR_SEQ_ACT(__val__)  (((__val__) << 8u) & 0x100u)
#define     I2CM1_STATUS_START_SENT_SHIFT 9u
#define     I2CM1_STATUS_START_SENT_MASK  0x200u
#define GET_I2CM1_STATUS_START_SENT(__reg__)  (((__reg__) & 0x200) >> 9u)
#define SET_I2CM1_STATUS_START_SENT(__val__)  (((__val__) << 9u) & 0x200u)
#define     I2CM1_STATUS_SDA_LOW_STAT_SHIFT 10u
#define     I2CM1_STATUS_SDA_LOW_STAT_MASK  0x400u
#define GET_I2CM1_STATUS_SDA_LOW_STAT(__reg__)  (((__reg__) & 0x400) >> 10u)
#define SET_I2CM1_STATUS_SDA_LOW_STAT(__val__)  (((__val__) << 10u) & 0x400u)
#define     I2CM1_STATUS_START_TRANS_STAT_SHIFT 11u
#define     I2CM1_STATUS_START_TRANS_STAT_MASK  0x800u
#define GET_I2CM1_STATUS_START_TRANS_STAT(__reg__)  (((__reg__) & 0x800) >> 11u)
#define SET_I2CM1_STATUS_START_TRANS_STAT(__val__)  (((__val__) << 11u) & 0x800u)
#define     I2CM1_STATUS_STOP_TRANS_STAT_SHIFT 12u
#define     I2CM1_STATUS_STOP_TRANS_STAT_MASK  0x1000u
#define GET_I2CM1_STATUS_STOP_TRANS_STAT(__reg__)  (((__reg__) & 0x1000) >> 12u)
#define SET_I2CM1_STATUS_STOP_TRANS_STAT(__val__)  (((__val__) << 12u) & 0x1000u)
#define     I2CM1_STATUS_PAUSE_STAT_SHIFT 13u
#define     I2CM1_STATUS_PAUSE_STAT_MASK  0x2000u
#define GET_I2CM1_STATUS_PAUSE_STAT(__reg__)  (((__reg__) & 0x2000) >> 13u)
#define SET_I2CM1_STATUS_PAUSE_STAT(__val__)  (((__val__) << 13u) & 0x2000u)
#define     I2CM1_STATUS_RESERVED_15_14_SHIFT 14u
#define     I2CM1_STATUS_RESERVED_15_14_MASK  0xc000u
#define GET_I2CM1_STATUS_RESERVED_15_14(__reg__)  (((__reg__) & 0xc000) >> 14u)
#define SET_I2CM1_STATUS_RESERVED_15_14(__val__)  (((__val__) << 14u) & 0xc000u)
#define     I2CM1_STATUS_TRANS_LENGTH_SHIFT 16u
#define     I2CM1_STATUS_TRANS_LENGTH_MASK  0x1f0000u
#define GET_I2CM1_STATUS_TRANS_LENGTH(__reg__)  (((__reg__) & 0x1f0000) >> 16u)
#define SET_I2CM1_STATUS_TRANS_LENGTH(__val__)  (((__val__) << 16u) & 0x1f0000u)
#define     I2CM1_STATUS_RESERVED_31_21_SHIFT 21u
#define     I2CM1_STATUS_RESERVED_31_21_MASK  0xffe00000u
#define GET_I2CM1_STATUS_RESERVED_31_21(__reg__)  (((__reg__) & 0xffe00000) >> 21u)
#define SET_I2CM1_STATUS_RESERVED_31_21(__val__)  (((__val__) << 21u) & 0xffe00000u)

/** @brief Register definition for @ref I2CM1_t.Status. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief When '1', this bit indicates that an I2C transaction is in progress. This bit asserts when a START condition is issued and de-asserts when a STOP condition is issued. */
        UInt8 busy:1;
        /** @brief When '1', this bit indicates that clock stretching occurred during the I2C transaction. The bit is cleared every time an I2C transaction is started. */
        UInt8 ck_stretch_stat:1;
        /** @brief When '1', this bit indicates that a NAK has been issued to an attached slave. The bit is updated after each byte received from the slave and is cleared every time an I2C transaction is started. */
        UInt8 nak_sent:1;
        /** @brief 0: data transfer has not finished. 1: data transfer has finished. */
        UInt8 data_trans_stat:1;
        /** @brief When '1', this bit indicates that a STOP condition has been generated. */
        UInt8 stop_stat:1;
        /** @brief When '1', this bit indicates that a NAK has been received from the attached slave. The bit is updated after each byte transferred to the slave and is cleared every time an I2C transaction is started. */
        UInt8 nak_rcvd:1;
        /** @brief When '1', this bit indicates that the register address has been sent. */
        UInt8 reg_addr_sent:1;
        /** @brief When '1', this bit indicates that the slave device address has been sent and a read transaction has been activated. */
        UInt8 rd_seq_act:1;
        /** @brief When '1', this bit indicates that the slave device address has been sent and a write transaction has been activated. */
        UInt8 wr_seq_act:1;
        /** @brief When '1', this bit indicates that a START condition has been generated. */
        UInt8 start_sent:1;
        /** @brief When '1', this bit indicates that the SDA line is low before a START condition (i.e that the SDA line is obstructed by a slave on the I2C bus. */
        UInt8 sda_low_stat:1;
        /** @brief When '1', this bit indicates that a START condition has been issued. The bit is cleared every time an I2C transaction is started. */
        UInt8 start_trans_stat:1;
        /** @brief When '1', this bit indicates that a STOP condition has been issued. The bit is cleared every time an I2C transaction is started. */
        UInt8 stop_trans_stat:1;
        /** @brief Last transaction STOP status: 1: no STOP generated, transaction paused. */
        UInt8 pause_stat:1;
        /** @brief Reserved bits 15 to 14. Reads as 0. */
        UInt8 reserved_15_14:2;
        /** @brief This bit-field shows the number of bytes transferred during a read or write transaction. The bit-field is updated after each byte transacted and cleared every time an I2C transaction is started. */
        UInt16 trans_length:5;
        /** @brief Reserved bits 31 to 21. Reads as 0. */
        UInt16 reserved_31_21:11;
    } bits;
} RegI2CM1Status_t;

#define REG_I2CM1_TX_BUFFER0 ((volatile UInt32*)0xf00614) /* This register holds the data to be transmitted during write transactions. */
/** @brief Register definition for @ref I2CM1_t.TxBuffer0. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];
} RegI2CM1TxBuffer0_t;

#define REG_I2CM1_TX_BUFFER1 ((volatile UInt32*)0xf00618) /* This register holds the data to be transmitted during write transactions. */
/** @brief Register definition for @ref I2CM1_t.TxBuffer1. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];
} RegI2CM1TxBuffer1_t;

#define REG_I2CM1_TX_BUFFER2 ((volatile UInt32*)0xf0061c) /* This register holds the data to be transmitted during write transactions. */
/** @brief Register definition for @ref I2CM1_t.TxBuffer2. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];
} RegI2CM1TxBuffer2_t;

#define REG_I2CM1_TX_BUFFER3 ((volatile UInt32*)0xf00620) /* This register holds the data to be transmitted during write transactions. */
/** @brief Register definition for @ref I2CM1_t.TxBuffer3. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];
} RegI2CM1TxBuffer3_t;

#define REG_I2CM1_RX_BUFFER0 ((volatile UInt32*)0xf00624) /* This register holds the data received during read transactions. */
/** @brief Register definition for @ref I2CM1_t.RxBuffer0. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];
} RegI2CM1RxBuffer0_t;

#define REG_I2CM1_RX_BUFFER1 ((volatile UInt32*)0xf00628) /* This register holds the data received during read transactions. */
/** @brief Register definition for @ref I2CM1_t.RxBuffer1. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];
} RegI2CM1RxBuffer1_t;

#define REG_I2CM1_RX_BUFFER2 ((volatile UInt32*)0xf0062c) /* This register holds the data received during read transactions. */
/** @brief Register definition for @ref I2CM1_t.RxBuffer2. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];
} RegI2CM1RxBuffer2_t;

#define REG_I2CM1_RX_BUFFER3 ((volatile UInt32*)0xf00630) /* This register holds the data received during read transactions. */
/** @brief Register definition for @ref I2CM1_t.RxBuffer3. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];
} RegI2CM1RxBuffer3_t;

/** @brief Component definition for @ref I2CM1. */
typedef struct {
    /** @brief  */
    RegI2CM1Control_t Control;

    /** @brief  */
    RegI2CM1TimingConfig0_t TimingConfig0;

    /** @brief  */
    RegI2CM1TimingConfig1_t TimingConfig1;

    /** @brief  */
    RegI2CM1Config_t Config;

    /** @brief  */
    RegI2CM1Status_t Status;

    /** @brief This register holds the data to be transmitted during write transactions. */
    RegI2CM1TxBuffer0_t TxBuffer0;

    /** @brief This register holds the data to be transmitted during write transactions. */
    RegI2CM1TxBuffer1_t TxBuffer1;

    /** @brief This register holds the data to be transmitted during write transactions. */
    RegI2CM1TxBuffer2_t TxBuffer2;

    /** @brief This register holds the data to be transmitted during write transactions. */
    RegI2CM1TxBuffer3_t TxBuffer3;

    /** @brief This register holds the data received during read transactions. */
    RegI2CM1RxBuffer0_t RxBuffer0;

    /** @brief This register holds the data received during read transactions. */
    RegI2CM1RxBuffer1_t RxBuffer1;

    /** @brief This register holds the data received during read transactions. */
    RegI2CM1RxBuffer2_t RxBuffer2;

    /** @brief This register holds the data received during read transactions. */
    RegI2CM1RxBuffer3_t RxBuffer3;

} I2CM1_t;

/** @brief  */
extern volatile I2CM1_t I2CM1;


#endif /* !I2CM_H */

/** @} */
