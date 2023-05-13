////////////////////////////////////////////////////////////////////////////////
///
/// @file       common/7189/includes/em7189.h
///
/// @project    EM7189
///
/// @brief      em7189 registers
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

/** @defgroup EM7189_H    em7189 registers */
/** @addtogroup EM7189_H
 * @{
 */
#ifndef EM7189_H
#define EM7189_H

#include <types.h>

#define REG_FPGA_CONFIG ((volatile UInt8*)0x14) /*  */
#define     FPGA_CONFIG_FPGA_DET_DIS_SHIFT 0u
#define     FPGA_CONFIG_FPGA_DET_DIS_MASK  0x1u
#define GET_FPGA_CONFIG_FPGA_DET_DIS(__reg__)  (((__reg__) & 0x1) >> 0u)
#define SET_FPGA_CONFIG_FPGA_DET_DIS(__val__)  (((__val__) << 0u) & 0x1u)
#define     FPGA_CONFIG_RESERVED_7_1_SHIFT 1u
#define     FPGA_CONFIG_RESERVED_7_1_MASK  0xfeu
#define GET_FPGA_CONFIG_RESERVED_7_1(__reg__)  (((__reg__) & 0xfe) >> 1u)
#define SET_FPGA_CONFIG_RESERVED_7_1(__val__)  (((__val__) << 1u) & 0xfeu)

/** @brief Register definition for @ref FPGA_t.Config. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;

    struct {
        /** @brief  */
        UInt8 fpga_det_dis:1;
        /** @brief Reserved bits 7 to 1. Reads as 0. */
        UInt8 reserved_7_1:7;
    } bits;
} RegFPGAConfig_t;

/** @brief Component definition for @ref FPGA. */
typedef struct {
    /** @brief  */
    RegFPGAConfig_t Config;

} FPGA_t;

/** @brief  */
extern volatile FPGA_t FPGA;


#define REG_HOST_INPUT_CHANNEL0 ((volatile UInt8*)0x0) /* This bit-field holds the data to be transferred to CCM via channel 0. A write to this register initiates a transfer (i.e. the host throttles the upload). */
/** @brief Register definition for @ref HOST_t.InputChannel0. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;
} RegHOSTInputChannel0_t;

#define REG_HOST_OUTPUT_CHANNEL1 ((volatile UInt8*)0x1) /* This bit-field holds the data to be transferred from CCM via channel 1. A read from this register initiates a transfer (i.e. the host throttles the download). */
/** @brief Register definition for @ref HOST_t.OutputChannel1. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;
} RegHOSTOutputChannel1_t;

#define REG_HOST_OUTPUT_CHANNEL2 ((volatile UInt8*)0x2) /* This bit-field holds the data to be transferred from CCM via channel 2. A read from this register initiates a transfer (i.e. the host throttles the download). */
/** @brief Register definition for @ref HOST_t.OutputChannel2. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;
} RegHOSTOutputChannel2_t;

#define REG_HOST_OUTPUT_CHANNEL3 ((volatile UInt8*)0x3) /* This bit-field holds the data to be transferred from CCM via channel 3. A read from this register initiates a transfer (i.e. the host throttles the download). */
/** @brief Register definition for @ref HOST_t.OutputChannel3. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;
} RegHOSTOutputChannel3_t;

#define REG_HOST_RESERVED_0 ((volatile UInt8*)0x4) /* General purpose registers for communicating information between host and sensor hub FW. */
/** @brief Register definition for @ref HOST_t.Reserved0. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;
} RegHOSTReserved0_t;

#define REG_HOST_CHIP_CONTROL ((volatile UInt8*)0x5) /* Host writeable register to control run level during boot. */
#define     HOST_CHIP_CONTROL_TURBO_MODE_DISABLE_SHIFT 0u
#define     HOST_CHIP_CONTROL_TURBO_MODE_DISABLE_MASK  0x1u
#define GET_HOST_CHIP_CONTROL_TURBO_MODE_DISABLE(__reg__)  (((__reg__) & 0x1) >> 0u)
#define SET_HOST_CHIP_CONTROL_TURBO_MODE_DISABLE(__val__)  (((__val__) << 0u) & 0x1u)
#define     HOST_CHIP_CONTROL_TURBO_MODE_DISABLE_20_MHZ 0x0u
#define     HOST_CHIP_CONTROL_TURBO_MODE_DISABLE_50_MHZ 0x1u

#define     HOST_CHIP_CONTROL_CLEAR_ERROR_REGS_SHIFT 1u
#define     HOST_CHIP_CONTROL_CLEAR_ERROR_REGS_MASK  0x2u
#define GET_HOST_CHIP_CONTROL_CLEAR_ERROR_REGS(__reg__)  (((__reg__) & 0x2) >> 1u)
#define SET_HOST_CHIP_CONTROL_CLEAR_ERROR_REGS(__val__)  (((__val__) << 1u) & 0x2u)

/** @brief Register definition for @ref HOST_t.ChipControl. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;

    struct {
        /** @brief Keep CPU at lower clock frequency. */
        UInt8 turbo_mode_disable:1;
        /** @brief Reset registers 0x2E - 0x31 to zero. */
        UInt8 clear_error_regs:1;
    } bits;
} RegHOSTChipControl_t;

#define REG_HOST_HOST_INTERFACE_CONTROL ((volatile UInt8*)0x6) /* General purpose registers for communicating information between host and sensor hub FW. */
#define     HOST_HOST_INTERFACE_CONTROL_ABORT_TRANSFER_CH0_SHIFT 0u
#define     HOST_HOST_INTERFACE_CONTROL_ABORT_TRANSFER_CH0_MASK  0x1u
#define GET_HOST_HOST_INTERFACE_CONTROL_ABORT_TRANSFER_CH0(__reg__)  (((__reg__) & 0x1) >> 0u)
#define SET_HOST_HOST_INTERFACE_CONTROL_ABORT_TRANSFER_CH0(__val__)  (((__val__) << 0u) & 0x1u)
#define     HOST_HOST_INTERFACE_CONTROL_ABORT_TRANSFER_CH1_SHIFT 1u
#define     HOST_HOST_INTERFACE_CONTROL_ABORT_TRANSFER_CH1_MASK  0x2u
#define GET_HOST_HOST_INTERFACE_CONTROL_ABORT_TRANSFER_CH1(__reg__)  (((__reg__) & 0x2) >> 1u)
#define SET_HOST_HOST_INTERFACE_CONTROL_ABORT_TRANSFER_CH1(__val__)  (((__val__) << 1u) & 0x2u)
#define     HOST_HOST_INTERFACE_CONTROL_ABORT_TRANSFER_CH2_SHIFT 2u
#define     HOST_HOST_INTERFACE_CONTROL_ABORT_TRANSFER_CH2_MASK  0x4u
#define GET_HOST_HOST_INTERFACE_CONTROL_ABORT_TRANSFER_CH2(__reg__)  (((__reg__) & 0x4) >> 2u)
#define SET_HOST_HOST_INTERFACE_CONTROL_ABORT_TRANSFER_CH2(__val__)  (((__val__) << 2u) & 0x4u)
#define     HOST_HOST_INTERFACE_CONTROL_ABORT_TRANSFER_CH3_SHIFT 3u
#define     HOST_HOST_INTERFACE_CONTROL_ABORT_TRANSFER_CH3_MASK  0x8u
#define GET_HOST_HOST_INTERFACE_CONTROL_ABORT_TRANSFER_CH3(__reg__)  (((__reg__) & 0x8) >> 3u)
#define SET_HOST_HOST_INTERFACE_CONTROL_ABORT_TRANSFER_CH3(__val__)  (((__val__) << 3u) & 0x8u)
#define     HOST_HOST_INTERFACE_CONTROL_AP_SUSPENDED_SHIFT 4u
#define     HOST_HOST_INTERFACE_CONTROL_AP_SUSPENDED_MASK  0x10u
#define GET_HOST_HOST_INTERFACE_CONTROL_AP_SUSPENDED(__reg__)  (((__reg__) & 0x10) >> 4u)
#define SET_HOST_HOST_INTERFACE_CONTROL_AP_SUSPENDED(__val__)  (((__val__) << 4u) & 0x10u)
#define     HOST_HOST_INTERFACE_CONTROL_RESERVED_SHIFT 5u
#define     HOST_HOST_INTERFACE_CONTROL_RESERVED_MASK  0x20u
#define GET_HOST_HOST_INTERFACE_CONTROL_RESERVED(__reg__)  (((__reg__) & 0x20) >> 5u)
#define SET_HOST_HOST_INTERFACE_CONTROL_RESERVED(__val__)  (((__val__) << 5u) & 0x20u)
#define     HOST_HOST_INTERFACE_CONTROL_OUTPUT_HOST_EV_REQ_TS_SHIFT 6u
#define     HOST_HOST_INTERFACE_CONTROL_OUTPUT_HOST_EV_REQ_TS_MASK  0x40u
#define GET_HOST_HOST_INTERFACE_CONTROL_OUTPUT_HOST_EV_REQ_TS(__reg__)  (((__reg__) & 0x40) >> 6u)
#define SET_HOST_HOST_INTERFACE_CONTROL_OUTPUT_HOST_EV_REQ_TS(__val__)  (((__val__) << 6u) & 0x40u)
#define     HOST_HOST_INTERFACE_CONTROL_ASYNC_STATUS_CH_SHIFT 7u
#define     HOST_HOST_INTERFACE_CONTROL_ASYNC_STATUS_CH_MASK  0x80u
#define GET_HOST_HOST_INTERFACE_CONTROL_ASYNC_STATUS_CH(__reg__)  (((__reg__) & 0x80) >> 7u)
#define SET_HOST_HOST_INTERFACE_CONTROL_ASYNC_STATUS_CH(__val__)  (((__val__) << 7u) & 0x80u)

/** @brief Register definition for @ref HOST_t.HostInterfaceControl. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;

    struct {
        /** @brief Cancel FIFO input on channel 0. */
        UInt8 abort_transfer_ch0:1;
        /** @brief Cancel FIFO output on channel 1. */
        UInt8 abort_transfer_ch1:1;
        /** @brief Cancel FIFO output on channel 2. */
        UInt8 abort_transfer_ch2:1;
        /** @brief Cancel FIFO output on channel 3. */
        UInt8 abort_transfer_ch3:1;
        /** @brief Indicate the host is entering or leaving suspend mode. */
        UInt8 ap_suspended:1;
        /** @brief Reserved. */
        UInt8 reserved:1;
        /** @brief Output the EV Req timestamp to the Host IRQ Timestamp registers. */
        UInt8 output_host_ev_req_ts:1;
        /** @brief Output asynchronous async/debug status to ch3, otherwise synchronous cmd status to ch3. */
        UInt8 async_status_ch:1;
    } bits;
} RegHOSTHostInterfaceControl_t;

#define REG_HOST_HOST_INTERRUPT_CONTROL ((volatile UInt8*)0x7) /* General purpose registers for communicating information between host and sensor hub FW. */
#define     HOST_HOST_INTERRUPT_CONTROL_WAKEUP_FIFO_INTERRUPT_MASK_SHIFT 0u
#define     HOST_HOST_INTERRUPT_CONTROL_WAKEUP_FIFO_INTERRUPT_MASK_MASK  0x1u
#define GET_HOST_HOST_INTERRUPT_CONTROL_WAKEUP_FIFO_INTERRUPT_MASK(__reg__)  (((__reg__) & 0x1) >> 0u)
#define SET_HOST_HOST_INTERRUPT_CONTROL_WAKEUP_FIFO_INTERRUPT_MASK(__val__)  (((__val__) << 0u) & 0x1u)
#define     HOST_HOST_INTERRUPT_CONTROL_NONWAKEUP_FIFO_INTERRUPT_MASK_SHIFT 1u
#define     HOST_HOST_INTERRUPT_CONTROL_NONWAKEUP_FIFO_INTERRUPT_MASK_MASK  0x2u
#define GET_HOST_HOST_INTERRUPT_CONTROL_NONWAKEUP_FIFO_INTERRUPT_MASK(__reg__)  (((__reg__) & 0x2) >> 1u)
#define SET_HOST_HOST_INTERRUPT_CONTROL_NONWAKEUP_FIFO_INTERRUPT_MASK(__val__)  (((__val__) << 1u) & 0x2u)
#define     HOST_HOST_INTERRUPT_CONTROL_STATUS_AVAILABLE_INTERRUPT_MASK_SHIFT 2u
#define     HOST_HOST_INTERRUPT_CONTROL_STATUS_AVAILABLE_INTERRUPT_MASK_MASK  0x4u
#define GET_HOST_HOST_INTERRUPT_CONTROL_STATUS_AVAILABLE_INTERRUPT_MASK(__reg__)  (((__reg__) & 0x4) >> 2u)
#define SET_HOST_HOST_INTERRUPT_CONTROL_STATUS_AVAILABLE_INTERRUPT_MASK(__val__)  (((__val__) << 2u) & 0x4u)
#define     HOST_HOST_INTERRUPT_CONTROL_DEBUG_AVAILABLE_INTERRUPT_MASK_SHIFT 3u
#define     HOST_HOST_INTERRUPT_CONTROL_DEBUG_AVAILABLE_INTERRUPT_MASK_MASK  0x8u
#define GET_HOST_HOST_INTERRUPT_CONTROL_DEBUG_AVAILABLE_INTERRUPT_MASK(__reg__)  (((__reg__) & 0x8) >> 3u)
#define SET_HOST_HOST_INTERRUPT_CONTROL_DEBUG_AVAILABLE_INTERRUPT_MASK(__val__)  (((__val__) << 3u) & 0x8u)
#define     HOST_HOST_INTERRUPT_CONTROL_FAULT_INTERRUPT_MASK_SHIFT 4u
#define     HOST_HOST_INTERRUPT_CONTROL_FAULT_INTERRUPT_MASK_MASK  0x10u
#define GET_HOST_HOST_INTERRUPT_CONTROL_FAULT_INTERRUPT_MASK(__reg__)  (((__reg__) & 0x10) >> 4u)
#define SET_HOST_HOST_INTERRUPT_CONTROL_FAULT_INTERRUPT_MASK(__val__)  (((__val__) << 4u) & 0x10u)
#define     HOST_HOST_INTERRUPT_CONTROL_ACTIVE_LOW_INTERRUPT_SHIFT 5u
#define     HOST_HOST_INTERRUPT_CONTROL_ACTIVE_LOW_INTERRUPT_MASK  0x20u
#define GET_HOST_HOST_INTERRUPT_CONTROL_ACTIVE_LOW_INTERRUPT(__reg__)  (((__reg__) & 0x20) >> 5u)
#define SET_HOST_HOST_INTERRUPT_CONTROL_ACTIVE_LOW_INTERRUPT(__val__)  (((__val__) << 5u) & 0x20u)
#define     HOST_HOST_INTERRUPT_CONTROL_ACTIVE_LOW_INTERRUPT_HIGH 0x0u
#define     HOST_HOST_INTERRUPT_CONTROL_ACTIVE_LOW_INTERRUPT_LOW 0x1u

#define     HOST_HOST_INTERRUPT_CONTROL_EDGE_INTERRUPT_SHIFT 6u
#define     HOST_HOST_INTERRUPT_CONTROL_EDGE_INTERRUPT_MASK  0x40u
#define GET_HOST_HOST_INTERRUPT_CONTROL_EDGE_INTERRUPT(__reg__)  (((__reg__) & 0x40) >> 6u)
#define SET_HOST_HOST_INTERRUPT_CONTROL_EDGE_INTERRUPT(__val__)  (((__val__) << 6u) & 0x40u)
#define     HOST_HOST_INTERRUPT_CONTROL_EDGE_INTERRUPT_LEVEL 0x0u
#define     HOST_HOST_INTERRUPT_CONTROL_EDGE_INTERRUPT_EDGE 0x1u

#define     HOST_HOST_INTERRUPT_CONTROL_OPEN_DRAIN_INTERRUPT_SHIFT 7u
#define     HOST_HOST_INTERRUPT_CONTROL_OPEN_DRAIN_INTERRUPT_MASK  0x80u
#define GET_HOST_HOST_INTERRUPT_CONTROL_OPEN_DRAIN_INTERRUPT(__reg__)  (((__reg__) & 0x80) >> 7u)
#define SET_HOST_HOST_INTERRUPT_CONTROL_OPEN_DRAIN_INTERRUPT(__val__)  (((__val__) << 7u) & 0x80u)
#define     HOST_HOST_INTERRUPT_CONTROL_OPEN_DRAIN_INTERRUPT_PUSH-PULL 0x0u
#define     HOST_HOST_INTERRUPT_CONTROL_OPEN_DRAIN_INTERRUPT_OPEN_DRAIN 0x1u


/** @brief Register definition for @ref HOST_t.HostInterruptControl. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;

    struct {
        /** @brief Prevent wakeup FIFO from generating a host interrupt. */
        UInt8 wakeup_fifo_interrupt_mask:1;
        /** @brief Prevent nonwakeup FIFO from generating a host interrupt. */
        UInt8 nonwakeup_fifo_interrupt_mask:1;
        /** @brief Prevent status data in status FIFO from generating a host interrupt. */
        UInt8 status_available_interrupt_mask:1;
        /** @brief Prevent debug data in status FIFO from generating a host interrupt. */
        UInt8 debug_available_interrupt_mask:1;
        /** @brief Prevent a firmware fault occurance from causing a host interrupt. */
        UInt8 fault_interrupt_mask:1;
        /** @brief Make host interrupt active low output instead of default active high. */
        UInt8 active_low_interrupt:1;
        /** @brief Make host interrupt pulse instead of remain level during FIFO read, the default. */
        UInt8 edge_interrupt:1;
        /** @brief Make host interrupt output driver open-drain instead of default push-pull. */
        UInt8 open_drain_interrupt:1;
    } bits;
} RegHOSTHostInterruptControl_t;

#define REG_HOST_GP1_B0 ((volatile UInt8*)0x8) /* General purpose registers for communicating information between host and sensor hub FW. */
#define     HOST_GP1_B0_HOST_GP1_B0_SHIFT 0u
#define     HOST_GP1_B0_HOST_GP1_B0_MASK  0xffu
#define GET_HOST_GP1_B0_HOST_GP1_B0(__reg__)  (((__reg__) & 0xff) >> 0u)
#define SET_HOST_GP1_B0_HOST_GP1_B0(__val__)  (((__val__) << 0u) & 0xffu)

/** @brief Register definition for @ref HOST_t.Gp1B0. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;

    struct {
        /** @brief  */
        UInt8 host_gp1_b0;
    } bits;
} RegHOSTGp1B0_t;

#define REG_HOST_GP1_B1 ((volatile UInt8*)0x9) /* General purpose registers for communicating information between host and sensor hub FW. */
#define     HOST_GP1_B1_HOST_GP1_B1_SHIFT 0u
#define     HOST_GP1_B1_HOST_GP1_B1_MASK  0xffu
#define GET_HOST_GP1_B1_HOST_GP1_B1(__reg__)  (((__reg__) & 0xff) >> 0u)
#define SET_HOST_GP1_B1_HOST_GP1_B1(__val__)  (((__val__) << 0u) & 0xffu)

/** @brief Register definition for @ref HOST_t.Gp1B1. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;

    struct {
        /** @brief  */
        UInt8 host_gp1_b1;
    } bits;
} RegHOSTGp1B1_t;

#define REG_HOST_GP1_B2 ((volatile UInt8*)0xa) /* General purpose registers for communicating information between host and sensor hub FW. */
#define     HOST_GP1_B2_HOST_GP1_B2_SHIFT 0u
#define     HOST_GP1_B2_HOST_GP1_B2_MASK  0xffu
#define GET_HOST_GP1_B2_HOST_GP1_B2(__reg__)  (((__reg__) & 0xff) >> 0u)
#define SET_HOST_GP1_B2_HOST_GP1_B2(__val__)  (((__val__) << 0u) & 0xffu)

/** @brief Register definition for @ref HOST_t.Gp1B2. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;

    struct {
        /** @brief  */
        UInt8 host_gp1_b2;
    } bits;
} RegHOSTGp1B2_t;

#define REG_HOST_GP1_B3 ((volatile UInt8*)0xb) /* General purpose registers for communicating information between host and sensor hub FW. */
#define     HOST_GP1_B3_HOST_GP1_B3_SHIFT 0u
#define     HOST_GP1_B3_HOST_GP1_B3_MASK  0xffu
#define GET_HOST_GP1_B3_HOST_GP1_B3(__reg__)  (((__reg__) & 0xff) >> 0u)
#define SET_HOST_GP1_B3_HOST_GP1_B3(__val__)  (((__val__) << 0u) & 0xffu)

/** @brief Register definition for @ref HOST_t.Gp1B3. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;

    struct {
        /** @brief  */
        UInt8 host_gp1_b3;
    } bits;
} RegHOSTGp1B3_t;

#define REG_HOST_GP2_B0 ((volatile UInt8*)0xc) /* General purpose registers for communicating information between host and sensor hub FW. */
#define     HOST_GP2_B0_HOST_GP2_B0_SHIFT 0u
#define     HOST_GP2_B0_HOST_GP2_B0_MASK  0xffu
#define GET_HOST_GP2_B0_HOST_GP2_B0(__reg__)  (((__reg__) & 0xff) >> 0u)
#define SET_HOST_GP2_B0_HOST_GP2_B0(__val__)  (((__val__) << 0u) & 0xffu)

/** @brief Register definition for @ref HOST_t.Gp2B0. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;

    struct {
        /** @brief  */
        UInt8 host_gp2_b0;
    } bits;
} RegHOSTGp2B0_t;

#define REG_HOST_GP2_B1 ((volatile UInt8*)0xd) /* General purpose registers for communicating information between host and sensor hub FW. */
#define     HOST_GP2_B1_HOST_GP2_B1_SHIFT 0u
#define     HOST_GP2_B1_HOST_GP2_B1_MASK  0xffu
#define GET_HOST_GP2_B1_HOST_GP2_B1(__reg__)  (((__reg__) & 0xff) >> 0u)
#define SET_HOST_GP2_B1_HOST_GP2_B1(__val__)  (((__val__) << 0u) & 0xffu)

/** @brief Register definition for @ref HOST_t.Gp2B1. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;

    struct {
        /** @brief  */
        UInt8 host_gp2_b1;
    } bits;
} RegHOSTGp2B1_t;

#define REG_HOST_GP2_B2 ((volatile UInt8*)0xe) /* General purpose registers for communicating information between host and sensor hub FW. */
#define     HOST_GP2_B2_HOST_GP2_B2_SHIFT 0u
#define     HOST_GP2_B2_HOST_GP2_B2_MASK  0xffu
#define GET_HOST_GP2_B2_HOST_GP2_B2(__reg__)  (((__reg__) & 0xff) >> 0u)
#define SET_HOST_GP2_B2_HOST_GP2_B2(__val__)  (((__val__) << 0u) & 0xffu)

/** @brief Register definition for @ref HOST_t.Gp2B2. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;

    struct {
        /** @brief  */
        UInt8 host_gp2_b2;
    } bits;
} RegHOSTGp2B2_t;

#define REG_HOST_GP2_B3 ((volatile UInt8*)0xf) /* General purpose registers for communicating information between host and sensor hub FW. */
#define     HOST_GP2_B3_HOST_GP2_B3_SHIFT 0u
#define     HOST_GP2_B3_HOST_GP2_B3_MASK  0xffu
#define GET_HOST_GP2_B3_HOST_GP2_B3(__reg__)  (((__reg__) & 0xff) >> 0u)
#define SET_HOST_GP2_B3_HOST_GP2_B3(__val__)  (((__val__) << 0u) & 0xffu)

/** @brief Register definition for @ref HOST_t.Gp2B3. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;

    struct {
        /** @brief  */
        UInt8 host_gp2_b3;
    } bits;
} RegHOSTGp2B3_t;

#define REG_HOST_GP3_B0 ((volatile UInt8*)0x10) /* General purpose registers for communicating information between host and sensor hub FW. */
#define     HOST_GP3_B0_HOST_GP3_B0_SHIFT 0u
#define     HOST_GP3_B0_HOST_GP3_B0_MASK  0xffu
#define GET_HOST_GP3_B0_HOST_GP3_B0(__reg__)  (((__reg__) & 0xff) >> 0u)
#define SET_HOST_GP3_B0_HOST_GP3_B0(__val__)  (((__val__) << 0u) & 0xffu)

/** @brief Register definition for @ref HOST_t.Gp3B0. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;

    struct {
        /** @brief  */
        UInt8 host_gp3_b0;
    } bits;
} RegHOSTGp3B0_t;

#define REG_HOST_GP3_B1 ((volatile UInt8*)0x11) /* General purpose registers for communicating information between host and sensor hub FW. */
#define     HOST_GP3_B1_HOST_GP3_B1_SHIFT 0u
#define     HOST_GP3_B1_HOST_GP3_B1_MASK  0xffu
#define GET_HOST_GP3_B1_HOST_GP3_B1(__reg__)  (((__reg__) & 0xff) >> 0u)
#define SET_HOST_GP3_B1_HOST_GP3_B1(__val__)  (((__val__) << 0u) & 0xffu)

/** @brief Register definition for @ref HOST_t.Gp3B1. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;

    struct {
        /** @brief  */
        UInt8 host_gp3_b1;
    } bits;
} RegHOSTGp3B1_t;

#define REG_HOST_GP3_B2 ((volatile UInt8*)0x12) /* General purpose registers for communicating information between host and sensor hub FW. */
#define     HOST_GP3_B2_HOST_GP3_B2_SHIFT 0u
#define     HOST_GP3_B2_HOST_GP3_B2_MASK  0xffu
#define GET_HOST_GP3_B2_HOST_GP3_B2(__reg__)  (((__reg__) & 0xff) >> 0u)
#define SET_HOST_GP3_B2_HOST_GP3_B2(__val__)  (((__val__) << 0u) & 0xffu)

/** @brief Register definition for @ref HOST_t.Gp3B2. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;

    struct {
        /** @brief  */
        UInt8 host_gp3_b2;
    } bits;
} RegHOSTGp3B2_t;

#define REG_HOST_GP3_B3 ((volatile UInt8*)0x13) /* General purpose registers for communicating information between host and sensor hub FW. */
#define     HOST_GP3_B3_HOST_GP3_B3_SHIFT 0u
#define     HOST_GP3_B3_HOST_GP3_B3_MASK  0xffu
#define GET_HOST_GP3_B3_HOST_GP3_B3(__reg__)  (((__reg__) & 0xff) >> 0u)
#define SET_HOST_GP3_B3_HOST_GP3_B3(__val__)  (((__val__) << 0u) & 0xffu)

/** @brief Register definition for @ref HOST_t.Gp3B3. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;

    struct {
        /** @brief  */
        UInt8 host_gp3_b3;
    } bits;
} RegHOSTGp3B3_t;

#define REG_HOST_RESETREQ ((volatile UInt8*)0x14) /*  */
#define     HOST_RESETREQ_HREQ_RESET_SHIFT 0u
#define     HOST_RESETREQ_HREQ_RESET_MASK  0x1u
#define GET_HOST_RESETREQ_HREQ_RESET(__reg__)  (((__reg__) & 0x1) >> 0u)
#define SET_HOST_RESETREQ_HREQ_RESET(__val__)  (((__val__) << 0u) & 0x1u)
#define     HOST_RESETREQ_RESERVED_7_1_SHIFT 1u
#define     HOST_RESETREQ_RESERVED_7_1_MASK  0xfeu
#define GET_HOST_RESETREQ_RESERVED_7_1(__reg__)  (((__reg__) & 0xfe) >> 1u)
#define SET_HOST_RESETREQ_RESERVED_7_1(__val__)  (((__val__) << 1u) & 0xfeu)

/** @brief Register definition for @ref HOST_t.Resetreq. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;

    struct {
        /** @brief Asserting this bit '1' causes a chip reset. Reads as 0. */
        UInt8 hreq_reset:1;
        /** @brief Reserved bits 7 to 1. Reads as 0. */
        UInt8 reserved_7_1:7;
    } bits;
} RegHOSTResetreq_t;

#define REG_HOST_EVIRQREQ ((volatile UInt8*)0x15) /*  */
#define     HOST_EVIRQREQ_HREQ_EVIRQ_SHIFT 0u
#define     HOST_EVIRQREQ_HREQ_EVIRQ_MASK  0x1u
#define GET_HOST_EVIRQREQ_HREQ_EVIRQ(__reg__)  (((__reg__) & 0x1) >> 0u)
#define SET_HOST_EVIRQREQ_HREQ_EVIRQ(__val__)  (((__val__) << 0u) & 0x1u)
#define     HOST_EVIRQREQ_FPGA_VERSION_SHIFT 1u
#define     HOST_EVIRQREQ_FPGA_VERSION_MASK  0xfeu
#define GET_HOST_EVIRQREQ_FPGA_VERSION(__reg__)  (((__reg__) & 0xfe) >> 1u)
#define SET_HOST_EVIRQREQ_FPGA_VERSION(__val__)  (((__val__) << 1u) & 0xfeu)
#define     HOST_EVIRQREQ_RESERVED_7_1_SHIFT 1u
#define     HOST_EVIRQREQ_RESERVED_7_1_MASK  0xfeu
#define GET_HOST_EVIRQREQ_RESERVED_7_1(__reg__)  (((__reg__) & 0xfe) >> 1u)
#define SET_HOST_EVIRQREQ_RESERVED_7_1(__val__)  (((__val__) << 1u) & 0xfeu)

/** @brief Register definition for @ref HOST_t.Evirqreq. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;

    struct {
        /** @brief Asserting this bit '1' causes an event interrupt. The current timestamp as indicated by the real-time counter is captured in and an interrupt is generated. Reads as 0. */
        UInt8 hreq_evirq:1;
        /** @brief  */
        UInt8 FPGA_VERSION:7;
        /** @brief Reserved bits 7 to 1. Reads as 0. */
        UInt8 reserved_7_1:7;
    } bits;
} RegHOSTEvirqreq_t;

#define REG_HOST_HOST_CONTROL ((volatile UInt8*)0x16) /*  */
#define     HOST_HOST_CONTROL_THREE_WIRE_SPI_MODE_SHIFT 0u
#define     HOST_HOST_CONTROL_THREE_WIRE_SPI_MODE_MASK  0x1u
#define GET_HOST_HOST_CONTROL_THREE_WIRE_SPI_MODE(__reg__)  (((__reg__) & 0x1) >> 0u)
#define SET_HOST_HOST_CONTROL_THREE_WIRE_SPI_MODE(__val__)  (((__val__) << 0u) & 0x1u)
#define     HOST_HOST_CONTROL_THREE_WIRE_SPI_MODE_4-WIRE_SPI 0x0u
#define     HOST_HOST_CONTROL_THREE_WIRE_SPI_MODE_3-WIRE_SPI 0x1u

#define     HOST_HOST_CONTROL_I2C_WATCHDOG_ENABLE_SHIFT 1u
#define     HOST_HOST_CONTROL_I2C_WATCHDOG_ENABLE_MASK  0x2u
#define GET_HOST_HOST_CONTROL_I2C_WATCHDOG_ENABLE(__reg__)  (((__reg__) & 0x2) >> 1u)
#define SET_HOST_HOST_CONTROL_I2C_WATCHDOG_ENABLE(__val__)  (((__val__) << 1u) & 0x2u)
#define     HOST_HOST_CONTROL_I2C_WATCHDOG_ENABLE_DISABLED 0x0u
#define     HOST_HOST_CONTROL_I2C_WATCHDOG_ENABLE_ENABLED 0x1u

#define     HOST_HOST_CONTROL_I2C_WATCHDOG_TIMEOUT_SHIFT 2u
#define     HOST_HOST_CONTROL_I2C_WATCHDOG_TIMEOUT_MASK  0x4u
#define GET_HOST_HOST_CONTROL_I2C_WATCHDOG_TIMEOUT(__reg__)  (((__reg__) & 0x4) >> 2u)
#define SET_HOST_HOST_CONTROL_I2C_WATCHDOG_TIMEOUT(__val__)  (((__val__) << 2u) & 0x4u)
#define     HOST_HOST_CONTROL_I2C_WATCHDOG_TIMEOUT_1MS 0x0u
#define     HOST_HOST_CONTROL_I2C_WATCHDOG_TIMEOUT_50MS 0x1u

#define     HOST_HOST_CONTROL_RESERVED_SHIFT 3u
#define     HOST_HOST_CONTROL_RESERVED_MASK  0x78u
#define GET_HOST_HOST_CONTROL_RESERVED(__reg__)  (((__reg__) & 0x78) >> 3u)
#define SET_HOST_HOST_CONTROL_RESERVED(__val__)  (((__val__) << 3u) & 0x78u)
#define     HOST_HOST_CONTROL_SPI_BYPASS_ENABLE_SHIFT 7u
#define     HOST_HOST_CONTROL_SPI_BYPASS_ENABLE_MASK  0x80u
#define GET_HOST_HOST_CONTROL_SPI_BYPASS_ENABLE(__reg__)  (((__reg__) & 0x80) >> 7u)
#define SET_HOST_HOST_CONTROL_SPI_BYPASS_ENABLE(__val__)  (((__val__) << 7u) & 0x80u)

/** @brief Register definition for @ref HOST_t.HostControl. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;

    struct {
        /** @brief Select 3 wire instead of default 4 wire SPI. */
        UInt8 three_wire_spi_mode:1;
        /** @brief Enable I2C host interface watchdog to prevent bus hangs. */
        UInt8 i2c_watchdog_enable:1;
        /** @brief Select the long (50ms) timeout instead of the default short (1ms). */
        UInt8 i2c_watchdog_timeout:1;
        /** @brief Reserved bits. */
        UInt8 reserved:4;
        /** @brief Route host SPI interface to SIF1; chip reset required to resume normal behavior. */
        UInt8 spi_bypass_enable:1;
    } bits;
} RegHOSTHostControl_t;

#define REG_HOST_HOST_STATUS ((volatile UInt8*)0x17) /*  */
#define     HOST_HOST_STATUS_POWER_STATE_SHIFT 0u
#define     HOST_HOST_STATUS_POWER_STATE_MASK  0x1u
#define GET_HOST_HOST_STATUS_POWER_STATE(__reg__)  (((__reg__) & 0x1) >> 0u)
#define SET_HOST_HOST_STATUS_POWER_STATE(__val__)  (((__val__) << 0u) & 0x1u)
#define     HOST_HOST_STATUS_POWER_STATE_ACTIVE 0x0u
#define     HOST_HOST_STATUS_POWER_STATE_SLEEPING 0x1u

#define     HOST_HOST_STATUS_HOST_PROTOCOL_SHIFT 1u
#define     HOST_HOST_STATUS_HOST_PROTOCOL_MASK  0x2u
#define GET_HOST_HOST_STATUS_HOST_PROTOCOL(__reg__)  (((__reg__) & 0x2) >> 1u)
#define SET_HOST_HOST_STATUS_HOST_PROTOCOL(__val__)  (((__val__) << 1u) & 0x2u)
#define     HOST_HOST_STATUS_HOST_PROTOCOL_I2C 0x0u
#define     HOST_HOST_STATUS_HOST_PROTOCOL_SPI 0x1u

#define     HOST_HOST_STATUS_RESERVED_SHIFT 2u
#define     HOST_HOST_STATUS_RESERVED_MASK  0xcu
#define GET_HOST_HOST_STATUS_RESERVED(__reg__)  (((__reg__) & 0xc) >> 2u)
#define SET_HOST_HOST_STATUS_RESERVED(__val__)  (((__val__) << 2u) & 0xcu)
#define     HOST_HOST_STATUS_CH0_NOT_READY_SHIFT 4u
#define     HOST_HOST_STATUS_CH0_NOT_READY_MASK  0x10u
#define GET_HOST_HOST_STATUS_CH0_NOT_READY(__reg__)  (((__reg__) & 0x10) >> 4u)
#define SET_HOST_HOST_STATUS_CH0_NOT_READY(__val__)  (((__val__) << 4u) & 0x10u)
#define     HOST_HOST_STATUS_CH0_NOT_READY_READY 0x0u
#define     HOST_HOST_STATUS_CH0_NOT_READY_NOT_READY 0x1u

#define     HOST_HOST_STATUS_CH1_NOT_READY_SHIFT 5u
#define     HOST_HOST_STATUS_CH1_NOT_READY_MASK  0x20u
#define GET_HOST_HOST_STATUS_CH1_NOT_READY(__reg__)  (((__reg__) & 0x20) >> 5u)
#define SET_HOST_HOST_STATUS_CH1_NOT_READY(__val__)  (((__val__) << 5u) & 0x20u)
#define     HOST_HOST_STATUS_CH1_NOT_READY_READY 0x0u
#define     HOST_HOST_STATUS_CH1_NOT_READY_NOT_READY 0x1u

#define     HOST_HOST_STATUS_CH2_NOT_READY_SHIFT 6u
#define     HOST_HOST_STATUS_CH2_NOT_READY_MASK  0x40u
#define GET_HOST_HOST_STATUS_CH2_NOT_READY(__reg__)  (((__reg__) & 0x40) >> 6u)
#define SET_HOST_HOST_STATUS_CH2_NOT_READY(__val__)  (((__val__) << 6u) & 0x40u)
#define     HOST_HOST_STATUS_CH2_NOT_READY_READY 0x0u
#define     HOST_HOST_STATUS_CH2_NOT_READY_NOT_READY 0x1u

#define     HOST_HOST_STATUS_CH3_NOT_READY_SHIFT 7u
#define     HOST_HOST_STATUS_CH3_NOT_READY_MASK  0x80u
#define GET_HOST_HOST_STATUS_CH3_NOT_READY(__reg__)  (((__reg__) & 0x80) >> 7u)
#define SET_HOST_HOST_STATUS_CH3_NOT_READY(__val__)  (((__val__) << 7u) & 0x80u)
#define     HOST_HOST_STATUS_CH3_NOT_READY_READY 0x0u
#define     HOST_HOST_STATUS_CH3_NOT_READY_NOT_READY 0x1u


/** @brief Register definition for @ref HOST_t.HostStatus. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;

    struct {
        /** @brief 0 = active, 1 = sleeping. */
        UInt8 power_state:1;
        /** @brief 0 = I2C, 1 = SPI. */
        UInt8 host_protocol:1;
        /** @brief Reserved. Reads as 0. */
        UInt8 reserved:2;
        /** @brief Data written to ch0 will be ignored while this bit is set. */
        UInt8 ch0_not_ready:1;
        /** @brief Data read from ch1 will be all 0s; FIFO is being updated. */
        UInt8 ch1_not_ready:1;
        /** @brief Data read from ch2 will be all 0s; FIFO is being updated. */
        UInt8 ch2_not_ready:1;
        /** @brief Data read from ch3 will be all 0s; FIFO is being updated. */
        UInt8 ch3_not_ready:1;
    } bits;
} RegHOSTHostStatus_t;

#define REG_HOST_CRC0 ((volatile UInt8*)0x18) /* This bit-field is part of the 32-bit CRC calculated over the bytes transferred via the current down-load channel in use. The 32-bit CRC initializes to 0xFFFF_FFFF (initial CRC seed) every time a different channel is accessed. CRC0 holds the lower byte of the 32-bit CRC and CRC3 the upper byte. */
/** @brief Register definition for @ref HOST_t.Crc0. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;
} RegHOSTCrc0_t;

#define REG_HOST_CRC1 ((volatile UInt8*)0x19) /* This bit-field is part of the 32-bit CRC calculated over the bytes transferred via the current down-load channel in use. The 32-bit CRC initializes to 0xFFFF_FFFF (initial CRC seed) every time a different channel is accessed. CRC0 holds the lower byte of the 32-bit CRC and CRC3 the upper byte. */
/** @brief Register definition for @ref HOST_t.Crc1. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;
} RegHOSTCrc1_t;

#define REG_HOST_CRC2 ((volatile UInt8*)0x1a) /* This bit-field is part of the 32-bit CRC calculated over the bytes transferred via the current down-load channel in use. The 32-bit CRC initializes to 0xFFFF_FFFF (initial CRC seed) every time a different channel is accessed. CRC0 holds the lower byte of the 32-bit CRC and CRC3 the upper byte. */
/** @brief Register definition for @ref HOST_t.Crc2. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;
} RegHOSTCrc2_t;

#define REG_HOST_CRC3 ((volatile UInt8*)0x1b) /* This bit-field is part of the 32-bit CRC calculated over the bytes transferred via the current down-load channel in use. The 32-bit CRC initializes to 0xFFFF_FFFF (initial CRC seed) every time a different channel is accessed. CRC0 holds the lower byte of the 32-bit CRC and CRC3 the upper byte. */
/** @brief Register definition for @ref HOST_t.Crc3. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;
} RegHOSTCrc3_t;

/** @brief Component definition for @ref HOST. */
typedef struct {
    /** @brief This bit-field holds the data to be transferred to CCM via channel 0. A write to this register initiates a transfer (i.e. the host throttles the upload). */
    RegHOSTInputChannel0_t InputChannel0;

    /** @brief This bit-field holds the data to be transferred from CCM via channel 1. A read from this register initiates a transfer (i.e. the host throttles the download). */
    RegHOSTOutputChannel1_t OutputChannel1;

    /** @brief This bit-field holds the data to be transferred from CCM via channel 2. A read from this register initiates a transfer (i.e. the host throttles the download). */
    RegHOSTOutputChannel2_t OutputChannel2;

    /** @brief This bit-field holds the data to be transferred from CCM via channel 3. A read from this register initiates a transfer (i.e. the host throttles the download). */
    RegHOSTOutputChannel3_t OutputChannel3;

    /** @brief General purpose registers for communicating information between host and sensor hub FW. */
    RegHOSTReserved0_t Reserved0;

    /** @brief Host writeable register to control run level during boot. */
    RegHOSTChipControl_t ChipControl;

    /** @brief General purpose registers for communicating information between host and sensor hub FW. */
    RegHOSTHostInterfaceControl_t HostInterfaceControl;

    /** @brief General purpose registers for communicating information between host and sensor hub FW. */
    RegHOSTHostInterruptControl_t HostInterruptControl;

    /** @brief General purpose registers for communicating information between host and sensor hub FW. */
    RegHOSTGp1B0_t Gp1B0;

    /** @brief General purpose registers for communicating information between host and sensor hub FW. */
    RegHOSTGp1B1_t Gp1B1;

    /** @brief General purpose registers for communicating information between host and sensor hub FW. */
    RegHOSTGp1B2_t Gp1B2;

    /** @brief General purpose registers for communicating information between host and sensor hub FW. */
    RegHOSTGp1B3_t Gp1B3;

    /** @brief General purpose registers for communicating information between host and sensor hub FW. */
    RegHOSTGp2B0_t Gp2B0;

    /** @brief General purpose registers for communicating information between host and sensor hub FW. */
    RegHOSTGp2B1_t Gp2B1;

    /** @brief General purpose registers for communicating information between host and sensor hub FW. */
    RegHOSTGp2B2_t Gp2B2;

    /** @brief General purpose registers for communicating information between host and sensor hub FW. */
    RegHOSTGp2B3_t Gp2B3;

    /** @brief General purpose registers for communicating information between host and sensor hub FW. */
    RegHOSTGp3B0_t Gp3B0;

    /** @brief General purpose registers for communicating information between host and sensor hub FW. */
    RegHOSTGp3B1_t Gp3B1;

    /** @brief General purpose registers for communicating information between host and sensor hub FW. */
    RegHOSTGp3B2_t Gp3B2;

    /** @brief General purpose registers for communicating information between host and sensor hub FW. */
    RegHOSTGp3B3_t Gp3B3;

    /** @brief  */
    RegHOSTResetreq_t Resetreq;

    /** @brief  */
    RegHOSTEvirqreq_t Evirqreq;

    /** @brief  */
    RegHOSTHostControl_t HostControl;

    /** @brief  */
    RegHOSTHostStatus_t HostStatus;

    /** @brief This bit-field is part of the 32-bit CRC calculated over the bytes transferred via the current down-load channel in use. The 32-bit CRC initializes to 0xFFFF_FFFF (initial CRC seed) every time a different channel is accessed. CRC0 holds the lower byte of the 32-bit CRC and CRC3 the upper byte. */
    RegHOSTCrc0_t Crc0;

    /** @brief This bit-field is part of the 32-bit CRC calculated over the bytes transferred via the current down-load channel in use. The 32-bit CRC initializes to 0xFFFF_FFFF (initial CRC seed) every time a different channel is accessed. CRC0 holds the lower byte of the 32-bit CRC and CRC3 the upper byte. */
    RegHOSTCrc1_t Crc1;

    /** @brief This bit-field is part of the 32-bit CRC calculated over the bytes transferred via the current down-load channel in use. The 32-bit CRC initializes to 0xFFFF_FFFF (initial CRC seed) every time a different channel is accessed. CRC0 holds the lower byte of the 32-bit CRC and CRC3 the upper byte. */
    RegHOSTCrc2_t Crc2;

    /** @brief This bit-field is part of the 32-bit CRC calculated over the bytes transferred via the current down-load channel in use. The 32-bit CRC initializes to 0xFFFF_FFFF (initial CRC seed) every time a different channel is accessed. CRC0 holds the lower byte of the 32-bit CRC and CRC3 the upper byte. */
    RegHOSTCrc3_t Crc3;

} HOST_t;

/** @brief  */
extern volatile HOST_t HOST;


#define REG_PROC_PRODUCT_ID ((volatile UInt8*)0x1c) /*  */
/** @brief Register definition for @ref PROC_t.ProductId. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;
} RegPROCProductId_t;

#define REG_PROC_REVISION_ID ((volatile UInt8*)0x1d) /*  */
/** @brief Register definition for @ref PROC_t.RevisionId. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;
} RegPROCRevisionId_t;

#define REG_PROC_ROM_VERSION_0 ((volatile UInt8*)0x1e) /* General purpose registers for communicating information between sensor hub FW and host. */
/** @brief Register definition for @ref PROC_t.RomVersion0. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;
} RegPROCRomVersion0_t;

#define REG_PROC_ROM_VERSION_1 ((volatile UInt8*)0x1f) /* General purpose registers for communicating information between sensor hub FW and host. */
/** @brief Register definition for @ref PROC_t.RomVersion1. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;
} RegPROCRomVersion1_t;

#define REG_PROC_KERNEL_VERSION_0 ((volatile UInt8*)0x20) /* General purpose registers for communicating information between sensor hub FW and host. */
/** @brief Register definition for @ref PROC_t.KernelVersion0. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;
} RegPROCKernelVersion0_t;

#define REG_PROC_KERNEL_VERSION_1 ((volatile UInt8*)0x21) /* General purpose registers for communicating information between sensor hub FW and host. */
/** @brief Register definition for @ref PROC_t.KernelVersion1. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;
} RegPROCKernelVersion1_t;

#define REG_PROC_USER_VERSION_0 ((volatile UInt8*)0x22) /* General purpose registers for communicating information between sensor hub FW and host. */
/** @brief Register definition for @ref PROC_t.UserVersion0. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;
} RegPROCUserVersion0_t;

#define REG_PROC_USER_VERSION_1 ((volatile UInt8*)0x23) /* General purpose registers for communicating information between sensor hub FW and host. */
/** @brief Register definition for @ref PROC_t.UserVersion1. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;
} RegPROCUserVersion1_t;

#define REG_PROC_FEATURE_STATUS ((volatile UInt8*)0x24) /* General purpose registers for communicating information between sensor hub FW and host. */
#define     PROC_FEATURE_STATUS_RESERVED_1_0_SHIFT 0u
#define     PROC_FEATURE_STATUS_RESERVED_1_0_MASK  0x3u
#define GET_PROC_FEATURE_STATUS_RESERVED_1_0(__reg__)  (((__reg__) & 0x3) >> 0u)
#define SET_PROC_FEATURE_STATUS_RESERVED_1_0(__val__)  (((__val__) << 0u) & 0x3u)
#define     PROC_FEATURE_STATUS_HOST_INT_ID_SHIFT 2u
#define     PROC_FEATURE_STATUS_HOST_INT_ID_MASK  0x1cu
#define GET_PROC_FEATURE_STATUS_HOST_INT_ID(__reg__)  (((__reg__) & 0x1c) >> 2u)
#define SET_PROC_FEATURE_STATUS_HOST_INT_ID(__val__)  (((__val__) << 2u) & 0x1cu)
#define     PROC_FEATURE_STATUS_HOST_INT_ID_ANDROID_K 0x0u
#define     PROC_FEATURE_STATUS_HOST_INT_ID_ANDROID_L 0x1u
#define     PROC_FEATURE_STATUS_HOST_INT_ID_ANDROID_M 0x2u
#define     PROC_FEATURE_STATUS_HOST_INT_ID_ANDROID_N 0x3u
#define     PROC_FEATURE_STATUS_HOST_INT_ID_ANDROID_O 0x4u

#define     PROC_FEATURE_STATUS_ALGO_ID_SHIFT 5u
#define     PROC_FEATURE_STATUS_ALGO_ID_MASK  0xe0u
#define GET_PROC_FEATURE_STATUS_ALGO_ID(__reg__)  (((__reg__) & 0xe0) >> 5u)
#define SET_PROC_FEATURE_STATUS_ALGO_ID(__val__)  (((__val__) << 5u) & 0xe0u)
#define     PROC_FEATURE_STATUS_ALGO_ID_BSX4 0x2u


/** @brief Register definition for @ref PROC_t.FeatureStatus. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;

    struct {
        /** @brief Reserved bits 1 to 0. Reads as 0. */
        UInt8 reserved_1_0:2;
        /** @brief Host Interface ID. 0: Android K. 1: Android L. 2: Android M. 3: Android N. 4: Android O. */
        UInt8 host_int_id:3;
        /** @brief Algorithm ID. 2: BSX4 */
        UInt8 algo_id:3;
    } bits;
} RegPROCFeatureStatus_t;

#define REG_PROC_BOOT_STATUS ((volatile UInt8*)0x25) /* General purpose registers for communicating information between sensor hub FW and host. */
#define     PROC_BOOT_STATUS_FLASH_DETECTED_SHIFT 0u
#define     PROC_BOOT_STATUS_FLASH_DETECTED_MASK  0x1u
#define GET_PROC_BOOT_STATUS_FLASH_DETECTED(__reg__)  (((__reg__) & 0x1) >> 0u)
#define SET_PROC_BOOT_STATUS_FLASH_DETECTED(__val__)  (((__val__) << 0u) & 0x1u)
#define     PROC_BOOT_STATUS_FLASH_VERIFY_DONE_SHIFT 1u
#define     PROC_BOOT_STATUS_FLASH_VERIFY_DONE_MASK  0x2u
#define GET_PROC_BOOT_STATUS_FLASH_VERIFY_DONE(__reg__)  (((__reg__) & 0x2) >> 1u)
#define SET_PROC_BOOT_STATUS_FLASH_VERIFY_DONE(__val__)  (((__val__) << 1u) & 0x2u)
#define     PROC_BOOT_STATUS_FLASH_VERIFY_ERROR_SHIFT 2u
#define     PROC_BOOT_STATUS_FLASH_VERIFY_ERROR_MASK  0x4u
#define GET_PROC_BOOT_STATUS_FLASH_VERIFY_ERROR(__reg__)  (((__reg__) & 0x4) >> 2u)
#define SET_PROC_BOOT_STATUS_FLASH_VERIFY_ERROR(__val__)  (((__val__) << 2u) & 0x4u)
#define     PROC_BOOT_STATUS_NO_FLASH_SHIFT 3u
#define     PROC_BOOT_STATUS_NO_FLASH_MASK  0x8u
#define GET_PROC_BOOT_STATUS_NO_FLASH(__reg__)  (((__reg__) & 0x8) >> 3u)
#define SET_PROC_BOOT_STATUS_NO_FLASH(__val__)  (((__val__) << 3u) & 0x8u)
#define     PROC_BOOT_STATUS_HOST_INTERFACE_READY_SHIFT 4u
#define     PROC_BOOT_STATUS_HOST_INTERFACE_READY_MASK  0x10u
#define GET_PROC_BOOT_STATUS_HOST_INTERFACE_READY(__reg__)  (((__reg__) & 0x10) >> 4u)
#define SET_PROC_BOOT_STATUS_HOST_INTERFACE_READY(__val__)  (((__val__) << 4u) & 0x10u)
#define     PROC_BOOT_STATUS_FIRMWARE_VERIFY_DONE_SHIFT 5u
#define     PROC_BOOT_STATUS_FIRMWARE_VERIFY_DONE_MASK  0x20u
#define GET_PROC_BOOT_STATUS_FIRMWARE_VERIFY_DONE(__reg__)  (((__reg__) & 0x20) >> 5u)
#define SET_PROC_BOOT_STATUS_FIRMWARE_VERIFY_DONE(__val__)  (((__val__) << 5u) & 0x20u)
#define     PROC_BOOT_STATUS_FIRMWARE_VERIFY_ERROR_SHIFT 6u
#define     PROC_BOOT_STATUS_FIRMWARE_VERIFY_ERROR_MASK  0x40u
#define GET_PROC_BOOT_STATUS_FIRMWARE_VERIFY_ERROR(__reg__)  (((__reg__) & 0x40) >> 6u)
#define SET_PROC_BOOT_STATUS_FIRMWARE_VERIFY_ERROR(__val__)  (((__val__) << 6u) & 0x40u)
#define     PROC_BOOT_STATUS_FIRMWARE_IDLE_SHIFT 7u
#define     PROC_BOOT_STATUS_FIRMWARE_IDLE_MASK  0x80u
#define GET_PROC_BOOT_STATUS_FIRMWARE_IDLE(__reg__)  (((__reg__) & 0x80) >> 7u)
#define SET_PROC_BOOT_STATUS_FIRMWARE_IDLE(__val__)  (((__val__) << 7u) & 0x80u)

/** @brief Register definition for @ref PROC_t.BootStatus. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;

    struct {
        /** @brief A flash device has been detected. */
        UInt8 flash_detected:1;
        /** @brief The flash signature/crc verified correctly. */
        UInt8 flash_verify_done:1;
        /** @brief Flash signature/crc mismatch. */
        UInt8 flash_verify_error:1;
        /** @brief No QSPI Flash detected or boot_cfg pin = 1. */
        UInt8 no_flash:1;
        /** @brief HIF is ready for communication. */
        UInt8 host_interface_ready:1;
        /** @brief The upload signature/crc verified correctly. */
        UInt8 firmware_verify_done:1;
        /** @brief Upload signature/crc mismatch. */
        UInt8 firmware_verify_error:1;
        /** @brief Firmware waiting for ChipControl.bit0 = 1. */
        UInt8 firmware_idle:1;
    } bits;
} RegPROCBootStatus_t;

#define REG_PROC_HOST_IRQ_TIMESTAMP_0 ((volatile UInt8*)0x26) /* General purpose registers for communicating information between sensor hub FW and host. */
/** @brief Register definition for @ref PROC_t.HostIrqTimestamp0. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;
} RegPROCHostIrqTimestamp0_t;

#define REG_PROC_HOST_IRQ_TIMESTAMP_1 ((volatile UInt8*)0x27) /* General purpose registers for communicating information between sensor hub FW and host. */
/** @brief Register definition for @ref PROC_t.HostIrqTimestamp1. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;
} RegPROCHostIrqTimestamp1_t;

#define REG_PROC_HOST_IRQ_TIMESTAMP_2 ((volatile UInt8*)0x28) /* General purpose registers for communicating information between sensor hub FW and host. */
/** @brief Register definition for @ref PROC_t.HostIrqTimestamp2. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;
} RegPROCHostIrqTimestamp2_t;

#define REG_PROC_HOST_IRQ_TIMESTAMP_3 ((volatile UInt8*)0x29) /* General purpose registers for communicating information between sensor hub FW and host. */
/** @brief Register definition for @ref PROC_t.HostIrqTimestamp3. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;
} RegPROCHostIrqTimestamp3_t;

#define REG_PROC_HOST_IRQ_TIMESTAMP_4 ((volatile UInt8*)0x2a) /* General purpose registers for communicating information between sensor hub FW and host. */
/** @brief Register definition for @ref PROC_t.HostIrqTimestamp4. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;
} RegPROCHostIrqTimestamp4_t;

#define REG_PROC_BST_PRODUCT_TYPE ((volatile UInt8*)0x2b) /* Value stored in OTP byte 11. */
/** @brief Register definition for @ref PROC_t.BstProductType. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;
} RegPROCBstProductType_t;

#define REG_PROC_RESERVED_2 ((volatile UInt8*)0x2c) /* General purpose registers for communicating information between sensor hub FW and host. */
/** @brief Register definition for @ref PROC_t.Reserved2. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;
} RegPROCReserved2_t;

#define REG_PROC_INTERRUPT_STATUS ((volatile UInt8*)0x2d) /* General purpose registers for communicating information between sensor hub FW and host. */
#define     PROC_INTERRUPT_STATUS_HOST_IRQ_SHIFT 0u
#define     PROC_INTERRUPT_STATUS_HOST_IRQ_MASK  0x1u
#define GET_PROC_INTERRUPT_STATUS_HOST_IRQ(__reg__)  (((__reg__) & 0x1) >> 0u)
#define SET_PROC_INTERRUPT_STATUS_HOST_IRQ(__val__)  (((__val__) << 0u) & 0x1u)
#define     PROC_INTERRUPT_STATUS_HOST_IRQ_ASSERTED 0x1u

#define     PROC_INTERRUPT_STATUS_WAKEUP_FIFO_SHIFT 1u
#define     PROC_INTERRUPT_STATUS_WAKEUP_FIFO_MASK  0x6u
#define GET_PROC_INTERRUPT_STATUS_WAKEUP_FIFO(__reg__)  (((__reg__) & 0x6) >> 1u)
#define SET_PROC_INTERRUPT_STATUS_WAKEUP_FIFO(__val__)  (((__val__) << 1u) & 0x6u)
#define     PROC_INTERRUPT_STATUS_WAKEUP_FIFO_NO_DATA 0x0u
#define     PROC_INTERRUPT_STATUS_WAKEUP_FIFO_IMMEDIATE 0x1u
#define     PROC_INTERRUPT_STATUS_WAKEUP_FIFO_LATENCY 0x2u
#define     PROC_INTERRUPT_STATUS_WAKEUP_FIFO_WATERMARK 0x3u

#define     PROC_INTERRUPT_STATUS_NONWAKEUP_FIFO_SHIFT 3u
#define     PROC_INTERRUPT_STATUS_NONWAKEUP_FIFO_MASK  0x18u
#define GET_PROC_INTERRUPT_STATUS_NONWAKEUP_FIFO(__reg__)  (((__reg__) & 0x18) >> 3u)
#define SET_PROC_INTERRUPT_STATUS_NONWAKEUP_FIFO(__val__)  (((__val__) << 3u) & 0x18u)
#define     PROC_INTERRUPT_STATUS_NONWAKEUP_FIFO_NO_DATA 0x0u
#define     PROC_INTERRUPT_STATUS_NONWAKEUP_FIFO_IMMEDIATE 0x1u
#define     PROC_INTERRUPT_STATUS_NONWAKEUP_FIFO_LATENCY 0x2u
#define     PROC_INTERRUPT_STATUS_NONWAKEUP_FIFO_WATERMARK 0x3u

#define     PROC_INTERRUPT_STATUS_STATUS_SHIFT 5u
#define     PROC_INTERRUPT_STATUS_STATUS_MASK  0x20u
#define GET_PROC_INTERRUPT_STATUS_STATUS(__reg__)  (((__reg__) & 0x20) >> 5u)
#define SET_PROC_INTERRUPT_STATUS_STATUS(__val__)  (((__val__) << 5u) & 0x20u)
#define     PROC_INTERRUPT_STATUS_STATUS_NO_DATA 0x0u
#define     PROC_INTERRUPT_STATUS_STATUS_DATA_AVAILABLE 0x1u

#define     PROC_INTERRUPT_STATUS_DEBUG_SHIFT 6u
#define     PROC_INTERRUPT_STATUS_DEBUG_MASK  0x40u
#define GET_PROC_INTERRUPT_STATUS_DEBUG(__reg__)  (((__reg__) & 0x40) >> 6u)
#define SET_PROC_INTERRUPT_STATUS_DEBUG(__val__)  (((__val__) << 6u) & 0x40u)
#define     PROC_INTERRUPT_STATUS_DEBUG_NO_DATA 0x0u
#define     PROC_INTERRUPT_STATUS_DEBUG_DATA_AVAILABLE 0x1u

#define     PROC_INTERRUPT_STATUS_RESET_OR_FAULT_SHIFT 7u
#define     PROC_INTERRUPT_STATUS_RESET_OR_FAULT_MASK  0x80u
#define GET_PROC_INTERRUPT_STATUS_RESET_OR_FAULT(__reg__)  (((__reg__) & 0x80) >> 7u)
#define SET_PROC_INTERRUPT_STATUS_RESET_OR_FAULT(__val__)  (((__val__) << 7u) & 0x80u)

/** @brief Register definition for @ref PROC_t.InterruptStatus. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;

    struct {
        /** @brief The host interrupt is asserted. */
        UInt8 host_irq:1;
        /** @brief 0 = not cause of interrupt; 1 = immediate data; 2 = latency timeout; 3 = watermark. */
        UInt8 wakeup_fifo:2;
        /** @brief 0 = not cause of interrupt; 1 = immediate data; 2 = latency timeout; 3 = watermark. */
        UInt8 nonwakeup_fifo:2;
        /** @brief The status channel has available synchronous status data. */
        UInt8 status:1;
        /** @brief The status channel has available debug or async status data. */
        UInt8 debug:1;
        /** @brief The chip was reset due to WDT, POR, HIF, or pin, or there was a fatal firmware error. */
        UInt8 reset_or_fault:1;
    } bits;
} RegPROCInterruptStatus_t;

#define REG_PROC_ERROR ((volatile UInt8*)0x2e) /* Firmware error code. */
/** @brief Register definition for @ref PROC_t.Error. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;
} RegPROCError_t;

#define REG_PROC_INTERRUPT_STATE ((volatile UInt8*)0x2f) /* Debug-related interrupt state. */
/** @brief Register definition for @ref PROC_t.InterruptState. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;
} RegPROCInterruptState_t;

#define REG_PROC_DEBUG_VALUE ((volatile UInt8*)0x30) /* Debug value. */
/** @brief Register definition for @ref PROC_t.DebugValue. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;
} RegPROCDebugValue_t;

#define REG_PROC_DEBUG_STATE ((volatile UInt8*)0x31) /* Debug state. */
/** @brief Register definition for @ref PROC_t.DebugState. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;
} RegPROCDebugState_t;

#define REG_PROC_GP5_B0 ((volatile UInt8*)0x32) /* General purpose registers for communicating information between sensor hub FW and host. */
#define     PROC_GP5_B0_PROC_GP5_7_0_SHIFT 0u
#define     PROC_GP5_B0_PROC_GP5_7_0_MASK  0xffu
#define GET_PROC_GP5_B0_PROC_GP5_7_0(__reg__)  (((__reg__) & 0xff) >> 0u)
#define SET_PROC_GP5_B0_PROC_GP5_7_0(__val__)  (((__val__) << 0u) & 0xffu)

/** @brief Register definition for @ref PROC_t.Gp5B0. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;

    struct {
        /** @brief  */
        UInt8 proc_gp5_7_0;
    } bits;
} RegPROCGp5B0_t;

#define REG_PROC_GP5_B1 ((volatile UInt8*)0x33) /* General purpose registers for communicating information between sensor hub FW and host. */
#define     PROC_GP5_B1_PROC_GP5_15_8_SHIFT 0u
#define     PROC_GP5_B1_PROC_GP5_15_8_MASK  0xffu
#define GET_PROC_GP5_B1_PROC_GP5_15_8(__reg__)  (((__reg__) & 0xff) >> 0u)
#define SET_PROC_GP5_B1_PROC_GP5_15_8(__val__)  (((__val__) << 0u) & 0xffu)

/** @brief Register definition for @ref PROC_t.Gp5B1. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;

    struct {
        /** @brief  */
        UInt8 proc_gp5_15_8;
    } bits;
} RegPROCGp5B1_t;

#define REG_PROC_GP5_B2 ((volatile UInt8*)0x34) /* General purpose registers for communicating information between sensor hub FW and host. */
#define     PROC_GP5_B2_PROC_GP5_23_16_SHIFT 0u
#define     PROC_GP5_B2_PROC_GP5_23_16_MASK  0xffu
#define GET_PROC_GP5_B2_PROC_GP5_23_16(__reg__)  (((__reg__) & 0xff) >> 0u)
#define SET_PROC_GP5_B2_PROC_GP5_23_16(__val__)  (((__val__) << 0u) & 0xffu)

/** @brief Register definition for @ref PROC_t.Gp5B2. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;

    struct {
        /** @brief  */
        UInt8 proc_gp5_23_16;
    } bits;
} RegPROCGp5B2_t;

#define REG_PROC_GP5_B3 ((volatile UInt8*)0x35) /* General purpose registers for communicating information between sensor hub FW and host. */
#define     PROC_GP5_B3_PROC_GP5_31_24_SHIFT 0u
#define     PROC_GP5_B3_PROC_GP5_31_24_MASK  0xffu
#define GET_PROC_GP5_B3_PROC_GP5_31_24(__reg__)  (((__reg__) & 0xff) >> 0u)
#define SET_PROC_GP5_B3_PROC_GP5_31_24(__val__)  (((__val__) << 0u) & 0xffu)

/** @brief Register definition for @ref PROC_t.Gp5B3. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;

    struct {
        /** @brief  */
        UInt8 proc_gp5_31_24;
    } bits;
} RegPROCGp5B3_t;

#define REG_PROC_GP6_B0 ((volatile UInt8*)0x36) /* General purpose registers for communicating information between sensor hub FW and host. */
#define     PROC_GP6_B0_PROC_GP6_7_0_SHIFT 0u
#define     PROC_GP6_B0_PROC_GP6_7_0_MASK  0xffu
#define GET_PROC_GP6_B0_PROC_GP6_7_0(__reg__)  (((__reg__) & 0xff) >> 0u)
#define SET_PROC_GP6_B0_PROC_GP6_7_0(__val__)  (((__val__) << 0u) & 0xffu)

/** @brief Register definition for @ref PROC_t.Gp6B0. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;

    struct {
        /** @brief  */
        UInt8 proc_gp6_7_0;
    } bits;
} RegPROCGp6B0_t;

#define REG_PROC_GP6_B1 ((volatile UInt8*)0x37) /* General purpose registers for communicating information between sensor hub FW and host. */
#define     PROC_GP6_B1_PROC_GP6_15_8_SHIFT 0u
#define     PROC_GP6_B1_PROC_GP6_15_8_MASK  0xffu
#define GET_PROC_GP6_B1_PROC_GP6_15_8(__reg__)  (((__reg__) & 0xff) >> 0u)
#define SET_PROC_GP6_B1_PROC_GP6_15_8(__val__)  (((__val__) << 0u) & 0xffu)

/** @brief Register definition for @ref PROC_t.Gp6B1. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;

    struct {
        /** @brief  */
        UInt8 proc_gp6_15_8;
    } bits;
} RegPROCGp6B1_t;

#define REG_PROC_GP6_B2 ((volatile UInt8*)0x38) /* General purpose registers for communicating information between sensor hub FW and host. */
#define     PROC_GP6_B2_PROC_GP6_23_16_SHIFT 0u
#define     PROC_GP6_B2_PROC_GP6_23_16_MASK  0xffu
#define GET_PROC_GP6_B2_PROC_GP6_23_16(__reg__)  (((__reg__) & 0xff) >> 0u)
#define SET_PROC_GP6_B2_PROC_GP6_23_16(__val__)  (((__val__) << 0u) & 0xffu)

/** @brief Register definition for @ref PROC_t.Gp6B2. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;

    struct {
        /** @brief  */
        UInt8 proc_gp6_23_16;
    } bits;
} RegPROCGp6B2_t;

#define REG_PROC_GP6_B3 ((volatile UInt8*)0x39) /* General purpose registers for communicating information between sensor hub FW and host. */
#define     PROC_GP6_B3_PROC_GP6_31_24_SHIFT 0u
#define     PROC_GP6_B3_PROC_GP6_31_24_MASK  0xffu
#define GET_PROC_GP6_B3_PROC_GP6_31_24(__reg__)  (((__reg__) & 0xff) >> 0u)
#define SET_PROC_GP6_B3_PROC_GP6_31_24(__val__)  (((__val__) << 0u) & 0xffu)

/** @brief Register definition for @ref PROC_t.Gp6B3. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;

    struct {
        /** @brief  */
        UInt8 proc_gp6_31_24;
    } bits;
} RegPROCGp6B3_t;

#define REG_PROC_GP7_B0 ((volatile UInt8*)0x3a) /* General purpose registers for communicating information between sensor hub FW and host. */
#define     PROC_GP7_B0_PROC_GP7_7_0_SHIFT 0u
#define     PROC_GP7_B0_PROC_GP7_7_0_MASK  0xffu
#define GET_PROC_GP7_B0_PROC_GP7_7_0(__reg__)  (((__reg__) & 0xff) >> 0u)
#define SET_PROC_GP7_B0_PROC_GP7_7_0(__val__)  (((__val__) << 0u) & 0xffu)

/** @brief Register definition for @ref PROC_t.Gp7B0. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;

    struct {
        /** @brief  */
        UInt8 proc_gp7_7_0;
    } bits;
} RegPROCGp7B0_t;

#define REG_PROC_GP7_B1 ((volatile UInt8*)0x3b) /* General purpose registers for communicating information between sensor hub FW and host. */
#define     PROC_GP7_B1_PROC_GP7_15_8_SHIFT 0u
#define     PROC_GP7_B1_PROC_GP7_15_8_MASK  0xffu
#define GET_PROC_GP7_B1_PROC_GP7_15_8(__reg__)  (((__reg__) & 0xff) >> 0u)
#define SET_PROC_GP7_B1_PROC_GP7_15_8(__val__)  (((__val__) << 0u) & 0xffu)

/** @brief Register definition for @ref PROC_t.Gp7B1. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;

    struct {
        /** @brief  */
        UInt8 proc_gp7_15_8;
    } bits;
} RegPROCGp7B1_t;

#define REG_PROC_GP7_B2 ((volatile UInt8*)0x3c) /* General purpose registers for communicating information between sensor hub FW and host. */
#define     PROC_GP7_B2_PROC_GP7_23_16_SHIFT 0u
#define     PROC_GP7_B2_PROC_GP7_23_16_MASK  0xffu
#define GET_PROC_GP7_B2_PROC_GP7_23_16(__reg__)  (((__reg__) & 0xff) >> 0u)
#define SET_PROC_GP7_B2_PROC_GP7_23_16(__val__)  (((__val__) << 0u) & 0xffu)

/** @brief Register definition for @ref PROC_t.Gp7B2. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;

    struct {
        /** @brief  */
        UInt8 proc_gp7_23_16;
    } bits;
} RegPROCGp7B2_t;

#define REG_PROC_GP7_B3 ((volatile UInt8*)0x3d) /* General purpose registers for communicating information between sensor hub FW and host. */
#define     PROC_GP7_B3_PROC_GP7_31_24_SHIFT 0u
#define     PROC_GP7_B3_PROC_GP7_31_24_MASK  0xffu
#define GET_PROC_GP7_B3_PROC_GP7_31_24(__reg__)  (((__reg__) & 0xff) >> 0u)
#define SET_PROC_GP7_B3_PROC_GP7_31_24(__val__)  (((__val__) << 0u) & 0xffu)

/** @brief Register definition for @ref PROC_t.Gp7B3. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;

    struct {
        /** @brief  */
        UInt8 proc_gp7_31_24;
    } bits;
} RegPROCGp7B3_t;

/** @brief Component definition for @ref PROC. */
typedef struct {
    /** @brief  */
    RegPROCProductId_t ProductId;

    /** @brief  */
    RegPROCRevisionId_t RevisionId;

    /** @brief General purpose registers for communicating information between sensor hub FW and host. */
    RegPROCRomVersion0_t RomVersion0;

    /** @brief General purpose registers for communicating information between sensor hub FW and host. */
    RegPROCRomVersion1_t RomVersion1;

    /** @brief General purpose registers for communicating information between sensor hub FW and host. */
    RegPROCKernelVersion0_t KernelVersion0;

    /** @brief General purpose registers for communicating information between sensor hub FW and host. */
    RegPROCKernelVersion1_t KernelVersion1;

    /** @brief General purpose registers for communicating information between sensor hub FW and host. */
    RegPROCUserVersion0_t UserVersion0;

    /** @brief General purpose registers for communicating information between sensor hub FW and host. */
    RegPROCUserVersion1_t UserVersion1;

    /** @brief General purpose registers for communicating information between sensor hub FW and host. */
    RegPROCFeatureStatus_t FeatureStatus;

    /** @brief General purpose registers for communicating information between sensor hub FW and host. */
    RegPROCBootStatus_t BootStatus;

    /** @brief General purpose registers for communicating information between sensor hub FW and host. */
    RegPROCHostIrqTimestamp0_t HostIrqTimestamp0;

    /** @brief General purpose registers for communicating information between sensor hub FW and host. */
    RegPROCHostIrqTimestamp1_t HostIrqTimestamp1;

    /** @brief General purpose registers for communicating information between sensor hub FW and host. */
    RegPROCHostIrqTimestamp2_t HostIrqTimestamp2;

    /** @brief General purpose registers for communicating information between sensor hub FW and host. */
    RegPROCHostIrqTimestamp3_t HostIrqTimestamp3;

    /** @brief General purpose registers for communicating information between sensor hub FW and host. */
    RegPROCHostIrqTimestamp4_t HostIrqTimestamp4;

    /** @brief Value stored in OTP byte 11. */
    RegPROCBstProductType_t BstProductType;

    /** @brief General purpose registers for communicating information between sensor hub FW and host. */
    RegPROCReserved2_t Reserved2;

    /** @brief General purpose registers for communicating information between sensor hub FW and host. */
    RegPROCInterruptStatus_t InterruptStatus;

    /** @brief Firmware error code. */
    RegPROCError_t Error;

    /** @brief Debug-related interrupt state. */
    RegPROCInterruptState_t InterruptState;

    /** @brief Debug value. */
    RegPROCDebugValue_t DebugValue;

    /** @brief Debug state. */
    RegPROCDebugState_t DebugState;

    /** @brief General purpose registers for communicating information between sensor hub FW and host. */
    RegPROCGp5B0_t Gp5B0;

    /** @brief General purpose registers for communicating information between sensor hub FW and host. */
    RegPROCGp5B1_t Gp5B1;

    /** @brief General purpose registers for communicating information between sensor hub FW and host. */
    RegPROCGp5B2_t Gp5B2;

    /** @brief General purpose registers for communicating information between sensor hub FW and host. */
    RegPROCGp5B3_t Gp5B3;

    /** @brief General purpose registers for communicating information between sensor hub FW and host. */
    RegPROCGp6B0_t Gp6B0;

    /** @brief General purpose registers for communicating information between sensor hub FW and host. */
    RegPROCGp6B1_t Gp6B1;

    /** @brief General purpose registers for communicating information between sensor hub FW and host. */
    RegPROCGp6B2_t Gp6B2;

    /** @brief General purpose registers for communicating information between sensor hub FW and host. */
    RegPROCGp6B3_t Gp6B3;

    /** @brief General purpose registers for communicating information between sensor hub FW and host. */
    RegPROCGp7B0_t Gp7B0;

    /** @brief General purpose registers for communicating information between sensor hub FW and host. */
    RegPROCGp7B1_t Gp7B1;

    /** @brief General purpose registers for communicating information between sensor hub FW and host. */
    RegPROCGp7B2_t Gp7B2;

    /** @brief General purpose registers for communicating information between sensor hub FW and host. */
    RegPROCGp7B3_t Gp7B3;

} PROC_t;

/** @brief  */
extern volatile PROC_t PROC;


#define REG_SPI_BYPASS_STOP ((volatile UInt8*)0x20) /*  */
#define     SPI_BYPASS_STOP_SPI_BYPASS_STOP_6_0_SHIFT 0u
#define     SPI_BYPASS_STOP_SPI_BYPASS_STOP_6_0_MASK  0x7fu
#define GET_SPI_BYPASS_STOP_SPI_BYPASS_STOP_6_0(__reg__)  (((__reg__) & 0x7f) >> 0u)
#define SET_SPI_BYPASS_STOP_SPI_BYPASS_STOP_6_0(__val__)  (((__val__) << 0u) & 0x7fu)
#define     SPI_BYPASS_STOP_RESERVED_7_7_SHIFT 7u
#define     SPI_BYPASS_STOP_RESERVED_7_7_MASK  0x80u
#define GET_SPI_BYPASS_STOP_RESERVED_7_7(__reg__)  (((__reg__) & 0x80) >> 7u)
#define SET_SPI_BYPASS_STOP_RESERVED_7_7(__val__)  (((__val__) << 7u) & 0x80u)

/** @brief Register definition for @ref SPI_t.BypassStop. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;

    struct {
        /** @brief  */
        UInt8 spi_bypass_stop_6_0:7;
        /** @brief Reserved bit 7. Reads as 0. */
        UInt8 reserved_7_7:1;
    } bits;
} RegSPIBypassStop_t;

/** @brief Component definition for @ref SPI. */
typedef struct {
    /** @brief  */
    RegSPIBypassStop_t BypassStop;

} SPI_t;

/** @brief  */
extern volatile SPI_t SPI;


#endif /* !EM7189_H */

/** @} */
