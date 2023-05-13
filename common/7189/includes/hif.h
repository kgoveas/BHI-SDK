////////////////////////////////////////////////////////////////////////////////
///
/// @file       common/7189/includes/hif.h
///
/// @project    EM7189
///
/// @brief      hif registers
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

/** @defgroup HIF_H    hif registers */
/** @addtogroup HIF_H
 * @{
 */
#ifndef HIF_H
#define HIF_H

#include <types.h>

#define REG_HIF_CHN0 ((volatile UInt32*)0xf00000) /*  */
#define     HIF_CHN0_CHN0_PDATA_SHIFT 0u
#define     HIF_CHN0_CHN0_PDATA_MASK  0xffffffffu
#define GET_HIF_CHN0_CHN0_PDATA(__reg__)  (((__reg__) & 0xffffffff) >> 0u)
#define SET_HIF_CHN0_CHN0_PDATA(__val__)  (((__val__) << 0u) & 0xffffffffu)

/** @brief Register definition for @ref HIF_t.Chn0. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt32 chn0_pdata;
    } bits;
} RegHIFChn0_t;

#define REG_HIF_CHN1 ((volatile UInt32*)0xf00004) /*  */
#define     HIF_CHN1_CHN1_PDATA_SHIFT 0u
#define     HIF_CHN1_CHN1_PDATA_MASK  0xffffffffu
#define GET_HIF_CHN1_CHN1_PDATA(__reg__)  (((__reg__) & 0xffffffff) >> 0u)
#define SET_HIF_CHN1_CHN1_PDATA(__val__)  (((__val__) << 0u) & 0xffffffffu)

/** @brief Register definition for @ref HIF_t.Chn1. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt32 chn1_pdata;
    } bits;
} RegHIFChn1_t;

#define REG_HIF_CHN2 ((volatile UInt32*)0xf00008) /*  */
#define     HIF_CHN2_CHN2_PDATA_SHIFT 0u
#define     HIF_CHN2_CHN2_PDATA_MASK  0xffffffffu
#define GET_HIF_CHN2_CHN2_PDATA(__reg__)  (((__reg__) & 0xffffffff) >> 0u)
#define SET_HIF_CHN2_CHN2_PDATA(__val__)  (((__val__) << 0u) & 0xffffffffu)

/** @brief Register definition for @ref HIF_t.Chn2. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt32 chn2_pdata;
    } bits;
} RegHIFChn2_t;

#define REG_HIF_CHN3 ((volatile UInt32*)0xf0000c) /*  */
#define     HIF_CHN3_CHN3_PDATA_SHIFT 0u
#define     HIF_CHN3_CHN3_PDATA_MASK  0xffffffffu
#define GET_HIF_CHN3_CHN3_PDATA(__reg__)  (((__reg__) & 0xffffffff) >> 0u)
#define SET_HIF_CHN3_CHN3_PDATA(__val__)  (((__val__) << 0u) & 0xffffffffu)

/** @brief Register definition for @ref HIF_t.Chn3. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt32 chn3_pdata;
    } bits;
} RegHIFChn3_t;

#define REG_HIF_CHNCTRL0 ((volatile UInt32*)0xf00010) /*  */
#define     HIF_CHNCTRL0_CHN0_TBYTES_SHIFT 0u
#define     HIF_CHNCTRL0_CHN0_TBYTES_MASK  0xffffu
#define GET_HIF_CHNCTRL0_CHN0_TBYTES(__reg__)  (((__reg__) & 0xffff) >> 0u)
#define SET_HIF_CHNCTRL0_CHN0_TBYTES(__val__)  (((__val__) << 0u) & 0xffffu)
#define     HIF_CHNCTRL0_CHN0_WMARK_SHIFT 16u
#define     HIF_CHNCTRL0_CHN0_WMARK_MASK  0x1f0000u
#define GET_HIF_CHNCTRL0_CHN0_WMARK(__reg__)  (((__reg__) & 0x1f0000) >> 16u)
#define SET_HIF_CHNCTRL0_CHN0_WMARK(__val__)  (((__val__) << 16u) & 0x1f0000u)
#define     HIF_CHNCTRL0_CHN0_TSIZE_SHIFT 21u
#define     HIF_CHNCTRL0_CHN0_TSIZE_MASK  0x600000u
#define GET_HIF_CHNCTRL0_CHN0_TSIZE(__reg__)  (((__reg__) & 0x600000) >> 21u)
#define SET_HIF_CHNCTRL0_CHN0_TSIZE(__val__)  (((__val__) << 21u) & 0x600000u)
#define     HIF_CHNCTRL0_CHN0_TSIZE_BYTE 0x0u
#define     HIF_CHNCTRL0_CHN0_TSIZE_HALF_WORD 0x1u
#define     HIF_CHNCTRL0_CHN0_TSIZE_WORD 0x2u

#define     HIF_CHNCTRL0_RESERVED_23_23_SHIFT 23u
#define     HIF_CHNCTRL0_RESERVED_23_23_MASK  0x800000u
#define GET_HIF_CHNCTRL0_RESERVED_23_23(__reg__)  (((__reg__) & 0x800000) >> 23u)
#define SET_HIF_CHNCTRL0_RESERVED_23_23(__val__)  (((__val__) << 23u) & 0x800000u)
#define     HIF_CHNCTRL0_CHN0_BBYTES_SHIFT 24u
#define     HIF_CHNCTRL0_CHN0_BBYTES_MASK  0x3f000000u
#define GET_HIF_CHNCTRL0_CHN0_BBYTES(__reg__)  (((__reg__) & 0x3f000000) >> 24u)
#define SET_HIF_CHNCTRL0_CHN0_BBYTES(__val__)  (((__val__) << 24u) & 0x3f000000u)
#define     HIF_CHNCTRL0_CHN0_LOCK_SHIFT 30u
#define     HIF_CHNCTRL0_CHN0_LOCK_MASK  0x40000000u
#define GET_HIF_CHNCTRL0_CHN0_LOCK(__reg__)  (((__reg__) & 0x40000000) >> 30u)
#define SET_HIF_CHNCTRL0_CHN0_LOCK(__val__)  (((__val__) << 30u) & 0x40000000u)
#define     HIF_CHNCTRL0_CHN0_DISCARD_SHIFT 31u
#define     HIF_CHNCTRL0_CHN0_DISCARD_MASK  0x80000000u
#define GET_HIF_CHNCTRL0_CHN0_DISCARD(__reg__)  (((__reg__) & 0x80000000) >> 31u)
#define SET_HIF_CHNCTRL0_CHN0_DISCARD(__val__)  (((__val__) << 31u) & 0x80000000u)

/** @brief Register definition for @ref HIF_t.Chnctrl0. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief This bit-field reflects the number of bytes that the DMA has transferred to/from the channel buffer. After 64kBytes, the count rolls over. This bit-field clears when chn0_discard is asserted. */
        UInt16 chn0_tbytes;
        /** @brief This bit-field programs the buffer watermark. When the number of bytes currently in the buffer equals the watermark, a wake event / DMA request is generated. */
        UInt8 chn0_wmark:5;
        /** @brief  */
        UInt8 chn0_tsize:2;
        /** @brief Reserved bit 23. Reads as 0. */
        UInt8 reserved_23_23:1;
        /** @brief This bit-field reflects the number of bytes currently in the buffer: 0x00 indicates an empty buffer and 0x20 a full buffer. This bit-field clears when chn0_discard is asserted. */
        UInt8 chn0_bbytes:6;
        /** @brief When asserted '1', this bit blocks host access to the communication channel (upload channel: blocks write data to the channel buffer; download channel: presents '0' on read). */
        UInt8 chn0_lock:1;
        /** @brief When asserted '1', this bit discards the channel buffer content (i.e sets the controls on the core side such that the channel buffer is empty). This bit automatically clears when the buffer content has been discarded. */
        UInt8 chn0_discard:1;
    } bits;
} RegHIFChnctrl0_t;

#define REG_HIF_CHNCTRL1 ((volatile UInt32*)0xf00014) /*  */
#define     HIF_CHNCTRL1_CHN1_TBYTES_SHIFT 0u
#define     HIF_CHNCTRL1_CHN1_TBYTES_MASK  0xffffu
#define GET_HIF_CHNCTRL1_CHN1_TBYTES(__reg__)  (((__reg__) & 0xffff) >> 0u)
#define SET_HIF_CHNCTRL1_CHN1_TBYTES(__val__)  (((__val__) << 0u) & 0xffffu)
#define     HIF_CHNCTRL1_CHN1_WMARK_SHIFT 16u
#define     HIF_CHNCTRL1_CHN1_WMARK_MASK  0x1f0000u
#define GET_HIF_CHNCTRL1_CHN1_WMARK(__reg__)  (((__reg__) & 0x1f0000) >> 16u)
#define SET_HIF_CHNCTRL1_CHN1_WMARK(__val__)  (((__val__) << 16u) & 0x1f0000u)
#define     HIF_CHNCTRL1_CHN1_TSIZE_SHIFT 21u
#define     HIF_CHNCTRL1_CHN1_TSIZE_MASK  0x600000u
#define GET_HIF_CHNCTRL1_CHN1_TSIZE(__reg__)  (((__reg__) & 0x600000) >> 21u)
#define SET_HIF_CHNCTRL1_CHN1_TSIZE(__val__)  (((__val__) << 21u) & 0x600000u)
#define     HIF_CHNCTRL1_CHN1_TSIZE_BYTE 0x0u
#define     HIF_CHNCTRL1_CHN1_TSIZE_HALF_WORD 0x1u
#define     HIF_CHNCTRL1_CHN1_TSIZE_WORD 0x2u

#define     HIF_CHNCTRL1_RESERVED_23_23_SHIFT 23u
#define     HIF_CHNCTRL1_RESERVED_23_23_MASK  0x800000u
#define GET_HIF_CHNCTRL1_RESERVED_23_23(__reg__)  (((__reg__) & 0x800000) >> 23u)
#define SET_HIF_CHNCTRL1_RESERVED_23_23(__val__)  (((__val__) << 23u) & 0x800000u)
#define     HIF_CHNCTRL1_CHN1_BBYTES_SHIFT 24u
#define     HIF_CHNCTRL1_CHN1_BBYTES_MASK  0x3f000000u
#define GET_HIF_CHNCTRL1_CHN1_BBYTES(__reg__)  (((__reg__) & 0x3f000000) >> 24u)
#define SET_HIF_CHNCTRL1_CHN1_BBYTES(__val__)  (((__val__) << 24u) & 0x3f000000u)
#define     HIF_CHNCTRL1_CHN1_LOCK_SHIFT 30u
#define     HIF_CHNCTRL1_CHN1_LOCK_MASK  0x40000000u
#define GET_HIF_CHNCTRL1_CHN1_LOCK(__reg__)  (((__reg__) & 0x40000000) >> 30u)
#define SET_HIF_CHNCTRL1_CHN1_LOCK(__val__)  (((__val__) << 30u) & 0x40000000u)
#define     HIF_CHNCTRL1_CHN1_DISCARD_SHIFT 31u
#define     HIF_CHNCTRL1_CHN1_DISCARD_MASK  0x80000000u
#define GET_HIF_CHNCTRL1_CHN1_DISCARD(__reg__)  (((__reg__) & 0x80000000) >> 31u)
#define SET_HIF_CHNCTRL1_CHN1_DISCARD(__val__)  (((__val__) << 31u) & 0x80000000u)

/** @brief Register definition for @ref HIF_t.Chnctrl1. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief This bit-field reflects the number of bytes that the DMA has transferred to/from the channel buffer. After 64kBytes, the count rolls over. This bit-field clears when chn1_discard is asserted. */
        UInt16 chn1_tbytes;
        /** @brief This bit-field programs the buffer watermark. When the number of bytes currently in the buffer equals the watermark, a wake event / DMA request is generated. */
        UInt8 chn1_wmark:5;
        /** @brief  */
        UInt8 chn1_tsize:2;
        /** @brief Reserved bit 23. Reads as 0. */
        UInt8 reserved_23_23:1;
        /** @brief This bit-field reflects the number of bytes currently in the buffer: 0x00 indicates an empty buffer and 0x20 a full buffer. This bit-field clears when chn1_discard is asserted. */
        UInt8 chn1_bbytes:6;
        /** @brief When asserted '1', this bit blocks host access to the communication channel (upload channel: blocks write data to the channel buffer; download channel: presents '0' on read). */
        UInt8 chn1_lock:1;
        /** @brief When asserted '1', this bit discards the channel buffer content (i.e sets the controls on the core side such that the channel buffer is empty). This bit automatically clears when the buffer content has been discarded. */
        UInt8 chn1_discard:1;
    } bits;
} RegHIFChnctrl1_t;

#define REG_HIF_CHNCTRL2 ((volatile UInt32*)0xf00018) /*  */
#define     HIF_CHNCTRL2_CHN2_TBYTES_SHIFT 0u
#define     HIF_CHNCTRL2_CHN2_TBYTES_MASK  0xffffu
#define GET_HIF_CHNCTRL2_CHN2_TBYTES(__reg__)  (((__reg__) & 0xffff) >> 0u)
#define SET_HIF_CHNCTRL2_CHN2_TBYTES(__val__)  (((__val__) << 0u) & 0xffffu)
#define     HIF_CHNCTRL2_CHN2_WMARK_SHIFT 16u
#define     HIF_CHNCTRL2_CHN2_WMARK_MASK  0x1f0000u
#define GET_HIF_CHNCTRL2_CHN2_WMARK(__reg__)  (((__reg__) & 0x1f0000) >> 16u)
#define SET_HIF_CHNCTRL2_CHN2_WMARK(__val__)  (((__val__) << 16u) & 0x1f0000u)
#define     HIF_CHNCTRL2_CHN2_TSIZE_SHIFT 21u
#define     HIF_CHNCTRL2_CHN2_TSIZE_MASK  0x600000u
#define GET_HIF_CHNCTRL2_CHN2_TSIZE(__reg__)  (((__reg__) & 0x600000) >> 21u)
#define SET_HIF_CHNCTRL2_CHN2_TSIZE(__val__)  (((__val__) << 21u) & 0x600000u)
#define     HIF_CHNCTRL2_CHN2_TSIZE_BYTE 0x0u
#define     HIF_CHNCTRL2_CHN2_TSIZE_HALF_WORD 0x1u
#define     HIF_CHNCTRL2_CHN2_TSIZE_WORD 0x2u

#define     HIF_CHNCTRL2_RESERVED_23_23_SHIFT 23u
#define     HIF_CHNCTRL2_RESERVED_23_23_MASK  0x800000u
#define GET_HIF_CHNCTRL2_RESERVED_23_23(__reg__)  (((__reg__) & 0x800000) >> 23u)
#define SET_HIF_CHNCTRL2_RESERVED_23_23(__val__)  (((__val__) << 23u) & 0x800000u)
#define     HIF_CHNCTRL2_CHN2_BBYTES_SHIFT 24u
#define     HIF_CHNCTRL2_CHN2_BBYTES_MASK  0x3f000000u
#define GET_HIF_CHNCTRL2_CHN2_BBYTES(__reg__)  (((__reg__) & 0x3f000000) >> 24u)
#define SET_HIF_CHNCTRL2_CHN2_BBYTES(__val__)  (((__val__) << 24u) & 0x3f000000u)
#define     HIF_CHNCTRL2_CHN2_LOCK_SHIFT 30u
#define     HIF_CHNCTRL2_CHN2_LOCK_MASK  0x40000000u
#define GET_HIF_CHNCTRL2_CHN2_LOCK(__reg__)  (((__reg__) & 0x40000000) >> 30u)
#define SET_HIF_CHNCTRL2_CHN2_LOCK(__val__)  (((__val__) << 30u) & 0x40000000u)
#define     HIF_CHNCTRL2_CHN2_DISCARD_SHIFT 31u
#define     HIF_CHNCTRL2_CHN2_DISCARD_MASK  0x80000000u
#define GET_HIF_CHNCTRL2_CHN2_DISCARD(__reg__)  (((__reg__) & 0x80000000) >> 31u)
#define SET_HIF_CHNCTRL2_CHN2_DISCARD(__val__)  (((__val__) << 31u) & 0x80000000u)

/** @brief Register definition for @ref HIF_t.Chnctrl2. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief This bit-field reflects the number of bytes that the DMA has transferred to/from the channel buffer. After 64kBytes, the count rolls over. This bit-field clears when chn2_discard is asserted. */
        UInt16 chn2_tbytes;
        /** @brief This bit-field programs the buffer watermark. When the number of bytes currently in the buffer equals the watermark, a wake event / DMA request is generated. */
        UInt8 chn2_wmark:5;
        /** @brief  */
        UInt8 chn2_tsize:2;
        /** @brief Reserved bit 23. Reads as 0. */
        UInt8 reserved_23_23:1;
        /** @brief This bit-field reflects the number of bytes currently in the buffer: 0x00 indicates an empty buffer and 0x20 a full buffer. This bit-field clears when chn2_discard is asserted. */
        UInt8 chn2_bbytes:6;
        /** @brief When asserted '1', this bit blocks host access to the communication channel (upload channel: blocks write data to the channel buffer; download channel: presents '0' on read). */
        UInt8 chn2_lock:1;
        /** @brief When asserted '1', this bit discards the channel buffer content (i.e sets the controls on the core side such that the channel buffer is empty). This bit automatically clears when the buffer content has been discarded. */
        UInt8 chn2_discard:1;
    } bits;
} RegHIFChnctrl2_t;

#define REG_HIF_CHNCTRL3 ((volatile UInt32*)0xf0001c) /*  */
#define     HIF_CHNCTRL3_CHN3_TBYTES_SHIFT 0u
#define     HIF_CHNCTRL3_CHN3_TBYTES_MASK  0xffffu
#define GET_HIF_CHNCTRL3_CHN3_TBYTES(__reg__)  (((__reg__) & 0xffff) >> 0u)
#define SET_HIF_CHNCTRL3_CHN3_TBYTES(__val__)  (((__val__) << 0u) & 0xffffu)
#define     HIF_CHNCTRL3_CHN3_WMARK_SHIFT 16u
#define     HIF_CHNCTRL3_CHN3_WMARK_MASK  0x1f0000u
#define GET_HIF_CHNCTRL3_CHN3_WMARK(__reg__)  (((__reg__) & 0x1f0000) >> 16u)
#define SET_HIF_CHNCTRL3_CHN3_WMARK(__val__)  (((__val__) << 16u) & 0x1f0000u)
#define     HIF_CHNCTRL3_CHN3_TSIZE_SHIFT 21u
#define     HIF_CHNCTRL3_CHN3_TSIZE_MASK  0x600000u
#define GET_HIF_CHNCTRL3_CHN3_TSIZE(__reg__)  (((__reg__) & 0x600000) >> 21u)
#define SET_HIF_CHNCTRL3_CHN3_TSIZE(__val__)  (((__val__) << 21u) & 0x600000u)
#define     HIF_CHNCTRL3_CHN3_TSIZE_BYTE 0x0u
#define     HIF_CHNCTRL3_CHN3_TSIZE_HALF_WORD 0x1u
#define     HIF_CHNCTRL3_CHN3_TSIZE_WORD 0x2u

#define     HIF_CHNCTRL3_RESERVED_23_23_SHIFT 23u
#define     HIF_CHNCTRL3_RESERVED_23_23_MASK  0x800000u
#define GET_HIF_CHNCTRL3_RESERVED_23_23(__reg__)  (((__reg__) & 0x800000) >> 23u)
#define SET_HIF_CHNCTRL3_RESERVED_23_23(__val__)  (((__val__) << 23u) & 0x800000u)
#define     HIF_CHNCTRL3_CHN3_BBYTES_SHIFT 24u
#define     HIF_CHNCTRL3_CHN3_BBYTES_MASK  0x3f000000u
#define GET_HIF_CHNCTRL3_CHN3_BBYTES(__reg__)  (((__reg__) & 0x3f000000) >> 24u)
#define SET_HIF_CHNCTRL3_CHN3_BBYTES(__val__)  (((__val__) << 24u) & 0x3f000000u)
#define     HIF_CHNCTRL3_CHN3_LOCK_SHIFT 30u
#define     HIF_CHNCTRL3_CHN3_LOCK_MASK  0x40000000u
#define GET_HIF_CHNCTRL3_CHN3_LOCK(__reg__)  (((__reg__) & 0x40000000) >> 30u)
#define SET_HIF_CHNCTRL3_CHN3_LOCK(__val__)  (((__val__) << 30u) & 0x40000000u)
#define     HIF_CHNCTRL3_CHN3_DISCARD_SHIFT 31u
#define     HIF_CHNCTRL3_CHN3_DISCARD_MASK  0x80000000u
#define GET_HIF_CHNCTRL3_CHN3_DISCARD(__reg__)  (((__reg__) & 0x80000000) >> 31u)
#define SET_HIF_CHNCTRL3_CHN3_DISCARD(__val__)  (((__val__) << 31u) & 0x80000000u)

/** @brief Register definition for @ref HIF_t.Chnctrl3. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief This bit-field reflects the number of bytes that the DMA has transferred to/from the channel buffer. After 64kBytes, the count rolls over. This bit-field clears when chn3_discard is asserted. */
        UInt16 chn3_tbytes;
        /** @brief This bit-field programs the buffer watermark. When the number of bytes currently in the buffer equals the watermark, a wake event / DMA request is generated. */
        UInt8 chn3_wmark:5;
        /** @brief  */
        UInt8 chn3_tsize:2;
        /** @brief Reserved bit 23. Reads as 0. */
        UInt8 reserved_23_23:1;
        /** @brief This bit-field reflects the number of bytes currently in the buffer: 0x00 indicates an empty buffer and 0x20 a full buffer. This bit-field clears when chn3_discard is asserted. */
        UInt8 chn3_bbytes:6;
        /** @brief When asserted '1', this bit blocks host access to the communication channel (upload channel: blocks write data to the channel buffer; download channel: presents '0' on read). */
        UInt8 chn3_lock:1;
        /** @brief When asserted '1', this bit discards the channel buffer content (i.e sets the controls on the core side such that the channel buffer is empty). This bit automatically clears when the buffer content has been discarded. */
        UInt8 chn3_discard:1;
    } bits;
} RegHIFChnctrl3_t;

#define REG_HIF_I2CADDR ((volatile UInt32*)0xf00020) /*  */
#define     HIF_I2CADDR_I2C_DEV_ADDR_SHIFT 0u
#define     HIF_I2CADDR_I2C_DEV_ADDR_MASK  0x7fu
#define GET_HIF_I2CADDR_I2C_DEV_ADDR(__reg__)  (((__reg__) & 0x7f) >> 0u)
#define SET_HIF_I2CADDR_I2C_DEV_ADDR(__val__)  (((__val__) << 0u) & 0x7fu)
#define     HIF_I2CADDR_RESERVED_31_7_SHIFT 7u
#define     HIF_I2CADDR_RESERVED_31_7_MASK  0xffffff80u
#define GET_HIF_I2CADDR_RESERVED_31_7(__reg__)  (((__reg__) & 0xffffff80) >> 7u)
#define SET_HIF_I2CADDR_RESERVED_31_7(__val__)  (((__val__) << 7u) & 0xffffff80u)

/** @brief Register definition for @ref HIF_t.I2caddr. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief This bit-field holds the I2C device address by which the sensor hub will be identified when the host protocol is locked to I2C. This bit-field has no meaning for SPI host protocol. */
        UInt32 i2c_dev_addr:7;
        /** @brief Reserved bits 31 to 7. Reads as 0. */
        UInt32 reserved_31_7:25;
    } bits;
} RegHIFI2caddr_t;

#define REG_HIF_CRC ((volatile UInt32*)0xf00024) /*  */
/** @brief Register definition for @ref HIF_t.Crc. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];
} RegHIFCrc_t;

#define REG_HIF_HOSTINFO0 ((volatile UInt32*)0xf00028) /*  */
#define     HIF_HOSTINFO0_RD_ADDR_SHIFT 0u
#define     HIF_HOSTINFO0_RD_ADDR_MASK  0x7fu
#define GET_HIF_HOSTINFO0_RD_ADDR(__reg__)  (((__reg__) & 0x7f) >> 0u)
#define SET_HIF_HOSTINFO0_RD_ADDR(__val__)  (((__val__) << 0u) & 0x7fu)
#define     HIF_HOSTINFO0_RESERVED_7_7_SHIFT 7u
#define     HIF_HOSTINFO0_RESERVED_7_7_MASK  0x80u
#define GET_HIF_HOSTINFO0_RESERVED_7_7(__reg__)  (((__reg__) & 0x80) >> 7u)
#define SET_HIF_HOSTINFO0_RESERVED_7_7(__val__)  (((__val__) << 7u) & 0x80u)
#define     HIF_HOSTINFO0_RD_DATA_SHIFT 8u
#define     HIF_HOSTINFO0_RD_DATA_MASK  0xff00u
#define GET_HIF_HOSTINFO0_RD_DATA(__reg__)  (((__reg__) & 0xff00) >> 8u)
#define SET_HIF_HOSTINFO0_RD_DATA(__val__)  (((__val__) << 8u) & 0xff00u)
#define     HIF_HOSTINFO0_WR_ADDR_SHIFT 16u
#define     HIF_HOSTINFO0_WR_ADDR_MASK  0x7f0000u
#define GET_HIF_HOSTINFO0_WR_ADDR(__reg__)  (((__reg__) & 0x7f0000) >> 16u)
#define SET_HIF_HOSTINFO0_WR_ADDR(__val__)  (((__val__) << 16u) & 0x7f0000u)
#define     HIF_HOSTINFO0_RESERVED_23_23_SHIFT 23u
#define     HIF_HOSTINFO0_RESERVED_23_23_MASK  0x800000u
#define GET_HIF_HOSTINFO0_RESERVED_23_23(__reg__)  (((__reg__) & 0x800000) >> 23u)
#define SET_HIF_HOSTINFO0_RESERVED_23_23(__val__)  (((__val__) << 23u) & 0x800000u)
#define     HIF_HOSTINFO0_WR_DATA_SHIFT 24u
#define     HIF_HOSTINFO0_WR_DATA_MASK  0xff000000u
#define GET_HIF_HOSTINFO0_WR_DATA(__reg__)  (((__reg__) & 0xff000000) >> 24u)
#define SET_HIF_HOSTINFO0_WR_DATA(__val__)  (((__val__) << 24u) & 0xff000000u)

/** @brief Register definition for @ref HIF_t.Hostinfo0. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief This bit-field holds the register address from which the last host read took place. */
        UInt8 rd_addr:7;
        /** @brief Reserved bit 7. Reads as 0. */
        UInt8 reserved_7_7:1;
        /** @brief This bit-field holds the data read from the register address given by rd_addr for the last host read. */
        UInt8 rd_data;
        /** @brief This bit-field holds the register address to which the last host write took place. */
        UInt8 wr_addr:7;
        /** @brief Reserved bit 23. Reads as 0. */
        UInt8 reserved_23_23:1;
        /** @brief This bit-field holds the data written to the register address given by wr_addr for the last host write. */
        UInt8 wr_data;
    } bits;
} RegHIFHostinfo0_t;

#define REG_HIF_HOSTINFO1 ((volatile UInt32*)0xf0002c) /*  */
#define     HIF_HOSTINFO1_HOST_ACTIVE_SHIFT 0u
#define     HIF_HOSTINFO1_HOST_ACTIVE_MASK  0x1u
#define GET_HIF_HOSTINFO1_HOST_ACTIVE(__reg__)  (((__reg__) & 0x1) >> 0u)
#define SET_HIF_HOSTINFO1_HOST_ACTIVE(__val__)  (((__val__) << 0u) & 0x1u)
#define     HIF_HOSTINFO1_HOST_PROTO_SHIFT 1u
#define     HIF_HOSTINFO1_HOST_PROTO_MASK  0x2u
#define GET_HIF_HOSTINFO1_HOST_PROTO(__reg__)  (((__reg__) & 0x2) >> 1u)
#define SET_HIF_HOSTINFO1_HOST_PROTO(__val__)  (((__val__) << 1u) & 0x2u)
#define     HIF_HOSTINFO1_HOST_PROTO_I2C_MODE 0x0u
#define     HIF_HOSTINFO1_HOST_PROTO_SPI_MODE 0x1u

#define     HIF_HOSTINFO1_RESERVED_7_2_SHIFT 2u
#define     HIF_HOSTINFO1_RESERVED_7_2_MASK  0xfcu
#define GET_HIF_HOSTINFO1_RESERVED_7_2(__reg__)  (((__reg__) & 0xfc) >> 2u)
#define SET_HIF_HOSTINFO1_RESERVED_7_2(__val__)  (((__val__) << 2u) & 0xfcu)
#define     HIF_HOSTINFO1_HOST_CHN_ACC_SHIFT 8u
#define     HIF_HOSTINFO1_HOST_CHN_ACC_MASK  0x300u
#define GET_HIF_HOSTINFO1_HOST_CHN_ACC(__reg__)  (((__reg__) & 0x300) >> 8u)
#define SET_HIF_HOSTINFO1_HOST_CHN_ACC(__val__)  (((__val__) << 8u) & 0x300u)
#define     HIF_HOSTINFO1_RESERVED_31_10_SHIFT 10u
#define     HIF_HOSTINFO1_RESERVED_31_10_MASK  0xfffffc00u
#define GET_HIF_HOSTINFO1_RESERVED_31_10(__reg__)  (((__reg__) & 0xfffffc00) >> 10u)
#define SET_HIF_HOSTINFO1_RESERVED_31_10(__val__)  (((__val__) << 10u) & 0xfffffc00u)

/** @brief Register definition for @ref HIF_t.Hostinfo1. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief When '1', this bit indicates that the host is conducting transactions via the host interface. */
        UInt8 host_active:1;
        /** @brief  */
        UInt8 host_proto:1;
        /** @brief Reserved bits 7 to 2. Reads as 0. */
        UInt8 reserved_7_2:6;
        /** @brief  */
        UInt8 host_chn_acc:2;
        /** @brief Reserved bits 31 to 10. Reads as 0. */
        UInt8 reserved_15_10:6;
        /** @brief Reserved bits 31 to 10. Reads as 0. */
        UInt8 reserved_23_16;
        /** @brief Reserved bits 31 to 10. Reads as 0. */
        UInt8 reserved_31_24;
    } bits;
} RegHIFHostinfo1_t;

/** @brief Component definition for @ref HIF. */
typedef struct {
    /** @brief  */
    RegHIFChn0_t Chn0;

    /** @brief  */
    RegHIFChn1_t Chn1;

    /** @brief  */
    RegHIFChn2_t Chn2;

    /** @brief  */
    RegHIFChn3_t Chn3;

    /** @brief  */
    RegHIFChnctrl0_t Chnctrl0;

    /** @brief  */
    RegHIFChnctrl1_t Chnctrl1;

    /** @brief  */
    RegHIFChnctrl2_t Chnctrl2;

    /** @brief  */
    RegHIFChnctrl3_t Chnctrl3;

    /** @brief  */
    RegHIFI2caddr_t I2caddr;

    /** @brief  */
    RegHIFCrc_t Crc;

    /** @brief  */
    RegHIFHostinfo0_t Hostinfo0;

    /** @brief  */
    RegHIFHostinfo1_t Hostinfo1;

} HIF_t;

/** @brief  */
extern volatile HIF_t HIF;


#define REG_HOST_RESERVED_0 ((volatile UInt8*)0xf00058) /* 8 bit reserved register */
/** @brief Register definition for @ref HOST_t.Reserved0. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;
} RegHOSTReserved0_t;

#define REG_HOST_CHIP_CONTROL ((volatile UInt8*)0xf00059) /* Host writeable register to control run level during boot; also can clear error regs. */
#define     HOST_CHIP_CONTROL_TURBO_MODE_DISABLE_SHIFT 0u
#define     HOST_CHIP_CONTROL_TURBO_MODE_DISABLE_MASK  0x1u
#define GET_HOST_CHIP_CONTROL_TURBO_MODE_DISABLE(__reg__)  (((__reg__) & 0x1) >> 0u)
#define SET_HOST_CHIP_CONTROL_TURBO_MODE_DISABLE(__val__)  (((__val__) << 0u) & 0x1u)
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

#define REG_HOST_HOST_INTERFACE_CONTROL ((volatile UInt8*)0xf0005a) /* Host writeable register to cause immediate actions. */
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

#define REG_HOST_HOST_INTERRUPT_CONTROL ((volatile UInt8*)0xf0005b) /*  */
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
#define     HOST_HOST_INTERRUPT_CONTROL_EDGE_INTERRUPT_SHIFT 6u
#define     HOST_HOST_INTERRUPT_CONTROL_EDGE_INTERRUPT_MASK  0x40u
#define GET_HOST_HOST_INTERRUPT_CONTROL_EDGE_INTERRUPT(__reg__)  (((__reg__) & 0x40) >> 6u)
#define SET_HOST_HOST_INTERRUPT_CONTROL_EDGE_INTERRUPT(__val__)  (((__val__) << 6u) & 0x40u)
#define     HOST_HOST_INTERRUPT_CONTROL_OPEN_DRAIN_INTERRUPT_SHIFT 7u
#define     HOST_HOST_INTERRUPT_CONTROL_OPEN_DRAIN_INTERRUPT_MASK  0x80u
#define GET_HOST_HOST_INTERRUPT_CONTROL_OPEN_DRAIN_INTERRUPT(__reg__)  (((__reg__) & 0x80) >> 7u)
#define SET_HOST_HOST_INTERRUPT_CONTROL_OPEN_DRAIN_INTERRUPT(__val__)  (((__val__) << 7u) & 0x80u)

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

#define REG_HOST_GP1 ((volatile UInt32*)0xf0005c) /*  */
#define     HOST_GP1_HOST_GP1_7_0_SHIFT 0u
#define     HOST_GP1_HOST_GP1_7_0_MASK  0xffu
#define GET_HOST_GP1_HOST_GP1_7_0(__reg__)  (((__reg__) & 0xff) >> 0u)
#define SET_HOST_GP1_HOST_GP1_7_0(__val__)  (((__val__) << 0u) & 0xffu)
#define     HOST_GP1_HOST_GP1_15_8_SHIFT 8u
#define     HOST_GP1_HOST_GP1_15_8_MASK  0xff00u
#define GET_HOST_GP1_HOST_GP1_15_8(__reg__)  (((__reg__) & 0xff00) >> 8u)
#define SET_HOST_GP1_HOST_GP1_15_8(__val__)  (((__val__) << 8u) & 0xff00u)
#define     HOST_GP1_HOST_GP1_23_16_SHIFT 16u
#define     HOST_GP1_HOST_GP1_23_16_MASK  0xff0000u
#define GET_HOST_GP1_HOST_GP1_23_16(__reg__)  (((__reg__) & 0xff0000) >> 16u)
#define SET_HOST_GP1_HOST_GP1_23_16(__val__)  (((__val__) << 16u) & 0xff0000u)
#define     HOST_GP1_HOST_GP1_31_24_SHIFT 24u
#define     HOST_GP1_HOST_GP1_31_24_MASK  0xff000000u
#define GET_HOST_GP1_HOST_GP1_31_24(__reg__)  (((__reg__) & 0xff000000) >> 24u)
#define SET_HOST_GP1_HOST_GP1_31_24(__val__)  (((__val__) << 24u) & 0xff000000u)

/** @brief Register definition for @ref HOST_t.Gp1. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief Byte-oriented, general-purpose register for communicating information between sensor hub FW and host. */
        UInt8 host_gp1_7_0;
        /** @brief Byte-oriented, general-purpose register for communicating information between sensor hub FW and host. */
        UInt8 host_gp1_15_8;
        /** @brief Byte-oriented, general-purpose register for communicating information between sensor hub FW and host. */
        UInt8 host_gp1_23_16;
        /** @brief Byte-oriented, general-purpose register for communicating information between sensor hub FW and host. */
        UInt8 host_gp1_31_24;
    } bits;
} RegHOSTGp1_t;

#define REG_HOST_GP2 ((volatile UInt32*)0xf00060) /*  */
#define     HOST_GP2_HOST_GP2_7_0_SHIFT 0u
#define     HOST_GP2_HOST_GP2_7_0_MASK  0xffu
#define GET_HOST_GP2_HOST_GP2_7_0(__reg__)  (((__reg__) & 0xff) >> 0u)
#define SET_HOST_GP2_HOST_GP2_7_0(__val__)  (((__val__) << 0u) & 0xffu)
#define     HOST_GP2_HOST_GP2_15_8_SHIFT 8u
#define     HOST_GP2_HOST_GP2_15_8_MASK  0xff00u
#define GET_HOST_GP2_HOST_GP2_15_8(__reg__)  (((__reg__) & 0xff00) >> 8u)
#define SET_HOST_GP2_HOST_GP2_15_8(__val__)  (((__val__) << 8u) & 0xff00u)
#define     HOST_GP2_HOST_GP2_23_16_SHIFT 16u
#define     HOST_GP2_HOST_GP2_23_16_MASK  0xff0000u
#define GET_HOST_GP2_HOST_GP2_23_16(__reg__)  (((__reg__) & 0xff0000) >> 16u)
#define SET_HOST_GP2_HOST_GP2_23_16(__val__)  (((__val__) << 16u) & 0xff0000u)
#define     HOST_GP2_HOST_GP2_31_24_SHIFT 24u
#define     HOST_GP2_HOST_GP2_31_24_MASK  0xff000000u
#define GET_HOST_GP2_HOST_GP2_31_24(__reg__)  (((__reg__) & 0xff000000) >> 24u)
#define SET_HOST_GP2_HOST_GP2_31_24(__val__)  (((__val__) << 24u) & 0xff000000u)

/** @brief Register definition for @ref HOST_t.Gp2. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief Byte-oriented, general-purpose register for communicating information between sensor hub FW and host. */
        UInt8 host_gp2_7_0;
        /** @brief Byte-oriented, general-purpose register for communicating information between sensor hub FW and host. */
        UInt8 host_gp2_15_8;
        /** @brief Byte-oriented, general-purpose register for communicating information between sensor hub FW and host. */
        UInt8 host_gp2_23_16;
        /** @brief Byte-oriented, general-purpose register for communicating information between sensor hub FW and host. */
        UInt8 host_gp2_31_24;
    } bits;
} RegHOSTGp2_t;

#define REG_HOST_GP3 ((volatile UInt32*)0xf00064) /*  */
#define     HOST_GP3_HOST_GP3_7_0_SHIFT 0u
#define     HOST_GP3_HOST_GP3_7_0_MASK  0xffu
#define GET_HOST_GP3_HOST_GP3_7_0(__reg__)  (((__reg__) & 0xff) >> 0u)
#define SET_HOST_GP3_HOST_GP3_7_0(__val__)  (((__val__) << 0u) & 0xffu)
#define     HOST_GP3_HOST_GP3_15_8_SHIFT 8u
#define     HOST_GP3_HOST_GP3_15_8_MASK  0xff00u
#define GET_HOST_GP3_HOST_GP3_15_8(__reg__)  (((__reg__) & 0xff00) >> 8u)
#define SET_HOST_GP3_HOST_GP3_15_8(__val__)  (((__val__) << 8u) & 0xff00u)
#define     HOST_GP3_HOST_GP3_23_16_SHIFT 16u
#define     HOST_GP3_HOST_GP3_23_16_MASK  0xff0000u
#define GET_HOST_GP3_HOST_GP3_23_16(__reg__)  (((__reg__) & 0xff0000) >> 16u)
#define SET_HOST_GP3_HOST_GP3_23_16(__val__)  (((__val__) << 16u) & 0xff0000u)
#define     HOST_GP3_HOST_GP3_31_24_SHIFT 24u
#define     HOST_GP3_HOST_GP3_31_24_MASK  0xff000000u
#define GET_HOST_GP3_HOST_GP3_31_24(__reg__)  (((__reg__) & 0xff000000) >> 24u)
#define SET_HOST_GP3_HOST_GP3_31_24(__val__)  (((__val__) << 24u) & 0xff000000u)

/** @brief Register definition for @ref HOST_t.Gp3. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief Byte-oriented, general-purpose register for communicating information between sensor hub FW and host. */
        UInt8 host_gp3_7_0;
        /** @brief Byte-oriented, general-purpose register for communicating information between sensor hub FW and host. */
        UInt8 host_gp3_15_8;
        /** @brief Byte-oriented, general-purpose register for communicating information between sensor hub FW and host. */
        UInt8 host_gp3_23_16;
        /** @brief Byte-oriented, general-purpose register for communicating information between sensor hub FW and host. */
        UInt8 host_gp3_31_24;
    } bits;
} RegHOSTGp3_t;

/** @brief Component definition for @ref HOST. */
typedef struct {
    /** @brief 8 bit reserved register */
    RegHOSTReserved0_t Reserved0;

    /** @brief Host writeable register to control run level during boot; also can clear error regs. */
    RegHOSTChipControl_t ChipControl;

    /** @brief Host writeable register to cause immediate actions. */
    RegHOSTHostInterfaceControl_t HostInterfaceControl;

    /** @brief  */
    RegHOSTHostInterruptControl_t HostInterruptControl;

    /** @brief  */
    RegHOSTGp1_t Gp1;

    /** @brief  */
    RegHOSTGp2_t Gp2;

    /** @brief  */
    RegHOSTGp3_t Gp3;

} HOST_t;

/** @brief  */
extern volatile HOST_t HOST;


#define REG_PROC_IDINFO ((volatile UInt32*)0xf00034) /*  */
#define     PROC_IDINFO_PROD_ID_SHIFT 0u
#define     PROC_IDINFO_PROD_ID_MASK  0xffu
#define GET_PROC_IDINFO_PROD_ID(__reg__)  (((__reg__) & 0xff) >> 0u)
#define SET_PROC_IDINFO_PROD_ID(__val__)  (((__val__) << 0u) & 0xffu)
#define     PROC_IDINFO_REV_ID_SHIFT 8u
#define     PROC_IDINFO_REV_ID_MASK  0x3f00u
#define GET_PROC_IDINFO_REV_ID(__reg__)  (((__reg__) & 0x3f00) >> 8u)
#define SET_PROC_IDINFO_REV_ID(__val__)  (((__val__) << 8u) & 0x3f00u)
#define     PROC_IDINFO_CLOCK_STRETCH_EN_SHIFT 14u
#define     PROC_IDINFO_CLOCK_STRETCH_EN_MASK  0x4000u
#define GET_PROC_IDINFO_CLOCK_STRETCH_EN(__reg__)  (((__reg__) & 0x4000) >> 14u)
#define SET_PROC_IDINFO_CLOCK_STRETCH_EN(__val__)  (((__val__) << 14u) & 0x4000u)
#define     PROC_IDINFO_HIF_CH_REDIRECT_SHIFT 15u
#define     PROC_IDINFO_HIF_CH_REDIRECT_MASK  0x8000u
#define GET_PROC_IDINFO_HIF_CH_REDIRECT(__reg__)  (((__reg__) & 0x8000) >> 15u)
#define SET_PROC_IDINFO_HIF_CH_REDIRECT(__val__)  (((__val__) << 15u) & 0x8000u)
#define     PROC_IDINFO_RESERVED_31_16_SHIFT 16u
#define     PROC_IDINFO_RESERVED_31_16_MASK  0xffff0000u
#define GET_PROC_IDINFO_RESERVED_31_16(__reg__)  (((__reg__) & 0xffff0000) >> 16u)
#define SET_PROC_IDINFO_RESERVED_31_16(__val__)  (((__val__) << 16u) & 0xffff0000u)

/** @brief Register definition for @ref PROC_t.Idinfo. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief Chip's product ID. */
        UInt8 prod_id;
        /** @brief Chip's revision ID. */
        UInt8 rev_id:6;
        /** @brief Enables clock stretching in the slave id ACK phase if HIFComm interrupt has not been serviced. */
        UInt8 clock_stretch_en:1;
        /** @brief Redirects all i2c writes to channel 0, and all i2c reads to channel 1. */
        UInt8 hif_ch_redirect:1;
        /** @brief Reserved bits 31 to 16. Reads as 0. */
        UInt16 reserved_31_16;
    } bits;
} RegPROCIdinfo_t;

#define REG_PROC_VERSION_0 ((volatile UInt32*)0xf00038) /*  */
#define     PROC_VERSION_0_ROM_VERSION_SHIFT 0u
#define     PROC_VERSION_0_ROM_VERSION_MASK  0xffffu
#define GET_PROC_VERSION_0_ROM_VERSION(__reg__)  (((__reg__) & 0xffff) >> 0u)
#define SET_PROC_VERSION_0_ROM_VERSION(__val__)  (((__val__) << 0u) & 0xffffu)
#define     PROC_VERSION_0_KERNEL_VERSION_SHIFT 16u
#define     PROC_VERSION_0_KERNEL_VERSION_MASK  0xffff0000u
#define GET_PROC_VERSION_0_KERNEL_VERSION(__reg__)  (((__reg__) & 0xffff0000) >> 16u)
#define SET_PROC_VERSION_0_KERNEL_VERSION(__val__)  (((__val__) << 16u) & 0xffff0000u)

/** @brief Register definition for @ref PROC_t.Version0. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief The EM build number corresponding to the firmware placed in ROM. */
        UInt16 rom_version;
        /** @brief The kernel firmware version, if any.  If none is present, this will read back 0. */
        UInt16 kernel_version;
    } bits;
} RegPROCVersion0_t;

#define REG_PROC_VERSION_1 ((volatile UInt16*)0xf0003c) /*  */
#define     PROC_VERSION_1_USER_VERSION_SHIFT 0u
#define     PROC_VERSION_1_USER_VERSION_MASK  0xffffu
#define GET_PROC_VERSION_1_USER_VERSION(__reg__)  (((__reg__) & 0xffff) >> 0u)
#define SET_PROC_VERSION_1_USER_VERSION(__val__)  (((__val__) << 0u) & 0xffffu)

/** @brief Register definition for @ref PROC_t.Version1. */
 typedef union {
    /** @brief 16bit direct register access. */
    UInt16 r16;
    /** @brief 8bit direct register access. */
    UInt8 r8[2];

    struct {
        /** @brief The user firmware version, if any.  If none is present, this will read back 0. */
        UInt16 user_version;
    } bits;
} RegPROCVersion1_t;

#define REG_PROC_FEATURE_STATUS ((volatile UInt8*)0xf0003e) /* General purpose registers for communicating information between sensor hub FW and host. */
#define     PROC_FEATURE_STATUS_FLASH_DESCRIPTOR_LOADED_SHIFT 0u
#define     PROC_FEATURE_STATUS_FLASH_DESCRIPTOR_LOADED_MASK  0x1u
#define GET_PROC_FEATURE_STATUS_FLASH_DESCRIPTOR_LOADED(__reg__)  (((__reg__) & 0x1) >> 0u)
#define SET_PROC_FEATURE_STATUS_FLASH_DESCRIPTOR_LOADED(__val__)  (((__val__) << 0u) & 0x1u)
#define     PROC_FEATURE_STATUS_RESERVED_1_SHIFT 1u
#define     PROC_FEATURE_STATUS_RESERVED_1_MASK  0x2u
#define GET_PROC_FEATURE_STATUS_RESERVED_1(__reg__)  (((__reg__) & 0x2) >> 1u)
#define SET_PROC_FEATURE_STATUS_RESERVED_1(__val__)  (((__val__) << 1u) & 0x2u)
#define     PROC_FEATURE_STATUS_HOST_INT_ID_SHIFT 2u
#define     PROC_FEATURE_STATUS_HOST_INT_ID_MASK  0x1cu
#define GET_PROC_FEATURE_STATUS_HOST_INT_ID(__reg__)  (((__reg__) & 0x1c) >> 2u)
#define SET_PROC_FEATURE_STATUS_HOST_INT_ID(__val__)  (((__val__) << 2u) & 0x1cu)
#define     PROC_FEATURE_STATUS_HOST_INT_ID_KITKAT 0x0u
#define     PROC_FEATURE_STATUS_HOST_INT_ID_LOLLIPOP 0x1u
#define     PROC_FEATURE_STATUS_HOST_INT_ID_MARSHMALLOW 0x2u
#define     PROC_FEATURE_STATUS_HOST_INT_ID_NOUGAT 0x3u
#define     PROC_FEATURE_STATUS_HOST_INT_ID_REGISTER 0x7u

#define     PROC_FEATURE_STATUS_ALGO_ID_SHIFT 5u
#define     PROC_FEATURE_STATUS_ALGO_ID_MASK  0xe0u
#define GET_PROC_FEATURE_STATUS_ALGO_ID(__reg__)  (((__reg__) & 0xe0) >> 5u)
#define SET_PROC_FEATURE_STATUS_ALGO_ID(__val__)  (((__val__) << 5u) & 0xe0u)
#define     PROC_FEATURE_STATUS_ALGO_ID_CUSTOM 0x0u
#define     PROC_FEATURE_STATUS_ALGO_ID_BSX3 0x1u
#define     PROC_FEATURE_STATUS_ALGO_ID_BSX4 0x2u


/** @brief Register definition for @ref PROC_t.FeatureStatus. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;

    struct {
        /** @brief A valid flash descriptor was located on the flash device and has been loaded. */
        UInt8 flash_descriptor_loaded:1;
        /** @brief Reserved bit 1. Reads as 0. */
        UInt8 reserved_1:1;
        /** @brief Host Interface ID. 0: Android K. 1: Android L. 2: Android M. 3: Android N. 4: Android O. */
        UInt8 host_int_id:3;
        /** @brief Algorithm ID. 2: BSX4 */
        UInt8 algo_id:3;
    } bits;
} RegPROCFeatureStatus_t;

#define REG_PROC_BOOT_STATUS ((volatile UInt8*)0xf0003f) /* General purpose registers for communicating information between sensor hub FW and host. */
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

#define REG_PROC_HOST_TIMESTAMP_0 ((volatile UInt32*)0xf00040) /*  */
#define     PROC_HOST_TIMESTAMP_0_TIMESTAMP_0_SHIFT 0u
#define     PROC_HOST_TIMESTAMP_0_TIMESTAMP_0_MASK  0xffu
#define GET_PROC_HOST_TIMESTAMP_0_TIMESTAMP_0(__reg__)  (((__reg__) & 0xff) >> 0u)
#define SET_PROC_HOST_TIMESTAMP_0_TIMESTAMP_0(__val__)  (((__val__) << 0u) & 0xffu)
#define     PROC_HOST_TIMESTAMP_0_TIMESTAMP_1_SHIFT 8u
#define     PROC_HOST_TIMESTAMP_0_TIMESTAMP_1_MASK  0xff00u
#define GET_PROC_HOST_TIMESTAMP_0_TIMESTAMP_1(__reg__)  (((__reg__) & 0xff00) >> 8u)
#define SET_PROC_HOST_TIMESTAMP_0_TIMESTAMP_1(__val__)  (((__val__) << 8u) & 0xff00u)
#define     PROC_HOST_TIMESTAMP_0_TIMESTAMP_2_SHIFT 16u
#define     PROC_HOST_TIMESTAMP_0_TIMESTAMP_2_MASK  0xff0000u
#define GET_PROC_HOST_TIMESTAMP_0_TIMESTAMP_2(__reg__)  (((__reg__) & 0xff0000) >> 16u)
#define SET_PROC_HOST_TIMESTAMP_0_TIMESTAMP_2(__val__)  (((__val__) << 16u) & 0xff0000u)
#define     PROC_HOST_TIMESTAMP_0_TIMESTAMP_3_SHIFT 24u
#define     PROC_HOST_TIMESTAMP_0_TIMESTAMP_3_MASK  0xff000000u
#define GET_PROC_HOST_TIMESTAMP_0_TIMESTAMP_3(__reg__)  (((__reg__) & 0xff000000) >> 24u)
#define SET_PROC_HOST_TIMESTAMP_0_TIMESTAMP_3(__val__)  (((__val__) << 24u) & 0xff000000u)

/** @brief Register definition for @ref PROC_t.HostTimestamp0. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt8 timestamp_0;
        /** @brief  */
        UInt8 timestamp_1;
        /** @brief  */
        UInt8 timestamp_2;
        /** @brief  */
        UInt8 timestamp_3;
    } bits;
} RegPROCHostTimestamp0_t;

#define REG_PROC_TIMESTAMP_4 ((volatile UInt8*)0xf00044) /* MSB of timestamp. */
/** @brief Register definition for @ref PROC_t.Timestamp4. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;
} RegPROCTimestamp4_t;

#define REG_PROC_BST_PRODUCT_TYPE ((volatile UInt8*)0xf00045) /* Value stored in OTP byte 11. */
/** @brief Register definition for @ref PROC_t.BstProductType. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;
} RegPROCBstProductType_t;

#define REG_PROC_RESERVED_3 ((volatile UInt8*)0xf00046) /* reserved 8 bit register */
/** @brief Register definition for @ref PROC_t.Reserved3. */
 typedef union {
    /** @brief 8bit direct register access. */
    UInt8 r8;
} RegPROCReserved3_t;

#define REG_PROC_INTERRUPT_STATUS ((volatile UInt8*)0xf00047) /* General purpose registers for communicating information between sensor hub FW and host. */
#define     PROC_INTERRUPT_STATUS_HOST_IRQ_SHIFT 0u
#define     PROC_INTERRUPT_STATUS_HOST_IRQ_MASK  0x1u
#define GET_PROC_INTERRUPT_STATUS_HOST_IRQ(__reg__)  (((__reg__) & 0x1) >> 0u)
#define SET_PROC_INTERRUPT_STATUS_HOST_IRQ(__val__)  (((__val__) << 0u) & 0x1u)
#define     PROC_INTERRUPT_STATUS_WAKEUP_FIFO_SHIFT 1u
#define     PROC_INTERRUPT_STATUS_WAKEUP_FIFO_MASK  0x6u
#define GET_PROC_INTERRUPT_STATUS_WAKEUP_FIFO(__reg__)  (((__reg__) & 0x6) >> 1u)
#define SET_PROC_INTERRUPT_STATUS_WAKEUP_FIFO(__val__)  (((__val__) << 1u) & 0x6u)
#define     PROC_INTERRUPT_STATUS_NONWAKEUP_FIFO_SHIFT 3u
#define     PROC_INTERRUPT_STATUS_NONWAKEUP_FIFO_MASK  0x18u
#define GET_PROC_INTERRUPT_STATUS_NONWAKEUP_FIFO(__reg__)  (((__reg__) & 0x18) >> 3u)
#define SET_PROC_INTERRUPT_STATUS_NONWAKEUP_FIFO(__val__)  (((__val__) << 3u) & 0x18u)
#define     PROC_INTERRUPT_STATUS_STATUS_SHIFT 5u
#define     PROC_INTERRUPT_STATUS_STATUS_MASK  0x20u
#define GET_PROC_INTERRUPT_STATUS_STATUS(__reg__)  (((__reg__) & 0x20) >> 5u)
#define SET_PROC_INTERRUPT_STATUS_STATUS(__val__)  (((__val__) << 5u) & 0x20u)
#define     PROC_INTERRUPT_STATUS_DEBUG_SHIFT 6u
#define     PROC_INTERRUPT_STATUS_DEBUG_MASK  0x40u
#define GET_PROC_INTERRUPT_STATUS_DEBUG(__reg__)  (((__reg__) & 0x40) >> 6u)
#define SET_PROC_INTERRUPT_STATUS_DEBUG(__val__)  (((__val__) << 6u) & 0x40u)
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
        /** @brief The wakeup channel has available FIFO data. */
        UInt8 wakeup_fifo:2;
        /** @brief The non-wakeup channel has available FIFO data. */
        UInt8 nonwakeup_fifo:2;
        /** @brief The status channel has available status data. */
        UInt8 status:1;
        /** @brief The status channel has available debug or async status data. */
        UInt8 debug:1;
        /** @brief The chip was reset due to WDT, POR, HIF, or pin, or a fault occurred. */
        UInt8 reset_or_fault:1;
    } bits;
} RegPROCInterruptStatus_t;

#define REG_PROC_DEBUG ((volatile UInt32*)0xf00048) /*  */
#define     PROC_DEBUG_ERROR_SHIFT 0u
#define     PROC_DEBUG_ERROR_MASK  0xffu
#define GET_PROC_DEBUG_ERROR(__reg__)  (((__reg__) & 0xff) >> 0u)
#define SET_PROC_DEBUG_ERROR(__val__)  (((__val__) << 0u) & 0xffu)
#define     PROC_DEBUG_INTERRUPT_STATE_SHIFT 8u
#define     PROC_DEBUG_INTERRUPT_STATE_MASK  0xff00u
#define GET_PROC_DEBUG_INTERRUPT_STATE(__reg__)  (((__reg__) & 0xff00) >> 8u)
#define SET_PROC_DEBUG_INTERRUPT_STATE(__val__)  (((__val__) << 8u) & 0xff00u)
#define     PROC_DEBUG_DEBUG_VALUE_SHIFT 16u
#define     PROC_DEBUG_DEBUG_VALUE_MASK  0xff0000u
#define GET_PROC_DEBUG_DEBUG_VALUE(__reg__)  (((__reg__) & 0xff0000) >> 16u)
#define SET_PROC_DEBUG_DEBUG_VALUE(__val__)  (((__val__) << 16u) & 0xff0000u)
#define     PROC_DEBUG_DEBUG_STATE_SHIFT 24u
#define     PROC_DEBUG_DEBUG_STATE_MASK  0xff000000u
#define GET_PROC_DEBUG_DEBUG_STATE(__reg__)  (((__reg__) & 0xff000000) >> 24u)
#define SET_PROC_DEBUG_DEBUG_STATE(__val__)  (((__val__) << 24u) & 0xff000000u)

/** @brief Register definition for @ref PROC_t.Debug. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief  */
        UInt8 error;
        /** @brief  */
        UInt8 interrupt_state;
        /** @brief  */
        UInt8 debug_value;
        /** @brief  */
        UInt8 debug_state;
    } bits;
} RegPROCDebug_t;

#define REG_PROC_GP5 ((volatile UInt32*)0xf0004c) /*  */
#define     PROC_GP5_PROC_GP5_7_0_SHIFT 0u
#define     PROC_GP5_PROC_GP5_7_0_MASK  0xffu
#define GET_PROC_GP5_PROC_GP5_7_0(__reg__)  (((__reg__) & 0xff) >> 0u)
#define SET_PROC_GP5_PROC_GP5_7_0(__val__)  (((__val__) << 0u) & 0xffu)
#define     PROC_GP5_PROC_GP5_15_8_SHIFT 8u
#define     PROC_GP5_PROC_GP5_15_8_MASK  0xff00u
#define GET_PROC_GP5_PROC_GP5_15_8(__reg__)  (((__reg__) & 0xff00) >> 8u)
#define SET_PROC_GP5_PROC_GP5_15_8(__val__)  (((__val__) << 8u) & 0xff00u)
#define     PROC_GP5_PROC_GP5_23_16_SHIFT 16u
#define     PROC_GP5_PROC_GP5_23_16_MASK  0xff0000u
#define GET_PROC_GP5_PROC_GP5_23_16(__reg__)  (((__reg__) & 0xff0000) >> 16u)
#define SET_PROC_GP5_PROC_GP5_23_16(__val__)  (((__val__) << 16u) & 0xff0000u)
#define     PROC_GP5_PROC_GP5_31_24_SHIFT 24u
#define     PROC_GP5_PROC_GP5_31_24_MASK  0xff000000u
#define GET_PROC_GP5_PROC_GP5_31_24(__reg__)  (((__reg__) & 0xff000000) >> 24u)
#define SET_PROC_GP5_PROC_GP5_31_24(__val__)  (((__val__) << 24u) & 0xff000000u)

/** @brief Register definition for @ref PROC_t.Gp5. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief Byte-oriented, general-purpose register for communicating information between sensor hub FW and host. */
        UInt8 proc_gp5_7_0;
        /** @brief Byte-oriented, general-purpose register for communicating information between sensor hub FW and host. */
        UInt8 proc_gp5_15_8;
        /** @brief Byte-oriented, general-purpose register for communicating information between sensor hub FW and host. */
        UInt8 proc_gp5_23_16;
        /** @brief Byte-oriented, general-purpose register for communicating information between sensor hub FW and host. */
        UInt8 proc_gp5_31_24;
    } bits;
} RegPROCGp5_t;

#define REG_PROC_GP6 ((volatile UInt32*)0xf00050) /*  */
#define     PROC_GP6_PROC_GP6_7_0_SHIFT 0u
#define     PROC_GP6_PROC_GP6_7_0_MASK  0xffu
#define GET_PROC_GP6_PROC_GP6_7_0(__reg__)  (((__reg__) & 0xff) >> 0u)
#define SET_PROC_GP6_PROC_GP6_7_0(__val__)  (((__val__) << 0u) & 0xffu)
#define     PROC_GP6_PROC_GP6_15_8_SHIFT 8u
#define     PROC_GP6_PROC_GP6_15_8_MASK  0xff00u
#define GET_PROC_GP6_PROC_GP6_15_8(__reg__)  (((__reg__) & 0xff00) >> 8u)
#define SET_PROC_GP6_PROC_GP6_15_8(__val__)  (((__val__) << 8u) & 0xff00u)
#define     PROC_GP6_PROC_GP6_23_16_SHIFT 16u
#define     PROC_GP6_PROC_GP6_23_16_MASK  0xff0000u
#define GET_PROC_GP6_PROC_GP6_23_16(__reg__)  (((__reg__) & 0xff0000) >> 16u)
#define SET_PROC_GP6_PROC_GP6_23_16(__val__)  (((__val__) << 16u) & 0xff0000u)
#define     PROC_GP6_PROC_GP6_31_24_SHIFT 24u
#define     PROC_GP6_PROC_GP6_31_24_MASK  0xff000000u
#define GET_PROC_GP6_PROC_GP6_31_24(__reg__)  (((__reg__) & 0xff000000) >> 24u)
#define SET_PROC_GP6_PROC_GP6_31_24(__val__)  (((__val__) << 24u) & 0xff000000u)

/** @brief Register definition for @ref PROC_t.Gp6. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief Byte-oriented, general-purpose register for communicating information between sensor hub FW and host. */
        UInt8 proc_gp6_7_0;
        /** @brief Byte-oriented, general-purpose register for communicating information between sensor hub FW and host. */
        UInt8 proc_gp6_15_8;
        /** @brief Byte-oriented, general-purpose register for communicating information between sensor hub FW and host. */
        UInt8 proc_gp6_23_16;
        /** @brief Byte-oriented, general-purpose register for communicating information between sensor hub FW and host. */
        UInt8 proc_gp6_31_24;
    } bits;
} RegPROCGp6_t;

#define REG_PROC_GP7 ((volatile UInt32*)0xf00054) /*  */
#define     PROC_GP7_PROC_GP7_7_0_SHIFT 0u
#define     PROC_GP7_PROC_GP7_7_0_MASK  0xffu
#define GET_PROC_GP7_PROC_GP7_7_0(__reg__)  (((__reg__) & 0xff) >> 0u)
#define SET_PROC_GP7_PROC_GP7_7_0(__val__)  (((__val__) << 0u) & 0xffu)
#define     PROC_GP7_PROC_GP7_15_8_SHIFT 8u
#define     PROC_GP7_PROC_GP7_15_8_MASK  0xff00u
#define GET_PROC_GP7_PROC_GP7_15_8(__reg__)  (((__reg__) & 0xff00) >> 8u)
#define SET_PROC_GP7_PROC_GP7_15_8(__val__)  (((__val__) << 8u) & 0xff00u)
#define     PROC_GP7_PROC_GP7_23_16_SHIFT 16u
#define     PROC_GP7_PROC_GP7_23_16_MASK  0xff0000u
#define GET_PROC_GP7_PROC_GP7_23_16(__reg__)  (((__reg__) & 0xff0000) >> 16u)
#define SET_PROC_GP7_PROC_GP7_23_16(__val__)  (((__val__) << 16u) & 0xff0000u)
#define     PROC_GP7_PROC_GP7_31_24_SHIFT 24u
#define     PROC_GP7_PROC_GP7_31_24_MASK  0xff000000u
#define GET_PROC_GP7_PROC_GP7_31_24(__reg__)  (((__reg__) & 0xff000000) >> 24u)
#define SET_PROC_GP7_PROC_GP7_31_24(__val__)  (((__val__) << 24u) & 0xff000000u)

/** @brief Register definition for @ref PROC_t.Gp7. */
 typedef union {
    /** @brief 32bit direct register access. */
    UInt32 r32;
    /** @brief 16bit direct register access. */
    UInt16 r16[2];
    /** @brief 8bit direct register access. */
    UInt8 r8[4];

    struct {
        /** @brief Byte-oriented, general-purpose register for communicating information between sensor hub FW and host. */
        UInt8 proc_gp7_7_0;
        /** @brief Byte-oriented, general-purpose register for communicating information between sensor hub FW and host. */
        UInt8 proc_gp7_15_8;
        /** @brief Byte-oriented, general-purpose register for communicating information between sensor hub FW and host. */
        UInt8 proc_gp7_23_16;
        /** @brief Byte-oriented, general-purpose register for communicating information between sensor hub FW and host. */
        UInt8 proc_gp7_31_24;
    } bits;
} RegPROCGp7_t;

/** @brief Component definition for @ref PROC. */
typedef struct {
    /** @brief  */
    RegPROCIdinfo_t Idinfo;

    /** @brief  */
    RegPROCVersion0_t Version0;

    /** @brief  */
    RegPROCVersion1_t Version1;

    /** @brief General purpose registers for communicating information between sensor hub FW and host. */
    RegPROCFeatureStatus_t FeatureStatus;

    /** @brief General purpose registers for communicating information between sensor hub FW and host. */
    RegPROCBootStatus_t BootStatus;

    /** @brief  */
    RegPROCHostTimestamp0_t HostTimestamp0;

    /** @brief MSB of timestamp. */
    RegPROCTimestamp4_t Timestamp4;

    /** @brief Value stored in OTP byte 11. */
    RegPROCBstProductType_t BstProductType;

    /** @brief reserved 8 bit register */
    RegPROCReserved3_t Reserved3;

    /** @brief General purpose registers for communicating information between sensor hub FW and host. */
    RegPROCInterruptStatus_t InterruptStatus;

    /** @brief  */
    RegPROCDebug_t Debug;

    /** @brief  */
    RegPROCGp5_t Gp5;

    /** @brief  */
    RegPROCGp6_t Gp6;

    /** @brief  */
    RegPROCGp7_t Gp7;

} PROC_t;

/** @brief  */
extern volatile PROC_t PROC;


#endif /* !HIF_H */

/** @} */
