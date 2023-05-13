////////////////////////////////////////////////////////////////////////////////
///
/// @file       common/7189/includes/dma.h
///
/// @project    EM7189
///
/// @brief      Support header for DMA channels
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

#ifndef DMA_H
#define DMA_H

#if _ARCVER /* Only use for arc build */

#include <aux_registers.h>
#include <types.h>

#define DMA_CHANNEL_UPLOAD       0
#define DMA_CHANNEL_WAKE_FIFO    1
#define DMA_CHANNEL_NONWAKE_FIFO 2
#define DMA_CHANNEL_STATUS       3
#define NUM_DMA_CHANNELS         4

#define DMACTRL     (0x680)
#define DMACENB     (0x681)
#define DMACDSB     (0x682)
#define DMACHPRI    (0x683)
#define DMACNPRI    (0x684)
#define DMACREQ     (0x685)
#define DMACSTAT0   (0x686)
#define DMACSTAT1   (0x687)
#define DMACIRQ     (0x688)
#define DMACBASE    (0x689)
#define DMACRST     (0x68A)

#if defined(__CCAC__)
/**
 * @brief Use this register to enable or disable the DMA controller. When
 *      reset, the DMA controller is disabled.
 *
 * When the DMA controller is disabled, any DMA request from an external
 * peripheral or a software application is not serviced. The DMA controller
 * should not be disabled while there are active DMA operations. Disabling the
 * DMA controller too early may prevent outstanding DMA operations from
 * completing.
 */
static volatile __aux(DMACTRL) UInt32 AR_DMACTRL;

/**
 * @brief Use this register to enable DMA channels in the DMA controller.
 *
 * Each bit is assigned to a channel starting with bit 0 for channel 0, bit 1
 * for channel 1, and so on until bit 15 for channel 15. Write 1 to a bit to
 * enable the corresponding channel. This register can be read to determine
 * which channel is enabled. A DMA channel can only execute a data transfer
 * if it has been enabled. A write of 0 to this register is ignored.
 */
static volatile __aux(DMACENB) UInt32 AR_DMACENB;

/**
 * @brief Use this register to disable DMA channels in the DMA controller.
 *
 * Each bit is assigned to a channel starting with bit 0 for channel 0, bit 1
 * for channel 1, and so on until bit 15 for channel 15. Write 1 to a bit to
 * disable the corresponding channel. A write of 0 to this register is ignored.
 */
static volatile __aux(DMACDSB) UInt32 AR_DMACDSB;

/**
 * @brief Use this register to set the priority level of a DMA channel to high.
 *
 * By default, each channel is set to normal priority. Each bit is assigned to
 * a channel starting with bit 0 for channel 0, bit 1 for channel 1, and so on
 * until bit 15 for channel 15. Write 1 to a bit to set the priority level of
 * the corresponding channel to high. The register can be read to determine
 * which channels have high priority. A write of 0 to this register is ignored.
 */
static volatile __aux(DMACHPRI) UInt32 AR_DMACHPRI;

/**
 * @brief Use this register to set the priority level of a DMA channel to
 *      normal.
 *
 * By default each channel is set to normal priority. Each bit is assigned to a
 * channel starting with bit 0 for channel 0, bit 1 for channel 1, and so on
 * until bit 15 for channel 15. Write 1 to a bit to set the priority level of
 * the corresponding channel to normal. A write of 0 to the register is ignored.
 */
static volatile __aux(DMACNPRI) UInt32 AR_DMACNPRI;

/**
 * @brief Use this register to initiate a DMA transfer from software.
 *
 * Each bit is assigned to a channel starting with bit 0 for channel 0, bit 1
 * for channel 1, and so on until bit 15 for channel 15. Write 1 to a bit to
 * initiate a data transfer on the respective DMA channel. After you have
 * initiated a data transfer, subsequent writes to that bit of a DMA channel
 * that is currently processing a data transfer are ignored.
 */
static volatile __aux(DMACREQ) UInt32 AR_DMACREQ;

/**
 * @brief Use this register to read the active status of all the DMA channels.
 *
 * Each bit is assigned to a channel, starting with bit 0 for channel 0, bit 1
 * for channel 1, and so on until bit 15 for channel 15. When a DMA channel
 * receives a data transfer request and it is acknowledged, its respective bit
 * in the DMACSTAT0 register is immediately set to 1 to indicate that the data
 * transfer is in progress. When the data transfer is complete, the bit is
 * cleared to 0.
 */
static volatile __aux(DMACSTAT0) UInt32 AR_DMACSTAT0;

/**
 * @brief This register indicates the completion and the error status of all
 *          DMA channels.
 *
 * Bits [15:0] indicate the completion status where each bit is assigned to a
 * channel, starting at bit 0 for channel 0, bit 1 for channel 1, and so on
 * until bit 15 for channel 15. When a data transfer on a DMA channel has
 * completed, the corresponding bit in the DMASTAT1 register is set to 1.
 * These bits are only cleared when their respective DMA channels are disabled,
 * re-enabled, or a new data transfer is initiated.
 *
 * Bits [31:16] indicate the error status where each bit is assigned to a
 * channel, starting at bit 0 for channel 16, bit 1 for channel 17, and so on
 * until bit 31 is for channel 15. When a memory or auxiliary access error
 * occurs during a data transfer on a DMA channel, the corresponding bit is set
 * to 1, and the corresponding DMA channel is immediately disabled. Error bits
 * are cleared following an explicit write (with the target error bits to clear
 * set to 1) to the DMACSTAT1 register. You must clear an error bit only if you
 * have already read the register and detected that the respective error bit
 * has been set.
 */
static volatile __aux(DMACSTAT1) UInt32 AR_DMACSTAT1;

/**
 * @brief Use this register to read the interrupt request status of all the
 * DMA channels.
 *
 * Each bit is assigned to a channel, starting at bit 0 for channel 0, bit 1 for
 * channel 1, and so on until bit 15 for channel 15. When a DMA channel raises a
 * level interrupt, the corresponding bit is set in this register. To clear an
 * interrupt, write 1 to the channel's corresponding bit in this register.
 */
static volatile __aux(DMACIRQ) UInt32 AR_DMACIRQ;
#endif /* CCAC */

#define DMA_OPERATION_INVALID               0x00
#define DMA_OPERATION_SINGLE_TRANSFER       0x01
#define DMA_OPERATION_LINKED_LIST_AUTO      0x02
#define DMA_OPERATION_LINKED_LIST_MANUAL    0x03

#define DMA_REQUEST_TYPE_AUTO               0x00
#define DMA_REQUEST_TYPE_MANUAL             0x01

#define DMA_DEST_TYPE_MEM_TO_MEM            0x00
#define DMA_DEST_TYPE_MEM_TO_AUX            0x01
#define DMA_DEST_TYPE_AUX_TO_MEM            0x02
#define DMA_DEST_TYPE_AUT_TO_AUX            0x03

#define DMA_DATA_WIDTH_1B_INC_1B            0x00
#define DMA_DATA_WIDTH_1B_INC_2B            0x01
#define DMA_DATA_WIDTH_1B_INC_4B            0x02
#define DMA_DATA_WIDTH_2B_INC_2B            0x03
#define DMA_DATA_WIDTH_2B_INC_4B            0x04
#define DMA_DATA_WIDTH_4B_INC_4B            0x05
#define DMA_DATA_WIDTH_ZERO_MEM             0x06

#define DMA_BLOCK_SIZE(__size__)            ((__size__) - 1)

#define DMA_ADDR_MODE_CONSTANT              0x00
#define DMA_ADDR_MODE_SOURCE_INC_DEST_CONST 0x01
#define DMA_ADDR_MODE_SOURCE_CONST_DEST_INC 0x02
#define DMA_ADDR_MODE_INC                   0x03
typedef struct
{
    /**
     * @brief The operation field defines the type of data transfer that
     *      should be executed. Two types of data transfers are supported:
     *      Single and Link-List.
     */
    UInt32 operation:2;

    /** @brief When set, requests must be continually issued to the DMA channel
     * until the entire block data transfer completes.
     */
    UInt32 request_type:1;

    /**
     * @brief This field is used to define the source and destination target
     * types as: auxiliary or memory.
     */
    UInt32 trans_type:2;

    /**
     * @brief This field defines the data width and address increment for each
     * DMA cell transfer.
     */
    UInt32 data_width:3;

    /**
     * @brief This field defines the size of the entire data transfer in bytes.
     *
     * This field is 13-bits in width allowing a maximum of 8 Kbytes of data to
     * be transferred.
     */
    UInt32 block_size:13;

    /**
     * @brief Defines the number of DMA cells that can be transferred before
     *  the DMA controller arbitrates to determine which DMA channel should next
     *  get access to the memory bus.
     */
    UInt32 arb:8;

    /**
     * @brief defines whether an interrupt is raised when a data transfer is
     *  complete.
     */
    UInt32 irq_en:1;

    /**
     * @brief Defines whether the source and destination addresses should be
     *  updated after a DMA cell transfer.
     */
    UInt32 addr_mode:2;
} DMAControl_t;


struct DMADescriptor
{
    DMAControl_t ctrl;
    void* source_addr_end;
    void* dest_addr_end;
    struct DMADescriptor* next;
};
typedef struct DMADescriptor DMADescriptor_t; // the base descriptors must be aligned to 256, but other descriptors pointed to by them do not

#if defined(__CCAC__)
/**
 * @brief Use the DMACBASE register to specify the 16-byte aligned base address
 *   of the channel structure in memory.
 *
 * This base address must be aligned to 256 bytes. This base address assumes
 * that the address offsets are always relative to DMA channel 0. The DMA
 * controller uses the address value in this register to obtain all memory
 * mapped DMA channel registers.
 */
static DMADescriptor_t** volatile (__aux(0x689)  AR_DMACBASE);


/**
 * @brief The DMACRST register is used to explicitly reset a DMA channel.
 *
 * Writing a 1 to the bits of this register immediately resets the respective
 * DMA channel (including the channel's internal state). The respective enable
 * bits in the DMACENB are also reset to 0, disabling the channels.
 * You can also read this register to know the reset status of a channel. The
 * bits in this register are sticky. When a bit is set to 1, the bit remains set
 * until the DMA channel is reset.
 */
static volatile __aux(DMACRST) UInt32 AR_DMACRST;
#endif /* CCAC */

static void ALWAYS_INLINE EnableDMA(void)
{
#if defined(__CCAC__)
    AR_DMACTRL = 1;
#else
    writeAUX(1, (void*)DMACTRL);
#endif
}

static void ALWAYS_INLINE DisableDMA(void)
{
#if defined(__CCAC__)
    AR_DMACTRL = 0;
#else
    writeAUX(0, (void*)DMACTRL);
#endif
}

static bool ALWAYS_INLINE DMAEnabled()
{
    UInt32 ctrl;

#if defined(__CCAC__)
    ctrl = AR_DMACTRL;
#else
    ctrl = readAUX((void*)DMACTRL);
#endif

    return ((ctrl & 1) == 1);
}

static void ALWAYS_INLINE EnableDMAChannel(UInt32 channel)
{
#if defined(__CCAC__)
    AR_DMACENB = (1u << channel);
#else
    writeAUX((1u << channel), (void*)DMACENB);
#endif
}

static void ALWAYS_INLINE DisableDMAChannel(UInt32 channel)
{
#if defined(__CCAC__)
    AR_DMACDSB = (1u << channel);
#else
    writeAUX((1u << channel), (void*)DMACDSB);
#endif
}

static void ALWAYS_INLINE ResetDMAChannel(UInt32 channel)
{
#if defined(__CCAC__)
    AR_DMACRST = (1u << channel);
#else
    writeAUX((1u << channel), (void*)DMACRST);
#endif
}

static bool ALWAYS_INLINE DMAChannelEnabled(UInt32 channel)
{
    UInt32 cenb;

#if defined(__CCAC__)
    cenb = AR_DMACENB;
#else
    cenb = readAUX((void*)DMACENB);
#endif

    return ((cenb & (1u << channel)) == (1u << channel));
}

static void ALWAYS_INLINE SetDMAPriorityHigh(UInt32 channel)
{
#if defined(__CCAC__)
    AR_DMACHPRI = (1u << channel);
#else
    writeAUX((1u << channel), (void*)DMACHPRI);
#endif
}

static void ALWAYS_INLINE SetDMAPriorityLow(UInt32 channel)
{
#if defined(__CCAC__)
    AR_DMACNPRI = (1u << channel);
#else
    writeAUX((1u << channel), (void*)DMACNPRI);
#endif
}

static void ALWAYS_INLINE TriggerDMA(UInt32 channel)
{
#if defined(__CCAC__)
    AR_DMACREQ = (1u << channel);
#else
    writeAUX((1u << channel), (void*)DMACREQ);
#endif
}

static bool ALWAYS_INLINE DMAActive(UInt32 channel)
{
    UInt32 stat0;

#if defined(__CCAC__)
    stat0 = AR_DMACSTAT0;
#else
    stat0 = readAUX((void*)DMACSTAT0);
#endif

    return ((stat0 & (1u << channel)) == (1u << channel));
}

static bool ALWAYS_INLINE DMAComplete(UInt32 channel)
{
    UInt32 stat1;

#if defined(__CCAC__)
    stat1 = AR_DMACSTAT1;
#else
    stat1 = readAUX((void*)DMACSTAT1);
#endif

    return ((stat1 & (1u << channel)) == (1u << channel));
}

static bool ALWAYS_INLINE DMAError(UInt32 channel)
{
    UInt32 stat1;

#if defined(__CCAC__)
    stat1 = AR_DMACSTAT1;
#else
    stat1 = readAUX((void*)DMACSTAT1);
#endif

    return (((stat1 >> 16u) & (1u << channel)) == (1u << channel));
}

static void ALWAYS_INLINE ClearDMAError(UInt32 channel)
{
#if defined(__CCAC__)
    AR_DMACSTAT1 = (0x100u << channel);
#else
    writeAUX((0x100u << channel), (void*)DMACSTAT1);
#endif
}

static bool ALWAYS_INLINE DMAIRQStatus(UInt32 channel)
{
    UInt32 irq;

#if defined(__CCAC__)
    irq = AR_DMACIRQ;
#else
    irq = readAUX((void*)DMACIRQ);
#endif
    return ((irq & (1u << channel)) == (1u << channel));
}

static void ALWAYS_INLINE ClearDMAIRQ(UInt32 channel)
{
#if defined(__CCAC__)
    AR_DMACIRQ = (1u << channel);
#else
    writeAUX((1u << channel), (void*)DMACIRQ);
#endif
}

static void ALWAYS_INLINE SetDMADescriptors(DMADescriptor_t* descriptors[])
{
#if defined(__CCAC__)
    AR_DMACBASE = descriptors;
#else
    union {
        DMADescriptor_t** desc;
        UInt32 val;
    } caster;
    caster.desc = descriptors;
    writeAUX(caster.val, (void*)DMACBASE);
#endif
}

#endif /* _ARCVER */
#endif /* !DMA_H */
