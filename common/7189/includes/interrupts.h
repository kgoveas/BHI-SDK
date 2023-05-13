////////////////////////////////////////////////////////////////////////////////
///
/// @file       common/7189/includes/interrupts.h
///
/// @project    EM7189
///
/// @brief      Macros for interrupts.
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

#ifndef _INTERRUPT_H
#define _INTERRUPT_H

#include <aux_registers.h>
#include <hw_versions.h>
#include <eiaextensions.h>     //added by vtu
#include <arc_reg.h>         //added by vtu

#include "peripherals.h"

#if _ARCVER          /* ARC, use native prototype */
/**
 * @fn static void _Inline installInterruptHandler(int index, void(*handler)(void) _Interrupt)
 *
 * @brief Overwrite the specified interrupt handler in ram.
 */
static void ALWAYS_INLINE installInterruptHandler(int index, void(*handler)(void) _Interrupt)
{
#if _ARCVER == ARCV2_EM4_CORE2
    void(**base)(void) = (void*)readAUX(INT_VECTOR_BASE);
    base += index;
#if __CCAC__
    *base = (void*)handler;
#else
    *base = handler;
#endif
#else
#error Unknown CPU version _ARCVER
#endif
}

/**
 * @fn void *getInterruptHandler(int index)(void);
 *
 * @brief Retrieve the specified interrupt handler in ram.
 */
static ALWAYS_INLINE void (*getInterruptHandler(int index))(void)
{
    void (*handler)(void);
#if _ARCVER == ARCV2_EM4_CORE2
    void(**base)(void) = (void*)readAUX(INT_VECTOR_BASE);
    base += index;
    handler = *base;
#else
#error Unknown CPU version _ARCVER
#endif

    return handler;
}


/**
 * @fn UInt32 saveInterrupts()
 *
 * @brief Saves the current interrupt context. This is <b>not</b> interrupt save and @ref disableAllInterrupts should be used instead.
 */
static UInt32 ALWAYS_INLINE saveInterrupts(void)
{
   UInt32 temp;
#if _ARCVER == ARCV2_EM4_CORE2
    __asm__("" ::: "memory");


#if defined(__CCAC__)
   temp = ((UInt32)_lr(REG_STATUS32)) & 0x8000001E;
#elif defined(__GNUC__) || defined(__GNUG__)
   temp = __builtin_arc_lr(REG_STATUS32) & 0x8000001E;
#else
#error Unknown compiler.
#endif

    __asm__("" ::: "memory");
   temp >>= 1; // convert from STATUS32 to SETI format
   if (temp & 0x40000000) // IE is set
   {
      temp &= 0x0F; // remove IE
      temp |= 0x10; // this becomes bit 4 of SETI (IE is set)
   }
   temp |= 0x20; // this is the "restore" bit 5 for SETI
#else
   temp = 0;
#endif
   return temp;
}

/**
 * @fn void restoreInterrupts(UInt32 val)
 *
 * @brief restores the previously saved interrupt context from @ref saveInterrupts or @ref disableAllInterrupts.
 */
static void ALWAYS_INLINE restoreInterrupts(UInt32 val)
{
#if defined(DEBUG_TIME_SPENT_WITH_INTERRUPTS_OFF)
    enableGPIOOutput(4);
    if (val & 0x10)
        setGPIOHigh(4);
    else
        setGPIOLow(4);
#endif
#if _ARCVER == ARCV2_EM4_CORE2
     __asm__ __volatile__ (
     "  seti %0\n\t"
     :
     : "r"(val)
     : "memory"
     );
#endif
}


/**
 * @fn void enableAllInterrupts();
 *
 * @brief Globally enable interrupts.
 */
static void ALWAYS_INLINE enableAllInterrupts(void)
{
#if defined(DEBUG_TIME_SPENT_WITH_INTERRUPTS_OFF)
    enableGPIOOutput(4);
    setGPIOHigh(4);
#endif
    /* Enable Level 0 to Level 7 Interrupts */
#if _ARCVER == ARCV2_EM4_CORE2
    __asm__("seti 0x17"  ::: "memory");
#else
#error Unknown CPU version _ARCVER
#endif
}


/**
 * @fn void enableInterrupt1Only();
 *
 * @brief Globally enable priority 1 interrupts.
 */
static void ALWAYS_INLINE enableInterrupt1Only(void)
{
#if defined(DEBUG_TIME_SPENT_WITH_INTERRUPTS_OFF)
    enableGPIOOutput(4);
    setGPIOHigh(4);
#endif
    /* Enable Level 1 interrupts */
#if _ARCVER == ARCV2_EM4_CORE2
    __asm__("seti 0x11"  ::: "memory");
#else
#error Unknown CPU version _ARCVER
#endif
}


/**
 * @fn void enableInterrupt0Only();
 *
 * @brief Globally enable priority 0 interrupts.
 */
static void ALWAYS_INLINE enableInterrupt0Only(void)
{
#if defined(DEBUG_TIME_SPENT_WITH_INTERRUPTS_OFF)
    enableGPIOOutput(4);
    setGPIOLow(4);
#endif
    /* Enable Level 0 interrupts */
#if _ARCVER == ARCV2_EM4_CORE2
    __asm__("seti 0x10"  ::: "memory");
#else
#error Unknown CPU version _ARCVER
#endif
}


/**
 * @fn void disableInterrupts();
 *
 * @brief Globally disable low priority interrupts.
 */
static void ALWAYS_INLINE disableInterrupts(void)
{
    /* Disable all interrupts */
#if _ARCVER == ARCV2_EM4_CORE2
      __asm__("clri\n\t\n" \
            "sync"             // serializing instruction, flush the pipeline before continuing (ensure interrupts are really disabled)
            ::: "memory");
#else
#error Unknown CPU version _ARCVER
#endif
#if defined(DEBUG_TIME_SPENT_WITH_INTERRUPTS_OFF)
      enableGPIOOutput(4);
      setGPIOLow(4);
#endif
}


/**
 * @fn UInt32 disableAllInterrupts();
 *
 * @brief Globally disable all interrupts, and return current interrupt context
 */
static UInt32 ALWAYS_INLINE disableAllInterrupts(void)
{
    UInt32 enables;
    /* Disable all interrupts */
#if _ARCVER == ARCV2_EM4_CORE2
    __asm__ __volatile__ (
     "  clri %0\n\t" \
     "  sync\n\t"           // serializing instruction, flush the pipeline before continuing (ensure interrupts are really disabled)
     : "=r"(enables)
     :
     : "memory"
     );
#if defined(DEBUG_TIME_SPENT_WITH_INTERRUPTS_OFF)
    enableGPIOOutput(4);
    setGPIOLow(4);
#endif
    return enables;
#else
#error Unknown CPU version _ARCVER
#endif
}


/**
 * @fn UInt32 enableInterrupts();
 *
 * @brief Globally enables interrupts without modifying the enable levels
 */
static void ALWAYS_INLINE enableInterrupts(void)
{
#if defined(DEBUG_TIME_SPENT_WITH_INTERRUPTS_OFF)
    enableGPIOOutput(4);
    setGPIOHigh(4);
#endif
    /* Enable interrupts */
#if _ARCVER == ARCV2_EM4_CORE2
    __asm__ __volatile__ (
     "  seti\n\t" \
     :
     :
     : "memory"
     );
#else
#error Unknown CPU version _ARCVER
#endif
}

#if _ARCVER == ARCV2_EM4_CORE2
#define INTERRUPT_OFFSET    16
#else
#error Unknown CPU version _ARCVER
#endif

/** Interupt entry / index into IVT */

#define TIMER0_ENTRY        0       /**< @brief Interrupt entry for ARC timer 0 **/

#define DMA0_DONE_ENTRY     1       /**< @brief Interrupt entry for DMA0 Done **/

#define WATCHDOG_ENTRY      2

#define DMA1_DONE_ENTRY     3       /**< @brief Interrupt entry for DMA1 Done **/
#define DMA2_DONE_ENTRY     4       /**< @brief Interrupt entry for DMA2 Done **/
#define DMA3_DONE_ENTRY     5       /**< @brief Interrupt entry for DMA3 Done **/

#define HIF_COMM_ENTRY      6       /**< @brief Interrupt entry for HIF Comm **/

#define SPIM0_ENTRY         7       /**< @brief Interrupt entry for the SPI master 0 interrupt **/
#define SPIM1_ENTRY         8       /**< @brief Interrupt entry for the SPI master 1 interrupt **/

#define I2CM0_ENTRY         9       /**< @brief Interrupt entry for the I<sup>2</sup>C master 0 interrupt **/
#define I2CM1_ENTRY         10      /**< @brief Interrupt entry for the I<sup>2</sup>C master 1 interrupt **/

#define RTC_ENTRY           11

#define ITMR_ENTRY          12      /**< @brief Interrupt entry for timer 2 **/
#define UTMR_ENTRY          13      /**< @brief Interrupt entry for timer 4 **/

#define HOST_EV_ENTRY       14      /**< @brief Interrupt entry for the host event interrupt **/

#define GPIO_EV_ENTRY       15      /**< @brief Interrupt entry for GPIOx event interrupt **/

#define HIF_BUF_ERR_ENTRY   16      /**< @brief Interrupt entry for HIF buffer error **/

#define QSPI_ENTRY          17      /**< @brief Interrupt entry for the QSPI interrupt **/

#define DMA_ERR_ENTRY       18      /**< @brief Interrupt entry for DMAx error **/

#define HIF_READ_ENTRY      19       /**< @brief Interrupt entry for HIF Read **/
#define HIF_WRITE_ENTRY     20       /**< @brief Interrupt entry for HIF Write **/



#define SWI0_ENTRY          21      /**< @brief Interrupt entry for the software interrupt 0 **/
#define SWI1_ENTRY          22      /**< @brief Interrupt entry for the software interrupt 1 **/
#define SWI2_ENTRY          23      /**< @brief Interrupt entry for the software interrupt 2 **/
#define SWI3_ENTRY          24      /**< @brief Interrupt entry for the software interrupt 3 **/

#define NUM_INTERRUPT_VECTORS   (25 + INTERRUPT_OFFSET) /**< @brief The total number of interrupt vectors. */
#define NUM_GPIO_EV_ENTRIES 12      /**< @brief Total number of GPIO interrupts */



#define SWI(__N__)          (1U << (__N__))
#define SWI0                SWI(0)
#define SWI1                SWI(1)
#define SWI2                SWI(2)
#define SWI3                SWI(3)


#define SWI_PRIORITY_OFFSET  2


/* Interrupt Mask bit */
// No mask / en bits.
//#define TIMER0_INTERRUPT        (1U << TIMER0_ENTRY     )

//#define DMA0_DONE_INTERRUPT     (1U << DMA0_DONE_ENTRY  )

//#define DWATCHDOG_INTERRUPT     (1U << DMA0_DONE_ENTRY  )
//#define DMA1_DONE_INTERRUPT     (1U << DMA1_DONE_ENTRY  )
//#define DMA2_DONE_INTERRUPT     (1U << DMA2_DONE_ENTRY  )
//#define DMA3_DONE_INTERRUPT     (1U << DMA3_DONE_ENTRY  )
#define MASK_OFFSET             HIF_COMM_ENTRY // 6
#define HIF_COMM_INTERRUPT      (1U /*<< (HIF_COMM_ENTRY - MASK_OFFSET)*/)

#define SPIM0_INTERRUPT         (1U << (SPIM0_ENTRY     - MASK_OFFSET))
#define SPIM1_INTERRUPT         (1U << (SPIM1_ENTRY     - MASK_OFFSET))

#define I2CM0_INTERRUPT         (1U << (I2CM0_ENTRY     - MASK_OFFSET))
#define I2CM1_INTERRUPT         (1U << (I2CM1_ENTRY     - MASK_OFFSET))

#define RTC_INTERRUPT           (1U << (RTC_ENTRY       - MASK_OFFSET))

#define ITMR_INTERRUPT          (1U << (ITMR_ENTRY      - MASK_OFFSET))
#define UTMR_INTERRUPT          (1U << (UTMR_ENTRY      - MASK_OFFSET))

#define HOST_EV_INTERRUPT       (1U << (HOST_EV_ENTRY   - MASK_OFFSET))

#define GPIO_EV0_INTERRUPT      (1U << ((GPIO_EV_ENTRY   - MASK_OFFSET)/* + 0u*/))
#define GPIO_EV1_INTERRUPT      (1U << ((GPIO_EV_ENTRY   - MASK_OFFSET) + 1u))
#define GPIO_EV2_INTERRUPT      (1U << ((GPIO_EV_ENTRY   - MASK_OFFSET) + 2u))
#define GPIO_EV3_INTERRUPT      (1U << ((GPIO_EV_ENTRY   - MASK_OFFSET) + 3u))
#define GPIO_EV4_INTERRUPT      (1U << ((GPIO_EV_ENTRY   - MASK_OFFSET) + 4u))
#define GPIO_EV5_INTERRUPT      (1U << ((GPIO_EV_ENTRY   - MASK_OFFSET) + 5u))
#define GPIO_EV6_INTERRUPT      (1U << ((GPIO_EV_ENTRY   - MASK_OFFSET) + 6u))
#define GPIO_EV7_INTERRUPT      (1U << ((GPIO_EV_ENTRY   - MASK_OFFSET) + 7u))
#define GPIO_EV8_INTERRUPT      (1U << ((GPIO_EV_ENTRY   - MASK_OFFSET) + 8u))
#define GPIO_EV9_INTERRUPT      (1U << ((GPIO_EV_ENTRY   - MASK_OFFSET) + 9u))
#define GPIO_EV10_INTERRUPT     (1U << ((GPIO_EV_ENTRY   - MASK_OFFSET) + 10u))
#define GPIO_EV11_INTERRUPT     (1U << ((GPIO_EV_ENTRY   - MASK_OFFSET) + 11u))

#define HIF_BUF_ERR_INTERRUPT_MASK (1U << ((HIF_BUF_ERR_ENTRY - MASK_OFFSET) + 11u))
#define HIF_BUF0_OVERFLOW_EN    (1U << 21u)
#define HIF_BUF1_UNDERFLOW_EN   (1U << 22u)
#define HIF_BUF2_UNDERFLOW_EN   (1U << 23u)
#define HIF_BUF3_UNDERFLOW_EN   (1U << 24u)

#define QSPI_INTERRUPT_MASK     (1U << ((QSPI_ENTRY      - MASK_OFFSET) + 11u))
#define QSPI_DATA_INTERRUPT_EN  (1U << 25u)
#define QSPI_DONE_INTERRUPT_EN  (1U << 26u)

#define HIF_READ_INTERRUPT_MASK  (1 << ((HIF_READ_ENTRY  - MASK_OFFSET) + 10u))
#define HIF_WRITE_INTERRUPT_MASK (1 << ((HIF_WRITE_ENTRY - MASK_OFFSET) + 10u))

//#define DMA_ERR_INTERRUPT       (1U << DMA_ERR_ENTRY    )

//#define SWI0_INTERRUPT          (1U << SWI0_ENTRY       )
//#define SWI1_INTERRUPT          (1U << SWI1_ENTRY       )
//#define SWI2_INTERRUPT          (1U << SWI2_ENTRY       )
//#define SWI3_INTERRUPT          (1U << SWI3_ENTRY       )


// only allow a few proprities to work. 2, 3, 4 for SWI.
#define PRIORITY_LOWEST         PRIORITY_4            /**< @brief Lowest interrupt priority level supported by the 7189 hardware */


/**
 * @fn void triggerSoftwareInterrupt_unsafe(UInt32 bits);
 *
 * @brief Trigger the specified software interrupt. Note: this is not atomic.
 *
 * @param    bits    The specified interrupt bits
 */
static void ALWAYS_INLINE triggerSoftwareInterrupt_unsafe(UInt32 bits)
{
    __asm__("" ::: "memory");
    PIRQ.Swirqset.r32 = bits;
    __asm__("" ::: "memory");
}

/**
 * @fn void triggerSoftwareInterrupt(UInt32 bits);
 *
 * @brief Trigger the specified software interrupt.
 *
 * @param    bits    The specified interrupt bits
 */
static void ALWAYS_INLINE triggerSoftwareInterrupt(UInt32 bits)
{
    triggerSoftwareInterrupt_unsafe(bits);
}

/**
 * @fn void clearSoftwareInterrupt_unsafe(UInt32 bits);
 *
 * @brief Clear the pending software interrupt. Note: this is not atomic.
 *
 * @param    bits    The software interrupt to clear
 */
static void ALWAYS_INLINE clearSoftwareInterrupt_unsafe(UInt32 bits)
{
    __asm__("" ::: "memory");
    PIRQ.Swirqclr.r32 = bits;
    __asm__("" ::: "memory");
}

/**
 * @fn void clearSoftwareInterrupt(UInt32 bits);
 *
 * @brief Clear the pending software interrupt.
 *
 * @param    bits    The software interrupt to clear
 */
static void ALWAYS_INLINE clearSoftwareInterrupt(UInt32 bits)
{
    clearSoftwareInterrupt_unsafe(bits);
}

/**
 * @fn bool triggeredSoftwareInterrupt_unsafe(UInt32 bits);
 *
 * @brief Check for a pending software interrupt. Note: this is not atomic.
 *
 * @param    bits    The software interrupt to check.
 *
 * @retval   TRUE     The software interrupt is pending.
 * @retval   FALSE    The software interrupt is not pending.
 */
static bool ALWAYS_INLINE triggeredSoftwareInterrupt_unsafe(UInt32 bits)
{
    return (PIRQ.Swirqvalue.r32 & bits) == (bits);
}

/**
 * @fn bool triggeredSoftwareInterrupt(UInt32 bits);
 *
 * @brief Check for a pending software interrupt.
 *
 * @param    bits    The software interrupt to check.
 *
 * @retval   TRUE     The software interrupt is pending.
 * @retval   FALSE    The software interrupt is not pending.
 */
static bool ALWAYS_INLINE triggeredSoftwareInterrupt(UInt32 bits)
{
    return triggeredSoftwareInterrupt_unsafe(bits);
}

/**
 * @fn UInt32 getInterruptCause(void);
 *
 * @brief Returns the interrupt index of the currently running interrupt.
 */
static UInt32 ALWAYS_INLINE getInterruptCause(void)
{
    // our interrupts start at entry 16.
    return readAUX(ICAUSE) - INTERRUPT_OFFSET;
}


/**
 * @fn void setInterruptPriority(UInt32 entry, UInt8 priority);
 *
 * @brief Modify the interrupt priority level.  NOTE: this is not atomic.
 *
 * @param    entry        The interrupt to modify
 * @param    priority    The priority level to specify, 0 being highest.
 */
static void ALWAYS_INLINE setInterruptPriority(UInt32 entry, UInt8 priority)
{
    // our interrupts start at entry 16.
    writeAUX(entry + INTERRUPT_OFFSET, IRQ_INTERRUPT);
    writeAUX(priority, IRQ_PRIORITY);
}


/**
 * @fn UInt8 getInterruptPriority(UInt32 entry);
 *
 * @brief Query the interrupt priority level.  NOTE: this is not atomic.
 *
 * @param    entry        The interrupt to query
 * @return   priority    The priority level of the interrupt.
 */
static UInt8 ALWAYS_INLINE getInterruptPriority(UInt32 entry)
{
    // our interrupts start at entry 16.
    writeAUX(entry + INTERRUPT_OFFSET, IRQ_INTERRUPT);
    return readAUX(IRQ_PRIORITY);
}


/**
 * @fn bool isInterruptEnabled(UInt32 bits);
 *
 * @brief Determines if the specified interrupts are enabled.
 *
 * @param    bits        The interrupt(s) to check
 *
 * @retval   TRUE        The specified interrupts are enabled
 * @retval   FALSE        One or more of the specified interrupts are disabled
 */
static bool ALWAYS_INLINE isInterruptEnabled(UInt32 bits)
{
    return (PIRQ.Irqenvalue.r32 & bits) == bits;
}


/**
 * @fn bool isInterruptPending(UInt32 bits);
 *
 * @brief Determines if all of the specified interrupts are pending.
 *
 * @param    bits        The interrupt(s) to check
 *
 * @retval   TRUE        The specified interrupts are all pending
 * @retval   FALSE        One or more of the specified interrupts are not pending
 */
static bool ALWAYS_INLINE isInterruptPending(UInt32 bits)
{
    return (PIRQ.Statusvalue.r32 & bits) == bits;
}

/**
 * @fn bool isEitherInterruptPending(UInt32 bits);
 *
 * @brief Determines if any of the specified interrupts are pending.
 *
 * @param    bits        The interrupt(s) to check
 *
 * @retval   TRUE        The specified interrupts are all pending
 * @retval   FALSE        One or more of the specified interrupts are not pending
 */
static bool ALWAYS_INLINE isEitherInterruptPending(UInt32 bits)
{
    return (PIRQ.Statusvalue.r32 & bits) != 0;
}

/**
 * @fn void enableInterrupt_unsafe(UInt32 bits);
 *
 * @brief Enables the specified interrupts. Note: This is not atomic.
 *
 * @param    bits        The interrupt(s) to enable
 */
static void ALWAYS_INLINE enableInterrupt_unsafe(UInt32 bits)
{
    __asm__("" ::: "memory");
    PIRQ.Irqenset.r32 = bits;
    __asm__("" ::: "memory");
}

/**
 * @fn void enableInterrupt(UInt32 bits);
 *
 * @brief Enables the specified interrupts.
 *
 * @param    bits        The interrupt(s) to enable
 */
static void ALWAYS_INLINE enableInterrupt(UInt32 bits)
{
    enableInterrupt_unsafe(bits);
}


/**
 * @fn void disableInterrupt_unsafe(UInt32 bits);
 *
 * @brief Disable the specified interrupts.
 *
 * @param    bits        The interrupt(s) to enable
 */
static void ALWAYS_INLINE disableInterrupt_unsafe(UInt32 bits)
{
    __asm__("" ::: "memory");
    PIRQ.Irqenclr.r32 = bits;
    __asm__("" ::: "memory");
}

/**
 * @fn void disableInterrupt(UInt32 bits);
 *
 * @brief Disable the specified interrupts.
 *
 * @param    bits        The interrupt(s) to enable
 */
static void ALWAYS_INLINE disableInterrupt(UInt32 bits)
{
    disableInterrupt_unsafe(bits);
}


/**
 * @fn bool isInterruptMasked(UInt32 bits);
 *
 * @brief Determines if the specified interrupts are masked.
 *
 * @param    bits        The interrupt(s) to check
 *
 * @retval   TRUE        The specified interrupts are masked
 * @retval   FALSE        One or more of the specified interrupts are unmasked
 */
static bool ALWAYS_INLINE isInterruptMasked(UInt32 bits)
{
    return (PIRQ.Irqmaskvalue.r32 & bits) == bits;
}


/**
 * @fn void maskInterrupt_unsafe(UInt32 bits);
 *
 * @brief Mask the specified interrupts. Note: this is not atomic.
 *
 * @param    bits        The interrupt(s) to mask
 */
static void ALWAYS_INLINE maskInterrupt_unsafe(UInt32 bits)
{
    __asm__("" ::: "memory");
    PIRQ.Irqmaskset.r32 = bits;
    __asm__("" ::: "memory");
}

/**
 * @fn void maskInterrupt(UInt32 bits);
 *
 * @brief Mask the specified interrupts.
 *
 * @param    bits        The interrupt(s) to mask
 */
static void ALWAYS_INLINE maskInterrupt(UInt32 bits)
{
    maskInterrupt_unsafe(bits);
}

/**
 * @fn void unmaskInterrupt_unsafe(UInt32 bits);
 *
 * @brief Unmask the specified interrupts.
 *
 * @param    bits        The interrupt(s) to unmask
 */
static void ALWAYS_INLINE unmaskInterrupt_unsafe(UInt32 bits)
{
    __asm__("" ::: "memory");
    PIRQ.Irqmaskclr.r32 = bits;
    __asm__("" ::: "memory");
}

/**
 * @fn void unmaskInterrupt(UInt32 bits);
 *
 * @brief Unmask the specified interrupts.
 *
 * @param    bits        The interrupt(s) to unmask
 */
static void ALWAYS_INLINE unmaskInterrupt(UInt32 bits)
{
    unmaskInterrupt_unsafe(bits);
}


/**
 * @fn void clearInterrupt(UInt32 bits);
 *
 * @brief Clear the pending interrupt(s).
 *
 * @param    bits        The interrupt(s) to clear
 */
static void ALWAYS_INLINE clearInterrupt(UInt32 bits)
{
    __asm__("" ::: "memory");
    PIRQ.Statusclr.r32 = bits;
    __asm__("" ::: "memory");
}


/** HIF Read/Write specific configuration **/
#define HIF_READ_INTERRUPT_OFFSET       (0x2a) /* REG_PROC_GP3_B0, 0x2A */
#define HIF_WRITE_INTERRUPT_OFFSET      (0x04) /* REG_HOST_GP0_B0, 0x04 */
static void ALWAYS_INLINE enableHIFReadInterrupt(UInt8* hif_reg)
{
    union {
        UInt8* address;
        UInt32 value;
    } addr = {.address = hif_reg};
    SInt8 reg = (SInt8)addr.value;
    reg -= HIF_READ_INTERRUPT_OFFSET;
    if(reg >= 0)
    {
        UInt16 bitmask = 1u << reg;

        PIRQ.Hrwirqenset.bits.hrd_irq_en_set = bitmask;
    }
}

static void ALWAYS_INLINE enableHIFWriteInterrupt(UInt8* hif_reg)
{
    union {
        UInt8* address;
        UInt32 value;
    } addr = {.address = hif_reg};
    SInt8 reg = (SInt8)addr.value;
    reg -= HIF_WRITE_INTERRUPT_OFFSET;
    if(reg >= 0)
    {
        UInt16 bitmask = 1u << reg;

        PIRQ.Hrwirqenset.bits.hwr_irq_en_set = bitmask;
    }
}

static void ALWAYS_INLINE disableHIFReadInterrupt(UInt8* hif_reg)
{
    union {
        UInt8* address;
        UInt32 value;
    } addr = {.address = hif_reg};
    SInt8 reg = (SInt8)addr.value;
    reg -= HIF_READ_INTERRUPT_OFFSET;
    if(reg >= 0)
    {
        UInt16 bitmask = 1u << reg;

        PIRQ.Hrwirqenclr.bits.hrd_irq_en_clr = bitmask;
    }
}

static void ALWAYS_INLINE disableHIFWriteInterrupt(UInt8* hif_reg)
{
    union {
        UInt8* address;
        UInt32 value;
    } addr = {.address = hif_reg};
    SInt8 reg = (SInt8)addr.value;
    reg -= HIF_WRITE_INTERRUPT_OFFSET;
    if(reg >= 0)
    {
        UInt16 bitmask = 1u << reg;

        PIRQ.Hrwirqenclr.bits.hwr_irq_en_clr = bitmask;
    }
}

static void ALWAYS_INLINE clearHIFReadInterrupt(UInt8* hif_reg)
{
    union {
        UInt8* address;
        UInt32 value;
    } addr = {.address = hif_reg};
    SInt8 reg = (SInt8)addr.value;
    reg -= HIF_READ_INTERRUPT_OFFSET;
    if(reg >= 0)
    {
        UInt16 bitmask = 1u << reg;

        PIRQ.Hrwstatusclr.bits.hrd_irq_stat_clr = bitmask;
    }
}

static void ALWAYS_INLINE clearHIFWriteInterrupt(UInt8* hif_reg)
{
    union {
        UInt8* address;
        UInt32 value;
    } addr = {.address = hif_reg};
    SInt8 reg = (SInt8)addr.value;
    reg -= HIF_WRITE_INTERRUPT_OFFSET;
    if(reg >= 0)
    {
        UInt16 bitmask = 1u << reg;

        PIRQ.Hrwstatusclr.bits.hwr_irq_stat_clr = bitmask;
    }
}


/**
 * @fn bool isHIFReadInterruptPending(UInt8* hif_reg)
 *
 * @brief Determines if all of the specified read is pending.
 *
 * @param    hif_reg     The register to check
 *
 * @retval   TRUE        The specified interrupt is pending
 * @retval   FALSE       Not pending
 */
static bool ALWAYS_INLINE isHIFReadInterruptPending(UInt8 *hif_reg)
{
    union {
        UInt8* address;
        UInt32 value;
    } addr = {.address = hif_reg};
    SInt8 reg = (SInt8)addr.value;
    reg -= HIF_READ_INTERRUPT_OFFSET;
    if(reg >= 0)
    {
        UInt16 bitmask = 1u << reg;

        return (PIRQ.Hrwstatusvalue.bits.hrd_irq_stat_val & bitmask) != 0u;
    }
    return FALSE;
}

/**
 * @fn bool isHIFWriteInterruptPending(UInt8* hif_reg)
 *
 * @brief Determines if all of the specified write is pending.
 *
 * @param    hif_reg     The register to check
 *
 * @retval   TRUE        The specified interrupt is pending
 * @retval   FALSE       Not pending
 */
static bool ALWAYS_INLINE isHIFWriteInterruptPending(UInt8 *hif_reg)
{
    union {
        UInt8* address;
        UInt32 value;
    } addr = {.address = hif_reg};
    SInt8 reg = (SInt8)addr.value;
    reg -= HIF_WRITE_INTERRUPT_OFFSET;
    if(reg >= 0)
    {
        UInt16 bitmask = 1u << reg;

        return (PIRQ.Hrwstatusvalue.bits.hwr_irq_stat_val & bitmask) != 0u;
    }
    return FALSE;
}

/**
 * @fn bool isHIFReadInterruptEnabled(UInt8* hif_reg)
 *
 * @brief Determines if all of the specified read is enabled.
 *
 * @param    hif_reg     The register to check
 *
 * @retval   TRUE        The specified interrupt is enabled
 * @retval   FALSE       Not pending
 */
static bool ALWAYS_INLINE isHIFReadInterruptEnabled(UInt8 *hif_reg)
{
    union {
        UInt8* address;
        UInt32 value;
    } addr = {.address = hif_reg};
    SInt8 reg = (SInt8)addr.value;
    reg -= HIF_READ_INTERRUPT_OFFSET;
    if(reg >= 0)
    {
        UInt16 bitmask = 1u << reg;

        return (PIRQ.Hrwirqenvalue.bits.hrd_irq_en_val & bitmask) != 0u;
    }
    return FALSE;
}

/**
 * @fn bool isHIFWriteInterruptEnabled(UInt8* hif_reg)
 *
 * @brief Determines if all of the specified write is enabled.
 *
 * @param    hif_reg     The register to check
 *
 * @retval   TRUE        The specified interrupt is enabled
 * @retval   FALSE       Not pending
 */
static bool ALWAYS_INLINE isHIFWriteInterruptEnabled(UInt8 *hif_reg)
{
    union {
        UInt8* address;
        UInt32 value;
    } addr = {.address = hif_reg};
    SInt8 reg = (SInt8)addr.value;
    reg -= HIF_WRITE_INTERRUPT_OFFSET;
    if(reg >= 0)
    {
        UInt16 bitmask = 1u << reg;

        return (PIRQ.Hrwirqenvalue.bits.hwr_irq_en_val & bitmask) != 0u;
    }
    return FALSE;
}

/**
 * @fn void selectEventPolarityRising(UInt32 entry);
 *
 * @brief Set the specified GPIO interrupt to trigger on the rising edge.
 *
 * @param    entry        The gpio entry to modify
 */
static void ALWAYS_INLINE selectEventPolarityRising(UInt32 entry)
{
    if(entry >= NUM_GPIO_EV_ENTRIES) return;

    UInt32 pol = PAD.Gpioevirqpolarity.r32;
    pol |= (1 << entry);
    PAD.Gpioevirqpolarity.r32 = pol;
}


/**
 * @fn void selectEventPolarityFalling(UInt32 entry);
 *
 * @brief Set the specified GPIO interrupt to trigger on the falling edge.
 *
 * @param    entry        The gpio entry to modify
 */
static void ALWAYS_INLINE selectEventPolarityFalling(UInt32 entry)
{
    if(entry >= NUM_GPIO_EV_ENTRIES) return;

    UInt32 pol = PAD.Gpioevirqpolarity.r32;
    pol &= ~(1 << entry);
    PAD.Gpioevirqpolarity.r32 = pol;
}

/**
 * @fn void selectEventPolarity(UInt32 entry, bool rising);
 *
 * @brief Set the specified GPIO interrupt to trigger on the falling edge.
 *
 * @param    entry        The gpio entry to modify
 * @param    rising        Set if the gpo interrupt should trigger on the rising edge.
 */
static void ALWAYS_INLINE selectEventPolarity(UInt32 entry, bool rising)
{
    if(entry >= NUM_GPIO_EV_ENTRIES) return;

    UInt32 pol = PAD.Gpioevirqpolarity.r32;
    if(rising)
    {
        pol &= ~(1 << entry);    // unselected
    }
    else
    {
        pol |=  (1 << entry);    // select
    }
    PAD.Gpioevirqpolarity.r32 = pol;
}
#endif /* _ARCVER || _FRAMEWORK */

#endif /* _INTERRUPT_H */
