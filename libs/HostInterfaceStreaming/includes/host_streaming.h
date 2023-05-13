////////////////////////////////////////////////////////////////////////////////
///
/// @file       libs/HostInterfaceStreaming/includes/host_streaming.h
///
/// @project    EM7189
///
/// @brief      APIs for accessing the Streaming Host Interface
///
/// The below sequence diagrams show how these APIs could be used to implement an HID wrapper library.
/// @par Initializing the wrapper library
/// In order to use these APIs, the IRQ_trapSetInterruptStatus() and FIFO_trapOutputFIFO() APIs must be used.
/// Additionally, the IRQ_configureInterrupt() API should be used if a custom configuration is required. @n
/// If IRQ_configureInterrupt() is not used, the default state defined by the host interface will be used.
/// @image html libs/HostInterfaceStreaming/figures/HOOK_HIDLibrary.svg "Initialization"
///
/// @par FIFO Output Sequences
/// The first figure shows how to begin a new transfer when a FIFO callback has triggered. @n
/// Note that while a FIFO callback will not be called more than once for each FIFO_endFIFOTransfer(), multiple FIFOs can
/// trigger at the same time. Special care may be needed in this case to ensure that the DMA and IRQ APIs are managed properly.
/// @image html libs/HostInterfaceStreaming/figures/FIFO_trapOutputFIFO.svg "Initial FIFO Transaction"
///
/// Once the host has read out the packet, the DMA callback will fire.
/// @image html libs/HostInterfaceStreaming/figures/DMA_sendBufferU8.svg "DMA callbacks with more FIFO data"
///
/// Once all data has been read out from the host, the FIFO_endFIFOTransfer() API must be called to enable further FIFO transfers.
/// @image html libs/HostInterfaceStreaming/figures/DMA_sendBufferU8_last.svg "DMA callbacks with all data transferred"
///
///
/// @classification  Confidential
///
////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
///
/// @copyright Copyright (C) 2018 EM Microelectronic
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

#ifndef HOST_STREAMING_H
#define HOST_STREAMING_H

#include <types.h>
#include <arc.h>
#include <SensorAPI.h>

typedef enum {
    FIFO_TYPE_WAKEUP = 1,    /**< The Wakeup FIFO ID. Corresponds to the host channel id. */
    FIFO_TYPE_NONWAKEUP = 2, /**< The Non-Wakeup FIFO ID. Corresponds to the host channel id. */
    FIFO_TYPE_STATUS = 3,    /**< The Status and Debug FIFO ID. Corresponds to the host channel id. */
} fifo_t;


/******************************************************************************/
/*                 Interrupt APIS                                             */
/******************************************************************************/
/**
 * @fn void IRQ_trapSetInterruptStatus(void (*handler)(RegPROCInterruptStatus_t set_status));
 *
 * @brief Causes the handler to be called when the Host Interface would normally
 *    call IRQ_setInterruptStatus()
 *
 * Note: The host interface will trigger the debug interrupt in the event of printf data.
 * Note: Once the IRQ_trapSetInterruptStatus handler is updated, the host interface will never trigger the status interrupt.
 * Note: The host  interface will never trigger the reset_or_fault interrupt outside of the bootloader.
 *
 * @param handler   The setInterruptStatus handler to install.
 * @par Handler Parameters
 * @b set_status    The status bits that the host interface is attempting to set.
 *
 * @requirement{REQ-IRQ00} Once called, interrupts from the host interface will call the handler instead of asserting the interrupt.
 */
//lint -sem( IRQ_trapSetInterruptStatus, 1p) // Must not be NULL
void JLI IRQ_trapSetInterruptStatus(void (*handler)(RegPROCInterruptStatus_t set_status));

/**
 * @fn void JLI IRQ_configureInterrupt(RegHOSTHostInterruptControl_t config)
 *
 * @brief Configures the host interrupt to behave as requested.
 *
 * Note: This should only be called when no interrupts are pending. If this is
 *      called with pending interrupts, the current interrupt state will not be
 *      modified, but the configuration will be used starting with the next
 *      interrupt.
 *
 * @param config.wakeup_fifo_interrupt_mask         When set, blocks IRQ_setInterruptStatus() from triggering a wakeup interrupt.
 * @param config.nonwakeup_fifo_interrupt_mask      When set, blocks IRQ_setInterruptStatus() from triggering a nonwakeup interrupt.
 * @param config.status_available_interrupt_mask    When set, blocks IRQ_setInterruptStatus() from triggering a status interrupt.
 * @param config.debug_available_interrupt_mask     When set, blocks IRQ_setInterruptStatus() from triggering a debug interrupt.
 * @param config.active_low_interrupt               When set, the host interrupt will assert low.
 * @param config.edge_interrupt                     When set, the host interrupt will pulse.
 * @param config.open_drain_interrupt               When set, the host interrupt will only drive when asserted.
 *
 * @requirement{REQ-IRQ10} If @ref RegHOSTHostInterruptControl_t.wakeup_fifo_interrupt_mask "config.wakeup_fifo_interrupt_mask" is set, calls to IRQ_setInterruptStatus() will not trigger a wakeup interrupt
 * @requirement{REQ-IRQ11} If @ref RegHOSTHostInterruptControl_t.nonwakeup_fifo_interrupt_mask "config.nonwakeup_fifo_interrupt_mask" is set, calls to IRQ_setInterruptStatus() will not trigger a nonwakeup interrupt
 * @requirement{REQ-IRQ12} If @ref RegHOSTHostInterruptControl_t.status_available_interrupt_mask "config.status_available_interrupt_mask" is set, calls to IRQ_setInterruptStatus() will not trigger a status interrupt
 * @requirement{REQ-IRQ13} If @ref RegHOSTHostInterruptControl_t.debug_available_interrupt_mask "config.debug_available_interrupt_mask" is set, calls to IRQ_setInterruptStatus() will not trigger a debug interrupt
 * @requirement{REQ-IRQ14} When @ref RegHOSTHostInterruptControl_t.active_low_interrupt "config.active_low_interrupt" is set, the active level will be low and the inactive level will be high.
 * @requirement{REQ-IRQ15} When @ref RegHOSTHostInterruptControl_t.active_low_interrupt "config.active_low_interrupt" is clear, the active level will be high and the active level will be low.
 * @requirement{REQ-IRQ16} When @ref RegHOSTHostInterruptControl_t.edge_interrupt "config.edge_interrupt" is set, the GPIO pin will enter its inactive state 10us after going active.
 * @requirement{REQ-IRQ17} When @ref RegHOSTHostInterruptControl_t.open_drain_interrupt "config.open_drain_interrupt" is set, an inactive interrupt will place the GPIO pin in a HighZ state.
 * @requirement{REQ-IRQ18} When @ref RegHOSTHostInterruptControl_t.open_drain_interrupt "config.open_drain_interrupt" is clear, an inactive interrupt will drive the GPIO pin to the inactive level.
 */
void JLI IRQ_configureInterrupt(RegHOSTHostInterruptControl_t config);

/**
 * @fn void JLI IRQ_setInterruptStatus(RegPROCInterruptStatus_t set_status)
 *
 * @brief Causes an interrupt to be sent to the host if it's not masked.
 *
 * Note: Firmware will call @ref IRQ_clearInterruptStatus for all bits after the initOnce hook is called.
 *
 * @param set_status  The interrupt bits to set in the Interrupt Status register.
 *
 * @requirement{REQ-IRQ20} When called, cases the interrupt state to become active if a pending interrupt or an interrupt in set_status was enabled by IRQ_configureInterrupt().
 */
void JLI IRQ_setInterruptStatus(RegPROCInterruptStatus_t set_status);

/**
 * @fn void JLI IRQ_clearInterruptStatus(RegPROCInterruptStatus_t clear_status)
 *
 * @brief Clears the requested bits in the status register.
 *     If an interrupt was asserted and all bits are now clear, the host
 *     interrupt will deassert.
 *
 * Note: Firmware will call @ref IRQ_clearInterruptStatus for all bits after the initOnce hook is called.
 *
 * @param clear_status  The interrupt bits to clear in the Interrupt Status register.
 *
 * @requirement{REQ-IRQ30} When called, clears all pending interrupts with a bit set in clear_status.
 * @requirement{REQ-IRQ31} If no pending interrupts remain, causes the interrupt to enter the inactive state.
 */
void JLI IRQ_clearInterruptStatus(RegPROCInterruptStatus_t clear_status);

/**
 * @fn bool JLI IRQ_isInterruptEnabled(RegPROCInterruptStatus_t selected);
 *
 * @brief Determines if an interrupt is enabled.
 *
 * @param selected  The interrupt bits to check.
 *
 * @retval true     One of the selected interrupts is enabled.
 * @retval false    None of the selected interrupts are enabled.
 *
 * @requirement{REQ-IRQ40} Returns true if IRQ_configureInterrupt() was called with any of the selected interrupts set.
 * @requirement{REQ-IRQ41} Returns false if IRQ_configureInterrupt() was called with none of the selected interrupts set.
 */
bool JLI IRQ_isInterruptEnabled(RegPROCInterruptStatus_t chk);

/**
 * @fn bool JLI IRQ_isInterruptPending(RegPROCInterruptStatus_t selected);
 *
 * @brief Determines if an interrupt is pending.
 *
 * @param selected  The interrupt bits to check.
 *
 * @retval true     One of the selected interrupts is pending.
 * @retval false    None of the selected interrupts are pending.
 *
 * @requirement{REQ-IRQ50} Returns true if any of the selected interrupts are pending due to IRQ_setInterruptStatus().
 * @requirement{REQ-IRQ51} Returns false if all of the selected interrupts are cleared due to IRQ_clearInterruptStatus().
 */
bool JLI IRQ_isInterruptPending(RegPROCInterruptStatus_t chk);


/******************************************************************************/
/*                 FIFO APIS                                                  */
/******************************************************************************/
/**
 * @fn void FIFO_trapOutputFIFO(bool (*handler)(fifo_t fifo, UInt32 bytes))
 *
 * @brief Causes the handler to be called when the Streaming Host Interface would
 *    normally cause a FIFO to be outputted based on the requested triggers.
 *
 * Possible trigger cases include new sensor data via reportSensorEvent(), a
 *    latency timeout if set, or a programmed watermark being reached.
 *
 * Note: The fifo length (in bytes) is <b>not</b> placed in the FIFO byte steam.
 *    The value is passed as an argument to the handler instead.
 *
 * @param handler   The outputFIFO handler to install.
 * @par Handler Parameters
 * @b fifo      The FIFO with output events @n
 * @b bytes     The number of bytes to output
 *
 *
 * @par Handler Returns
 * @b true      The output was handled. @n
 * @b false     The output could not be handled.
 *
 * @requirement{REQ-FIFO00} Causes all FIFO triggers to cause the handler to be called.
 * @requirement{REQ-FIFO01} Once the handler is called, no additional calls will occur before FIFO_endFIFOTransfer() is called for the specified FIFO.
 */
//lint -sem( FIFO_trapOutputFIFO, 1p) // Must not be NULL
void JLI FIFO_trapOutputFIFO(bool (*handler)(fifo_t fifo,
                                        UInt32 bytes));

/**
 * @fn UInt16 JLI FIFO_getEventSizeByID(UInt8 event_id);
 *
 * @brief Determines the size of an event (including the event id) in the FIFO
 *
 * @param event_id  The event id, including special event ids (@ref SENSOR_TYPE_UPPER_START and above), to look up.
 *
 * @returns the size of the event if valid, otherwise 0.
 *
 * @requirement{REQ-FIFO10} If the event_id corresponds to a virtual sensor, @ref VirtualSensorDescriptor.outputPacketSize "outputPacketSize" + 1 is returned.
 * @requirement{REQ-FIFO11} If the event_id corresponds to a special sensor (@ref SENSOR_TYPE_UPPER_START and above), the associated size is returned.
 * @requirement{REQ-FIFO12} If the event_id is unknown, 0 is returned.
 */
UInt16 JLI FIFO_getEventSizeByID(UInt8 event_id);

/**
 * @fn UInt16 JLI FIFO_getEventSizeByDescriptor(const VirtualSensorDescriptor *p);
 *
 * @brief Determines the size of an event (including the event id) in the FIFO
 *
 * @param p     The virtual sensor descriptor to lookup.
 *
 * @returns     The size of the associated event, otherwise 0 if a NULL descriptor is specified.
 *
 * NOTE: If a non-NULL pointer is provided, p must point to a VirtualSensorDesciprtor.
 *
 * @requirement{REQ-FIFO20} If the descriptor is NULL (or invalid), 0 is returned.
 * @requirement{REQ-FIFO21} If the descriptor is valid, @ref VirtualSensorDescriptor.outputPacketSize "outputPacketSize" + 1 is returned.
 */
UInt16 JLI FIFO_getEventSizeByDescriptor(const VirtualSensorDescriptor *p);

/**
 * @fn UInt16 JLI FIFO_getEventSize(UInt8 event_id, const VirtualSensorDescriptor *p);
 *
 * @brief Determines the size of an event (including the event id) in the FIFO
 *
 * Note: This routine uses event_id only if p is NULL. If NULL is passed in event_id must be a special event id.
 *
 * @param event_id  The event id for special event (@ref SENSOR_TYPE_UPPER_START and above), to look up.
 * @param p     The virtual sensor descriptor to lookup.
 *
 * @returns     The size of the associated event, otherwise 0 if an invalid descriptor.
 *
 * @requirement{REQ-FIFO30} If the descriptor is valid, @ref VirtualSensorDescriptor.outputPacketSize "outputPacketSize" + 1 is returned.
 * @requirement{REQ-FIFO31} If the event_id corresponds to a special sensor (@ref SENSOR_TYPE_UPPER_START and above), the associated size is returned.
 * @requirement{REQ-FIFO32} If the descriptor is invalid and the event_id is unknown, 0 is returned.
 */
UInt16 JLI FIFO_getEventSize(UInt8 event_id, const VirtualSensorDescriptor *p);

/**
 * @fn void* ALIGNED(4) JLI FIFO_getNextBlock(fifo_t fifo, UInt32* bytes);
 *
 * @brief Returns the location and size of the next FIFO block.
 *
 * @param fifo          The FIFO to get the next block from.
 * @param[out] bytes    The number of bytes in the returned block. Only modified if non-NULL is returned.
 *
 * @returns             The address of the next block, or NULL if no next block.
 *
 * @requirement{REQ-FIFO40} Returns the address of the next block that is ready to be sent but not yet active.
 * @requirement{REQ-FIFO41} Returns NULL if no blocks are ready to be sent or if they are active.
 */
void* ALIGNED(4) JLI FIFO_getNextBlock(fifo_t fifo, UInt32* bytes);

/**
 * @fn bool JLI FIFO_finishBlock(fifo_t fifo);
 *
 * @brief   Marks the oldest FIFO block as finished and ready for reclamation.
 *              Once called, the FIFO block will be re-used for new sensor data.
 *
 * @param fifo      The FIFO to finish.
 *
 * @retval          true    The FIFO's oldest block has been finished.
 * @retval          false   No block exists that can be finished.
 *
 * @requirement{REQ-FIFO50} Returns true if the oldest active block was marked as finished.
 * @requirement{REQ-FIFO51} Returns false if no active blocks exist.
 */
bool JLI FIFO_finishBlock(fifo_t fifo);

/**
 * @fn void JLI FIFO_endFIFOTransfer(fifo_t fifo);
 *
 * @brief   Marks the FIFO output transfer as complete to enable additional outputs.
 *
 * Note: This routine must only be called after all FIFO data has been retrieved
 *           using FIFO_getNextBlock() and released using FIFO_finishBlock().
 *
 * @param fifo      The FIFO transfer to end.
 *
 * @requirement{REQ-FIFO60} When called, marks the current FIFO output transfer as complete.
 * @requirement{REQ-FIFO61} Ending the FIFO transfer will not impact current interrupt states.
 */
void JLI FIFO_endFIFOTransfer(fifo_t fifo);


/******************************************************************************/
/*                 Command APIs                                               */
/******************************************************************************/
/**
 * @fn void CMD_trapStatus(bool (*handler)(const void *status, UInt16 bytes))
 *
 * @brief Causes the handler to be called when the Streaming Host Interface would
 *    normally output a status event (command error, parameter read, etc).
 *
 * Note: This does not trap printf() output. All printf() output is still placed in the @ref FIFO_TYPE_STATUS FIFO.
 * Note: Once the default handler is removed, no status interrupts will be triggered, only the status handler will be called.
 *
 * @param handler   The status handler to install.
 * @par Handler Parameters
 * @b status    The status event being reported. @n
 * @b bytes     The number of bytes in the status event.
 *
 * @par Handler Returns
 * @b true      The output was handled. @n
 * @b false     The output could not be handled.
 *
 * @requirement{REQ-CMD00} printf/debug output will not be affected by CMD_trapStatus.
 * @requirement{REQ-CMD01} Once called, all status packets will be redirected to the handler.
 */
void JLI CMD_trapStatus(bool (*handler)(const void* status, UInt16 bytes));

/**
 * @fn bool JLI CMD_sendCommand(const void* buffer, UInt16 bytes);
 *
 * @brief   Sends an arbitrary command to the host interface command processor.
 *
 * Note: Command responses, if any, will be placed in the appropriate FIFO.
 *          Before calling this, the user should register the trap handler with CMD_trapStatus().
 *
 * @param buffer    The command to send.
 * @param bytes     The number of bytes in the command.
 *
 * @retval          true    The command was sent to the Streaming Host Interface.
 * @retval          false   The command could not be sent.
 */
bool JLI CMD_sendCommand(const void* buffer, UInt16 bytes);

/**
 * @fn void JLI CMD_removeDefaultHandler(void);
 *
 * @brief   Remove the previous command handler installed.
 *
 * Note: This will remove the current IRQ_trapSetInterruptStatus handler and replace it with a NULL handler.
 *
 */
void JLI CMD_removeDefaultHandler(void);

#endif /* HOST_STREAMING_H */
