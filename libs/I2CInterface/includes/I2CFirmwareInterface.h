////////////////////////////////////////////////////////////////////////////////
///
/// @file       libs/I2CInterface/includes/I2CFirmwareInterface.h
///
/// @project    EM7189
///
/// @brief      The following describes the I2C interface used by
///             firmware to configure and utilize the i2c master.
///             These functions and registers should <b>not</b> be used
///             by sensor drivers, they will be modified when appropriate.
///
/// @classification  Confidential
///
////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
///
/// @copyright Copyright (C) 2012-2018 EM Microelectronic
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
#ifndef __I2C_FIRMWARE_INTERFACE_H_
#define __I2C_FIRMWARE_INTERFACE_H_

#include <types.h>
#include <arc.h>

#define MAX_I2C_TRANS_LENGTH        16

/*****************************/
/*       Data Types          */
/*****************************/

/**
 * @typedef I2CClockFrequency
 *
 * @brief Possible frequency rates supported by the I2C master.
 *
 */
typedef enum {
    ClockFrequency1000 = 0, /**< I2C Fast+ mode, 1000Khz. */
    ClockFrequency833  = 1, /**< I2C Fast+ mode, 833Khz. */
    ClockFrequency400  = 2, /**< I2C Fast mode, 400Khz. */
    ClockFrequency333  = 3, /**< I2C Fast mode, 333Khz. */
    ClockFrequency100  = 4, /**< I2C Standard mode, 100Khz. */
    ClockFrequency83   = 5, /**< I2C Standard mode, 83Khz. */
} I2CClockFrequency;

/**
 * @typedef I2CClockFrequency
 *
 * @brief Possible frequency rates supported by the I2C master.
 *
 */
typedef enum {
    I2C_WRITE                 = 0,
    I2C_READ                = 1,
    I2C_SEND_REG_ADDRESS    = 2,
} I2CTransactionType;


#ifdef _ARCVER

/*****************************/
/*        Functions          */
/*****************************/

/**
 * @fn void i2c_set_clock_frequency(I2CClockFrequency freq, volatile I2CM0_t* regs);
 *
 * @brief Sets the I2C Master to use the specified clock frequency
 *
 * @param   freq            The frequency to use for I2C transactions
 * @param   regs            The hardware registers to use
 */
void i2c_set_clock_frequency(I2CClockFrequency freq, volatile I2CM0_t* regs);

/**
 * @fn void i2c_set_clock_max_frequency(UInt16 freq, volatile I2CM0_t* regs);
 *
 * @brief Sets the I2C Master to use the specified clock frequency
 *
 * @param   freq            The frequency to use for I2C transactions
 * @param   regs            The hardware registers to use
 */
void i2c_set_clock_max_frequency(UInt16 freq, volatile I2CM0_t* regs);

/**
 * @fn I2CClockFrequency i2c_get_clock_frequency(const volatile I2CM0_t* regs);
 *
 * @brief Reads out the I2C master clock frequency
 *
 * @param   regs            The hardware registers to use
 *
 * @returns                 The frequency that the I2C master is currently set to use.
 */
I2CClockFrequency i2c_get_clock_frequency(const volatile I2CM0_t* regs);

/**
 * @fn static void ALWAYS_INLINE i2c_set_clock_stretching(bool enable, volatile I2CM0_t* regs);
 *
 * @brief Enables or disabled clock stretching on the i2c master bus.
 *
 * @param   enable          Enable or disable clock stretching.
 * @param   regs            The hardware registers to use
 */
static void ALWAYS_INLINE i2c_set_clock_stretching(bool enable, volatile I2CM0_t* regs)
{
    regs->Config.bits.ck_stretch_en = enable ? 1 : 0;
}

/**
 * @fn static bool ALWAYS_INLINE i2c_get_clock_stretching(volatile I2CM0_t* regs);
 *
 * @brief Returns the state of the clock stretching enable bit.
 *
 * @param   regs            The hardware registers to use
 *
 * @retval  false           Clock stretching is currently disabled.
 * @retval  true            Clock stretching is currently enabled.
 */
static bool ALWAYS_INLINE i2c_get_clock_stretching(volatile I2CM0_t* regs)
{
    return (regs->Config.bits.ck_stretch_en == 1);
}

/**
 * @fn static bool ALWAYS_INLINE i2c_is_busy(volatile I2CM0_t* regs);
 *
 * @brief Determines if the I2C master is currently in use.
 *
 * @param   regs            The hardware registers to use
 *
 * @retval  false           The I2C master is available.
 * @retval  true            The I2C master is in use.
 */
static bool ALWAYS_INLINE i2c_is_busy(volatile I2CM0_t* regs)
{
    return (regs->Status.bits.busy == 1);
}

/**
 * @fn bool i2c_begin_transaction(I2CTransactionType type, UInt8 i2cDevice, UInt8 i2cRegister, UInt8 numBytes, volatile I2CM0_t* regs);
 *
 * @brief Begins an I2C transaction on the specified device and register.
 *
 * Data written using this function will be taken from the I2CMasDatWrX registers.
 * Data read using this function will be stored in the I2CMasDatRdX registers.
 *
 * @param   type            Perform the specified operation type.
 * @param   i2cDevice       The I2C slave address to communicate with.
 * @param   i2cRegister     The I2C slave register address to begin writing to.
 * @param   numBytes        The number of bytes to write to the slave device.
 * @param   regs            The hardware registers to use
 *
 * @retval  false           The I2C transaction was unable to start.
 * @retval  true            The I2C transaction has started.
 */
bool i2c_begin_transaction(I2CTransactionType type, UInt8 i2cDevice, UInt8 i2cRegister, UInt8 numBytes, volatile I2CM0_t* regs);

/**
 * @fn static bool ALWAYS_INLINE i2c_begin_transaction_ctrl(RegI2CM0Control_t ctrl, volatile I2CM0_t* regs);
 *
 * @brief Begins an I2C transaction specified by the ctrl register.
 *
 * Data written using this function will be taken from the I2CMasDatWrX registers.
 * Data read using this function will be stored in the I2CMasDatRdX registers.
 *
 * @param   ctrl            New control register value.
 * @param   regs            The hardware registers to use
 *
 * @retval  false           The I2C transaction was unable to start.
 * @retval  true            The I2C transaction has started.
 */
static bool ALWAYS_INLINE i2c_begin_transaction_ctrl(RegI2CM0Control_t ctrl, volatile I2CM0_t* regs)
{
    if(i2c_is_busy(regs)) return FALSE;

    regs->Config.bits.i2cm_en = 1; // ensure enable bit is set.
    regs->Control = ctrl;

    return TRUE;
}

/**
 * @fn static bool ALWAYS_INLINE i2c_wait_until_free(volatile I2CM0_t* regs);
 *
 * @brief Waits for the i2c transaction to complete.
 *
 * Waits until the I2C transaction is complete. Do note that this function
 *  will block other code from execution. This should be used primarily
 *  for debugging purposes only.
 *
 * @param   regs            The hardware registers to use
 *
 * @retval  false           No I2C transaction was pending.
 * @retval  true            The I2C master finished the transaction.
 */
//lint -esym(534,i2c_wait_until_free)
static bool ALWAYS_INLINE i2c_wait_until_free(volatile I2CM0_t* regs)
{
    while(i2c_is_busy(regs))
    {
        SleepCPU();
    }

    return TRUE;
}

/**
 * @fn static bool ALWAYS_INLINE i2c_did_ack(volatile I2CM0_t* regs);
 *
 * @brief Waits for the i2c transaction to complete.
 *
 * @param   regs            The hardware registers to use
 *
 * @retval  false           A NACK was received.
 * @retval  true            An ACK was received.
 */
static bool ALWAYS_INLINE i2c_did_ack(volatile I2CM0_t* regs)
{
    if(regs->Status.bits.pause_stat) return TRUE; // treat as an ack for the driver interface.
    return !regs->Status.bits.nak_rcvd;
}


#endif /* _ARCVER */

#endif /* __I2C_FIRMWARE_INTERFACE_H_ */
