////////////////////////////////////////////////////////////////////////////////
///
/// @file       libs/I2CInterface/includes/I2CDriverInterface.h
///
/// @project    EM7189
///
/// @brief      The following describes the I<sup>2</sup>C interface used by
///             sensor drivers to read and write I<sup>2</sup>C devices.
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
#ifndef __I2C_DRIVER_INTERFACE_H_
#define __I2C_DRIVER_INTERFACE_H_

#include <hw_versions.h>
#include <arc.h>

/*****************************/
/*        Structures         */
/*****************************/

struct I2CState;
typedef struct I2CState I2CState;

typedef struct I2C_Device_t {
    UInt16 maxClock:10;   /**< @brief Maximum clock speed supported by the device, in kHz. */
    UInt16 rsvd:6;

    UInt8  hasClockStretching:1;   /**< @brief Set high if the sensor supports clock stretching. */
    UInt8  address:7;           /**< @brief Set sensor slave address being used. */  // Note: only 7 bits are really needed here.

    UInt8 reserved_0;
} I2C_Device_t;

#if defined(_ARCVER)

#include <SensorAPI.h>

#define I2C_TRANSACTION_SIZE    (20u)

/*****************************/
/*        Functions          */
/*****************************/

/**
 * @cond Internal use only
 */
/**
 * @fn void i2c_initialize_queue(I2CTransaction* queue, UInt16 depth)
 *
 * @brief Initializes the I<sup>2</sup>C library with a buffer to contain transactions.
 *
 * @param queue                                         The I<sup>2</sup>C transaction buffer.
 * @param depth                                         The number of transactions the buffer can hold.
 */
//lint -function( fopen(1), i2c_initialize_queue(1) ) // arg cannot be null
//lint -function( fopen(1), i2c_initialize_queue(3) ) // arg cannot be null
void i2c_initialize_queue(void* buffer, UInt16 depth, I2CState* state);
/**
 * @endcond
 */


/**
 * @fn bool i2c_wait_until_queue_empty(const I2CState* state);
 *
 * @brief Waits for the i2c queue to empty.
 *
 * @deprecated          The @ref i2c_drain_queue function should be used instead.
 *
 *  Waits until the I2C queue is empty. Do note that this function
 *      will block other code from execution.
 *
 * @retval  false       No I2C transaction was pending.
 * @retval  true        The I2C master finished the transaction.
 */
//lint -function( fopen(1), i2c_wait_until_queue_empty(1) ) // arg cannot be null
bool i2c_wait_until_queue_empty(const I2CState* state) DEPRECATED("The i2c_drain_queue function should be used instead.");

/**
 * @fn bool JLI i2c_drain_queue(const I2CState* state);
 *
 * @brief Waits for all currently scheduled transactions to run.
 *
 *  Do note that this function will block other code from execution.
 *
 * @retval  false       No I2C transaction was pending.
 * @retval  true        The I2C master finished the transaction.
 */
//lint -function( fopen(1), i2c_drain_queue(1) ) // arg cannot be null
bool JLI i2c_drain_queue(const I2CState* state);

/**
 * @fn SensorStatus JLI i2c_write_data(UInt8 i2cDevice,
 *                           UInt8 reg,
 *                           const UInt8 *buffer,
 *                           UInt8 bytes, I2CState* state);
 *
 * @brief Writes data out the I<sup>2</sup>C bus.
 *
 * This is a blocking operation.
 *
 * @deprecated                      The @ref i2c_write_data_nonblocking function should be used instead.
 *
 * @param i2cDevice                 The I<sup>2</sup>C device address to communicate with.
 * @param reg                       The register to begin the write operation at.
 * @param[in] buffer                The data array to send to the I<sup>2</sup>C device.
 * @param bytes                     The number of bytes in the data array.
 * @param state                     The I<sup>2</sup>C interface handle.
 *
 * @retval SensorOK                 The communication completed successfully.
 * @retval SensorErrorNonExistant   The sensor did not respond.
 */
//lint -function( fopen(1), i2c_write_data(3) ) // arg cannot be null
//lint -function( fopen(1), i2c_write_data(5) ) // arg cannot be null
//lint -sem( i2c_write_data, (3P >= 4n) ) // bytes must be <= buffer size
SensorStatus JLI i2c_write_data(UInt8 i2cDevice,
                            UInt8 reg,
                            const UInt8 *buffer,
                            UInt8 bytes,
                            I2CState* state) DEPRECATED("The i2c_write_data_nonblocking function should be used instead.");

/**
 * @fn SensorStatus i2c_read_data(UInt8 i2cDevice,
 *              UInt8 reg,
 *              UInt8 *buffer,
 *              UInt8 bytes, I2CState* state);
 *
 * @brief Performs an I<sup>2</sup>C read operation on the selected I<sup>2</sup>C device.
 *
 * Performs and I<sup>2</sup>C read starting at the specified register.
 * Results are returned are saved in the buffer specified.
 * This is a blocking operation.
 *
 * @deprecated                      The @ref i2c_read_data_nonblocking function should be used instead.
 *
 * @param i2cDevice                 The I<sup>2</sup>C device address to communicate with.
 * @param reg                       The register to begin the read operation at.
 * @param[out] buffer               The memory location to write the results to.
 * @param bytes                     The number of bytes to read in.
 * @param state                     The I<sup>2</sup>C interface handle.
 *
 * @retval SensorOK                 The communication completed successfully.
 * @retval SensorErrorNonExistant   The sensor did not respond.
 */
//lint -function( fopen(1), i2c_read_data(3) ) // arg cannot be null
//lint -function( fopen(1), i2c_read_data(5) ) // arg cannot be null
//lint -sem( i2c_read_data, (3P >= 4n) ) // bytes must be <= buffer size
SensorStatus JLI i2c_read_data(UInt8 i2cDevice,
                           UInt8 reg,
                           UInt8 *buffer,
                           UInt8 bytes,
                           I2CState* state) DEPRECATED("The i2c_read_data_nonblocking function should be used instead.");

/**
 * @fn void i2c_write_data_nonblocking(UInt8 i2cDevice,
 *              UInt8 reg,
 *              const UInt8 *buffer,
 *              UInt8 bytes,
 *              void (*callback_fn)(SensorStatus, void*),
 *              void* data, I2CState* state);
 *
 *
 * @brief Performs an I<sup>2</sup>C write operation on the selected I<sup>2</sup>C device.
 *
 * This is a non-blocking operation, the input buffer cannot be modified until the
 * transaction is complete. Note: If the buffer is on the stack, you must ensure that
 * the transaction completes before the function exits.
 *
 * @param i2cDevice                 The I<sup>2</sup>C device address to communicate with.
 * @param reg                       The register to write to.
 * @param[in] buffer                An array of data to write to the device.
 * @param bytes                     The number of entries in the source array.
 * @param[in] callback_fn           The function to call after the I<sup>2</sup>C transaction is complete, the return status is passed as an argument.
 * @param[in] data                  The data to pass to the callback function.
 * @param state                     The I<sup>2</sup>C interface handle.
 */
//lint -function( fopen(1), i2c_write_data_nonblocking(3) ) // arg cannot be null
//lint -function( fopen(1), i2c_write_data_nonblocking(7) ) // arg cannot be null
//lint -sem( i2c_write_data_nonblocking, (3P >= 4n) ) // bytes must be <= buffer size
//lint -sem( i2c_write_data_nonblocking, thread_create(5))
void JLI i2c_write_data_nonblocking(UInt8 i2cDevice,
                UInt8 reg,
                const UInt8 *buffer,
                UInt8 bytes,
                void (*callback_fn)(SensorStatus, void*),
                void* data,
                I2CState* state);

/**
 * @fn void i2c_write_data_inline_nonblocking(UInt8 i2cDevice,
 *              UInt8 reg,
 *              const UInt8 *buffer,
 *              UInt8 bytes,
 *              void (*callback_fn)(SensorStatus, void*),
 *              void* data, I2CState* state);
 *
 *
 * @brief Performs an I<sup>2</sup>C write operation on the selected I<sup>2</sup>C device.
 *
 * This is a non-blocking operation.
 *
 * @param i2cDevice                 The I<sup>2</sup>C device address to communicate with.
 * @param reg                       The register to write to.
 * @param[in] buffer                An array of data to write to the device.
 * @param bytes                     The number of entries in the source array.
 * @param[in] callback_fn           The function to call after the I<sup>2</sup>C transaction is complete, the return status is passed as an argument.
 * @param[in] data                  The data to pass to the callback function.
 * @param state                     The I<sup>2</sup>C interface handle.
 */
//lint -function( fopen(1), i2c_write_data_inline_nonblocking(3) ) // arg cannot be null
//lint -function( fopen(1), i2c_write_data_inline_nonblocking(7) ) // arg cannot be null
//lint -sem( i2c_write_data_inline_nonblocking, 4n <= 4) // bytes must be 4 bytes or less
//lint -sem( i2c_write_data_inline_nonblocking, (3P >= 4n) ) // bytes must be <= buffer size
//lint -sem( i2c_write_data_inline_nonblocking, thread_create(5))
void JLI i2c_write_data_inline_nonblocking(UInt8 i2cDevice,
                UInt8 reg,
                const UInt8 *buffer,
                UInt8 bytes,
                void (*callback_fn)(SensorStatus, void*),
                void* data,
                I2CState* state);


/**
 * @fn void i2c_write_data_slow_nonblocking(UInt8 i2cDevice,
 *              UInt8 reg,
 *              const UInt8 *buffer,
 *              UInt8 bytes,
 *              void (*callback_fn)(SensorStatus, void*),
 *              void* data,
 *              float delay, I2CState* state);
 *
 *
 * @brief Performs an I<sup>2</sup>C write operation on the selected I<sup>2</sup>C device.
 *
 * This is a non-blocking operation, the input buffer cannot be modified until the
 * transaction is complete. Note: If the buffer is on the stack, you must ensure that
 * the transaction completes before the function exits.
 *
 * @param i2cDevice                 The I<sup>2</sup>C device address to communicate with.
 * @param reg                       The register to write to.
 * @param[in] buffer                An array of data to write to the device.
 * @param bytes                     The number of entries in the source array.
 * @param[in] callback_fn           The function to call after the I<sup>2</sup>C transaction is complete, the return status is passed as an argument.
 * @param[in] data                  The data to pass to the callback function.
 * @param delay                     The delay in ms between bytes in the transaction, and the start of the next transaction.
 * @param state                     The I<sup>2</sup>C interface handle.
*/
//lint -function( fopen(1), i2c_write_data_slow_nonblocking(3) ) // arg cannot be null
//lint -function( fopen(1), i2c_write_data_slow_nonblocking(8) ) // arg cannot be null
//lint -sem( i2c_write_data_slow_nonblocking, 3P >= 4n ) // bytes must be <= buffer size
//lint -sem( i2c_write_data_slow_nonblocking, thread_create(5))
void JLI i2c_write_data_slow_nonblocking(UInt8 i2cDevice,
                                UInt8 reg,
                                const UInt8 *buffer,
                                UInt8 bytes,
                                void (*callback_fn)(SensorStatus, void*),
                                void* data,
                                float delay,
                                I2CState* state);

/**
 * @fn void i2c_write_data_slow_inline_nonblocking(UInt8 i2cDevice,
 *              UInt8 reg,
 *              const UInt8 *buffer,
 *              UInt8 bytes,
 *              void (*callback_fn)(SensorStatus, void*),
 *              void* data,
 *              float delay, I2CState* state);
 *
 *
 * @brief Performs an I<sup>2</sup>C write operation on the selected I<sup>2</sup>C device.
 *
 * This is a non-blocking operation.
 *
 * @param i2cDevice                 The I<sup>2</sup>C device address to communicate with.
 * @param reg                       The register to write to.
 * @param[in] buffer                An array of data to write to the device.
 * @param bytes                     The number of entries in the source array.
 * @param[in] callback_fn           The function to call after the I<sup>2</sup>C transaction is complete, the return status is passed as an argument.
 * @param[in] data                  The data to pass to the callback function.
 * @param delay                     The delay in ms between bytes in the transaction, and the start of the next transaction.
 * @param state                     The I<sup>2</sup>C interface handle.
 */
//lint -function( fopen(1), i2c_write_data_slow_inline_nonblocking(3) ) // arg cannot be null
//lint -function( fopen(1), i2c_write_data_slow_inline_nonblocking(8) ) // arg cannot be null
//lint -sem( i2c_write_data_slow_inline_nonblocking, 4n <= 4) // bytes must be 4 bytes or less
//lint -sem( i2c_write_data_slow_inline_nonblocking, 3P >= 4n ) // bytes must be <= buffer size
//lint -sem( i2c_write_data_slow_inline_nonblocking, thread_create(5))
void JLI i2c_write_data_slow_inline_nonblocking(UInt8 i2cDevice,
                                UInt8 reg,
                                const UInt8 *buffer,
                                UInt8 bytes,
                                void (*callback_fn)(SensorStatus, void*),
                                void* data,
                                float delay,
                                I2CState* state);
/**
 * @fn void i2c_read_data_nonblocking(UInt8 i2cDevice,
 *              UInt8 reg,
 *              UInt8 *buffer,
 *              UInt8 bytes,
 *              void (*callback_fn)(SensorStatus, void*),
 *              void* data, I2CState* state);
 *
 * @brief Performs one or more I<sup>2</sup>C read operations on the selected I<sup>2</sup>C device.
 *
 * This is a non-blocking operation. The input buffer will be overwritten once the I<sup>2</sup>C transaction has completed.
 *
 * @param i2cDevice             The I<sup>2</sup>C device address to communicate with.
 * @param reg                   The register to read from.
 * @param[out] buffer           The memory location to save the read data.
 * @param bytes                 The number of bytes to read.
 * @param[in] callback_fn       The function to call after the I<sup>2</sup>C transaction is complete, the return status is passed as an argument.
 * @param[in] data              The data to pass to the callback function.
 * @param state                 The I<sup>2</sup>C interface handle.
 */
//lint -function( fopen(1), i2c_read_data_nonblocking(3) ) // arg cannot be null
//lint -function( fopen(1), i2c_read_data_nonblocking(7) ) // arg cannot be null
//lint -sem( i2c_read_data_nonblocking, (3P >= 4n) ) // bytes must be <= buffer size
//lint -sem( i2c_read_data_nonblocking, thread_create(5))
void JLI i2c_read_data_nonblocking(UInt8 i2cDevice,
                               UInt8 reg,
                               UInt8 *buffer,
                               UInt8 bytes,
                               void (*callback_fn)(SensorStatus, void*),
                               void* data,
                               I2CState* state);

/**
 * @fn void i2c_read_data_slow_nonblocking(UInt8 i2cDevice,
 *              UInt8 reg,
 *              UInt8 *buffer,
 *              UInt8 bytes,
 *              void (*callback_fn)(SensorStatus, void*),
 *              void* data,
 *              float delay, I2CState* state);
 *
 * @brief Performs one or more I<sup>2</sup>C read operations on the selected I<sup>2</sup>C device.
 *
 * This is a non-blocking operation. The input buffer will be overwritten once the I<sup>2</sup>C transaction has completed.
 *
 * @param i2cDevice             The I<sup>2</sup>C device address to communicate with.
 * @param reg                   The register to read from.
 * @param[out] buffer           The memory location to save the read data.
 * @param bytes                 The number of bytes to read.
 * @param[in] callback_fn       The function to call after the I<sup>2</sup>C transaction is complete, the return status is passed as an argument.
 * @param[in] data              The data to pass to the callback function.
 * @param delay                 The delay in ms between bytes in the transaction, and the start of the next transaction.
 * @param state                 The I<sup>2</sup>C interface handle.
 */
//lint -function( fopen(1), i2c_read_data_slow_nonblocking(3) ) // arg cannot be null
//lint -function( fopen(1), i2c_read_data_slow_nonblocking(8) ) // arg cannot be null
//lint -sem( i2c_read_data_slow_nonblocking, (3P >= 4n) ) // bytes must be <= buffer size
//lint -sem( i2c_read_data_slow_nonblocking, thread_create(5))
void JLI i2c_read_data_slow_nonblocking(UInt8 i2cDevice,
                                UInt8 reg,
                                UInt8 *buffer,
                                UInt8 bytes,
                                void (*callback_fn)(SensorStatus, void*),
                                void* data,
                                float delay,
                                I2CState* state);

/**
 * @fn void i2c_read_bytes_nonblocking(UInt8 i2cDevice,
 *              UInt8 *buffer,
 *              UInt8 bytes,
 *              void (*callback_fn)(SensorStatus, void*),
 *              void* data, I2CState* state);
 *
 * @brief Performs one or more I<sup>2</sup>C read operations on the selected I<sup>2</sup>C device.
 *
 * This is a non-blocking operation. The input buffer will be overwritten once the I<sup>2</sup>C transaction has completed.
 *
 * @param i2cDevice             The I<sup>2</sup>C device address to communicate with.
 * @param[out] buffer           The memory location to save the read data.
 * @param bytes                 The number of bytes to read.
 * @param[in] callback_fn       The function to call after the I<sup>2</sup>C transaction is complete, the return status is passed as an argument.
 * @param[in] data              The data to pass to the callback function.
 * @param state                 The I<sup>2</sup>C interface handle.
 */
//lint -function( fopen(1), i2c_read_bytes_nonblocking(2) ) // arg cannot be null
//lint -function( fopen(1), i2c_read_bytes_nonblocking(6) ) // arg cannot be null
//lint -sem( i2c_read_bytes_nonblocking, (2P >= 3n) ) // bytes must be <= buffer size
//lint -sem( i2c_read_bytes_nonblocking, thread_create(4))
void JLI i2c_read_bytes_nonblocking(UInt8 i2cDevice,
                                UInt8 *buffer,
                                UInt8 bytes,
                                void (*callback_fn)(SensorStatus, void*),
                                void* data,
                                I2CState* state);

/**
 * @fn void i2c_read_bytes_slow_nonblocking(UInt8 i2cDevice,
 *              UInt8 *buffer,
 *              UInt8 bytes,
 *              void (*callback_fn)(SensorStatus, void*),
 *              void* data, float delay, I2CState* state);
 *
 * @brief Performs one or more I<sup>2</sup>C read operations on the selected I<sup>2</sup>C device.
 *
 * This is a non-blocking operation. The input buffer will be overwritten once the I<sup>2</sup>C transaction has completed.
 *
 * @param i2cDevice             The I<sup>2</sup>C device address to communicate with.
 * @param[out] buffer           The memory location to save the read data.
 * @param bytes                 The number of bytes to read.
 * @param[in] callback_fn       The function to call after the I<sup>2</sup>C transaction is complete, the return status is passed as an argument.
 * @param[in] data              The data to pass to the callback function.
 * @param delay                 The delay in ms between bytes in the transaction, and the start of the next transaction.
 * @param state                 The I<sup>2</sup>C interface handle.
 */
//lint -function( fopen(1), i2c_read_bytes_slow_nonblocking(2) ) // arg cannot be null
//lint -function( fopen(1), i2c_read_bytes_slow_nonblocking(7) ) // arg cannot be null
//lint -sem( i2c_read_bytes_slow_nonblocking, (2P >= 3n) ) // bytes must be <= buffer size
//lint -sem( i2c_read_bytes_slow_nonblocking, thread_create(4))
void JLI i2c_read_bytes_slow_nonblocking(UInt8 i2cDevice,
                                UInt8 *buffer,
                                UInt8 bytes,
                                void (*callback_fn)(SensorStatus, void*),
                                void* data,
                                float delay,
                                I2CState* state);
/**
 * @fn void i2c_write_bytes_nonblocking(UInt8 i2cDevice,
 *              const UInt8 *buffer,
 *              UInt8 bytes,
 *              void (*callback_fn)(SensorStatus, void*),
 *              void* data, I2CState* state);
 *
 * @brief Performs one or more I<sup>2</sup>C write operations on the selected I<sup>2</sup>C device.
 *
 * This is a non-blocking operation.
 *
 * @param i2cDevice             The I<sup>2</sup>C device address to communicate with.
 * @param[in] buffer            An array of data to write to the device.
 * @param bytes                 The number of entries in the source array.
 * @param[in] callback_fn       The function to call after the I<sup>2</sup>C transaction is complete, the return status is passed as an argument.
 * @param[in] data              The data to pass to the callback function.
 * @param state                 The I<sup>2</sup>C interface handle.
 */
//lint -function( fopen(1), i2c_write_bytes_nonblocking(2) ) // arg cannot be null
//lint -function( fopen(1), i2c_write_bytes_nonblocking(6) ) // arg cannot be null
//lint -sem( i2c_write_bytes_nonblocking, (2P >= 3n) ) // bytes must be <= buffer size
//lint -sem( i2c_write_bytes_nonblocking, thread_create(4))
void JLI i2c_write_bytes_nonblocking(UInt8 i2cDevice,
                                const UInt8 *buffer,
                                UInt8 bytes,
                                void (*callback_fn)(SensorStatus, void*),
                                void* data,
                                I2CState* state);

/**
 * @fn void i2c_write_bytes_inline_nonblocking(UInt8 i2cDevice,
 *              const UInt8 *buffer,
 *              UInt8 bytes,
 *              void (*callback_fn)(SensorStatus, void*),
 *              void* data, I2CState* state);
 *
 * @brief Performs one or more I<sup>2</sup>C write operations on the selected I<sup>2</sup>C device.
 *
 * This is a non-blocking operation.
 *
 * @param i2cDevice             The I<sup>2</sup>C device address to communicate with.
 * @param[in] buffer            An array of data to write to the device.
 * @param bytes                 The number of entries in the source array.
 * @param[in] callback_fn       The function to call after the I<sup>2</sup>C transaction is complete, the return status is passed as an argument.
 * @param[in] data              The data to pass to the callback function.
 * @param state                 The I<sup>2</sup>C interface handle.
 */
//lint -function( fopen(1), i2c_write_bytes_inline_nonblocking(2) ) // arg cannot be null
//lint -function( fopen(1), i2c_write_bytes_inline_nonblocking(6) ) // arg cannot be null
//lint -sem( i2c_write_bytes_inline_nonblocking, 4n <= 4) // bytes must be 4 bytes or less
//lint -sem( i2c_write_bytes_inline_nonblocking, (2P >= 3n) ) // bytes must be <= buffer size
//lint -sem( i2c_write_bytes_inline_nonblocking, thread_create(4))
void JLI i2c_write_bytes_inline_nonblocking(UInt8 i2cDevice,
                                const UInt8 *buffer,
                                UInt8 bytes,
                                void (*callback_fn)(SensorStatus, void*),
                                void* data,
                                I2CState* state);

/**
 * @fn void i2c_write_bytes_slow_nonblocking(UInt8 i2cDevice,
 *              const UInt8 *buffer,
 *              UInt8 bytes,
 *              void (*callback_fn)(SensorStatus, void*),
 *              void* data, I2CState* state);
 *
 * @brief Performs one or more I<sup>2</sup>C write operations on the selected I<sup>2</sup>C device.
 *
 * This is a non-blocking operation.
 *
 * @param i2cDevice             The I<sup>2</sup>C device address to communicate with.
 * @param[in] buffer            An array of data to write to the device.
 * @param bytes                 The number of entries in the source array.
 * @param[in] callback_fn       The function to call after the I<sup>2</sup>C transaction is complete, the return status is passed as an argument.
 * @param[in] data              The data to pass to the callback function.
 * @param delay                 The delay in ms between bytes in the transaction, and the start of the next transaction.
 * @param state                 The I<sup>2</sup>C interface handle.
 */
//lint -function( fopen(1), i2c_write_bytes_slow_nonblocking(2) ) // arg cannot be null
//lint -function( fopen(1), i2c_write_bytes_slow_nonblocking(7) ) // arg cannot be null
//lint -sem( i2c_write_bytes_slow_nonblocking, (2P >= 3n) ) // bytes must be <= buffer size
//lint -sem( i2c_write_bytes_slow_nonblocking, thread_create(4))
void JLI i2c_write_bytes_slow_nonblocking(UInt8 i2cDevice,
                                const UInt8 *buffer,
                                UInt8 bytes,
                                void (*callback_fn)(SensorStatus, void*),
                                void* data,
                                float delay,
                                I2CState* state);

/**
 * @fn void i2c_write_bytes_slow_inline_nonblocking(UInt8 i2cDevice,
 *              const UInt8 *buffer,
 *              UInt8 bytes,
 *              void (*callback_fn)(SensorStatus, void*),
 *              void* data, float delay, I2CState* state);
 *
 * @brief Performs one or more I<sup>2</sup>C write operations on the selected I<sup>2</sup>C device.
 *
 * This is a non-blocking operation.
 *
 * @param i2cDevice             The I<sup>2</sup>C device address to communicate with.
 * @param[in] buffer            An array of data to write to the device.
 * @param bytes                 The number of entries in the source array.
 * @param[in] callback_fn       The function to call after the I<sup>2</sup>C transaction is complete, the return status is passed as an argument.
 * @param[in] data              The data to pass to the callback function.
 * @param delay                 The delay in ms between bytes in the transaction, and the start of the next transaction.
 * @param state                 The I<sup>2</sup>C interface handle.
 */
//lint -function( fopen(1), i2c_write_bytes_slow_inline_nonblocking(2) ) // arg cannot be null
//lint -function( fopen(1), i2c_write_bytes_slow_inline_nonblocking(7) ) // arg cannot be null
//lint -sem( i2c_write_bytes_slow_inline_nonblocking, 4n <= 4) // bytes must be 4 bytes or less
//lint -sem( i2c_write_bytes_slow_inline_nonblocking, (2P >= 3n) ) // bytes must be <= buffer size
//lint -sem( i2c_write_bytes_slow_inline_nonblocking, thread_create(4))
void JLI i2c_write_bytes_slow_inline_nonblocking(UInt8 i2cDevice,
                                const UInt8 *buffer,
                                UInt8 bytes,
                                void (*callback_fn)(SensorStatus, void*),
                                void* data,
                                float delay,
                                I2CState* state);

#endif /* _ARCVER */

#endif /* __I2C_DRIVER_INTERFACE_H_ */
