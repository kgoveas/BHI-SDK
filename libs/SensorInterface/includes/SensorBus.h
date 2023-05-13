////////////////////////////////////////////////////////////////////////////////
///
/// @file       libs/SensorInterface/includes/SensorBus.h
///
/// @project    EM7189
///
/// @brief      Generic sensor interface routines.
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

/** @defgroup SENSOR_DRIVER_INT    Sensor Driver Interface */
/** @addtogroup SENSOR_DRIVER_INT
 * @{
 */

#ifndef SENSOR_BUS_H
#define SENSOR_BUS_H

#include "../../SPIInterface/includes/SPIInterface.h"
#include "../../I2CInterface/includes/I2CDriverInterface.h"

typedef enum
{
    // Valid HW selections
    InterfaceNone   = 0x00, /**< @brief The sensor does not attach to I2C or SPI, and is an interrupt input only. */

    InterfaceAnyI2C = 0x10, /**< @brief The sensor can work on any I2C master. */
    InterfaceI2C0   = 0x11, /**< @brief The sensor can work on I2C master 0 only. */
    InterfaceI2C1   = 0x12, /**< @brief The sensor can work on I2C master 1 only. */

    InterfaceAnySPI = 0x20, /**< @brief The sensor can work on any SPI master. */
    InterfaceSPI0   = 0x21, /**< @brief The sensor can work on SPI master 0 only. */
    InterfaceSPI1   = 0x22, /**< @brief The sensor can work on SPI master 1 only. */

    InterfaceAny    = 0xF0, /**< @brief The sensor can work on any sensor interface */
} SensorInterfaceType;
#define INTERFACE_MASK         (0xF0u)    /**< @brief The mask needed to check if an interface type is selected. */
#define INTERFACE_MASK_SHIFT   (0x04u)    /**< @brief The shift value need to access the interface type. */

typedef struct
{
    PTR(void*)          handle; /**< @brief The interface handle selected by the configuration */
    SensorInterfaceType type;   /**< @brief The selected interface type for the sensor. */
    SensorInterfaceType options;/**< @brief All possible interface types for this sensor driver. */
} SensorInterface;


#define IRQ_EDGE_FALLING    (0) /**< @brief The sensor driver should be called on the falling edge of the IRQ line. */
#define IRQ_EDGE_RISING     (1) /**< @brief The sensor driver should be called on the rising edge of the IRQ line. */
#define IRQ_EDGE_BOTH       (2) /**< @brief The sensor driver should be called on the both edges of the IRQ line. */

typedef struct Device {
    SensorInterface     interface;  /**< @brief Information needed to select the interface */

    struct {
        I2C_Device_t    i2c;    /**< @brief I2C Specific device information. */
        SPIM_Device_t   spi;    /**< @brief SPI Specific device information. */
    } options;

    UInt8 irqPin:5;             /**< @brief Select which GPIO pin should be monitored for interrupts. */
    UInt8 irqEdge:2;            /**< @brief Selects the rising (1) or falling (0) edge on the GPIO pin to cause an interrupt. */
    UInt8 irqDis:1;             /**< @brief No GPIO pins are used for this sensor when (1). */
} Device;

#if defined(_ARCVER)

/** @cond */
extern I2CState I2CM1_state;
extern I2CState I2CM0_state;
extern SPIM_Handle_t SPIM0_handle;
extern SPIM_Handle_t SPIM1_handle;
/** @endcond */

/**
 * @fn SensorStatus write_data(const Device* device,
 *                              UInt8 reg,
 *                              UInt8 *buffer,
 *                              UInt8 bytes);
 *
 * @brief Writes data out the bus.
 *
 * This is a blocking operation.
 *
 * @deprecated                  The @ref write_data_nonblocking function should be used instead.
 *
 * @param device                    The device to communicate with.
 * @param reg                       The register to begin the write operation at.
 * @param[in] buffer                The data array to send to the device.
 * @param bytes                     The number of bytes in the data array.
 *
 * @retval SensorOK                 The communication completed successfully.
 * @retval SensorErrorNonExistant   The sensor did not respond.
 */
//lint -function( fopen(1), write_data(1) ) // arg cannot be null
//lint -function( fopen(1), write_data(3) ) // arg cannot be null
//lint -sem( write_data, (3P >= 4n) ) // bytes must be <= buffer size
SensorStatus JLI write_data(const Device* device,
                            UInt8 reg,
                            const UInt8 *buffer,
                            UInt8 bytes) DEPRECATED("The write_data_nonblocking function should be used instead.");

/**
 * @fn SensorStatus read_data(const Device* device,
 *              UInt8 reg,
 *              UInt8 *buffer,
 *              UInt8 bytes);
 *
 * @brief Performs a read operation on the selected device.
 *
 * Performs and read starting at the specified register.
 * Results are returned are saved in the buffer specified.
 * This is a blocking operation.
 *
 * @deprecated                      The @ref read_data_nonblocking function should be used instead.
 *
 * @param device                    The device to communicate with.
 * @param reg                       The register to begin the read operation at.
 * @param[out] buffer               The memory location to write the results to.
 * @param bytes                     The number of bytes to read in.
 *
 * @retval SensorOK                 The communication completed successfully.
 * @retval SensorErrorNonExistant   The sensor did not respond.
 */
//lint -function( fopen(1), read_data(1) ) // arg cannot be null
//lint -function( fopen(1), read_data(3) ) // arg cannot be null
//lint -sem( read_data, (3P >= 4n) ) // bytes must be <= buffer size
SensorStatus JLI read_data(const Device* device,
                           UInt8 reg,
                           UInt8 *buffer,
                           UInt8 bytes) DEPRECATED("The read_data_nonblocking function should be used instead.");

/**
 * @fn void write_data_nonblocking(const Device* device,
 *              UInt8 reg,
 *              UInt8 *buffer,
 *              UInt8 bytes,
 *              void (*callback_fn)(SensorStatus, void*),
 *              void* data);
 *
 *
 * @brief Performs a write operation on the selected device.
 *
 * This is a non-blocking operation, the input buffer cannot be modified until the
 * transaction is complete. Note: If the buffer is on the stack, you must ensure that
 * the transaction completes before the function exits.
 *
 * @param device                    The device to communicate with.
 * @param reg                       The register to write to.
 * @param[in] buffer                An array of data to write to the device.
 * @param bytes                     The number of entries in the source array.
 * @param[in] callback_fn           The function to call after the transaction is complete, the return status is passed as an argument.
 * @param[in] data                  The data to pass to the callback function.
 */
//lint -function( fopen(1), write_data_nonblocking(1) ) // arg cannot be null
//lint -function( fopen(1), write_data_nonblocking(3) ) // arg cannot be null
//lint -sem( write_data_nonblocking, (3P >= 4n) ) // bytes must be <= buffer size
//lint -sem( write_data_nonblocking, thread_create(5))
void JLI write_data_nonblocking(const Device* device,
                UInt8 reg,
                const UInt8 *buffer,
                UInt8 bytes,
                void (*callback_fn)(SensorStatus, void*),
                void* data);

/**
 * @fn void write_data_inline_nonblocking(const Device* device,
 *              UInt8 reg,
 *              UInt8 *buffer,
 *              UInt8 bytes,
 *              void (*callback_fn)(SensorStatus, void*),
 *              void* data);
 *
 *
 * @brief Performs a write operation on the selected device with up to 4 bytes.
 *
 * Unlike the @ref write_data_nonblocking routine, this is safe to use with a
 * buffer that is not persistent - stack buffers are OK to use here.
 *
 * This is a non-blocking operation.
 *
 * @param device                    The device to communicate with.
 * @param reg                       The register to write to.
 * @param[in] buffer                An array of data to write to the device.
 * @param bytes                     The number of entries in the source array.
 * @param[in] callback_fn           The function to call after the transaction is complete, the return status is passed as an argument.
 * @param[in] data                  The data to pass to the callback function.
 */
//lint -function( fopen(1), write_data_inline_nonblocking(1) ) // arg cannot be null
//lint -function( fopen(1), write_data_inline_nonblocking(3) ) // arg cannot be null
//lint -sem( write_data_inline_nonblocking, 4n <= 4) // bytes must be 4 bytes or less
//lint -sem( write_data_inline_nonblocking, (3P >= 4n) ) // bytes must be <= buffer size
//lint -sem( write_data_inline_nonblocking, thread_create(5))
void JLI write_data_inline_nonblocking(const Device* device,
                UInt8 reg,
                const UInt8 *buffer,
                UInt8 bytes,
                void (*callback_fn)(SensorStatus, void*),
                void* data);


/**
 * @fn void write_data_slow_nonblocking(const Device* device,
 *              UInt8 reg,
 *              UInt8 *buffer,
 *              UInt8 bytes,
 *              void (*callback_fn)(SensorStatus, void*),
 *              void* data,
 *              float delay);
 *
 *
 * @brief Performs a write operation on the selected device.
 *
 * This is a non-blocking operation, the input buffer cannot be modified until the
 * transaction is complete. Note: If the buffer is on the stack, you must ensure that
 * the transaction completes before the function exits.
 *
 * @param device                    The device to communicate with.
 * @param reg                       The register to write to.
 * @param[in] buffer                An array of data to write to the device.
 * @param bytes                     The number of entries in the source array.
 * @param[in] callback_fn           The function to call after the transaction is complete, the return status is passed as an argument.
 * @param[in] data                  The data to pass to the callback function.
 * @param delay                     The delay in ms between bytes in the transaction, and the start of the next transaction.
 */
//lint -function( fopen(1), write_data_slow_nonblocking(1) ) // arg cannot be null
//lint -function( fopen(1), write_data_slow_nonblocking(3) ) // arg cannot be null
//lint -sem( write_data_slow_nonblocking, 3P >= 4n ) // bytes must be <= buffer size
//lint -sem( write_data_slow_nonblocking, thread_create(5))
void JLI write_data_slow_nonblocking(const Device* device,
                                UInt8 reg,
                                const UInt8 *buffer,
                                UInt8 bytes,
                                void (*callback_fn)(SensorStatus, void*),
                                void* data,
                                float delay);

/**
 * @fn void write_data_slow_inline_nonblocking(const Device* device,
 *              UInt8 reg,
 *              UInt8 *buffer,
 *              UInt8 bytes,
 *              void (*callback_fn)(SensorStatus, void*),
 *              void* data,
 *              float delay);
 *
 *
 * @brief Performs a write operation on the selected device with up to 4 bytes.
 *
 * Unlike the @ref write_data_slow_nonblocking routine, this is safe to use with
 * a buffer that is not persistent - stack buffers are OK to use here.
 *
 * This is a non-blocking operation.
 *
 * @param device                    The device to communicate with.
 * @param reg                       The register to write to.
 * @param[in] buffer                An array of data to write to the device.
 * @param bytes                     The number of entries in the source array.
 * @param[in] callback_fn           The function to call after the transaction is complete, the return status is passed as an argument.
 * @param[in] data                  The data to pass to the callback function.
 * @param delay                     The delay in ms between bytes in the transaction, and the start of the next transaction.
 */
//lint -function( fopen(1), write_data_slow_inline_nonblocking(1) ) // arg cannot be null
//lint -function( fopen(1), write_data_slow_inline_nonblocking(3) ) // arg cannot be null
//lint -sem( write_data_slow_inline_nonblocking, 4n <= 4) // bytes must be 4 bytes or less
//lint -sem( write_data_slow_inline_nonblocking, 3P >= 4n ) // bytes must be <= buffer size
//lint -sem( write_data_slow_inline_nonblocking, thread_create(5))
void JLI write_data_slow_inline_nonblocking(const Device* device,
                                UInt8 reg,
                                const UInt8 *buffer,
                                UInt8 bytes,
                                void (*callback_fn)(SensorStatus, void*),
                                void* data,
                                float delay);
/**
 * @fn void read_data_nonblocking(const Device* device,
 *              UInt8 reg,
 *              UInt8 *buffer,
 *              UInt8 bytes,
 *              void (*callback_fn)(SensorStatus, void*),
 *              void* data);
 *
 * @brief Performs one or more read operations on the selected device.
 *
 * This is a non-blocking operation. The input buffer will be overwritten once the transaction has completed.
 *
 * @param device                The device to communicate with.
 * @param reg                   The register to read from.
 * @param[out] buffer           The memory location to save the read data.
 * @param bytes                 The number of bytes to read.
 * @param[in] callback_fn       The function to call after the transaction is complete, the return status is passed as an argument.
 * @param[in] data              The data to pass to the callback function.
 */
//lint -function( fopen(1), read_data_nonblocking(1) ) // arg cannot be null
//lint -function( fopen(1), read_data_nonblocking(3) ) // arg cannot be null
//lint -sem( read_data_nonblocking, (3P >= 4n) ) // bytes must be <= buffer size
//lint -sem( read_data_nonblocking, thread_create(5))
void JLI read_data_nonblocking(const Device* device,
                               UInt8 reg,
                               UInt8 *buffer,
                               UInt8 bytes,
                               void (*callback_fn)(SensorStatus, void*),
                               void* data);

/**
 * @fn void read_data_slow_nonblocking(const Device* device,
 *              UInt8 reg,
 *              UInt8 *buffer,
 *              UInt8 bytes,
 *              void (*callback_fn)(SensorStatus, void*),
 *              void* data,
 *              float delay);
 *
 * @brief Performs one or more read operations on the selected device.
 *
 * This is a non-blocking operation. The input buffer will be overwritten once the transaction has completed.
 *
 * @param device                The device to communicate with.
 * @param reg                   The register to read from.
 * @param[out] buffer           The memory location to save the read data.
 * @param bytes                 The number of bytes to read.
 * @param[in] callback_fn       The function to call after the transaction is complete, the return status is passed as an argument.
 * @param[in] data              The data to pass to the callback function.
 * @param delay                 The delay in ms between bytes in the transaction, and the start of the next transaction.
 */
//lint -function( fopen(1), read_data_slow_nonblocking(1) ) // arg cannot be null
//lint -function( fopen(1), read_data_slow_nonblocking(3) ) // arg cannot be null
//lint -sem( read_data_slow_nonblocking, (3P >= 4n) ) // bytes must be <= buffer size
//lint -sem( read_data_slow_nonblocking, thread_create(5))
void JLI read_data_slow_nonblocking(const Device* device,
                                UInt8 reg,
                                UInt8 *buffer,
                                UInt8 bytes,
                                void (*callback_fn)(SensorStatus, void*),
                                void* data,
                                float delay);

/**
 * @fn void read_bytes_nonblocking(const Device* device,
 *              UInt8 *buffer,
 *              UInt8 bytes,
 *              void (*callback_fn)(SensorStatus, void*),
 *              void* data);
 *
 * @brief Performs one or more read operations on the selected device.
 *
 * This is a non-blocking operation. The input buffer will be overwritten once the transaction has completed.
 *
 * @param device                The device to communicate with.
 * @param[out] buffer           The memory location to save the read data.
 * @param bytes                 The number of bytes to read.
 * @param[in] callback_fn       The function to call after the transaction is complete, the return status is passed as an argument.
 * @param[in] data              The data to pass to the callback function.
 */
//lint -function( fopen(1), read_bytes_nonblocking(1) ) // arg cannot be null
//lint -function( fopen(1), read_bytes_nonblocking(2) ) // arg cannot be null
//lint -sem( read_bytes_nonblocking, (2P >= 3n) ) // bytes must be <= buffer size
//lint -sem( read_bytes_nonblocking, thread_create(4))
void JLI read_bytes_nonblocking(const Device* device,
                                UInt8 *buffer,
                                UInt8 bytes,
                                void (*callback_fn)(SensorStatus, void*),
                                void* data);


/**
 * @fn void read_bytes_slow_nonblocking(const Device* device,
 *              UInt8 *buffer,
 *              UInt8 bytes,
 *              void (*callback_fn)(SensorStatus, void*),
 *              void* data,
 *              float delay);
 *
 * @brief Performs one or more read operations on the selected device.
 *
 * This is a non-blocking operation. The input buffer will be overwritten once the transaction has completed.
 *
 * @param device                The device to communicate with.
 * @param[out] buffer           The memory location to save the read data.
 * @param bytes                 The number of bytes to read.
 * @param[in] callback_fn       The function to call after the transaction is complete, the return status is passed as an argument.
 * @param[in] data              The data to pass to the callback function.
 * @param delay                 The delay in ms between bytes in the transaction, and the start of the next transaction.
 */
//lint -function( fopen(1), read_bytes_slow_nonblocking(1) ) // arg cannot be null
//lint -function( fopen(1), read_bytes_slow_nonblocking(2) ) // arg cannot be null
//lint -sem( read_bytes_slow_nonblocking, (2P >= 3n) ) // bytes must be <= buffer size
//lint -sem( read_bytes_slow_nonblocking, thread_create(4))
void NOJLI read_bytes_slow_nonblocking(const Device* device,
                                UInt8 *buffer,
                                UInt8 bytes,
                                void (*callback_fn)(SensorStatus, void*),
                                void* data,
                                float delay);

/**
 * @fn void write_bytes_nonblocking(const Device* device,
 *              UInt8 *buffer,
 *              UInt8 bytes,
 *              void (*callback_fn)(SensorStatus, void*),
 *              void* data);
 *
 * This is a non-blocking operation, the input buffer cannot be modified until the
 * transaction is complete. Note: If the buffer is on the stack, you must ensure that
 * the transaction completes before the function exits.
 *
 * This is a non-blocking operation.
 *
 * @param device                The device to communicate with.
 * @param[in] buffer            An array of data to write to the device.
 * @param bytes                 The number of entries in the source array.
 * @param[in] callback_fn       The function to call after the transaction is complete, the return status is passed as an argument.
 * @param[in] data              The data to pass to the callback function.
 */
//lint -function( fopen(1), write_bytes_nonblocking(1) ) // arg cannot be null
//lint -function( fopen(1), write_bytes_nonblocking(2) ) // arg cannot be null
//lint -sem( write_bytes_nonblocking, (2P >= 3n) ) // bytes must be <= buffer size
//lint -sem( write_bytes_nonblocking, thread_create(4))
void JLI write_bytes_nonblocking(const Device* device,
                                const UInt8 *buffer,
                                UInt8 bytes,
                                void (*callback_fn)(SensorStatus, void*),
                                void* data);

/**
 * @fn void write_bytes_inline_nonblocking(const Device* device,
 *              const UInt8 *buffer,
 *              UInt8 bytes,
 *              void (*callback_fn)(SensorStatus, void*),
 *              void* data);
 *
 * @brief Performs one or more write operations on the selected device.
 *
 * Unlike the @ref write_bytes_nonblocking routine, this is safe to use with
 * a buffer that is not persistent - stack buffers are OK to use here.
 *
 * This is a non-blocking operation.
 *
 * @param device                The device to communicate with.
 * @param[in] buffer            An array of data to write to the device.
 * @param bytes                 The number of entries in the source array.
 * @param[in] callback_fn       The function to call after the transaction is complete, the return status is passed as an argument.
 * @param[in] data              The data to pass to the callback function.
 */
//lint -function( fopen(1), write_bytes_inline_nonblocking(1) ) // arg cannot be null
//lint -function( fopen(1), write_bytes_inline_nonblocking(2) ) // arg cannot be null
//lint -sem( write_bytes_inline_nonblocking, 3n <= 4) // bytes must be 4 bytes or less
//lint -sem( write_bytes_inline_nonblocking, (2P >= 3n) ) // bytes must be <= buffer size
//lint -sem( write_bytes_inline_nonblocking, thread_create(4))
void JLI write_bytes_inline_nonblocking(const Device* device,
                                const UInt8 *buffer,
                                UInt8 bytes,
                                void (*callback_fn)(SensorStatus, void*),
                                void* data);


/**
 * @fn void write_bytes_slow_nonblocking(const Device* device,
 *              const UInt8 *buffer,
 *              UInt8 bytes,
 *              void (*callback_fn)(SensorStatus, void*),
 *              void* data,
 *              float delay);
 *
 * @brief Performs one or more write operations on the selected device.
 *
 * This is a non-blocking operation, the input buffer cannot be modified until the
 * transaction is complete. Note: If the buffer is on the stack, you must ensure that
 * the transaction completes before the function exits.
 *
 * This is a non-blocking operation.
 *
 * @param device                The device to communicate with.
 * @param[in] buffer            An array of data to write to the device.
 * @param bytes                 The number of entries in the source array.
 * @param[in] callback_fn       The function to call after the transaction is complete, the return status is passed as an argument.
 * @param[in] data              The data to pass to the callback function.
 * @param delay                 The delay in ms between bytes in the transaction, and the start of the next transaction.
*/
//lint -function( fopen(1), write_bytes_slow_nonblocking(1) ) // arg cannot be null
//lint -function( fopen(1), write_bytes_slow_nonblocking(2) ) // arg cannot be null
//lint -sem( write_bytes_slow_nonblocking, (2P >= 3n) ) // bytes must be <= buffer size
//lint -sem( write_bytes_slow_nonblocking, thread_create(4))
void NOJLI write_bytes_slow_nonblocking(const Device* device,
                                const UInt8 *buffer,
                                UInt8 bytes,
                                void (*callback_fn)(SensorStatus, void*),
                                void* data,
                                float delay);

/**
 * @fn void write_bytes_slow_inline_nonblocking(const Device* device,
 *              const UInt8 *buffer,
 *              UInt8 bytes,
 *              void (*callback_fn)(SensorStatus, void*),
 *              void* data,
 *              float delay);
 *
 * @brief Performs one or more write operations on the selected device.
 *
 * Unlike the @ref write_bytes_slow_nonblocking routine, this is safe to use with
 * a buffer that is not persistent - stack buffers are OK to use here.
 *
 * This is a non-blocking operation.
 *
 * @param device                The device to communicate with.
 * @param[in] buffer            An array of data to write to the device.
 * @param bytes                 The number of entries in the source array.
 * @param[in] callback_fn       The function to call after the transaction is complete, the return status is passed as an argument.
 * @param[in] data              The data to pass to the callback function.
 * @param delay                 The delay in ms between bytes in the transaction, and the start of the next transaction.
*/
//lint -function( fopen(1), write_bytes_slow_inline_nonblocking(1) ) // arg cannot be null
//lint -function( fopen(1), write_bytes_slow_inline_nonblocking(2) ) // arg cannot be null
//lint -sem( write_bytes_slow_inline_nonblocking, 3n <= 4) // bytes must be 4 bytes or less
//lint -sem( write_bytes_slow_inline_nonblocking, (2P >= 3n) ) // bytes must be <= buffer size
//lint -sem( write_bytes_slow_inline_nonblocking, thread_create(4))
void NOJLI write_bytes_slow_inline_nonblocking(const Device* device,
                                const UInt8 *buffer,
                                UInt8 bytes,
                                void (*callback_fn)(SensorStatus, void*),
                                void* data,
                                float delay);


/**
 * @fn bool JLI drain_queue(const Device* device);
 *
 * @brief Waits for all currently scheduled transactions to run.
 *
 *  Do note that this function will block other code from execution.
 *
 * @retval  false       No transaction was pending.
 * @retval  true        The master finished the transaction.
 */
bool JLI drain_queue(const Device* device);

void patchSPIMDevice(Device *dev);

#endif /* _ARCVER */

#endif /* !SENSOR_BUS_H */

/** @} */
