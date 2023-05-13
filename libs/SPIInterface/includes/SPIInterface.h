////////////////////////////////////////////////////////////////////////////////
///
/// @file       libs/SPIInterface/includes/SPIInterface.h
///
/// @project    EM7189
///
/// @brief      SPI master driver.
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

#ifndef SPI_INTERFACE_H
#define SPI_INTERFACE_H

#include <types.h>

#include "driver.h"

#define SPI_TRANSACTION_SIZE    (32u)

typedef struct SPIM_Handle_t SPIM_Handle_t;

#ifdef _ARCVER
//lint -function( fopen(1), spi_init(1) ) // arg cannot be null
//lint -function( fopen(1), spi_init(3) ) // arg cannot be null
void spi_init(void* buffer, UInt8 depth, SPIM_Handle_t* pHandle) asm("SPIM_Init");

/**
 * @brief Performs one or more read operations.
 *
 * This is a non-blocking operation. The input buffer will be overwritten once
 * the transaction has completed.
 *
 * @param[in]  device           The device descriptor for the target.
 * @param[in]  reg              The register to address on the device.
 * @param[out] pReadBuffer      The memory location to save the read data.
 * @param      bytes            The number of bytes to read.
 * @param[in]  callbackFunction The function to call after the transaction is
 *                                  complete, the return status is passed as an
 *                                  argument.
 * @param[in]  pUserData        The data to pass to the callback function.
 * @param      pHandle          The sensor interface handle.
 * @returns true if the transaction was scheduled, false otherwise.
 */
//lint -function( fopen(1), spi_read_data(1) ) // arg cannot be null
//lint -function( fopen(1), spi_read_data(3) ) // arg cannot be null
//lint -function( fopen(1), spi_read_data(7) ) // arg cannot be null
//lint -sem( spi_read_data, (3P >= 4n) ) // bytes must be <= buffer size
//lint -sem( spi_read_data, thread_create(5))
bool JLI spi_read_data(const SPIM_Device_t* device, UInt8 reg, UInt8 *pReadBuffer, UInt8 bytes,
    Driver_Callback_t callbackFunction, void *pUserData, SPIM_Handle_t* pHandle) asm("SPIM_CmdReadBytes");

/**
 * @brief Performs one or more read operations.
 *
 * This is a non-blocking operation. The input buffer will be overwritten once
 * the transaction has completed.
 *
 * @param[in]  device           The device descriptor for the target.
 * @param[in]  reg              The register to address on the device.
 * @param[out] pReadBuffer      The memory location to save the read data.
 * @param      bytes            The number of bytes to read.
 * @param[in]  callbackFunction The function to call after the transaction is
 *                                  complete, the return status is passed as an
 *                                  argument.
 * @param[in]  pUserData The data to pass to the callback function.
 * @param      delay            The delay in ms between the transaction and the
 *                                  start of the next transaction.
 * @param pHandle The sensor interface handle.
 * @returns true if the transaction was scheduled, false otherwise.
 */
//lint -function( fopen(1), spi_read_data_slow(1) ) // arg cannot be null
//lint -function( fopen(1), spi_read_data_slow(3) ) // arg cannot be null
//lint -function( fopen(1), spi_read_data_slow(8) ) // arg cannot be null
//lint -sem( spi_read_data_slow, (3P >= 4n) ) // bytes must be <= buffer size
//lint -sem( spi_read_data_slow, thread_create(5))
bool JLI spi_read_data_slow(const SPIM_Device_t* device, UInt8 reg, UInt8 *pReadBuffer,
    UInt8 bytes, Driver_Callback_t callbackFunction, void *pUserData,
    float delay, SPIM_Handle_t* pHandle) asm("SPIM_CmdReadBytesWithDelay");

/**
 * @brief Performs one or more read operations.
 *
 * This is a non-blocking operation. The input buffer will be overwritten once
 * the transaction has completed.
 *
 * @param[in]  device           The device descriptor for the target.
 * @param[in]  reg              The register to address on the device.
 * @param[out] pReadBuffer      The memory location to save the read data.
 * @param      bytes            The number of bytes to read.
 * @param[in]  callbackFunction The function to call after the transaction is
 *                                  complete, the return status is passed as an
 *                                  argument.
 * @param[in]  pUserData The data to pass to the callback function.
 * @param      delay            The delay in ms between each
 *                              byte read.
 * @param pHandle The sensor interface handle.
 * @returns true if the transaction was scheduled, false otherwise.
 */
//lint -function( fopen(1), spi_read_single_data_slow(1) ) // arg cannot be null
//lint -function( fopen(1), spi_read_single_data_slow(3) ) // arg cannot be null
//lint -function( fopen(1), spi_read_single_data_slow(8) ) // arg cannot be null
//lint -sem( spi_read_single_data_slow, (3P >= 4n) ) // bytes must be <= buffer size
//lint -sem( spi_read_single_data_slow, thread_create(5))
bool JLI spi_read_single_data_slow(const SPIM_Device_t* device, UInt8 reg, UInt8 *pReadBuffer,
    UInt8 bytes, Driver_Callback_t callbackFunction, void *pUserData,
    float delay, SPIM_Handle_t* pHandle) asm("SPIM_CmdReadBytesSlow");

/**
 * @brief Performs one or more write operations.
 *
 * This is a non-blocking operation, the input buffer cannot be modified until
 * the transaction is complete.
 *
 * @note If the buffer is on the stack, you must ensure that the transaction
 *   completes before the function exits.
 *
 * @param[in]  device           The device descriptor for the target.
 * @param[in]  reg              The register to address on the device.
 * @param[in]  pWriteBuffer     An array of data to write to the device.
 * @param      bytes            The number of entries in the source array.
 * @param[in]  callbackFunction The function to call after the transaction is
 *                                  complete, the return status is passed as an
 *                                  argument.
 * @param[in]  pUserData        The data to pass to the callback function.
 * @param      pHandle          The sensor interface handle.
 * @returns true if the transaction was scheduled, false otherwise.
 */
//lint -function( fopen(1), spi_write_data(1) ) // arg cannot be null
//lint -function( fopen(1), spi_write_data(3) ) // arg cannot be null
//lint -function( fopen(1), spi_write_data(7) ) // arg cannot be null
//lint -sem( spi_write_data, (3P >= 4n) ) // bytes must be <= buffer size
//lint -sem( spi_write_data, thread_create(5))
bool JLI spi_write_data(const SPIM_Device_t* device, UInt8 reg, const UInt8 *pWriteBuffer,
    UInt8 bytes, Driver_Callback_t callbackFunction, void *pUserData, SPIM_Handle_t* pHandle)
    asm("SPIM_CmdWriteBytes");

/**
 * @brief Performs one or more write operations.
 *
 * This is a non-blocking operation, the input buffer cannot be larger than 4 bytes.
 *
 * @param[in]  device           The device descriptor for the target.
 * @param[in]  reg              The register to address on the device.
 * @param[in]  pWriteBuffer     An array of data to write to the device.
 * @param      bytes            The number of entries in the source array.
 * @param[in]  callbackFunction The function to call after the transaction is
 *                                  complete, the return status is passed as an
 *                                  argument.
 * @param[in]  pUserData        The data to pass to the callback function.
 * @param      pHandle          The sensor interface handle.
 * @returns true if the transaction was scheduled, false otherwise.
 */
//lint -function( fopen(1), spi_write_data_inline(1) ) // arg cannot be null
//lint -function( fopen(1), spi_write_data_inline(3) ) // arg cannot be null
//lint -function( fopen(1), spi_write_data_inline(7) ) // arg cannot be null
//lint -sem( spi_write_data_inline, 4n <= 4) // bytes must be 4 bytes or less
//lint -sem( spi_write_data_inline, (3P >= 4n) ) // bytes must be <= buffer size
//lint -sem( spi_write_data_inline, thread_create(5))
bool JLI spi_write_data_inline(const SPIM_Device_t* device, UInt8 reg, const UInt8 *pWriteBuffer,
    UInt8 bytes, Driver_Callback_t callbackFunction, void *pUserData, SPIM_Handle_t* pHandle)
    asm("SPIM_CmdWriteBytesInline");

 /**
 * @brief Performs one or more write operations.
 *
 * This is a non-blocking operation, the input buffer cannot be modified until
 * the transaction is complete.
 *
 * @note If the buffer is on the stack, you must ensure that the transaction
 *   completes before the function exits.
 *
 * @param[in]  device           The device descriptor for the target.
 * @param[in]  reg              The register to address on the device.
 * @param[in]  pWriteBuffer     An array of data to write to the device.
 * @param      bytes            The number of entries in the source array.
 * @param[in]  callbackFunction The function to call after the transaction is
 *                                  complete, the return status is passed as an
 *                                  argument.
 * @param[in]  pUserData        The data to pass to the callback function.
 * @param      delay            The delay in ms between the transaction and the
 *                                  start of the next transaction.
 * @param      pHandle          The sensor interface handle.
 * @returns true if the transaction was scheduled, false otherwise.
 */
//lint -function( fopen(1), spi_write_data_slow(1) ) // arg cannot be null
//lint -function( fopen(1), spi_write_data_slow(3) ) // arg cannot be null
//lint -function( fopen(1), spi_write_data_slow(8) ) // arg cannot be null
//lint -sem( spi_write_data_slow, (3P >= 4n) ) // bytes must be <= buffer size
//lint -sem( spi_write_data_slow, thread_create(5))
bool JLI spi_write_data_slow(const SPIM_Device_t* device, UInt8 reg,
    const UInt8 *pWriteBuffer, UInt8 bytes,
    Driver_Callback_t callbackFunction, void *pUserData,
    float delay, SPIM_Handle_t* pHandle) asm("SPIM_CmdWriteBytesWithDelay");


 /**
 * @brief Performs one or more write operations.
 *
 * This is a non-blocking operation, the input buffer cannot larger than 4 bytes.
 *
 * @param[in]  device           The device descriptor for the target.
 * @param[in]  reg              The register to address on the device.
 * @param[in]  pWriteBuffer     An array of data to write to the device.
 * @param      bytes            The number of entries in the source array.
 * @param[in]  callbackFunction The function to call after the transaction is
 *                                  complete, the return status is passed as an
 *                                  argument.
 * @param[in]  pUserData        The data to pass to the callback function.
 * @param      delay            The delay in ms between the transaction and the
 *                                  start of the next transaction.
 * @param      pHandle          The sensor interface handle.
 * @returns true if the transaction was scheduled, false otherwise.
 */
//lint -function( fopen(1), spi_write_data_slow_inline(1) ) // arg cannot be null
//lint -function( fopen(1), spi_write_data_slow_inline(3) ) // arg cannot be null
//lint -function( fopen(1), spi_write_data_slow_inline(8) ) // arg cannot be null
//lint -sem( spi_write_data_slow_inline, 4n <= 4) // bytes must be 4 bytes or less
//lint -sem( spi_write_data_slow_inline, (3P >= 4n) ) // bytes must be <= buffer size
//lint -sem( spi_write_data_slow_inline, thread_create(5))
bool JLI spi_write_data_slow_inline(const SPIM_Device_t* device, UInt8 reg,
    const UInt8 *pWriteBuffer, UInt8 bytes,
    Driver_Callback_t callbackFunction, void *pUserData,
    float delay, SPIM_Handle_t* pHandle) asm("SPIM_CmdWriteBytesWithDelayInline");



/**
 * @brief Performs one or more read operations.
 *
 * This is a non-blocking operation. The input buffer will be overwritten once
 * the transaction has completed.
 *
 * @param[in]  device           The device descriptor for the target.
 * @param[out] pReadBuffer      The memory location to save the read data.
 * @param      bytes            The number of bytes to read.
 * @param[in]  callbackFunction The function to call after the transaction is
 *                                  complete, the return status is passed as an
 *                                  argument.
 * @param[in]  pUserData        The data to pass to the callback function.
 * @param      pHandle          The sensor interface handle.
 * @returns true if the transaction was scheduled, false otherwise.
 */
//lint -function( fopen(1), spi_read_bytes(1) ) // arg cannot be null
//lint -function( fopen(1), spi_read_bytes(2) ) // arg cannot be null
//lint -function( fopen(1), spi_read_bytes(6) ) // arg cannot be null
//lint -sem( spi_read_bytes, (2P >= 3n) ) // bytes must be <= buffer size
//lint -sem( spi_read_bytes, thread_create(4))
bool JLI spi_read_bytes(const SPIM_Device_t* device, UInt8 *pReadBuffer, UInt8 bytes,
    Driver_Callback_t callbackFunction, void *pUserData, SPIM_Handle_t* pHandle) asm("SPIM_ReadBytes");

/**
 * @brief Performs one or more read operations.
 *
 * This is a non-blocking operation. The input buffer will be overwritten once
 * the transaction has completed.
 *
 * @param[in]  device           The descriptor for the target.
 * @param[out] pReadBuffer      The memory location to save the read data.
 * @param      bytes            The number of bytes to read.
 * @param[in]  callbackFunction The function to call after the transaction is
 *                                  complete, the return status is passed as an
 *                                  argument.
 * @param[in]  pUserData        The data to pass to the callback function.
 * @param      delay            The delay in ms between the transaction and the
 *                                  start of the next transaction.
 * @param      pHandle          The sensor interface handle.
 * @returns true if the transaction was scheduled, false otherwise.
 */
//lint -function( fopen(1), spi_read_bytes_slow(1) ) // arg cannot be null
//lint -function( fopen(1), spi_read_bytes_slow(2) ) // arg cannot be null
//lint -function( fopen(1), spi_read_bytes_slow(7) ) // arg cannot be null
//lint -sem( spi_read_bytes_slow, (2P >= 3n) ) // bytes must be <= buffer size
//lint -sem( spi_read_bytes_slow, thread_create(4))
bool JLI spi_read_bytes_slow(const SPIM_Device_t* device, UInt8 *pReadBuffer,
    UInt8 bytes, Driver_Callback_t callbackFunction, void *pUserData,
    float delay, SPIM_Handle_t* pHandle) asm("SPIM_ReadBytesWithDelay");

/**
 * @brief Performs one or more read operations.
 *
 * This is a non-blocking operation. The input buffer will be overwritten once
 * the transaction is complete.
 *
 * @param[in]  device           The descriptor for the target.
 * @param[out] pReadBuffer      The memory location to save the read data.
 * @param      bytes            The number of bytes to read.
 * @param[in]  callbackFunction The function to call after the transaction is
 *                                  complete, the return status is passed as an
 *                                  argument.
 * @param[in]  pUserData        The data to pass to the callback function.
 * @param      delay            The delay in ms between the transaction and the
 *                                  start of the next transaction.
 * @param      pHandle          The sensor interface handle.
 * @returns true if the transaction was scheduled, false otherwise.
 */
//lint -function( fopen(1), spi_read_single_bytes_slow(1) ) // arg cannot be null
//lint -function( fopen(1), spi_read_single_bytes_slow(2) ) // arg cannot be null
//lint -function( fopen(1), spi_read_single_bytes_slow(7) ) // arg cannot be null
//lint -sem( spi_read_single_bytes_slow, (2P >= 3n) ) // bytes must be <= buffer size
//lint -sem( spi_read_single_bytes_slow, thread_create(4))
bool JLI spi_read_single_bytes_slow(const SPIM_Device_t* device, UInt8 *pReadBuffer,
    UInt8 bytes, Driver_Callback_t callbackFunction, void *pUserData,
    float delay, SPIM_Handle_t* pHandle) asm("SPIM_ReadBytesSlow");

/**
 * @brief Performs one or more write operations.
 *
 * This is a non-blocking operation, the input buffer cannot be modified until
 * the transaction is complete.
 *
 * @note If the buffer is on the stack, you must ensure that the transaction
 *   completes before the function exits.
 *
 * @param[in]  device           The descriptor for the target.
 * @param[in]  pWriteBuffer     An array of data to write to the device.
 * @param      bytes            The number of entries in the source array.
 * @param[in]  callbackFunction The function to call after the transaction is
 *                                  complete, the return status is passed as an
 *                                  argument.
 * @param[in]  pUserData        The data to pass to the callback function.
 * @param      pHandle          The sensor interface handle.
 * @returns true if the transaction was scheduled, false otherwise.
 */
//lint -function( fopen(1), spi_write_bytes(1) ) // arg cannot be null
//lint -function( fopen(1), spi_write_bytes(2) ) // arg cannot be null
//lint -function( fopen(1), spi_write_bytes(6) ) // arg cannot be null
//lint -sem( spi_write_bytes, (2P >= 3n) ) // bytes must be <= buffer size
//lint -sem( spi_write_bytes, thread_create(4))
bool JLI spi_write_bytes(const SPIM_Device_t* device, const UInt8 *pWriteBuffer,
    UInt8 bytes, Driver_Callback_t callbackFunction, void *pUserData, SPIM_Handle_t* pHandle)
    asm("SPIM_WriteBytes");

/**
 * @brief Performs one or more write operations.
 *
 * This is a non-blocking operation, the input buffer cannot larger than 4 bytes.
 *
 * @param[in]  device           The descriptor for the target.
 * @param[in]  pWriteBuffer     An array of data to write to the device.
 * @param      bytes            The number of entries in the source array.
 * @param[in]  callbackFunction The function to call after the transaction is
 *                                  complete, the return status is passed as an
 *                                  argument.
 * @param[in]  pUserData        The data to pass to the callback function.
 * @param      pHandle          The sensor interface handle.
 * @returns true if the transaction was scheduled, false otherwise.
 */
//lint -function( fopen(1), spi_write_bytes_inline(1) ) // arg cannot be null
//lint -function( fopen(1), spi_write_bytes_inline(2) ) // arg cannot be null
//lint -function( fopen(1), spi_write_bytes_inline(6) ) // arg cannot be null
//lint -sem( spi_write_bytes_inline, 3n <= 4) // bytes must be 4 bytes or less
//lint -sem( spi_write_bytes_inline, (2P >= 3n) ) // bytes must be <= buffer size
//lint -sem( spi_write_bytes_inline, thread_create(4))
bool JLI spi_write_bytes_inline(const SPIM_Device_t* device, const UInt8 *pWriteBuffer,
    UInt8 bytes, Driver_Callback_t callbackFunction, void *pUserData, SPIM_Handle_t* pHandle)
    asm("SPIM_WriteBytesInline");

 /**
 * @brief Performs one or more write operations.
 *
 * This is a non-blocking operation, the input buffer cannot be modified until
 * the transaction is complete.
 *
 * @note If the buffer is on the stack, you must ensure that the transaction
 *   completes before the function exits.
 *
 * @param[in]  device           The descriptor for the target.
 * @param[in]  pWriteBuffer     An array of data to write to the device.
 * @param      bytes            The number of entries in the source array.
 * @param[in]  callbackFunction The function to call after the transaction is
 *                                  complete, the return status is passed as an
 *                                  argument.
 * @param[in]  pUserData        The data to pass to the callback function.
 * @param      delay            The delay in ms between the transaction and the
 *                                  start of the next transaction.
 * @param      pHandle          The sensor interface handle.
 * @returns true if the transaction was scheduled, false otherwise.
 */
//lint -function( fopen(1), spi_write_bytes_slow(1) ) // arg cannot be null
//lint -function( fopen(1), spi_write_bytes_slow(2) ) // arg cannot be null
//lint -function( fopen(1), spi_write_bytes_slow(7) ) // arg cannot be null
//lint -sem( spi_write_bytes_slow, (2P >= 3n) ) // bytes must be <= buffer size
//lint -sem( spi_write_bytes_slow, thread_create(4))
bool JLI spi_write_bytes_slow(const SPIM_Device_t* device,
    const UInt8 *pWriteBuffer, UInt8 bytes,
    Driver_Callback_t callbackFunction, void *pUserData,
    float delay, SPIM_Handle_t* pHandle) asm("SPIM_WriteBytesWithDelay");

 /**
 * @brief Performs one or more write operations.
 *
 * @param[in]  device           The descriptor for the target.
 * @param[in]  pWriteBuffer     An array of data to write to the device.
 * @param      bytes            The number of entries in the source array.
 * @param[in]  callbackFunction The function to call after the transaction is
 *                                  complete, the return status is passed as an
 *                                  argument.
 * @param[in]  pUserData        The data to pass to the callback function.
 * @param      delay            The delay in ms between the transaction and the
 *                                  start of the next transaction.
 * @param      pHandle          The sensor interface handle.
 * @returns true if the transaction was scheduled, false otherwise.
 */
//lint -function( fopen(1), spi_write_bytes_slow_inline(1) ) // arg cannot be null
//lint -function( fopen(1), spi_write_bytes_slow_inline(2) ) // arg cannot be null
//lint -function( fopen(1), spi_write_bytes_slow_inline(7) ) // arg cannot be null
//lint -sem( spi_write_bytes_slow_inline, (2P >= 3n) ) // bytes must be <= buffer size
//lint -sem( spi_write_bytes_slow_inline, 3n <= 4) // bytes must be 4 bytes or less
//lint -sem( spi_write_bytes_slow_inline, thread_create(4))
bool JLI spi_write_bytes_slow_inline(const SPIM_Device_t* device,
    const UInt8 *pWriteBuffer, UInt8 bytes,
    Driver_Callback_t callbackFunction, void *pUserData,
    float delay, SPIM_Handle_t* pHandle) asm("SPIM_WriteBytesWithDelayInline");

/**
 * @brief Performs one or more write operations.
 *
 * This is a non-blocking operation, the input buffer cannot be modified until
 * the transaction is complete.
 *
 * @note If the buffer is on the stack, you must ensure that the transaction
 *   completes before the function exits.
 *
 * @param[in]  device           The descriptor for the target.
 * @param[in]  pWriteBuffer     An array of data to write to the device.
 * @param      bytes            The number of entries in the source array.
 * @param[in]  callbackFunction The function to call after the transaction is
 *                                  complete, the return status is passed as an
 *                                  argument.
 * @param[in]  pUserData        The data to pass to the callback function.
 * @param      delay            The delay in ms between the transaction and the
 *                                  start of the next transaction.
 * @param      pHandle          The sensor interface handle.
 * @returns true if the transaction was scheduled, false otherwise.
 */
//lint -function( fopen(1), spi_write_single_bytes_slow(1) ) // arg cannot be null
//lint -function( fopen(1), spi_write_single_bytes_slow(2) ) // arg cannot be null
//lint -function( fopen(1), spi_write_single_bytes_slow(7) ) // arg cannot be null
//lint -sem( spi_write_single_bytes_slow, (2P >= 3n) ) // bytes must be <= buffer size
//lint -sem( spi_write_single_bytes_slow, thread_create(4))
bool JLI spi_write_single_bytes_slow(const SPIM_Device_t* device, const UInt8 *pWriteBuffer,
    UInt8 bytes, Driver_Callback_t callbackFunction, void *pUserData,
    float delay, SPIM_Handle_t* pHandle) asm("SPIM_WriteBytesSlow");

/**
 * @brief Performs one or more write operations.
 *
 * @param[in]  device           The descriptor for the target.
 * @param[in]  pWriteBuffer     An array of data to write to the device.
 * @param      bytes            The number of entries in the source array.
 * @param[in]  callbackFunction The function to call after the transaction is
 *                                  complete, the return status is passed as an
 *                                  argument.
 * @param[in]  pUserData        The data to pass to the callback function.
 * @param      delay            The delay in ms between the transaction and the
 *                                  start of the next transaction.
 * @param      pHandle          The sensor interface handle.
 * @returns true if the transaction was scheduled, false otherwise.
 */
//lint -function( fopen(1), spi_write_single_bytes_slow_inline(1) ) // arg cannot be null
//lint -function( fopen(1), spi_write_single_bytes_slow_inline(2) ) // arg cannot be null
//lint -function( fopen(1), spi_write_single_bytes_slow_inline(7) ) // arg cannot be null
//lint -sem( spi_write_single_bytes_slow_inline, (2P >= 3n) ) // bytes must be <= buffer size
//lint -sem( spi_write_single_bytes_slow_inline, thread_create(4))
bool JLI spi_write_single_bytes_slow_inline(const SPIM_Device_t* device, const UInt8 *pWriteBuffer,
    UInt8 bytes, Driver_Callback_t callbackFunction, void *pUserData,
    float delay, SPIM_Handle_t* pHandle) asm("SPIM_WriteBytesSlowInline");

/**
 * @brief Performs one or more write operations.
 *
 * This is a non-blocking operation, the input buffer cannot be modified until
 * the transaction is complete.
 *
 * @note If the buffer is on the stack, you must ensure that the transaction
 *   completes before the function exits.
 *
 * @param[in]  device           The descriptor for the target.
 * @param[in]  reg              The register to address on the device.
 * @param[in]  pWriteBuffer     An array of data to write to the device.
 * @param      bytes            The number of entries in the source array.
 * @param[in]  callbackFunction The function to call after the transaction is
 *                                  complete, the return status is passed as an
 *                                  argument.
 * @param[in]  pUserData        The data to pass to the callback function.
 * @param      delay            The delay in ms between each
 *                              byte write.
 * @param      pHandle          The sensor interface handle.
 * @returns true if the transaction was scheduled, false otherwise.
 */
//lint -function( fopen(1), spi_write_single_data_slow(1) ) // arg cannot be null
//lint -function( fopen(1), spi_write_single_data_slow(3) ) // arg cannot be null
//lint -function( fopen(1), spi_write_single_data_slow(8) ) // arg cannot be null
//lint -sem( spi_write_single_data_slow, (3P >= 4n) ) // bytes must be <= buffer size
//lint -sem( spi_write_single_data_slow, thread_create(5))
bool JLI spi_write_single_data_slow(const SPIM_Device_t* device, UInt8 reg,
    const UInt8 *pWriteBuffer, UInt8 bytes,
    Driver_Callback_t callbackFunction, void *pUserData,
    float delay, SPIM_Handle_t* pHandle) asm("SPIM_CmdWriteBytesSlow");

/**
 * @brief Performs one or more write operations.
 *
 * This is a non-blocking operation, the input buffer cannot be larger than 4 bytes.
 *
 * @param[in]  device           The device descriptor for the target.
 * @param[in]  reg              The register to address on the device.
 * @param[in]  pWriteBuffer     An array of data to write to the device.
 * @param      bytes            The number of entries in the source array.
 * @param[in]  callbackFunction The function to call after the transaction is
 *                                  complete, the return status is passed as an
 *                                  argument.
 * @param[in]  pUserData        The data to pass to the callback function.
 * @param      delay            The delay in ms between each
 *                              byte write.
 * @param      pHandle          The sensor interface handle.
 * @returns true if the transaction was scheduled, false otherwise.
 */
//lint -function( fopen(1), spi_write_single_data_slow_inline(1) ) // arg cannot be null
//lint -function( fopen(1), spi_write_single_data_slow_inline(3) ) // arg cannot be null
//lint -function( fopen(1), spi_write_single_data_slow_inline(8) ) // arg cannot be null
//lint -sem( spi_write_single_data_slow_inline, 4n <= 4) // bytes must be 4 bytes or less
//lint -sem( spi_write_single_data_slow_inline, (3P >= 4n) ) // bytes must be <= buffer size
//lint -sem( spi_write_single_data_slow_inline, thread_create(5))
bool JLI spi_write_single_data_slow_inline(const SPIM_Device_t* device, UInt8 reg,
    const UInt8 *pWriteBuffer, UInt8 bytes,
    Driver_Callback_t callbackFunction, void *pUserData,
    float delay, SPIM_Handle_t* pHandle) asm("SPIM_CmdWriteBytesSlowInline");

/**
 * @brief Performs one or more read and write operations.
 *
 * This is a non-blocking operation. The read buffer will be overwritten once
 * the transaction has completed.
 *
 * @param[in]  device           The descriptor for the target.
 * @param[in]  pWriteBuffer     An array of data to write to the device.
 * @param[out] pReadBuffer      The memory location to save the read data.
 * @param      bytes            The number of entries in the source array.
 * @param[in]  callbackFunction The function to call after the transaction is
 *                                  complete, the return status is passed as an
 *                                  argument.
 * @param[in]  pUserData        The data to pass to the callback function.
 * @param      pHandle          The sensor interface handle.
 * @returns true if the transaction was scheduled, false otherwise.
 */
//lint -function( fopen(1), spi_transfer_bytes(1) ) // arg cannot be null
//lint -function( fopen(1), spi_transfer_bytes(2) ) // arg cannot be null
//lint -function( fopen(1), spi_transfer_bytes(3) ) // arg cannot be null
//lint -function( fopen(1), spi_transfer_bytes(7) ) // arg cannot be null
//lint -sem( spi_transfer_bytes, (2P >= 4n) ) // bytes must be <= buffer size
//lint -sem( spi_transfer_bytes, (3P >= 4n) ) // bytes must be <= buffer size
//lint -sem( spi_transfer_bytes, thread_create(5))
bool JLI spi_transfer_bytes(const SPIM_Device_t* device, const UInt8 *pWriteBuffer,
    UInt8 *pReadBuffer, UInt8 bytes, Driver_Callback_t callbackFunction,
    void *pUserData, SPIM_Handle_t* pHandle) asm("SPIM_TransferBytes");

/**
 * @brief Performs one or more read and write operations.
 *
 * This is a non-blocking operation. The read buffer will be overwritten once
 * the transaction has completed.
 *
 * @param[in]  device           The descriptor for the target.
 * @param[in]  pWriteBuffer     An array of data to write to the device.
 * @param[out] pReadBuffer      The memory location to save the read data.
 * @param      bytes            The number of entries in the source array.
 * @param[in]  callbackFunction The function to call after the transaction is
 *                                  complete, the return status is passed as an
 *                                  argument.
 * @param[in]  pUserData        The data to pass to the callback function.
 * @param      delay            The delay in ms between the transaction and the
 *                                  start of the next transaction.
 * @param      pHandle          The sensor interface handle.
 * @returns true if the transaction was scheduled, false otherwise.
 */
//lint -function( fopen(1), spi_transfer_bytes_slow(1) ) // arg cannot be null
//lint -function( fopen(1), spi_transfer_bytes_slow(2) ) // arg cannot be null
//lint -function( fopen(1), spi_transfer_bytes_slow(3) ) // arg cannot be null
//lint -function( fopen(1), spi_transfer_bytes_slow(8) ) // arg cannot be null
//lint -sem( spi_transfer_bytes_slow, (2P >= 4n) ) // bytes must be <= buffer size
//lint -sem( spi_transfer_bytes_slow, (3P >= 4n) ) // bytes must be <= buffer size
//lint -sem( spi_transfer_bytes_slow, thread_create(5))
bool JLI spi_transfer_bytes_slow(const SPIM_Device_t* device, const UInt8 *pWriteBuffer,
    UInt8 *pReadBuffer, UInt8 bytes, Driver_Callback_t callbackFunction,
    void *pUserData, float delay, SPIM_Handle_t* pHandle) asm("SPIM_TransferBytesSlow");

/**
 * @brief Performs one or more read and write operations.
 *
 * This is a non-blocking operation. The read buffer will be overwritten once
 * the transaction has completed.
 *
 * @param[in]  device           The descriptor for the target.
 * @param[in]  pWriteBuffer     An array of data to write to the device.
 * @param[out] pReadBuffer      The memory location to save the read data.
 * @param      bytes            The number of entries in the source array.
 * @param[in]  callbackFunction The function to call after the transaction is
 *                                  complete, the return status is passed as an
 *                                  argument.
 * @param[in]  pUserData        The data to pass to the callback function.
 * @param      delay            The delay in ms between the transaction and the
 *                                  start of the next transaction.
 * @param      pHandle          The sensor interface handle.
 * @returns true if the transaction was scheduled, false otherwise.
 */
//lint -function( fopen(1), spi_transfer_single_bytes_slow(1) ) // arg cannot be null
//lint -function( fopen(1), spi_transfer_single_bytes_slow(2) ) // arg cannot be null
//lint -function( fopen(1), spi_transfer_single_bytes_slow(3) ) // arg cannot be null
//lint -function( fopen(1), spi_transfer_single_bytes_slow(8) ) // arg cannot be null
//lint -sem( spi_transfer_single_bytes_slow, (2P >= 4n) ) // bytes must be <= buffer size
//lint -sem( spi_transfer_single_bytes_slow, (3P >= 4n) ) // bytes must be <= buffer size
//lint -sem( spi_transfer_single_bytes_slow, thread_create(5))
bool JLI spi_transfer_single_bytes_slow(const SPIM_Device_t* device,
    const UInt8 *pWriteBuffer, UInt8 *pReadBuffer, UInt8 bytes,
    Driver_Callback_t callbackFunction, void *pUserData,
    float delay, SPIM_Handle_t* pHandle) asm("SPIM_TransferBytesWithDelay");



/**
 * @fn bool JLI spi_drain_queue(const SPIM_Handle_t* pHandle);
 *
 * @brief Waits for all currently scheduled transactions to run.
 *
 *  Do note that this function will block other code from execution.
 *
 * @retval  false       No transaction was pending.
 * @retval  true        The master finished the transaction.
 */
//lint -function( fopen(1), spi_drain_queue(1) ) // arg cannot be null
bool JLI spi_drain_queue(const SPIM_Handle_t* pHandle) asm("SPIM_drainQueue");

#endif /* _ARCVER */
#endif /* !SPI_INTERFACE_H */
