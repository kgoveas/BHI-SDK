////////////////////////////////////////////////////////////////////////////////
///
/// @file       utils/elfio/ByteArray.h
///
/// @project    EM7189
///
/// @brief      Dynamic byte array.
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

#ifndef BYTE_ARRAY_H
#define BYTE_ARRAY_H

#include <stdint.h>

class ByteArrayData;

/**
 * Dynamic array.
 */
class ByteArray
{
public:
    /**
     * @brief Create an empty array.
     */
    ByteArray();

    /**
     * @brief Clone an existing array.
     * @param other Existing array to clone.
     */
    ByteArray(const ByteArray& other);

    /**
     * @brief Create the array from an existing buffer.
     * @param pData Pointer to the data buffer.
     * @param size Size of the data buffer.
     */
    ByteArray(const void *pData, uint32_t size);

    /**
     * @brief Free the array.
     */
    ~ByteArray();

    /**
     * @brief Get a pointer to the data within the array.
     * @retuns Pointer to the data within the array.
     */
    const void* Data() const;

    /**
     * @brief Size of the array.
     * @returns Size of the array.
     */
    uint32_t Size() const;

    /**
     * @brief Empty the array.
     */
    void Clear();

    /**
     * @brief Determine if the array is empty.
     * @retval true The array is empty.
     * @retval false The array is not empty.
     */
    bool IsEmpty() const;

    /**
     * @brief Append another array onto the end of this one.
     * @param other Other array to append onto the end of this one.
     */
    void Append(const ByteArray& other);

    /**
     * @brief Append a buffer onto the end of this array.
     * @param pData Pointer to the buffer.
     * @param size Size of the buffer.
     */
    void Append(const void *pData, uint32_t size);

    /**
     * @brief Assignment operator to clone this array.
     * @param other Existing array to clone.
     * @returns Refrence to this object.
     */
    ByteArray& operator=(const ByteArray& other);

private:
    /// Array data.
    ByteArrayData *mData;
};

#endif // BYTE_ARRAY_H
