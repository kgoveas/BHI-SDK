////////////////////////////////////////////////////////////////////////////////
///
/// @file       utils/elfio/ByteArray.cpp
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

#include "ByteArray.h"

#include <cstring>

class ByteArrayData
{
public:
    ByteArrayData();
    ~ByteArrayData();

    uint8_t* pData;
    uint32_t size;
    uint32_t refCount;
};

ByteArrayData::ByteArrayData()
{
    pData = 0;
    size = 0;
    refCount = 1;
}

ByteArrayData::~ByteArrayData()
{
    delete[] pData;
    pData = 0;
    size = 0;
    refCount = 0;
}

ByteArray::ByteArray() : mData(new ByteArrayData)
{
}

ByteArray::ByteArray(const ByteArray& other) : mData(other.mData)
{
    mData->refCount++;
}

ByteArray::ByteArray(const void *pData, uint32_t size) :
    mData(new ByteArrayData)
{
    mData->size = size;
    mData->pData = new uint8_t[size];

    memcpy(mData->pData, pData, size);
}

ByteArray::~ByteArray()
{
    if(1 == mData->refCount--)
    {
        delete mData;
    }

    mData = 0;
}

ByteArray& ByteArray::operator=(const ByteArray& other)
{
    mData = other.mData;
    mData->refCount++;

    return *this;
}

const void* ByteArray::Data() const
{
    return mData->pData;
}

uint32_t ByteArray::Size() const
{
    return mData->size;
}

bool ByteArray::IsEmpty() const
{
    return 0 == mData->size;
}

void ByteArray::Append(const ByteArray& other)
{
    Append(other.mData->pData, other.mData->size);
}

void ByteArray::Append(const void *pData, uint32_t size)
{
    uint32_t newSize = mData->size + size;
    uint8_t *pNewData = new uint8_t[newSize];

    memcpy(&pNewData[0], mData->pData, mData->size);
    memcpy(&pNewData[mData->size], pData, size);

    if(1 < mData->refCount)
    {
        mData->refCount--;
        mData = new ByteArrayData;
    }
    else
    {
        delete[] mData->pData;
    }

    mData->pData = pNewData;
    mData->size = newSize;
}

void ByteArray::Clear()
{
    if(0 < --mData->refCount)
    {
        mData = new ByteArrayData;
    }
    else
    {
        delete[] mData->pData;
        mData->pData = 0;
        mData->size = 0;
    }
}
