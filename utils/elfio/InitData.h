////////////////////////////////////////////////////////////////////////////////
///
/// @file       utils/elfio/InitData.h
///
/// @project    EM7189
///
/// @brief      Class to read and modify the .initdat section.
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

#ifndef INIT_DATA_H
#define INIT_DATA_H

#include <list>
#include <stdint.h>

class InitDataEntry
{
public:
    typedef enum
    {
        INIT_DATA_ENTRY_DATA = 0,
        INIT_DATA_ENTRY_ZERO
    } InitDataEntryType_t;

    InitDataEntry(uint32_t address, uint32_t size);
    InitDataEntry(uint32_t address, const uint8_t *pData, uint32_t size);
    ~InitDataEntry();

    uint32_t GetAddress() const;
    void SetAddress(uint32_t address);

    uint32_t GetSize() const;
    void SetSize(uint32_t size);

    const uint8_t* GetData() const;
    void SetData(const uint8_t *pData, uint32_t size);

    InitDataEntryType_t GetType() const;

private:
    InitDataEntryType_t mType;
    uint32_t mAddress;
    uint8_t *mData;
    uint32_t mSize;
};


/**
 * @brief Class to read and modify the .initdat section.
 */
class InitData
{
public:
    InitData();
    ~InitData();

    bool Load(const uint8_t *pInitData, int64_t size);

    uint8_t* Generate(uint32_t& size) const;

    uint32_t GetVersion() const;

    bool GetCompressed() const;
    void SetCompressed(bool enabled);

    uint32_t GetDecompressAddress() const;
    void SetDecompressAddress(uint32_t addr);

    const uint8_t* GetDataAtAddress(uint32_t address, uint32_t size = 0) const;
    bool SetDataAtAddress(uint32_t address, uint32_t size, const uint8_t* new_data);
    uint32_t GetOffsetForAddress(uint32_t address) const;

    void AddEntry(InitDataEntry *pEntry);
    void RemoveEntry(InitDataEntry *pEntry);

    std::list<InitDataEntry*> Entries() const;

private:
    bool mCompressed;
    uint32_t mVersion;
    uint32_t mDecompressAddr;
    std::list<InitDataEntry*> mEntries;
};

#endif // INIT_DATA_H
