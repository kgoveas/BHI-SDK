////////////////////////////////////////////////////////////////////////////////
///
/// @file       utils/elfio/InitData.cpp
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

#include "InitData.h"
#include "ByteArray.h"

#include <cassert>
#include <cstring>
#include <iostream>
extern "C"
{

void lzrw1_decompress(const char *p_src_first, unsigned src_len,
    char *p_dst_first, unsigned *p_dst_len);
void lzrw1_compress(const char *p_src_first, unsigned src_len,
    char *p_dst_first, unsigned *p_dst_len);

#define FLAG_BYTES    1     /* Number of bytes used by copy flag. */

} // extern "C"

#define INITDAT_VERSION    (2)
#define INITDAT_VERSION_LZ (3)
#define INITDAT_MAGIC "INI"

#ifdef _MSC_VER
#define PACKED
#else
#define PACKED __attribute__((packed))
#endif // _MSC_VER

#ifdef _MSC_VER
#pragma pack(push,1)
#endif // _MSC_VER
typedef struct
{
    uint32_t address;
    int32_t count;
    char data[1];
} PACKED initdatEntry_t;
#ifdef _MSC_VER
#pragma pack(pop)
#endif // _MSC_VER

#ifdef _MSC_VER
#pragma pack(push,1)
#endif // _MSC_VER
typedef struct
{
    char magic[3];
    unsigned char version;
    initdatEntry_t entry[1];
} PACKED initdat_t;
#ifdef _MSC_VER
#pragma pack(pop)
#endif // _MSC_VER

#ifdef _MSC_VER
#pragma pack(push,1)
#endif // _MSC_VER
typedef struct
{
    char magic[3];
    unsigned char version;
    uint32_t decompress;
    initdatEntry_t entry[1];
} PACKED initdat_compressed_t;
#ifdef _MSC_VER
#pragma pack(pop)
#endif // _MSC_VER

InitDataEntry::InitDataEntry(uint32_t address, uint32_t size) : mType(
    InitDataEntry::INIT_DATA_ENTRY_ZERO), mAddress(address),
    mData(0), mSize(size)
{
}

InitDataEntry::InitDataEntry(uint32_t address, const uint8_t *pData,
    uint32_t size) : mType(InitDataEntry::INIT_DATA_ENTRY_DATA),
    mAddress(address), mData(0), mSize(0)
{
    SetData(pData, size);
}

InitDataEntry::~InitDataEntry()
{
    delete[] mData;
    mData = 0;
}

uint32_t InitDataEntry::GetAddress() const
{
    return mAddress;
}

void InitDataEntry::SetAddress(uint32_t address)
{
    mAddress = address;
}

uint32_t InitDataEntry::GetSize() const
{
    return mSize;
}

void InitDataEntry::SetSize(uint32_t size)
{
    mSize = size;
}

const uint8_t* InitDataEntry::GetData() const
{
    return mData;
}

void InitDataEntry::SetData(const uint8_t *pData, uint32_t size)
{
    delete[] mData;

    if(InitDataEntry::INIT_DATA_ENTRY_ZERO != mType && 0 < size)
    {
        mData = new uint8_t[size];
        mSize = size;
        memcpy(mData, pData, size);
    }
}

InitDataEntry::InitDataEntryType_t InitDataEntry::GetType() const
{
    return mType;
}

InitData::InitData() : mCompressed(false), mVersion(INITDAT_VERSION),
    mDecompressAddr(0u)
{
}

InitData::~InitData()
{
    for(std::list<InitDataEntry*>::const_iterator it = mEntries.begin();
        it != mEntries.end(); ++it)
    {
        delete *it;
    }
}

bool InitData::Load(const uint8_t *pInitData, int64_t size)
{
    if(0 == pInitData)
    {
        return false;
    }

    // Delete all old entries.
    for(std::list<InitDataEntry*>::const_iterator it = mEntries.begin();
        it != mEntries.end(); ++it)
    {
        delete *it;
    }

    mEntries.clear();

    // Check for the size of the header.
    if((int64_t)sizeof(initdat_t) > size)
    {
        return false;
    }

    const initdat_t *pInitDataStruct = reinterpret_cast<const initdat_t*>(
        pInitData);
    const initdat_compressed_t *pInitDataCompStruct = reinterpret_cast<
        const initdat_compressed_t*>(pInitData);

    // Check for the magic.
    if(0 != memcmp(pInitDataStruct->magic, INITDAT_MAGIC,
        strlen(INITDAT_MAGIC)))
    {
        return false;
    }

    // Check for the version.
    if(INITDAT_VERSION != pInitDataStruct->version)
    {
        if(INITDAT_VERSION_LZ > pInitDataStruct->version ||
            (int64_t)sizeof(initdat_compressed_t) > size)
        {
            return false;
        }
        else
        {
            mCompressed = true;
            mDecompressAddr = pInitDataCompStruct->decompress;
        }
    }

    // Save the version used.
    mVersion = pInitDataStruct->version;

    // Adjust the size.
    if(mCompressed)
    {
        size -= strlen(INITDAT_MAGIC) + sizeof(pInitDataCompStruct->version) + sizeof(pInitDataCompStruct->decompress);
    }
    else
    {
        size -= strlen(INITDAT_MAGIC) + sizeof(pInitDataStruct->version);
    }

    const initdatEntry_t *pEntry;

    if(mCompressed)
    {
        pEntry = &pInitDataCompStruct->entry[0];
    }
    else
    {
        pEntry = &pInitDataStruct->entry[0];
    }

    // Parse each entry.
    while(0 < size && pEntry->count && pEntry->address)
    {
        // Sanity check the size meets the requirement for the entry.
        if((int64_t)(sizeof(uint32_t) + sizeof(int32_t)) > size)
        {
            return false;
        }

        // Adjust the size.
        size -= sizeof(uint32_t) + sizeof(int32_t);

        // Determine the type for the entry.
        if(pEntry->count < 0)
        {
            uint32_t entrySize = -pEntry->count;

            AddEntry(new InitDataEntry(pEntry->address, entrySize));

            pEntry = (initdatEntry_t*)pEntry->data;
        }
        else if(pEntry->count > 0)
        {
            uint32_t entrySize = pEntry->count;
            uint32_t alignedSize = (entrySize + 3) / 4 * 4;

            // Ensure the data exists within the buffer.
            if(size < alignedSize)
            {
                return false;
            }

            // Adjust the size.
            size -= alignedSize;

            if(mCompressed)
            {
                // Maximum compression ration is 8x per the lzrw source code.
                char *pData = new char[8u * entrySize];
                uint32_t decompressedSize = 0u;

                lzrw1_decompress(&pEntry->data[0], entrySize,
                    pData, &decompressedSize);

                assert(decompressedSize <= 8u * entrySize);

                AddEntry(new InitDataEntry(pEntry->address,
                    (uint8_t*)&pData[0], decompressedSize));

                delete[] pData;
            }
            else
            {
                AddEntry(new InitDataEntry(pEntry->address,
                    (uint8_t*)&pEntry->data[0], entrySize));
            }

            pEntry = (initdatEntry_t*)(pEntry->data + alignedSize);
        }
        else
        {
            pEntry = (initdatEntry_t*) (((char*)pEntry) +
                sizeof(long) + sizeof(void*));
        }
    }

    return true;
}

uint8_t* InitData::Generate(uint32_t& size) const
{
    ByteArray data;

    // Write the header.
    if(mCompressed)
    {
        initdat_compressed_t header;
        header.version = mVersion;
        header.decompress = mDecompressAddr;
        memcpy(header.magic, INITDAT_MAGIC, strlen(INITDAT_MAGIC));
        data.Append(&header, strlen(INITDAT_MAGIC) + sizeof(header.decompress) +
            sizeof(header.version));
    }
    else
    {
        initdat_t header;
        header.version = mVersion;
        memcpy(header.magic, INITDAT_MAGIC, strlen(INITDAT_MAGIC));
        data.Append(&header, strlen(INITDAT_MAGIC) + sizeof(header.version));
    }

    // Write each entry.
    for(std::list<InitDataEntry*>::const_iterator it = mEntries.begin();
        it != mEntries.end(); ++it)
    {
        const InitDataEntry *pEntry = *it;

        /// @todo Check for overlap of memory regions.
        /// @todo Add code to combine entries of the same type and adjacent.

        initdatEntry_t entry;
        entry.address = pEntry->GetAddress();

        int32_t entrySize = (int32_t)pEntry->GetSize();

        const uint8_t *pEntryData = pEntry->GetData();
        char *pBuffer = 0;

        if(InitDataEntry::INIT_DATA_ENTRY_ZERO != pEntry->GetType() &&
            mCompressed)
        {
            uint32_t compressedSize;

            // Maximum size per lzrw source code is input size + FLAG_BYTES.
            pBuffer = new char[entrySize + FLAG_BYTES];
            lzrw1_compress((char*)pEntryData, entrySize,
                pBuffer, &compressedSize);
            entrySize = (int32_t)compressedSize;
            pEntryData = (uint8_t*)pBuffer;
        }

        if(InitDataEntry::INIT_DATA_ENTRY_ZERO == pEntry->GetType())
        {
            entry.count = -entrySize;
        }
        else
        {
            entry.count = entrySize;
        }

        data.Append(&entry, sizeof(entry.address) + sizeof(entry.count));

        if(InitDataEntry::INIT_DATA_ENTRY_DATA == pEntry->GetType())
        {
            data.Append(pEntryData, (uint32_t)entrySize);

            if(0 != (entrySize % sizeof(uint32_t)))
            {
                uint32_t zero = 0;
                uint32_t padding = sizeof(uint32_t) -
                    (entrySize % sizeof(uint32_t));

                data.Append(&zero, padding);
            }
        }

        if(0 != pBuffer)
        {
            delete[] pBuffer;
        }
    }

    // Add a blank entry at the end.
    uint64_t zero = 0;
    data.Append(&zero, sizeof(zero));

    // Copy the final data into a buffer.
    size = data.Size();
    uint8_t *pBuffer = new uint8_t[data.Size()];
    memcpy(pBuffer, data.Data(), data.Size());

    return pBuffer;
}

uint32_t InitData::GetVersion() const
{
    return mVersion;
}

bool InitData::GetCompressed() const
{
    return mCompressed;
}

uint32_t InitData::GetDecompressAddress() const
{
    return mDecompressAddr;
}

void InitData::SetDecompressAddress(uint32_t addr)
{
    mDecompressAddr = addr;
}

void InitData::SetCompressed(bool enabled)
{
    mCompressed = enabled;

    if(enabled)
    {
        mVersion = INITDAT_VERSION_LZ;
    }
    else
    {
        mVersion = INITDAT_VERSION;
    }
}

void InitData::AddEntry(InitDataEntry *pEntry)
{
    mEntries.push_back(pEntry);
}

void InitData::RemoveEntry(InitDataEntry *pEntry)
{
    mEntries.remove(pEntry);
}

std::list<InitDataEntry*> InitData::Entries() const
{
    return mEntries;
}

const uint8_t* InitData::GetDataAtAddress(uint32_t address, uint32_t size) const
{
    for(std::list<InitDataEntry*>::const_iterator it = mEntries.begin();
        it != mEntries.end(); ++it)
    {
        const InitDataEntry *pEntry = *it;

        if(address >= pEntry->GetAddress() && address < (pEntry->GetAddress() +
            pEntry->GetSize()) && (address + size) >= pEntry->GetAddress() &&
            (address + size) <= (pEntry->GetAddress() + pEntry->GetSize()))
        {
            return pEntry->GetData() + (address - pEntry->GetAddress());
        }
    }

    return 0;
}

bool InitData::SetDataAtAddress(uint32_t address, uint32_t size, const uint8_t* new_data)
{
    for(std::list<InitDataEntry*>::const_iterator it = mEntries.begin();
        it != mEntries.end(); ++it)
    {
        InitDataEntry *pEntry = *it;

        if( address >= pEntry->GetAddress() &&
            address < (pEntry->GetAddress() + pEntry->GetSize()) &&
            (address + size) >= pEntry->GetAddress() &&
            (address + size) <= (pEntry->GetAddress() + pEntry->GetSize()))
        {
            const uint8_t* data = pEntry->GetData();
            uint8_t* maleable_data = new uint8_t[pEntry->GetSize()];
            memcpy(maleable_data, data, pEntry->GetSize());
            // Overwrite modified area.
            memcpy(maleable_data + (address - pEntry->GetAddress()), new_data, size);

            // Save it back
            pEntry->SetData(maleable_data, pEntry->GetSize());
            delete[] maleable_data;
            return true;
        }
    }

    return false;
}


uint32_t InitData::GetOffsetForAddress(uint32_t address) const
{
    uint32_t offset = strlen(INITDAT_MAGIC) + sizeof(((initdat_t*)NULL)->version);

    for(std::list<InitDataEntry*>::const_iterator it = mEntries.begin();
        it != mEntries.end(); ++it)
    {
        const InitDataEntry *pEntry = *it;

        offset += sizeof(((initdatEntry_t*)NULL)->address) + sizeof(((initdatEntry_t*)NULL)->count);

        if(address >= pEntry->GetAddress() &&
            address < (pEntry->GetAddress() + pEntry->GetSize()))
        {
            return offset + (address - pEntry->GetAddress());
        }

        if(InitDataEntry::INIT_DATA_ENTRY_DATA == pEntry->GetType())
        {
            offset += pEntry->GetSize();

            if(0 != (pEntry->GetSize() % sizeof(uint32_t)))
            {
                offset += sizeof(uint32_t) -
                    (pEntry->GetSize() % sizeof(uint32_t));
            }
        }
    }

    return 0;
}
