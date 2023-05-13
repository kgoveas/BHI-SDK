////////////////////////////////////////////////////////////////////////////////
///
/// @file       utils/isign/crc32.cpp
///
/// @project    EM7189
///
/// @brief
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

//----- Type defines ----------------------------------------------------------
typedef unsigned char      byte;   // Byte is a char
typedef unsigned short int word16; // 16-bit word is a short int
#ifdef __linux__
typedef unsigned int word32; // 32-bit word is an int
#elif _WIN32
typedef unsigned int word32; // 32-bit word is an int
#endif

word32 crc32(word32 crc_val, word32 data_val);

word32 update_crc_32(word32 crc_accum, word32 *data_blk_ptr, word32 data_blk_size)
{
    while (data_blk_size--)
    {
        crc_accum = crc32(crc_accum, *data_blk_ptr);
        data_blk_ptr++;
        //printf("CRC32 = %08X (%08X) \n", crc_accum, ~crc_accum);
    }
    return crc_accum;
}

word32 crc32(word32 crc_val, word32 data_val)
{
    // this probably should have been a for loop....
    unsigned int crc[32] = {
        crc_val >> 0 & 1,
        crc_val >> 1 & 1,
        crc_val >> 2 & 1,
        crc_val >> 3 & 1,
        crc_val >> 4 & 1,
        crc_val >> 5 & 1,
        crc_val >> 6 & 1,
        crc_val >> 7 & 1,
        crc_val >> 8 & 1,
        crc_val >> 9 & 1,
        crc_val >> 10 & 1,
        crc_val >> 11 & 1,
        crc_val >> 12 & 1,
        crc_val >> 13 & 1,
        crc_val >> 14 & 1,
        crc_val >> 15 & 1,
        crc_val >> 16 & 1,
        crc_val >> 17 & 1,
        crc_val >> 18 & 1,
        crc_val >> 19 & 1,
        crc_val >> 20 & 1,
        crc_val >> 21 & 1,
        crc_val >> 22 & 1,
        crc_val >> 23 & 1,
        crc_val >> 24 & 1,
        crc_val >> 25 & 1,
        crc_val >> 26 & 1,
        crc_val >> 27 & 1,
        crc_val >> 28 & 1,
        crc_val >> 29 & 1,
        crc_val >> 30 & 1,
        crc_val >> 31 & 1,
    };

    unsigned int data[32] = {
        data_val >> 0 & 1,
        data_val >> 1 & 1,
        data_val >> 2 & 1,
        data_val >> 3 & 1,
        data_val >> 4 & 1,
        data_val >> 5 & 1,
        data_val >> 6 & 1,
        data_val >> 7 & 1,
        data_val >> 8 & 1,
        data_val >> 9 & 1,
        data_val >> 10 & 1,
        data_val >> 11 & 1,
        data_val >> 12 & 1,
        data_val >> 13 & 1,
        data_val >> 14 & 1,
        data_val >> 15 & 1,
        data_val >> 16 & 1,
        data_val >> 17 & 1,
        data_val >> 18 & 1,
        data_val >> 19 & 1,
        data_val >> 20 & 1,
        data_val >> 21 & 1,
        data_val >> 22 & 1,
        data_val >> 23 & 1,
        data_val >> 24 & 1,
        data_val >> 25 & 1,
        data_val >> 26 & 1,
        data_val >> 27 & 1,
        data_val >> 28 & 1,
        data_val >> 29 & 1,
        data_val >> 30 & 1,
        data_val >> 31 & 1,
    };

    unsigned int crc_32_res[32];


    crc_32_res[0]  = data[31] ^ data[30] ^ data[29] ^ data[28] ^ data[26] ^ data[25] ^ data[24] ^ data[16] ^ data[12] ^ data[10] ^ data[9] ^ data[6] ^ data[0] ^ crc[0] ^ crc[6] ^ crc[9] ^ crc[10] ^ crc[12] ^ crc[16] ^ crc[24] ^ crc[25] ^ crc[26] ^ crc[28] ^ crc[29] ^ crc[30] ^ crc[31];
    crc_32_res[1]  = data[28] ^ data[27] ^ data[24] ^ data[17] ^ data[16] ^ data[13] ^ data[12] ^ data[11] ^ data[9] ^ data[7] ^ data[6] ^ data[1] ^ data[0] ^ crc[0] ^ crc[1] ^ crc[6] ^ crc[7] ^ crc[9] ^ crc[11] ^ crc[12] ^ crc[13] ^ crc[16] ^ crc[17] ^ crc[24] ^ crc[27] ^ crc[28];
    crc_32_res[2]  = data[31] ^ data[30] ^ data[26] ^ data[24] ^ data[18] ^ data[17] ^ data[16] ^ data[14] ^ data[13] ^ data[9] ^ data[8] ^ data[7] ^ data[6] ^ data[2] ^ data[1] ^ data[0] ^ crc[0] ^ crc[1] ^ crc[2] ^ crc[6] ^ crc[7] ^ crc[8] ^ crc[9] ^ crc[13] ^ crc[14] ^ crc[16] ^ crc[17] ^ crc[18] ^ crc[24] ^ crc[26] ^ crc[30] ^ crc[31];
    crc_32_res[3]  = data[31] ^ data[27] ^ data[25] ^ data[19] ^ data[18] ^ data[17] ^ data[15] ^ data[14] ^ data[10] ^ data[9] ^ data[8] ^ data[7] ^ data[3] ^ data[2] ^ data[1] ^ crc[1] ^ crc[2] ^ crc[3] ^ crc[7] ^ crc[8] ^ crc[9] ^ crc[10] ^ crc[14] ^ crc[15] ^ crc[17] ^ crc[18] ^ crc[19] ^ crc[25] ^ crc[27] ^ crc[31];
    crc_32_res[4]  = data[31] ^ data[30] ^ data[29] ^ data[25] ^ data[24] ^ data[20] ^ data[19] ^ data[18] ^ data[15] ^ data[12] ^ data[11] ^ data[8] ^ data[6] ^ data[4] ^ data[3] ^ data[2] ^ data[0] ^ crc[0] ^ crc[2] ^ crc[3] ^ crc[4] ^ crc[6] ^ crc[8] ^ crc[11] ^ crc[12] ^ crc[15] ^ crc[18] ^ crc[19] ^ crc[20] ^ crc[24] ^ crc[25] ^ crc[29] ^ crc[30] ^ crc[31];
    crc_32_res[5]  = data[29] ^ data[28] ^ data[24] ^ data[21] ^ data[20] ^ data[19] ^ data[13] ^ data[10] ^ data[7] ^ data[6] ^ data[5] ^ data[4] ^ data[3] ^ data[1] ^ data[0] ^ crc[0] ^ crc[1] ^ crc[3] ^ crc[4] ^ crc[5] ^ crc[6] ^ crc[7] ^ crc[10] ^ crc[13] ^ crc[19] ^ crc[20] ^ crc[21] ^ crc[24] ^ crc[28] ^ crc[29];
    crc_32_res[6]  = data[30] ^ data[29] ^ data[25] ^ data[22] ^ data[21] ^ data[20] ^ data[14] ^ data[11] ^ data[8] ^ data[7] ^ data[6] ^ data[5] ^ data[4] ^ data[2] ^ data[1] ^ crc[1] ^ crc[2] ^ crc[4] ^ crc[5] ^ crc[6] ^ crc[7] ^ crc[8] ^ crc[11] ^ crc[14] ^ crc[20] ^ crc[21] ^ crc[22] ^ crc[25] ^ crc[29] ^ crc[30];
    crc_32_res[7]  = data[29] ^ data[28] ^ data[25] ^ data[24] ^ data[23] ^ data[22] ^ data[21] ^ data[16] ^ data[15] ^ data[10] ^ data[8] ^ data[7] ^ data[5] ^ data[3] ^ data[2] ^ data[0] ^ crc[0] ^ crc[2] ^ crc[3] ^ crc[5] ^ crc[7] ^ crc[8] ^ crc[10] ^ crc[15] ^ crc[16] ^ crc[21] ^ crc[22] ^ crc[23] ^ crc[24] ^ crc[25] ^ crc[28] ^ crc[29];
    crc_32_res[8]  = data[31] ^ data[28] ^ data[23] ^ data[22] ^ data[17] ^ data[12] ^ data[11] ^ data[10] ^ data[8] ^ data[4] ^ data[3] ^ data[1] ^ data[0] ^ crc[0] ^ crc[1] ^ crc[3] ^ crc[4] ^ crc[8] ^ crc[10] ^ crc[11] ^ crc[12] ^ crc[17] ^ crc[22] ^ crc[23] ^ crc[28] ^ crc[31];
    crc_32_res[9]  = data[29] ^ data[24] ^ data[23] ^ data[18] ^ data[13] ^ data[12] ^ data[11] ^ data[9] ^ data[5] ^ data[4] ^ data[2] ^ data[1] ^ crc[1] ^ crc[2] ^ crc[4] ^ crc[5] ^ crc[9] ^ crc[11] ^ crc[12] ^ crc[13] ^ crc[18] ^ crc[23] ^ crc[24] ^ crc[29];
    crc_32_res[10] = data[31] ^ data[29] ^ data[28] ^ data[26] ^ data[19] ^ data[16] ^ data[14] ^ data[13] ^ data[9] ^ data[5] ^ data[3] ^ data[2] ^ data[0] ^ crc[0] ^ crc[2] ^ crc[3] ^ crc[5] ^ crc[9] ^ crc[13] ^ crc[14] ^ crc[16] ^ crc[19] ^ crc[26] ^ crc[28] ^ crc[29] ^ crc[31];
    crc_32_res[11] = data[31] ^ data[28] ^ data[27] ^ data[26] ^ data[25] ^ data[24] ^ data[20] ^ data[17] ^ data[16] ^ data[15] ^ data[14] ^ data[12] ^ data[9] ^ data[4] ^ data[3] ^ data[1] ^ data[0] ^ crc[0] ^ crc[1] ^ crc[3] ^ crc[4] ^ crc[9] ^ crc[12] ^ crc[14] ^ crc[15] ^ crc[16] ^ crc[17] ^ crc[20] ^ crc[24] ^ crc[25] ^ crc[26] ^ crc[27] ^ crc[28] ^ crc[31];
    crc_32_res[12] = data[31] ^ data[30] ^ data[27] ^ data[24] ^ data[21] ^ data[18] ^ data[17] ^ data[15] ^ data[13] ^ data[12] ^ data[9] ^ data[6] ^ data[5] ^ data[4] ^ data[2] ^ data[1] ^ data[0] ^ crc[0] ^ crc[1] ^ crc[2] ^ crc[4] ^ crc[5] ^ crc[6] ^ crc[9] ^ crc[12] ^ crc[13] ^ crc[15] ^ crc[17] ^ crc[18] ^ crc[21] ^ crc[24] ^ crc[27] ^ crc[30] ^ crc[31];
    crc_32_res[13] = data[31] ^ data[28] ^ data[25] ^ data[22] ^ data[19] ^ data[18] ^ data[16] ^ data[14] ^ data[13] ^ data[10] ^ data[7] ^ data[6] ^ data[5] ^ data[3] ^ data[2] ^ data[1] ^ crc[1] ^ crc[2] ^ crc[3] ^ crc[5] ^ crc[6] ^ crc[7] ^ crc[10] ^ crc[13] ^ crc[14] ^ crc[16] ^ crc[18] ^ crc[19] ^ crc[22] ^ crc[25] ^ crc[28] ^ crc[31];
    crc_32_res[14] = data[29] ^ data[26] ^ data[23] ^ data[20] ^ data[19] ^ data[17] ^ data[15] ^ data[14] ^ data[11] ^ data[8] ^ data[7] ^ data[6] ^ data[4] ^ data[3] ^ data[2] ^ crc[2] ^ crc[3] ^ crc[4] ^ crc[6] ^ crc[7] ^ crc[8] ^ crc[11] ^ crc[14] ^ crc[15] ^ crc[17] ^ crc[19] ^ crc[20] ^ crc[23] ^ crc[26] ^ crc[29];
    crc_32_res[15] = data[30] ^ data[27] ^ data[24] ^ data[21] ^ data[20] ^ data[18] ^ data[16] ^ data[15] ^ data[12] ^ data[9] ^ data[8] ^ data[7] ^ data[5] ^ data[4] ^ data[3] ^ crc[3] ^ crc[4] ^ crc[5] ^ crc[7] ^ crc[8] ^ crc[9] ^ crc[12] ^ crc[15] ^ crc[16] ^ crc[18] ^ crc[20] ^ crc[21] ^ crc[24] ^ crc[27] ^ crc[30];
    crc_32_res[16] = data[30] ^ data[29] ^ data[26] ^ data[24] ^ data[22] ^ data[21] ^ data[19] ^ data[17] ^ data[13] ^ data[12] ^ data[8] ^ data[5] ^ data[4] ^ data[0] ^ crc[0] ^ crc[4] ^ crc[5] ^ crc[8] ^ crc[12] ^ crc[13] ^ crc[17] ^ crc[19] ^ crc[21] ^ crc[22] ^ crc[24] ^ crc[26] ^ crc[29] ^ crc[30];
    crc_32_res[17] = data[31] ^ data[30] ^ data[27] ^ data[25] ^ data[23] ^ data[22] ^ data[20] ^ data[18] ^ data[14] ^ data[13] ^ data[9] ^ data[6] ^ data[5] ^ data[1] ^ crc[1] ^ crc[5] ^ crc[6] ^ crc[9] ^ crc[13] ^ crc[14] ^ crc[18] ^ crc[20] ^ crc[22] ^ crc[23] ^ crc[25] ^ crc[27] ^ crc[30] ^ crc[31];
    crc_32_res[18] = data[31] ^ data[28] ^ data[26] ^ data[24] ^ data[23] ^ data[21] ^ data[19] ^ data[15] ^ data[14] ^ data[10] ^ data[7] ^ data[6] ^ data[2] ^ crc[2] ^ crc[6] ^ crc[7] ^ crc[10] ^ crc[14] ^ crc[15] ^ crc[19] ^ crc[21] ^ crc[23] ^ crc[24] ^ crc[26] ^ crc[28] ^ crc[31];
    crc_32_res[19] = data[29] ^ data[27] ^ data[25] ^ data[24] ^ data[22] ^ data[20] ^ data[16] ^ data[15] ^ data[11] ^ data[8] ^ data[7] ^ data[3] ^ crc[3] ^ crc[7] ^ crc[8] ^ crc[11] ^ crc[15] ^ crc[16] ^ crc[20] ^ crc[22] ^ crc[24] ^ crc[25] ^ crc[27] ^ crc[29];
    crc_32_res[20] = data[30] ^ data[28] ^ data[26] ^ data[25] ^ data[23] ^ data[21] ^ data[17] ^ data[16] ^ data[12] ^ data[9] ^ data[8] ^ data[4] ^ crc[4] ^ crc[8] ^ crc[9] ^ crc[12] ^ crc[16] ^ crc[17] ^ crc[21] ^ crc[23] ^ crc[25] ^ crc[26] ^ crc[28] ^ crc[30];
    crc_32_res[21] = data[31] ^ data[29] ^ data[27] ^ data[26] ^ data[24] ^ data[22] ^ data[18] ^ data[17] ^ data[13] ^ data[10] ^ data[9] ^ data[5] ^ crc[5] ^ crc[9] ^ crc[10] ^ crc[13] ^ crc[17] ^ crc[18] ^ crc[22] ^ crc[24] ^ crc[26] ^ crc[27] ^ crc[29] ^ crc[31];
    crc_32_res[22] = data[31] ^ data[29] ^ data[27] ^ data[26] ^ data[24] ^ data[23] ^ data[19] ^ data[18] ^ data[16] ^ data[14] ^ data[12] ^ data[11] ^ data[9] ^ data[0] ^ crc[0] ^ crc[9] ^ crc[11] ^ crc[12] ^ crc[14] ^ crc[16] ^ crc[18] ^ crc[19] ^ crc[23] ^ crc[24] ^ crc[26] ^ crc[27] ^ crc[29] ^ crc[31];
    crc_32_res[23] = data[31] ^ data[29] ^ data[27] ^ data[26] ^ data[20] ^ data[19] ^ data[17] ^ data[16] ^ data[15] ^ data[13] ^ data[9] ^ data[6] ^ data[1] ^ data[0] ^ crc[0] ^ crc[1] ^ crc[6] ^ crc[9] ^ crc[13] ^ crc[15] ^ crc[16] ^ crc[17] ^ crc[19] ^ crc[20] ^ crc[26] ^ crc[27] ^ crc[29] ^ crc[31];
    crc_32_res[24] = data[30] ^ data[28] ^ data[27] ^ data[21] ^ data[20] ^ data[18] ^ data[17] ^ data[16] ^ data[14] ^ data[10] ^ data[7] ^ data[2] ^ data[1] ^ crc[1] ^ crc[2] ^ crc[7] ^ crc[10] ^ crc[14] ^ crc[16] ^ crc[17] ^ crc[18] ^ crc[20] ^ crc[21] ^ crc[27] ^ crc[28] ^ crc[30];
    crc_32_res[25] = data[31] ^ data[29] ^ data[28] ^ data[22] ^ data[21] ^ data[19] ^ data[18] ^ data[17] ^ data[15] ^ data[11] ^ data[8] ^ data[3] ^ data[2] ^ crc[2] ^ crc[3] ^ crc[8] ^ crc[11] ^ crc[15] ^ crc[17] ^ crc[18] ^ crc[19] ^ crc[21] ^ crc[22] ^ crc[28] ^ crc[29] ^ crc[31];
    crc_32_res[26] = data[31] ^ data[28] ^ data[26] ^ data[25] ^ data[24] ^ data[23] ^ data[22] ^ data[20] ^ data[19] ^ data[18] ^ data[10] ^ data[6] ^ data[4] ^ data[3] ^ data[0] ^ crc[0] ^ crc[3] ^ crc[4] ^ crc[6] ^ crc[10] ^ crc[18] ^ crc[19] ^ crc[20] ^ crc[22] ^ crc[23] ^ crc[24] ^ crc[25] ^ crc[26] ^ crc[28] ^ crc[31];
    crc_32_res[27] = data[29] ^ data[27] ^ data[26] ^ data[25] ^ data[24] ^ data[23] ^ data[21] ^ data[20] ^ data[19] ^ data[11] ^ data[7] ^ data[5] ^ data[4] ^ data[1] ^ crc[1] ^ crc[4] ^ crc[5] ^ crc[7] ^ crc[11] ^ crc[19] ^ crc[20] ^ crc[21] ^ crc[23] ^ crc[24] ^ crc[25] ^ crc[26] ^ crc[27] ^ crc[29];
    crc_32_res[28] = data[30] ^ data[28] ^ data[27] ^ data[26] ^ data[25] ^ data[24] ^ data[22] ^ data[21] ^ data[20] ^ data[12] ^ data[8] ^ data[6] ^ data[5] ^ data[2] ^ crc[2] ^ crc[5] ^ crc[6] ^ crc[8] ^ crc[12] ^ crc[20] ^ crc[21] ^ crc[22] ^ crc[24] ^ crc[25] ^ crc[26] ^ crc[27] ^ crc[28] ^ crc[30];
    crc_32_res[29] = data[31] ^ data[29] ^ data[28] ^ data[27] ^ data[26] ^ data[25] ^ data[23] ^ data[22] ^ data[21] ^ data[13] ^ data[9] ^ data[7] ^ data[6] ^ data[3] ^ crc[3] ^ crc[6] ^ crc[7] ^ crc[9] ^ crc[13] ^ crc[21] ^ crc[22] ^ crc[23] ^ crc[25] ^ crc[26] ^ crc[27] ^ crc[28] ^ crc[29] ^ crc[31];
    crc_32_res[30] = data[30] ^ data[29] ^ data[28] ^ data[27] ^ data[26] ^ data[24] ^ data[23] ^ data[22] ^ data[14] ^ data[10] ^ data[8] ^ data[7] ^ data[4] ^ crc[4] ^ crc[7] ^ crc[8] ^ crc[10] ^ crc[14] ^ crc[22] ^ crc[23] ^ crc[24] ^ crc[26] ^ crc[27] ^ crc[28] ^ crc[29] ^ crc[30];
    crc_32_res[31] = data[31] ^ data[30] ^ data[29] ^ data[28] ^ data[27] ^ data[25] ^ data[24] ^ data[23] ^ data[15] ^ data[11] ^ data[9] ^ data[8] ^ data[5] ^ crc[5] ^ crc[8] ^ crc[9] ^ crc[11] ^ crc[15] ^ crc[23] ^ crc[24] ^ crc[25] ^ crc[27] ^ crc[28] ^ crc[29] ^ crc[30] ^ crc[31];


    return (crc_32_res[0] << 0) |
           (crc_32_res[1] << 1) |
           (crc_32_res[2] << 2) |
           (crc_32_res[3] << 3) |
           (crc_32_res[4] << 4) |
           (crc_32_res[5] << 5) |
           (crc_32_res[6] << 6) |
           (crc_32_res[7] << 7) |
           (crc_32_res[8] << 8) |
           (crc_32_res[9] << 9) |
           (crc_32_res[10] << 10) |
           (crc_32_res[11] << 11) |
           (crc_32_res[12] << 12) |
           (crc_32_res[13] << 13) |
           (crc_32_res[14] << 14) |
           (crc_32_res[15] << 15) |
           (crc_32_res[16] << 16) |
           (crc_32_res[17] << 17) |
           (crc_32_res[18] << 18) |
           (crc_32_res[19] << 19) |
           (crc_32_res[20] << 20) |
           (crc_32_res[21] << 21) |
           (crc_32_res[22] << 22) |
           (crc_32_res[23] << 23) |
           (crc_32_res[24] << 24) |
           (crc_32_res[25] << 25) |
           (crc_32_res[26] << 26) |
           (crc_32_res[27] << 27) |
           (crc_32_res[28] << 28) |
           (crc_32_res[29] << 29) |
           (crc_32_res[30] << 30) |
           (crc_32_res[31] << 31);
}
