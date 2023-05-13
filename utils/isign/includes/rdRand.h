////////////////////////////////////////////////////////////////////////////////
///
/// @file       utils/isign/includes/rdRand.h
///
/// @project    EM7189
///
/// @brief     Random utilities
///
/// @classification  Confidential
///
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
///
/// @copyright Copyright (C) 2015-2019 EM Microelectronic
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
/////////////////////////////////////////////////////////////////////////////////
#ifndef _RD_RAND_H
#define _RD_RAND_H

#include <stdint.h>

/**
* @defgroup RD_RAND RD_RAND
*/

/** OK Status
\ingroup RD_RAND
*/
#define SW_OK                       0

/** Random rdrand not supported by this platform
\ingroup RD_RAND
*/
#define SW_RDRAND_NOT_SUPPORTED     7

/** Random generation with rdrand failure
\ingroup RD_RAND
*/
#define SW_RDRAND_FAILED            8

/**
* \fn uint8_t GenRandomBuffer(uint32_t *Random, uint32_t Size)

* \brief Generate a buffer with rdRand instruction
* \param[out] Random Pointer on the recieving buffer
* \param[in] Size Number of 32-bit word to generate
* \return : SW_OK if the generation was successful, SW_RDRAND_NOT_SUPPORTED if rdRand instruction is not supported. SW_RDRAND_FAILED if the generation failed.

\ingroup RD_RAND
*/
extern uint8_t GenRandomBuffer(uint32_t *Random, uint32_t Size);

#endif