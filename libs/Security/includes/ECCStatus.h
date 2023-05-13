////////////////////////////////////////////////////////////////////////////////
///
/// @file       libs/Security/includes/ECCStatus.h
///
/// @project    EM7189
///
/// @brief      ECC status words
///
/// @classification  Confidential
///
////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
///
/// @copyright Copyright (C) 2015-2018 EM Microelectronic
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


#ifndef ECC_STATUS_H
#define ECC_STATUS_H

/**
 * @defgroup ECC ECC
 */


//High level functions status

/** OK Status
\ingroup ECC
*/
#define SW_OK                       0
/** NOK Status
\ingroup ECC
*/
#define SW_NOK                      1
/** Key reduced Status
\ingroup ECC
*/
#define SW_REDUCED_PRIVATE_KEY      2
/** Incorrect parameters: X coordinate is null
\ingroup ECC
*/
#define SW_NOK_X_NULL               3
/** Incorrect parameters: Y coordinate is null
\ingroup ECC
*/
#define SW_NOK_Y_NULL               4
/** Incorrect parameters: X value is incorrect
\ingroup ECC
*/
#define SW_NOK_X                    5
/** Incorrect parameters: Y value is incorrect
\ingroup ECC
*/
#define SW_NOK_Y                    6

//Status for decisions
#define YES                         1
#define NO                          0

//Status for the comparison
#define EQUAL                       0
#define A_BIGGER                    1
#define B_BIGGER                   -1
#endif
