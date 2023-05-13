////////////////////////////////////////////////////////////////////////////////
///
/// @file       libs/Security/includes/ECC_Typedef.h
///
/// @project    EM7189
///
/// @brief      ECC specific typedefs
///
/// @classification  Confidential
///
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
////////////////////////////////////////////////////////////////////////////////
#ifndef ECC_TYPEDFE_H
#define ECC_TYPEDFE_H

#include <stdint.h>

/**
 * @defgroup ECC_TYPEDEF ECC specific typedefs
*/

/**Size of a point coordinate for P-160
* \ingroup ECC_TYPEDEF
*/
#define CURVE_SIZE_P160   5

/**Size of a Scalar for P-160
* \ingroup ECC_TYPEDEF
*/
#define SCALAR_SIZE_P160  6

/**Size of a coordinate(Max size supported)
* \ingroup ECC_TYPEDEF
*/
#define CURVE_SIZE_MAX    CURVE_SIZE_P160
/**Size of a coordinate(Max size supported)
* \ingroup ECC_TYPEDEF
*/
#define CURVE_SIZE        CURVE_SIZE_P160

/**Size of a scalar(Max size supported)
* \ingroup ECC_TYPEDEF
*/
#define SCALAR_SIZE       SCALAR_SIZE_P160

/**Size of a scalar(Max size supported)
* \ingroup ECC_TYPEDEF
*/
#define SCALAR_SIZE_MAX   SCALAR_SIZE

/**Generic point typedef
* \ingroup ECC_TYPEDEF
*/
typedef struct
{
    /**Coordinate x*/
    uint32_t x[CURVE_SIZE_MAX];
    /**Coordinate y*/
    uint32_t y[CURVE_SIZE_MAX];
} POINT;

/**Point on P160
* \ingroup ECC_TYPEDEF
*/

typedef struct
{
    /**Coordinate x*/
    uint32_t x[CURVE_SIZE_P160];
    /**Coordinate y*/
    uint32_t y[CURVE_SIZE_P160];
} POINT_P160;

/**Certificate type
* \ingroup ECC_TYPEDEF
*/
typedef struct
{
    /**Coordinate x*/
    uint32_t  x[SCALAR_SIZE_MAX];
    /**Coordinate y*/
    uint32_t  y[SCALAR_SIZE_MAX];
} PointCertificate;

#endif
