////////////////////////////////////////////////////////////////////////////////
///
/// @file       common/includes/hw_versions.h
///
/// @project    EM7189
///
/// @brief      Hardware / ROM version of 718x.
///
/// @classification  Confidential
///
////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
///
/// @copyright Copyright (C) 2013-2018 EM Microelectronic
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

#ifndef _HW_VERSIONS_H
#define _HW_VERSIONS_H
#if __ASM__ /* Used when include from .s files */
.define ROM_SVN,  0xff

.define ROM_7189_DI01,     0x19
.define ROM_7189_DI01_RC1, 0x19
.define ROM_7189_DI01_RC2, 0x19
.define ROM_7189_DI01_RC3, 0x19
.define ROM_7189_DI01_RC4, 0x19

.define ROM_7189_DI02,     0x29

.define ROM_7189_DI03,     0x39

.if 0 /* Disable remaining file for asm processor. */
#endif /** __ASM__ */

#define ARCV2_EM4_CORE0 0x40    /**< Metaware value for EM4 v1.0 */
#define ARCV2_EM4_CORE1 0x41    /**< Metaware value for EM4 v1.1 */
#define ARCV2_EM4_CORE2 0x42    /**< Metaware value for EM4 v2.x */

#define ROM_VER( _major_, _minor_ ) (( (_major_) << 4) | ( _minor_ )) /**< Macro used to select the specified rom version */
#define MINOR(__version__)          ((__version__) & 0x0F)
#define MAJOR(__version__)          ((__version__) >> 4 & 0x0F)

#define ROM_SVN             ROM_VER(0x0F, 0x0F) /**< Current SVN version */

/** 7183 **/
#define PRODUCT_7181        0x1
#define PRODUCT_7183        0x3
#define PRODUCT_7189        0x9

#define ROM_7189_DI01_RC1   ROM_VER(0x01, PRODUCT_7189) /**< DI01 ROM version */
#define ROM_7189_DI01_RC2   ROM_VER(0x01, PRODUCT_7189) /**< DI01 ROM version */
#define ROM_7189_DI01_RC3   ROM_VER(0x01, PRODUCT_7189) /**< DI01 ROM version */
#define ROM_7189_DI01_RC4   ROM_VER(0x01, PRODUCT_7189) /**< DI01 ROM version */
#define ROM_7189_DI01       ROM_VER(0x01, PRODUCT_7189) /**< DI01 ROM version */

#define ROM_7189_DI02       ROM_VER(0x02, PRODUCT_7189) /**< DI02 ROM version */
#define ROM_7189_DI02_RC1   ROM_VER(0x02, PRODUCT_7189) /**< DI02 ROM version */
#define ROM_7189_DI02_RC2   ROM_VER(0x02, PRODUCT_7189) /**< DI02 ROM version */
#define ROM_7189_DI02_RC3   ROM_VER(0x02, PRODUCT_7189) /**< DI02 ROM version */

#define ROM_7189_DI03       ROM_VER(0x03, PRODUCT_7189) /**< DI03 ROM version */

#define PRODUCT(__version__)        ((__version__) & 0x0F)
#define REVISION(__version__)       ((__version__) >> 4 & 0x0F)

#ifndef _FRAMEWORK      /* condition added by vtu*/

#if ROM_VERSION == ROM_SVN
#define ARC_CORE        ARCV2_EM4_CORE1
#elif ROM_VERSION >= ROM_DI01
#define ARC_CORE        ARCV2_EM4_CORE1
#else
#ifdef _ARCVER
#error Unknown ROM version specified.
#endif
#endif

#endif


#if __ASM__ /* Used when include from .s files */
.endif
#endif
#endif /* _HW_VERSIONS_H */
