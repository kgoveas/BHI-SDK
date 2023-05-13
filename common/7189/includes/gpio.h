////////////////////////////////////////////////////////////////////////////////
///
/// @file       common/7189/includes/gpio.h
///
/// @project    EM7189
///
/// @brief      Macros for gpio access.
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

#ifndef _GPIO_H
#define _GPIO_H

#include <arc.h>
#include <interrupts.h>

/**
 * @cond
 * internal only
 */

#define NUM_GPIO_PINS               25U  /**< The total number of GPIO pins available. */
#define NUM_GPIO_PULLS              28U  /**< The total number of GPIO pulls available. */

#define GPIO_BOOT_PIN   (7)
#define GPIO_QSPI_IO0   (10)
#define GPIO_QSPI_IO1   (11)
#define GPIO_QSPI_IO2   (12)
#define GPIO_QSPI_IO3   (13)
#define GPIO_SA0_PIN    (24)

#define GPIO_SIF1CLK_PULL (25)
#define GPIO_SIF1DIN_PULL (26)
#define GPIO_SIF1DIO_PULL (27)

/**
 * @endcond
 */

#if defined(_ARCVER) || defined(_FRAMEWORK)
/**
 * @fn UInt8 getGPIOValues();
 *
 * @brief Read the GPIO states for all pads.
 *
 * @returns The current state of the GPIO inputs.
 */
static ALWAYS_INLINE UInt32 getGPIOValues()
{
    return PAD.Gpiovalue.r32;
}


/**
 * @fn void setGPIOValues(UInt32 vals);
 *
 * @brief Sets all gpio outputs for all pads.
 *
 * @param   vals    The values to set the GPIO outputs to.
 */
static ALWAYS_INLINE void setGPIOValues(UInt32 vals)
{
    PAD.Gpiodout.r32 = vals;
}


/**
 * @fn bool getGPIOValue(UInt8 gpio);
 *
 * @brief Reads the specified GPIO input value.
 *
 * @retval  TRUE    The GPIO input is high.
 * @retval  FALSE   The GPIO input is low.
 */
static ALWAYS_INLINE bool getGPIOValue(UInt8 gpio)
{
    UInt32 vals = PAD.Gpiovalue.r32;
    return (vals & (1 << gpio)) == (1 << gpio);
}


/**
 * @fn UInt8 setGPIOHigh(UInt8 gpio);
 *
 * @brief Set the specified GPIO pin high.
 *
 * WARNING: The user must be careful to never use
 * a GPIO pin as an output if it is already being
 * used by a sensor driver as an input -- which means
 * the sensor is actively driving this pin.  There is
 * no protection built-in to prevent this "fight."
 *
 * @param   gpio    The gpio pin to set high.
 */
static ALWAYS_INLINE void setGPIOHigh(UInt8 gpio)
{
    UInt32 saved = disableAllInterrupts();
    PAD.Gpiodout.r32 |= (1 << gpio);
    restoreInterrupts(saved);
}


/**
 * @fn UInt8 setGPIOLow(UInt8 gpio);
 *
 * @brief Set the specified GPIO pin low.
 *
 * WARNING: The user must be careful to never use
 * a GPIO pin as an output if it is already being
 * used by a sensor driver as an input -- which means
 * the sensor is actively driving this pin.  There is
 * no protection built-in to prevent this "fight."
 *
 * @param   gpio    The gpio pin to set low.
 */
static ALWAYS_INLINE void setGPIOLow(UInt8 gpio)
{
    UInt32 saved = disableAllInterrupts();
    PAD.Gpiodout.r32 &= ~(1 << gpio);
    restoreInterrupts(saved);
}


/**
 * @fn UInt8 toggleGPIO(UInt8 gpio);
 *
 * @brief Toggle the specified GPIO output.
 *
 * WARNING: The user must be careful to never use
 * a GPIO pin as an output if it is already being
 * used by a sensor driver as an input -- which means
 * the sensor is actively driving this pin.  There is
 * no protection built-in to prevent this "fight."
 *
 * @param   gpio    The gpio pin to set toggle.
 */
static ALWAYS_INLINE void toggleGPIO(UInt8 gpio)
{
    UInt32 saved = disableAllInterrupts();
    PAD.Gpiodout.r32 ^= (1 << gpio);
    restoreInterrupts(saved);
}

/**
 * @fn void enableGPIOPull(UInt8 gpio);
 *
 * @brief Enable the hw defined pull on the GPIO pad.
 *
 * @param   gpio    The gpio pin to modify.
 */
static ALWAYS_INLINE void enableGPIOPull(UInt8 gpio)
{
    UInt32 saved = disableAllInterrupts();
    UInt32 conf = PAD.Pullenable.r32;
    conf |= (1 << gpio);
    PAD.Pullenable.r32 = conf;
    restoreInterrupts(saved);
}


/**
 * @fn void disableGPIOPull(UInt8 gpio);
 *
 * @brief Disable the hw defined pull on the GPIO pad.
 *
 * @param   gpio    The gpio pin to modify.
 */
static ALWAYS_INLINE void disableGPIOPull(UInt8 gpio)
{
    UInt32 saved = disableAllInterrupts();
    UInt32 conf = PAD.Pullenable.r32;
    conf &= ~(1 << gpio);
    PAD.Pullenable.r32 = conf;
    restoreInterrupts(saved);
}

/**
 * @fn void enableGPIOOutput(UInt8 gpio);
 *
 * @brief Enable the output driver of the specified GPIO pin.
 *
 * NOTE: This does not disable any pulls on the pin. This must be done manually
 *          by the user to avoid excessive current consumption.
 *
 * @param   gpio    The gpio pin to modify.
 */
static ALWAYS_INLINE void enableGPIOOutput(UInt8 gpio)
{
    PAD.Gpiooutd.r32 = (1 << gpio);
}

/**
 * @fn void disableGPIOOutput(UInt8 gpio);
 *
 * @brief Disable the output driver of the specified GPIO pin.
 *
 * @param   gpio    The gpio pin to modify.
 */
static ALWAYS_INLINE void disableGPIOOutput(UInt8 gpio)
{
    //PAD.Gpiooutzh.r32 = (1 << gpio);
    PAD.Gpioin.r32 = (1 << gpio);
}

/**
 * @fn void enableGPIOOutputZ(UInt8 gpio);
 *
 * @brief Change the output driver of the specified GPIO pin to high impedance (Z).
 *
 * @param   gpio    The gpio pin to modify.
 */
static ALWAYS_INLINE void enableGPIOOutputZ(UInt8 gpio)
{
    UInt32 saved = disableAllInterrupts();
    PAD.Gpiooutz.r32 |= (1 << gpio);
    restoreInterrupts(saved);
}

/**
 * @fn void setGPIODriveStrength(UInt8 gpio, bool high);
 *
 * @brief Change the output driver strength of the specified GPIO pin.
 *
 * @param   gpio    The gpio pin to modify.
 * @param   high    If set, the pad strength will be high, otherwise it will be low.
 */
static ALWAYS_INLINE void setGPIODriveStrength(UInt8 gpio, bool high)
{
    UInt32 saved = disableAllInterrupts();
    if(high)
    {
        // Set the high drive bit.
        PAD.Drivecontrol.r32 |= (1u << gpio);
    }
    else
    {
        // Clear the high drive bit.
        PAD.Drivecontrol.r32 &= ~(1u << gpio);
    }
    restoreInterrupts(saved);
}


#define GPIO_SETTING_HIGHZ          (0u)
#define GPIO_SETTING_INPUT          (1u)
#define GPIO_SETTING_OUTPUT         (2u)
#define GPIO_SETTING_INPUT_OUTPUT   (3u)
#define GPIO_SETTING_MASK           (3u)
/**
 * @fn UInt8 getGPIOSetting(UInt8 gpio);
 *
 * @brief Read the current configuration for the specified GPIO.
 *
 * @param   gpio    The gpio pin to read.
 *
 * @returns The current state of the GPIO inputs.
 */
static ALWAYS_INLINE UInt8 getGPIOSetting(UInt8 gpio)
{
    UInt32 reg;
    if(gpio >= 16)
    {
        gpio -= 16;
        reg = PAD.Gpiosetting1.r32;
    }
    else
    {
        reg = PAD.Gpiosetting0.r32;
    }

    return ((reg >> (gpio * 2)) & GPIO_SETTING_MASK);
}

#endif /* _ARCVER */
#endif /* _GPIO_H */
