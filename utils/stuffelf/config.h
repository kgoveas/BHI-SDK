////////////////////////////////////////////////////////////////////////////////
///
/// @file       utils/stuffelf/config.h
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>


#define STUFFELF_CONFIG_VERSION 13

#define PRIORITY_LOWEST      PRIORITY_M

#include <types.h>
#include <SensorBus.h>

#ifdef _FRAMEWORK
#undef _FRAMEWORK
#endif

#define NUM_SENSORS_SUPPORTED        256

#define STR_PHYS_SENSOR_DESCRIPTOR            ".phys_sensor_descriptors"
#define STR_VIRT_SENSOR_DESCRIPTOR            ".virt_sensor_descriptors"
#define STR_TIME_SENSOR_DESCRIPTOR            ".timer_sensor_descriptors"

#define PHYS_SENSOR_DESCRIPTOR_SIZE        sizeof(PhysicalSensorDescriptor)
#define VIRT_SENSOR_DESCRIPTOR_SIZE        sizeof(VirtualSensorDescriptor)

#define ADDR_SPECIFIED(sensor)              do { specify(sensor, addrSpecified); } while(0)
#define ADDR_WAS_SPECIFIED(sensor)          (wasSpecified(sensor, addrSpecified))

#define INTERFACE_SPECIFIED(sensor)         do { specify(sensor, interfaceSpecified); } while(0)
#define INTERFACE_WAS_SPECIFIED(sensor)     (wasSpecified(sensor, interfaceSpecified))

#define GPIO_SPECIFIED(sensor)              do { specify(sensor, gpioSpecified); } while(0)
#define GPIO_WAS_SPECIFIED(sensor)          (wasSpecified(sensor, gpioSpecified))

#define CAL_SPECIFIED(sensor)               do { specify(sensor, calSpecified); } while(0)
#define CAL_WAS_SPECIFIED(sensor)           (wasSpecified(sensor, calSpecified))

#define OFFSET_SPECIFIED(sensor)            do { specify(sensor, offsetSpecified); } while(0)
#define OFFSET_WAS_SPECIFIED(sensor)        (wasSpecified(sensor, offsetSpecified))

#define RANGE_SPECIFIED(sensor)             do { specify(sensor, rangeSpecified); } while(0)
#define RANGE_WAS_SPECIFIED(sensor)         (wasSpecified(sensor, rangeSpecified))

#define MAX_RATE_SPECIFIED(sensor)          do { specify(sensor, maxRateSpecified); } while(0)
#define MAX_RATE_WAS_SPECIFIED(sensor)      (wasSpecified(sensor, maxRateSpecified))


#define NOISE_SPECIFIED(sensor)             do { specify(sensor, noiseSpecified); } while(0)
#define NOISE_WAS_SPECIFIED(sensor)         (wasSpecified(sensor, noiseSpecified))

#define NOISE_MODE_SPECIFIED(sensor)        do { specify(sensor, noiseModeSpecified); } while(0)
#define NOISE_MODE_WAS_SPECIFIED(sensor)    (wasSpecified(sensor, noiseModeSpecified))

#define TEMPERATURE_SOURCE_SPECIFIED(sensor)     do { virtSpecify(sensor, temperatureSource); } while(0)
#define TEMPERATURE_SOURCE_WAS_SPECIFIED(sensor) (wasVirtSpecified(sensor, temperatureSource))

#define VIRT_MAX_RATE_SPECIFIED(sensor)     do { virtSpecify(sensor, maxRateSpecified); } while(0)
#define VIRT_MAX_RATE_WAS_SPECIFIED(sensor) (wasVirtSpecified(sensor, maxRateSpecified))

bool wasSpecified(PhysicalSensorDescriptor* sensor, bool array[]);
void specify(PhysicalSensorDescriptor* sensor, bool array[]);
bool wasVirtSpecified(VirtualSensorDescriptor* sensor, bool array[]);
void virtSpecify(VirtualSensorDescriptor* sensor, bool array[]);

extern bool interfaceSpecified[NUM_SENSORS_SUPPORTED];
extern bool addrSpecified[NUM_SENSORS_SUPPORTED];
extern bool gpioSpecified[NUM_SENSORS_SUPPORTED];
extern bool calSpecified[NUM_SENSORS_SUPPORTED];
extern bool offsetSpecified[NUM_SENSORS_SUPPORTED];
extern bool rangeSpecified[NUM_SENSORS_SUPPORTED];
extern bool maxRateSpecified[NUM_SENSORS_SUPPORTED];
extern bool noiseSpecified[NUM_SENSORS_SUPPORTED];
extern bool noiseModeSpecified[NUM_SENSORS_SUPPORTED];
extern bool temperatureSource[NUM_SENSORS_SUPPORTED];

bool hasEM7189Config(void* elf);

const char* SIFModeToName(UInt8 mode);

const char* pullSelectionToName(UInt8 pullSelection);
const char* GPIOSelectionToName(UInt8 gpioSelection);
SInt8 stringToGPIOselection(const char* string);
SInt8 stringToPullselection(const char* string);
SInt8 stringToNoiseMode(const char* string);
const char* noiseModeToString(SInt8 mode);

const char* interfaceToName(SensorInterfaceType interface);

const char* typeToName(UInt8 type);
const char* typeToPhysicalName(UInt8 type);

bool readConfig(const char* configfile, void* elf);
bool writeConfig(const char* configfile, void* elf);
char *trim(char *str);
int isnumeric(char *str);

extern char quiet;
#define verbose(...)      if(!quiet) printf(__VA_ARGS__)
#define error(...)        fprintf(stderr, __VA_ARGS__)

extern EM7189Config mEM7189Config;

extern SInt32 expected_rom_version;

extern int stuffelfCfgVersion;

extern PhysicalSensorDescriptor gPhysSensorDrivers[NUM_SENSORS_SUPPORTED]; // for now allow up to 256 phys sensors...
extern VirtualSensorDescriptor gVirtSensorDrivers[NUM_SENSORS_SUPPORTED]; // for now allow up to 256 virt sensors...
