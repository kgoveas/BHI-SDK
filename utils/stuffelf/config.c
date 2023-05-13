////////////////////////////////////////////////////////////////////////////////
///
/// @file       utils/stuffelf/config.c
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
#define STUFFELF
#define NO_JLI_CALLS // JLI only supported on arc.


#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#define PRIORITY_LOWEST      PRIORITY_M

#include <types.h>
#include <aux_registers.h>

#if(_MSC_VER)
    int getline(char **lineptr, size_t *n, FILE *fp);
#endif

#ifdef _FRAMEWORK
#undef _FRAMEWORK
#endif



#ifdef VERSION_MAJOR
#undef VERSION_MAJOR
#endif
#define VERSION_MAJOR 1 /* Allow legacy api */

#include <EEPROMImage.h>
#include <I2CFirmwareInterface.h>
#include <SensorAPI.h>
#include <EM718xConfig.h>

#include "config.h"
#include "elf_support.h"

int stuffelfCfgVersion = -1; // invalid by default

VirtualSensorDescriptor* getVirtualDescriptorByType(UInt8 type);
PhysicalSensorDescriptor* getDescriptorByType(UInt8 type);
SInt8 stringToPullselection(const char* string);

char* rom_name_string;
char* target_hw_string;
char* build_type;
bool gNoCustomVersion;

EM7189Config mEM7189Config;

bool interfaceSpecified[NUM_SENSORS_SUPPORTED];
bool addrSpecified[NUM_SENSORS_SUPPORTED];
bool gpioSpecified[NUM_SENSORS_SUPPORTED];
bool calSpecified[NUM_SENSORS_SUPPORTED];
bool offsetSpecified[NUM_SENSORS_SUPPORTED];
bool rangeSpecified[NUM_SENSORS_SUPPORTED];
bool maxRateSpecified[NUM_SENSORS_SUPPORTED];
bool noiseSpecified[NUM_SENSORS_SUPPORTED];
bool noiseModeSpecified[NUM_SENSORS_SUPPORTED];
bool temperatureSource[NUM_SENSORS_SUPPORTED];

typedef struct string_list_t {
    char* string;
    struct string_list_t* next;
} string_list_t;

string_list_t* gIgnoredCommands;

bool wasSpecified(PhysicalSensorDescriptor* sensor, bool array[])
{
    int i;
    for(i = 0; i < NUM_SENSORS_SUPPORTED; i++)
    {
        if(sensor == &gPhysSensorDrivers[i]) return array[i];
    }
    return FALSE;
}

void specify(PhysicalSensorDescriptor* sensor, bool array[])
{
    int i;
    for(i = 0; i < NUM_SENSORS_SUPPORTED; i++)
    {
        if(sensor == &gPhysSensorDrivers[i]) array[i] = TRUE;
    }
}

bool wasVirtSpecified(VirtualSensorDescriptor* sensor, bool array[])
{
    int i;
    for(i = 0; i < NUM_SENSORS_SUPPORTED; i++)
    {
        if(sensor == &gVirtSensorDrivers[i]) return array[i];
    }
    return FALSE;
}

void virtSpecify(VirtualSensorDescriptor* sensor, bool array[])
{
    int i;
    for(i = 0; i < NUM_SENSORS_SUPPORTED; i++)
    {
        if(sensor == &gVirtSensorDrivers[i]) array[i] = TRUE;
    }
}

bool parseComment(char* line)
{
    // Comment line, do nothing for now
    return TRUE;
}

bool parseGlobalConfig(void* elf, char* line)
{
    char* origline = strdup(line);
    char* idstr = strtok(line, ",");
    if(!idstr) return FALSE; // no data in line.

    if(strcmp(idstr, "stuffelf") == 0)
    {
        int stuffelfVersion;
        idstr = strtok(NULL, ",");
        if(!idstr) return TRUE;
        if(sscanf(idstr,"%d", &stuffelfVersion))
        {
            stuffelfCfgVersion = stuffelfVersion;
        }
    }

    else if(strcmp(idstr, "irq") == 0)
    {
        int irq;
        idstr = strtok(NULL, ",");
        if(!idstr) return TRUE; // no irq pin specified.
        if(sscanf(idstr, "%d", &irq))
        {
            mEM7189Config.PinSelection.bits.HostIRQ = irq;
        }
    }

    else if(strcmp(idstr, "evcfg") == 0)
    {
        int pin = 0;
        while((idstr = strtok(NULL, ",")))
        {
            // trim whitespace from idstr
            idstr = trim(idstr);
            int selection = atoi(idstr);

            // Valid values are 0 and 1.
            if(selection >= 1) selection = 1;
            else selection = 0;

            mEM7189Config.PinSelection.bits.EvCfg &= ~(0x1 << pin);
            mEM7189Config.PinSelection.bits.EvCfg |= (selection << pin);
            pin++;
        }
    }

    else if(strcmp(idstr, "pull") == 0)
    {
        int pin = 0;
        while((idstr = strtok(NULL, ",")))
        {
            // trim whitespace from idstr
            idstr = trim(idstr);
            int selection = stringToPullselection(idstr);
            if(pin >= NUM_GPIO_PULLS)
            {
                fprintf(stderr, "ERROR: More pulls than exist (%d) are being configured.\n", NUM_GPIO_PULLS);
                exit(-1);
            }
            if(selection >= 0)
            {
                mEM7189Config.PullSelection.PullSelection &= ~((UInt64)0x1 << (1 * (UInt64)pin));
                mEM7189Config.PullSelection.PullSelection |= ((UInt64)selection << (1 * (UInt64)pin));
            }
            else
            {
                fprintf(stderr, "ERROR: Pull selection '%s' for pin %d is not valid, ignoring\n", idstr, pin);
                exit(-1);
            }
            pin++;
        }
    }
    else if(strcmp(idstr, "gpio") == 0)
    {
        int pin = 0;
        while((idstr = strtok(NULL, ",")))
        {
            // trim whitespace from idstr
            idstr = trim(idstr);
            int selection = stringToGPIOselection(idstr);
            if(pin >= NUM_GPIO_PINS)
            {
                fprintf(stderr, "ERROR: More pins than exist (%d) are being configured.\n", NUM_GPIO_PINS);
                exit(-1);
            }
            if(selection >= 0)
            {
                mEM7189Config.PinMode.PinMode &= ~((UInt64)0x2 << (2 * (UInt64)pin));
                mEM7189Config.PinMode.PinMode |= ((UInt64)selection << (2 * (UInt64)pin));
            }
            else
            {
                fprintf(stderr, "ERROR: GPIO selection '%s' for pin %d is not valid, ignoring\n", idstr, pin);
                exit(-1);
            }
            pin++;
        }
    }
    else if((strcmp(idstr, "sif_selection") == 0) ||    /* Stuffelf version <=  9 */
            (strcmp(idstr, "sif_cfg") == 0))            /* Stuffelf version >= 10 */
    {
        int selection;
        idstr = strtok(NULL, ",");
        if(!idstr) return TRUE; // no irq pin specified.
        if(sscanf(idstr, "%d", &selection))
        {
            mEM7189Config.SIFSelection = selection;
        }
    }
    else if(strcmp(idstr, "hif_disable") == 0)
    {
        int disable;
        idstr = strtok(NULL, ",");
        if(!idstr) return TRUE; // no irq pin specified.
        if(sscanf(idstr, "%d", &disable))
        {
            mEM7189Config.PinSelection.bits.HIFDisable = disable;
        }
    }
    else if(strcmp(idstr, "rom_name") == 0)
    {
        idstr = strtok(NULL, ",");
        if(idstr) rom_name_string = strdup(idstr);
    }
    else if(strcmp(idstr, "hw") == 0)
    {
        idstr = strtok(NULL, ",");
        if(idstr) target_hw_string = strdup(idstr);
    }
    else if(strcmp(idstr, "fifo") == 0)
    {
        char* floatstr = strtok(NULL, ",");
        float data;
        if(!floatstr) return TRUE;; // exit for loop, no more data
        if(sscanf(floatstr, "%f", &data))
        {
            if(hasEM7189Config(elf))
            {
                mEM7189Config.FIFOAllocationRatio = ((data - 50.0F) * 2.0F);
            }
        }
        else
        {
            return TRUE;
        }
    }
    else if(strcmp(idstr, "wordsreq") == 0)
    {
        int wordsreq;
        idstr = strtok(NULL, ",");
        if(!idstr) return TRUE; // no words required specified
        if(sscanf(idstr, "%d", &wordsreq))
        {
            mEM7189Config.FIFOWordsRequired = (unsigned short)wordsreq;
        }
    }
    else if(strcmp(idstr, "turbo") == 0)
    {
        int turbomode;
        idstr = strtok(NULL, ",");
        if(!idstr) return TRUE; // no words required specified
        if(sscanf(idstr, "%d", &turbomode))
        {
            if(turbomode)
            {
                // Set turbo mode flag
                mEM7189Config.Flags |= FLAGS_USE_TURBO_MODE;
            }
            else
            {
                // clear turbo mode flag.
                mEM7189Config.Flags &= ~FLAGS_USE_TURBO_MODE;
            }
        }
    }
    else if(strcmp(idstr, "noexec") == 0)
    {
        fprintf(stderr, "noexec option no longer supported. Ignoring.\n");
    }
    else if(strcmp(idstr, "version") == 0)
    {
        int version;
        idstr = strtok(NULL, ",");
        if(!idstr) return TRUE; // no version.
        if(sscanf(idstr, "%d", &version))
        {
            if(version)
            {
                // Use the version in the cfg file.
                mEM7189Config.CustomVersion = version;
            }
            else
            {
                // Default version will be used here.
                // Don't write out the custom version as 0 was specified.
                gNoCustomVersion = TRUE;
            }
        }
    }
    else if(strcmp(idstr, "build_type") == 0)
    {
        idstr = strtok(NULL, ",");
        if(idstr) build_type = strdup(idstr);
    }
    else
    {
        if((strcmp(line, "config_list") != 0) && /* config_list is handled by macros.cmake in the build system */
           (strcmp(line, "config_spec") != 0) && /* config_spec is handled by macros.cmake in the build system */
           (strcmp(idstr, "rom") != 0) &&        /* rom is legacy, can be ignored */
           (strcmp(line, "lib") != 0)  &&        /* lib is handled by macros.cmake in the build system */
           (strcmp(line, "ram_patches") != 0)    /* ram_patches is handled by macros.cmake in the build system */
          )
        {
            printf("Ignoring command %s\n", line);
        }
        string_list_t* container = (string_list_t*)malloc(sizeof(string_list_t));
        if(container)
        {
            container->string = origline;
            origline = NULL; // don't free on exit.
            container->next = NULL;
            if(gIgnoredCommands)
            {
                // walk to end and add
                string_list_t* end = gIgnoredCommands;
                while(end->next) end = end->next;

                end->next = container;
            }
            else
            {
                gIgnoredCommands = container;
            }
        }
    }

    if(origline)
    {
        free(origline);
    }
    return TRUE;
}

void writeIgnoredCommands(FILE* outfile)
{
    string_list_t* container = gIgnoredCommands;
    while(container )
    {
        fprintf(outfile, "%s\n", container->string);
        container = container->next;
    }

    fprintf(outfile, "\n");
}

bool updateInterface(PhysicalSensorDescriptor* sensor, const char* idstr)
{
    SensorInterfaceType interface = InterfaceAny;
    if(strcmp(idstr, "none") == 0) interface = InterfaceNone;
    if(strcmp(idstr, "i2c0") == 0) interface = InterfaceI2C0;
    if(strcmp(idstr, "i2c1") == 0) interface = InterfaceI2C1;
    if(strcmp(idstr, "spi0") == 0) interface = InterfaceSPI0;
    if(strcmp(idstr, "spi1") == 0) interface = InterfaceSPI1;

    if(interface != InterfaceAny)
    {
        sensor->device.interface.type = interface;
        INTERFACE_SPECIFIED(sensor);
    }
    else
    {
        fprintf(stderr, "Error: unknown interface '%s' specified.\n", idstr);
        exit(-1);
    }

    return FALSE;
}

const char* interfaceToName(SensorInterfaceType interface)
{
    switch(interface)
    {
        case InterfaceNone:   return "none";
        case InterfaceI2C0:   return "i2c0";
        case InterfaceI2C1:   return "i2c1";
        case InterfaceSPI0:   return "spi0";
        case InterfaceSPI1:   return "spi1";

        case InterfaceAnyI2C: return "i2c";
        case InterfaceAnySPI: return "spi";
        case InterfaceAny:    return "any";
        default:
            break;
    }

    fprintf(stderr, "Error: unknown interface specified.\n");
    exit(-1);
    return "unknown";
}

bool parsePhysicalConfig(char* line)
{
    char* idstr = strtok(line, ",");
    if(!idstr) return FALSE; // no data in line.


    int id;
    if(!sscanf(idstr, "%d", &id)) return FALSE;
    PhysicalSensorDescriptor* sensor = NULL;
    int i;
    int numSlots = (sizeof(gPhysSensorDrivers) / sizeof(PhysicalSensorDescriptor));
    for(i = 0; i < numSlots; i++)
    {
        // find first driver with correct id.
        if(gPhysSensorDrivers[i].info.id == id)
        {
            sensor = &gPhysSensorDrivers[i];
            break;
        }
    }

    if(!sensor)
    {
        return FALSE;
    }

    // Parsing config for physical sensor descriptor.

    // Determine sensor interface
    idstr = strtok(NULL, ",");
    if(!idstr) return TRUE; // no more data
    updateInterface(sensor, idstr);

    // Populate sensor address, if it exists
    idstr = strtok(NULL, ",");
    if(!idstr) return TRUE; // no more data
    int addr;
    if(sscanf(idstr, "%d", &addr))
    {
        if((int)sensor->device.interface.type & (int)InterfaceAnyI2C)
        {
            sensor->device.options.i2c.address = addr;
            ADDR_SPECIFIED(sensor);
        }
        else if((int)sensor->device.interface.type & (int)InterfaceAnySPI)
        {
            sensor->device.options.spi.csPin = addr;
            ADDR_SPECIFIED(sensor);
        }
        else
        {
            // error?
        }
    }

    // Populate GPIO Pin, if it exists
    idstr = strtok(NULL, ",");
    if(!idstr) return TRUE; // no more data
    int gpiopin;
    if(idstr[0] == '-')
    {
        sensor->device.irqDis = 1;
        GPIO_SPECIFIED(sensor);
    }
    else if(sscanf(idstr, "%d", &gpiopin))
    {
        sensor->device.irqPin = gpiopin;
        GPIO_SPECIFIED(sensor);
    }

    // Calibration Matrix
    int ci = 0;
    for(ci = 0; ci < 9; ci++)
    {
        char* floatstr = strtok(NULL, ",");
        float data;
        if(!floatstr) break; // exit for loop, no more data
        if(sscanf(floatstr, "%f", &data))
        {
            sensor->CalMatrix[ci] = data;
        }
        else
        {
            break;
        }
    }
    if(ci == 9) CAL_SPECIFIED(sensor);

    // Offset
    for(ci = 0; ci < 3; ci++)
    {
        char* floatstr = strtok(NULL, ",");
        float data;
        if(!floatstr) break; // exit for loop, no more data
        if(sscanf(floatstr, "%f", &data))
        {
#if (CONFIG_HAS_FLOAT_MATRIX == 1)
            sensor->CalOffset[ci] = data;
#endif
        }
        else
        {
            break;
        }
    }
    if(ci == 3) OFFSET_SPECIFIED(sensor);

#if SENSORS_MIN_NOISE
    // min  noise.
    char* fltstr = strtok(NULL, ",");
    float noise;
    if(!fltstr) return TRUE; // exit for loop, no more data
    if(sscanf(fltstr, "%f", &noise))
    {
        sensor->minNoiseFactor = noise;
    }
    else
    {
        return TRUE;
    }

    // min  noise.
    char* modestr = strtok(NULL, ",");
    if(!modestr) return TRUE; // exit for loop, no more data
    modestr = trim(modestr);
    int selection = stringToNoiseMode(modestr);
    if(selection >= 0)
    {
        sensor->noiseMode = selection;
        NOISE_SPECIFIED(sensor);
    }
    else
    {
        return TRUE;
    }
#endif

    // max rate.
    char* fltstr = strtok(NULL, ",");
    float maxRate;
    if(!fltstr) return TRUE; // exit for loop, no more data
    if(sscanf(fltstr, "%f", &maxRate))
    {
        if(maxRate > 0.0F)
        {
            sensor->maxRate = maxRate;
            MAX_RATE_SPECIFIED(sensor);
        }
    }
    else
    {
        return TRUE;
    }

#if INTF_ANDROIDL || INTF_STREAMING
    // default dynamic range.
    char* intstr = strtok(NULL, ",");
    int range;
    if(!intstr) return TRUE; // exit for loop, no more data
    if(sscanf(intstr, "%d", &range))
    {
        if(0 != range)
        {
            sensor->defaultDynamicRange = range;
            RANGE_SPECIFIED(sensor);
        }
    }
    else
    {
        return TRUE;
    }
#endif

    return TRUE;
}

bool parseVirtualConfig(char* line)
{
    /*** Grab the virtual id ***/
    char* idstr = strtok(line, ",");
    if(!idstr) return FALSE; // no data in line.

    int id;
    if(!sscanf(idstr, "%d", &id)) return FALSE;

    VirtualSensorDescriptor* virt = NULL;
    int i;
    int numVirtSlots = (sizeof(gVirtSensorDrivers) / sizeof(VirtualSensorDescriptor));
    for(i = 0; i < numVirtSlots; i++)
    {
        if(gVirtSensorDrivers[i].info.id == id)
        {
            virt = &gVirtSensorDrivers[i];
            break;
        }
    }

    if(!virt) return FALSE;

    /*** Grab the virtual temperature parent. ***/

    // Partsing configuration for virtual sensor descriptor.
#if defined(SENSOR_TEMPERATURE)
    // Check for temperature source
    idstr = strtok(NULL, ",");
    if(!idstr) return TRUE; // no more data
    int source = 0;
    if(sscanf(idstr, "%d", &source))
    {
        if(source)
        {
            SensorDescriptorHeader* tempsrc = NULL;
            // (1) check for physical parent
            int numSlots = (sizeof(gPhysSensorDrivers) / sizeof(PhysicalSensorDescriptor));
            for(i = 0; i < numSlots; i++)
            {
                // find first driver with correct id.
                if(gPhysSensorDrivers[i].info.id == source)
                {
                    tempsrc = cast_PhysicalToHeader(&gPhysSensorDrivers[i]);
                    break;
                }
            }
            // (2) check for virtual parent
            if(!tempsrc)
            {
                int numVirtSlots = (sizeof(gVirtSensorDrivers) / sizeof(VirtualSensorDescriptor));
                for(i = 0; i < numVirtSlots; i++)
                {
                    if(gVirtSensorDrivers[i].info.id == source)
                    {
                        // parent found in virtual area.
                        tempsrc = cast_VirtualToHeader(&gVirtSensorDrivers[i]);
                        break;
                    }
                }
            }
            if(!tempsrc)
            {
                printf("No sensor located in the firmware image with driver id %d.\n", source);
                exit(1);
            }
            else
            {
                TEMPERATURE_SOURCE_SPECIFIED(virt);
                virt->temperatureSource.sensor.type.value = tempsrc->type.value;
                virt->temperatureSource.sensor.type.flags = tempsrc->type.flags;
            }
        }
        else
        {
            TEMPERATURE_SOURCE_SPECIFIED(virt);
            virt->temperatureSource.header = 0;
        }
    }
#endif


    /*** Grab the virtual maxRate. ***/
    // max rate.
    char* fltstr = strtok(NULL, ",");
    float maxRate;
    if(!fltstr) return TRUE; // exit for loop, no more data
    if(sscanf(fltstr, "%f", &maxRate))
    {
        if(maxRate > 0.0F)
        {
            virt->maxRate = maxRate;
            VIRT_MAX_RATE_SPECIFIED(virt);
        }
    }
    else
    {
        return TRUE;
    }


    return TRUE;
}

bool readConfig(const char* configfile, void* elf)
{
    bool hasError = FALSE;
    if(!strlen(configfile))
    {
        return 1;
    }

    FILE* infile = fopen(configfile, "r");
    if(!infile) {
        fprintf(stderr, "failed to open config file %s\n", configfile);
        return 0;
    }

    if(!quiet)
    {
        printf("reading configuration data from '%s'\n", configfile);
    }

    char* line = NULL;
    size_t size = 0;
    while(getline(&line, &size, infile) != -1)
    {
        if(size && strlen(trim(line)))
        {
//                printf("Line '%s' read in\n", line);

            if(line[0] == '#')
            {
                parseComment(line);
            }
            else
            {
                char* idstr = line;
                char* origidstr = idstr;
                if(!idstr) continue; // no data in line.

                if(idstr[0] == 'm' || idstr[0] == 'a' || idstr[0] == 'g')
                {
                    idstr++;
                }

                int id;
                if(!sscanf(idstr, "%d", &id))
                {
                    parseGlobalConfig(elf, origidstr);
                }
                else
                {
                    char* linefull = strdup(idstr);

                    char* str = strdup(idstr); // ensure both Virtual and Physical config get the complete string.
                    bool status = parsePhysicalConfig(str);
                    free(str);

                    if(!status)
                    {
                        status = parseVirtualConfig(idstr);
                    }

                    if(!status)
                    {
                        printf("ERROR: no driver in firmware for line '%s'\n", linefull);
                        hasError = TRUE;
                    }

                    free(linefull);
                }
            }
        }
    }
    free(line);
    line = NULL;

    fclose(infile);

    if(hasError)
    {
        printf("Failed to read in cfg file.\n");
        exit(2);
    }

    return (hasError == FALSE);
}


bool writeConfig(const char* configfile, void* elf)
{
    if(!configfile || !strlen(configfile))
    {
        // No config file to write, OK.
        return TRUE;
    }

    if(get_section_size(elf, STR_PHYS_SENSOR_DESCRIPTOR))
    {
        UInt64 bits = 0;
        int numEntries = 0;
        FILE* outfile = fopen(configfile, "w");
        if(!outfile)
        {
            error("ERROR: Unable to open '%s' for writing.\n", configfile);
            return FALSE;
        }

        verbose("writing latest configuration data to '%s'\n", configfile);

        int numPhysSensors = get_section_size(elf, STR_PHYS_SENSOR_DESCRIPTOR) / PHYS_SENSOR_DESCRIPTOR_SIZE;
        int numTimers   = get_section_size(elf, STR_TIME_SENSOR_DESCRIPTOR) / VIRT_SENSOR_DESCRIPTOR_SIZE;
        int numVirts    = get_section_size(elf, STR_VIRT_SENSOR_DESCRIPTOR) / VIRT_SENSOR_DESCRIPTOR_SIZE;
        int numVirtSensors = numTimers + numVirts;
        int i;
        fprintf(outfile, "#Global Configuration\n");
        fprintf(outfile, "stuffelf,%d\n", STUFFELF_CONFIG_VERSION);


        if(hasEM7189Config(elf))
        {
            UInt32 hostIRQ = mEM7189Config.PinSelection.bits.HostIRQ;
            fprintf(outfile, "irq,%d\n", hostIRQ);
        }


        fprintf(outfile, "evcfg");
        for(i = 0; i < 12; i++) // 12 event interrupts.
        {
            UInt32 cfg = (mEM7189Config.PinSelection.bits.EvCfg >> i) & 1;
            fprintf(outfile, ",%d", cfg);
        }
        fprintf(outfile, "\n");



        if(hasEM7189Config(elf))
        {
            bits = mEM7189Config.PullSelection.PullSelection;
        }
        if(hasEM7189Config(elf)) numEntries = NUM_GPIO_PULLS;

        fprintf(outfile, "#Pin");
        for(i = 0; i < numEntries; i++)
        {
            fprintf(outfile, ",%4d", i);
        }
        fprintf(outfile, "\n");

        fprintf(outfile, "pull");

        for(i = 0; i < numEntries; i++)
        {
            fprintf(outfile, ",%4s", pullSelectionToName(bits & 0x01));
            bits >>= 1;
        }
        fprintf(outfile, "\n");

        fprintf(outfile, "gpio");

        if(hasEM7189Config(elf))
        {
            bits = mEM7189Config.PinSelection.PinSelection;
        }
        if(hasEM7189Config(elf)) numEntries = NUM_GPIO_PINS;

        for(i = 0; i < numEntries; i++)
        {
            fprintf(outfile, ",%4s", GPIOSelectionToName(bits & 0x03));
            bits >>= 2;
        }
        fprintf(outfile, "\n");

        if(hasEM7189Config(elf))
        {
            fprintf(outfile, "sif_cfg,%d\n\n", mEM7189Config.SIFSelection);
        }

        if(hasEM7189Config(elf))
        {
            UInt32 hif_disable = mEM7189Config.PinSelection.bits.HIFDisable;
            fprintf(outfile, "hif_disable,%d\n\n", hif_disable);
        }

        if(hasEM7189Config(elf))
        {
            fprintf(outfile, "fifo,%0.2f\n\n", ((float)mEM7189Config.FIFOAllocationRatio/2.0F) + 50.0F);
        }

        if(hasEM7189Config(elf))
        {
            fprintf(outfile, "wordsreq,%u\n\n", mEM7189Config.FIFOWordsRequired);
        }

        if(hasEM7189Config(elf))
        {
            fprintf(outfile, "turbo,%u\n\n", (mEM7189Config.Flags & FLAGS_USE_TURBO_MODE) ? 1 : 0);
        }

        if(build_type)
        {
            fprintf(outfile, "build_type,%s\n\n", build_type);
        }
        else
        {
            fprintf(outfile, "build_type,%s\n\n",  "all");
        }

        if(rom_name_string)
        {
            fprintf(outfile, "rom_name,%s\n\n", rom_name_string);
        }

        if(target_hw_string)
        {
            fprintf(outfile, "hw,%s\n\n", target_hw_string);
        }

        if(hasEM7189Config(elf))
        {
            uint16_t version = mEM7189Config.CustomVersion;
            if(gNoCustomVersion)
            {
                version = 0;
            }
            fprintf(outfile, "version,%d\n\n", version);
        }

        // Print ignored commands
        writeIgnoredCommands(outfile);

        fprintf(outfile, "#Physical Drivers\n");

        fprintf(outfile, "#DriverID,Bus,Addr,GPIO,");
        fprintf(outfile, "Cal0,Cal1,Cal2,");
        fprintf(outfile, "Cal3,Cal4,Cal5,");
        fprintf(outfile, "Cal6,Cal7,Cal8,");
        fprintf(outfile, "Off0,Off1,Off2,maxRate");
#if INTF_ANDROIDL || INTF_STREAMING
        fprintf(outfile, ",Range");
#endif
#if SENSORS_MIN_NOISE
        fprintf(outfile, ",Noise,NoiseMode,maxRate");
#endif
        fprintf(outfile, "\n");
        for(i = 0; i < numPhysSensors; i++)
        {
            PhysicalSensorDescriptor* sensor = &gPhysSensorDrivers[i];

            if(!sensor->info.id) continue; // Don't output sensor drivers with an ID of 0;
            const char* iface = interfaceToName(sensor->device.interface.type);

            int address = 0;
            if(sensor->device.interface.type & InterfaceAnyI2C)
            {
                address = sensor->device.options.i2c.address;
            }
            else if(sensor->device.interface.type & InterfaceAnySPI)
            {
                address = sensor->device.options.spi.csPin;
            }

            if(sensor->device.irqDis)
            {
                fprintf(outfile, "%d,%s,%d,-,", sensor->info.id, iface, address);
            }
            else
            {
                fprintf(outfile, "%d,%s,%d,%d,", sensor->info.id, iface, address, sensor->device.irqPin);
            }
#if CONFIG_HAS_FLOAT_MATRIX == 0
            fprintf(outfile, "% d,% d,% d,", sensor->CalMatrix[0], sensor->CalMatrix[1], sensor->CalMatrix[2]);
            fprintf(outfile, "% d,% d,% d,", sensor->CalMatrix[3], sensor->CalMatrix[4], sensor->CalMatrix[5]);
            fprintf(outfile, "% d,% d,% d,", sensor->CalMatrix[6], sensor->CalMatrix[7], sensor->CalMatrix[8]);
            fprintf(outfile, "% d,% d,% d", 0, 0, 0);
#else
            fprintf(outfile, "% f,% f,% f,", sensor->CalMatrix[0], sensor->CalMatrix[1], sensor->CalMatrix[2]);
            fprintf(outfile, "% f,% f,% f,", sensor->CalMatrix[3], sensor->CalMatrix[4], sensor->CalMatrix[5]);
            fprintf(outfile, "% f,% f,% f,", sensor->CalMatrix[6], sensor->CalMatrix[7], sensor->CalMatrix[8]);
            fprintf(outfile, "% f,% f,% f",  sensor->CalOffset[0], sensor->CalOffset[1], sensor->CalOffset[2]);
#endif

#if SENSORS_MIN_NOISE
            fprintf(outfile, ",% f, %s", sensor.minNoiseFactor, noiseModeToString(sensor.noiseMode));
#endif

            if(MAX_RATE_WAS_SPECIFIED(sensor))
            {
                fprintf(outfile, ", %f", sensor->maxRate);
            }
            else
            {
                fprintf(outfile, ", %f", -1.0F);
            }

#if INTF_ANDROIDL || INTF_STREAMING
            if(RANGE_WAS_SPECIFIED(sensor))
            {
                // Range was request from the command line or board file.
                fprintf(outfile, ", %d", sensor->defaultDynamicRange);
            }
            else
            {
                // Mark range as unspecified in the board file.
                fprintf(outfile, ", 0");
            }
#endif

            fprintf(outfile, "\n");
        }

#if defined(SENSOR_TEMPERATURE)
        fprintf(outfile, "\n#Virtual Drivers,tempSource,maxRate\n");
#else
        fprintf(outfile, "\n#Virtual Drivers,maxRate\n");
#endif

        for(i = 0; i < numVirtSensors; i++)
        {
            VirtualSensorDescriptor* sensor = &gVirtSensorDrivers[i];

            if(!sensor->info.id) continue; // Don't output sensor drivers with an ID of 0;

            fprintf(outfile, "%d", sensor->info.id);

#if defined(SENSOR_TEMPERATURE)
            int temperature_id = 0;

            if(sensor.temperatureSource.header)
            {
                // Locate driver id for given sensor.
                if(sensor->temperatureSource.sensor.type.flags == DRIVER_TYPE_PHYSICAL_FLAG)
                {
                    PhysicalSensorDescriptor* phys = (PhysicalSensorDescriptor*)getDescriptorByType(sensor->temperatureSource.sensor.type.value);
                    if(phys) temperature_id = phys->info.id;
                }
                else
                {
                    VirtualSensorDescriptor* virt = (VirtualSensorDescriptor*)getVirtualDescriptorByType(sensor->temperatureSource.sensor.type.value);
                    if(virt) temperature_id = virt->info.id;
                }
            }
            fprintf(outfile, ", %d", temperature_id);
#endif
            const char* sensorName = typeToName(sensor->type.value);

            const char* dependName = "unknown";
            const char* dependType = "unknown";
            if(VIRT_MAX_RATE_WAS_SPECIFIED(sensor))
            {
                fprintf(outfile, ", %f # %s", sensor->maxRate, sensorName);
            }
            else
            {
                fprintf(outfile, ", %f # %s", -1.0F, sensorName);
            }

            if(sensor->type.flags == DRIVER_TYPE_TIMER_FLAG)
            {
                fprintf(outfile, " depends on a %dHz timer.", sensor->triggerSource.timer.rate);
            }
            else
            {
                if((sensor->type.flags == DRIVER_TYPE_VIRTUAL_FLAG) &&
                   (sensor->triggerSource.header == 0) &&
                   ((sensor->type.value  >= SENSOR_TYPE_BSX_RESERVED_START) && (sensor->type.value  <= SENSOR_TYPE_BSX_RESERVED_END)))
                {
                    dependType = "virtual";
                    dependName = "BSX";
                }
                else
                {
                    if((SensorDescriptorHeader*)sensor->triggerSource.header == NULL)
                    {
                        dependType = "programatic";
                        dependName = "trigger";
                    }
                    else if(sensor->triggerSource.sensor.type.flags == DRIVER_TYPE_PHYSICAL_FLAG)
                    {
                        dependType = "physical";
                        dependName = typeToPhysicalName(sensor->triggerSource.sensor.type.value);
                    }
                    else
                    {
                        dependType = "virtual";
                        dependName = typeToName(sensor->triggerSource.sensor.type.value);
                    }
                }
                fprintf(outfile, " depends on a %s %s source.", dependType, dependName);
            }

            fprintf(outfile, "\n");
        }
        fclose(outfile);
    }
    return TRUE;
}


int isnumeric(char *str)
{
  while(*str)
  {
    if(!isdigit(*str))
      return 0;
    str++;
  }

  return 1;
}




char *trim(char *str)
{
    char *end;

    if(str)
    {
        // Trim leading space
        while(isspace(*str) && *str) str++;

        // Trim trailing space
        end = str + strlen(str) - 1;
        while(end > str && isspace(*end)) end--;

        end[1] = '\0';
    }
    return str;
}
