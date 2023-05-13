/* file elf2bin.c */
/*--------------------------------------------------------------------------
   (C) Copyright 2000-2009; ARC International - MetaWare Tools;  Santa Cruz, CA 95060

2.1 11  Jan 2010 MJM Rewrote argument parsing and added -P option (cr101001).
2.0 20  Aug 2009 DP  Fixed indentation, etc.
1.9 16  Mar 2004 TBH fix to top address not padding to top address
        -ttop_address_limit.
1.8 21  May 2003 TBH for Dom Cobley, Alphamosaic - low address limit low bug
        (-b) fixed.
1.7 03  Feb 2003 TBH fix to load segment sorting algorithm causing crash.
1.6 18  Nov 2002 TBH start and stop address pad and cut as needed rather than
        simply selecting the segment.Handles out of address order load segments.
1.5 21  May 2001 TBH check for segment address backup in image before size to
        correct inproper error message.
        -f fill32bit fill value added option.
        End of image is 4byte aligned.
        Change in 1.5 has options can be ordered in any manor but all options
        must have -flags
    Default is a.out for input, and output file default will be input
        with .bin extension.
    Big endian host ready
        -h -? give usage message.
1.4 1   Nov 2001 TBH added not dump of empty segments

---------------------------------------------------------------------------*/
/* ELF to binary conversion tool */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <inttypes.h>

#define STUFFELF    1

#define MAX_VIRTUAL_SENSORS 256

#define BSX_DRIVER_ID   240

#define DATA_RAM_START 0xa04000
#define CODE_RAM_START 0x124000
#define FLASH_START    0x200000
#define INITIAL_BANK_SIZE   (16 * 1024)
#define BANK_SIZE           (32 * 1024)
#define NO_JLI_CALLS // JLI only supported on arc.

/* Various FIFO related sizes for estimated FIFO size calculations */
#define FIFO_FIXED_OVERHEAD     (32u)       /* Fixed overhead per FIFO being allocated. */
#define NUM_FIFOS               (2u)        /* Number of FIFOs to allocate - Wake and Non-Wake. */
#define FIFO_BLOCK_SIZE         (512u)      /* Block size for the Wake/Non-Wake FIFOs. */
#define MIN_FIFO_BLOCKS         (3u)        /* Minumum number of blocks in each FIFO. */
#define FIFO_STATUS_SIZE        (688u)      /* Amount of RAM allocated by firmware for the Status FIFO. */
#define CMD_BUFFER_SIZE         (4096u)     /* Amount of RAM allocated for the command buffer */

#define PRIORITY_LOWEST      PRIORITY_M

#include <types.h>
#include <aux_registers.h>


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

char quiet = 0;

void printTriggerList(SensorDescriptorHeader* sensor, UInt32 depth_offset);
void printPhysicalSensorDrivers(PhysicalSensorDescriptor* phys, UInt32 numPhys);
void printVirtualSensorDrivers(VirtualSensorDescriptor* virt, UInt32 numVirt, PhysicalSensorDescriptor* physdr, UInt32 numPhys);
UInt32 getSensorDepth(SensorDescriptorHeader* sensor);
/*Note: we can't use "long" because it is 64-bits on 64-bit machines*/
const char* driverTypeToName(UInt8 type);
char* GPIOToName(UInt8 pin);



bool parseSensors(void* elf);

bool stuffSensors(void* elf);

bool parseSensorDrivers(PhysicalSensorDescriptor* phys, UInt32 numPhys, VirtualSensorDescriptor* virt, UInt32 numVirt);
VirtualSensorDescriptor* gVirts;
UInt8 gNumVirts;

// Support up to 256 sensors.
void* gTriggerListEntries[MAX_VIRTUAL_SENSORS];
void* gTriggerSourceEntries[MAX_VIRTUAL_SENSORS];


#define GPIO_RISING             1
#define GPIO_FALLING            0

#define STR_EM7189_DESCRIPTOR            ".em7189_descriptor"


#define STR_FWHDR                       ".fw_header"
#define STR_FLASH_FWHDR                 ".flash_fw_header"
#define STR_INITDAT                     ".initdat"

#define MAX_PATH 256

#define USAGE_STR "EM Microelectronic Stuff-Elf Utility v1.0\n"\
    "Usage: %s <elffile> [<options...>]\n"\
    "Where:\n"\
    "    <elffile> is the path of the ELF file to be converted.\n"\
    "    options are:\n"\
    "       -q\n" \
    "           Do not print status information on stdout\n" \
    "       -f<config file>\n" \
    "           Use the specified configuration file for sensor information.\n" \
    "           The configuration file will be created if nonexistant, or updated\n" \
    "            with any parameters specified on the command line.\n" \
    "       --irq<pin>\n" \
    "           Specify the host interrupt pin.\n" \
    "       --pull<pin><up|down|default|none>\n" \
    "           Specify the pull configuration for the gpio pin.\n" \
    "       --gpio<pin><hiz|in|high|1|low|0>\n" \
    "           Specify the intput/output configuration for the gpio pin.\n" \
    "       --fifo<%%wakeup>\n" \
    "           Specify the percentage of the FIFO in the wakeup FIFO (Android L+).\n" \
    "       --wordsreq<words|0>\n" \
    "           Specify the number of words of RAM, minimum, to be available for the FIFO,\n" \
    "            with 0 being the default, meaning use whatever is already free.\n" \
    "\n" \
    "       -m\n" \
    "           Select the mag sensor.\n" \
    "       -a\n" \
    "           Select the accel sensor.\n" \
    "       -g\n" \
    "           Select the gyro sensor.\n" \
    "       -<id 0..9>\n" \
    "           Select the <id>th sensor.\n" \
    "\n" \
    "       -d<addr>\n" \
    "           Specify the I2C address for the selected sensor.\n" \
    "       -p<pin>\n" \
    "           Specify the GPIO pin used for the selected sensor.\n" \
    "       -c<v1,v2,v3,v4,v5,v6,v76,v8,v9>\n" \
    "           Specify the cal matric used for the selected sensor.\n" \
    "       -o<v1,v2,v3>\n" \
    "           Specify the cal offset used for the selected sensor.\n" \
    "       --range<default range>\n" \
    "           Specify the default dynamic range of a physical sensor (Android L+).\n" \
    "       --noise<min noise>\n" \
    "           Specify the minimum noise allowed for the sensor.\n" \
    "       --noise_mode<0,1>\n" \
    "           Specify the noise measurement mode.\n" \
    "               0: The noise is averaged.\n" \
    "               1: The noise is directly measured.\n" \
    "       --version<m>\n" \
    "           Specify the custom version number of the firmware image.\n" \

static char infile[MAX_PATH];
static char configfile[MAX_PATH];

/*  Parses the input argument as an unsigned integer (typically hex or decimal)
    and returns the result.  Returns 0 if successful and -1 if not.
*/
static int parse_uint32(char *arg, uint32_t *uval) {
    uint32_t tmp_ul;
    char *endp;
    int ret = 0;
    tmp_ul = strtoul(arg,&endp,0);
    if (endp == arg || *endp != '\0')
    ret = -1;
    *uval = tmp_ul;
    return ret;
    }

PhysicalSensorDescriptor gPhysSensorDrivers[NUM_SENSORS_SUPPORTED]; // for now allow up to 256 phys sensors...
VirtualSensorDescriptor gVirtSensorDrivers[NUM_SENSORS_SUPPORTED]; // for now allow up to 256 virt sensors...



PhysicalSensorDescriptor* getDescriptorByType(void* elf, UInt8 type)
{
    PhysicalSensorDescriptor* selectedSensor;
    int numSensors =  get_section_size(elf, STR_PHYS_SENSOR_DESCRIPTOR) / PHYS_SENSOR_DESCRIPTOR_SIZE;
    int i;
    selectedSensor = NULL;
    for(i = 0; i < numSensors; i++)
    {
        if(gPhysSensorDrivers[i].type.value == type)
        {
            selectedSensor = &gPhysSensorDrivers[i];
            break;
        }
    }
    if(!selectedSensor)
    {
        printf("No %s sensor located in the firmware image.\n", typeToPhysicalName(type));
        exit(1);
    }

    return selectedSensor;
}


VirtualSensorDescriptor* getVirtualDescriptorByType(void* elf, UInt8 type)
{
    VirtualSensorDescriptor* selectedSensor;
    int numSensors =  (get_section_size(elf, STR_VIRT_SENSOR_DESCRIPTOR) + get_section_size(elf, STR_TIME_SENSOR_DESCRIPTOR)) / VIRT_SENSOR_DESCRIPTOR_SIZE;
    int i;
    selectedSensor = NULL;
    for(i = 0; i < numSensors; i++)
    {
        if(gVirtSensorDrivers[i].type.value == type)
        {
            selectedSensor = &gVirtSensorDrivers[i];
            break;
        }
    }
    if(!selectedSensor)
    {
        printf("No %s sensor located in the firmware image.\n", typeToName(type));
        exit(1);
    }

    return selectedSensor;
}


bool hasEM7189Config(void* elf)
{
    return (get_section_size(elf, STR_EM7189_DESCRIPTOR) != 0);
}


void validateEM7189Config(void* elf)
{
    if(hasEM7189Config(elf))
    {
        const char* config = get_section_data(elf, STR_EM7189_DESCRIPTOR);
        if(config)
        {
            // Read current descriptor
            memcpy(&mEM7189Config, config, sizeof(EM7189Config));

            if(mEM7189Config.MagicNumber != EM7189_CONFIG_MAGIC_NUMBER)
            {
                error("ERROR: EM7189Config magic mismatch, disabling usage of em7189 config: 0x%x, expected 0x%X\n", mEM7189Config.MagicNumber, EM7189_CONFIG_MAGIC_NUMBER);
                exit(-1);
            }
            else if(mEM7189Config.Version != EM7189_CONFIG_VERSION)
            {
                error("ERROR: Unable to handle config version %d, disabling usage of em7189 config (expected %d)\n", mEM7189Config.Version, EM7189_CONFIG_VERSION);
                exit(-1);
            }
            else
            {
                // Cfg valid.
            }
        }
        else
        {
            printf("ERROR: Unable to read in EM7189Config\n");
            exit(-1);
        }
    }
}

void parseElfSensors(void* elf)
{
    UInt32 hdr_pos = get_section_size(elf, STR_FWHDR);
    UInt32 flash_hdr_pos = get_section_size(elf, STR_FLASH_FWHDR);

    if (0 == flash_hdr_pos &&
        0 == hdr_pos)
    {
        exit(-1);
    }

    if(parseSensors(elf))
    {
        verbose("Handling 2.x sensors.\n");
    }
    else
    {
        error("ERROR: Unable to locate sensor descriptors in firmware image.\n");
        //exit(0);
    }
}

int main(int argc, char *argv[]) {
    PhysicalSensorDescriptor* selectedSensor = 0;
    int num_errors = 0;
    int i;
    void* elf;
    char *cp, *arg;
    int option_count;
    char c;

    strcpy(infile, "");

    /* these macros provide option arguments either adjacent to the option
    character or if there is nothing adjacent, from the next argument
    in the argv array.
    */
    #define UNSIGNED_ARG(uint) \
    if (*cp)\
        arg = cp;\
    else if (option_count < argc) {\
        arg = argv[option_count++];\
        }\
    else {\
        error(USAGE_STR, argv[0]);\
        return 1;\
        }\
    if (parse_uint32(arg, &uint) != 0) {\
        error("%s invalid numeric argument for '%c' option\n", argv[0], c);\
        return 1;\
        }

    #define CHARP_ARG(cp_arg) \
    if (*cp)\
        cp_arg = cp;\
    else if (option_count < argc) {\
        cp_arg = argv[option_count++];\
        }\
    else {\
        error(USAGE_STR, argv[0]);\
        return 1;\
        }

    // loop through args to determine file name.
    option_count = 1;
    while (option_count < argc) {
        c= 0;
        cp = argv[option_count];
        ++option_count;
        if (*cp == '-') {
            if (*(cp+1)) {
                ++cp;
                c= *cp; // tolower(*cp);
                ++cp;
            }
            else continue;  /* we ignore a dash by itself */
        } /* was neg appended option */
        else {
            if (!strlen(infile)) {
                strcpy(infile,cp);
                continue;
            }
        }

        if (c == 'q') {
            if (*cp) {
                error(USAGE_STR, argv[0]);
                return 1;
            }
            else quiet = 1;
        }
    }

    if (!strlen(infile)) {
        error(USAGE_STR, argv[0]);
        return 1;
    }

    elf = OpenElf(infile);

    if (NULL == elf) {
        error("%s file  %s: open failed\n",argv[0], infile);
        return 1;
    }

    verbose("%s: ELF file %s", argv[0], infile);
    verbose("; all segments\n");

    validateEM7189Config(elf);

    bool status = FALSE;
    if (!status)
    {
        parseElfSensors(elf);
    }


    strcpy(infile, "");
    strcpy(configfile, "");
    option_count = 1;
    while (option_count < argc) {
        c= 0;
        cp = argv[option_count];
        ++option_count;
        if (*cp == '-') {
            if (*(cp+1)) {
                ++cp;
                c= *cp; // tolower(*cp);
                ++cp;
            }
            else continue;  /* we ignore a dash by itself */
        } /* was neg appended option */
        else {
            if (!strlen(infile)) {
                strcpy(infile,cp);
                continue;
            }
            else /* unknown option */ {
                error(USAGE_STR, argv[0]);
                return 1;
            }
        }

        if (c == '-')
        {
            // extended args
            char* args;
            CHARP_ARG(args);

            if(strncmp(args, "irq", strlen("irq")) == 0)
            {
                // found irq argument
                int pin;
                if(sscanf(args, "irq%d", &pin))
                {
                    mEM7189Config.PinSelection.bits.HostIRQ = pin;
                }
                else
                {
                    error("ERROR: Unable to determine irq pin selected from '%s'\n", args);
                    error(USAGE_STR, argv[0]);
                    return 1;
                }
            }
#if INTF_ANDROIDL || INTF_STREAMING
            else if(strncmp(args, "range", strlen("range")) == 0)
            {
                unsigned int range;
                if(!sscanf(args, "range%d", &range))
                {
                    error("ERROR: Unable to determine dynamic range selected from '%s'\n", args);
                    error(USAGE_STR, argv[0]);
                    return 1;
                }

                if(selectedSensor == 0)
                {
                    error("ERROR: no sensor selected\n");
                    error(USAGE_STR, argv[0]);
                    return 1;
                }

                selectedSensor->defaultDynamicRange = range;
                RANGE_SPECIFIED(selectedSensor);
            }
#endif
#if SENSORS_MIN_NOISE
            else if(strncmp(args, "noise", strlen("noise")) == 0)
            {
                float noise;
                if(!sscanf(args, "noise%f", &noise))
                {
                    error("ERROR: Unable to determine min noise selected from '%s'\n", args);
                    error(USAGE_STR, argv[0]);
                    return 1;
                }

                if(selectedSensor == 0)
                {
                    error("ERROR: no sensor selected\n");
                    error(USAGE_STR, argv[0]);
                    return 1;
                }

                selectedSensor->minNoiseFactor = noise;
                NOISE_SPECIFIED(selectedSensor);
            }
            else if(strncmp(args, "noise_mode", strlen("noise_mode")) == 0)
            {
                int mode;
                if(!sscanf(args, "noise_mode%d", &mode))
                {
                    error("ERROR: Unable to determine noise mode selected from '%s'\n", args);
                    error(USAGE_STR, argv[0]);
                    return 1;
                }

                if(selectedSensor == 0)
                {
                    error("ERROR: no sensor selected\n");
                    error(USAGE_STR, argv[0]);
                    return 1;
                }
                selectedSensor->noiseMode = mode;
                NOISE_MODE_SPECIFIED(selectedSensor);
            }
#endif
            else if(strncmp(args, "version", strlen("version")) == 0)
            {
                int version;
                if(sscanf(args, "version%d", &version))
                {
                    mEM7189Config.CustomVersion = version;
                }
                else
                {
                    error("ERROR: Unable to determine version number from '%s'\n", args);
                    error(USAGE_STR, argv[0]);
                    return 1;
                }
            }
            else if(strncmp(args, "fifo", strlen("fifo")) == 0)
            {
                // found fifo argument
                float split;
                if(sscanf(args, "fifo%f", &split))
                {
                    mEM7189Config.FIFOAllocationRatio = ((split - 50.0F) * 2.0F);
                }
                else
                {
                    error("ERROR: Unable to determine FIFO split selected from '%s'\n", args);
                    error(USAGE_STR, argv[0]);
                    return 1;
                }
            }
            else if(strncmp(args, "wordsreq", strlen("wordsreq")) == 0)
            {
                // found wordsreq argument
                int wordsreq;
                if(sscanf(args, "wordsreq%d", &wordsreq))
                {
                    mEM7189Config.FIFOWordsRequired = (unsigned short)wordsreq;
                }
                else
                {
                    error("ERROR: Unable to determine FIFO words required selected from '%s'\n", args);
                    error(USAGE_STR, argv[0]);
                    return 1;
                }
            }
            else if(strncmp(args, "pull", strlen("pull")) == 0)
            {
                // found pull argument
                int pin;
                if(sscanf(args, "pull%d%s", &pin, args))
                {
                    if(pin > NUM_GPIO_PULLS)
                    {
                        error("ERROR: Invalid pin %d \n", pin);
                        error(USAGE_STR, argv[0]);
                        return 1;
                    }

                    SInt8 selection = stringToPullselection(args);
                    if(selection < 0)
                    {
                        error("ERROR: Invalid pull selection '%s'\n", args);
                        error(USAGE_STR, argv[0]);
                        return 1;
                    }
                    else
                    {
                        mEM7189Config.PullSelection.PullSelection &= ~((UInt64)0x1 << (1 * (UInt64)pin));
                        mEM7189Config.PullSelection.PullSelection |= ((UInt64)selection << (1 * (UInt64)pin));
                    }
                }
                else
                {
                    error("ERROR: Unable to determine pull pin selected from '%s'\n", args);
                    error(USAGE_STR, argv[0]);
                    return 1;
                }
            }
            else if(strncmp(args, "gpio", strlen("gpio")) == 0)
            {
                // found pull argument
                int pin;
                if(sscanf(args, "gpio%d%s", &pin, args))
                {
                    if(pin > NUM_GPIO_PINS)
                    {
                        error("ERROR: Invalid pin %d \n", pin);
                        error(USAGE_STR, argv[0]);
                        return 1;
                    }

                    SInt8 selection = stringToPullselection(args);
                    if(selection < 0)
                    {
                        error("ERROR: Invalid gpio selection '%s'\n", args);
                        error(USAGE_STR, argv[0]);
                        return 1;
                    }
                    else
                    {
                        mEM7189Config.PinMode.PinMode &= ~((UInt64)0x3 << (2 * (UInt64)pin));
                        mEM7189Config.PinMode.PinMode |= ((UInt64)selection << (2 * (UInt64)pin));
                    }
                }
                else
                {
                    error("ERROR: Unable to determine gpio pin selected from '%s'\n", args);
                    error(USAGE_STR, argv[0]);
                    return 1;
                }
            }
            else if((strncmp(args, "noexec", strlen("noexec")) == 0) ||
                    (strncmp(args, "exec",    strlen("exec"))  == 0))
            {
                error("'%s' option ignored. No longer supported.\n", args);
                error(USAGE_STR, argv[0]);
                return 1;
            }
            else
            {
                error(USAGE_STR, argv[0]);
                return 1;
            }
        }
        else if (c == 'm')  selectedSensor = getDescriptorByType(elf, BSX_INPUT_ID_MAGNETICFIELD);
        else if (c == 'a')  selectedSensor = getDescriptorByType(elf, BSX_INPUT_ID_ACCELERATION);
        else if (c == 'g')  selectedSensor = getDescriptorByType(elf, BSX_INPUT_ID_ANGULARRATE);
        else if (isdigit(c))
        {
            // sensor descriptor id = c
               int numSensors =  get_section_size(elf, STR_PHYS_SENSOR_DESCRIPTOR) / PHYS_SENSOR_DESCRIPTOR_SIZE;
               if(c - '0' >= numSensors)
               {
                   error("Unable to locate driver in slot %d\n", c - '0');
                   exit(1);
               }
            selectedSensor = &(gPhysSensorDrivers[c - '0']);
        }
        else if (c == 'd')
        {
            unsigned int addr;
            UNSIGNED_ARG(addr);
            if(selectedSensor == 0)
            {
                error("ERROR: no sensor selected\n");
                error(USAGE_STR, argv[0]);
                return 1;
            }

            selectedSensor->device.options.i2c.address = addr;
            ADDR_SPECIFIED(selectedSensor);
        }
        else if (c == 'p')
        {
            int gpiodis = 0;
            unsigned int gpio;
            UNSIGNED_ARG(gpio);
            if(selectedSensor == 0)
            {
                error("ERROR: no sensor selected\n");
                error(USAGE_STR, argv[0]);
                return 1;
            }

            if(gpio > 24)
            {
                printf("WARNING: Disabling GPIO interrupt for sensor \n");
                gpiodis = 1; // also set disable bit for new firmware
            }

            selectedSensor->device.irqPin = gpio;
            selectedSensor->device.irqDis = gpiodis;
            GPIO_SPECIFIED(selectedSensor);
        }
        else if (c == 'c') {
            if(selectedSensor == 0)
            {
                error("ERROR: no sensor selected\n");
                error(USAGE_STR, argv[0]);
                return 1;
            }

            float tempfloat[9];
            if(sscanf(cp, "%f,%f,%f,%f,%f,%f,%f,%f,%f",
                &tempfloat[0], &tempfloat[1], &tempfloat[2],
                &tempfloat[3], &tempfloat[4], &tempfloat[5],
                &tempfloat[6], &tempfloat[7], &tempfloat[8]) != 9)
            {
                error("ERROR: expected number of offset values not found.\n");
                error(USAGE_STR, argv[0]);
                return 1;
            }
            for(i = 0; i < 9; i++)
            {
               selectedSensor->CalMatrix[i] = tempfloat[i];
            }
            CAL_SPECIFIED(selectedSensor);
        }
        else if (c == 'o') {
            if(selectedSensor == 0)
            {
                error("ERROR: no sensor selected\n");
                error(USAGE_STR, argv[0]);
                return 1;
            }

#if CONFIG_HAS_FLOAT_MATRIX == 0
            error("ERROR: cal offset is not supported with 2.0 firmware.\n");
            error(USAGE_STR, argv[0]);
            return 1;
#else /* CONFIG_HAS_FLOAT_MATRIX == 1 */
            if(sscanf(cp, "%f,%f,%f", &selectedSensor->CalOffset[0], &selectedSensor->CalOffset[1], &selectedSensor->CalOffset[2]) != 3)
            {
                error("ERROR: expected number of offset values not found.\n");
                error(USAGE_STR, argv[0]);
                return 1;
            }
            OFFSET_SPECIFIED(selectedSensor);
#endif
        }
        else if (c == 'q') {
            if (*cp) {
                error(USAGE_STR, argv[0]);
                return 1;
            }
            else quiet = 1;
        }
        else if (c == 'f') {
            if (!*cp) {
                error(USAGE_STR, argv[0]);
                return 1;
            }
            strcpy(configfile, cp);
            readConfig(configfile, elf);

            if(argc > 4) // FIXME. This does not properly check all arguments...
            {
                // this wasn't the only argument, force re-writing cfg file.
                stuffelfCfgVersion = -1;
            }
        }


        else /* unknown option */ {
            error(USAGE_STR, argv[0]);
            return 1;
        }
    } /* for each option in option count */

    if (!strlen(infile)) {
        // strcpy(infile, "a.out");
        error(USAGE_STR, argv[0]);
        return 1;
    }

    if(hasEM7189Config(elf))
    {
        // Check Phys
        int numSensors = get_section_size(elf, STR_PHYS_SENSOR_DESCRIPTOR) / PHYS_SENSOR_DESCRIPTOR_SIZE;
        int numVirtual = (get_section_size(elf, STR_VIRT_SENSOR_DESCRIPTOR) + get_section_size(elf, STR_TIME_SENSOR_DESCRIPTOR)) / VIRT_SENSOR_DESCRIPTOR_SIZE;

        if(mEM7189Config.DriverCount && mEM7189Config.DriverCount != numSensors)
        {
            error("ERROR: DriverCount (%d) in elf file does not match expected amount of drivers (%d).\n", mEM7189Config.DriverCount, numSensors);
            return 1;
        }
        else if(!mEM7189Config.DriverCount)
        {
            mEM7189Config.DriverCount = numSensors;
        }

        if(mEM7189Config.VirtualCount && mEM7189Config.VirtualCount != numVirtual)
        {
            error("ERROR: VirtualCount (%d) in elf file does not match expected amount of drivers (%d).\n", mEM7189Config.VirtualCount, numVirtual);
            mEM7189Config.VirtualCount = numVirtual;
        }
        else if(!mEM7189Config.VirtualCount)
        {
            mEM7189Config.VirtualCount = numVirtual;
        }
    }

    if(hasEM7189Config(elf))
    {
        UInt32 HostIRQ = mEM7189Config.PinSelection.bits.HostIRQ;
        UInt32 EvCfg = mEM7189Config.PinSelection.bits.EvCfg;
        verbose("\n=== EM7189 Config Version %d ===\n", mEM7189Config.Version);
        verbose("CustomVersion:         %d\n", mEM7189Config.CustomVersion);
        verbose("RamVersion:            %d\n", mEM7189Config.RamVersion);
        verbose("Flags:                 0x%X\n", mEM7189Config.Flags);
        verbose("    Turbo Mode:        %s\n", (mEM7189Config.Flags & FLAGS_USE_TURBO_MODE) ? "enabled" : "disabled");
        verbose("HostGPIO:              %d\n", HostIRQ);
        verbose("EvCfg:                 0x%X\n", EvCfg);
        verbose("Host Interface:        %s\n", mEM7189Config.PinSelection.bits.HIFDisable ? "disabled" : "enabled");
        verbose("Physical Drivers:      %d\n", mEM7189Config.DriverCount);
        verbose("Virtual Drivers:       %d\n", mEM7189Config.VirtualCount);
        verbose("SIF Selection:         %s\n", SIFModeToName(mEM7189Config.SIFSelection));
        for(i = 0; i < NUM_GPIO_PULLS; i++)
        {
            const char* gpioPull  = pullSelectionToName(mEM7189Config.PullSelection.PullSelection >> (i * 1));
            const char* gpioValue;
            if(i < NUM_GPIO_PINS)
            {
                gpioValue = GPIOSelectionToName(mEM7189Config.PinMode.PinMode >> (i * 2));
            }
            else
            {
                gpioValue = "N/A";
            }
            char* name = GPIOToName(i);
            verbose("%10s: %4s, pull %3s\n", name, gpioValue, gpioPull);
            free(name);
        }
        verbose("FIFO Wakeup:           %0.2f%%\n", ((float)mEM7189Config.FIFOAllocationRatio/2.0F) + 50.0F);
        verbose("FIFO Non-Wakeup:       %0.2f%%\n", 50.0F - ((float)mEM7189Config.FIFOAllocationRatio/2.0F));
        verbose("FIFO Words Required:   %u\n", mEM7189Config.FIFOWordsRequired);
        verbose("\n");

        // Update file

        if(!set_section_data(elf, STR_EM7189_DESCRIPTOR, (void*)&mEM7189Config, sizeof(EM7189Config)))
        {
            error("Unable to update '%s' section.\n", STR_EM7189_DESCRIPTOR);
            exit(-1);
        }

    }

    stuffSensors(elf);

    if(!SaveElf(elf, infile))
    {
        printf("Unable to save to file %s\n", infile);
        exit(-1);
    }

    if(hasEM7189Config(elf))
    {
        uint64_t _optionalcodebanks = get_symbol_addr(elf, "_optionalcodebanks");
        uint64_t _optionaldatabanks = get_symbol_addr(elf, "_optionaldatabanks");
        uint64_t max_ram_banks = get_symbol_addr(elf, "_OPTIONAL_RAM_BANKS");
        int      additional_banks = 0;

        uint64_t _ffreedataram = get_symbol_addr(elf, "_ffreedataram");
        uint64_t _ffreecoderam = get_symbol_addr(elf, "_ffreecoderam");
        uint64_t _efreecoderam = get_symbol_addr(elf, "_efreecoderam");
        uint64_t _freclaim = get_section_addr(elf, ".reclaim");

        uint64_t initdat_addr = get_section_addr(elf, STR_INITDAT);
        uint64_t initdat_size = get_section_size(elf, STR_INITDAT);

        uint64_t user_fw_end = initdat_addr + initdat_size;
        uint64_t kernel_fw_start = get_symbol_addr(elf, "KERNEL_FW_START");
        uint64_t fw_size = user_fw_end - kernel_fw_start;
        uint64_t max_fw_size = 0;
        uint64_t use_flash_bootloader = 0;

        // Check for max firmware upload size
        if (kernel_fw_start >= FLASH_START)
        {
            // Check flash upload size compared to available FLASH max firmware size
            max_fw_size = get_symbol_addr(elf, "MAX_FLASH_FW_SIZE");
            if (fw_size > max_fw_size)
            {
                use_flash_bootloader = get_symbol_addr(elf, "USE_FLASH_BOOTLOADER");
                if (use_flash_bootloader)
                {
                    printf("Firmware size (0x%" PRIx64 ") exceed maximum FLASH bootloader image size (0x%" PRIx64 ")\n", fw_size, max_fw_size);
                    exit(-1);
                }
                else
                {
                    printf("Firmware size (0x%" PRIx64 ") exceed maximum FLASH image size (0x%" PRIx64 ")\n", fw_size, max_fw_size);
                    exit(-1);
                }
            }
        }
        else
        {
            // Check RAM upload size compared to available RAM code banks
            max_fw_size = INITIAL_BANK_SIZE + ((max_ram_banks - _optionaldatabanks) * BANK_SIZE);
            if (fw_size > max_fw_size)
            {
                printf("Firmware size (0x%" PRIx64 ") exceed maximum RAM image size (0x%" PRIx64 ")\n", fw_size, max_fw_size);
                exit(-1);
            }
        }

        // For ram firmware, use _freclaim.
        // For flash firmware, use _ffreecoderam.
        _ffreecoderam = MIN(_ffreecoderam, _freclaim);

        if ((_ffreedataram == INVALID_ADDRESS) || (_ffreecoderam == INVALID_ADDRESS))
        {
            printf("Error determining free ram for FIFO\n");
        }
        else
        {
            if (_freclaim < FLASH_START) /* If ram firmware */
            {
                // firmware is set to reclaim initdata space.
                printf("Reclaiming .initdata: %" PRIu64 " bytes\n", initdat_size);
                printf("Reclaiming .reclaim: %" PRIu64 " bytes\n", get_section_size(elf, ".reclaim"));
            }

            uint64_t _efreedataram;
            // Determine amount of RAM banks enabled.
            // First data ram bank is located at 0xa04000
            // First code ram bank is located at 0x124000
            _efreedataram = _ffreedataram - DATA_RAM_START;

            // First ram bank is 16KB in size, otherwise it's 32KB each.
            if(_efreedataram <  INITIAL_BANK_SIZE)
            {
                _efreedataram = INITIAL_BANK_SIZE;
            }
            else
            {
                // Determine end of last bank.
                _efreedataram -= INITIAL_BANK_SIZE;
                uint32_t remainder = _efreedataram % BANK_SIZE;
                if(remainder)
                {
                    // Round to next bank.
                    _efreedataram += BANK_SIZE - remainder;
                }
                _efreedataram += INITIAL_BANK_SIZE;
            }

            _efreedataram += DATA_RAM_START;

            printf("Free code ram for FIFO: 0x%08" PRIx64 " to 0x%08" PRIx64 " (%" PRIu64 " bytes)\n", _ffreecoderam, _efreecoderam, (_efreecoderam - _ffreecoderam));
            printf("Free data ram for FIFO: 0x%08" PRIx64 " to 0x%08" PRIx64 " (%" PRIu64 " bytes)\n", _ffreedataram, _efreedataram, (_efreedataram - _ffreedataram));
            uint32_t fifoSize = (_efreedataram - _ffreedataram) + (_efreecoderam - _ffreecoderam);
            while(fifoSize < (mEM7189Config.FIFOWordsRequired * sizeof(UInt32)))
            {
                additional_banks++;
                fifoSize += BANK_SIZE; // add additional ram bank.
                _efreedataram += BANK_SIZE;
            }
            if(additional_banks)
            {
                printf("FIFO Words Required:    %u, %d additional bank(s) enabled.\n", mEM7189Config.FIFOWordsRequired, additional_banks);
            }

            printf("Optional RAM bank usage: Code banks=%" PRIu64 ", Data banks=%" PRIu64 "\n", _optionalcodebanks, _optionaldatabanks + additional_banks);

            // Check/report ram bank information
            uint64_t num_banks_needed = _optionalcodebanks + _optionaldatabanks + additional_banks;
            if(num_banks_needed > max_ram_banks)
            {
                error("Error: RAM usage (%" PRIu64 " banks) exceeds available RAM banks (%" PRIu64 ")\n", num_banks_needed, max_ram_banks);
                exit(-1);
            }

            uint32_t actualFifoSize = fifoSize;
            actualFifoSize -= CMD_BUFFER_SIZE + 4u;             // Remove cmd buffer for the host interface
            actualFifoSize -= NUM_FIFOS * FIFO_FIXED_OVERHEAD;  // Remove Fifo overhead for the wake/nonwake FIFOs.
            actualFifoSize -= FIFO_STATUS_SIZE;                 // Remove status FIFO size

            printf("Free ram for FIFO:        %d\n", actualFifoSize);

            int blocks_available = actualFifoSize / FIFO_BLOCK_SIZE;
            int blocks_needed = (NUM_FIFOS * MIN_FIFO_BLOCKS);
            if(blocks_available < blocks_needed)
            {
                error("Not enough memory for all FIFOs. Estimated %d blocks available, but %d are needed.\n", blocks_available, blocks_needed);
            }
            else
            {
                // Calculate FIFO split
                int fraction = mEM7189Config.FIFOAllocationRatio;
                fraction = MAX(fraction, -100);
                fraction = MIN(fraction, 100);
                UInt32 diff = (UInt32)((UInt8)((SInt8)100 - fraction));



                // Firmware calculates Non-Wake first.
                int nonwake = ((actualFifoSize * diff) / 200u) & ~0x03u;
                int nonwake_blocks = nonwake / (FIFO_BLOCK_SIZE);
                nonwake_blocks = MAX(MIN_FIFO_BLOCKS, nonwake_blocks);
                // Determine size available for wake FIFO.
                int wake = actualFifoSize - (nonwake_blocks * FIFO_BLOCK_SIZE);
                int wake_blocks = wake / FIFO_BLOCK_SIZE;
                if(wake_blocks < MIN_FIFO_BLOCKS)
                {
                    // Take the blocks from the nonwake FIFO.
                    int delta = MIN_FIFO_BLOCKS - wake_blocks;
                    wake_blocks += delta;
                    nonwake_blocks -= delta;
                }

                // Actual visible FIFO sizes
                wake = wake_blocks * FIFO_BLOCK_SIZE;
                nonwake = nonwake_blocks * FIFO_BLOCK_SIZE;

                printf("Estimated Wake FIFO:      %d\n", wake);
                printf("Estimated Non-Wake FIFO:  %d\n", nonwake);
            }
        }
    }

    if(stuffelfCfgVersion != STUFFELF_CONFIG_VERSION)
    {
        num_errors += !writeConfig(configfile, elf);
    }

    CloseElf(elf);

    return num_errors;
}  /*  main */


const char* driverTypeToName(UInt8 type)
{
    switch(type)
    {
        case DRIVER_TYPE_VIRTUAL_FLAG:  return "virtual";
        case DRIVER_TYPE_TIMER_FLAG:    return "timer";
        case DRIVER_TYPE_PHYSICAL_FLAG: return "physical";
        default: return "unknown";
    }
}

const char* typeToPhysicalName(UInt8 type)
{
    switch (type)
    {
        case BSX_INPUT_ID_ACCELERATION: return "accel";
        case BSX_INPUT_ID_ANGULARRATE: return "gyro";
        case BSX_INPUT_ID_MAGNETICFIELD: return "mag";
        case BSX_INPUT_ID_TEMPERATURE_GYROSCOPE: return "gyro temp";
        case BSX_INPUT_ID_ANYMOTION: return "any motion";
        case BSX_INPUT_ID_PRESSURE: return "pressure";
        case BSX_INPUT_ID_POSITION: return "postion";
        case BSX_INPUT_ID_HUMIDITY: return "humidity";
        case BSX_INPUT_ID_TEMPERATURE: return "temp";
        case BSX_INPUT_ID_GASRESISTOR: return "gas";
        case SENSOR_TYPE_INPUT_STEP_COUNTER: return "stepcnt";
        case SENSOR_TYPE_INPUT_STEP_DETECTOR: return "stepdet";
        case SENSOR_TYPE_INPUT_SIGNIFICANT_MOTION: return "sigmot";
        case SENSOR_TYPE_INPUT_ANY_MOTION: return "anymot";
        case SENSOR_TYPE_INPUT_EXCAMERA: return "excamera";
        case SENSOR_TYPE_INPUT_GPS: return "gps";
        case SENSOR_TYPE_INPUT_LIGHT: return "light";
        case SENSOR_TYPE_INPUT_PROXIMITY: return "proximity";

        default: return "unknown";
    }
}

const char* typeToName(UInt8 type)
{
    switch(type)
    {
        case SENSOR_TYPE_BSX(BSX_OUTPUT_ID_ACCELERATION_PASSTHROUGH): return "accel passthrough";
        case SENSOR_TYPE_BSX(BSX_CUSTOM_ID_ACCELERATION_PASSTHROUGH): return "custom accel passthrough";
        case SENSOR_TYPE_BSX(BSX_OUTPUT_ID_ACCELERATION_RAW): return "accel raw";
        case SENSOR_TYPE_BSX(BSX_OUTPUT_ID_ACCELERATION_CORRECTED): return "accel corr";
        case SENSOR_TYPE_BSX(BSX_OUTPUT_ID_ACCELERATION_OFFSET): return "accel offset";
        case SENSOR_TYPE_BSX(BSX_WAKEUP_ID_ACCELERATION_OFFSET): return "wakeup accel offset";
        case SENSOR_TYPE_BSX(BSX_WAKEUP_ID_ACCELERATION_CORRECTED): return "wakeup accel corr";
        case SENSOR_TYPE_BSX(BSX_WAKEUP_ID_ACCELERATION_RAW): return "wakeup accel raw";
        case SENSOR_TYPE_BSX(BSX_CUSTOM_ID_ACCELERATION_CORRECTED): return "custom accel corr";
        case SENSOR_TYPE_BSX(BSX_CUSTOM_ID_ACCELERATION_RAW): return "custom accel raw";
        case SENSOR_TYPE_BSX(BSX_OUTPUT_ID_ANGULARRATE_PASSTHROUGH): return "gyro passthrough";
        case SENSOR_TYPE_BSX(BSX_CUSTOM_ID_ANGULARRATE_PASSTHROUGH): return "custom gyro passthrough";
        case SENSOR_TYPE_BSX(BSX_OUTPUT_ID_ANGULARRATE_RAW): return "gyro raw";
        case SENSOR_TYPE_BSX(BSX_OUTPUT_ID_ANGULARRATE_CORRECTED): return "gyro corr";
        case SENSOR_TYPE_BSX(BSX_OUTPUT_ID_ANGULARRATE_OFFSET): return "gyro offset";
        case SENSOR_TYPE_BSX(BSX_WAKEUP_ID_ANGULARRATE_OFFSET): return "wakeup gyro offset";
        case SENSOR_TYPE_BSX(BSX_WAKEUP_ID_ANGULARRATE_CORRECTED): return "wakeup gyro corr";
        case SENSOR_TYPE_BSX(BSX_WAKEUP_ID_ANGULARRATE_RAW): return "wakeup gyro raw";
        case SENSOR_TYPE_BSX(BSX_CUSTOM_ID_ANGULARRATE_CORRECTED): return "custom gyro corr";
        case SENSOR_TYPE_BSX(BSX_CUSTOM_ID_ANGULARRATE_RAW): return "custom gyro raw";
        case SENSOR_TYPE_BSX(BSX_OUTPUT_ID_MAGNETICFIELD_PASSTHROUGH): return "mag passthrough";
        case SENSOR_TYPE_BSX(BSX_CUSTOM_ID_MAGNETICFIELD_PASSTHROUGH): return "custom mag passthrough";
        case SENSOR_TYPE_BSX(BSX_OUTPUT_ID_MAGNETICFIELD_RAW): return "mag raw";
        case SENSOR_TYPE_BSX(BSX_OUTPUT_ID_MAGNETICFIELD_CORRECTED): return "mag corr";
        case SENSOR_TYPE_BSX(BSX_OUTPUT_ID_MAGNETICFIELD_OFFSET): return "mag offset";
        case SENSOR_TYPE_BSX(BSX_WAKEUP_ID_MAGNETICFIELD_OFFSET): return "wakeup mag offset";
        case SENSOR_TYPE_BSX(BSX_WAKEUP_ID_MAGNETICFIELD_CORRECTED): return "wakeup mag corr";
        case SENSOR_TYPE_BSX(BSX_WAKEUP_ID_MAGNETICFIELD_RAW): return "wakeup mag raw";
        case SENSOR_TYPE_BSX(BSX_CUSTOM_ID_MAGNETICFIELD_CORRECTED): return "custom mag corr";
        case SENSOR_TYPE_BSX(BSX_CUSTOM_ID_MAGNETICFIELD_RAW): return "custom mag raw";
        case SENSOR_TYPE_BSX(BSX_OUTPUT_ID_GRAVITY): return "grav";
        case SENSOR_TYPE_BSX(BSX_WAKEUP_ID_GRAVITY): return "wakeup grav";
        case SENSOR_TYPE_BSX(BSX_CUSTOM_ID_GRAVITY): return "custom grav";
        case SENSOR_TYPE_BSX(BSX_OUTPUT_ID_LINEARACCELERATION): return "linaccel";
        case SENSOR_TYPE_BSX(BSX_WAKEUP_ID_LINEARACCELERATION): return "wakeup linaccel";
        case SENSOR_TYPE_BSX(BSX_CUSTOM_ID_LINEARACCELERATION): return "custom linaccel";
        case SENSOR_TYPE_BSX(BSX_OUTPUT_ID_ROTATION): return "rotvec";
        case SENSOR_TYPE_BSX(BSX_WAKEUP_ID_ROTATION): return "wakeup rotvec";
        case SENSOR_TYPE_BSX(BSX_CUSTOM_ID_ROTATION): return "custom rotvec";
        case SENSOR_TYPE_BSX(BSX_OUTPUT_ID_ROTATION_GAME): return "game rotvec";
        case SENSOR_TYPE_BSX(BSX_WAKEUP_ID_ROTATION_GAME): return "wakeup game rotvec";
        case SENSOR_TYPE_BSX(BSX_CUSTOM_ID_ROTATION_GAME): return "custom game rotvec";
        case SENSOR_TYPE_BSX(BSX_OUTPUT_ID_ROTATION_GEOMAGNETIC): return "mag rotvec";
        case SENSOR_TYPE_BSX(BSX_WAKEUP_ID_ROTATION_GEOMAGNETIC): return "wakeup mag rotvec";
        case SENSOR_TYPE_BSX(BSX_CUSTOM_ID_ROTATION_GEOMAGNETIC): return "custom mag rotvec";
        case SENSOR_TYPE_BSX(BSX_OUTPUT_ID_ORIENTATION): return "orient";
        case SENSOR_TYPE_BSX(BSX_WAKEUP_ID_ORIENTATION): return "wakeup orient";
        case SENSOR_TYPE_BSX(BSX_CUSTOM_ID_ORIENTATION): return "custom orient";
        case SENSOR_TYPE_BSX(BSX_OUTPUT_ID_FLIP_STATUS): return "flip";
        case SENSOR_TYPE_BSX(BSX_CUSTOM_ID_FLIP_STATUS): return "custom flip";
        case SENSOR_TYPE_BSX(BSX_OUTPUT_ID_TILT_STATUS): return "tilt";
        case SENSOR_TYPE_BSX(BSX_CUSTOM_ID_TILT_STATUS): return "custom tilt";
        case SENSOR_TYPE_BSX(BSX_OUTPUT_ID_STEPDETECTOR): return "step det";
        case SENSOR_TYPE_BSX(BSX_WAKEUP_ID_STEPDETECTOR): return "wake step det";
        case SENSOR_TYPE_BSX(BSX_CUSTOM_ID_STEPDETECTOR): return "custom step det";
        case SENSOR_TYPE_BSX(BSX_OUTPUT_ID_STEPCOUNTER): return "step count";
        case SENSOR_TYPE_BSX(BSX_WAKEUP_ID_STEPCOUNTER): return "wake step count";
        case SENSOR_TYPE_BSX(BSX_CUSTOM_ID_STEPCOUNTER): return "custom step count";
        case SENSOR_TYPE_BSX(BSX_OUTPUT_ID_SIGNIFICANTMOTION_STATUS): return "sigmo";
        case SENSOR_TYPE_BSX(BSX_CUSTOM_ID_SIGNIFICANTMOTION_STATUS): return "custom sigmo";
        case SENSOR_TYPE_BSX(BSX_OUTPUT_ID_WAKE_STATUS): return "wake status";
        case SENSOR_TYPE_BSX(BSX_CUSTOM_ID_WAKE_STATUS): return "custom wake state";
        case SENSOR_TYPE_BSX(BSX_OUTPUT_ID_GLANCE_STATUS): return "glance status";
        case SENSOR_TYPE_BSX(BSX_CUSTOM_ID_GLANCE_STATUS): return "custom glance status";
        case SENSOR_TYPE_BSX(BSX_OUTPUT_ID_PICKUP_STATUS): return "pickup status";
        case SENSOR_TYPE_BSX(BSX_CUSTOM_ID_PICKUP_STATUS): return "custom pickup status";
        case SENSOR_TYPE_BSX(BSX_OUTPUT_ID_ACTIVITY): return "activity";
        case SENSOR_TYPE_BSX(BSX_CUSTOM_ID_ACTIVITY): return "custom activity";
        case SENSOR_TYPE_BSX(BSX_OUTPUT_ID_PROPAGATION): return "prop";
        case SENSOR_TYPE_BSX(BSX_OUTPUT_ID_POSITION_STEPS): return "position steps";
        case SENSOR_TYPE_BSX(BSX_OUTPUT_ID_WRIST_TILT_STATUS): return "wrist tilt";
        case SENSOR_TYPE_BSX(BSX_CUSTOM_ID_WRIST_TILT_STATUS): return "custom wrist tilt";
        case SENSOR_TYPE_BSX(BSX_OUTPUT_ID_DEVICE_ORIENTATION): return "dev orient";
        case SENSOR_TYPE_BSX(BSX_WAKEUP_ID_DEVICE_ORIENTATION): return "wakeup dev orient";
        case SENSOR_TYPE_BSX(BSX_CUSTOM_ID_DEVICE_ORIENTATION): return "custom dev orient";
        case SENSOR_TYPE_BSX(BSX_OUTPUT_ID_POSE_6DOF): return "pose 6d0f";
        case SENSOR_TYPE_BSX(BSX_WAKEUP_ID_POSE_6DOF): return "wakeup pose 6d0f";
        case SENSOR_TYPE_BSX(BSX_CUSTOM_ID_POSE_6DOF): return "custom pose 6d0f";
        case SENSOR_TYPE_BSX(BSX_OUTPUT_ID_STATIONARY_DETECT): return "stationary";
        case SENSOR_TYPE_BSX(BSX_CUSTOM_ID_STATIONARY_DETECT): return "custom stationary";
        case SENSOR_TYPE_BSX(BSX_OUTPUT_ID_MOTION_DETECT): return "motion";
        case SENSOR_TYPE_BSX(BSX_CUSTOM_ID_MOTION_DETECT): return "custom motion";
        case SENSOR_TYPE_BSX(BSX_OUTPUT_ID_STANDBY_STATUS): return "standby";
        case SENSOR_TYPE_BSX(BSX_OUTPUT_ID_ACCELERATION_STATUS): return "accel status";
        case SENSOR_TYPE_BSX(BSX_OUTPUT_ID_ACCELERATION_DYNAMIC): return "accel dynamic";
        case SENSOR_TYPE_BSX(BSX_OUTPUT_ID_ANGULARRATE_STATUS): return "gyro status";
        case SENSOR_TYPE_BSX(BSX_OUTPUT_ID_MAGNETICFIELD_STATUS): return "mag status";

        case SENSOR_TYPE_TEMPERATURE: return "temperature";
        case SENSOR_TYPE_PRESSURE: return "pressure";
        case SENSOR_TYPE_HUMIDITY: return "humidity";
        case SENSOR_TYPE_GAS: return "gas";
        case SENSOR_TYPE_WAKE_TEMPERATURE: return "wakeup temperature";
        case SENSOR_TYPE_WAKE_PRESSURE: return "wakeup pressure";
        case SENSOR_TYPE_WAKE_HUMIDITY: return "wakeup humidity";
        case SENSOR_TYPE_WAKE_GAS: return "wakeup gas";
        case SENSOR_TYPE_STEP_COUNTER: return "hw stepcnt";
        case SENSOR_TYPE_STEP_DETECTOR: return "hw stepdet";
        case SENSOR_TYPE_SIGNIFICANT_MOTION: return "hw sigmot";
        case SENSOR_TYPE_WAKE_STEP_COUNTER: return "wakeup hw stepcnt";
        case SENSOR_TYPE_WAKE_STEP_DETECTOR: return "wakeup hw stepdet";
        case SENSOR_TYPE_WAKE_SIGNIFICANT_MOTION: return "wakeup hw sigmot";
        case SENSOR_TYPE_ANY_MOTION: return "hw anymot";
        case SENSOR_TYPE_WAKE_ANY_MOTION: return "wakeup hw anymot";
        case SENSOR_TYPE_VIRT_EXCAMERA: return "excamera";
        case SENSOR_TYPE_GPS: return "gps";
        case SENSOR_TYPE_LIGHT: return "light";
        case SENSOR_TYPE_PROXIMITY: return "proximity";
        case SENSOR_TYPE_WAKE_LIGHT: return "wakeup light";
        case SENSOR_TYPE_WAKE_PROXIMITY: return "wakeup prox";
        case SENSOR_TYPE_VIRT_BSX: return "BSX";
        case SENSOR_TYPE_HANG_DETECTOR: return "hang detector";

        default:
        {
            if(type & SENSOR_TYPE_CUSTOMER_BEGIN) return "custom";
            return "unknown";
        }
    }
}

SInt8 stringToPullselection(const char* string)
{
    if(strcmp(string, "off") == 0) return GPIO_PULL_DISABLED;
    if(strcmp(string, "on") == 0) return GPIO_PULL_ENABLED;

    /* Legacy Names */
    if(strcmp(string, "none") == 0) return GPIO_PULL_DISABLED;
    if(strcmp(string, "down") == 0) return GPIO_PULL_ENABLED;
    if(strcmp(string, "up") == 0) return GPIO_PULL_ENABLED;
    if(strcmp(string, "default") == 0) return GPIO_PULL_ENABLED; // default to pulls enabled
    return -1;
}

SInt8 stringToGPIOselection(const char* string)
{
    if(strcmp(string, "hiz") == 0) return GPIO_MODE_HIZ;
    if(strcmp(string, "in") == 0) return GPIO_MODE_INPUT;

    if(strcmp(string, "0") == 0) return GPIO_MODE_OUTPUT_0;
    if(strcmp(string, "low") == 0) return GPIO_MODE_OUTPUT_0;

    if(strcmp(string, "1") == 0) return GPIO_MODE_OUTPUT_1;
    if(strcmp(string, "high") == 0) return GPIO_MODE_OUTPUT_1;
    return -1;
}


#if SENSORS_MIN_NOISE
SInt8 stringToNoiseMode(const char* string)
{
    if(strcmp(string, "avg") == 0)     return NOISE_MODE_AVERAGE;
    if(strcmp(string, "average") == 0) return NOISE_MODE_AVERAGE;
    if(strcmp(string, "meas") == 0)    return NOISE_MODE_MEASURE;
    if(strcmp(string, "measure") == 0) return NOISE_MODE_MEASURE;
    return -1;
}

const char* noiseModeToString(SInt8 mode)
{
    switch(mode)
    {
        default:
        case NOISE_MODE_AVERAGE:    return "average";
        case NOISE_MODE_MEASURE:    return "measure";
    }
}
#endif

static bool interfaceAllowed(UInt8 sif_mode, UInt8 interface)
{
    switch (interface)
    {
        case InterfaceI2C0:
            // Allowed when mode is 1, 2, or 3.
            return (sif_mode > 0);

        case InterfaceI2C1:
            // Allowed always
            return TRUE;

        case InterfaceSPI0:
            // Allowed when mode is 0 or 1.
            return (sif_mode <= 1);

        case InterfaceSPI1:
            // Allowed when mode is 0, 1, or 3.
            return (sif_mode != 2);

        case InterfaceNone:
            return TRUE;
    }

    error("Unknown interface 0x%02x\n", interface);
    return FALSE;
}

const char* SIFModeToName(UInt8 mode)
{
    switch(mode)
    {
        case 0: return "SIF1: spi0. SIF2: spi1. SIF3: i2c1";
        case 1: return "SIF1: spi0. SIF2: i2c0. SIF3: i2c1";
        case 2: return "SIF1: i2c0. SIF2: spi1. SIF3: i2c1";
    }
    return "unknown";
}
const char* pullSelectionToName(UInt8 pullSelection)
{
    pullSelection &= 0x1; // 2 bits only
    switch(pullSelection)
    {
        case GPIO_PULL_DISABLED:            return "off";
        case GPIO_PULL_ENABLED:             return "on";
    }
    return "(null)";
}

const char* GPIOSelectionToName(UInt8 gpioSelection)
{
    gpioSelection &= 0x3; // 2 bits only
    switch(gpioSelection)
    {
        case GPIO_MODE_HIZ:         return "hiz";
        case GPIO_MODE_INPUT:       return "in";
        case GPIO_MODE_OUTPUT_0:    return "low";
        case GPIO_MODE_OUTPUT_1:    return "high";
    }
    return "(null)";
}

char* GPIOToName(UInt8 pin)
{

    char* retstring;
    int numBytes;
    if(pin < NUM_GPIO_PINS)
    {
        numBytes = strlen("MFPAD[%d]") + 1;
        retstring = malloc(numBytes);
        snprintf(retstring, numBytes, "MFPAD[%d]", pin);
    }
    else
    {
        switch(pin)
        {
            case GPIO_SIF1CLK_PULL: retstring = strdup("sif1clk"); break;
            case GPIO_SIF1DIN_PULL: retstring = strdup("sif1din"); break;
            case GPIO_SIF1DIO_PULL: retstring = strdup("sif1dio"); break;
            default: retstring = strdup("unknown"); break;
        }
    }

    return retstring;
}

UInt8 typeToAxis(UInt8 type)
{
    switch(type)
    {
        case BSX_INPUT_ID_MAGNETICFIELD:    return 3;
        case BSX_INPUT_ID_ACCELERATION:     return 3;
        case BSX_INPUT_ID_ANGULARRATE:      return 3;


        default:
        {
            if(type & SENSOR_TYPE_CUSTOMER_BEGIN) return 1;
            return 1;
        }
    }
}

void printSensor(PhysicalSensorDescriptor *sensor, int i)
{
    printf("\n=== Sensor %d (0x%02X: %s) ===\n", i, sensor->type.value, typeToPhysicalName(sensor->type.value));
    printf("[%d] Driver ID:            0x%.2X\n", i, sensor->info.id);
    printf("[%d] Driver Revision:      0x%.2X\n", i, sensor->info.version);

    printf("[%d] Driver Interface:     %s\n", i, interfaceToName(sensor->device.interface.type));
    if((int)sensor->device.interface.type & (int)InterfaceAnyI2C)
    {
        printf("[%d] Sensor Address:       0x%.2X\n", i, sensor->device.options.i2c.address);
        printf("[%d] Sensor I2C Max:       %dkHz\n", i, sensor->device.options.i2c.maxClock);
    }
    else if((int)sensor->device.interface.type & (int)InterfaceAnySPI)
    {
        printf("[%d] Sensor Chip Select:   %d\n", i, sensor->device.options.spi.csPin);
        printf("[%d] Sensor SPI Max:       %dkHz\n", i, sensor->device.options.spi.maxClock);
    }
    if(sensor->device.irqDis)
    {
        printf("[%d] Sensor GPIO:          DISABLED\n", i);
    }
    else
    {
        printf("[%d] Sensor GPIO:          %d\n", i, sensor->device.irqPin);
        printf("[%d] Sensor GPIO Edge:     %s\n", i, (sensor->device.irqEdge == GPIO_RISING) ? "rising" : "falling");
    }
    printf("[%d] Sensor Max Rate:      %fHz\n", i, sensor->maxRate);
    printf("[%d] Sensor Max Current:   %0.2fmA\n", i, sensor->maxCurrent);
    printf("[%d] Sensor Max Range:     %d\n", i, sensor->maxDynamicRange);
#if INTF_ANDROIDL || INTF_STREAMING
    printf("[%d] Sensor Default Range: %d\n", i, sensor->defaultDynamicRange);
#endif
#if SENSORS_MIN_NOISE
    printf("[%d] Sensor min noise:     %f\n", i, sensor->minNoiseFactor);
    printf("[%d] Sensor noise mode:    %s\n", i, noiseModeToString(sensor->noiseMode));
#endif
    printf("[%d] Sensor Resolution:    %d\n", i, sensor->info.resolution);
    printf("[%d] Sensor Axis:          %d\n", i, sensor->numAxis);
    if(sensor->numAxis == 3)
    {
#if CONFIG_HAS_FLOAT_MATRIX == 0
        printf("[%d] Sensor Cal:          % d   % d   % d\n", i, sensor->CalMatrix[0], sensor->CalMatrix[1], sensor->CalMatrix[2]);
        printf("                         % d   % d   % d\n", sensor->CalMatrix[3], sensor->CalMatrix[4], sensor->CalMatrix[5]);
        printf("                         % d   % d   % d\n", sensor->CalMatrix[6], sensor->CalMatrix[7], sensor->CalMatrix[8]);
#else /* CONFIG_HAS_FLOAT_MATRIX == 1 */
        printf("[%d] Sensor Cal:          % f   % f   % f\n", i, sensor->CalMatrix[0], sensor->CalMatrix[1], sensor->CalMatrix[2]);
        printf("                         % f   % f   % f\n", sensor->CalMatrix[3], sensor->CalMatrix[4], sensor->CalMatrix[5]);
        printf("                         % f   % f   % f\n", sensor->CalMatrix[6], sensor->CalMatrix[7], sensor->CalMatrix[8]);
        printf("[%d] Sensor Offset:       % f   % f   % f\n", i, sensor->CalOffset[0], sensor->CalOffset[1], sensor->CalOffset[2]);
#endif
    }
#if 0

    printf("[%d] Sensor Offset   % f   % f   % f\n", i, sensor->CalOffset[0], sensor->CalOffset[1], sensor->CalOffset[2]);
#endif
}
void printVirtSensor(VirtualSensorDescriptor *sensor, int i)
{
    printf("\n=== Sensor %d (0x%02X: %s) ===\n", i, sensor->type.value, typeToName(sensor->type.value));
    printf("[%d] Driver ID:            0x%.2X\n", i, sensor->info.id);
    printf("[%d] Driver Revision:      0x%.2X\n", i, sensor->info.version);
#if defined(SENSOR_TEMPERATURE)
    printf("[%d] Temperature source:   %s\n", i, sensor->temperatureSource.sensor->type.value ? typeToName(sensor->temperatureSource.sensor->type.value) : "none");
#endif
    printf("[%d] Output size:          %d bytes.\n", i, sensor->outputPacketSize);
    printf("[%d] Max Rate:             %f\n", i, sensor->maxRate);
    printf("[%d] Min Rate:             %f\n", i, sensor->minRate);
    printf("[%d] hidden:               %s\n", i, sensor->type.hidden ? "YES" : "no");
    printf("[%d] wakeup:               %s\n", i, sensor->type.wakeup_ap ? "YES" : "no");
    printf("[%d] on change:            %s\n", i, sensor->type.on_change ? "YES" : "no");
    printf("[%d] Init function:        %p\n", i, (void*)sensor->initialize);
    printf("[%d] Data function:        %p\n", i, (void*)sensor->handle_sensor_data);
    printf("[%d] Priority:             %c\n", i, getPriorityLevel(cast_VirtualToHeader(sensor)) == PRIORITY_M ? 'M' : '0' + getPriorityLevel(cast_VirtualToHeader(sensor)));
    if(sensor->type.flags == DRIVER_TYPE_TIMER_FLAG)
    {
        printf("[%d] Trigger source:       Timer (%d Hz)\n", i, sensor->triggerSource.timer.rate);
    }
    else
    {
        SensorDescriptorHeader* parent = gTriggerSourceEntries[sensor->triggerSource.header];
        if(!parent)
        {
            printf("[%d] Trigger source:       Unknown\n", i);
        }
        else
        {
            int value = sensor->triggerSource.sensor.type.value;
            int flags = sensor->triggerSource.sensor.type.flags;
            printf("[%d] Trigger source:       %s 0x%02X (%s)\n", i, driverTypeToName(flags), value,
                   (flags == DRIVER_TYPE_PHYSICAL_FLAG) ? typeToPhysicalName(value) : typeToName(value));
        }
    }
    if(!sensor->physicalSource.header)
    {
            printf("[%d] Physical source:      None\n", i);
    }
    else
    {
        int value = sensor->physicalSource.sensor.type.value;
        int flags = sensor->physicalSource.sensor.type.flags;
        printf("[%d] Physical source:      %s 0x%02X (%s)\n", i, driverTypeToName(flags), value,
               (flags == DRIVER_TYPE_PHYSICAL_FLAG) ? typeToPhysicalName(value) : typeToName(value));
    }
#if defined(SENSOR_TEMPERATURE)
    if(!sensor->temperatureSource.header)
    {
        printf("[%d] Temperature source:   None\n", i);
    }
    else
    {
        int value = sensor->temperatureSource.sensor->type.value;
        int flags = sensor->temperatureSource.sensor->type.flags;
        printf("[%d] Temperature source:   %s 0x%02X (%s)\n", i, driverTypeToName(flags), value,
               (flags == DRIVER_TYPE_PHYSICAL_FLAG) ? typeToPhysicalName(value) : typeToName(value));
    }
#endif
}

bool stuffSensors(void* elf)
{
    bool hasDriverID[MAX_VIRTUAL_SENSORS] = {0};
    bool hasPhysicalSensor[SENSOR_TYPE_MAX_DESCRIPTORS] = {0};
    bool hasVirtualSensor[SENSOR_TYPE_MAX_DESCRIPTORS] = {0};
    PhysicalSensorDescriptor* gpioPins[NUM_GPIO_PINS + 1] = {0}; // +1 for SIF1CNS
    bool gpioIsInterrupt[NUM_GPIO_PINS + 1] = {0}; // +1 for SIF1CNS
    SInt32 hifGpio = -1;
    if(hasEM7189Config(elf))
    {
        if(mEM7189Config.PinSelection.bits.HIFDisable)
        {
            hifGpio = -1;
        }
        else
        {
            hifGpio = mEM7189Config.PinSelection.bits.HostIRQ;
        }
    }

    int numSensors =  get_section_size(elf, STR_PHYS_SENSOR_DESCRIPTOR) / PHYS_SENSOR_DESCRIPTOR_SIZE;
    if(numSensors)
    {
        int i;
        for(i = 0; i < numSensors; i++)
        {
            PhysicalSensorDescriptor* sensor = &gPhysSensorDrivers[i];

            int allowed_interface  = (int)sensor->device.interface.options;
            int selected_interface = (int)sensor->device.interface.type;
            if(  (allowed_interface & selected_interface) ||
                ((allowed_interface == InterfaceNone) && (selected_interface == InterfaceNone))) // None case
            {
                // Check to see if the interface selected is allowed per the SIF Selection
                if(!interfaceAllowed(mEM7189Config.SIFSelection, selected_interface))
                {
                    error("Error: invalid interface '%s' (0x%02x) selected for sensor (0x%02X: %s) due to the SIF selection being %d (%s).\n",
                            interfaceToName(sensor->device.interface.type), sensor->device.interface.type,
                            sensor->type.value, typeToPhysicalName(sensor->type.value),
                            mEM7189Config.SIFSelection, SIFModeToName(mEM7189Config.SIFSelection));

                    exit(-1);
                }
            }
            else
            {
                error("Error: invalid interface '%s' (0x%02x) selected for sensor (0x%02X: %s). Valid options: '%s' (%d)\n",
                        interfaceToName(sensor->device.interface.type), sensor->device.interface.type,
                        sensor->type.value, typeToPhysicalName(sensor->type.value),
                        interfaceToName(sensor->device.interface.options), sensor->device.interface.options);
                exit(-1);
            }

            if(!sensor->numAxis) sensor->numAxis = typeToAxis(sensor->type.value);

            // Print Descriptor
            if(!quiet)
            {
                printSensor(sensor, i);
            }

            if(sensor->type.value > SENSOR_TYPE_MAX_DESCRIPTORS)
            {
                error("Error: physical sensors of type '%s' (%d) is invalid.\n",
                        typeToPhysicalName(sensor->type.value), sensor->type.value);
                exit(-1);
            }

            if(hasDriverID[sensor->info.id])
            {
                printf("Error: Multiple sensors with id (%d) exist.\n",
                        sensor->info.id);
                exit(-1);
            }
            hasDriverID[sensor->info.id] = 1;

            if(hasPhysicalSensor[sensor->type.value])
            {
                error("Error: Multiple physical sensors of type '%s' (%d) exist.\n",
                        typeToPhysicalName(sensor->type.value), sensor->type.value);
                exit(-1);
            }
            hasPhysicalSensor[sensor->type.value] = 1;

            if((int)sensor->device.interface.type & (int)InterfaceAnySPI)
            {
                if(gpioPins[sensor->device.options.spi.csPin])
                {
                    PhysicalSensorDescriptor* conflict = gpioPins[sensor->device.options.spi.csPin];
                    if(gpioIsInterrupt[sensor->device.options.spi.csPin])
                    {
                        error("ERROR: Chip Select pin %d for sensor (0x%02X: %s) is used as an interrupt for another sensor (0x%02X: %s).\n",
                            sensor->device.options.spi.csPin,
                            sensor->type.value, typeToPhysicalName(sensor->type.value),
                            conflict->type.value, typeToPhysicalName(conflict->type.value));
                        exit(-1);
                    }
                    else
                    {
                        // Multiple sensors can have the same chip select specified if they are composite sensors. Since we don't error about i2c addresses matching, don't error here.
                        #if 0
                        error("Warning: Chip Select pin %d for sensor (0x%02X: %s) is already in used another sensor (0x%02X: %s).\n",
                            sensor.device.options.spi.csPin,
                            sensor.type.value, typeToPhysicalName(sensor.type.value),
                            conflict->type.value, typeToPhysicalName(conflict->type.value));
                        #endif
                    }
                }
                else
                {
                    gpioPins[sensor->device.options.spi.csPin] = sensor;
                    gpioIsInterrupt[sensor->device.options.spi.csPin] = FALSE;
                }

                if(hifGpio == sensor->device.options.spi.csPin)
                {
                    error("Error: Chip Select pin %d for sensor (0x%02X: %s) is configured as the host interrupt pin.\n",
                        hifGpio,
                        sensor->type.value, typeToPhysicalName(sensor->type.value));
                    exit(-1);
                }
            }

            if(!sensor->device.irqDis)
            {
                if(gpioPins[sensor->device.irqPin])
                {
                    PhysicalSensorDescriptor* conflict = gpioPins[sensor->device.irqPin];
                    if(gpioIsInterrupt[sensor->device.irqPin])
                    {
                        // Only one sensor can have an interrupt source configured.
                        error("ERROR: IRQ pin %d for sensor (0x%02X: %s) is already in used another sensor (0x%02X: %s).\n",
                            sensor->device.irqPin,
                            sensor->type.value, typeToPhysicalName(sensor->type.value),
                            conflict->type.value, typeToPhysicalName(conflict->type.value));
                        exit(-1);
                    }
                    else
                    {
                        error("ERROR: IRQ pin %d for sensor (0x%02X: %s) is already in used as the chip select for another sensor (0x%02X: %s).\n",
                            sensor->device.irqPin,
                            sensor->type.value, typeToPhysicalName(sensor->type.value),
                            conflict->type.value, typeToPhysicalName(conflict->type.value));
                        exit(-1);
                    }
                }
                else
                {
                    gpioPins[sensor->device.irqPin] = sensor;
                    gpioIsInterrupt[sensor->device.irqPin] = TRUE;
                }

                if(hifGpio == sensor->device.irqPin)
                {
                    error("Error: IRQ pin %d for sensor (0x%02X: %s) is configured as the host interrupt pin.\n",
                        hifGpio,
                        sensor->type.value, typeToPhysicalName(sensor->type.value));
                    exit(-1);
                }
            }
        }

        if(!set_section_data(elf, STR_PHYS_SENSOR_DESCRIPTOR, (void*)gPhysSensorDrivers, get_section_size(elf, STR_PHYS_SENSOR_DESCRIPTOR)))
        {
            error("Unable to update '%s' section.\n", STR_PHYS_SENSOR_DESCRIPTOR);
            exit(-1);
        }
    }

    /** Stuff virtual sensors as needed **/
    int numVirts = get_section_size(elf, STR_VIRT_SENSOR_DESCRIPTOR) / VIRT_SENSOR_DESCRIPTOR_SIZE;
    int numTimers = get_section_size(elf, STR_TIME_SENSOR_DESCRIPTOR) / VIRT_SENSOR_DESCRIPTOR_SIZE;
    numSensors =  numVirts + numTimers;
    if(numSensors)
    {
        int i;
        if(!quiet)
        {
            for(i = 0; i < numSensors; i++)
            {
                VirtualSensorDescriptor* sensor = &gVirtSensorDrivers[i];
                printVirtSensor(sensor, i);

                if(sensor->type.value > SENSOR_TYPE_MAX_DESCRIPTORS)
                {
                    error("Error: virtual sensors of type '%s' (%d) is invalid.\n",
                            typeToName(sensor->type.value), sensor->type.value);
                    exit(-1);
                }

                if((sensor->info.id != BSX_DRIVER_ID) &&  /* Special case - BSX has a calibration driver and a calculation driver */
                    hasDriverID[sensor->info.id])
                {
                    printf("Error: Multiple sensors with id (%d) exist.\n",
                            sensor->info.id);
                    exit(-1);
                }
                hasDriverID[sensor->info.id] = 1;

                if(hasVirtualSensor[sensor->type.value])
                {
                    printf("Error: Multiple virtual sensors of type '%s' (%d) exist.\n",
                            typeToName(sensor->type.value), sensor->type.value);
                    exit(-1);
                }
                hasVirtualSensor[sensor->type.value] = 1;
            }
        }

        if(numVirts)
        {
            if(!set_section_data(elf, STR_VIRT_SENSOR_DESCRIPTOR, (void*)gVirtSensorDrivers, get_section_size(elf, STR_VIRT_SENSOR_DESCRIPTOR)))
            {
                error("Unable to update '%s' section.\n", STR_VIRT_SENSOR_DESCRIPTOR);
                exit(-1);
            }
        }
        if(numTimers)
        {
            if(!set_section_data(elf, STR_TIME_SENSOR_DESCRIPTOR, (void*)&gVirtSensorDrivers[numVirts], get_section_size(elf, STR_TIME_SENSOR_DESCRIPTOR)))
            {
                error("Unable to update '%s' section.\n", STR_TIME_SENSOR_DESCRIPTOR);
                exit(-1);
            }
        }
    }
    return TRUE;
}

bool parseSensors(void* elf)
{
    bool status = TRUE;
    bool hasPhys = FALSE;
    // Prepar descriptors
    PhysicalSensorDescriptor* phys = NULL;
    VirtualSensorDescriptor* virt = NULL;
    UInt32 numPhys = 0;
    UInt32 numBoth = 0;
    if(get_section_size(elf, STR_PHYS_SENSOR_DESCRIPTOR))
    {
        hasPhys = TRUE;
        numPhys = get_section_size(elf, STR_PHYS_SENSOR_DESCRIPTOR) / PHYS_SENSOR_DESCRIPTOR_SIZE;
        if(numPhys * PHYS_SENSOR_DESCRIPTOR_SIZE != get_section_size(elf, STR_PHYS_SENSOR_DESCRIPTOR))
        {
            printf("ERROR: Expected size of physical sensor descriptors (%zu) does not match size in elf file (%" PRIu64 " / N).\n", PHYS_SENSOR_DESCRIPTOR_SIZE, get_section_size(elf, STR_PHYS_SENSOR_DESCRIPTOR));
            status = FALSE;
        }

        verbose("%d physical sensors found.\n", numPhys);


        phys = (void*)get_section_data(elf, STR_PHYS_SENSOR_DESCRIPTOR);
        if(phys)
        {
            memcpy(gPhysSensorDrivers, phys, numPhys * PHYS_SENSOR_DESCRIPTOR_SIZE);
        }
        else
        {
            printf("ERROR: Unable to read in physical drivers, size: %" PRIu64 "\n", get_section_size(elf, STR_PHYS_SENSOR_DESCRIPTOR));
            return FALSE;
        }
    }
    else
    {
        verbose("No physical sensor drivers located in firmware image.\n");
        status = FALSE;
    }

    if(get_section_size(elf, STR_VIRT_SENSOR_DESCRIPTOR) + get_section_size(elf, STR_TIME_SENSOR_DESCRIPTOR))
    {
        int timerBytes = get_section_size(elf, STR_TIME_SENSOR_DESCRIPTOR);
        int virtBytes = get_section_size(elf, STR_VIRT_SENSOR_DESCRIPTOR);
        int numTimers = timerBytes / VIRT_SENSOR_DESCRIPTOR_SIZE;
        int numVirts =  virtBytes / VIRT_SENSOR_DESCRIPTOR_SIZE;
        numBoth = numTimers + numVirts;
        if(numBoth * VIRT_SENSOR_DESCRIPTOR_SIZE != (timerBytes + virtBytes))
        {
            error("ERROR: Expected size of virtual sensor descriptors (%zu) does not match size in elf file (%d / N).\n", VIRT_SENSOR_DESCRIPTOR_SIZE, timerBytes + virtBytes);
            status = FALSE;
        }

        verbose("%d virtual sensors found.\n", numVirts);
        verbose("%d timers sensors found.\n", numTimers);

        if (virtBytes)
        {
            virt = (void*)get_section_data(elf, STR_VIRT_SENSOR_DESCRIPTOR);
            memcpy(gVirtSensorDrivers, virt, virtBytes);
        }

        // seek to timer sensors.
        if (numTimers)
        {
            virt = (void*)get_section_data(elf, STR_TIME_SENSOR_DESCRIPTOR);
            memcpy(&gVirtSensorDrivers[numVirts], virt, timerBytes);
        }
    }
    else
    {
        if(!quiet && !hasPhys) printf("No virtual sensor drivers located in firmware image.\n");
        else fprintf(stdout, "No virtual sensor drivers located in firmware image.\n");
        //status = FALSE; // TODO: enable this once all board.cfg files include correct virtual drivers.
    }

    // parseSensorDrivers mangles the pointers... make a copy here.
    phys = malloc(numPhys * PHYS_SENSOR_DESCRIPTOR_SIZE);
    memcpy(phys, gPhysSensorDrivers, numPhys * PHYS_SENSOR_DESCRIPTOR_SIZE);
    virt = malloc(numBoth * VIRT_SENSOR_DESCRIPTOR_SIZE);
    memcpy(virt, gVirtSensorDrivers, numBoth * VIRT_SENSOR_DESCRIPTOR_SIZE);

    if(!virt || !phys)
    {
        status = FALSE;
    }
    else
    {
        if(!parseSensorDrivers(phys, numPhys, virt, numBoth))
        {
            status = FALSE;
        }

        free(phys);
        free(virt);
    }
    return status;
}

void printTriggerList(SensorDescriptorHeader* sensor, UInt32 depth_offset)
{
    char priority = (getPriorityLevel(sensor) == PRIORITY_M) ? 'M' : getPriorityLevel(sensor) + '0';
    if(sensor->type.flags == DRIVER_TYPE_TIMER_FLAG)
    {
        VirtualSensorDescriptor* timer = cast_HeaderToVirtual(sensor);
        // timer as trigger.
        if(timer->triggerSource.timer.rate)
        {
            verbose("%dHz Timer\n", timer->triggerSource.timer.rate);
        }
        else
        {
            verbose("continuous\n");
        }
        verbose("   [%c] %s sensor [DriverID %d]\n", priority, typeToName(timer->type.value), timer->info.id);
        depth_offset++;
    }
    else if(sensor->type.flags == DRIVER_TYPE_VIRTUAL_FLAG)
    {
        const char* name = typeToName(sensor->type.value);
        if(sensor->info.id == BSX_DRIVER_ID)
        {
            name = "BSX";
        }
        // No trigger list source
        UInt32 depth = depth_offset;
        while(depth--)
        {
            verbose("  ");
        }
        verbose("   [%c] %s sensor [DriverID %d]\n", priority, name, sensor->info.id);
    }
    else
    {
        char priority = '1';
        // Physical trigger.
        verbose("[%c] %s sensor [DriverID %d]\n", priority, typeToPhysicalName(sensor->type.value), sensor->info.id);
    }

    while(sensor->triggerList)
    {
        sensor = gTriggerListEntries[sensor->triggerList];
        UInt32 depth = depth_offset + getSensorDepth(sensor);
        priority = (getPriorityLevel(sensor) == PRIORITY_M) ? 'M' : getPriorityLevel(sensor) + '0';;
        while(depth--)
        {
            verbose("   ");
        }
        verbose("  [%c] %s sensor [DriverID %d]\n", priority, typeToName(sensor->type.value), sensor->info.id);
    }
}

void printPhysicalSensorDrivers(PhysicalSensorDescriptor* phys, UInt32 numPhys)
{
    int i;

    verbose("\n--------- Physical Sensor Drivers ---------\n");
    for(i = 0; i < numPhys; i++)
    {
        SensorDescriptorHeader* sensor = cast_PhysicalToHeader(&phys[i]);

        verbose("[%d] %s sensor [DriverID %d]\n", i, typeToPhysicalName(sensor->type.value), sensor->info.id);
    }
}

void printVirtualSensorDrivers(VirtualSensorDescriptor* virt, UInt32 numVirt, PhysicalSensorDescriptor* phys, UInt32 numPhys)
{
    int i;
    verbose("\n--------- Virtual Sensor Drivers ---------\n");

    for(i = 0; i < numVirt; i++)
    {
        SensorDescriptorHeader* sensor = cast_VirtualToHeader(&virt[i]);

        verbose("[%d] %s sensor [DriverID %d]\n", i, typeToName(sensor->type.value), sensor->info.id);
    }
}

UInt8 getPriorityLevel(SensorDescriptorHeader* sensor)
{
    if(sensor->type.flags == DRIVER_TYPE_PHYSICAL_FLAG)
    {
        // physical drivers run at priority 1, but return 2 to ensure children always run at 2
        return PRIORITY_2;
    }
    else
    {
        VirtualSensorDescriptor* vs = cast_HeaderToVirtual(sensor);
        return vs->priority;
    }
}

UInt32 getSensorDepth(SensorDescriptorHeader* sensor)
{
    UInt32 depth = 0;
    while( (sensor = getSensorParent(sensor)) != NULL)
    {
        depth++;
    }

    return depth;
}

SensorDescriptorHeader* getSensorParent(const SensorDescriptorHeader* sensor)
{
    UInt32 type = sensor->type.flags;
    SensorDescriptorHeader* parent = NULL;
    if( type == DRIVER_TYPE_VIRTUAL_FLAG)
    {
        // Physical sensors and timer sensor have no parent sensor, only virtual sensor have parents
        if(cast_ConstHeaderToVirtual(sensor)->triggerSource.header)
        {
            parent = gTriggerSourceEntries[cast_ConstHeaderToVirtual(sensor)->triggerSource.header];
        }
    }

    return parent;
}

bool parseSensorDrivers(PhysicalSensorDescriptor* phys, UInt32 numPhys, VirtualSensorDescriptor* virt, UInt32 numVirt)
{
    int triggerListIndex = 1;
    int triggerSourceIndex = 1;
    gVirts = virt;
    gNumVirts = numVirt;
    bool priorityUpdated;
    int i;
    for(i = 0; i < numVirt; i++)
    {
        // Error checking...
        SensorDescriptorHeader* sensor = cast_VirtualToHeader(&virt[i]);
        VirtualSensorDescriptor* vs = &virt[i];

        // (1) Ensure all sensor drivers run at either priority 2 or M

        if(getPriorityLevel(sensor) < PRIORITY_2)
        {
            error("ERROR: Sensor driver '%s' (%d) requested a priority of %d\n", typeToName(sensor->type.value), sensor->type.value, getPriorityLevel(sensor));
        }

        if(getPriorityLevel(sensor) >= PRIORITY_LOWEST)
        {
            vs->priority = PRIORITY_M;
        }

        // (2) Ensure drivers have a vlid trigger source.
        if(sensor->type.flags == DRIVER_TYPE_TIMER_FLAG)
        {
            // OK
        }
        else
        {
            SensorDescriptorHeader* parent = NULL;
            switch(vs->triggerSource.sensor.type.flags)
            {
                case DRIVER_TYPE_TIMER_FLAG:
                case DRIVER_TYPE_VIRTUAL_FLAG:  parent = cast_VirtualToHeader(getVirtualSensorDescriptorByType (vs->triggerSource.sensor.type.value, virt, numVirt)); break;
                case DRIVER_TYPE_PHYSICAL_FLAG: parent = cast_PhysicalToHeader(getPhysicalSensorDescriptorByType(vs->triggerSource.sensor.type.value, phys, numPhys)); break;
                default:
                    error("ERROR: invalid trigger source for driver.\n");
                    exit(-1);
            }

            if(!parent)
            {
                if(vs->triggerSource.sensor.type.value)
                {
                    error("[%d] %s sensor [DriverID %d]\n", i, typeToName(sensor->type.value), sensor->info.id);
                    error("ERROR: Unable to locate parent for sensor id %d (parent type %d, %s).\n", sensor->info.id, vs->triggerSource.sensor.type.value, typeToName(vs->triggerSource.sensor.type.value));
                    exit(-4);
                }
                vs->triggerSource.header = (PTR(SensorDescriptorHeader*)) NULL;
            }
            else
            {
                vs->triggerSource.header = triggerSourceIndex;
                gTriggerSourceEntries[triggerSourceIndex] = parent;
                triggerSourceIndex++;

                if(parent->triggerList)
                {
                    while(parent->triggerList)
                    {
                        parent = gTriggerListEntries[parent->triggerList];
                    }
                }

                parent->triggerList = triggerListIndex;
                gTriggerListEntries[triggerListIndex] = sensor;
                triggerListIndex++;
            }
        }

        // (3) Locate physical source
        if((SensorDescriptorHeader*)vs->physicalSource.header != NULL)
        {
            SensorDescriptorHeader* parent;
            switch(vs->physicalSource.sensor.type.flags)
            {
                case DRIVER_TYPE_TIMER_FLAG:
                case DRIVER_TYPE_VIRTUAL_FLAG:
                default:
                        error("ERROR: invalid physical source for driver - physical source must be a physical driver type.\n");
                        exit(-1);

                case DRIVER_TYPE_PHYSICAL_FLAG:
                    parent = cast_PhysicalToHeader(getPhysicalSensorDescriptorByType(vs->physicalSource.sensor.type.value, phys, numPhys));
                    vs->physicalSource.header = (PTR(SensorDescriptorHeader*))parent;
                    break;

            }
        }
    }

    // (3) Check for priority inversions
    do
    {

        // Run through as many passes are needed until no priority inversions are detected
        priorityUpdated = FALSE;
        for(i = 0; i < numVirt; i++)
        {
            VirtualSensorDescriptor* vs = &virt[i];
            SensorDescriptorHeader* sensor = cast_VirtualToHeader(vs);
            if( sensor && getSensorParent(sensor) &&
                (sensor->type.flags != DRIVER_TYPE_TIMER_FLAG) && // Primary trigger source, no parent
                getPriorityLevel(sensor) < getPriorityLevel(getSensorParent(sensor)))
            {
                priorityUpdated = TRUE;
                vs->priority = getPriorityLevel(getSensorParent(sensor)); // Don't allow priority inversions
            }
        }
    }
    while(priorityUpdated);

    printPhysicalSensorDrivers(phys, numPhys);
    printVirtualSensorDrivers(virt, numVirt, phys, numPhys);


    verbose("\n--------- Trigger Lists (physical) ---------\n");
    for(i = 0; i < numPhys; i++)
    {
        SensorDescriptorHeader* sensor = cast_PhysicalToHeader(&phys[i]);

        printTriggerList(sensor, 0);
    }
    verbose("\n--------- Trigger Lists (virtual: timer) ---------\n");
    for(i = 0; i < numVirt; i++)
    {
        SensorDescriptorHeader* sensor = cast_VirtualToHeader(&virt[i]);
        if(sensor->type.flags == DRIVER_TYPE_TIMER_FLAG)
        {
            printTriggerList(sensor, 0);
        }
    }
    bool header_printed = 0;
    for(i = 0; i < numVirt; i++)
    {
        SensorDescriptorHeader* sensor = cast_VirtualToHeader(&virt[i]);
        if(
            (sensor->type.flags == DRIVER_TYPE_VIRTUAL_FLAG) &&
            (cast_HeaderToVirtual(sensor)->triggerSource.header == 0) &&
            !((sensor->type.value  >= SENSOR_TYPE_BSX_RESERVED_START) && (sensor->type.value  <= SENSOR_TYPE_BSX_RESERVED_END))
        )

        {
            if(!header_printed)
            {
                header_printed = 1;
                verbose("\n--------- Trigger Lists (virtual: NO Source) ---------\n");
                verbose("unknown\n");
            }
            printTriggerList(sensor, 0);

            if(sensor->info.id == BSX_DRIVER_ID)  //BSX, special
            {
                int j;
                for(j = 0; j < numVirt; j++)
                {
                    sensor = cast_VirtualToHeader(&virt[j]);
                    if(
                        (sensor->type.flags == DRIVER_TYPE_VIRTUAL_FLAG) &&
                        (cast_HeaderToVirtual(sensor)->triggerSource.header == 0) &&
                        ((sensor->type.value  >= SENSOR_TYPE_BSX_RESERVED_START) && (sensor->type.value  <= SENSOR_TYPE_BSX_RESERVED_END))
                        )
                    {
                        printTriggerList(sensor, 2);
                    }
                }

            }
        }
    }
    verbose("\n");

    return TRUE;
}

PhysicalSensorDescriptor* getPhysicalSensorDescriptorByType(UInt32 type, PhysicalSensorDescriptor* phys, UInt32 numPhys)
{
    int i;
    for(i = 0; i < numPhys; i++)
    {
        PhysicalSensorDescriptor* sensor = &phys[i];
        if(sensor->type.value == type)
        {
            return sensor;
        }
    }
    return NULL;    // No sensor found for given type.
}

VirtualSensorDescriptor* getVirtOrTimerSensorDescriptorByType(UInt32 type)
{
    UInt8 numVirt = gNumVirts;
    VirtualSensorDescriptor* virt = gVirts;
    int i;
    for(i = 0; i < numVirt; i++)
    {
        VirtualSensorDescriptor* sensor = &virt[i];
        if(sensor->type.value == type)
        {
            return sensor;
        }
    }
    return NULL;    // No sensor found for given type.

}
