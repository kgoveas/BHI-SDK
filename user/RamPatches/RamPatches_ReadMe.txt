This RamPatches_ReadMe.txt provides some general information on RAMPatches, how to enable them and what kind of RAMPatches are delivered with the SDK:

General Information:
RAMPatches can contain code and functions that is compiled into the firmware image.
The main usage is to include Hooks into firmware images. For more information on Hooks, please refer to the Programmer's Manual.

There are two ways to add RAMPatches to firmware images, globally or firmware specifically:
1. Enabling globally:
In Bosch_Sensortec_Fuser2_SDK/common/config.7189_di03_*.cmake, add the name of the RAMPatch .c file without extension in "set(RAM_PATCHES" to enable the RAMPatch.
The RAMPatch will be compiled in all firmware images with the next build of the SDK.

2. Firmware/Board specifically e.g. example_board.cfg :
In Bosch_Sensortec_Fuser2_SDK/boards/example_board.cfg, add anywhere before the listing of physical drivers the keyword 'ram_patches,' in a new line.
Add the RAMPatch .c file name without extension after 'ram_patches,'. Multiple RAMPatches are allowed with comma separation.
The RAMPatch will be compiled into the specific board's firmware image with the next build of the SDK.


User RAM patch descriptions
Name:           custom_BSX_low_power_sensors
Description:    Provides means to define custom BSX sensors as low-power capable.
                Per default, any custom BSX sensor is defined as non-low-power capable and prevents BSX to operate in low-power mode.
Requirement:    Optional


Name:           getBuildTime
Description:    Updates the BuildTime information with the timestamp from the current compilation.
Requirement:    Optional, enabled by default
