#pragma once

#include "Bios/include/modules.h"
#include "Bios/include/protocol.h"
#include "Bios/include/error_def.h"
#include "Bios/include/ConfigureParameters.h"
#include "Bios/src/fw/fet/FetVersion.h"
#include "Bios/src/fw/fet/communicationDefs.h"
#include "Bios/src/hal/JTAG_defs.h"
#include "Bios/src/hal/EEM_defs.h"

const int16_t SW_0 = (VERSION_MAJOR - 1) << 14 | (VERSION_MINOR << 8) | VERSION_PATCH;
const int16_t SW_1 = VERSION_BUILD;

const int16_t SWCMP_0 = (VERSION_MAJOR_CMP - 1) << 14 | (VERSION_MINOR_CMP << 8) | VERSION_PATCH_CMP;
const int16_t SWCMP_1 = VERSION_BUILD_CMP;

const uint16_t HIL_Version = MSP_FET_HIL_VERSION;
const uint16_t HIL_VersionCMP = MSP_FET_HIL_VERSION_CMP;

#define FPGA_VERSION 0x016
#define FPGA_SIGNATURE 0xADACADAC

#define BIOS_RX_SIZE        258
#define BIOS_RX_QUEUS       4
#define BIOS_TX_SIZE        258
#define BIOS_TX_QUEUS       1
#define BIOS_TX_FLAGS       1

#define STREAM_CORE_ZERO_VERSION    0x00
#define STREAM_CORE_ZERO_MACRO_SIZE 0x01
#define STREAM_CORE_ZERO_MACRO_ADDR 0x02
#define STREAM_CORE_ZERO_PUC_RESET  0x03

#define HAL_SIGNATURE   0xBEEFBEEFul
#define HIL_SIGNATURE   0xF00DF00Dul
#define DCDC_SIGNATURE  0xABBAABBAul
#define COM_SIGNATURE   0xACDCACDCul

#define INFO_U1_HW_0 0xFF55
#define INFO_U1_HW_1 0x0140

#define MESSAGE_NEW_MSG 0x0001
#define MESSAGE_LAST_MSG 0x0002

#define eZ_FET_WITH_DCDC                0xAAAA
#define eZ_FET_NO_DCDC                  0xAAAB
#define eZ_FET_WITH_DCDC_NO_FLOWCTL     0xAAAC
#define MSP_FET_WITH_DCDC               0xBBBB
#define MSP_FET_WITH_DCDC_V2x           0xBBBC
#define eZ_FET_WITH_DCDC_V2x            0xAAAD

#define TOOL_ID      MSP_FET_WITH_DCDC_V2x

#define COM_CHANNEL 2
#define DEBUG_CHANNEL 0

struct _DeviceSettings_
{
    uint32_t  clockControlType;
    uint16_t stopFLL;
    uint16_t assertBslValidBit;
};
typedef struct _DeviceSettings_ DeviceSettings;

struct _DevicePowerSettings_
{
    uint32_t powerTestRegMask;
    uint32_t powerTestRegDefault;
    uint32_t enableLpmx5TestReg;
    uint32_t disableLpmx5TestReg;

    uint16_t powerTestReg3VMask;
    uint16_t powerTestReg3VDefault;
    uint16_t enableLpmx5TestReg3V;
    uint16_t disableLpmx5TestReg3V;
};
typedef struct _DevicePowerSettings_ DevicePowerSettings;

typedef struct _ARMConfigSettings
{
    uint32_t scsBase; // System Control Space base address
    uint32_t fpbBase; // FLASH Patch Block base address
    uint32_t interruptOptions; // Options to enable/disable interrupts when single stepping or letting the device run
    uint32_t ulpDebug; // Options to enable/disable entry to LPM. poll for PB hit in low poer mode
} ARMConfigSettings;
