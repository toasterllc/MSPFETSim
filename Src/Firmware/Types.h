#pragma once

#include "Bios/include/modules.h"
#include "Bios/include/protocol.h"
#include "Bios/include/error_def.h"
#include "Bios/include/ConfigureParameters.h"
#include "Bios/src/fw/fet/FetVersion.h"
#include "Bios/src/fw/fet/communicationDefs.h"
#include "Bios/src/hal/JTAG_defs.h"
#include "Bios/src/hal/EEM_defs.h"

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

#define _DINT_FET()
#define _EINT_FET()

#define REQUIRED(x)

const int16_t SW_0 = (VERSION_MAJOR - 1) << 14 | (VERSION_MINOR << 8) | VERSION_PATCH;
const int16_t SW_1 = VERSION_BUILD;

const int16_t SWCMP_0 = (VERSION_MAJOR_CMP - 1) << 14 | (VERSION_MINOR_CMP << 8) | VERSION_PATCH_CMP;
const int16_t SWCMP_1 = VERSION_BUILD_CMP;

const uint16_t HIL_Version = MSP_FET_HIL_VERSION;
const uint16_t HIL_VersionCMP = MSP_FET_HIL_VERSION_CMP;

enum {JTAG = 0, SPYBIWIRE = 1, SPYBIWIREJTAG = 2, JTAGUNDEF = 4, SPYBIWIRE_SUBMCU = 5, SPYBIWIRE_MSP_FET=6, JTAG_432 = 7, SWD_432 = 8};

#define RSTHIGH 0
#define RSTLOW  1

#define Invalid_Manufactor_IdCode 0x000000FEul
#define Mask_Manufactor_IdCode 0x000000FEul

#define  A_VREFPLUS     2.5f         // conversion reference voltage Vref+=2.5V,

#define DCDC_STATUS     (P3IN & (BIT0+BIT1)) // Status bits from the DCDC Sub MCU

// Fuse blow related constants
#define VF_PWM_TIMEOUT  4000
#define VF_PWM_PERIOD   50
#define VF_PWM_ON_TIME  40
#define VF_PWM_OFF_TIME (VF_PWM_PERIOD - VF_PWM_ON_TIME)

const uint16_t ADC_CONV_RANGE = 4096;
const uint16_t ADC_AVERAGE = 1000;

struct jtag
{
  uint8_t  TCK;
  uint8_t  TMS;
  uint8_t  TDI;
  uint8_t  TDO;
  uint8_t* In;
  uint8_t* Out;
  uint8_t RST;
  uint8_t TST;
  uint8_t* DIRECTION;
};

struct vfuse_ctrl
{
  uint8_t  VF2TEST;
  uint8_t  VF2TDI;
  uint8_t  TDI_OFF;
  uint8_t* CTRL;
  uint8_t  PWM_SETVF;
  uint8_t* PWM_CTRL;
};

//const struct jtag _Jtag_SubMcu =
//{
//  0,  // TCK, unused in SBW mode
//  0,  // TMS, unused in SBW mode
//  0,  // TDI, unused in SBW mode
//  0,  // TDO, unused in SBW mode
//  (uint8_t*)&P7IN,  // JTAG input Register
//  (uint8_t*)&P7OUT, // JTAG output Register
//  (uint8_t)BIT5,    // RST, P7.5, (inout)
//  (uint8_t)BIT4,    // TST, P7.4, (out)
//  (uint8_t*)&P7DIR  // JTAG direction Register
//};
//
//const struct jtag _SBW_Back =
//{
//  0,  // TCK, unused in SBW mode
//  0,  // TMS, unused in SBW mode
//  0,  // TDI, unused in SBW mode
//  0,  // TDO, unused in SBW mode
//  (uint8_t*)&P3IN,  // JTAG input Register
//  (uint8_t*)&P3OUT, // JTAG output Register
//  (uint8_t)BIT3,    // RST, P3.3  TDO(in/out)
//  (uint8_t)BIT0,    // TST, P3.0  TCK(out)
//  (uint8_t*)&P3DIR  // JTAG direction Register
//};
//
//const struct jtag _Jtag_Target =
//{
//  (uint8_t)BIT0,    // TCK, P3.0 (out) (high)
//  (uint8_t)BIT1,    // TMS, P3.1 (out) (high)
//  (uint8_t)BIT2,    // TDI, P3.2 (out) (high)
//  (uint8_t)BIT3,    // TDO, P3.3 (in)
//  (uint8_t*)&P3IN,  // JTAG input Register
//  (uint8_t*)&P3OUT, // JTAG output Register
//  (uint8_t)BIT4,    // RST, P3.4 (out)
//  (uint8_t)BIT5,    // TST, P3.5 (out)
//  (uint8_t*)&P3DIR  // JTAG direction Register
//};
//
//const struct jtag _Jtag_FPGA =
//{
//  (uint8_t)BIT0,    // TCK, P9.0 (out) (high)
//  (uint8_t)BIT2,    // TMS, P9.2 (out) (high)
//  (uint8_t)BIT1,    // TDI, P9.1 (out) (high)
//  (uint8_t)BIT3,    // TDO, P9.3 (in)
//  (uint8_t*)&P9IN,  // JTAG input Register
//  (uint8_t*)&P9OUT, // JTAG output Register
//  (uint8_t)BIT4,    // RST
//  (uint8_t)BIT4,    // TST
//  (uint8_t*)&P9DIR  // JTAG direction Register
//};
//
//const struct vfuse_ctrl _Msp_Fet =
//{
//  (uint8_t)BIT2,      // VF2TEST, P5.2 (out)
//  (uint8_t)BIT4,      // VF2TDI, P5.4 (out)
//  (uint8_t)BIT5,      // TDIOFF, P5.5 (out)
//  (uint8_t*)&P5OUT,   // VF control register
//  (uint8_t)BIT7,      // VF PWM, P9.7 (out)
//  (uint8_t*)&P9OUT    // VF PWM control register
//};

struct savedDacValues
{
    uint16_t DAC0;
    uint16_t DAC1;
    uint16_t Vcc;
    uint16_t valid;
};
typedef struct savedDacValues savedDacValues_t;

#define VALID_DATA   0x1A
#define SYNC_ONGOING 0x2A
#define INVALID_DATA 0x3A
#define JTAG_LOCKED  0x4A

#define BP_HIT_MASK_J              0x0400000000000000ull
#define LPMX5_MASK_J               0x4000000000000000ull
#define LPM4_1MASK_J               0x8000000000000000ull
#define LPM4_2MASK_J               0x8300000000000000ull
#define EIGHT_JSTATE_BITS          0x100000000000000ull
#define SYNC_BROKEN  0x5A

#define JSTATE_FLOW_CONTROL_BITS   0xC7
#define JSTATE_BP_HIT              0x4
#define JSTATE_SYNC_ONGOING        0x83
#define JSTATE_LOCKED_STATE        0x40
#define JSTATE_INVALID_STATE       0x81
#define JSTATE_LPM_ONE_TWO         0x82
#define JSTATE_LPM_THREE_FOUR      0x80
#define JSTATE_VALID_CAPTURE       0x03
#define JSTATE_LPM_X_FIVE          0xC0

#define JSTATE_SYNC_BROKEN_MASK         0xC3
#define JSTATE_SYNC_BROKEN_PGACT        0x02
#define JSTATE_SYNC_BROKEN_MCLK         0x01
#define JSTATE_SYNC_BROKEN_MCLK_PGACT   0x00

#define FET_TRUE                   0x1
#define FET_FALSE                  0x0

#define L092_MODE 0xA55AA55A
#define C092_MODE 0x5AA55AA5
