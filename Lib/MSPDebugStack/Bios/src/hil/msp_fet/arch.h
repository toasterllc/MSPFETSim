/*
 * Copyright (C) 2007 - 2014 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//! \ingroup MODULHIL
//! \file arch.h
//! \brief
//!
#include "msp430.h"
#define uController_uif

#ifndef _UIFV1_ARCH_H_
#define _UIFV1_ARCH_H_

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

static const unsigned short ADC_CONV_RANGE = 4096;
static const unsigned short ADC_AVERAGE = 1000;

struct jtag
{
  unsigned char  TCK;
  unsigned char  TMS;
  unsigned char  TDI;
  unsigned char  TDO;
  unsigned char* In;
  unsigned char* Out;
  unsigned char RST;
  unsigned char TST;
  unsigned char* DIRECTION;
};

struct vfuse_ctrl
{
  unsigned char  VF2TEST;
  unsigned char  VF2TDI;
  unsigned char  TDI_OFF;
  unsigned char* CTRL;
  unsigned char  PWM_SETVF;
  unsigned char* PWM_CTRL;
};

static const struct jtag _Jtag_SubMcu =
{
  0,  // TCK, unused in SBW mode
  0,  // TMS, unused in SBW mode
  0,  // TDI, unused in SBW mode
  0,  // TDO, unused in SBW mode
  (unsigned char*)&P7IN,  // JTAG input Register
  (unsigned char*)&P7OUT, // JTAG output Register
  (unsigned char)BIT5,    // RST, P7.5, (inout)
  (unsigned char)BIT4,    // TST, P7.4, (out)
  (unsigned char*)&P7DIR  // JTAG direction Register
};

static const struct jtag _SBW_Back =
{
  0,  // TCK, unused in SBW mode
  0,  // TMS, unused in SBW mode
  0,  // TDI, unused in SBW mode
  0,  // TDO, unused in SBW mode
  (unsigned char*)&P3IN,  // JTAG input Register
  (unsigned char*)&P3OUT, // JTAG output Register
  (unsigned char)BIT3,    // RST, P3.3  TDO(in/out)
  (unsigned char)BIT0,    // TST, P3.0  TCK(out)
  (unsigned char*)&P3DIR  // JTAG direction Register
};

static const struct jtag _Jtag_Target =
{
  (unsigned char)BIT0,    // TCK, P3.0 (out) (high)
  (unsigned char)BIT1,    // TMS, P3.1 (out) (high)
  (unsigned char)BIT2,    // TDI, P3.2 (out) (high)
  (unsigned char)BIT3,    // TDO, P3.3 (in)
  (unsigned char*)&P3IN,  // JTAG input Register
  (unsigned char*)&P3OUT, // JTAG output Register
  (unsigned char)BIT4,    // RST, P3.4 (out)
  (unsigned char)BIT5,    // TST, P3.5 (out)
  (unsigned char*)&P3DIR  // JTAG direction Register
};

static const struct jtag _Jtag_FPGA =
{
  (unsigned char)BIT0,    // TCK, P9.0 (out) (high)
  (unsigned char)BIT2,    // TMS, P9.2 (out) (high)
  (unsigned char)BIT1,    // TDI, P9.1 (out) (high)
  (unsigned char)BIT3,    // TDO, P9.3 (in)
  (unsigned char*)&P9IN,  // JTAG input Register
  (unsigned char*)&P9OUT, // JTAG output Register
  (unsigned char)BIT4,    // RST
  (unsigned char)BIT4,    // TST
  (unsigned char*)&P9DIR  // JTAG direction Register
};

static const struct vfuse_ctrl _Msp_Fet =
{
  (unsigned char)BIT2,      // VF2TEST, P5.2 (out)
  (unsigned char)BIT4,      // VF2TDI, P5.4 (out)
  (unsigned char)BIT5,      // TDIOFF, P5.5 (out)
  (unsigned char*)&P5OUT,   // VF control register
  (unsigned char)BIT7,      // VF PWM, P9.7 (out)
  (unsigned char*)&P9OUT    // VF PWM control register
};

struct savedDacValues
{
    unsigned short DAC0;
    unsigned short DAC1;
    unsigned short Vcc;
    unsigned short valid;
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

#endif
