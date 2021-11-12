/* Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
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

#pragma once

#define uController_uif

enum {JTAG = 0, SPYBIWIRE = 1, SPYBIWIREJTAG = 2, JTAGUNDEF = 4, SPYBIWIRE_SUBMCU = 5, SPYBIWIRE_MSP_FET=6, JTAG_432 = 7, SWD_432 = 8};


typedef enum {
    NO_SUPPLY = 0x04,
    ET_OFF = 0x05,
    ET_ON = 0x06,
    LDO_OFF = 0x07,
    ALL_OFF = 0x8,
    LDO_ON = 0x09
} vccSupply_t;

#define RSTHIGH 0
#define RSTLOW  1

#define Invalid_Manufactor_IdCode 0x000000FEul
#define Mask_Manufactor_IdCode 0x000000FEul

#define  ExtLimit        1700            // a level higher than this means external voltage available

#define  R27            250.0f           // 250kOhm
#define  R28            250.0f           // 250kOhm
#define  R29            500.0f           // 500kOhm
#define  R30            500.0f           // 500kOhm
#define  R31            250.0f           // 250kOhm
#define  R32            150.0f           // 150kOhm
#define  A_VREFPLUS     2.5f         // conversion reference voltage Vref+=2.5V,

#define DCDC_STATUS     (P3IN & (BIT0+BIT1)) // Status bits from the DCDC Sub MCU

// Calculate ExtVcc based on Vref+=2.5V, R27=250kOhm, R28=250kOhm
// Calculate Vcc    based on Vref+=2.5V, R29=500kOhm, R30=500kOhm

#define  A_VBUS          0 /* ADC12 Input Channel Select Bit 0 */
#define  A_VCCOUT        1 /* ADC12 Input Channel Select Bit 1 */
#define  A_VCCTARGET     2 /* ADC12 Input Channel Select Bit 2 */

#pragma push_macro("static")
#undef static
static constexpr float ADC_CONV_RANGE = 4096;
static constexpr uint16_t ADC_AVERAGE = 1000;
#pragma pop_macro("static")

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

const struct jtag _Jtag_SubMcu =
{
//  0,  // TCK, P4.4 (out) (high)
//  0,  // TMS, P4.5 (out) (high)
//  0,  // TDI, P4.6 (out) (high)
//  0,  // TDO, P4.7 (in)
//  (uint8_t*)&P6IN,  // JTAG input Register
//  (uint8_t*)&P6OUT, // JTAG output Register
//  (uint8_t)BIT6,   // BIT6.6 RST
//  (uint8_t)BIT7,   // BIT6.7 TST
//  (uint8_t*)&P6DIR // JTAG direction Register
};

//CONST_AT( struct jtag _Jtag, HAL_ADDR_CONST_JTAG) = {
const struct jtag _Jtag_Target =
{
//  (uint8_t)BIT4,  // TCK, P4.4 (out) (high)
//  (uint8_t)BIT5,  // TMS, P4.5 (out) (high)
//  (uint8_t)BIT6,  // TDI, P4.6 (out) (high)
//  (uint8_t)BIT7,  // TDO, P4.7 (in)
//  (uint8_t*)&P4IN,
//  (uint8_t*)&P4OUT,
//  (uint8_t)BIT2, //RST
//  (uint8_t)BIT3, //TST
//  (uint8_t*)&P4DIR
};

#define VALID_DATA   0x1A
#define SYNC_ONGOING 0x2A
#define INVALID_DATA 0x3A
#define JTAG_LOCKED  0x4A
#define SYNC_BROKEN  0x5A

#define BP_HIT_MASK_J              0x0400000000000000ull
#define LPMX5_MASK_J               0x4000000000000000ull
#define LPM4_1MASK_J               0x8000000000000000ull
#define LPM4_2MASK_J               0x8300000000000000ull
#define EIGHT_JSTATE_BITS          0x100000000000000ull

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
