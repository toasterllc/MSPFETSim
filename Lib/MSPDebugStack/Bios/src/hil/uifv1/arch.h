/**
* \ingroup MODULHIL
*
* \file arch.h
*
* \brief Important defines for the firmware
*
*/
/*
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
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

#include "msp430.h"

#ifndef _UIFV1_ARCH_H_
#define _UIFV1_ARCH_H_

    #define RSTHIGH 0
    #define RSTLOW  1

    #define DEFAULT_RSTDELAY 5

    enum {JTAG = 0, SPYBIWIRE = 1, SPYBIWIREJTAG = 2, JTAGUNDEF = 4, SPYBIWIRE_SUBMCU = 5, SPYBIWIRE_MSP_FET=6, JTAG_432 = 7, SWD_432 = 8};

    #define CRYSTAL         8000000         // externally connected HF crystal
    #define SETVFFREQ       100000          // pulse frequency to setup VF, this one's the fastest ~6ms settletime
    #define FLASHFREQ       450000
    #define TA_SETVF_DIV    CRYSTAL/SETVFFREQ/2
    #define TA_STROBE_DIV   CRYSTAL/FLASHFREQ
    #define TA_STROBE_DIV2  CRYSTAL/FLASHFREQ/2

    #define FREQUENCY		(CRYSTAL/1000)  // CPU frequency (master/slave) in KHz
    #define SEC_TIMEOUT     7
    #define DEF_TIMEOUT     (CRYSTAL*SEC_TIMEOUT)/13

    #define  VFCHN          2

    #define FET_TRUE                   0x1
    #define FET_FALSE                  0x0

    #define  ExtLimit        1700            // a level higher than this means external voltage available
    #define  ConvRange       4095.0f         // 12bit conversion range both for ADC and DAC
    #define  VCCTmin         ConvRange       // 0xFFF, write this value to the DAC register to setup minimum VCCT
    #define  Res1            60.4f           // FB2VCCT
    #define  Res2            30.1f           // FB2GND
    #define  Res3            39.2f           // FB2SETVCCT
    #define  Res4            30.1f           // VCCT2ADC1
    #define  Res5            22.1f           // ADC12GND
    #define  Vref            1224.0f         // the reference voltage of the voltage regulator
    #define  VCref           2500.0f         // conversion reference voltage
    #define  minVCCT         ((Res1*((Vref/Res2)-((VCref-Vref)/Res3)))+Vref)
    #define  maxVCCT         (((Res1*((Res2+Res3)/(Res2*Res3)))+1)*Vref)
    #define  VCCICHN         0
    #define  VCCTCHN         1
    #define  VCCRCHN         3

    #ifdef ARCH_MSP432

        struct jtag
        {
            unsigned char   TCK;
            unsigned char   TMS;
            unsigned char   TDI;
            unsigned char   TDO;
            unsigned char*  In;
            unsigned char*  Out;
            unsigned char*  DIRECTION;
            unsigned char   RST;
            unsigned char   _SELT;
            unsigned char*  RST_PORT;
            unsigned char*  RST_PORT_DIRECTION;
            unsigned char   _TST;
            unsigned char   _ENI2O;
            unsigned char   VF2TEST;
            unsigned char   VF2TDI;
            unsigned char   VCCTON;
            unsigned char   _TDIOFF;
            unsigned char*  TSTCTRL_PORT;
            unsigned char*  TSTCTRL_PORT_DIRECTION;
        };

        static const struct jtag _Jtag_Target =
        {
            (unsigned char) BIT3,        // TCK, P5.3 (out) (high)
            (unsigned char) BIT0,        // TMS, P5.0 (out) (high)
            (unsigned char) BIT1,        // TDI, P5.1 (out) (high)
            (unsigned char) BIT2,        // TDO, P5.2 (in)
            (unsigned char*)&P5IN,
            (unsigned char*)&P5OUT,
            (unsigned char*)&P5DIR,
            (unsigned char) BIT6,        // RST
            (unsigned char) BIT5,        // _selt
            (unsigned char*)&P2OUT,
            (unsigned char*)&P2DIR,
            (unsigned char) BIT0,       // _TST
            (unsigned char) BIT2,       // _ENI2O
            (unsigned char) BIT6,       // VF2TEST
            (unsigned char) BIT5,       // VF2TDI,
            (unsigned char) BIT3,       // VCCTON
            (unsigned char) BIT4,       // _TDIOFF
            (unsigned char*)&P4OUT,
            (unsigned char*)&P4DIR,
        };

    #endif

    #ifdef ARCH_MSP430

        struct jtag {
          unsigned char  TCK;
          unsigned char  TMS;
          unsigned char  TDI;
          unsigned char  TDO;
          unsigned char* In;
          unsigned char* Out;
        };

        extern const struct jtag _Jtag;
        // 4-wire JTAG: low level signal access
        #define TMSset1    { *_Jtag.Out |=  _Jtag.TMS; }
        #define TMSset0    { *_Jtag.Out &= ~_Jtag.TMS; }
        #define TDIset1    { *_Jtag.Out |=  _Jtag.TDI; }
        #define TDIset0    { *_Jtag.Out &= ~_Jtag.TDI; }
        #define TCKset1    { *_Jtag.Out |=  _Jtag.TCK; }
        #define TCKset0    { *_Jtag.Out &= ~_Jtag.TCK; }
        #define TCLKset1   { IHIL_Tclk(1); }
        #define TCLKset0   { IHIL_Tclk(0); }
        #define TCLK       { TCLKset0 TCLKset1 }
        #define TDIset1TMSset1    { *_Jtag.Out |=  _Jtag.TDI | _Jtag.TMS; }
        #define TDIset0TMSset1    { *_Jtag.Out &= ~_Jtag.TDI; *_Jtag.Out |= _Jtag.TMS;}


        // 2-wire Spy-Bi-Wire: low level signal access
        #define JTAGOUT (*_Jtag.Out)
        #define JTAGIN  (*_Jtag.In)
        #define sbwdato (_Jtag.TDI)
        #define sbwclk  (_Jtag.TCK)
        #define sbwdati (_Jtag.TDO)

        #define TEST           BIT0  // P4.0 (out) (high)
        #define TSTCTRLOUT     *tstctrl_port_
        #define TGTCTRLOUT     P2OUT
        #define ENTDI2TDO      entdi2tdo_
        #define SELT           BIT5  // P2.5 out (high) (aktiv low)
        #define TGTRST         BIT6  // P2.6 out (high) (aktiv low)

        #define RSTset1    {   if (gprotocol_id == SPYBIWIRE)                          \
                               {                                                       \
                                 { JTAGOUT |= sbwdato; }                               \
                               }                                                       \
                               else                                                    \
                               {                                                       \
                                 { TGTCTRLOUT &= ~SELT; TGTCTRLOUT |=  TGTRST; }       \
                               }                                                       \
                               IHIL_Delay_1ms(DEFAULT_RSTDELAY);                        \
                           }
        #define RSTset0    {   if (gprotocol_id == SPYBIWIRE)                          \
                               {                                                       \
                                 { JTAGOUT &= ~sbwdato; }                              \
                               }                                                       \
                               else                                                    \
                               {                                                       \
                                 { TGTCTRLOUT &= ~SELT; TGTCTRLOUT &= ~TGTRST; }       \
                               }                                                       \
                               IHIL_Delay_1ms(DEFAULT_RSTDELAY);                        \
                           }
        #define TSTset1    {   if (gprotocol_id == SPYBIWIRE)                          \
                               {                                                       \
                                 { JTAGOUT |= sbwclk;}                                \
                               }                                                       \
                               else                                                    \
                               {                                                       \
                                 { ((TSTCTRLOUT) &= (~TEST)); }                        \
                               }                                                       \
                               IHIL_Delay_1ms(DEFAULT_RSTDELAY);                        \
                           }
        #define TSTset0    {   if (gprotocol_id == SPYBIWIRE)                          \
                               {                                                       \
                                 {JTAGOUT &= ~sbwclk;  }                               \
                               }                                                       \
                               else                                                    \
                               {                                                       \
                                 { ((TSTCTRLOUT) |= (TEST)); }                      \
                               }                                                       \
                               IHIL_Delay_1ms(DEFAULT_RSTDELAY);                        \
                           }

        #define TMSL    JTAGOUT &= ~sbwdato; JTAGOUT &= ~sbwclk; JTAGOUT |= sbwclk;
        #define TMSLDH  JTAGOUT &= ~sbwdato; JTAGOUT &= ~sbwclk; JTAGOUT |= sbwdato; JTAGOUT |= sbwclk;
        #define TDIH    JTAGOUT |= sbwdato;  JTAGOUT &= ~sbwclk; JTAGOUT |= sbwclk;
        #define TDIL    JTAGOUT &= ~sbwdato; JTAGOUT &= ~sbwclk; JTAGOUT |= sbwclk;
        #define TDOsbw  TSTCTRLOUT ^= ENTDI2TDO; __disable_interrupt(); JTAGOUT &= ~sbwclk; JTAGOUT |= sbwclk; __enable_interrupt(); TSTCTRLOUT ^= ENTDI2TDO;
        #define TDO_RD  TSTCTRLOUT ^= ENTDI2TDO; __disable_interrupt(); JTAGOUT &= ~sbwclk; TDOvalue <<= 1; TDOvalue |= (JTAGIN & sbwdati) != 0; JTAGOUT |= sbwclk; __enable_interrupt(); TSTCTRLOUT ^= ENTDI2TDO;

        // Constants for VPP connection and process state at Blow-Fuse
        #define VPP_ON_TDI				0
        #define VPP_ON_TEST				1

        #define SETTDOIN    ((P4OUT) &= (~BIT2))
        #define SETTDOOUT   ((P4OUT) |= (BIT2))
        #define SETTDION    ((P4OUT) |= (BIT4))
        #define SETTDIOFF   ((P4OUT) &= (~BIT4))

        #define VPPon(x)		(x == VPP_ON_TEST ? (P4OUT |= BIT6) : (P4OUT |= BIT5))
        #define VPPoff()		((P4OUT) &= ~(BIT6 | BIT5))
        #define  VFCHN            2

        #define  STROBEOUT      BIT4  // P2.4 (out) (secondary function)
        #define  STROBEIN       BIT7  // P4.7 (in)  (secondary function)

        #define  VCCTON         BIT3  // P4.3 (out) (low)

        #define VCCTon()        ((TSTCTRLOUT) |= (VCCTON))
        #define VCCToff()       ((TSTCTRLOUT) &= (~VCCTON))

        #define    JSBWsbwdato   TGTRST
        #define    JSBsbwclk    TEST

        #define VALID_DATA   0x1A
        #define SYNC_ONGOING 0x2A
        #define INVALID_DATA 0x3A
        #define JTAG_LOCKED  0x4A

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

        #define L092_MODE 0xA55AA55A
        #define C092_MODE 0x5AA55AA5

    #endif

#endif
