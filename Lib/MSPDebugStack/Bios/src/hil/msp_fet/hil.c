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
//! \file hil.c
//! \brief
//!

//! \page hil_page1  (HIL)
//! \brief The HIL ...
//! \author  Berenbrinker Florian (01/01/2012)
//!
//! <b>files</b>\n
//! \li hil.c
//! \li hil_2w.c
//! \li hil_4w.c
//! \li hil_delays.s43
//! \li hil_delays.h
//! \li arch.h

#include <stdint.h>
#include "hw_compiler_specific.h"
#include "arch.h"
#include "hilDelays.h"

#include "hilFpgaAccess.h"
#include "archFpga.h"

#include "hilAdcControl.h"
#include "hilDacControl.h"

#include "JTAG_defs.h"
#include "hil_Structs.h"
#include "../../fw/fet/FetVersion.h"


//#define HIL_VERSION        0x000C
#define HIL_SIGNATURE      0xF00DF00Dul
#define EXTERNAL_VCC_ON    1
#define EXTERNAL_VCC_OFF   0

#define REGULATION_ON    1
#define REGULATION_OFF   0

//! \brief version of bios code
const unsigned short hil_version_ @ "HILVERSION" = MSP_FET_HIL_VERSION;
#pragma required=hil_version_

const unsigned short hil_versionCmp_ @ "HILVERSIONCMP" = MSP_FET_HIL_VERSION_CMP;
#pragma required=hil_versionCmp_

const unsigned long hil_signature_ @ "HILSIGNATURE" = HIL_SIGNATURE;
#pragma required=hil_signature_




unsigned short gTclkHighWhilePsa;
unsigned short setValueVcc;
unsigned short externalVccOn;
unsigned short regulationOn;

unsigned short jtagReleased;
unsigned short setPCclockBeforeCapture;

// function prototypes for map initialization
// common HIL configuration methods
short _hil_Init( void );
short _hil_SetVcc(unsigned short Vcc);
short _hil_GetVcc(double* Vcc, double* ExtVcc);
short _hil_SetProtocol(unsigned short protocol_id);
void  _hil_SetPsaSetup(unsigned short enhanced);
void  _hil_SetPsaTCLK(unsigned short tclkValue);

short _hil_Open( unsigned char state );
short _hil_Close( void );
short IccMonitor_Process(unsigned short flags); // flags: to be compatible with HIL calls
void  _hil_EntrySequences(unsigned char states);

void _hil_SetReset(unsigned char value);
void _hil_SetTest(unsigned char value);
void _hil_SetTMS(unsigned char value);
void _hil_SetTDI(unsigned char value);
void _hil_SetTCK(unsigned char value);

void _hil_SetJtagSpeed(unsigned short jtagSpeed, unsigned short sbwSpeed);
void _hil_ConfigureSetPc (unsigned short PCclockBeforeCapture);

void _hil_setDacValues(unsigned short dac0, unsigned short dac1);
short _hil_regulateVcc(void);
void _hil_switchVccFET(unsigned short switchVccFET);
short _hil_SetVcc(unsigned short Vcc);

unsigned long hil_Jtag_read_idcode();
unsigned long hil_swd_read_idcode();

// SBW4
extern short _hil_4w_TapReset(void);
extern short _hil_4w_CheckJtagFuse(void);
extern void  _hil_4w_StepPsa(unsigned long length);
extern void  _hil_4w_StepPsaTclkHigh(unsigned long length);
extern short _hil_4w_BlowFuse(unsigned char targetHasTestVpp);
extern unsigned char _hil_4w_Instr(unsigned char Instruction);

// SBW2 DMA
extern short _hil_2w_TapReset_Dma(void);
extern short _hil_2w_CheckJtagFuse_Dma(void);
extern unsigned char _hil_2w_Instr_Dma(unsigned char Instruction);
extern unsigned char _hil_2w_SetReg_XBits08_Dma(unsigned char Data);
extern unsigned short _hil_2w_SetReg_XBits16_Dma(unsigned short Data);
extern unsigned long _hil_2w_SetReg_XBits20_Dma(unsigned long Data);
extern unsigned long _hil_2w_SetReg_XBits32_Dma(unsigned long Data);
extern unsigned long long _hil_2w_SetReg_XBits64_Dma(unsigned long long Data);
extern unsigned long long _hil_2w_SetReg_XBits8_64_Dma(unsigned long long Data, unsigned short loopCount, unsigned short PG);
extern void _hil_2w_Tclk_Dma(unsigned char state);
extern void _hil_2w_StepPsa_Dma(unsigned long length);
extern void _hil_2w_StepPsa_Dma_Xv2(unsigned long length);
extern void _hil_2w_StepPsaTclkHigh_Dma(unsigned long length);
extern short _hil_2w_BlowFuse_Dma(unsigned char targetHasTestVpp);
extern void _hil_2w_ConfigureSpeed_Dma(unsigned short speed);
extern unsigned char _hil_2w_GetPrevInstruction_Dma();
extern unsigned char DMA_TMSL_TDIL[];
extern void setProtocolSbw2Dma(unsigned short id);

short _hil_dummy_TapReset(void) {return 0;}
short _hil_dummy_CheckJtagFuse(void){return 0;}
unsigned char _hil_dummy_Instr(unsigned char Instruction){return 0;}
unsigned char _hil_dummy_SetReg_XBits08(unsigned char Data){return 0;}
unsigned char _hil_dummy_Instr_4(unsigned char Data){return 0;}

unsigned short _hil_dummy_SetReg_XBits16(unsigned short Data){return 0;}
unsigned long _hil_dummy_SetReg_XBits20(unsigned long Data){return 0;}
unsigned long _hil_dummy_SetReg_XBits32(unsigned long Data){return 0;}
unsigned long long _hil_dummy_SetReg_XBits64(unsigned long long Data){return 0;}
unsigned long long _hil_dummy_SetReg_XBits8_64(unsigned long long Data, unsigned short loopCount, unsigned short JStateVersion){return 0;}
unsigned long long _hil_dummy_SetReg_XBits(unsigned long long *data, unsigned short count){return 0;}
unsigned long long _hil_dummy_SetReg_XBits35(unsigned long long *data){return 0;}

void _hil_dummy_Tclk(unsigned char state){return;}
void _hil_dummy_StepPsa(unsigned long length){return;}
short _hil_dummy_BlowFuse(unsigned char targetHasTestVpp){return 0;}
unsigned char _hil_dummy_GetPrevInstruction(){return 0;}

short _hil_dummy_Write_Read_Ap(unsigned long address ,unsigned long *data, unsigned short rnw){return -1;};
short _hil_dummy_Write_Read_Dp(unsigned char address ,unsigned long *data, unsigned short rnw){return -1;};
short _hil_dummy_Write_Read_Mem_Ap(unsigned short ap_sel, unsigned long address, unsigned long *data, unsigned short rnw){return -1;};
unsigned long _hil_read_idcodeDummy(){return -1;}
unsigned char _hil_dummy_TransferData(unsigned char regiser, unsigned long* data, unsigned char rnw){return -1;};
void _hil_dummy_SetToolID(unsigned short id) {return;}

// FPGA generic 430
extern void initGeneric();
extern unsigned char _hil_generic_Instr(unsigned char Instruction);

extern unsigned char _hil_generic_SetReg_XBits08(unsigned char Data);
extern unsigned short _hil_generic_SetReg_XBits16(unsigned short Data);
extern unsigned long _hil_generic_SetReg_XBits20(unsigned long Data);
extern unsigned long _hil_generic_SetReg_XBits32(unsigned long Data);
extern unsigned long long _hil_generic_SetReg_XBits64(unsigned long long Data);
extern unsigned long long _hil_generic_XBits8_64(unsigned long long Data, unsigned short loopCount, unsigned short PG);
extern void  _hil_generic_Tclk(unsigned char state);
extern void  _hil_generic_ConfigureSpeed(unsigned short speed);
extern unsigned char _hil_generic_GetPrevInstruction();

// FPGA generic 432
extern unsigned char _hil_generic432_Instr_4(unsigned char Data);
//extern unsigned long long _hil_generic432_SetReg_XBits(unsigned long long *data, unsigned short count);
extern unsigned char _hil_generic432_SetReg_XBits08(unsigned char Data);
extern unsigned short _hil_generic432_SetReg_XBits16(unsigned short Data);
extern unsigned long _hil_generic432_SetReg_XBits32(unsigned long Data);
extern unsigned long long _hil_generic432_SetReg_XBits64(unsigned long long Data);
extern unsigned long long _hil_generic432_SetReg_XBits35(unsigned long long *data);

extern void hil_4w_432_Seq(unsigned short length, unsigned char *sequence);

// FPGA generic
extern unsigned short hil_fpga_get_version(void);
extern void _hil_FpgaAccess_setTimeOut(unsigned short state);

// protocol specific methods
// must be implemeted in SWD mode
extern void hil_Swd_InitJtag(struct jtag tmp);
extern void hil_Swd_Seq(unsigned short length, unsigned char *sequence);
extern unsigned char Swd_TransferData(unsigned char regiser, unsigned long* data, unsigned char rnw);

extern void SWDTCKset1();
extern void SWDTCKset0();

void SWD_Entry();
void JTAG_Entry();

short _hil_Write_Read_Ap_Jtag(unsigned long address ,unsigned long *data, unsigned short rnw);
short _hil_Write_Read_Dp_Jtag(unsigned char address ,unsigned long *data, unsigned short rnw);
short _hil_Write_Read_Mem_Ap_Jtag(unsigned short ap_sel, unsigned long address, unsigned long *data, unsigned short rnw);

short _hil_Write_Read_Dp_Swd(unsigned char address ,unsigned long *data, unsigned short rnw);
short _hil_Write_Read_Ap_Swd(unsigned long address, unsigned long *data, unsigned short rnw);
short _hil_Write_Read_Mem_Ap_Swd(unsigned short ap_sel, unsigned long address, unsigned long *data, unsigned short rnw);

void _hil_Release(void);
void  hil_initTimerA2(void);
void _hil_BSL_EntrySequence(unsigned short);
void _hil_BSL_EntrySequence1xx_4xx();
void _hil_ReadADC12(void);
void _hil_ConfigFpgaIoMode(unsigned short mode);

static void _hil_Connect(unsigned char state);

extern void initJtagSbw2Dma(struct jtag tmp);
extern void initJtagSbw4(struct jtag tmp);

edt_common_methods_t   _Common_Methods;
edt_distinct_methods_t _Distinct_Methods;

static struct jtag _Jtag =
{
  0,  // TCK, P4.4 (out) (high)
  0,  // TMS, P4.5 (out) (high)
  0,  // TDI, P4.6 (out) (high)
  0,  // TDO, P4.7 (in)
  0,
  0,
  0, //RST
  0, //TST
  0
};

// fuse blow control
static struct vfuse_ctrl _VFuse =
{
  0,    // VF2TEST
  0,    // VF2TDI
  0,    // TDIOFF
  0,    // VF control register
  0,    // PWM_SETVF
  0,    // VF PWM control register
};

unsigned short gprotocol_id;
static unsigned short hil_sbw2Speed_;
static unsigned short hil_jtagSpeed_;
static short dtVccSenseState = 0;

static volatile unsigned char useTDI = 0;

#pragma inline=forced
void RSTset1()
{
    { (*_Jtag.Out) |= _Jtag.RST; }
    _hil_Delay_1ms(5);
}
#pragma inline=forced
void RSTset0()
{
    { (*_Jtag.Out) &= ~_Jtag.RST; }
     _hil_Delay_1ms(5);
}

#pragma inline=forced
void TMSset1()
{
    { (*_Jtag.Out) |= _Jtag.TMS; }
    _hil_Delay_1ms(5);
}
#pragma inline=forced
void TMSset0()
{
    { (*_Jtag.Out) &= ~_Jtag.TMS; }
     _hil_Delay_1ms(5);
}

#pragma inline=forced
void TCKset1()
{
    { (*_Jtag.Out) |= _Jtag.TCK; }
    _hil_Delay_1ms(5);
}
#pragma inline=forced
void TCKset0()
{
    { (*_Jtag.Out) &= ~_Jtag.TCK; }
     _hil_Delay_1ms(5);
}

#pragma inline=forced
void TCKset1NoDelay()
{
    { (*_Jtag.Out) |= _Jtag.TCK; }
}

#pragma inline=forced
void TCKset0NoDelay()
{
    { (*_Jtag.Out) &= ~_Jtag.TCK; }
}

#pragma inline=forced
void TDIset1()
{
    { (*_Jtag.Out) |= _Jtag.TDI; }
    _hil_Delay_1ms(5);
}
#pragma inline=forced
void TDIset0()
{
    { (*_Jtag.Out) &= ~_Jtag.TDI; }
     _hil_Delay_1ms(5);
}

#pragma inline=forced
void TSTset1()
{
    { (*_Jtag.Out) |= _Jtag.TST;}
    _hil_Delay_1ms(5);
}
#pragma inline=forced
void TSTset0()
{
    {(*_Jtag.Out) &= ~_Jtag.TST; }
    _hil_Delay_1ms(5);
}
#pragma inline=forced
void TSTset1NoDelay()
{
    { (*_Jtag.Out) |= _Jtag.TST;}
}
#pragma inline=forced
void TSTset0NoDelay()
{
    {(*_Jtag.Out) &= ~_Jtag.TST; }
}

//#pragma inline=forced
void TCLKset1()
{
   _Distinct_Methods.Tclk(1);
}

//#pragma inline=forced
void TCLKset0()
{
   _Distinct_Methods.Tclk(0);
}
#pragma inline=forced
void TCLK()
{
    TCLKset0();
    TCLKset1();
}

static void _hil_initEdtCommenMethods()
{
    _Common_Methods.Init = _hil_Init;

    _Common_Methods.SetVcc = _hil_SetVcc;
    _Common_Methods.SwitchVccFET = _hil_switchVccFET;
    _Common_Methods.GetVcc = _hil_GetVcc;

    _Common_Methods.SetProtocol = _hil_SetProtocol;
    _Common_Methods.SetPsaTCLK = _hil_SetPsaTCLK;

    _Common_Methods.Open = _hil_Open;
    _Common_Methods.Close = _hil_Close;

    _Common_Methods.Delay_1us = _hil_Delay_1us;
    _Common_Methods.Delay_1ms = _hil_Delay_1ms;

    _Common_Methods.Loop = 0;

    _Common_Methods.EntrySequences = _hil_EntrySequences;

    _Common_Methods.SetReset = _hil_SetReset;
    _Common_Methods.SetTest  = _hil_SetTest;
    _Common_Methods.SetTMS = _hil_SetTMS;
    _Common_Methods.SetTCK  = _hil_SetTCK;
    _Common_Methods.SetTDI  = _hil_SetTDI;

    _Common_Methods.SetJtagSpeed = _hil_SetJtagSpeed;

    _Common_Methods.ConfigureSetPc = _hil_ConfigureSetPc;
    _Common_Methods.initDelayTimer = hil_initTimerA2;

    _Common_Methods.BSL_EntrySequence = _hil_BSL_EntrySequence;
    _Common_Methods.BSL_EntrySequence1xx_4xx = _hil_BSL_EntrySequence1xx_4xx;
    _Common_Methods.SetToolID = _hil_dummy_SetToolID;


    _Common_Methods.regulateVcc =  _hil_regulateVcc;
    _Common_Methods.setFpgaTimeOut = _hil_FpgaAccess_setTimeOut;
    _Common_Methods.getFpgaVersion = hil_fpga_get_version;
    _Common_Methods.ReadADC12 = _hil_ReadADC12;
    _Common_Methods.ConfigFpgaIoMode = _hil_ConfigFpgaIoMode;
}

void _hil_getEdtDistinct(edt_distinct_methods_t* edt_distinct);
void _hil_getEdtCommen(edt_common_methods_t* edt_commen);
void _hil_switchVccFET(unsigned short switchVccFET);

#pragma required=_hil_getEdtDistinct
#pragma required=_hil_getEdtCommen

void _hil_startUp()
{
    _hil_initEdtCommenMethods();
    _Common_Methods.Init();
    
    // DMA4 workaround
    DMACTL4 |= DMARMWDIS;
    
    return;
}
#pragma location="INFO_C_DISTINCT"
void _hil_getEdtDistinct(edt_distinct_methods_t* edt_distinct)
{
    if(edt_distinct == 0)
    {
      return;
    }
    
    *edt_distinct = _Distinct_Methods;
}

#pragma location="INFO_C_COMMEN"
void _hil_getEdtCommen(edt_common_methods_t* edt_commen)
{
    if(edt_commen == 0)
    {
      return;
    }
    
    *edt_commen = _Common_Methods;
}

void hil_initFuseBlow(struct vfuse_ctrl tmp)
{
    _VFuse = tmp;
}

void hil_initTimerB0(void)
{
    // Setup timer_B0 for current pulse measurement
    TB0CTL = MC__STOP;
    TB0CTL = TBSSEL__TBCLK + MC__CONTINUOUS + TBIE;
}

// -----------------------------------------------------------------------------
void hil_initTimerA0(void)
{
    // Setup time_A0 for timestamping
    TA0CTL = MC__STOP;                                  // STOP Timer
    TA0CTL = ID__8 + TASSEL__SMCLK;                     // Timer_A0 source:SMCLK/8 = 20 / 8 = 2.500 MHz = 1/400ns
    TA0EX0 = 1;                                         // Timer_A0 set additional divider to /2
    TA0CTL |= TACLR + MC__CONTINOUS + TAIE;             // START the timer in free-running mode
}

// -----------------------------------------------------------------------------
void hil_initTimerA2(void)
{
    // Setup timer_A2 for hardware delay
    TA2CTL = 0;                                          // STOP Timer
    TA2CTL =  ID__8 +TASSEL__SMCLK;                      // Timer_B source:SMCLK ,SMCLK/0 = 20
    TA2CCR0 = 0x8A0;                                     // Load CCR0 with delay... (1ms delay)
    TA2CCTL0 &= ~CCIE;
}

#pragma inline=forced
void qDriveJTAG(void)
{
    (*_Jtag.Out) |= (_Jtag.TCK + _Jtag.TMS + _Jtag.TDI + _Jtag.RST );
    (*_Jtag.Out) &= (~_Jtag.TST);
    (*_Jtag.DIRECTION) |= (_Jtag.TCK + _Jtag.TMS + _Jtag.TDI);
    (*_Jtag.DIRECTION) |= (_Jtag.TST + _Jtag.RST);
    (*_Jtag.DIRECTION) &= (~_Jtag.TDO);
}

#pragma inline=forced
void qDriveSbw(void)
{
    (*_Jtag.Out) |= _Jtag.RST;
    (*_Jtag.Out) &= ~_Jtag.TST;
    (*_Jtag.DIRECTION) |= ( _Jtag.TST +  _Jtag.RST);
}

#pragma inline=forced
void qDriveSwd(void)
{
   (*_Jtag.Out) |= _Jtag.TCK;
   (*_Jtag.Out) &= ~_Jtag.TMS;
   (*_Jtag.DIRECTION) |= ( _Jtag.TCK + _Jtag.TMS);
}

void _hil_ConfigFpgaIoMode(unsigned short mode)
{
    hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_TARGET_IO_CONFIGURATION, mode);
}


void _hil_BSL_EntrySequence1xx_4xx()
{
    hil_fpga_enable_bypass();

    if(gprotocol_id == SPYBIWIRE)
    {
        qDriveSbw();
    }
    else
    {
        qDriveJTAG();
    }

    RSTset0();    // set RST 0
    // INIT phase
    TSTset0();
    _hil_Delay_1ms(100);

    TSTset1();    // set test to 1
    _hil_Delay_1ms(100);

    //this is the first pulse keep it low for less than 15us for 5xx
    TSTset0NoDelay();

    TCKset0NoDelay();
    _hil_Delay_1us(5);

    TCKset1NoDelay();
    _hil_Delay_1us(5);

    TSTset1();     // set test 1;

    _hil_Delay_1ms(50);
    TCKset0();

    RSTset1();     // set RST 1;
    _hil_Delay_1ms(50);

    TCKset1();

    TSTset0();     // set test 0;
    _hil_Delay_1ms(50);

    hil_fpga_disable_bypass();
}


void _hil_BSL_EntrySequence(unsigned short switchBypassOff)
{
    hil_fpga_enable_bypass();

    if(gprotocol_id == SPYBIWIRE)
    {
        qDriveSbw();
    }
    else
    {
        qDriveJTAG();
    }

    RSTset0();    // set RST 0
    // INIT phase
    TSTset0();
    _hil_Delay_1ms(100);

    TSTset1();    // set test to 1
    _hil_Delay_1ms(100);

    //this is the first pulse keep it low for less than 15us for 5xx
    TSTset0NoDelay();
    if(!switchBypassOff)
    {
        TCKset0NoDelay();
        _hil_Delay_1us(5);
    }
    else
    {
        _hil_Delay_1us(10);
    }

    if(!switchBypassOff)
    {
        TCKset1NoDelay();
        _hil_Delay_1us(5);
    }
    TSTset1();     // set test 1;

    _hil_Delay_1ms(50);
    if(!switchBypassOff)
    {
        TCKset0();
    }
    RSTset1();     // set RST 1;
    _hil_Delay_1ms(50);
    if(!switchBypassOff)
    {
        TCKset1();
    }
    TSTset0();     // set test 0;
    _hil_Delay_1ms(50);

    if(switchBypassOff)
    {
        hil_fpga_disable_bypass();
    }
}


// -----------------------------------------------------------------------------

short _hil_Init( void )
{
    hil_initTimerA0();              // TimeStamp
    hil_initTimerA2();              // Current pulse counter
    hil_initTimerB0();              // Delay loop timer

    hil_DacControlInitDAC0();
    hil_DacControlInitDAC1();
    hil_AdcControlInitADC();

    DMACTL2 = 0;
    DMACTL1 = 0;

    // default config
    externalVccOn = EXTERNAL_VCC_OFF;
    regulationOn = REGULATION_OFF;
    dtVccSenseState = 0;

   // set default to TCLK low when doing PSA
    gTclkHighWhilePsa = 0;
    // initialize function pointers to distinct functions

    // initialize FPGA
    hil_fpga_init();
    // set default debug protocol to JTAG
    gprotocol_id = SPYBIWIREJTAG;
    // set default to TCLK low when doing PSA
    gTclkHighWhilePsa = 0;
    // initialize function pointers to distinct functions
    _hil_SetProtocol(SPYBIWIREJTAG);
    jtagReleased = 1;

    // initialize fuse blow control
    hil_initFuseBlow(_Msp_Fet);

    // Default SBW2 speed --> 60MHz/(98+2) = 600 kHz
    hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_TEST_CLK_FREQUENCY, 35);
    hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_TCLK_CLK_FREQUENCY, 35);
    // set Io config to IO_CONFIG_HIGH_Z_UART
    _hil_ConfigFpgaIoMode(0x0);
    return 0;
}


short _hil_regulateVcc(void)
{
    volatile unsigned short externalVcc = 0, dtVcc = 0;
    volatile short actualDeviation = 0;

    externalVcc = hil_AdcControlGetExternalVcc();
    dtVcc  = hil_AdcControlGetVccDt();

    if(dtVccSenseState)
    {
        P8OUT |= BIT0;
        _hil_Delay_1ms(500);
        dtVccSenseState = 0;
    }

    if(hil_AdcControlGetDtVccSense() > 280 )
    {
        P8OUT &= ~BIT0;
        dtVccSenseState = 1;
        _hil_Delay_1ms(1000);
        return 1;
    }

    if((externalVcc < 1600 || externalVcc > 3800 )&& externalVccOn)
    {
        externalVccOn = EXTERNAL_VCC_OFF;
        return 0;
    }

    if(externalVcc > 1600 && externalVcc < 3800)
    {
        // interal target VCC set 0
        hil_DacControlSetDAC1(0);
        setValueVcc = externalVcc;
        externalVccOn = EXTERNAL_VCC_ON;
        regulationOn = REGULATION_ON;
    }

    // if VCC or no External VCC measured , do not regulate
    if(!regulationOn || !externalVccOn)
    {
        return 0;
    }
    // JAG VCC - I - part  - must be regulated with external and internal VCC to keep the same level
    if(dtVcc < (setValueVcc - 10))
    {
        actualDeviation = setValueVcc  - dtVcc;
        if(DAC12_0DAT > 0)
        {
            if(actualDeviation > 500)
            {
                 hil_DacControlSetDAC0(DAC12_0DAT - 100);
            }
            else if(actualDeviation > 100)
            {
                 hil_DacControlSetDAC0(DAC12_0DAT - 10);
            }
            else if(actualDeviation > 20)
            {
                 hil_DacControlSetDAC0(DAC12_0DAT - 5);
            }
            else
            {
                hil_DacControlSetDAC0(DAC12_0DAT - 1);
            }
        }
    }
    else if(dtVcc > (setValueVcc + 10))
    {
        actualDeviation = dtVcc - setValueVcc;

        if(DAC12_0DAT <= 0xD00)
        {
            if(actualDeviation > 500)
            {
                 hil_DacControlSetDAC0(DAC12_0DAT + 100);
            }
            else if(actualDeviation > 100)
            {
                 hil_DacControlSetDAC0(DAC12_0DAT + 10);
            }
            else if(actualDeviation > 20)
            {
                 hil_DacControlSetDAC0(DAC12_0DAT + 5);
            }
            else
            {
                hil_DacControlSetDAC0(DAC12_0DAT + 1 );
            }
        }
    }

    return 0;
}

void _hil_setDacValues(unsigned short dac0, unsigned short dac1)
{
    hil_DacControlSetDAC1(dac1);
    hil_DacControlSetDAC0(dac0);
}

void _hil_ReadADC12(void)
{
    hil_AdcControlRead();
}

void _hil_switchVccFET(unsigned short switchVccFET)
{
    if(switchVccFET)
    {
        hil_fpga_enable_bypass();
        //This workaround is necessary to set TCK & TEST to a defined low level before
        //applying Vcc to the target device
        //otherwise the target will not power up correctly and the UIF can't
        //establish a SBW communication
        TSTset0();
        (*_Jtag.Out) &= ~(_Jtag.TCK );
        (*_Jtag.DIRECTION) |= (_Jtag.TCK);

        // During USB power up, TDI and TMS output pins need to be put to Hi-Z state.
        // configuer level shifter as input - Hi-Z
        (*_Jtag.Out) &= ~(_Jtag.TDI +  _Jtag.TMS);
        (*_Jtag.DIRECTION) &=  ~(_Jtag.TDI + _Jtag.TMS);
        FPGA_BYPASS_DIR_CTRL_PORT_OUT &= ~(_Jtag.TDI + _Jtag.TMS);

        P8DIR |= (BIT0 + BIT7);
        P8OUT |= (BIT0 + BIT7);

        _hil_Delay_1ms(10);
        hil_fpga_disable_bypass();
    }
    else
    {
        P8OUT &= ~(BIT0 + BIT7);
    }
}

savedDacValues_t _savedDacValues;

#pragma optimize = low
short _hil_SetVccSupplyDt(unsigned short Vcc)
{
    int correctVccDt = 0;
    int maxCount = 400;
    short actualDeviation = 0;

    if(Vcc < 2200)
    {
       maxCount = 800;
    }

    ADC12IE = 0;
    hil_DacControlSetDAC0((unsigned short)(((float)((float)setValueVcc/(float)1000)*(-1707) + 6145)));

    if(DAC12_0DAT > 0xD00)
    {
        DAC12_0DAT = 0xD00;
    }

    while( correctVccDt < 3 && maxCount-- > 0)
    {
        unsigned short dtVcc = 0;
        actualDeviation = 0;
        {
            int i = 0;
            while(i++ < 43)
            {
                ADC12CTL0 |= ADC12SC;       // Sampling and conversion start
                hil_AdcControlRead();
                ADC12CTL0 &= ~ADC12SC;
                _hil_Delay_1us(20);
            }
        }
        dtVcc   = hil_AdcControlGetVccDt();
        // JAG VCC - I - part  - must be regulated with external and internal VCC to keep the same level
        if(dtVcc < (setValueVcc - 10))
        {
            actualDeviation = setValueVcc  - dtVcc;
            if(actualDeviation > 500 && ((DAC12_0DAT - 100) > 0))
            {
                 hil_DacControlSetDAC0(DAC12_0DAT - 100);
            }
            else if(actualDeviation > 100 && ((DAC12_0DAT - 30) > 0))
            {
                 hil_DacControlSetDAC0(DAC12_0DAT - 30);
            }
            else if(actualDeviation > 20 && ((DAC12_0DAT - 5) > 0))
            {
                 hil_DacControlSetDAC0(DAC12_0DAT - 5);
            }
            else if((DAC12_0DAT - 1) > 0)
            {
                hil_DacControlSetDAC0(DAC12_0DAT - 1);
            }
        }
        else if(dtVcc > (setValueVcc + 10))
        {
            actualDeviation = dtVcc - setValueVcc;

            if(actualDeviation > 500 && ((DAC12_0DAT + 100) <= 0xD00))
            {
                 hil_DacControlSetDAC0(DAC12_0DAT + 100);
            }
            else if(actualDeviation > 100 && ((DAC12_0DAT + 30) <= 0xD00))
            {
                 hil_DacControlSetDAC0(DAC12_0DAT + 30);
            }
            else if(actualDeviation > 20&& ((DAC12_0DAT + 5) <= 0xD00))
            {
                 hil_DacControlSetDAC0(DAC12_0DAT + 5);
            }
            else if((DAC12_0DAT + 1) <= 0xD00)
            {
                hil_DacControlSetDAC0(DAC12_0DAT + 1 );
            }
        }
        else
        {
            correctVccDt++;
        }
    }

    ADC12IE = 0x08;// Enable ADC12IFG.2
    return (correctVccDt > 2) ? 1 : 0;
}

#pragma optimize = low
short _hil_SetVccSupply(unsigned short Vcc)
{
    int correctVcc = 0;
    int maxCount = 400;
    short actualDeviation = 0;
    setValueVcc = Vcc;

    if(Vcc < 2400)
    {
       maxCount = 800;
    }

    ADC12IE = 0;
    hil_DacControlSetDAC1((unsigned short)(((float)((float)setValueVcc/(float)1000)*820)));

    // check if p regulate values are out of range - if yes correct them
    if(DAC12_1DAT > 0xC10)
    {
      DAC12_1DAT = 0xC10;
    }
    else if( DAC12_1DAT <= 0x550)
    {
        DAC12_1DAT = 0x552;
    }
    // integrnal target VCC - I part of regulation - small delta VCC
    while(correctVcc < 3 && maxCount-- > 0)
    {
        unsigned short vccSupply = 0;
        actualDeviation = 0;
        {
            int i = 0;
            while(i++ < 43)
            {
                ADC12CTL0 |= ADC12SC;       // Sampling and conversion start
                hil_AdcControlRead();
                ADC12CTL0 &= ~ADC12SC;
                _hil_Delay_1us(20);
            }
        }
        vccSupply = hil_AdcControlGetSupplyVcc();
        // VCC supply ------------------------------------------------------
        //negative deviation
        if(vccSupply < (setValueVcc - 10))
        {
            correctVcc = 0;
            actualDeviation = setValueVcc - vccSupply;

            if(actualDeviation > 500 && ((DAC12_1DAT + 100) <= 0xC10))
            {
                hil_DacControlSetDAC1(DAC12_1DAT + 100);
            }
            else if(actualDeviation > 100 && ((DAC12_1DAT + 30) <= 0xC10))
            {
                hil_DacControlSetDAC1(DAC12_1DAT + 30);
            }
            else if(actualDeviation > 20 && ((DAC12_1DAT + 5) <= 0xC10))
            {
                hil_DacControlSetDAC1(DAC12_1DAT + 5);
            }
            else if((DAC12_1DAT + 1) <= 0xC10)
            {
                hil_DacControlSetDAC1(DAC12_1DAT + 1);
            }
        }
        //positve deviation
        else if(vccSupply > (setValueVcc + 10))
        {
            actualDeviation = vccSupply - setValueVcc;
            correctVcc = 0;

            if(actualDeviation > 500 && ((DAC12_1DAT - 100) > (0x550)))
            {
                hil_DacControlSetDAC1(DAC12_1DAT -100);
            }
            else if(actualDeviation > 100 && ((DAC12_1DAT - 30) > (0x550)))
            {
                hil_DacControlSetDAC1(DAC12_1DAT - 30);
            }
            else if(actualDeviation > 20 && ((DAC12_1DAT - 5) > (0x550)))
            {
                hil_DacControlSetDAC1(DAC12_1DAT - 5);
            }
            else if(((DAC12_1DAT - 1) > (0x550)))
            {
                hil_DacControlSetDAC1(DAC12_1DAT - 1);
            }
        }
        else
        {
            correctVcc++;
        }
        //-----------------------------------------------------------------
    }
    ADC12IE = 0x08;// Enable ADC12IFG.2

    return (correctVcc > 2) ? 1 : 0;
}

#pragma optimize = low
// -----------------------------------------------------------------------------
short _hil_SetVcc(unsigned short Vcc)
{
    ADC12IE = 0;
    if(Vcc < 1600 || Vcc > 3800)
    {
        //Turn off the Power switch
        //Vcc Supply
        _hil_Delay_1ms(1);
        _hil_Release();
        setValueVcc = 0;
        externalVccOn = EXTERNAL_VCC_OFF;
        regulationOn = REGULATION_OFF;
        ADC12IE = 0x08;
        return 0;
    }
    if(_savedDacValues.Vcc == Vcc && _savedDacValues.valid == 1)
    {
        hil_DacControlSetDAC1(_savedDacValues.DAC1);
        hil_DacControlSetDAC0(_savedDacValues.DAC0);
        ADC12IE = 0x08;// Enable ADC12IFG.2
        return 0;
    }
    if((!_hil_SetVccSupply(Vcc)) || (!_hil_SetVccSupplyDt(Vcc)))
    {
        _hil_SetVccSupply(Vcc);
        _hil_SetVccSupplyDt(Vcc);
    }

    _savedDacValues.DAC0 = DAC12_0DAT;
    _savedDacValues.DAC1 = DAC12_1DAT;
    _savedDacValues.Vcc = Vcc;
    _savedDacValues.valid = 1;

    externalVccOn = EXTERNAL_VCC_OFF;
    return 0;
}
// -----------------------------------------------------------------------------

short _hil_GetVcc(double* Vcc, double* ExtVcc)
{
    float vccSupply = 0, externalVcc = 0;

    externalVcc = hil_AdcControlGetExternalVcc();
    vccSupply = hil_AdcControlGetSupplyVcc();

    if(externalVcc >= 1650 && externalVcc < 3800)
    {
        setValueVcc = (unsigned short)externalVcc;
        externalVccOn = EXTERNAL_VCC_ON;
        regulationOn = REGULATION_ON;

        if(gprotocol_id != SWD_432)
        {
            hil_fpga_enable_bypass();
        }

        P8DIR |= (BIT0);
        P8OUT |= (BIT0);

        _hil_Delay_1ms(10);
        if(gprotocol_id != SWD_432)
        {
            hil_fpga_disable_bypass();
        }
    }
    else
    {
        externalVccOn = EXTERNAL_VCC_OFF;
    }
    *Vcc = vccSupply;
    *ExtVcc = externalVcc;
    return 0;
}

#pragma optimize = low
// -----------------------------------------------------------------------------
short _hil_SetProtocol(unsigned short protocol_id)
{
    short ret_value = 0;

    if( protocol_id == SPYBIWIRE )
    {
        gprotocol_id = SPYBIWIRE;
        _Jtag = _SBW_Back;
        initJtagSbw2Dma(_Jtag);
        initJtagBypass(_Jtag);
        initGeneric();
        _hil_2w_ConfigureSpeed_Dma(SBW100KHz);
    }
    else if( protocol_id == SPYBIWIREJTAG || protocol_id == JTAG || protocol_id == JTAG_432)
    {
        gprotocol_id = protocol_id;
        _Jtag = _Jtag_Target;
        initJtagSbw4(_Jtag);
        initJtagBypass(_Jtag);
        initGeneric();
    }
    else if(protocol_id == SWD_432)
    {
        gprotocol_id = protocol_id;
        _Jtag = _Jtag_Target;
        hil_Swd_InitJtag(_Jtag);
        initJtagBypass(_Jtag);
        initGeneric();
    }
    else if( protocol_id == SPYBIWIRE_SUBMCU )
    {
        _Jtag = _Jtag_SubMcu;
        initJtagSbw2Dma(_Jtag);
        gprotocol_id = SPYBIWIRE_SUBMCU;
        setProtocolSbw2Dma(SPYBIWIRE_SUBMCU);
        _hil_2w_ConfigureSpeed_Dma(SBW600KHz);
    }
    else if( protocol_id == SPYBIWIRE_MSP_FET)
    {
        gprotocol_id = protocol_id;
        _Jtag = _Jtag_Target;
        initJtagSbw2Dma(_Jtag);
        initJtagBypass(_Jtag);
        initGeneric();
        _hil_2w_ConfigureSpeed_Dma(SBW100KHz);
    }
    else
    {
        ret_value = -1;
    }

    if( protocol_id == SPYBIWIRE || protocol_id == SPYBIWIRE_SUBMCU || protocol_id == SPYBIWIRE_MSP_FET)
    {
        // load DMA1 with size just default
        DMA1CTL = ( DMADT0 | DMASRCINCR1 | DMASRCINCR0 | DMASRCBYTE | DMADSTBYTE);
        DMA1DA =  (_Jtag.Out); //JTAGOUT;       // set destination address
        DMA2CTL = ( DMADT0 | DMASRCINCR1 | DMASRCINCR0 | DMASRCBYTE | DMADSTBYTE);
        DMA2DA =  (_Jtag.Out); //JTAGOUT;       // set destination address

        // Functionality executed in FPGA bypass mode by Sub-MCU
        _Distinct_Methods.TapReset =            _hil_2w_TapReset_Dma;
        _Distinct_Methods.CheckJtagFuse =       _hil_2w_CheckJtagFuse_Dma;

        // SBW communication with Sub-MCU is executed in Bit-Banging mode
        if ( protocol_id == SPYBIWIRE_SUBMCU )
        {
            _Distinct_Methods.Instr =               _hil_2w_Instr_Dma;
            _Distinct_Methods.SetReg_XBits08 =      _hil_2w_SetReg_XBits08_Dma;
            _Distinct_Methods.SetReg_XBits16 =      _hil_2w_SetReg_XBits16_Dma;
            _Distinct_Methods.SetReg_XBits20 =      _hil_2w_SetReg_XBits20_Dma;
            _Distinct_Methods.SetReg_XBits32 =      _hil_2w_SetReg_XBits32_Dma;
            _Distinct_Methods.SetReg_XBits64 =      _hil_2w_SetReg_XBits64_Dma;
            _Distinct_Methods.SetReg_XBits8_64 =    _hil_2w_SetReg_XBits8_64_Dma;
            _Distinct_Methods.Tclk =                _hil_2w_Tclk_Dma;
            _Distinct_Methods.GetPrevInstruction =  _hil_2w_GetPrevInstruction_Dma;
            _Distinct_Methods.SetReg_XBits  =       _hil_dummy_SetReg_XBits;
            _Distinct_Methods.Instr04       =       _hil_dummy_Instr_4;
            _Distinct_Methods.write_read_Dp =       _hil_dummy_Write_Read_Dp;
            _Distinct_Methods.write_read_Ap =       _hil_dummy_Write_Read_Ap;
            _Distinct_Methods.write_read_mem_Ap =   _hil_dummy_Write_Read_Mem_Ap;
            _Distinct_Methods.SetReg_XBits35 =      _hil_dummy_SetReg_XBits35;
            _Distinct_Methods.SwdTransferData =     _hil_dummy_TransferData;
        }
        // SBW communication with Target-MCU is executed in FPGA mode
        else if( protocol_id == SPYBIWIRE || protocol_id == SPYBIWIRE_MSP_FET)
        {
            _Distinct_Methods.Instr =               _hil_generic_Instr;
            _Distinct_Methods.SetReg_XBits08 =      _hil_generic_SetReg_XBits08;
            _Distinct_Methods.SetReg_XBits16 =      _hil_generic_SetReg_XBits16;
            _Distinct_Methods.SetReg_XBits20 =      _hil_generic_SetReg_XBits20;
            _Distinct_Methods.SetReg_XBits32 =      _hil_generic_SetReg_XBits32;
            _Distinct_Methods.SetReg_XBits64 =      _hil_generic_SetReg_XBits64;
            _Distinct_Methods.SetReg_XBits8_64 =    _hil_generic_XBits8_64;
            _Distinct_Methods.GetPrevInstruction =  _hil_generic_GetPrevInstruction;
            _Distinct_Methods.Tclk =                _hil_generic_Tclk;
            _Distinct_Methods.SetReg_XBits  =       _hil_dummy_SetReg_XBits;
            _Distinct_Methods.Instr04       =       _hil_dummy_Instr_4;
            _Distinct_Methods.write_read_Dp =       _hil_dummy_Write_Read_Dp;
            _Distinct_Methods.write_read_Ap =       _hil_dummy_Write_Read_Ap;
            _Distinct_Methods.write_read_mem_Ap =   _hil_dummy_Write_Read_Mem_Ap;
            _Distinct_Methods.SetReg_XBits35 =      _hil_dummy_SetReg_XBits35;
            _Distinct_Methods.SwdTransferData =     _hil_dummy_TransferData;
        }
        if(gTclkHighWhilePsa == 1)
        {
            _Distinct_Methods.StepPsa = _hil_2w_StepPsaTclkHigh_Dma;
        }
        else if(gTclkHighWhilePsa == 2)
        {
            _Distinct_Methods.StepPsa = _hil_2w_StepPsa_Dma_Xv2;
        }
        else
        {
            _Distinct_Methods.StepPsa = _hil_2w_StepPsa_Dma;
        }
        _Distinct_Methods.BlowFuse = _hil_2w_BlowFuse_Dma;

        DMA2SA = (unsigned char*)DMA_TMSL_TDIL;
    }
    else if ( protocol_id == SPYBIWIREJTAG  || protocol_id == JTAG)
    {
        _Distinct_Methods.TapReset = _hil_4w_TapReset;
        _Distinct_Methods.CheckJtagFuse = _hil_4w_CheckJtagFuse;
        _Distinct_Methods.Instr = _hil_generic_Instr;
        _Distinct_Methods.SetReg_XBits08 = _hil_generic_SetReg_XBits08;
        _Distinct_Methods.SetReg_XBits16 = _hil_generic_SetReg_XBits16;
        _Distinct_Methods.SetReg_XBits20 = _hil_generic_SetReg_XBits20;
        _Distinct_Methods.SetReg_XBits32 = _hil_generic_SetReg_XBits32;
        _Distinct_Methods.SetReg_XBits64 = _hil_generic_SetReg_XBits64;
        _Distinct_Methods.SetReg_XBits8_64 =  _hil_generic_XBits8_64;
        _Distinct_Methods.GetPrevInstruction = _hil_generic_GetPrevInstruction;
        _Distinct_Methods.Tclk = _hil_generic_Tclk;

        // load dummies for 432
        _Distinct_Methods.Instr04 = _hil_dummy_Instr_4;
        _Distinct_Methods.write_read_Dp = _hil_dummy_Write_Read_Dp;
        _Distinct_Methods.write_read_Ap = _hil_dummy_Write_Read_Ap;
        _Distinct_Methods.write_read_mem_Ap = _hil_dummy_Write_Read_Mem_Ap;
        _Distinct_Methods.SetReg_XBits  = _hil_dummy_SetReg_XBits;
        _Distinct_Methods.SetReg_XBits35 = _hil_dummy_SetReg_XBits35;
        _Distinct_Methods.SwdTransferData = _hil_dummy_TransferData;

        if(gTclkHighWhilePsa == 0 || gTclkHighWhilePsa == 2)
        {
            _Distinct_Methods.StepPsa = _hil_4w_StepPsa;
        }
        else
        {
            _Distinct_Methods.StepPsa = _hil_4w_StepPsaTclkHigh;
        }
        _Distinct_Methods.BlowFuse = _hil_4w_BlowFuse;
    }
    else if (protocol_id == JTAG_432)
    {
        // load dummies for 430
        _Distinct_Methods.CheckJtagFuse = _hil_dummy_CheckJtagFuse;
        _Distinct_Methods.Instr = _hil_dummy_Instr;
        _Distinct_Methods.SetReg_XBits20 = _hil_dummy_SetReg_XBits20;
        _Distinct_Methods.GetPrevInstruction =  _hil_dummy_GetPrevInstruction;
        _Distinct_Methods.Tclk = _hil_dummy_Tclk;
        _Distinct_Methods.SetReg_XBits8_64 = _hil_dummy_SetReg_XBits8_64;
        _Distinct_Methods.StepPsa = _hil_dummy_StepPsa;
        _Distinct_Methods.BlowFuse = _hil_dummy_BlowFuse;

        _Distinct_Methods.TapReset = _hil_4w_TapReset;
        _Distinct_Methods.SetReg_XBits08 = _hil_generic432_SetReg_XBits08;
        _Distinct_Methods.SetReg_XBits16 = _hil_generic432_SetReg_XBits16;
        _Distinct_Methods.SetReg_XBits32 = _hil_generic432_SetReg_XBits32;
        _Distinct_Methods.SetReg_XBits64 = _hil_generic432_SetReg_XBits64;
        _Distinct_Methods.Instr04 = _hil_generic432_Instr_4;
        _Distinct_Methods.write_read_Dp = _hil_Write_Read_Dp_Jtag;
        _Distinct_Methods.write_read_Ap = _hil_Write_Read_Ap_Jtag;
        _Distinct_Methods.write_read_mem_Ap = _hil_Write_Read_Mem_Ap_Jtag;
        _Distinct_Methods.GetJtagIdCode = hil_Jtag_read_idcode;
        _Distinct_Methods.SetReg_XBits35 = _hil_generic432_SetReg_XBits35;
        _Distinct_Methods.SetReg_XBits = _hil_dummy_SetReg_XBits;
        _Distinct_Methods.SetReg_XBits20 = _hil_dummy_SetReg_XBits20;
        _Distinct_Methods.SwdTransferData = _hil_dummy_TransferData;
    }
    else if (protocol_id == SWD_432)
    {
        // load dummies for 430
        _Distinct_Methods.CheckJtagFuse = _hil_dummy_CheckJtagFuse;
        _Distinct_Methods.Instr = _hil_dummy_Instr;
        _Distinct_Methods.SetReg_XBits20 = _hil_dummy_SetReg_XBits20;
        _Distinct_Methods.GetPrevInstruction =  _hil_dummy_GetPrevInstruction;
        _Distinct_Methods.Tclk = _hil_dummy_Tclk;
        _Distinct_Methods.SetReg_XBits8_64 = _hil_dummy_SetReg_XBits8_64;
        _Distinct_Methods.StepPsa = _hil_dummy_StepPsa;
        _Distinct_Methods.BlowFuse = _hil_dummy_BlowFuse;
        _Distinct_Methods.TapReset = _hil_dummy_TapReset;
        _Distinct_Methods.SetReg_XBits08 = _hil_dummy_SetReg_XBits08;
        _Distinct_Methods.SetReg_XBits16 = _hil_dummy_SetReg_XBits16;
        _Distinct_Methods.SetReg_XBits20 = _hil_dummy_SetReg_XBits20;
        _Distinct_Methods.SetReg_XBits32 = _hil_dummy_SetReg_XBits32;
        _Distinct_Methods.SetReg_XBits64 = _hil_dummy_SetReg_XBits64;
        _Distinct_Methods.Instr04 = _hil_dummy_Instr_4;
        _Distinct_Methods.write_read_Dp = _hil_Write_Read_Dp_Swd;
        _Distinct_Methods.write_read_Ap = _hil_Write_Read_Ap_Swd;
        _Distinct_Methods.write_read_mem_Ap = _hil_Write_Read_Mem_Ap_Swd;
        _Distinct_Methods.GetJtagIdCode = hil_swd_read_idcode;
        _Distinct_Methods.SetReg_XBits35 = _hil_dummy_SetReg_XBits35;
        _Distinct_Methods.SetReg_XBits = _hil_dummy_SetReg_XBits;
        _Distinct_Methods.SwdTransferData = Swd_TransferData;
    }
    jtagReleased = 0;
    return(ret_value);
}

void _hil_SetPsaTCLK(unsigned short tclkValue)
{
    gTclkHighWhilePsa = tclkValue;
    if(gprotocol_id == SPYBIWIRE || gprotocol_id == SPYBIWIRE_SUBMCU || gprotocol_id == SPYBIWIRE_MSP_FET)
    {
        if(gTclkHighWhilePsa == 1)
        {
            _Distinct_Methods.StepPsa = _hil_2w_StepPsaTclkHigh_Dma;
        }
        else if(gTclkHighWhilePsa == 2)
        {
            _Distinct_Methods.StepPsa = _hil_2w_StepPsa_Dma_Xv2;
        }
        else
        {
            _Distinct_Methods.StepPsa = _hil_2w_StepPsa_Dma;
        }
    }
    if(gprotocol_id == SPYBIWIREJTAG  || gprotocol_id == JTAG)
    {
        if(gTclkHighWhilePsa == 1)
        {
            _Distinct_Methods.StepPsa = _hil_4w_StepPsaTclkHigh;
        }
        else
        {
            _Distinct_Methods.StepPsa = _hil_4w_StepPsa;
        }
    }
}
extern unsigned char lastTestState;

void _hil_Release(void)
{
    if(gprotocol_id == SPYBIWIRE)
    {
        hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_PROTOCOL, TRI_STATE_FPGA_SBW);
    }
    else if(gprotocol_id == SWD_432)
    {
            hil_fpga_disable_bypass();
            hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_PROTOCOL, TRI_STATE_FPGA_JTAG);
    }
    else
    {
        hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_PROTOCOL, TRI_STATE_FPGA_JTAG);
    }
    hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_JTAG_4_WIRE_FPGA_432, 0);

    lastTestState = 0;

    _Distinct_Methods.TapReset  = _hil_dummy_TapReset;
    _Distinct_Methods.CheckJtagFuse = _hil_dummy_CheckJtagFuse;
    _Distinct_Methods.Instr = _hil_dummy_Instr;
    _Distinct_Methods.SetReg_XBits08 = _hil_dummy_SetReg_XBits08;
    _Distinct_Methods.SetReg_XBits16 = _hil_dummy_SetReg_XBits16;
    _Distinct_Methods.SetReg_XBits20 = _hil_dummy_SetReg_XBits20;
    _Distinct_Methods.SetReg_XBits32 = _hil_dummy_SetReg_XBits32;
    _Distinct_Methods.SetReg_XBits64 = _hil_dummy_SetReg_XBits64;
    _Distinct_Methods.SetReg_XBits8_64 = _hil_dummy_SetReg_XBits8_64;
    _Distinct_Methods.Tclk = _hil_dummy_Tclk;
    _Distinct_Methods.GetPrevInstruction = _hil_dummy_GetPrevInstruction;
    _Distinct_Methods.StepPsa = _hil_dummy_StepPsa;
    _Distinct_Methods.BlowFuse = _hil_dummy_BlowFuse;
    _Distinct_Methods.SetReg_XBits  =       _hil_dummy_SetReg_XBits;
    _Distinct_Methods.Instr04 =             _hil_dummy_Instr_4;
    _Distinct_Methods.write_read_Dp =       _hil_dummy_Write_Read_Dp;
    _Distinct_Methods.write_read_Ap =       _hil_dummy_Write_Read_Ap;
    _Distinct_Methods.write_read_mem_Ap =   _hil_dummy_Write_Read_Mem_Ap;
    _Distinct_Methods.GetJtagIdCode =    _hil_read_idcodeDummy;
    _Distinct_Methods.SetReg_XBits35 = _hil_dummy_SetReg_XBits35;
    _Distinct_Methods.SetReg_XBits = _hil_dummy_SetReg_XBits;
    _Distinct_Methods.SwdTransferData = _hil_dummy_TransferData;

    jtagReleased = 1;
}

/*-------------RstLow_JTAG----------------
            ________           __________
Test ______|        |_________|
                          _______________
Rst_____________________|
----------------------------------------*/

INLINE(forced)
static void _hil_EntrySequences_RstLow_JTAG()
{
    _DINT_FET();
    TSTset0();                    //1
    _hil_Delay_1ms(4);           //reset TEST logic

    RSTset0();                    //2

    //TSTset1();                    //3
    (*_Jtag.Out) |= _Jtag.TST;
    _hil_Delay_1ms(50);         //activate TEST logic

    //RSTset0();                    //4
    (*_Jtag.Out) &= ~_Jtag.RST;
    _hil_Delay_1us(40);

     // for 4-wire JTAG clear Test pin Test(0)
    //TSTset0();   //5
    (*_Jtag.Out) &= ~_Jtag.TST;
    _hil_Delay_1us(2);

    // for 4-wire JTAG -dry  Reset(1)
    //(*_Jtag.Out) |= _Jtag.RST;
    (*_Jtag.Out) &= ~_Jtag.RST;  // This actually starts the BSL
    _hil_Delay_1us(2);

    // 4-wire JTAG - Test (1)
    //TSTset1();  //7
    (*_Jtag.Out) |= _Jtag.TST;
    _hil_Delay_1ms(5);
    _EINT_FET();
}

/*-------------RstHigh_JTAG--------------
            ________           __________
Test ______|        |_________|
         _______                   ______
Rst____|       |_________________|
----------------------------------------*/

INLINE(forced)
static void _hil_EntrySequences_RstHigh_JTAG()
{
    TSTset0();                    //1
    _hil_Delay_1ms(1);           //reset TEST logic

    RSTset1();                    //2

    TSTset1();                    //3
    _hil_Delay_1ms(100);         //activate TEST logic

    RSTset0();                    //4
    _hil_Delay_1us(40);

    // for 4-wire JTAG clear Test pin Test(0)
    (*_Jtag.Out) &= ~_Jtag.TST;   //5
    _hil_Delay_1us(1);

    // for 4-wire JTAG - Test (1)
    (*_Jtag.Out) |= _Jtag.TST;  //7
    _hil_Delay_1us(40);

    // phase 5 Reset(1)
    RSTset1();
    _hil_Delay_1ms(5);
}

/*-------------RstHigh_SBW---------------
            ________           __________
Test ______|        |_________|
        _________________________________
Rst____|
----------------------------------------*/
INLINE(forced)
static void _hil_EntrySequences_RstHigh_SBW()
{
    TSTset0();                //1
    _hil_Delay_1ms(1);       // reset TEST logic

    RSTset1();                //2

    TSTset1();                //3
    _hil_Delay_1ms(100);     // activate TEST logic

    // phase 1
    RSTset1();                //4
    _hil_Delay_1us(40);

    // phase 2 -> TEST pin to 0, no change on RST pin
    // for Spy-Bi-Wire
    _DINT_FET();
    (*_Jtag.Out) &= ~_Jtag.TST;     //5
    _hil_Delay_1us(1);

    // phase 4 -> TEST pin to 1, no change on RST pin
    // for Spy-Bi-Wire
    (*_Jtag.Out) |= _Jtag.TST;      //7
    _EINT_FET();
    _hil_Delay_1us(40);

    _hil_Delay_1ms(5);
}

/*-------------RstLow_SBW----------------
            ________           __________
Test ______|        |_________|
               __________________________
Rst__________|
----------------------------------------*/
INLINE(forced)
static void _hil_EntrySequences_RstLow_SBW()
{
    TSTset0();                //1
    _hil_Delay_1ms(1);       // reset TEST logic

    RSTset0();                //2

    TSTset1();                //3
    _hil_Delay_1ms(100);     // activate TEST logic

    // phase 1
    RSTset1();                //4
    _hil_Delay_1us(40);

    // phase 2 -> TEST pin to 0, no change on RST pin
    // for Spy-Bi-Wire
    _DINT_FET();
    (*_Jtag.Out) &= ~_Jtag.TST;     //5
    _hil_Delay_1us(1);

    // phase 4 -> TEST pin to 1, no change on RST pin
    // for Spy-Bi-Wire
    (*_Jtag.Out) |= _Jtag.TST;      //7
    _EINT_FET();
    _hil_Delay_1us(40);
    _hil_Delay_1ms(5);
}

void _hil_EntrySequences(unsigned char states)
{
    unsigned short protocol = JTAG_4_WIRE_FPGA;

    //This should probably never be called with SPYBIWIRE_SUBMCU anyway
    if(gprotocol_id == SPYBIWIRE_SUBMCU)
    {
        hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_PROTOCOL, TRI_STATE_FPGA_JTAG);
        gprotocol_id = SPYBIWIRE;
    }
    hil_fpga_enable_bypass();

    switch(gprotocol_id)
    {
    case SPYBIWIRE:
    case SPYBIWIRE_MSP_FET:
      {
        if (states == RSTLOW)
        {
            _hil_EntrySequences_RstLow_SBW();
        }
        if (states == RSTHIGH)
        {
            _hil_EntrySequences_RstHigh_SBW();
        }
        protocol = (gprotocol_id == SPYBIWIRE) ? SBW_2_BACK_FPGA : SBW_2_MSP_FET_FPGA;
        break;
      }
    case SPYBIWIREJTAG:
      {
        if (states == RSTLOW)
        {
            _hil_EntrySequences_RstLow_JTAG();
        }
        if (states == RSTHIGH)
        {
            _hil_EntrySequences_RstHigh_JTAG();
        }
        break;
      }
    case JTAG_432:
        {
            JTAG_Entry();
            RSTset1();
            break;
        }
    case SWD_432:
        {
            SWD_Entry();
            SWDTCKset1();
            break;
        }
    default:
        if (states == RSTLOW)
        {
            RSTset0();
            TSTset1();
        }
        if (states == RSTHIGH)
        {
            TSTset1();
        }
        break;
    }
    if(gprotocol_id != SWD_432)
    {
        hil_fpga_disable_bypass();
        hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_PROTOCOL, protocol);

        if(gprotocol_id == JTAG_432)
        {
            hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_JTAG_4_WIRE_FPGA_432, 1);
        }
        else
        {
             hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_JTAG_4_WIRE_FPGA_432, 0);
        }
    }
}

#pragma optimize = medium
static void _hil_Connect(unsigned char state)
{
    if(jtagReleased)
    {
        _hil_SetProtocol(gprotocol_id);
    }

    hil_fpga_enable_bypass();

    if(state == RSTHIGH && gprotocol_id != JTAG_432 && gprotocol_id != SWD_432)
    {
        if(gprotocol_id == SPYBIWIRE || gprotocol_id == SPYBIWIRE_MSP_FET)
        {
            _hil_Delay_1ms(10);
            qDriveSbw();
            _hil_Delay_1ms(30);
            _hil_EntrySequences_RstHigh_SBW();
        }
        else
        {
            _hil_Delay_1ms(10);
            qDriveJTAG();
            _hil_Delay_1ms(30);
            if(gprotocol_id == SPYBIWIREJTAG)
            {
                _hil_EntrySequences_RstHigh_JTAG();
            }
            else
            {
                TSTset1();
            }
        }
    }
    else if(state == RSTLOW && gprotocol_id != JTAG_432 && gprotocol_id != SWD_432)
    {
        if(gprotocol_id == SPYBIWIRE || gprotocol_id == SPYBIWIRE_MSP_FET)
        {
            qDriveSbw();
            _hil_Delay_1ms(1);
            _hil_EntrySequences_RstLow_SBW();
        }
        else
        {
            qDriveJTAG();
            _hil_Delay_1ms(1);
            if(gprotocol_id == SPYBIWIREJTAG)
            {
                _hil_EntrySequences_RstLow_JTAG();
            }
            else
            {
                RSTset0();
                TSTset1();
            }
        }
    }
    else if(gprotocol_id == JTAG_432)
    {
        _hil_Delay_1ms(10);
        qDriveJTAG();
        _hil_Delay_1ms(30);
        JTAG_Entry();
        RSTset1();
    }
    else if(gprotocol_id == SWD_432)
    {
        _hil_Delay_1ms(10);
        qDriveSwd();
        _hil_Delay_1ms(30);
        SWD_Entry();
        RSTset1();
        hil_swd_read_idcode();
    }

    if(gprotocol_id != SWD_432)
    {
        hil_fpga_disable_bypass();
    }

    jtagReleased = 0;
}

// -----------------------------------------------------------------------------
short _hil_Open( unsigned char state)
{
    if(gprotocol_id == SPYBIWIRE_MSP_FET)
    {
        _hil_Connect(state);
        hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_PROTOCOL, SBW_2_MSP_FET_FPGA);
        hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_JTAG_4_WIRE_FPGA_432, 0);
        _hil_generic_ConfigureSpeed(hil_sbw2Speed_);
    }
    else if(gprotocol_id == SPYBIWIRE)
    {
        _hil_Connect(state);
         hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_PROTOCOL, SBW_2_BACK_FPGA);
         hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_JTAG_4_WIRE_FPGA_432, 0);
        _hil_generic_ConfigureSpeed(hil_sbw2Speed_);
    }
    else if(gprotocol_id == SPYBIWIREJTAG || gprotocol_id == JTAG)
    {
        _hil_Connect(state);
        hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_PROTOCOL, JTAG_4_WIRE_FPGA);
        hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_JTAG_4_WIRE_FPGA_432, 0);
        _hil_generic_ConfigureSpeed(hil_jtagSpeed_);
    }
    else if(gprotocol_id == JTAG_432)
    {
        _hil_Connect(state);
        hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_PROTOCOL, JTAG_4_WIRE_FPGA);
        hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_JTAG_4_WIRE_FPGA_432, 1);
        _hil_generic_ConfigureSpeed(hil_jtagSpeed_);
    }
    else if(gprotocol_id == SPYBIWIRE_SUBMCU)
    {
        hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_PROTOCOL, TRI_STATE_FPGA_JTAG);
        hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_JTAG_4_WIRE_FPGA_432, 0);
        gprotocol_id = SPYBIWIRE;
        _hil_Connect(state);
    }
    else if(gprotocol_id == SWD_432)
    {
        gprotocol_id = SWD_432;
        _hil_Connect(state);
    }
    return 0;
}
// -----------------------------------------------------------------------------
short _hil_Close( void )
{
    externalVccOn = EXTERNAL_VCC_OFF;
    _hil_Release();
    return 0;
}

// -----------------------------------------------------------------------------

void _hil_SetJtagSpeed(unsigned short jtagSpeed, unsigned short sbwSpeed)
{
    if( sbwSpeed )
    {
        hil_sbw2Speed_ =  sbwSpeed;
    }
    if ( jtagSpeed )
    {
        hil_jtagSpeed_ =  jtagSpeed;
    }
}

// -----------------------------------------------------------------------------

void _hil_ConfigureSetPc (unsigned short PCclockBeforeCapture)
{
    setPCclockBeforeCapture = PCclockBeforeCapture;
}

// -----------------------------------------------------------------------------
void SetVFuse(void)
{
    unsigned long timeOut = VF_PWM_TIMEOUT;
    unsigned short runningAverageBuffer[4] = {0,0,0,0};
    unsigned short avg = 0;
    unsigned char i = 0;

    do
    {
        *(_VFuse.PWM_CTRL) |= _VFuse.PWM_SETVF;
        __delay_cycles(VF_PWM_ON_TIME);
        *(_VFuse.PWM_CTRL) &= ~(_VFuse.PWM_SETVF);
        __delay_cycles(VF_PWM_OFF_TIME);
        runningAverageBuffer[i] = hil_AdcControlGetVFuse();
        i = (i + 1) % 4;
        avg = (runningAverageBuffer[0] + runningAverageBuffer[1] + runningAverageBuffer[2] + runningAverageBuffer[3]) >> 2;
    }
    while((6800 > avg) && timeOut--);
}

// -----------------------------------------------------------------------------
void setVpp(long voltage)
{
    static unsigned char prevP4value = 0;
    // Reconfigure ADC to measure Fuse Voltage without interrupts
    hil_AdcControlInitADCvFuse();

    if(voltage)
    {
        SetVFuse();

        while(hil_AdcControlGetVFuse() >= 6300);

        if(useTDI)
        {
            // Vpp via TDI
            *(_VFuse.CTRL) &= ~(_VFuse.TDI_OFF);
            *(_VFuse.CTRL) &= ~(_VFuse.VF2TEST);
            *(_VFuse.CTRL) |= _VFuse.VF2TDI;
        }
        else
        {
            _DINT_FET();
            prevP4value = P4OUT & BIT5;
            // Vpp via TST
            *(_VFuse.CTRL) &= ~(_VFuse.VF2TDI);
            *(_VFuse.CTRL) |= _VFuse.TDI_OFF;
            P4OUT &= ~BIT5;
            *(_VFuse.CTRL) |= _VFuse.VF2TEST;
            _EINT_FET();
        }
        _hil_Delay_1ms(3);
    }
    else
    {
        // Turn of Vpp
        _DINT_FET();
        *(_VFuse.CTRL) &= ~(_VFuse.VF2TDI);
        *(_VFuse.CTRL) &= ~(_VFuse.VF2TEST);
        if(!useTDI)
        {
            _hil_Delay_1us(3);
            P4OUT |= prevP4value;
        }
        *(_VFuse.CTRL) |= _VFuse.TDI_OFF;
        _EINT_FET();
    }

    // Reconfigure ADC to measure VCC – supply/dt/externalVcc in sequence mode
    hil_AdcControlInitADC();
    ADC12IE = 0x08;                           // Enable ADC12IFG.3
}

// -----------------------------------------------------------------------------
void testVpp(unsigned char mode)
{
    if(mode)
    {
        // Fuse blow via TST
        // Switch back to JTAG protocol
        hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_PROTOCOL, JTAG_4_WIRE_FPGA);
        hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_JTAG_4_WIRE_FPGA_432, 0);

    }
    else
    {
        // Fuse blow via TDI
        // Switch to JTAG fuse blow protocol, TDI is mapped to TDO
        hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_PROTOCOL, TDI_TO_TDO_FPGA);
        hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_JTAG_4_WIRE_FPGA_432, 0);
    }

    useTDI = !mode;
}


// -----------------------------------------------------------------------------
void _hil_SetReset(unsigned char value)
{

    hil_fpga_enable_bypass();
    if(value)
    {
        RSTset1();
    }
    else
    {
        RSTset0();
    }
    hil_fpga_disable_bypass();
}

// -----------------------------------------------------------------------------
void _hil_SetTest(unsigned char value)
{
    hil_fpga_enable_bypass();

    if(value)
    {
        TSTset1();
    }
    else
    {
        TSTset0();
    }
    hil_fpga_disable_bypass();
}

// -----------------------------------------------------------------------------
void _hil_SetTMS(unsigned char value)
{
    hil_fpga_enable_bypass();

    if(value)
    {
        TMSset1();
    }
    else
    {
        TMSset0();
    }
    hil_fpga_disable_bypass();
}

// -----------------------------------------------------------------------------
void _hil_SetTDI(unsigned char value)
{
    hil_fpga_enable_bypass();

    if(value)
    {
        TDIset1();
    }
    else
    {
        TDIset0();
    }
    hil_fpga_disable_bypass();
}

// -----------------------------------------------------------------------------
void _hil_SetTCK(unsigned char value)
{
    hil_fpga_enable_bypass();

    if(value)
    {
        TCKset1();
    }
    else
    {
        TCKset0();
    }
    hil_fpga_disable_bypass();
}
// --------------------------------------------ARM defines end------------------

void SWD_Entry()
{
    unsigned char sequence[8];
    unsigned char i = 0;
    unsigned short entry = 0xE79E;
    for (i = 0; i < 8; i++)
    {
        sequence[i] = 0xff;
    }
    hil_Swd_Seq(56, sequence);

    sequence[0] = entry & 0xff;
    sequence[1] = (entry >> 8) & 0xff;
    hil_Swd_Seq(16, sequence);

    for (i = 0; i < 8; i++)
    {
        sequence[i] = 0xff;
    }
    hil_Swd_Seq(56, sequence);
}

 void JTAG_Entry()
{
    unsigned char sequence[8];
    unsigned char i = 0;
    unsigned short entry = 0xE73C;
    for (i = 0; i < 8; i++)
    {
        sequence[i] = 0xff;
    }
    hil_4w_432_Seq(51, sequence);

    sequence[0] = entry & 0xff;
    sequence[1] = (entry >> 8) & 0xff;
    hil_4w_432_Seq(16, sequence);

    for (i = 0; i < 1; i++)
    {
        sequence[i] = 0xff;
    }
    hil_4w_432_Seq(8, sequence);
}

unsigned long hil_swd_read_idcode()
{
    unsigned char sequence[1] = {0};
    unsigned long id = 0;
    hil_Swd_Seq(8, sequence);

    if (_Distinct_Methods.write_read_Dp(0, &id, READ) != SWD_ACK)
    {
        return 0xFFFF;
    }
    return id;
}


unsigned long hil_Jtag_read_idcode()
{
    unsigned long JtagId = 0;

    JtagId = _Distinct_Methods.Instr04(IR4_IDCODE);

    if (JtagId == 0x1)
    {
        JtagId = _Distinct_Methods.SetReg_XBits32(0);
    }
    return JtagId;
}


short _hil_Write_Read_Dp_Swd(unsigned char address ,unsigned long *data, unsigned short rnw)
{
    unsigned char retVal = 0;
    unsigned char i = 0;

    unsigned char regiser = SWD_DP | rnw<<1 | (address & 0x0c);

    for (i = 0; i < MAX_RETRY; i++)
    {
        retVal = Swd_TransferData(regiser, data, rnw);
        // if not wait for answer
        if (retVal != 0x2)
        {
            return SWD_ACK;
        }
    }
    if(i == MAX_RETRY || retVal != 0x1)
    {
        return -1;
    }
    return SWD_ACK;
}

//#pragma optimize = none
short _hil_Write_Read_Ap_Swd(unsigned long address, unsigned long *data, unsigned short rnw)
{
    unsigned long long retVal;
    unsigned char i = 0;
    unsigned long dummydata = 0;

    unsigned long apsel_bank_sel = address & DP_SELECT_APSEL_MASK;
    apsel_bank_sel |= address & DP_SELECT_APBANKSEL_MASK;

    if (_hil_Write_Read_Dp_Swd(DP_SELECT, &apsel_bank_sel, WRITE) != SWD_ACK)
    {
        return -1;
    }
    unsigned char regiser = SWD_AP | rnw<<1 | (address & 0x0c);
    do
    {
        retVal = Swd_TransferData(regiser, data, rnw);
    }
    while(i++ < MAX_RETRY && retVal != 0x1);

    if(i == MAX_RETRY)
    {
        return -1;
    }

    if(rnw == WRITE)
    {
        unsigned char regiser = SWD_DP | READ<<1 | (DP_RDBUFF & 0x0c);
        do
        {
             retVal = Swd_TransferData(regiser, &dummydata, READ);
        }
        while(i++ < MAX_RETRY && retVal != 0x1);

        if(i == MAX_RETRY)
        {
            return -1;
        }
    }
    else
    {
        do
        {
            Swd_TransferData(regiser, data, rnw);
        }
        while(i++ == MAX_RETRY && retVal != 0x1);

        if(i == MAX_RETRY)
        {
            return -1;
        }
    }
    return SWD_ACK;
}

short _hil_Write_Read_Mem_Ap_Swd(unsigned short ap_sel, unsigned long address, unsigned long *data, unsigned short rnw)
{
    // First Read current CSW value
    unsigned long tmp = 0;
    unsigned long apsel = ((uint32_t)ap_sel);
    if (_hil_Write_Read_Ap_Swd(AP_CSW | apsel, &tmp, READ) != SWD_ACK)
    {
        return -1;
    }

    // Configure the transfer
     tmp = (tmp & ~(AP_CSW_SIZE_MASK | AP_CSW_ADDRINC_MASK)) | AP_CSW_ADDRINC_OFF | AP_CSW_SIZE_32BIT;
    if (_hil_Write_Read_Ap_Swd(AP_CSW | apsel, &tmp, WRITE) != SWD_ACK)
    {
        return -1;
    }

    // Write the address
    tmp = address;
    if (_hil_Write_Read_Ap_Swd(AP_TAR | apsel, &tmp, WRITE) != SWD_ACK)
    {
        return -1;
    }
    // Write/Read data
    return _hil_Write_Read_Ap_Swd(AP_DRW | apsel, data, rnw);
}

// -----------------------------------------------------------------------------
short _hil_Write_Read_Dp_Jtag(unsigned char address ,unsigned long *data, unsigned short rnw)
{
    unsigned long long retVal;
    unsigned char i = 0;
    unsigned long long dataAlig = 0ull;
    dataAlig = ((unsigned long long)*data) << 3;

    dataAlig |= (((address & 0xC) >> 1) + (rnw & 0x1));

    if(_Distinct_Methods.Instr04(DPACC) != 0x1)
    {
        return -1;
    }

    for (i = 0; i < MAX_RETRY; i++)
    {
        retVal = _Distinct_Methods.SetReg_XBits35(&dataAlig);
        // if ack == ACK
        if ((retVal & 0x3) == ACK)
        {
            *data = retVal >> 3;
            return ACK;
        }
    }
    return -1;
}

// -----------------------------------------------------------------------------
short _hil_Write_Read_Ap_Jtag(unsigned long address, unsigned long *data, unsigned short rnw)
{
    unsigned long long retVal;
    unsigned char i = 0;
    unsigned long long dataAlig = 0ull;

    dataAlig = ((unsigned long long)*data) << 3;

    unsigned long apsel_bank_sel = address & DP_SELECT_APSEL_MASK;
    apsel_bank_sel |= address & DP_SELECT_APBANKSEL_MASK;

    if (_hil_Write_Read_Dp_Jtag(DP_SELECT, &apsel_bank_sel, WRITE) != ACK)
    {
        return -1;
    }

    dataAlig |= (((address & 0xC) >> 1) + (rnw & 0x1));

    if(_Distinct_Methods.Instr04(APACC) != 0x1)
    {
        return -1;
    }

    do
    {
        retVal = _Distinct_Methods.SetReg_XBits35(&dataAlig);
    } while((retVal & 0x3 != ACK) && (++i < MAX_RETRY));

    if(i == MAX_RETRY)
    {
        return -1;
    }
    else
    {
        if(rnw == READ)
        {
            return _hil_Write_Read_Dp_Jtag(DP_RDBUFF, data, READ);
        }
        else
        {
            return ACK;
        }
    }
}

// -----------------------------------------------------------------------------
// Currently only supports single 32-bit transfers
short _hil_Write_Read_Mem_Ap_Jtag(unsigned short ap_sel, unsigned long address, unsigned long *data, unsigned short rnw)
{
    // First Read current CSW value
    unsigned long tmp = 0;
    unsigned long apsel = (((uint32_t)ap_sel) << 24);
    if (_hil_Write_Read_Ap_Jtag(AP_CSW | apsel, &tmp, READ) != ACK)
    {
        return -1;
    }

    // Configure the transfer
    tmp = (tmp & ~(AP_CSW_SIZE_MASK | AP_CSW_ADDRINC_MASK)) | AP_CSW_ADDRINC_OFF | AP_CSW_SIZE_32BIT;
    if (_hil_Write_Read_Ap_Jtag(AP_CSW | apsel, &tmp, WRITE) != ACK)
    {
        return -1;
    }

    // Write the address
    tmp = address;
    if (_hil_Write_Read_Ap_Jtag(AP_TAR | apsel, &tmp, WRITE) != ACK)
    {
        return -1;
    }
    // Write/Read data
    return _hil_Write_Read_Ap_Jtag(AP_DRW | apsel, data, rnw);
}
