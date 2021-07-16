/*
 * \ingroup MODULHIL
 *
 * \file hil430.c
 *
 * \brief Hardware Independent Layer Interface
 *
 *
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
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

#include "hw_compiler_specific.h"
#include "arch.h"
#include "edt.h"
#include "hilDelays.h"
#include "HalGlobalVars.h"
#include "stream.h"

unsigned short gprotocol_id;
unsigned short gTclkHighWhilePsa;
unsigned char *tstctrl_port_;
unsigned char entdi2tdo_;
unsigned short jtagReleased;

// function prototypes for map initialization
// common HIL configuration methods
short _hil_Init( void );
short _hil_SetProtocol(unsigned short protocol_id);
void  _hil_SetPsaSetup(unsigned short enhanced);
void  _hil_SetPsaTCLK(unsigned short tclkValue);
short _hil_Open( unsigned char state );
short _hil_Close( void );
void  _hil_EntrySequences(unsigned char states);
void _hil_ConfigureSetPc(unsigned short x) {}

void _hil_SetReset(unsigned char);
void _hil_SetTest(unsigned char);
void _hil_SetTMS(unsigned char);
void _hil_SetTCK(unsigned char);
void _hil_SetTDI(unsigned char);
void _hil_initTimerB0(void);
void _hil_BSL_EntrySequence(unsigned short dummy);
void _hil_BSL_EntrySequence1xx_4xx();
void _hil_SwitchVccFET(unsigned short SwitchVccFET){};
short _hil_regulateVcc(void) { return 0; }
void _hil_setFpgaTimeOut(unsigned short state) {}
unsigned short _hil_getFpgaVersion(void) { return 0; }
void _hil_ReadADC12(void) {}
void _hil_ConfigFpgaIoMode(unsigned short mode) {}

extern void _hilGeneric_Init( void );
extern short _hilGeneric_GetVcc(double* Vcc, double* ExtVcc);
extern void _hilGeneric_SetJtagSpeed(unsigned short speed, unsigned short sbwSpeed);
extern short IccMonitor_Process(unsigned short flags);
extern short _hilGeneric_SetVcc(unsigned short Vcc);

edt_common_methods_t _edt_Common_Methods =
{
    _hil_Init,
    _hilGeneric_SetVcc,
    _hil_SwitchVccFET,
    _hilGeneric_GetVcc,
    _hil_SetProtocol,
    _hil_SetPsaTCLK,
    _hil_Open,
    _hil_Close,
    _hil_Delay_1us,
    _hil_Delay_1ms,
    IccMonitor_Process,
    _hil_EntrySequences,
    _hil_SetReset,
    _hil_SetTest,
    _hilGeneric_SetJtagSpeed,
    _hil_ConfigureSetPc,
    _hil_initTimerB0,
    _hil_BSL_EntrySequence,
    _hil_SetTMS,
    _hil_SetTCK,
    _hil_SetTDI,
    _hil_regulateVcc,
    _hil_setFpgaTimeOut,
    _hil_getFpgaVersion,
    _hil_ReadADC12,
    _hil_ConfigFpgaIoMode,
    _hil_BSL_EntrySequence1xx_4xx
};
// init by HIL init function
edt_distinct_methods_t _edt_Distinct_Methods;

// protocol specific methods
// (must be implemeted in 4-wire JTAG and 2-wire Spy-Bi-Wire mode)
extern short _hil_4w_TapReset(void);
extern short _hil_4w_CheckJtagFuse(void);
extern unsigned char _hil_4w_Instr(unsigned char Instruction);
//extern unsigned long _hil_4w_SetReg_XBits(unsigned long Data, short Bits);
extern unsigned char _hil_4w_SetReg_XBits08(unsigned char Data);
extern unsigned short _hil_4w_SetReg_XBits16(unsigned short Data);
extern unsigned long _hil_4w_SetReg_XBits20(unsigned long Data);
extern unsigned long _hil_4w_SetReg_XBits32(unsigned long Data);
extern unsigned long long _hil_4w_SetReg_XBits64(unsigned long long Data);
extern unsigned long long _hil_4w_SetReg_XBits8_64(unsigned long long Data, unsigned short loopCount, unsigned short JStateVersion);

extern void _hil_4w_Tclk(unsigned char state);
extern void _hil_4w_StepPsa(unsigned long length);
extern void _hil_4w_StepPsaTclkHigh(unsigned long length);
extern short _hil_4w_BlowFuse(unsigned char targetHasTestVpp);

extern short _hil_2w_TapReset(void);
extern short _hil_2w_CheckJtagFuse(void);
extern unsigned char _hil_2w_Instr(unsigned char Instruction);
//extern unsigned long _hil_2w_SetReg_XBits(unsigned long Data, short Bits);
extern unsigned char _hil_2w_SetReg_XBits08(unsigned char Data);
extern unsigned short _hil_2w_SetReg_XBits16(unsigned short Data);
extern unsigned long _hil_2w_SetReg_XBits20(unsigned long Data);
extern unsigned long _hil_2w_SetReg_XBits32(unsigned long Data);
extern unsigned long long _hil_2w_SetReg_XBits64(unsigned long long Data);
extern unsigned long long _hil_2w_SetReg_XBits8_64(unsigned long long Data, unsigned short loopCount, unsigned short JStateVersion);
extern void _hil_2w_Tclk(unsigned char state);
extern void _hil_2w_StepPsa(unsigned long length);
extern void _hil_2w_StepPsaTclkHigh(unsigned long length);
extern short _hil_2w_BlowFuse(unsigned char targetHasTestVpp);
void _hil_2w_StepPsa_Xv2(unsigned long length);

extern unsigned char _hil_2w_GetPrevInstruction();
extern unsigned char _hil_4w_GetPrevInstruction();

short _hil_dummy_TapReset(void) {return 0;}
short _hil_dummy_CheckJtagFuse(void){return 0;}
unsigned char _hil_dummy_Instr(unsigned char Instruction){return 0;}
unsigned char _hil_dummy_SetReg_XBits08(unsigned char Data){return 0;}
unsigned short _hil_dummy_SetReg_XBits16(unsigned short Data){return 0;}
unsigned long _hil_dummy_SetReg_XBits20(unsigned long Data){return 0;}
unsigned long _hil_dummy_SetReg_XBits32(unsigned long Data){return 0;}
unsigned long long _hil_dummy_SetReg_XBits64(unsigned long long Data){return 0;}
unsigned long long _hil_dummy_SetReg_XBits8_64(unsigned long long Data, unsigned short loopCount, unsigned short JStateVersion){return 0;}

void _hil_dummy_Tclk(unsigned char state){return;}
void _hil_dummy_StepPsa(unsigned long length){return;}
short _hil_dummy_BlowFuse(unsigned char targetHasTestVpp){return 0;}
unsigned char _hil_dummy_GetPrevInstruction(){return 0;}

//dummy 432 defines
unsigned char _hil_dummy_Instr_4(unsigned char Data){return 0;}
unsigned long long hil_dummy_SetReg_35Bits (unsigned long long *Data) {return 0;}
unsigned long long hil_dummy_SetReg_XBits (unsigned long long *Data, unsigned short count) {return 0;}
short _hil_dummy_Write_Read_Ap(unsigned long address ,unsigned long *data, unsigned short rnw){return -1;};
short _hil_dummy_Write_Read_Dp(unsigned char address ,unsigned long *data, unsigned short rnw){return -1;};
short _hil_dummy_Write_Read_Mem_Ap(unsigned short ap_sel, unsigned long address, unsigned long *data, unsigned short rnw){return -1;};
unsigned long _hil_dummy_read_idcoe(){return -1;}


// PSA distinct methods
void _hil_EnhancedPsaSetup(unsigned long address);
void _hil_PsaSetup(unsigned long address);
void _hil_EnhancedPsaEnd(void);
void _hil_PsaEnd(void);

extern const unsigned char DMA_TMSL_TDIL[];

extern volatile unsigned char VFSettleFlag;
static volatile unsigned char useTDI = 0;

// -----------------------------------------------------------------------------
short _hil_Init( void )
{
    _hilGeneric_Init();
    // DMA init
    DMACTL0 = 0;                  // URXIFG0 trigger for DMS channel0, channel1 and channel2 are software triggered
    DMACTL1 = 0;                           // DAM immediatly, no round-robin, no NMI interrupt

    DMA1CTL = ( DMADT0 | DMASRCINCR1 | DMASRCINCR0 | DMASRCBYTE | DMADSTBYTE);
    DMA1DA = (unsigned int)_Jtag.Out; //JTAGOUT;       // set destination address
    DMA2CTL = ( DMADT0 | DMASRCINCR1 | DMASRCINCR0 | DMASRCBYTE | DMADSTBYTE);
    DMA2DA = (unsigned int)_Jtag.Out; //JTAGOUT;       // set destination address
    DMA2SZ = 4;                            // JTAG test/idle sequence is always 4 transisions

    // set default debug protocol to JTAG
    gprotocol_id = JTAG;
    // set default to TCLK low when doing PSA
    gTclkHighWhilePsa = 0;
    // initialize function pointers to distinct functions
    _hil_SetProtocol(gprotocol_id);
    jtagReleased = 1;

    _edt_Common_Methods.BSL_EntrySequence1xx_4xx = _hil_BSL_EntrySequence1xx_4xx;

    return 0;
}

// -----------------------------------------------------------------------------
short _hil_SetProtocol(unsigned short protocol_id)
{
    short ret_value = 0;

    if((protocol_id == JTAG))
    {
        gprotocol_id = JTAG;
    }
    else if((protocol_id == SPYBIWIRE))
    {
        gprotocol_id = SPYBIWIRE;
    }
    else if((protocol_id == SPYBIWIREJTAG))
    {
        gprotocol_id = SPYBIWIREJTAG;
    }
    else
    {
        ret_value = -1;
    }

    tstctrl_port_ = (unsigned char*)&P4OUT;
    entdi2tdo_ = BIT2;

    // set MSP4323 to dummy values
    _edt_Distinct_Methods.SetReg_XBits35 = hil_dummy_SetReg_35Bits;
    _edt_Distinct_Methods.SetReg_XBits = hil_dummy_SetReg_XBits;
    _edt_Distinct_Methods.Instr04 =             _hil_dummy_Instr_4;
    _edt_Distinct_Methods.write_read_Dp =       _hil_dummy_Write_Read_Dp;
    _edt_Distinct_Methods.write_read_Ap =       _hil_dummy_Write_Read_Ap;
    _edt_Distinct_Methods.write_read_mem_Ap =   _hil_dummy_Write_Read_Mem_Ap;
    _edt_Distinct_Methods.GetJtagIdCode =       _hil_dummy_read_idcoe;

    if(gprotocol_id == SPYBIWIRE)
    {
        _edt_Distinct_Methods.TapReset = _hil_2w_TapReset;
        _edt_Distinct_Methods.CheckJtagFuse = _hil_2w_CheckJtagFuse;
        _edt_Distinct_Methods.Instr = _hil_2w_Instr;
        _edt_Distinct_Methods.SetReg_XBits08 = _hil_2w_SetReg_XBits08;
        _edt_Distinct_Methods.SetReg_XBits16 = _hil_2w_SetReg_XBits16;
        _edt_Distinct_Methods.SetReg_XBits20 = _hil_2w_SetReg_XBits20;
        _edt_Distinct_Methods.SetReg_XBits32 = _hil_2w_SetReg_XBits32;
        _edt_Distinct_Methods.SetReg_XBits64 = _hil_2w_SetReg_XBits64;
        _edt_Distinct_Methods.SetReg_XBits8_64 = _hil_2w_SetReg_XBits8_64;
        _edt_Distinct_Methods.Tclk = _hil_2w_Tclk;
        _edt_Distinct_Methods.GetPrevInstruction = _hil_2w_GetPrevInstruction;

        if(gTclkHighWhilePsa == 1)
        {
            _edt_Distinct_Methods.StepPsa = _hil_2w_StepPsaTclkHigh;
        }
        else if(gTclkHighWhilePsa == 2)
        {
          _edt_Distinct_Methods.StepPsa = _hil_2w_StepPsa_Xv2;
        }
        else
        {
            _edt_Distinct_Methods.StepPsa = _hil_2w_StepPsa;
        }

        _edt_Distinct_Methods.BlowFuse = _hil_2w_BlowFuse;
        DMA1SZ = 6;                         // load DMA1 with size
        DMA2SZ = 6;                         // load DMA1 with size
        DMA2SA = (unsigned int)DMA_TMSL_TDIL;
    }
    else
    {
        _edt_Distinct_Methods.TapReset = _hil_4w_TapReset;
        _edt_Distinct_Methods.CheckJtagFuse = _hil_4w_CheckJtagFuse;
        _edt_Distinct_Methods.Instr = _hil_4w_Instr;
        _edt_Distinct_Methods.SetReg_XBits08 = _hil_4w_SetReg_XBits08;
        _edt_Distinct_Methods.SetReg_XBits16 = _hil_4w_SetReg_XBits16;
        _edt_Distinct_Methods.SetReg_XBits20 = _hil_4w_SetReg_XBits20;
        _edt_Distinct_Methods.SetReg_XBits32 = _hil_4w_SetReg_XBits32;
        _edt_Distinct_Methods.SetReg_XBits64 = _hil_4w_SetReg_XBits64;
        _edt_Distinct_Methods.SetReg_XBits8_64 = _hil_4w_SetReg_XBits8_64;
        _edt_Distinct_Methods.Tclk = _hil_4w_Tclk;
        _edt_Distinct_Methods.GetPrevInstruction = _hil_4w_GetPrevInstruction;
        if(gTclkHighWhilePsa == 0 || gTclkHighWhilePsa == 2)
        {
            _edt_Distinct_Methods.StepPsa = _hil_4w_StepPsa;
        }
        else
        {
            _edt_Distinct_Methods.StepPsa = _hil_4w_StepPsaTclkHigh;
        }
        _edt_Distinct_Methods.BlowFuse = _hil_4w_BlowFuse;
        DMA2SZ = 4;                         // load DMA1 with size
    }
    return(ret_value);
}

void _hil_SetPsaTCLK(unsigned short tclkValue)
{
    gTclkHighWhilePsa = tclkValue;
    if(gprotocol_id == SPYBIWIRE)
    {
        if(gTclkHighWhilePsa == 1)
        {
            _edt_Distinct_Methods.StepPsa = _hil_2w_StepPsaTclkHigh;
        }
        else if(gTclkHighWhilePsa == 2)
        {
          _edt_Distinct_Methods.StepPsa = _hil_2w_StepPsa_Xv2;
        }
        else
        {
            _edt_Distinct_Methods.StepPsa = _hil_2w_StepPsa;
        }
    }
    else
    {
        if(gTclkHighWhilePsa == 1)
        {
            _edt_Distinct_Methods.StepPsa = _hil_4w_StepPsaTclkHigh;
        }
        else
        {
            _edt_Distinct_Methods.StepPsa = _hil_4w_StepPsa;
        }
    }
}

static void _hil_Release(void)
{
    if(!jtagReleased)
    {
        if(gprotocol_id == SPYBIWIRE)
        {
            // drive target RST/SBWTDIO pin high
            JTAGOUT |=  sbwdato;                // TDI drives target RST high
            TSTCTRLOUT |= ENTDI2TDO;            // disable TDI2TDO
            IHIL_Delay_1ms(1);
            // drive target TEST/SBWTCK pin low
            JTAGOUT &= ~sbwclk;                 // TCK drives target TEST low - release Spy-Bi-Wire logic
        }
        else
        {
            // hands off from target RST pin  -> will be pulled up
            TSTCTRLOUT |= ENTDI2TDO;          // disable TDI2TDO
            TGTCTRLOUT |=  SELT;               // disable JTAG & RST pin drivers
            IHIL_Delay_1ms(1);
            // hands off from target TEST pin -> will be pulled down
            TSTCTRLOUT |=  TEST;               // reset target TEST pin low
        }
        _edt_Distinct_Methods.TapReset  = _hil_dummy_TapReset;
        _edt_Distinct_Methods.CheckJtagFuse = _hil_dummy_CheckJtagFuse;
        _edt_Distinct_Methods.Instr = _hil_dummy_Instr;
        _edt_Distinct_Methods.SetReg_XBits08 = _hil_dummy_SetReg_XBits08;
        _edt_Distinct_Methods.SetReg_XBits16 = _hil_dummy_SetReg_XBits16;
        _edt_Distinct_Methods.SetReg_XBits20 = _hil_dummy_SetReg_XBits20;
        _edt_Distinct_Methods.SetReg_XBits32 = _hil_dummy_SetReg_XBits32;
        _edt_Distinct_Methods.SetReg_XBits64 = _hil_dummy_SetReg_XBits64;
        _edt_Distinct_Methods.SetReg_XBits8_64 = _hil_dummy_SetReg_XBits8_64;
        _edt_Distinct_Methods.Tclk = _hil_dummy_Tclk;
        _edt_Distinct_Methods.GetPrevInstruction = _hil_dummy_GetPrevInstruction;
        _edt_Distinct_Methods.StepPsa = _hil_dummy_StepPsa;
        _edt_Distinct_Methods.BlowFuse = _hil_dummy_BlowFuse;
        _edt_Distinct_Methods.SetReg_XBits35 = hil_dummy_SetReg_35Bits;
        _edt_Distinct_Methods.SetReg_XBits = hil_dummy_SetReg_XBits;
        _edt_Distinct_Methods.Instr04 =             _hil_dummy_Instr_4;
        _edt_Distinct_Methods.write_read_Dp =       _hil_dummy_Write_Read_Dp;
        _edt_Distinct_Methods.write_read_Ap =       _hil_dummy_Write_Read_Ap;
        _edt_Distinct_Methods.write_read_mem_Ap =   _hil_dummy_Write_Read_Mem_Ap;
        _edt_Distinct_Methods.GetJtagIdCode =       _hil_dummy_read_idcoe;

        jtagReleased = 1;
    }
    IHIL_Delay_1ms(5);
}

#define RSTLOW_SBW   0
#define RSTLOW_JTAG  1
#define RSTHIGH_SBW  2
#define RSTHIGH_JTAG 3

#define qDriveSignals() { *_Jtag.Out |= (_Jtag.TCK + _Jtag.TMS + _Jtag.TDI);   \
                          TSTCTRLOUT |=  ENTDI2TDO;                            \
                          TGTCTRLOUT |=  TGTRST;                               \
                          TGTCTRLOUT &= ~SELT;                                 \
                        }

#define qDriveSbw()     { TSTCTRLOUT |=  TEST;                                 \
                          JTAGOUT |=  sbwdato;                                 \
                          TSTCTRLOUT &= ~ENTDI2TDO;                            \
                          IHIL_Delay_1ms(1);                                    \
                          JTAGOUT &= ~sbwclk;                                  \
                          TGTCTRLOUT &= ~SELT;                                 \
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
    __disable_interrupt();
    TSTset0;                    //1
    IHIL_Delay_1ms(4);           //reset TEST logic

    RSTset0;                    //2

    TSTset1;                    //3
    IHIL_Delay_1ms(50);         //activate TEST logic

    RSTset0;                    //4
    IHIL_Delay_1us(40);

     // for 4-wire JTAG clear Test pin Test(0)
    ((TSTCTRLOUT) |= (TEST));   //5
    IHIL_Delay_1us(2);

    // for 4-wire JTAG -dry  Reset(1)
    TGTCTRLOUT &= ~SELT;
    TGTCTRLOUT |=  TGTRST;
    IHIL_Delay_1us(2);

    // 4-wire JTAG - Test (1)
    ((TSTCTRLOUT) &= (~TEST));  //7
    IHIL_Delay_1ms(5);
    __enable_interrupt();
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
    TSTset0;                    //1
    IHIL_Delay_1ms(1);           //reset TEST logic

    RSTset1;                    //2

    TSTset1;                    //3
    IHIL_Delay_1ms(100);         //activate TEST logic

    RSTset0;                    //4
    IHIL_Delay_1us(40);

    // for 4-wire JTAG clear Test pin Test(0)
    ((TSTCTRLOUT) |= (TEST));   //5
    IHIL_Delay_1us(1);

    // for 4-wire JTAG - Test (1)
    ((TSTCTRLOUT) &= (~TEST));  //7
    IHIL_Delay_1us(40);

    // phase 5 Reset(1)
    TGTCTRLOUT &= ~SELT; TGTCTRLOUT |=  TGTRST;
    IHIL_Delay_1ms(5);
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
    TSTset0;                //1
    IHIL_Delay_1ms(1);       // reset TEST logic

    RSTset1;                //2

    TSTset1;                //3
    IHIL_Delay_1ms(100);     // activate TEST logic

    // phase 1
    RSTset1;                //4
    IHIL_Delay_1us(40);

    // phase 2 -> TEST pin to 0, no change on RST pin
    // for Spy-Bi-Wire
    __disable_interrupt();
    JTAGOUT &= ~sbwclk;     //5
    IHIL_Delay_1us(1);

    // phase 4 -> TEST pin to 1, no change on RST pin
    // for Spy-Bi-Wire
    JTAGOUT |= sbwclk;      //7
    __enable_interrupt();
    IHIL_Delay_1us(40);

    IHIL_Delay_1ms(5);
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
    TSTset0;                //1
    IHIL_Delay_1ms(1);       // reset TEST logic

    RSTset0;                //2

    TSTset1;                //3
    IHIL_Delay_1ms(100);     // activate TEST logic

    // phase 1
    RSTset1;                //4
    IHIL_Delay_1us(40);

    // phase 2 -> TEST pin to 0, no change on RST pin
    // for Spy-Bi-Wire
    __disable_interrupt();
    JTAGOUT &= ~sbwclk;     //5
    IHIL_Delay_1us(1);

    // phase 4 -> TEST pin to 1, no change on RST pin
    // for Spy-Bi-Wire
    JTAGOUT |= sbwclk;      //7
    __enable_interrupt();
    IHIL_Delay_1us(40);
    IHIL_Delay_1ms(5);
}

void _hil_EntrySequences(unsigned char states)
{
    switch(gprotocol_id)
    {
    case SPYBIWIRE:
        if (states == RSTLOW)
        {
            _hil_EntrySequences_RstLow_SBW();
        }
        if (states == RSTHIGH)
        {
            _hil_EntrySequences_RstHigh_SBW();
        }
        break;

    case SPYBIWIREJTAG:
        if (states == RSTLOW)
        {
            _hil_EntrySequences_RstLow_JTAG();
        }
        if (states == RSTHIGH)
        {
            _hil_EntrySequences_RstHigh_JTAG();
        }
        break;

    default:
        TSTset1
        break;
    }
}

// -----------------------------------------------------------------------------
static void _hil_Connect(unsigned char state)
{
    if(jtagReleased)
    {
        _hil_SetProtocol(gprotocol_id);
    }

    if(state == RSTHIGH)
    {
        if(gprotocol_id == SPYBIWIRE)
        {
            qDriveSbw();
            IHIL_Delay_1ms(1);
            _hil_EntrySequences_RstHigh_SBW();
        }
        else
        {
            qDriveSignals();
            IHIL_Delay_1ms(1);
            if(gprotocol_id == SPYBIWIREJTAG)
            {
                _hil_EntrySequences_RstHigh_JTAG();
            }
            else
            {
                TSTset1;
            }
        }
    }
    else // state  == RSTLOW
    {
        if(gprotocol_id == SPYBIWIRE)
        {
            qDriveSbw();
            IHIL_Delay_1ms(1);
            _hil_EntrySequences_RstLow_SBW();
        }
        else
        {
            qDriveSignals();
            IHIL_Delay_1ms(1);
            if(gprotocol_id == SPYBIWIREJTAG)
            {
                _hil_EntrySequences_RstLow_JTAG();

            }
            else
            {
                TSTset1;
            }
        }
    }
    jtagReleased = 0;
}

// -----------------------------------------------------------------------------
short _hil_Open( unsigned char state)
{
    _hil_Connect(state);
    return 0;
}
// -----------------------------------------------------------------------------
short _hil_Close( void )
{
    _hil_Release();
    return 0;
}

// -----------------------------------------------------------------------------
void SetVFuse(void)
{
    long TimeOut = DEF_TIMEOUT;
    VFSettleFlag = 1;
    P2DIR |= BIT7;
    P2SEL |= BIT7;
    CCR0  = TA_SETVF_DIV;
    TACCTL0 = OUTMOD_4 + CCIE;        // outmode toggle
    TACTL = (TASSEL_1 + TACLR + MC_1);// clear TAR and start Timer A in up-mode
    while(VFSettleFlag && TimeOut)
    {
        TimeOut--;
    }
    TACCTL0 = 0;                      // disable CCR0 interrupt
    P2SEL &= ~BIT7;
    P2DIR &= ~BIT7;
}

// -----------------------------------------------------------------------------
void SetVpp(long voltage)
{
    if(voltage)
    {
        SetVFuse();
        if(useTDI)
        {
            VPPon(VPP_ON_TDI);
        }
        else
        {
            VPPon(VPP_ON_TEST);
        }
        IHIL_Delay_1ms(10);
    }
    else
    {
        VPPoff();
    }
}

// -----------------------------------------------------------------------------
void testVpp(unsigned char mode)
{
    if(mode)
    {
        SETTDOOUT;
        SETTDION;
    }
    else
    {
        SETTDIOFF;
        SETTDOIN;
    }

    useTDI = !mode;
}

// -----------------------------------------------------------------------------
void _hil_SetReset(unsigned char value)
{
    if(value)
    {
        RSTset1
    }
    else
    {
        RSTset0
    }
}

// -----------------------------------------------------------------------------
void _hil_SetTMS(unsigned char value)
{
    if(value)
    {
        TMSset1
    }
    else
    {
        TMSset0
    }
}

// -----------------------------------------------------------------------------
void _hil_SetTDI(unsigned char value)
{
    if(value)
    {
        TDIset1
    }
    else
    {
        TDIset0
    }
}

// -----------------------------------------------------------------------------
void _hil_SetTCK(unsigned char value)
{
    if(value)
    {
        TCKset1
    }
    else
    {
        TCKset0
    }
}

// -----------------------------------------------------------------------------
void _hil_SetTest(unsigned char value)
{
    if(value)
    {
        TSTset1
    }
    else
    {
        TSTset0
    }
}
#ifdef uController
    #pragma optimize = low
#endif

// -----------------------------------------------------------------------------
void _hil_initTimerB0(void)
{
    //Empty for UIF
}



void _hil_BSL_EntrySequence1xx_4xx()
{
    if(gprotocol_id == SPYBIWIRE)
    {
        // set Default state of RST and TST
        JTAGOUT |= sbwclk;            // set test to 1
        IHIL_Delay_1ms(1);
        RSTset1;                      // set RST 1;

        // INIT phase
        JTAGOUT &= ~sbwclk;           // set test 0;
        IHIL_Delay_1ms(100);

        JTAGOUT &= ~sbwdato;          // set RST low
        IHIL_Delay_1ms(100);

        JTAGOUT |= sbwclk;            // set test to 1
        IHIL_Delay_1ms(100);

        /*this is the first pulse keep it low for less than 15us */
        JTAGOUT &= ~sbwclk;          // set test 0;
        IHIL_Delay_1us(10);
        JTAGOUT |= sbwclk;           // set test 1;

        IHIL_Delay_1ms(100);
        RSTset1;
        IHIL_Delay_1ms(100);
        JTAGOUT &= ~sbwclk;         // set test 0;
        IHIL_Delay_1ms(100);
    }
    else
    {
        // set Default state of RST and TST
        ((TSTCTRLOUT) &= (~TEST));    // set test to 1
        IHIL_Delay_1ms(1);
        RSTset1;                      // set RST 1;

        // INIT phase
        ((TSTCTRLOUT) |= (TEST));     // set test 0;
        IHIL_Delay_1ms(10);

        { TGTCTRLOUT &= ~SELT; TGTCTRLOUT &= ~TGTRST; }// set RST 0
        IHIL_Delay_1ms(10);

        ((TSTCTRLOUT) &= (~TEST));    // set test to 1
        IHIL_Delay_1ms(10);

        /*this is the first pulse keep it low for less than 15us */

        ((TSTCTRLOUT) |= (TEST));     // set test 0;
        _hil_SetTCK(0);
        IHIL_Delay_1us(10);
        ((TSTCTRLOUT) &= ~(TEST));    // set test 1;
        _hil_SetTCK(1);

        IHIL_Delay_1ms(10);
        _hil_SetTCK(0);
        RSTset1;                      // set RST 1;
        IHIL_Delay_1ms(10);
        ((TSTCTRLOUT) |= (TEST));     // set test 0 ;
        _hil_SetTCK(1);
        IHIL_Delay_1ms(10);
    }
 }


/*------------------------------------------------------------------------*/
/*                      __________     <15us    __________                */
/* Test/SBWTCK ________|          |____________|          |___            */
/*                                                  __________            */
/* RST/SBWTDIO ____________________________________|                      */
/*                                                                        */
/*                                                 BSL Enabled here!!     */
/*------------------------------------------------------------------------*/
void _hil_BSL_EntrySequence(unsigned short dummy)
{
    if(gprotocol_id == SPYBIWIRE)
    {
        // set Default state of RST and TST
        JTAGOUT |= sbwclk;            // set test to 1
        IHIL_Delay_1ms(1);
        RSTset1;                      // set RST 1;

        // INIT phase
        JTAGOUT &= ~sbwclk;           // set test 0;
        IHIL_Delay_1ms(100);

        JTAGOUT &= ~sbwdato;          // set RST low
        IHIL_Delay_1ms(100);

        JTAGOUT |= sbwclk;            // set test to 1
        IHIL_Delay_1ms(100);

        /*this is the first pulse keep it low for less than 15us */
        JTAGOUT &= ~sbwclk;          // set test 0;
        IHIL_Delay_1us(10);
        JTAGOUT |= sbwclk;           // set test 1;

        IHIL_Delay_1ms(100);
        RSTset1;
        IHIL_Delay_1ms(100);
        JTAGOUT &= ~sbwclk;         // set test 0;
        IHIL_Delay_1ms(100);
    }
    else
    {
        // set Default state of RST and TST
        ((TSTCTRLOUT) &= (~TEST));    // set test to 1
        IHIL_Delay_1ms(1);
        RSTset1;                      // set RST 1;

        // INIT phase
        ((TSTCTRLOUT) |= (TEST));     // set test 0;
        IHIL_Delay_1ms(10);

        { TGTCTRLOUT &= ~SELT; TGTCTRLOUT &= ~TGTRST; }// set RST 0
        IHIL_Delay_1ms(10);

        ((TSTCTRLOUT) &= (~TEST));    // set test to 1
        IHIL_Delay_1ms(10);

        /*this is the first pulse keep it low for less than 15us */
        ((TSTCTRLOUT) |= (TEST));     // set test 0;
        IHIL_Delay_1us(10);
        ((TSTCTRLOUT) &= ~(TEST));    // set test 1;

        IHIL_Delay_1ms(10);
        RSTset1;                      // set RST 1;
        IHIL_Delay_1ms(10);
        ((TSTCTRLOUT) |= (TEST));     // set test 0 ;
        IHIL_Delay_1ms(10);
    }
 }

// -----------------------------------------------------------------------------
#ifdef uController
    #pragma optimize = none
#endif
void JSBW_EntrySequences(unsigned char states)
{

    qDriveSignals();
    IHIL_Delay_1ms(10);

    TSTset0;
    IHIL_Delay_1ms(1); // reset TEST logic

    if(states == 0 || states == 1)
    {
        TGTCTRLOUT &=  ~TGTRST;//RST 0
    }
    else
    {
        RSTset1;
    }
    IHIL_Delay_1us(10);

    TSTset1;
    IHIL_Delay_1ms(25); // activate TEST logic

    // phase 1
    if(states == 1 || states == 3)
    {
        RSTset0;
    }
    else
    {
        RSTset1;
    }
    IHIL_Delay_1us(40);


    // phase 2 -> TEST pin to 0, no change on RST pin
    if(states == 0 || states == 2)
    { // for Spy-Bi-Wire
        TSTCTRLOUT |= JSBsbwclk;
    }
    else
    { // for 4-wire JTAG
        ((TSTCTRLOUT) |= (TEST));   //5 clear the bastard test
    }

    // phase 3
    if(states == 1)
    {
         TGTCTRLOUT &= ~SELT;    // set the reset 1
         TGTCTRLOUT |=  TGTRST;  //6
    }
    IHIL_Delay_1us(1);

    // phase 4 -> TEST pin to 1, no change on RST pin
    if(states == 0 || states == 2)
    { // for Spy-Bi-Wire
        TSTCTRLOUT &=  ~JSBsbwclk;
    }
    else
    { // for 4-wire JTAG set Test pin
        ((TSTCTRLOUT) &= (~TEST));  //5
    }
    IHIL_Delay_1us(40);

    // phase 5
    if(states == 3)
    {
        TGTCTRLOUT &= ~SELT;
        TGTCTRLOUT |=  TGTRST;
    }
    IHIL_Delay_1ms(5);
}
/* EOF */
