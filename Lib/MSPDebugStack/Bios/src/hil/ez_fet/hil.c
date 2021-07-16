/*
 * hil.c
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
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
#include "edt.h"
#include "hilDelays.h"
#include "error_def.h"
#include "JTAG_defs.h"
#include "../../fw/fet/FetVersion.h"

#define HIL_SIGNATURE      0xF00DF00Dul

//! \brief version of bios code
const unsigned short hil_version_ @ "HILVERSION" = EZ_FET_HIL_VERSION;
#pragma required=hil_version_
const unsigned long hil_signature_ @ "HILSIGNATURE" = HIL_SIGNATURE;
#pragma required=hil_signature_

const unsigned short hil_versionCmp_ @ "HILVERSIONCMP" = EZ_FET_HIL_VERSION_CMP;
#pragma required=hil_versionCmp_


unsigned short gTclkHighWhilePsa;
unsigned short bVccOn;  // Target Vcc is switched off by default
                        // dependant on calls to _hil_SetVcc()

unsigned short setPCclockBeforeCapture;
unsigned short jtagReleased;


unsigned short toolId;

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
void _hil_EntrySequences(unsigned char states);
void _hil_BSL_EntrySequence1xx_4xx();

void _hil_SetReset(unsigned char value);
void _hil_SetTest(unsigned char value);
void _hil_SetTMS(unsigned char value);
void _hil_SetTCK(unsigned char value);
void _hil_SetTDI(unsigned char value);

void _hil_SetJtagSpeed(unsigned short jtagSpeed, unsigned short sbwSpeed);
void _hil_ConfigureSetPc (unsigned short PCclockBeforeCapture);

void _hil_SwitchVccFET(unsigned short switchVccFET);

void _hil_SetToolID(unsigned short id);
// protocol specific methods
/*extern short _hil_4w_TapReset(void);
extern short _hil_4w_CheckJtagFuse(void);
extern unsigned short _hil_4w_EnumChain(void);
extern unsigned char _hil_4w_Instr(unsigned char Instruction);
extern unsigned char _hil_4w_SetReg_XBits08(unsigned char Data);
extern unsigned short _hil_4w_SetReg_XBits16(unsigned short Data);
extern unsigned long _hil_4w_SetReg_XBits20(unsigned long Data);
extern unsigned long _hil_4w_SetReg_XBits32(unsigned long Data);
extern unsigned long long _hil_4w_SetReg_XBits64(unsigned long long Data);
extern void  _hil_4w_Tclk(unsigned char state);
extern void  _hil_4w_StepPsa(unsigned long length);
extern void  _hil_4w_StepPsaTclkHigh(unsigned long length);
extern short _hil_4w_BlowFuse(unsigned char targetHasTestVpp);*/
//extern void  _hil_4w_ConfigureSpeed(unsigned short speed);

//SBW2 DMA
extern short _hil_2w_TapReset_Dma(void);
extern short _hil_2w_CheckJtagFuse_Dma(void);
extern unsigned short _hil_2w_EnumChain_Dma(void);
extern unsigned char _hil_2w_Instr_Dma(unsigned char Instruction);
extern unsigned char _hil_2w_SetReg_XBits08_Dma(unsigned char Data);
extern unsigned short _hil_2w_SetReg_XBits16_Dma(unsigned short Data);
extern unsigned long _hil_2w_SetReg_XBits20_Dma(unsigned long Data);
extern unsigned long _hil_2w_SetReg_XBits32_Dma(unsigned long Data);
extern unsigned long long _hil_2w_SetReg_XBits64_Dma(unsigned long long Data);
extern unsigned long long _hil_2w_SetReg_XBits8_64_Dma(unsigned long long Data, unsigned short loopCount, unsigned short JStateVersion);
extern void  _hil_2w_Tclk_Dma(unsigned char state);
extern void  _hil_2w_StepPsa_Dma(unsigned long length);
extern void  _hil_2w_StepPsaTclkHigh_Dma(unsigned long length);
extern short _hil_2w_BlowFuse_Dma(unsigned char targetHasTestVpp);
extern void  _hil_2w_ConfigureSpeed_Dma(unsigned short speed);
void _hil_2w_StepPsa_Dma_Xv2(unsigned long length);
extern unsigned char _hil_2w_GetPrevInstruction_Dma();
extern unsigned char DMA_TMSL_TDIL[];

short _hil_dummy_TapReset(void) {return 0;}
short _hil_dummy_CheckJtagFuse(void){return 0;}
unsigned short _hil_dummy_EnumChain(void){return 0;}
unsigned char _hil_dummy_Instr(unsigned char Instruction){return 0;}
unsigned char _hil_dummy_SetReg_XBits08(unsigned char Data){return 0;}
unsigned short _hil_dummy_SetReg_XBits16(unsigned short Data){return 0;}
unsigned long _hil_dummy_SetReg_XBits20(unsigned long Data){return 0;}
unsigned long _hil_dummy_SetReg_XBits32(unsigned long Data){return 0;}
unsigned long long _hil_dummy_SetReg_XBits64(unsigned long long Data){return 0;}
unsigned long long _hil_dummy_SetReg_XBits8_64(unsigned long long Data, unsigned short loopCount, unsigned short JStateVersion){return 0;}
void _hil_dummy_Tclk(unsigned char state){return;}
void _hil_dummy_StepPsa(unsigned long length){return;}
void _hil_dummy_StepPsaTclkHigh(unsigned long length){return;}
short _hil_dummy_BlowFuse(unsigned char targetHasTestVpp){return 0;}
void _hil_dummy_StepPsa_Xv2(unsigned long length){return;}
unsigned char _hil_dummy_GetPrevInstruction(){return 0;}
short _dummy_regulateVcc(void) { return 0; }

// PSA distinct methods
void _hil_EnhancedPsaSetup(unsigned long address);
void _hil_PsaSetup(unsigned long address);
void _hil_EnhancedPsaEnd(void);
void _hil_PsaEnd(void);
void _hil_Release(void);
void _hil_initTimerB0(void);
void _hil_BSL_EntrySequence(unsigned short dummy);
static void _hil_Connect(unsigned char state);

extern void initJtagSbw2Dma(struct jtag tmp);
//extern void initJtagSbw4(struct jtag tmp);


edt_common_methods_t   _edt_Common_Methods;
edt_distinct_methods_t _edt_Distinct_Methods;

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

unsigned short gprotocol_id;
static unsigned short hil_sbw2Speed_;

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
//#pragma inline=forced
void TCLKset1()
{
   _edt_Distinct_Methods.Tclk(1);
}

//#pragma inline=forced
void TCLKset0()
{
   _edt_Distinct_Methods.Tclk(0);
}
#pragma inline=forced
void TCLK()
{
    TCLKset0();
    TCLKset1();
}

#define CAL_ADC_GAIN_FACTOR  *((unsigned short *)0x1A16)
#define CAL_ADC_OFFSET  *((short *)0x1A18)

/*----------------------------------------------------------------------------
   This function performs a single AD conversion on the selected pin.
   Uses internal reference 2500mV (VR+ = VREF+).
   Arguments: word pinmask (bit position of selected analog input Ax)
   Result:    word (12 bit ADC conversion result)
*/
static float ConvertAD(short channel)
{
    volatile unsigned long notCalibratedAdc = 0;

    ADC12CTL0  &= ~ADC12ENC;              // Disable conversion, write controls
    ADC12MCTL0  = ADC12SREF_1 + channel;  // select Vref and analog channel Ax
    ADC12CTL0  |= ADC12ENC;               // Enable conversions
    ADC12CTL0  |= ADC12SC;                // Start conversions
    while ((ADC12IFG & BIT0) == 0);       // wait until conversion is done

    notCalibratedAdc = ADC12MEM0;
    notCalibratedAdc = ((notCalibratedAdc * CAL_ADC_GAIN_FACTOR) / 32768) + (long)CAL_ADC_OFFSET;

    return(((float)ADC12MEM0* A_VREFPLUS) / ADC_CONV_RANGE);      // return conversion result from MEM0 value in mv
}

static int SetVCoreUp (unsigned short level)
{
        // Open PMM registers for write access
    PMMCTL0_H = 0xA5;
        // Set SVS/SVM high side new level
    SVSMHCTL = SVSHE + SVSHRVL0 * level + SVMHE + SVSMHRRL0 * level;
        // Set SVM low side to new level
    SVSMLCTL = SVSLE + SVMLE + SVSMLRRL0 * level;
        // Wait till SVM is settled
    while ((PMMIFG & SVSMLDLYIFG) == 0);
        // Clear already set flags
    PMMIFG &= ~(SVMLVLRIFG + SVMLIFG);
        // Set VCore to new level
    PMMCTL0_L = PMMCOREV0 * level;
        // Wait till new level reached
    if ((PMMIFG & SVMLIFG))
      while ((PMMIFG & SVMLVLRIFG) == 0);
        // Set SVS/SVM low side to new level
    SVSMLCTL = SVSLE + SVSLRVL0 * level + SVMLE + SVSMLRRL0 * level;
        // Lock PMM registers for write access
    PMMCTL0_H = 0x00;

    return 0;
}

void _hil_Init_Stand_Alone( void )
{
    // Stop watchdog timer to prevent time out reset

    SetVCoreUp(2);
     __delay_cycles(30000);

    //****** WE ARE NOT USING XT1 top source the clock **********

    UCSCTL5 = 0;            // DIVPA, DIVA, DIVS, DIVM -> all direct (DIV=1)
    P5SEL = BIT2+BIT3;      // XT2 SELECT BITS
    UCSCTL6 &=~XT2OFF;      // XT2 ON
    UCSCTL3 = SELREF_2;     // DCO SELECTION

    //Loop until XT1,XT2 & DCO stabilizes
    UCSCTL4 |= SELA__REFOCLK + SELA__XT2CLK; // SELECT ACLK_XT1 AND SMCLK_XT2 TO AVOID OSCILLATOR FAULT FLAG XT1OIFG
    do
    {
        UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG /*+ XT1HFOFFG*/ + DCOFFG); //Clear XT2,XT1,DCO fault flags
        SFRIFG1 &= ~OFIFG;                                      //Clear fault flags
    }
    while( SFRIFG1 & OFIFG );


    __bis_SR_register(SCG0);    // disable the FLL for config
    UCSCTL1 = DCORSEL_6;       // DCO-freq range up to min 39MHz (must be higher then 18MHz*2 = 36 MHz)
    UCSCTL2 = FLLD0 + 9*FLLN0;  //DCO-DIV/2 + PLL MULTI*(9+1), freq = 10*2 = 20 MHz
    UCSCTL3 = SELREF_5+FLLREFDIV_2; //Reference - XT2-CLK, XT2/2 = 2MHz
    __bic_SR_register(SCG0);  // Enable the FLL control loop after config

    //Loop until XT1,XT2 & DCO stabilizes
    do
    {
        UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG /*+ XT1HFOFFG*/ + DCOFFG);     //Clear XT2,XT1,DCO fault flags
        SFRIFG1 &= ~OFIFG;                                              //Clear fault flags
    }
    while( SFRIFG1 & OFIFG );
    __delay_cycles(60000);

    UCSCTL4 = SELA__XT2CLK + SELS__DCOCLK + SELM__DCOCLK ;//SELM__DCOCLKDIV; //SELM__DCOCLK;//  + SELM__DCOCLKDIV;

    // Enable Vref=2.5V for ADC
    REFCTL0 |= REFMSTR;
    REFCTL0 |= REFVSEL1;
    REFCTL0 |= REFON;

   // Configure ADC12
   // select channel and do conversion
   ADC12CTL0 = ADC12ON + ADC12SHT0_10 + ADC12REF2_5V + ADC12REFON + ADC12MSC;         // Turn on ADC12, set sampling time
   ADC12CTL1 = ADC12SHP + ADC12CONSEQ_1 + ADC12SSEL_3 + ADC12DIV_7;                   // Use sampling timer

   ADC12MCTL0  = ADC12SREF_1 + 0;               // select Vref and analog channel A0
   __delay_cycles((15*90));
   ADC12CTL0 |= ADC12ENC;                       // Enable conversion

    // set default debug protocol to JTAG
    gprotocol_id = SPYBIWIRE;
    // set default to TCLK low when doing PSA
    gTclkHighWhilePsa = 0;
    // initialize function pointers to distinct functions
    _hil_SetProtocol(SPYBIWIRE);

    toolId = 0;
}

static void _hil_initEdtCommenMethods()
{
    _edt_Common_Methods.Init = _hil_Init;

    _edt_Common_Methods.SetVcc = _hil_SetVcc;
    _edt_Common_Methods.GetVcc = _hil_GetVcc;

    _edt_Common_Methods.SetProtocol = _hil_SetProtocol;
    _edt_Common_Methods.SetPsaTCLK = _hil_SetPsaTCLK;

    _edt_Common_Methods.Open = _hil_Open;
    _edt_Common_Methods.Close = _hil_Close;

    _edt_Common_Methods.Delay_1us = _hil_Delay_1us;
    _edt_Common_Methods.Delay_1ms = _hil_Delay_1ms;

    _edt_Common_Methods.Loop = 0;

    _edt_Common_Methods.EntrySequences = _hil_EntrySequences;

    _edt_Common_Methods.SetReset = _hil_SetReset;
    _edt_Common_Methods.SetTest  = _hil_SetTest;
    _edt_Common_Methods.SetTMS = _hil_SetTMS;
    _edt_Common_Methods.SetTCK  = _hil_SetTCK;
    _edt_Common_Methods.SetTDI  = _hil_SetTDI;

    _edt_Common_Methods.SetJtagSpeed = _hil_SetJtagSpeed;

    _edt_Common_Methods.ConfigureSetPc = _hil_ConfigureSetPc;
    _edt_Common_Methods.initDelayTimer = _hil_initTimerB0;

    _edt_Common_Methods.BSL_EntrySequence = _hil_BSL_EntrySequence;
    _edt_Common_Methods.BSL_EntrySequence1xx_4xx = _hil_BSL_EntrySequence1xx_4xx;
    _edt_Common_Methods.regulateVcc = _dummy_regulateVcc;

    _edt_Common_Methods.SwitchVccFET = _hil_SwitchVccFET;
    _edt_Common_Methods.SetToolID = _hil_SetToolID;

}

void _hil_SetToolID(unsigned short id)
{
    toolId = id;
}


void _hil_getEdtDistinct(edt_distinct_methods_t* edt_distinct);
void _hil_getEdtCommen(edt_common_methods_t* edt_commen);

#pragma required=_hil_getEdtDistinct
#pragma required=_hil_getEdtCommen

void _hil_startUp()
{
    _hil_initEdtCommenMethods();
    _edt_Common_Methods.Init();
    
    // DMA4 workaround
    DMACTL4 |= DMARMWDIS;
    
    return;
}
#pragma location="INFO_C_DISTINCT"
void _hil_getEdtDistinct(edt_distinct_methods_t* edt_distinct)
{
    *edt_distinct = _edt_Distinct_Methods;
    if(edt_distinct!= 0)
    {
        return;
    }
}

#pragma location="INFO_C_COMMEN"
void _hil_getEdtCommen(edt_common_methods_t* edt_commen)
{
    *edt_commen = _edt_Common_Methods;
    if(edt_commen != 0)
    {
        return;
    }
}

void _hil_initTimerB0(void)
{
    // Setup timer_A for hardware delay
    TB0CTL = 0;                                          // STOP Timer
    TB0CTL =  ID__8 +TBSSEL__SMCLK;                      // Timer_B source:SMCLK ,SMCLK/0 = 20
    TB0CCR0 = 0xc6c;                                     // Load CCR0 with delay... (1ms delay)
    TB0CCTL0 &= ~CCIE;
}

void _hil_BSL_EntrySequence1xx_4xx()
{
    _hil_BSL_EntrySequence(0);
}

// -----------------------------------------------------------------------------
void _hil_BSL_EntrySequence(unsigned short dummy)
{
    // set Default state of RST and TST
    (*_Jtag.Out) |= _Jtag.TST;
    _hil_Delay_1ms(1);
    RSTset1();

    // INIT phase
    TSTset0();
    _hil_Delay_1ms(100);

    RSTset0();    // set RST 0
    _hil_Delay_1ms(100);

    TSTset1();    // set test to 1
    _hil_Delay_1ms(100);

    /*this is the first pulse keep it low for less than 15us */
    (*_Jtag.Out) &= ~_Jtag.TST;
    _hil_Delay_1us(10);
    TSTset1();     // set test 1;

    _hil_Delay_1ms(100);
    RSTset1();     // set RST 1;
    _hil_Delay_1ms(100);
    TSTset0();     // set test 0;
    _hil_Delay_1ms(100);
}

// -----------------------------------------------------------------------------
void hil_initTimerA0(void)
{
    // Setup time_A0 for timestamping
    TA0CTL = MC__STOP;                                  // STOP Timer
    TA0CCR0 = 0xFFFF;                                   // TimerA2 Period
    TA0CTL = ID__8 + TASSEL__SMCLK;                     // Timer_A0 source:SMCLK/8 = 25 / 8 = 3.125 MHz = 1/320ns
    TA0EX0 = 1;                                         // Timer_A0 set additional divider to /2
    TA0CTL |= TACLR + MC__CONTINOUS + TAIE;             // START the timer in free-running mode
}

// -----------------------------------------------------------------------------
void hil_initTimerA2(void)
{
    // Setup timer_A2 for current pulse measurement
    P2DIR &= ~(1<<2);                                   // P2.2 input
    P2SEL |=  (1<<2);                                   // Select pin
    TA2CTL = MC__STOP;                                  // STOP Timer
    TA2CCR0 = 0xFFFF;                                   // TimerA2 Period
    TA2CTL = TASSEL__TACLK + MC_2 + TAIE;               // Timer_A2 source
}

// -----------------------------------------------------------------------------

static vccSupply_t lastSupply = NO_SUPPLY;


#pragma optimize = low
short _hil_Init( void )
{
    hil_initTimerA0();              // TimeStamp
    hil_initTimerA2();              // Current pulse counter
    _hil_initTimerB0();              // Delay loop timer

    // Enable Vref=2.5V for ADC
    REFCTL0 |= REFMSTR;
    REFCTL0 |= REFVSEL1;
    REFCTL0 |= REFON;

    // Configure ADC12
    // select channel and do conversion
    ADC12CTL0 = ADC12ON + ADC12SHT02;     	// Turn on ADC12, set sampling time
    ADC12CTL1 = ADC12SHP;                        // Use sampling timer
    ADC12MCTL0  = ADC12SREF_1 + 0;               // select Vref and analog channel A0
    __delay_cycles((15*90));
    ADC12CTL0 |= ADC12ENC;                       // Enable conversion

    DMACTL2 = 0;
    DMACTL1 = 0;

    //make sure that ET is on
    P2DIR |= BIT0;
    P2OUT |= BIT0;

    lastSupply = ET_ON;

    // set default debug protocol to JTAG
    gprotocol_id = SPYBIWIRE;
    // set default to TCLK low when doing PSA
    gTclkHighWhilePsa = 0;
    // initialize function pointers to distinct functions
    _hil_SetProtocol(SPYBIWIRE);

    jtagReleased = 1;

    return 0;
}

#pragma optimize = low
void _hil_SwitchVccFET(unsigned short switchVccFET)
{

    if(toolId == eZ_FET_WITH_DCDC_V2x)
    {
        _DINT_FET();
        vccSupply_t type = (vccSupply_t)switchVccFET;
        if(type == ET_ON)
        {
            P2DIR |= (BIT0|BIT1);
            if(lastSupply == LDO_ON)
            {
                // Switch ET on the power switch
                P2OUT |= BIT0;
                // Switch LDO off on the power switch
                P2OUT &= ~BIT1;
            }
            else
            {
                // Switch LDO off on the power switch
                P2OUT &= ~BIT1;
                // Switch ET on the power switch
                P2OUT |= BIT0;
            }
            //LDO LED OFF
            P1DIR &= ~BIT4;
            P1OUT &= ~BIT4;
            lastSupply = type;
        }
        else if(type == ALL_OFF)
        {
            // Turn off the Power switch
            P2DIR &= ~(BIT0|BIT1);
            P2OUT &= ~(BIT0|BIT1);

            //LED LDO off
            P1OUT &= ~BIT4;
            _hil_Release();
            lastSupply = type;
        }
        else if(type == LDO_ON)
        {
            // Switch LDO on the power switch
            P2DIR |= (BIT0|BIT1);
            if(lastSupply == ET_ON)
            {
                // Switch LDO ON on the power switch
                P2OUT |= BIT1;
                // Switch ET off on the power switch
                P2OUT &= ~BIT0;
            }
            else
            {
                // Switch LDO ON on the power switch
                P2OUT |= BIT1;
                 // Switch ET off on the power switch
                P2OUT &= ~BIT0;
            }
            //LDO LED ON
            P1DIR |= BIT4;
            P1OUT |= BIT4;
            lastSupply = type;
        }
        else
        {
            // Turn off the Power switch
            P2DIR &= ~(BIT0|BIT1);
            P2OUT &= ~(BIT0|BIT1);

            //LED LDO off
            P1OUT &= ~BIT4;
            _hil_Release();
            lastSupply = NO_SUPPLY;
        }
        _EINT_FET();
    }
    else
    {
        if(switchVccFET == LDO_ON || switchVccFET == ET_ON)
        {
            _hil_SetVcc(3300);
        }
        else
        {
            _hil_SetVcc(0);
        }
    }
}


// -----------------------------------------------------------------------------
short _hil_SetVcc(unsigned short Vcc)
{
    if(Vcc)
    { // Turn on the power switch
        P2DIR &= ~BIT1;
        P2DIR |= BIT0;
        P2OUT |= BIT0;
    }
    else
    { // Turn off the Power switch
        P2DIR &= ~(BIT0|BIT1);
        P2OUT &= ~(BIT0|BIT1);

        _hil_Release();
    }
    return 0;
}
// -----------------------------------------------------------------------------

// Calculate ExtVcc based on Vref+=2.5V, R27=250kOhm, R28=250kOhm
// Calculate Vcc    based on Vref+=2.5V, R29=500kOhm, R30=500kOhm
// Calculate VBus   based on Vref+=2.5V, R31=250kOhm, R32=150kOhm

#define  R31            250.0f           // 250kOhm
#define  R32            150.0f           // 150kOhm
short _hil_GetVBus(float* VccVBus)
{
    unsigned short i = 0;
    float vbus_mv = 0, vbus_mv0 = 0;
    for (i=0; i<ADC_AVERAGE; i++)
    {
        vbus_mv0 = ConvertAD(A_VBUS);
        vbus_mv0 = (vbus_mv0 * (R31 + R32)) / R32;
        vbus_mv0 = vbus_mv0*1000;

        // Low pass filter
        if (vbus_mv == 0)
        {
            vbus_mv = vbus_mv0;
        }
        vbus_mv = ((vbus_mv0 * 3) + (vbus_mv * 7)) / 10;
    }
    *VccVBus = vbus_mv;
    return 0;
}

#pragma optimize = low
short _hil_GetVcc(double* Vcc, double* ExtVcc)
{
    float VccTmp = 0;
    VccTmp = ((ConvertAD(A_VCCOUT)) * (R29 + R30)) / R30;
    VccTmp = VccTmp * 1000; // change to mV
    *Vcc = VccTmp;
    *ExtVcc = 0;
    return 0;
}
// -----------------------------------------------------------------------------
short _hil_SetProtocol(unsigned short protocol_id)
{
    short ret_value = 0;

    if(protocol_id == SPYBIWIRE)
    {
        gprotocol_id = SPYBIWIRE;
        _Jtag = _Jtag_Target;
        initJtagSbw2Dma(_Jtag);
        _hil_2w_ConfigureSpeed_Dma(hil_sbw2Speed_);
    }
    else if(protocol_id == SPYBIWIRE_SUBMCU)
    {
        gprotocol_id = SPYBIWIRE;
        _Jtag = _Jtag_SubMcu;
        initJtagSbw2Dma(_Jtag);
        _hil_2w_ConfigureSpeed_Dma(SBW400KHz);
    }
    else if(protocol_id == SPYBIWIREJTAG)
    {
        ret_value = HALERR_START_JTAG_PROTOCOL_UNKNOWN;
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
        return ret_value;
    }
    else
    {
        ret_value = -1;
    }
    // load DMA1 with size just default
    DMA1CTL = ( DMADT0 | DMASRCINCR1 | DMASRCINCR0 | DMASRCBYTE | DMADSTBYTE);
    DMA1DA =  (_Jtag.Out); //JTAGOUT;       // set destination address
    DMA2CTL = ( DMADT0 | DMASRCINCR1 | DMASRCINCR0 | DMASRCBYTE | DMADSTBYTE);
    DMA2DA =  (_Jtag.Out); //JTAGOUT;       // set destination address

    _edt_Distinct_Methods.TapReset =            _hil_2w_TapReset_Dma;
    _edt_Distinct_Methods.CheckJtagFuse =       _hil_2w_CheckJtagFuse_Dma;
    _edt_Distinct_Methods.Instr =               _hil_2w_Instr_Dma;
    _edt_Distinct_Methods.SetReg_XBits08 =      _hil_2w_SetReg_XBits08_Dma;
    _edt_Distinct_Methods.SetReg_XBits16 =      _hil_2w_SetReg_XBits16_Dma;
    _edt_Distinct_Methods.SetReg_XBits20 =      _hil_2w_SetReg_XBits20_Dma;
    _edt_Distinct_Methods.SetReg_XBits32 =      _hil_2w_SetReg_XBits32_Dma;
    _edt_Distinct_Methods.SetReg_XBits64 =      _hil_2w_SetReg_XBits64_Dma;
    _edt_Distinct_Methods.SetReg_XBits8_64 =    _hil_2w_SetReg_XBits8_64_Dma;
    _edt_Distinct_Methods.Tclk =                _hil_2w_Tclk_Dma;
    _edt_Distinct_Methods.GetPrevInstruction =  _hil_2w_GetPrevInstruction_Dma;

    if(gTclkHighWhilePsa == 1)
    {
        _edt_Distinct_Methods.StepPsa = _hil_2w_StepPsaTclkHigh_Dma;
    }
    else if(gTclkHighWhilePsa == 2)
    {
        _edt_Distinct_Methods.StepPsa = _hil_2w_StepPsa_Dma_Xv2;
    }
    else
    {
        _edt_Distinct_Methods.StepPsa = _hil_2w_StepPsa_Dma;
    }

    _edt_Distinct_Methods.BlowFuse = _hil_2w_BlowFuse_Dma;

    DMA2SA = (unsigned char*)DMA_TMSL_TDIL;

    jtagReleased = 0;
    return(ret_value);
}

void _hil_SetPsaTCLK(unsigned short tclkValue)
{
    gTclkHighWhilePsa = tclkValue;

    if(gprotocol_id == SPYBIWIRE)
    {
        if(gTclkHighWhilePsa == 1)
        {
            _edt_Distinct_Methods.StepPsa = _hil_2w_StepPsaTclkHigh_Dma;
        }
        else if(gTclkHighWhilePsa == 2)
        {
            _edt_Distinct_Methods.StepPsa = _hil_2w_StepPsa_Dma_Xv2;
        }
        else
        {
            _edt_Distinct_Methods.StepPsa = _hil_2w_StepPsa_Dma;
        }
    }
}

void _hil_Release(void)
{
    if(!jtagReleased)
    {
        (*_Jtag.Out) |= _Jtag.RST;
        (*_Jtag.Out) &= ~_Jtag.TST;
        _hil_Delay_1ms(50);

        (*_Jtag.DIRECTION) &= ( ~_Jtag.TST );
        (*_Jtag.DIRECTION) &= ( ~_Jtag.RST );

        _hil_Delay_1ms(5);

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

        jtagReleased = 1;
    }
}

INLINE(forced)
void qDriveJTAG()
{
    (*_Jtag.Out) |= (_Jtag.TCK + _Jtag.TMS + _Jtag.TDI + _Jtag.RST + _Jtag.TST);
    (*_Jtag.DIRECTION) |= (_Jtag.TCK + _Jtag.TMS + _Jtag.TDI);
    (*_Jtag.DIRECTION) |= (_Jtag.TST + _Jtag.RST);
    (*_Jtag.DIRECTION) &= (~_Jtag.TDO);
}


INLINE(forced)
void qDriveSbw()
{
    (*_Jtag.Out) |= _Jtag.RST;
    (*_Jtag.Out) &= ~_Jtag.TST;
    (*_Jtag.DIRECTION) |= ( _Jtag.TST +  _Jtag.RST);
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
    TSTset0();    //1
    _hil_Delay_1ms(4); // reset TEST logic

    RSTset0();    //2

    TSTset1();    //3
    _hil_Delay_1ms(20); // activate TEST logic

    // phase 1
    RSTset0();    //4
    _hil_Delay_1us(60);

    // phase 2 -> TEST pin to 0, no change on RST pin
    // for 4-wire JTAG clear Test pin
    TSTset0();  //5

    // phase 3
    RSTset0();  //6
    _hil_Delay_1us(1);

    // phase 4 -> TEST pin to 1, no change on RST pin
    // for 4-wire JTAG
    TSTset1();//7
    _hil_Delay_1us(60);

    // phase 5
    _hil_Delay_1ms(5);
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
    TSTset0();    //1
    _hil_Delay_1ms(4); // reset TEST logic

    RSTset1();    //2

    TSTset1();    //3
    _hil_Delay_1ms(20); // activate TEST logic

    // phase 1
    RSTset0();    //4
    _hil_Delay_1us(60);

    // phase 2 -> TEST pin to 0, no change on RST pin
    // for 4-wire JTAG clear Test pin
    TSTset0();  //5

    // phase 3
    _hil_Delay_1us(1);

    // phase 4 -> TEST pin to 1, no change on RST pin
    // for 4-wire JTAG
    TSTset1();//7
    _hil_Delay_1us(60);

    // phase 5
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
    TSTset0();    //1
    _hil_Delay_1ms(4); // reset TEST logic

    RSTset1();    //2

    TSTset1();    //3
    _hil_Delay_1ms(20); // activate TEST logic

    // phase 1
    RSTset1();    //4
    _hil_Delay_1us(60);

    // phase 2 -> TEST pin to 0, no change on RST pin
    // for Spy-Bi-Wire
    _DINT_FET();
    (*_Jtag.Out) &= ~_Jtag.TST; //5

    // phase 3
    _hil_Delay_1us(1);

    // phase 4 -> TEST pin to 1, no change on RST pin
    // for Spy-Bi-Wire
    (*_Jtag.Out) |= _Jtag.TST;  //7
    _EINT_FET();
    _hil_Delay_1us(60);

    // phase 5
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
    TSTset0();    //1
    _hil_Delay_1ms(4); // reset TEST logic

    RSTset0();    //2

    TSTset1();    //3
    _hil_Delay_1ms(20); // activate TEST logic

    // phase 1
    RSTset1();    //4
    _hil_Delay_1us(60);

    // phase 2 -> TEST pin to 0, no change on RST pin
    // for Spy-Bi-Wire
    _DINT_FET();
    (*_Jtag.Out) &= ~_Jtag.TST; //5

    // phase 3
    _hil_Delay_1us(1);

    // phase 4 -> TEST pin to 1, no change on RST pin
    // for Spy-Bi-Wire
    (*_Jtag.Out) |= _Jtag.TST;  //7
    _EINT_FET();
    _hil_Delay_1us(60);

    // phase 5
    _hil_Delay_1ms(5);
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
            _hil_Delay_1ms(1);
            _hil_EntrySequences_RstHigh_SBW();
        }
        else
        {
            qDriveSbw();
            _hil_Delay_1ms(1);
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
    else // state  == RSTLOW
    {
        if(gprotocol_id == SPYBIWIRE)
        {
            qDriveSbw();
            _hil_Delay_1ms(1);
            _hil_EntrySequences_RstLow_SBW();
        }
        else
        {
            qDriveSbw();
            _hil_Delay_1ms(1);
            if(gprotocol_id == SPYBIWIREJTAG)
            {
                _hil_EntrySequences_RstLow_JTAG();
            }
            else
            {
                TSTset1();
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

void _hil_SetJtagSpeed(unsigned short jtagSpeed, unsigned short sbwSpeed)
{
    if(sbwSpeed)
    {
        hil_sbw2Speed_ =  sbwSpeed;
       _hil_2w_ConfigureSpeed_Dma(sbwSpeed);
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
    return;
}

// -----------------------------------------------------------------------------
void SetVpp(long voltage)
{
    return;
}

// -----------------------------------------------------------------------------
void testVpp(unsigned char mode)
{
    return;
}


// -----------------------------------------------------------------------------
void _hil_SetReset(unsigned char value)
{
    if(value)
    {
        RSTset1();
    }
    else
    {
        RSTset0();
    }
}

// -----------------------------------------------------------------------------
void _hil_SetTest(unsigned char value)
{
    if(value)
    {
        TSTset1();
    }
    else
    {
        TSTset0();
    }
}

// -----------------------------------------------------------------------------
void _hil_SetTMS(unsigned char value)
{
  //this function is not use
}

void _hil_SetTCK(unsigned char value)
{
  //this function is not use
}

void _hil_SetTDI(unsigned char value)
{
  //this function is not use
}
/* EOF */
