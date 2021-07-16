/*
 * hil_w2.c
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
//! \file hil_2w.c
//! \brief
//!

#include "hw_compiler_specific.h"
//#include "HilGlobalVars.h"
#include "arch.h"
#include "hilDelays.h"
#include "JTAG_defs.h"

//#define sbwdato _Jtag.RST
//#define sbwclk  _Jtag.TST
//#define sbwdati _Jtag.RST


//VAR_AT(
unsigned short TCLK_saved;//, HIL_ADDR_VAR_TCLK_SAVED);

void _hil_2w_Tclk(unsigned char state);

//! \ingroup MODULHIL
//! \file hil_2w.c
//! \brief
//!

unsigned char tdo_bit = 0;

// defined in hil.c
extern void SetVpp(long voltage);
extern void TCLKset1();
extern void TCLKset0();

static struct jtag _Jtag = {0};

#define SBW_DELAY   { _NOP(); _NOP(); _NOP(); _NOP(); _NOP(); _NOP(); _NOP();}

#pragma inline=forced
void TMSH()
{
    /*_DINT();*/ (*_Jtag.Out) |=  _Jtag.RST; (*_Jtag.Out) &= ~_Jtag.TST; SBW_DELAY; (*_Jtag.Out) |= _Jtag.TST; /*_EINT();*/ // TMS = 1
}

#pragma inline=forced
void TMSL()
{
    /*_DINT();*/ (*_Jtag.Out) &= ~_Jtag.RST;(*_Jtag.Out) &= ~_Jtag.TST; SBW_DELAY; (*_Jtag.Out) |= _Jtag.TST; /*_EINT();*/  // TMS = 0
}

#pragma inline=forced
void TMSLDH()
{
    /*_DINT();*/ (*_Jtag.Out) &= ~_Jtag.RST;(*_Jtag.Out) &= ~_Jtag.TST; SBW_DELAY; (*_Jtag.Out) |= _Jtag.RST; (*_Jtag.Out) |= _Jtag.TST; /*_EINT();*/ // TMS = 0, then TCLK immediately = 1
}

#pragma inline=forced
void TDIH()
{
  /*_DINT();*/ (*_Jtag.Out) |=  _Jtag.RST; (*_Jtag.Out) &= ~_Jtag.TST; SBW_DELAY; (*_Jtag.Out) |= _Jtag.TST;  /*_EINT();*/ // TDI = 1
}

#pragma inline=forced
void TDIL()
{
    /*_DINT();*/ (*_Jtag.Out) &= ~_Jtag.RST; (*_Jtag.Out) &= ~_Jtag.TST; SBW_DELAY; (*_Jtag.Out) |= _Jtag.TST;  /*_EINT();*/
}

#pragma inline=forced
void TDOsbw()
{
    /*_DINT();*/ (*_Jtag.DIRECTION) &= ~_Jtag.RST; (*_Jtag.Out) &= ~_Jtag.TST; SBW_DELAY; (*_Jtag.Out) |= _Jtag.TST; SBW_DELAY;SBW_DELAY;(*_Jtag.DIRECTION) |= _Jtag.RST;/*_EINT();*/ // TDO cycle without reading TDO
}
#pragma inline=forced
void TDO_RD()
{
    /*_DINT();*/ (*_Jtag.DIRECTION) &= ~_Jtag.RST; (*_Jtag.Out) &= ~_Jtag.TST; SBW_DELAY; tdo_bit = (*_Jtag.In);  SBW_DELAY;SBW_DELAY;(*_Jtag.Out) |= _Jtag.TST; ; (*_Jtag.DIRECTION) |= _Jtag.RST; /*_EINT();*/  // TDO cycle with TDO read
}
// 2-wire Spy-Bi-Wire: low level signal access

#pragma inline=forced
void TMSL_TDIL() {  _DINT_FET() ;TMSL(); TDIL(); TDOsbw(); _EINT_FET(); }

#pragma inline=forced
void  TMSH_TDIL() {  _DINT_FET(); TMSH(); TDIL(); TDOsbw(); _EINT_FET();}

#pragma inline=forced
static void  TMSL_TDIH() {  _DINT_FET(); TMSL(); TDIH(); TDOsbw(); _EINT_FET();}

#pragma inline=forced
void  TMSH_TDIH() {  _DINT_FET(); TMSH(); TDIH(); TDOsbw(); _EINT_FET();}

#pragma inline=forced
void  TMSL_TDIH_TDOrd() {  _DINT_FET(); TMSL(); TDIH(); TDO_RD(); _EINT_FET();}

#pragma inline=forced
void  TMSL_TDIL_TDOrd() { _DINT_FET();  TMSL(); TDIL(); TDO_RD(); _EINT_FET();}

#pragma inline=forced
void  TMSH_TDIH_TDOrd() {  _DINT_FET(); TMSH(); TDIH(); TDO_RD(); _EINT_FET();}

#pragma inline=forced
void  TMSH_TDIL_TDOrd() {  _DINT_FET(); TMSH(); TDIL(); TDO_RD(); _EINT_FET();}

void initJtagSbw2(struct jtag tmp)
{
    _Jtag = tmp;
}
#pragma inline=forced
//#pragma optimize = low
unsigned long long sbw_Shift(unsigned long long Data, short Bits)
{
    unsigned long long TDOvalue = 0x0000000000000000;
    unsigned long long MSB = 0x0000000000000000;

    switch(Bits)
    {
        case F_BYTE: MSB = 0x00000080;
            break;
        case F_WORD: MSB = 0x00008000;
            break;
        case F_ADDR: MSB = 0x00080000;
            break;
        case F_LONG: MSB = 0x80000000;
            break;
        case F_LONG_LONG: MSB = 0x8000000000000000;
            break;
        default: // this is an unsupported format, function will just return 0
            return TDOvalue;
    }
    do
    {
        if ((MSB & 1) == 1)                       // Last bit requires TMS=1
        {
            if(Data & MSB)
            {
                TMSH_TDIH_TDOrd();
            }
            else
            {
                TMSH_TDIL_TDOrd();
            }
        }
        else
        {
            if(Data & MSB)
            {
                TMSL_TDIH_TDOrd();
            }
            else
            {
                TMSL_TDIL_TDOrd();
            }
        }
        TDOvalue <<= 1;                    // TDO could be any port pin
        TDOvalue |= (tdo_bit & _Jtag.RST) > 0;
    }
    while(MSB >>= 1);

    if (TCLK_saved & _Jtag.RST)
    {
        TMSH_TDIH();
        TMSL_TDIH();
    }
    else
    {
        TMSH_TDIL();
        TMSL_TDIL();
    }
    // de-scramble upper 4 bits if it was a 20bit shift
    if(Bits == F_ADDR)
    {
        TDOvalue = ((TDOvalue >> 4) | (TDOvalue << 16)) & 0x000FFFFF;
    }
    return(TDOvalue);
}

// -----------------------------------------------------------------------------
short _hil_2w_TapReset(void)
{
  unsigned short i;

  // Reset JTAG FSM
  for (i = 6; i > 0; i--)      // 6 is nominal
  {
      TMSH_TDIH();
  }
  // JTAG FSM is now in Test-Logic-Reset
  TMSL_TDIH();                 // now in Run/Test Idle
  return 0;
}

unsigned short _hil_2w_EnumChain(void)
{
  return 1; // always return 1, as with Spy-Bi-Wire only one device can be handled
}

short _hil_2w_CheckJtagFuse(void)
{
      TMSL_TDIH();
      TMSH_TDIH();
      _hil_Delay_1ms(1);

      TMSL_TDIH();
      TMSH_TDIH();
      _hil_Delay_1ms(1);

      TMSL_TDIH();
      TMSH_TDIH();
      _hil_Delay_1ms(1);
       // In every TDI slot a TCK for the JTAG machine is generated.
       // Thus we need to get TAP in Run/Test Idle state back again.
      TMSH_TDIH();
      TMSL_TDIH();
      return 0;
}

// -----------------------------------------------------------------------------
unsigned char _hil_2w_Instr(unsigned char Instruction)
{
  // JTAG FSM state = Run-Test/Idle
  if (TCLK_saved & _Jtag.RST) //PrepTCLK
  {
    TMSH_TDIH();
  }
  else
  {
    TMSH_TDIL();
  }
  // JTAG FSM state = Select DR-Scan
  TMSH_TDIH();
  // JTAG FSM state = Select IR-Scan
  TMSL_TDIH();
  // JTAG FSM state = Capture-IR
  TMSL_TDIH();
  // JTAG FSM state = Shift-IR, Shiftin TDI (8 bit)

  return(sbw_Shift(Instruction,F_BYTE));  // JTAG FSM state = Run-Test/Idle
}
// -----------------------------------------------------------------------------
unsigned char _hil_2w_SetReg_XBits08(unsigned char Data)
{
  // JTAG FSM state = Run-Test/Idle
  if (TCLK_saved & _Jtag.RST) //PrepTCLK
  {
    TMSH_TDIH();
  }
  else
  {
    TMSH_TDIL();
  }
  // JTAG FSM state = Select DR-Scan
  TMSL_TDIH();
  // JTAG FSM state = Capture-DR
  TMSL_TDIH();
  // JTAG FSM state = Shift-DR, Shiftin TDI (16 bit)
  return (sbw_Shift(Data, F_BYTE));
  // JTAG FSM state = Run-Test/Idle
}

unsigned short _hil_2w_SetReg_XBits16(unsigned short Data)
{
  // JTAG FSM state = Run-Test/Idle
  if (TCLK_saved & _Jtag.RST) //PrepTCLK
  {
    TMSH_TDIH();
  }
  else
  {
    TMSH_TDIL();
  }
  // JTAG FSM state = Select DR-Scan
  TMSL_TDIH();
  // JTAG FSM state = Capture-DR
  TMSL_TDIH();
  // JTAG FSM state = Shift-DR, Shiftin TDI (16 bit)
  return (sbw_Shift(Data, F_WORD));
  // JTAG FSM state = Run-Test/Idle
}


unsigned long _hil_2w_SetReg_XBits20(unsigned long Data)
{
  // JTAG FSM state = Run-Test/Idle
  if (TCLK_saved & _Jtag.RST) //PrepTCLK
  {
    TMSH_TDIH();
  }
  else
  {
    TMSH_TDIL();
  }
  // JTAG FSM state = Select DR-Scan
  TMSL_TDIH();
  // JTAG FSM state = Capture-DR
  TMSL_TDIH();
  // JTAG FSM state = Shift-DR, Shiftin TDI (16 bit)
  return (sbw_Shift(Data, F_ADDR));
  // JTAG FSM state = Run-Test/Idle
}

unsigned long _hil_2w_SetReg_XBits32(unsigned long Data)
{
  // JTAG FSM state = Run-Test/Idle
  if (TCLK_saved & _Jtag.RST) //PrepTCLK
  {
    TMSH_TDIH();
  }
  else
  {
    TMSH_TDIL();
  }
  // JTAG FSM state = Select DR-Scan
  TMSL_TDIH();
  // JTAG FSM state = Capture-DR
  TMSL_TDIH();
  // JTAG FSM state = Shift-DR, Shiftin TDI (16 bit)
  return (sbw_Shift(Data, F_LONG));
  // JTAG FSM state = Run-Test/Idle
}

unsigned long  long _hil_2w_SetReg_XBits64(unsigned long long Data)
{
  // JTAG FSM state = Run-Test/Idle
  if (TCLK_saved & _Jtag.RST) //PrepTCLK
  {
    TMSH_TDIH();
  }
  else
  {
    TMSH_TDIL();
  }
  // JTAG FSM state = Select DR-Scan
  TMSL_TDIH();
  // JTAG FSM state = Capture-DR
  TMSL_TDIH();
  // JTAG FSM state = Shift-DR, Shiftin TDI (16 bit)
  return (sbw_Shift(Data, F_LONG_LONG));
  // JTAG FSM state = Run-Test/Idle
}

// -----------------------------------------------------------------------------
void _hil_2w_Tclk(unsigned char state)
{
    _DINT();
    if (TCLK_saved & _Jtag.RST) //PrepTCLK
    {
       TMSLDH();
    }
    else
    {
        TMSL();
    }
    if(state)
    {
        (*_Jtag.Out) |= _Jtag.RST;
        TDIH();  TDOsbw();     //ExitTCLK
        TCLK_saved = _Jtag.RST;
    }
    else
    {
        (*_Jtag.Out) &= ~_Jtag.RST;// original
        TDIL();  TDOsbw();     //ExitTCLK
        TCLK_saved = ~_Jtag.RST;
    }
     _EINT();
}

// -----------------------------------------------------------------------------
void _hil_2w_StepPsa(unsigned long length)
{
  while(length > 0)
  {
    TCLKset1();         // inverted DF
    TCLKset0();         // original
    TMSH_TDIH();
    TMSL_TDIH();
    TMSL_TDIH();
    TMSH_TDIH();
    TMSH_TDIH();
    TMSL_TDIH();
    length--;
  }
}

// -----------------------------------------------------------------------------
void _hil_2w_StepPsaTclkHigh(unsigned long length)
{
    while(length--)
    {
        TCLKset1();
        TMSH_TDIH();
        TMSL_TDIH();
        TMSL_TDIH();
        TMSH_TDIH();
        TMSH_TDIH();
        TMSL_TDIH();
        TCLKset0();
    }
}

short _hil_2w_BlowFuse(unsigned char targetHasTestVpp)
{

  return 1;
}
/* EOF */
