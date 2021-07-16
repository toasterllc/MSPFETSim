/*
 * hil_4w.c
 *
 * <FILEBRIEF>
 *
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
//! \ingroup MODULHIL
//! \file hil_4w.c
//! \brief
//!

#include "hw_compiler_specific.h"
#include "arch.h"
#include "hilDelays.h"
#include "JTAG_defs.h"
#include "hilFpgaAccess.h"



//! \ingroup MODULHIL
//! \file hil_4w.c
//! \brief
//!

#define uController_uif

extern void TCLKset1();
extern void TCLKset0();

extern void testVpp(unsigned char mode);
extern void setVpp(long voltage);

static struct jtag _Jtag = {0};
//#define JTAG_DELAY   { _NOP();}
void _hil_4w_Tclk(unsigned char state);

// 4-wire JTAG: low level signal access
#pragma inline=forced
void TMSset1()
{ (*_Jtag.Out) |=  _Jtag.TMS;}

#pragma inline=forced
void TMSset0()
{ (*_Jtag.Out) &= ~_Jtag.TMS;}

#pragma inline=forced
void TDIset1()
{ (*_Jtag.Out) |=  _Jtag.TDI;}

#pragma inline=forced
void TDIset0()
{ (*_Jtag.Out) &= ~_Jtag.TDI;}

#pragma inline=forced
void TCKset1()
{ (*_Jtag.Out) |=  _Jtag.TCK;}

#pragma inline=forced
static void TCKset0()
{ (*_Jtag.Out) &= ~_Jtag.TCK;}

#pragma inline=forced
void TDIset1TMSset1()
{ (*_Jtag.Out) |=  _Jtag.TDI | _Jtag.TMS; }

#pragma inline=forced
void TDIset0TMSset1()
{ (*_Jtag.Out) &= ~_Jtag.TDI; *_Jtag.Out |= _Jtag.TMS;}

#pragma inline=forced
unsigned char StoreTTDI()
{
    return *_Jtag.Out;
}
#pragma inline=forced
void RestoreTTDI(unsigned long long x)
{
  if(x &_Jtag.TDI)
  {
    *_Jtag.Out |= _Jtag.TDI;
    //JTAG_DELAY;
  }
  else
  {
    *_Jtag.Out &=  ~_Jtag.TDI;
  }
}
#pragma inline=forced
unsigned char ScanTDO()
{
    if((*_Jtag.In   &  _Jtag.TDO)!= 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void initJtagSbw4(struct jtag tmp)
{
    _Jtag = tmp;
}


// -----------------------------------------------------------------------------
short _hil_4w_CheckJtagFuse(void)
{
    // perform a JTAG fuse check
    hil_fpga_enable_bypass();
    TMSset1();_NOP();_NOP();_NOP();
    TMSset0();_NOP();_NOP();_NOP();
    _hil_Delay_1us(15);
    TMSset1();_NOP();_NOP();_NOP();
    TMSset0();_NOP();_NOP();_NOP();
    _hil_Delay_1us(15);
    TMSset1();_NOP();_NOP();_NOP();
    hil_fpga_disable_bypass();
    return 0;
}

// -----------------------------------------------------------------------------
short _hil_4w_TapReset(void)
{
    // Reset TAP Controller State Machine
    // Set default state for JTAG signals (TDI = TMS = TCK = 1)
    hil_fpga_enable_bypass();
    TDIset1(); _NOP(); _NOP();_NOP();
    TMSset1(); _NOP(); _NOP();_NOP();
    TCKset1(); _NOP(); _NOP();_NOP();
    // Clock TCK six (6) times
    TCKset0(); _NOP(); _NOP();_NOP();
    TCKset1(); _NOP(); _NOP();_NOP();
    TCKset0(); _NOP(); _NOP();_NOP();
    TCKset1(); _NOP(); _NOP();_NOP();
    TCKset0(); _NOP(); _NOP();_NOP();
    TCKset1(); _NOP(); _NOP();_NOP();
    TCKset0(); _NOP(); _NOP();_NOP();
    TCKset1(); _NOP(); _NOP();_NOP();
    TCKset0(); _NOP(); _NOP();_NOP();
    TCKset1(); _NOP(); _NOP();_NOP();
    TCKset0(); _NOP(); _NOP();_NOP();
    TCKset1(); _NOP(); _NOP();_NOP();
    // TAP Controller State Machine is now in "Test-Logic Reset" state
    // Clock TCK one more time with TMS = 0
    TMSset0(); _NOP(); _NOP();_NOP();
    TCKset0(); _NOP(); _NOP();_NOP();
    TCKset1(); _NOP(); _NOP();_NOP();
    hil_fpga_disable_bypass();
    return 0;
}

// -----------------------------------------------------------------------------
void _hil_4w_StepPsa(unsigned long length)
{
    hil_fpga_enable_bypass();

    while(length-- > 0)
    {
        TDIset1(); _NOP();
        TDIset0(); _NOP();

        TCKset0(); _NOP();
        TMSset1(); _NOP();
        TCKset1(); _NOP(); // select DR scan
        TCKset0(); _NOP();
        TMSset0(); _NOP();

        TCKset1(); _NOP(); // capture DR
        TCKset0(); _NOP();
        TCKset1(); _NOP(); // shift DR
        TCKset0(); _NOP();

        TMSset1(); _NOP();
        TCKset1(); _NOP();// exit DR
        TCKset0(); _NOP();

        // Set JTAG FSM back into Run-Test/Idle
        TCKset1(); _NOP();
        TMSset0(); _NOP();
        TCKset0(); _NOP();
        TCKset1(); _NOP();
        _NOP(); _NOP(); _NOP(); _NOP(); _NOP();
    }
    hil_fpga_disable_bypass();
}
// -----------------------------------------------------------------------------
void _hil_4w_StepPsaTclkHigh(unsigned long length)
{
    hil_fpga_enable_bypass();

    while(length--)
    {
        //TCLKset1(); _NOP();
        TDIset1(); _NOP();
        TCKset0(); _NOP();
        TMSset1(); _NOP();
        TCKset1(); _NOP();// select DR scan
        TCKset0(); _NOP();
        TMSset0(); _NOP();

        TCKset1(); _NOP();// capture DR
        TCKset0(); _NOP();
        TCKset1(); _NOP();// shift DR
        TCKset0(); _NOP();

        TMSset1(); _NOP();
        TCKset1(); _NOP(); // exit DR
        TCKset0(); _NOP();

        // Set JTAG FSM back into Run-Test/Idle
        TCKset1(); _NOP();
        TMSset0(); _NOP();
        TCKset0(); _NOP();
        TCKset1(); _NOP();
        _NOP(); _NOP(); _NOP(); _NOP(); _NOP();
        //TCLKset0(); _NOP();
        TDIset0(); _NOP();
    }
    hil_fpga_disable_bypass();
}

// -----------------------------------------------------------------------------
extern unsigned char _hil_generic_Instr(unsigned char Instruction);
extern unsigned char _hil_generic_SetReg_XBits08(unsigned char Data);
extern unsigned short _hil_generic_SetReg_XBits16(unsigned short Data);
extern unsigned long _hil_generic_SetReg_XBits20(unsigned long Data);
extern unsigned long _hil_generic_SetReg_XBits32(unsigned long Data);

extern unsigned char lastTestState;
extern unsigned char lastResetState;


short _hil_4w_BlowFuse(unsigned char targetHasTestVpp)
{
#ifdef MSP_FET

    _DINT_FET();
    if(!targetHasTestVpp)
    {
        _hil_generic_Instr(IR_CNTRL_SIG_16BIT);
        _hil_generic_SetReg_XBits16(0x7401);        // TDOs get TDI functionality (internal)
        testVpp(0);                                 // Switchs also TDO functionality to TDI.
    }
    _hil_generic_Instr(IR_PREPARE_BLOW);            // initialize fuse blowing
    _hil_Delay_1ms(1);

    lastResetState = 1;
    lastTestState = 1;

    hil_fpga_enable_bypass();
    setVpp(1);                                      // Apply fuse blow voltage
    hil_fpga_disable_bypass();

    _hil_generic_Instr(IR_EX_BLOW);                 // execute fuse blowing
    _hil_Delay_1ms(1);
    setVpp(0);                                      // switch VPP off

    testVpp(1);                                     // Restore the normal function of TDO and TDI

    // now perform a BOR via JTAG - we loose control of the device then...
    _hil_generic_Instr(IR_TEST_REG);
    _hil_generic_SetReg_XBits32(0x00000200);
    _EINT_FET();
#endif

    return 0;
}

void hil_4w_432_Seq(unsigned short length, unsigned char *sequence)
{
    unsigned char dataptr = 0;
    unsigned short n = 0;
    while (length--)
    {
        if (n == 0)
        {
            dataptr = *sequence++;
            n = 8;
        }
        if (dataptr & 1)
        {
            (*_Jtag.Out) |= _Jtag.TMS;
            TCKset0();
            _NOP();
            TCKset1();
        }
        else
        {
            (*_Jtag.Out) &= ~_Jtag.TMS;
            TCKset0();
            _NOP();
            TCKset1();
        }
        dataptr >>= 1;
        n--;
    }
}

