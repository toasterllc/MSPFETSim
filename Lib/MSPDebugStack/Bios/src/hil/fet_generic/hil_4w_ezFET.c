/*
 * hil_4w_ezFET.c
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
//! \file hil_4w_ezFET.c
//! \brief
//!

#include "hw_compiler_specific.h"
#include "HalGlobalVars.h"
#include "arch.h"
#include "edt.h"
#include <stdio.h>

#include "stream.h"

extern struct jtag _Jtag;

#define Invalid_Manufactor_IdCode 0x000000FEul
#define Mask_Manufactor_IdCode 0x000000FEul

#define StoreTCLK()     (*_Jtag.Out)
#define RestoreTCLK(x)  (x &  _Jtag.TDI ? (*_Jtag.Out |= _Jtag.TDI) : (*_Jtag.Out &=  ~_Jtag.TDI))
#define ScanTDO()       ((*_Jtag.In   &  _Jtag.TDO) ? 1 : 0)

///< Debug and trace functionality
extern char traceBuffer[256];

// -----------------------------------------------------------------------------
short _hil_4w_ezFET_CheckJtagFuse(void)
{
    // perform a JTAG fuse check
    TMSset1;
    TMSset0;
    EDT_Delay_1us(5);
    TMSset1;
    TMSset0;
    EDT_Delay_1us(5);
    TMSset1;
    return 0;
}

// -----------------------------------------------------------------------------
short _hil_4w_ezFET_TapReset(void)
{
    // Reset TAP Controller State Machine
    // Set default state for JTAG signals (TDI = TMS = TCK = 1)
    TDIset1;
    TMSset1;
    TCKset1;
    // Clock TCK six (6) times
    TCKset0;
    TCKset1;
    TCKset0;
    TCKset1;
    TCKset0;
    TCKset1;
    TCKset0;
    TCKset1;
    TCKset0;
    TCKset1;
    TCKset0;
    TCKset1;
    // TAP Controller State Machine is now in "Test-Logic Reset" state
    // Clock TCK one more time with TMS = 0
    TMSset0;
    TCKset0;
    TCKset1;
    return 0;
}

unsigned short _hil_4w_ezFET_EnumChain(void)
{
/*    unsigned long TDOval = 0x00000000;    // Initialize shifted-in word
    unsigned long MSB = 0x80000000;
    unsigned long LSB = 0x00000001;
    unsigned long DataIn = Invalid_Manufactor_IdCode | LSB;
    unsigned char i = 0;
    unsigned char detIdCode = 0;
    unsigned short numOfDevices = 0;

    _hil_4w_ezFET_TapReset();

    // JTAG FSM state = Run-Test/Idle
    TMSset1;
    EDT_Delay_1us(10);
    TCKset0;
    EDT_Delay_1us(10);
    TCKset1;
    EDT_Delay_1us(10);
    // JTAG FSM state = Select DR-Scan
    TMSset0;
    EDT_Delay_1us(10);
    TCKset0;
    EDT_Delay_1us(10);
    TCKset1;
    EDT_Delay_1us(10);
    // JTAG FSM state = Capture-DR
    TCKset0;
    EDT_Delay_1us(10);
    TCKset1;
    EDT_Delay_1us(10);
    // JTAG FSM state = Shift-IR

    while(1)
    {
        if((DataIn & LSB) == 0)
        {
            TDIset0;
        }
        else
        {
            TDIset1;
        }
        DataIn >>= 1;
        TCKset0;
        EDT_Delay_1us(10);
        TCKset1;
        TDOval >>= 1;			    // TDO could be any port pin
        if (ScanTDO())
        {
            TDOval |= MSB;
            if(0 == detIdCode)                // Test if LSB of IdCode
            {
                i = 0;
                detIdCode = 1;
            }
        }
        else if(0 == detIdCode)
        {
            numOfDevices++;
        }

        i += detIdCode;
        if(32 == i)
        {
            detIdCode = 0;
            i = 0;
            if(Invalid_Manufactor_IdCode == (TDOval & Mask_Manufactor_IdCode))
            {
                // End of chain detected
                break;
            }
            // Device with valid IdCode detected
            numOfDevices++;
        }
        if(0xFF == numOfDevices)
        {
            // Only 254 devices are supported
            // Probably hardware connection is broken
            numOfDevices = 0;
            break;
        }
    }

    TMSset1;
    EDT_Delay_1us(10);
    TCKset0;
    EDT_Delay_1us(10);
    TCKset1;
    // JTAG FSM = Exit-DR
    TCKset0;
    EDT_Delay_1us(10);
    TCKset1;
    EDT_Delay_1us(10);
    // JTAG FSM = Update-DR
    TMSset0;
    EDT_Delay_1us(10);
    TCKset0;
    EDT_Delay_1us(10);
    TCKset1;
    EDT_Delay_1us(10);
    // JTAG FSM = Run-Test/Idle
    TMSset1; // to save power during debugging
    EDT_Delay_1us(10);
    return numOfDevices;*/

    return 1;
}

// -----------------------------------------------------------------------------
unsigned char _hil_4w_ezFET_Instr(unsigned char Instruction)
{
    unsigned char rotInstr = 0;
    for(unsigned char i = 0; i < 7; ++i)
    {
        rotInstr |= Instruction & (1 << (7 - i)) ? (1 << i) : 0;
    }

    STREAM_sendDebug(traceBuffer,sprintf(traceBuffer, "EDT_Instr(0x%x)\n", rotInstr));

    unsigned char TDOvalue = 0;
    unsigned char TClk = StoreTCLK();

    TMSset1
    TCKset0
    TCKset1    // Select-DR

    TCKset0
    TCKset1    // Select-IR

    TMSset0
    TCKset0
    TCKset1    // Capture-IR

    TCKset0
    TCKset1    // Shift-IR

    /// \note Bit-banging application
    for(unsigned char count = 0; count < 7; ++count)
    {
        if(Instruction & (1 << count))
        {
            TDIset1
        }
        else
        {
            TDIset0
        }
        TCKset0
        TCKset1
        TDOvalue <<= 1;
        TDOvalue |= ScanTDO();
    }

    if(Instruction & 0x80)
    {
        TDIset1
    }
    else
    {
        TDIset0
    }
    TMSset1
    TCKset0
    TCKset1    // Exit1-IR

    TDOvalue <<= 1;
    TDOvalue |= ScanTDO();

    RestoreTCLK(TClk); // Restore TDI state since it is used for TCLK

    TCKset0
    TCKset1    // Update-IR

    TMSset0
    TCKset0
    TCKset1    // Run-Test/Idle

    return TDOvalue;
}

// -----------------------------------------------------------------------------
unsigned long long _hil_4w_ezFET_SetReg_XBits(unsigned long long Data, short Bits)
{
  unsigned short tclk = StoreTCLK();		// Store TCLK state;
  unsigned long long TDOvalue = 0;
  unsigned long long MSB = 0x00000001;

  STREAM_sendDebug(traceBuffer,sprintf(traceBuffer, "EDT_SetReg_XBits%d(0x%x)\n", Bits, Data));

  MSB <<= (Bits-1);

  // JTAG FSM state = Run-Test/Idle
  TMSset1
  TCKset0
  TCKset1

  // JTAG FSM state = Select DR-Scan
  TMSset0
  TCKset0
  TCKset1

  // JTAG FSM state = Capture-DR
  TCKset0
  TCKset1

  // JTAG FSM state = Shift-DR
  {
    short i;
    for (i = Bits; i > 0; i--)
    {
      if(Data & MSB)
      {
        TDIset1
      }
      else
      {
        TDIset0
      }
      Data <<= 1;
      if (i == 1)                       // Last bit requires TMS=1
      {
        TMSset1
      }
      TCKset0
      TCKset1
      TDOvalue <<= 1;                    // TDO could be any port pin
      if (ScanTDO())
      {
        TDOvalue++;
      }
    }
  }

  // common exit
  RestoreTCLK(tclk);  		  // restore TCLK state

  // JTAG FSM = Exit-DR
  TCKset0
  TCKset1
  // JTAG FSM = Update-DR
  TMSset0
  TCKset0
  TCKset1
  // JTAG FSM = Run-Test/Idle

  // de-scramble upper 4 bits if it was a 20bit shift
  if(Bits == 20)
  {
     TDOvalue = ((TDOvalue << 16) + (TDOvalue >> 4)) & 0x000FFFFF;
  }

  return TDOvalue;
}


unsigned char _hil_4w_ezFET_SetReg_XBits08(unsigned char Data)
{
    return _hil_4w_ezFET_SetReg_XBits(Data,8);
}

unsigned short _hil_4w_ezFET_SetReg_XBits16(unsigned short Data)
{
    return _hil_4w_ezFET_SetReg_XBits(Data,16);
}

unsigned long _hil_4w_ezFET_SetReg_XBits20(unsigned long Data)
{
    return _hil_4w_ezFET_SetReg_XBits(Data,20);
}

// -----------------------------------------------------------------------------
unsigned long _hil_4w_ezFET_SetReg_XBits32(unsigned long Data)
{
    return _hil_4w_ezFET_SetReg_XBits(Data,32);
}

unsigned long long _hil_4w_ezFET_SetReg_XBits64(unsigned long long Data)
{
    return _hil_4w_ezFET_SetReg_XBits(Data,64);
}

// -----------------------------------------------------------------------------
void _hil_4w_ezFET_Tclk(unsigned char state)
{
    if(state)
    {
        TDIset1
    }
    else
    {
        TDIset0
    }
}

// -----------------------------------------------------------------------------
void _hil_4w_ezFET_StepPsa(unsigned long length)
{
    while(length > 0)
    {
        TCLKset0
        TCKset0
        TMSset1
        TCKset1 // select DR scan
        TCKset0
        TMSset0

        TCKset1 // capture DR
        TCKset0
        TCKset1 // shift DR
        TCKset0

        TMSset1
        TCKset1 // exit DR
        TCKset0

        // Set JTAG FSM back into Run-Test/Idle
        TCKset1
        TMSset0
        TCKset0
        TCKset1

        // Clock through the PSA
        TCLKset1

        length--;
    }
}

void _hil_4w_ezFET_StepPsaTclkHigh(unsigned long length)
{
    while(length--)
    {
        TCLKset1 _NOP();

        TCKset0 _NOP();
        TMSset1 _NOP();
        TCKset1 _NOP();// select DR scan
        TCKset0 _NOP();
        TMSset0 _NOP();

        TCKset1 _NOP();// capture DR
        TCKset0 _NOP();
        TCKset1 _NOP();// shift DR
        TCKset0 _NOP();

        TMSset1 _NOP();
        TCKset1 _NOP(); // exit DR
        TCKset0 _NOP();

        // Set JTAG FSM back into Run-Test/Idle
        TCKset1 _NOP();
        TMSset0 _NOP();
        TCKset0 _NOP();
        TCKset1 _NOP();

        _NOP(); _NOP(); _NOP(); _NOP(); _NOP();
        TCLKset0 _NOP();
    }
}

// -----------------------------------------------------------------------------
short _hil_4w_ezFET_BlowFuse(unsigned char targetHasTestVpp)
{
    return 0;
}
/* EOF */
