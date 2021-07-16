/*
 * hil_4w_432.c
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
//! \file hil_4w_432.c
//! \brief
//!

#include "hw_compiler_specific.h"
#include "HalGlobalVars.h"
#include "arch.h"
#include "edt.h"
#include <stdio.h>

static struct jtag _Jtag = {0};

// 4-wire JTAG: low level signal access

void TMSset1()
{ (*_Jtag.Out) |=  _Jtag.TMS;}

void TMSset0()
{ (*_Jtag.Out) &= ~_Jtag.TMS;}

void TDIset1()
{ (*_Jtag.Out) |=  _Jtag.TDI;}

void TDIset0()
{ (*_Jtag.Out) &= ~_Jtag.TDI;}

void TCKset1()
{ (*_Jtag.Out) |=  _Jtag.TCK;}

void TCKset0()
{ (*_Jtag.Out) &= ~_Jtag.TCK;}

void TDIset1TMSset1()
{ (*_Jtag.Out) |=  _Jtag.TDI | _Jtag.TMS; }

void TDIset0TMSset1()
{ (*_Jtag.Out) &= ~_Jtag.TDI; *_Jtag.Out |= _Jtag.TMS;}

// Array to store JTAG tap IR entry sequence for DMA
unsigned short JTAG_ENTRY_IR[4] = {0};

//Array to store JTAG tap DR entry sequence for DMA
unsigned short JTAG_ENTRY_DR[3] = {0};

//Array to store JTAG tap IR/DR shift exit sequence for DMA
unsigned short JTAG_IDLE[2] = {0};

//Array to strobe  TDI set 1 for DMA exectuion - TCK included
unsigned short TDI_1[2] = {0};

//Array to strobe  TDI set 0 for DMA exectuion - TCK included
unsigned short TDI_0[1] = {0};

//lookup table to reverse 8 bit values
static const unsigned char table_8bits[] =
{
    0x00, 0x80, 0x40, 0xc0, 0x20, 0xa0, 0x60, 0xe0,
    0x10, 0x90, 0x50, 0xd0, 0x30, 0xb0, 0x70, 0xf0,
    0x08, 0x88, 0x48, 0xc8, 0x28, 0xa8, 0x68, 0xe8,
    0x18, 0x98, 0x58, 0xd8, 0x38, 0xb8, 0x78, 0xf8,
    0x04, 0x84, 0x44, 0xc4, 0x24, 0xa4, 0x64, 0xe4,
    0x14, 0x94, 0x54, 0xd4, 0x34, 0xb4, 0x74, 0xf4,
    0x0c, 0x8c, 0x4c, 0xcc, 0x2c, 0xac, 0x6c, 0xec,
    0x1c, 0x9c, 0x5c, 0xdc, 0x3c, 0xbc, 0x7c, 0xfc,
    0x02, 0x82, 0x42, 0xc2, 0x22, 0xa2, 0x62, 0xe2,
    0x12, 0x92, 0x52, 0xd2, 0x32, 0xb2, 0x72, 0xf2,
    0x0a, 0x8a, 0x4a, 0xca, 0x2a, 0xaa, 0x6a, 0xea,
    0x1a, 0x9a, 0x5a, 0xda, 0x3a, 0xba, 0x7a, 0xfa,
    0x06, 0x86, 0x46, 0xc6, 0x26, 0xa6, 0x66, 0xe6,
    0x16, 0x96, 0x56, 0xd6, 0x36, 0xb6, 0x76, 0xf6,
    0x0e, 0x8e, 0x4e, 0xce, 0x2e, 0xae, 0x6e, 0xee,
    0x1e, 0x9e, 0x5e, 0xde, 0x3e, 0xbe, 0x7e, 0xfe,
    0x01, 0x81, 0x41, 0xc1, 0x21, 0xa1, 0x61, 0xe1,
    0x11, 0x91, 0x51, 0xd1, 0x31, 0xb1, 0x71, 0xf1,
    0x09, 0x89, 0x49, 0xc9, 0x29, 0xa9, 0x69, 0xe9,
    0x19, 0x99, 0x59, 0xd9, 0x39, 0xb9, 0x79, 0xf9,
    0x05, 0x85, 0x45, 0xc5, 0x25, 0xa5, 0x65, 0xe5,
    0x15, 0x95, 0x55, 0xd5, 0x35, 0xb5, 0x75, 0xf5,
    0x0d, 0x8d, 0x4d, 0xcd, 0x2d, 0xad, 0x6d, 0xed,
    0x1d, 0x9d, 0x5d, 0xdd, 0x3d, 0xbd, 0x7d, 0xfd,
    0x03, 0x83, 0x43, 0xc3, 0x23, 0xa3, 0x63, 0xe3,
    0x13, 0x93, 0x53, 0xd3, 0x33, 0xb3, 0x73, 0xf3,
    0x0b, 0x8b, 0x4b, 0xcb, 0x2b, 0xab, 0x6b, 0xeb,
    0x1b, 0x9b, 0x5b, 0xdb, 0x3b, 0xbb, 0x7b, 0xfb,
    0x07, 0x87, 0x47, 0xc7, 0x27, 0xa7, 0x67, 0xe7,
    0x17, 0x97, 0x57, 0xd7, 0x37, 0xb7, 0x77, 0xf7,
    0x0f, 0x8f, 0x4f, 0xcf, 0x2f, 0xaf, 0x6f, 0xef,
    0x1f, 0x9f, 0x5f, 0xdf, 0x3f, 0xbf, 0x7f, 0xff,
 };
#pragma inline=forced
unsigned char reverse_byte(unsigned char x)
{
    return table_8bits[x];
}

#pragma inline=forced
unsigned char ScanTDO()
{
    if((*_Jtag.In  & _Jtag.TDO) != 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void hil_4w_432_InitJtag(struct jtag tmp)
{
    _Jtag = tmp;

    // Load DR JTAG tap IR entry sequence for DMA into Array
    // The comment out code shows which signal is set low
    JTAG_ENTRY_IR[0] = (_Jtag.TMS );  //~_Jtag.TCK
    JTAG_ENTRY_IR[0] |= (_Jtag.TMS | _Jtag.TCK)<<8;
    JTAG_ENTRY_IR[1] = (_Jtag.TMS );  //~_Jtag.TCK
    JTAG_ENTRY_IR[1] |= (_Jtag.TMS | _Jtag.TCK)<<8;
    JTAG_ENTRY_IR[2] = 0; //~_Jtag.TMS | ~_Jtag.TCK
    JTAG_ENTRY_IR[2] |= (_Jtag.TCK)<<8;//~_Jtag.TMS
    JTAG_ENTRY_IR[3] = 0; //~_Jtag.TMS | ~_Jtag.TCK
    JTAG_ENTRY_IR[3] |= (_Jtag.TCK)<<8;//~_Jtag.TMS

    // Load DR JTAG tap DR entry sequence for DMA into Array
    // The comment out code shows which signal is set low
    JTAG_ENTRY_DR[0] = (_Jtag.TMS ); //~_Jtag.TCK
    JTAG_ENTRY_DR[0] |= (_Jtag.TMS | _Jtag.TCK)<<8;
    JTAG_ENTRY_DR[1] = 0;//~_Jtag.TMS | ~_Jtag.TCK
    JTAG_ENTRY_DR[1] |= (_Jtag.TCK)<<8;//~_Jtag.TMS
    JTAG_ENTRY_DR[2] = 0;//~_Jtag.TMS | ~_Jtag.TCK
    JTAG_ENTRY_DR[2] |= (_Jtag.TCK)<<8;//~_Jtag.TMS


    // Load JTAG tap IR/DR shift exit sequence for DMA into Array
    // The comment out code shows which signal is set low
    JTAG_IDLE[0] = (_Jtag.TMS ); //~_Jtag.TCK
    JTAG_IDLE[0] |= (_Jtag.TMS | _Jtag.TCK)<<8;
    JTAG_IDLE[1] = 0;//~_Jtag.TMS | ~_Jtag.TCK
    JTAG_IDLE[1] |= (_Jtag.TCK)<<8;//~_Jtag.TMS

    // Load TDI set 1 for DMA - TDI must be high before TCK goes HIGH
    TDI_1[0] = 0;
    TDI_1[0] |= _Jtag.TDI<<8;
    TDI_1[1] = ( _Jtag.TDI | _Jtag.TCK);
    TDI_1[1] |= ( _Jtag.TDI | _Jtag.TCK) << 8;

    // Load TDI set 0 for DMA
    TDI_0[0] = 0;
    TDI_0[0] |= (_Jtag.TCK) << 8;
}

//The hil_4w_432_Seq() function can apply a various sequence on TCK and TMS.
//It is used to apply the ARM JTAG entry sequence
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

// -----------------------------------------------------------------------------
short hil_4w_432_TapReset(void)
{
    // Reset TAP Controller State Machine
    // Set default state for JTAG signals (TDI = TMS = TCK = 1)
    TDIset1();
    TMSset1();
    TCKset1();
    // Clock TCK six (6) times
    TCKset0();
    TCKset1();
    TCKset0();
    TCKset1();
    TCKset0();
    TCKset1();
    TCKset0();
    TCKset1();
    TCKset0();
    TCKset1();
    TCKset0();
    TCKset1();
    // TAP Controller State Machine is now in "Test-Logic Reset" state
    // Clock TCK one more time with TMS = 0
    TMSset0();
    TCKset0();
    TCKset1();
    return 0;
}

/*
The hil_4w_432_Instr_4 function executes a 4 bit ARM IR shift
It uses the DMA to enter the tap “Shift IR” state. Also the  shift itself is done my using 2 DMA channels.
In the end the DMA is use to enter the tap “Run test idle” state
*/
unsigned char hil_4w_432_Instr_4(unsigned char Instruction)
{
    unsigned char TDOvalue = 0x00;
    volatile unsigned char count = 0;

    DMA1SZ = 8;
    DMA1SA = (unsigned short)JTAG_ENTRY_IR;

    // Fire DMA enter Shift-IR
    DMA1CTL |= DMAEN;
    DMA1CTL |= DMAREQ;

    //configre DMA to shit in the IR
    DMA1SZ = 3;
    DMA2SZ = 2;
    DMA1SA = (unsigned short)TDI_1;
    DMA2SA = (unsigned short)TDI_0;

    for(count = 0; count < 3; ++count)
    {
        if(Instruction & (1 << count))
        {
            DMA1CTL |= DMAEN;
            DMA1CTL |= DMAREQ;
        }
        else
        {
            DMA2CTL |= DMAEN;
            DMA2CTL |= DMAREQ;
        }
        TDOvalue |= (((unsigned char)ScanTDO()) << count);
    }

    if(Instruction & 0x8)
    {
        TDIset1();
    }
    else
    {
        TDIset0();
    }
    TMSset1();
    TCKset0();
    TCKset1();    // Exit1-IR

    TDOvalue |=  (((unsigned char)ScanTDO()) << count);

    // common exit to Run-Test/Idle
    DMA2SZ = 4;
    DMA2SA = (unsigned short)JTAG_IDLE;
    DMA2CTL |= DMAEN;
    DMA2CTL |= DMAREQ;
    return TDOvalue;
}

/*

The  hil_4w_432_SetReg_35Bits function  executes a 35 bit ARM DR shift
It uses the DMA to enter the tap “Shift DR” state. The fist 32Bit data is shifted in by using the SPI module.
Therefore the Bit order has to be reversed.  The last 3 bits are shifted in by bit banging.
In the end the DMA is use to enter the tap “Run test idle” state
*/
unsigned long long hil_4w_432_SetReg_35Bits( unsigned long long *Data)
{
    unsigned long  long TDOvalue = 0;
    unsigned long long  data_ = 0, retVal = 0;
    unsigned char  *outptr =(unsigned char*)&data_;
    unsigned char  *retValPtr = (unsigned char*)&retVal;
    unsigned char *inptr = (unsigned char*)&TDOvalue;
    unsigned char  *DataPtr = (unsigned char*)Data;

    // reverse first 32 Bits
    *(outptr+3) = reverse_byte(*(DataPtr+3));
    *(outptr+2) = reverse_byte(*(DataPtr+2));
    *(outptr+1) = reverse_byte(*(DataPtr+1));
    *(outptr+0) = reverse_byte(*(DataPtr+0));

    //configre DMA to shit in the DR
    DMA1SZ = 6;
    DMA1SA = (unsigned short)JTAG_ENTRY_DR;

    // Fire DMA enter Shift-DR
    DMA1CTL |= DMAEN;
    DMA1CTL |= DMAREQ;

    P5SEL |= 0x0E; // enablespi
    UCTL1 |= CHAR;

    U1TXBUF = *(outptr + 0);
    while(!(UTCTL1 & TXEPT));
    *(inptr + 0) = U1RXBUF;

    U1TXBUF = *(outptr + 1);
    while(!(UTCTL1 & TXEPT));
    *(inptr + 1) = U1RXBUF;

    U1TXBUF = *(outptr + 2);
    while(!(UTCTL1 & TXEPT));
    *(inptr + 2) = U1RXBUF;

    U1TXBUF = *(outptr + 3);
    while(!(UTCTL1 & TXEPT));
    *(inptr + 3) = U1RXBUF;

    P5SEL &= ~0x0E; // disable spi

    TCKset0();
    if(*(DataPtr+4) & 0x01)
    {
        TDIset1();
    }
    else
    {
        TDIset0();
    }
    TCKset1();
    *(retValPtr+4) |= ScanTDO();

    TCKset0();
    if(*(DataPtr+4) & 0x02)
    {
        TDIset1();
    }
    else
    {
        TDIset0();
    }
    TCKset1();
    *(retValPtr+4) |= ScanTDO() << 1;

    TCKset0();
    TMSset1();
    if(*(DataPtr+4) & 0x04)
    {
        TDIset1();
    }
    else
    {
        TDIset0();
    }
    TCKset1();
    *(retValPtr+4) |= ScanTDO()<< 2;

    // common exit to Run-Test/Idle
    DMA2SZ = 4;
    DMA2SA = (unsigned short)JTAG_IDLE;
    DMA2CTL |= DMAEN;
    DMA2CTL |= DMAREQ;

    // reverse  32 Bits if shifted out value
    *(retValPtr+0) = reverse_byte(*(inptr+0));
    *(retValPtr+1) = reverse_byte(*(inptr+1));
    *(retValPtr+2) = reverse_byte(*(inptr+2));
    *(retValPtr+3) = reverse_byte(*(inptr+3));
    return retVal;
}


unsigned char hil_4w_432_SetReg_XBits08(unsigned char Data)
{
    return 0;
}

unsigned short hil_4w_432_SetReg_XBits16(unsigned short Data)
{
    return 0;
}

/*
The  hil_4w_432_SetReg_32Bits function  executes a 32 bit ARM DR shift
It uses the DMA to enter the tap “Shift DR” state. The 32Bit data is shifted in by using the SPI module.
Therefore the Bit order has to be reversed.  In the end the DMA is use to enter the tap “Run test idle” state
*/
unsigned long hil_4w_432_SetReg_XBits32(unsigned long Data)
{
    unsigned long  TDOvalue = 0;
    unsigned long data_ = 0, retVal = 0;
    unsigned char  *outptr =(unsigned char*)&data_;
    unsigned char  *DataPtr = (unsigned char*)&Data;
    unsigned char  *retValPtr = (unsigned char*)&retVal;

    // reverse 32 Bit data
    *(outptr+0) = reverse_byte(*(DataPtr+3));
    *(outptr+1) = reverse_byte(*(DataPtr+2));
    *(outptr+2) = reverse_byte(*(DataPtr+1));
    *(outptr+3) = reverse_byte(*(DataPtr+0));

    unsigned char *inptr = (unsigned char*)&TDOvalue;

    //configre DMA to shit in the DR
    DMA1SZ = 6;
    DMA1SA = (unsigned short)JTAG_ENTRY_DR;

    DMA1CTL |= DMAEN;
    DMA1CTL |= DMAREQ;

    P5SEL |= 0x0E; // enablespi
    UCTL1 |= CHAR;
    U1TXBUF = *(outptr + 3);
    while(!(UTCTL1 & TXEPT));
    *(inptr + 3) = U1RXBUF;
    U1TXBUF = *(outptr + 2);
    while(!(UTCTL1 & TXEPT));
    *(inptr + 2) = U1RXBUF;
    U1TXBUF = *(outptr + 1);
    while(!(UTCTL1 & TXEPT));
    *(inptr + 1) = U1RXBUF;
    UCTL1 &= ~CHAR;
    U1TXBUF = *outptr;
    while(!(UTCTL1 & TXEPT));
    *inptr = U1RXBUF << 1;
    P5SEL &= ~0x0E; // disable spi

    TMSset1();
    TCKset0();
    if(Data & 0x1)
    {
        TDIset1();
    }
    else
    {
        TDIset0();
    }
    TCKset1();
    *inptr |= ScanTDO();

    // common exit to Run-Test/Idle
    DMA2SZ = 4;
    DMA2SA = (unsigned short)JTAG_IDLE;
    DMA2CTL |= DMAEN;
    DMA2CTL |= DMAREQ;
    // JTAG FSM = Run-Test/Idle

    *(retValPtr+3) = reverse_byte(*(inptr+0));
    *(retValPtr+2) = reverse_byte(*(inptr+1));
    *(retValPtr+1) = reverse_byte(*(inptr+2));
    *(retValPtr+0) = reverse_byte(*(inptr+3));
    return retVal;
}

unsigned long long hil_4w_432_SetReg_XBits64(unsigned long long Data)
{
    return 0;
}

/* EOF */
