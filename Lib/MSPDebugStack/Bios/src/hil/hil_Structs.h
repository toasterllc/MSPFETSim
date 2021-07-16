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

#ifndef _HIL_STRUCTS_H__
#define _HIL_STRUCTS_H__

/**
 @brief Structure of pointers to all exported EDT functions.
*/
struct edt_common_methods
{
    short (*Init)(void);
    short (*SetVcc)(unsigned short);
    void  (*SwitchVccFET)(unsigned short);
    short (*GetVcc)(double*, double*);
    short (*SetProtocol)(unsigned short);
    void  (*SetPsaTCLK)(unsigned short);
    short (*Open)(unsigned char state);
    short (*Close)(void);
    void  (*Delay_1us)(unsigned short);
    void  (*Delay_1ms)(unsigned short);
    short (*Loop)(unsigned short);
    void  (*EntrySequences)(unsigned char);
    void (*SetReset)(unsigned char);      // Set the Reset pin to the specified value
    void (*SetTest)(unsigned char);       // Set the Test pin to the specified value
    void (*SetJtagSpeed)(unsigned short, unsigned short);
    void (*ConfigureSetPc)(unsigned short);
    void (*initDelayTimer)(void);
    void (*BSL_EntrySequence)(unsigned short switchBypassOff);
    void (*SetTMS)(unsigned char);      // Set the TMS pin to the specified value
    void (*SetTCK)(unsigned char);      // Set the TCK pin to the specified value
    void (*SetTDI)(unsigned char);      // Set the TDI pin to the specified value
    short (*regulateVcc)(void);
    void (*setFpgaTimeOut)(unsigned short state);
    unsigned short (*getFpgaVersion)(void);
    void (*ReadADC12)(void);
    void (*ConfigFpgaIoMode)(unsigned short mode);
    void (*BSL_EntrySequence1xx_4xx)(void);
    void (*SetToolID)(unsigned short id);
};
typedef struct edt_common_methods edt_common_methods_t;

struct edt_distinct_methods
{
    short (*TapReset)(void);
    short (*CheckJtagFuse)(void);
    unsigned char (*Instr)(unsigned char);
    unsigned char (*Instr04)(unsigned char);
    unsigned char (*SetReg_XBits08)(unsigned char);
    unsigned short (*SetReg_XBits16)(unsigned short);
    unsigned long (*SetReg_XBits20)(unsigned long);
    unsigned long (*SetReg_XBits32)(unsigned long);
    unsigned long long (*SetReg_XBits35)(unsigned long long *Data);
    unsigned long long (*SetReg_XBits64)(unsigned long long);
    unsigned long long (*SetReg_XBits8_64)(unsigned long long, unsigned short, unsigned short);
    unsigned long long (*SetReg_XBits)(unsigned long long *Data, unsigned short count);
    void (*Tclk)(unsigned char);
    void (*StepPsa)(unsigned long);
    short (*BlowFuse)(unsigned char); // Blow the JTAG acces fuse
    unsigned char (*GetPrevInstruction)(void);
    short (*write_read_Dp)(unsigned char address, unsigned long *data, unsigned short rnw);
    short (*write_read_Ap)(unsigned long address, unsigned long *data, unsigned short rnw);
    short (*write_read_mem_Ap)(unsigned short ap_sel, unsigned long address, unsigned long *data, unsigned short rnw);
    unsigned long (*GetJtagIdCode)();
    unsigned char (*SwdTransferData)(unsigned char regiser, unsigned long* data, unsigned char rnw);
};
typedef struct edt_distinct_methods edt_distinct_methods_t;

#endif
