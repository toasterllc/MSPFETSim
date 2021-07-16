/**
* \ingroup MODULHAL
*
* \file edt.h
*
* \brief <FILEBRIEF>
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

#ifndef _EDT_H_
#define _EDT_H_
#include "JTAG_defs.h"
#include "hil_Structs.h"
#include "hal.h"

typedef void (*HilInitFunc)();
typedef void (*HilInitGetEdtCommenFunc)(edt_common_methods_t* edt_commen);
typedef void (*HilInitGetEdtDistinctFunc)(edt_distinct_methods_t* edt_distinct);

extern edt_common_methods_t  _edt_Common_Methods;
extern edt_distinct_methods_t _edt_Distinct_Methods;

extern unsigned short altRomAddressForCpuRead;
extern unsigned short wdtctlAddress5xx;
extern DevicePowerSettings devicePowerSettings;

// Hil common methods
#pragma inline=forced
short IHIL_Init(void)   { return _edt_Common_Methods.Init();}

#pragma inline=forced
short IHIL_SetVcc(unsigned short vcc){ return _edt_Common_Methods.SetVcc(vcc); }

#pragma inline=forced
void IHIL_SwitchVccFET(unsigned short switchVccFET){ _edt_Common_Methods.SwitchVccFET(switchVccFET);}

#pragma inline=forced
short IHIL_GetVcc(double* Vcc, double* ExtVcc){ return _edt_Common_Methods.GetVcc(Vcc, ExtVcc);}

#pragma inline=forced
short IHIL_SetProtocol(unsigned short protocol_id){ return _edt_Common_Methods.SetProtocol(protocol_id);}

#pragma inline=forced
void IHIL_SetPsaTCLK(unsigned short tclkValue){_edt_Common_Methods.SetPsaTCLK(tclkValue);}

#pragma inline=forced
short IHIL_Open(unsigned char state){ return _edt_Common_Methods.Open(state);}

#pragma inline=forced
short IHIL_Close(void){return _edt_Common_Methods.Close();}

#pragma inline=forced
void IHIL_Delay_1us(unsigned short usecs){_edt_Common_Methods.Delay_1us(usecs);}

#pragma inline=forced
void IHIL_Delay_1ms(unsigned short msecs){_edt_Common_Methods.Delay_1ms(msecs);}

#pragma inline=forced
short IHIL_IccMonitor_Process(unsigned short flags){ return _edt_Common_Methods.Loop(flags);}

#pragma inline=forced
void IHIL_EntrySequences(unsigned char states){_edt_Common_Methods.EntrySequences(states);}

#pragma inline=forced
void IHIL_BSL_EntrySequence(unsigned short switchBypassOff){_edt_Common_Methods.BSL_EntrySequence(switchBypassOff);}

#pragma inline=forced
void IHIL_BSL_EntrySequence1xx_4xx(){ _edt_Common_Methods.BSL_EntrySequence1xx_4xx();}

#pragma inline=forced
void IHIL_SetReset(unsigned char value){_edt_Common_Methods.SetReset(value);}

#pragma inline=forced
void IHIL_SetTest(unsigned char value){_edt_Common_Methods.SetTest(value);}

#pragma inline=forced
void IHIL_SetTMS(unsigned char value){_edt_Common_Methods.SetTMS(value);}

#pragma inline=forced
void IHIL_SetTCK(unsigned char value){_edt_Common_Methods.SetTCK(value);}

#pragma inline=forced
void IHIL_SetTDI(unsigned char value){_edt_Common_Methods.SetTDI(value);}

#pragma inline=forced
void IHIL_InitDelayTimer(void){_edt_Common_Methods.initDelayTimer();}

#pragma inline=forced
void IHIL_SetJtagSpeed(unsigned short jtagSpeed, unsigned short sbwSpeed){_edt_Common_Methods.SetJtagSpeed(jtagSpeed,sbwSpeed);}

#pragma inline=forced
void IHIL_ConfigureSetPc(unsigned short PCclockBeforeCapture){_edt_Common_Methods.ConfigureSetPc(PCclockBeforeCapture);}
// Hil distinct methods
#pragma inline=forced
short IHIL_TapReset(void){return _edt_Distinct_Methods.TapReset();}

#pragma inline=forced
short IHIL_CheckJtagFuse(void){return _edt_Distinct_Methods.CheckJtagFuse();}

#pragma inline=forced
void IHIL_Tclk(unsigned char tclk){_edt_Distinct_Methods.Tclk(tclk);}

#pragma inline=forced
char IHIL_Instr4(unsigned char ir){return _edt_Distinct_Methods.Instr04(ir);}

#pragma inline=forced
void IHIL_StepPsa(unsigned long length){_edt_Distinct_Methods.StepPsa(length);}

#pragma inline=forced
short IHIL_BlowFuse(unsigned char targetHasTestVpp){return _edt_Distinct_Methods.BlowFuse(targetHasTestVpp);}

#pragma inline=forced
unsigned char IHIL_GetPrevInstruction(){ return _edt_Distinct_Methods.GetPrevInstruction();}

#pragma inline=forced
void IHIL_TCLK(void){IHIL_Tclk(0); IHIL_Tclk(1);}

#pragma inline=forced
unsigned long long SetReg_XBits(unsigned long long *Data, unsigned short count){return _edt_Distinct_Methods.SetReg_XBits(Data,count);}

#pragma inline=forced
unsigned long long SetReg_35Bits(unsigned long long *Data){return _edt_Distinct_Methods.SetReg_XBits35(Data);}

#pragma inline=forced
unsigned char SetReg_8Bits(unsigned char data){return _edt_Distinct_Methods.SetReg_XBits08(data);}

#pragma inline=forced
unsigned short SetReg_16Bits(unsigned short data){return _edt_Distinct_Methods.SetReg_XBits16(data);}

#pragma inline=forced
unsigned long SetReg_20Bits(unsigned long data) {return _edt_Distinct_Methods.SetReg_XBits20(data);}

#pragma inline=forced
unsigned long SetReg_32Bits(unsigned long data){return _edt_Distinct_Methods.SetReg_XBits32(data);}

#pragma inline=forced
unsigned long long SetReg_64Bits(unsigned long long data){return _edt_Distinct_Methods.SetReg_XBits64(data);}

//ARM interface functions
#pragma inline=forced
short IHIL_Write_Read_Dp(unsigned char address, unsigned long *data, unsigned short rnw)
    {return _edt_Distinct_Methods.write_read_Dp(address,data, rnw);}
#pragma inline=forced
short IHIL_Write_Read_Ap(unsigned long address, unsigned long *data, unsigned short rnw)
    {return _edt_Distinct_Methods.write_read_Ap(address, data, rnw);}
#pragma inline=forced
short IHIL_Write_Read_Mem_Ap(unsigned short ap_sel, unsigned long address, unsigned long *data, unsigned short rnw)
{ return _edt_Distinct_Methods.write_read_mem_Ap(ap_sel, address, data, rnw);}
#pragma inline=forced
unsigned char IHIL_SwdTransferData(unsigned char regiser, unsigned long* data, unsigned char rnw)
{ return _edt_Distinct_Methods.SwdTransferData(regiser, data, rnw);}

#pragma inline=forced
unsigned long long SetReg8_64Bits(unsigned long long data, unsigned short loopCount, unsigned short PG)
    {return _edt_Distinct_Methods.SetReg_XBits8_64(data, loopCount, PG);}

// JTAG instruction register access
#pragma inline=forced
short cntrl_sig_low_byte()        { return _edt_Distinct_Methods.Instr(IR_CNTRL_SIG_LOW_BYTE); }

#pragma inline=forced
short cntrl_sig_capture()         { return _edt_Distinct_Methods.Instr(IR_CNTRL_SIG_CAPTURE); }

#pragma inline=forced
short cntrl_sig_high_byte()       { return _edt_Distinct_Methods.Instr(IR_CNTRL_SIG_HIGH_BYTE); }

#pragma inline=forced
short cntrl_sig_16bit()           { return _edt_Distinct_Methods.Instr(IR_CNTRL_SIG_16BIT); }

#pragma inline=forced
short cntrl_sig_release()         { return _edt_Distinct_Methods.Instr(IR_CNTRL_SIG_RELEASE); }

#pragma inline=forced
short addr_16bit()                { return _edt_Distinct_Methods.Instr(IR_ADDR_16BIT); }

#pragma inline=forced
short addr_capture()              { return _edt_Distinct_Methods.Instr(IR_ADDR_CAPTURE); }

#pragma inline=forced
short data_16bit()                { return _edt_Distinct_Methods.Instr(IR_DATA_16BIT); }

#pragma inline=forced
short data_capture()              {return  _edt_Distinct_Methods.Instr(IR_DATA_CAPTURE); }

#pragma inline=forced
short data_to_addr()              { return _edt_Distinct_Methods.Instr(IR_DATA_TO_ADDR); }

#pragma inline=forced
short data_quick()                { return _edt_Distinct_Methods.Instr(IR_DATA_QUICK); }

#pragma inline=forced
short config_fuses()              { return _edt_Distinct_Methods.Instr(IR_CONFIG_FUSES); }

#pragma inline=forced
short eem_data_exchange()         { return _edt_Distinct_Methods.Instr(IR_EMEX_DATA_EXCHANGE); }

#pragma inline=forced
short eem_data_exchange32()       { return _edt_Distinct_Methods.Instr(IR_EMEX_DATA_EXCHANGE32); }

#pragma inline=forced
short eem_read_control()          { return _edt_Distinct_Methods.Instr(IR_EMEX_READ_CONTROL); }

#pragma inline=forced
short eem_write_control()         { return _edt_Distinct_Methods.Instr(IR_EMEX_WRITE_CONTROL); }

#pragma inline=forced
short eem_read_trigger()          { return _edt_Distinct_Methods.Instr(IR_EMEX_READ_TRIGGER); }

#pragma inline=forced
short data_psa()                  { return _edt_Distinct_Methods.Instr(IR_DATA_PSA); }

#pragma inline=forced
short shift_out_psa()             { return _edt_Distinct_Methods.Instr(IR_SHIFT_OUT_PSA); }

#pragma inline=forced
short flash_16bit_update()        { return _edt_Distinct_Methods.Instr(IR_FLASH_16BIT_UPDATE); }

#pragma inline=forced
short jmb_exchange()              { return _edt_Distinct_Methods.Instr(IR_JMB_EXCHANGE); }

#pragma inline=forced
short device_ip_pointer()         { return _edt_Distinct_Methods.Instr(IR_DEVICE_ID);}

#pragma inline=forced
short core_ip_pointer()           { return _edt_Distinct_Methods.Instr(IR_COREIP_ID);}

#pragma inline=forced
short jstate_read()               { return _edt_Distinct_Methods.Instr(IR_JSTATE_ID);}

#pragma inline=forced
short test_reg()                  { return _edt_Distinct_Methods.Instr(IR_TEST_REG);}

#pragma inline=forced
short test_reg_3V()               { return _edt_Distinct_Methods.Instr(IR_TEST_3V_REG);}

#pragma inline=forced
short prepare_blow()               { return _edt_Distinct_Methods.Instr(IR_PREPARE_BLOW);}

#pragma inline=forced
short ex_blow()               { return _edt_Distinct_Methods.Instr(IR_EX_BLOW);}

#define OUT1RDY 0x0008
#define OUT0RDY 0x0004
#define IN1RDY  0x0002
#define IN0RDY  0x0001

#define JMB32B  0x0010
#define OUTREQ  0x0004
#define INREQ   0x0001

// JTAG logic functions
#pragma inline=forced
short isInstrLoad()
{
    cntrl_sig_capture();
    if((SetReg_16Bits(0) & (CNTRL_SIG_INSTRLOAD | CNTRL_SIG_READ)) != (CNTRL_SIG_INSTRLOAD | CNTRL_SIG_READ))
    {
        return -1;
    }
    return 0;
}

#pragma inline=forced
short instrLoad()
{
    unsigned short i = 0;

    cntrl_sig_low_byte();
    SetReg_8Bits(CNTRL_SIG_READ);
    IHIL_Tclk(1);

    for(i = 0; i < 10; i++)
    {
        if(isInstrLoad() == 0)
        {
           return 0;
        }
        IHIL_TCLK();
    }
    return -1;
}

#pragma inline=forced
void halt_cpu()
{
    data_16bit();
    SetReg_16Bits(0x3FFF);
    IHIL_Tclk(0);
    cntrl_sig_16bit();
    SetReg_16Bits(0x2409);
    IHIL_Tclk(1);
}

#pragma inline=forced
void release_cpu()
{
    IHIL_Tclk(0);
    cntrl_sig_16bit();
    SetReg_16Bits(0x2401);
    addr_capture();
    IHIL_Tclk(1);
}

#pragma inline=forced
unsigned short ReadMemWord(unsigned short address)
{
    unsigned short data = 0;
    halt_cpu();
    IHIL_Tclk(0);
    addr_16bit();
    SetReg_16Bits(address);
    data_to_addr();
    IHIL_Tclk(1);
    IHIL_Tclk(0);
    data = SetReg_16Bits(0);
    release_cpu();
    return data;
}

#pragma inline=forced
unsigned short ReadMemWordX(unsigned long address)
{
    unsigned short data = 0;
    halt_cpu();
    IHIL_Tclk(0);
    addr_16bit();
    SetReg_20Bits(address & 0xFFFFF);
    data_to_addr();
    IHIL_Tclk(1);
    IHIL_Tclk(0);
    data = SetReg_16Bits(0);
    release_cpu();
    return data;
}

#pragma inline=forced
unsigned short ReadMemWordXv2(unsigned long address)
{
    unsigned short data = 0;
    IHIL_Tclk(0);
    addr_16bit();
    SetReg_20Bits(address & 0xFFFFF);
    IHIL_Tclk(1);
    IHIL_Tclk(0);
    data_capture();
    data = SetReg_16Bits(0);
    IHIL_Tclk(1);
    IHIL_Tclk(0);
    IHIL_Tclk(1);
    return data;
}

#pragma inline=forced
void WriteMemWord(unsigned short address, unsigned short data)
{
    halt_cpu();
    IHIL_Tclk(0);
    cntrl_sig_low_byte();
    SetReg_8Bits(0x08);
    addr_16bit();
    SetReg_16Bits(address);
    data_to_addr();
    SetReg_16Bits(data);
    IHIL_Tclk(1);
    release_cpu();
}

#pragma inline=forced
void WriteMemWordX(unsigned long address, unsigned short data)
{
    halt_cpu();
    IHIL_Tclk(0);
    cntrl_sig_low_byte();
    SetReg_8Bits(0x08);
    addr_16bit();
    SetReg_20Bits(address & 0xFFFFF);
    data_to_addr();
    SetReg_16Bits(data);
    IHIL_Tclk(1);
    release_cpu();
}

#pragma inline=forced
void WriteMemWordXv2(unsigned long address, unsigned short data)
{
    IHIL_Tclk(0);
    cntrl_sig_16bit();
    SetReg_16Bits(0x0500);
    addr_16bit();
    SetReg_20Bits(address);
    IHIL_Tclk(1);
    data_to_addr();
    SetReg_16Bits(data);
    IHIL_Tclk(0);
    cntrl_sig_16bit();
    SetReg_16Bits(0x0501);
    IHIL_Tclk(1);
    IHIL_Tclk(0);
    IHIL_Tclk(1);
}

#pragma inline=forced
void WriteMemByte(unsigned long address, unsigned short data)
{
    halt_cpu();
    IHIL_Tclk(0);
    addr_16bit();
    SetReg_16Bits(address);
    data_to_addr();
    SetReg_8Bits(data);
    IHIL_Tclk(1);
    IHIL_Tclk(0);
    release_cpu();
}

#pragma inline=forced
unsigned short ReadCpuReg_uShort(unsigned short reg)
{
    short op = 0;
    unsigned short data = 0;

    cntrl_sig_16bit();
    SetReg_16Bits(0x3401);
    data_16bit();
    op = ((reg << 8) & 0x0F00) | 0x4082;
    SetReg_16Bits(op);
    IHIL_Tclk(0);
    data_capture();
    IHIL_Tclk(1);
    data_16bit();
    SetReg_16Bits(0x00fe);
    IHIL_Tclk(0);
    data_capture();
    IHIL_Tclk(1);
    IHIL_Tclk(0);
    IHIL_Tclk(1);
    SetReg_16Bits(0);
    data = SetReg_16Bits(0);
    IHIL_Tclk(0);
    cntrl_sig_16bit();
    SetReg_16Bits(0x2401);
    IHIL_Tclk(1);
    return data;
}

#pragma inline=forced
unsigned short ReadCpuReg(unsigned short reg)
{
    short op = 0;
    unsigned short data = 0;

    cntrl_sig_16bit();
    SetReg_16Bits(0x3401);
    data_16bit();
    op = ((reg << 8) & 0x0F00) | 0x4082;
    SetReg_16Bits(op);
    IHIL_Tclk(0);
    data_capture();
    IHIL_Tclk(1);
    data_16bit();
    SetReg_16Bits(0x00fe);
    IHIL_Tclk(0);
    data_capture();
    IHIL_Tclk(1);
    IHIL_Tclk(0);
    IHIL_Tclk(1);
    data = SetReg_16Bits(0);
    IHIL_Tclk(0);
    cntrl_sig_16bit();
    SetReg_16Bits(0x2401);
    IHIL_Tclk(1);
    return data;
}

#pragma inline=forced
unsigned long ReadCpuRegX(unsigned short reg)
{
    short op = 0;
    unsigned short Rx_l = 0;
    unsigned short Rx_h = 0;

    cntrl_sig_high_byte();
    SetReg_16Bits(0x34);
    op = ((reg << 8) & 0x0F00) | 0x60;
    data_16bit();
    IHIL_Tclk(1);
    SetReg_16Bits(op);
    IHIL_Tclk(0);
    data_capture();
    IHIL_Tclk(1);
    data_16bit();
    IHIL_Tclk(1);
    SetReg_16Bits(0x00fc);
    IHIL_Tclk(0);
    data_capture();
    IHIL_Tclk(1);
    Rx_l = SetReg_16Bits(0);
    IHIL_Tclk(0);
    IHIL_Tclk(1);
    Rx_h = SetReg_16Bits(0);
    IHIL_Tclk(0);
    cntrl_sig_high_byte();
    SetReg_8Bits(0x24);
    IHIL_Tclk(1);
    return ((unsigned long)Rx_h<<16) + Rx_l;
}

#pragma inline=forced
unsigned long ReadCpuRegXv2(unsigned short reg)
{
    unsigned short Rx_l = 0;
    unsigned short Rx_h = 0;
    unsigned short jtagId = cntrl_sig_capture();
    const unsigned short jmbAddr = (jtagId == 0x98) ? 0x14c : 0x18c;

    IHIL_Tclk(0);
    data_16bit();
    IHIL_Tclk(1);
    SetReg_16Bits(reg);
    cntrl_sig_16bit();
    SetReg_16Bits(0x1401);
    data_16bit();
    IHIL_TCLK();
    if (altRomAddressForCpuRead)
    {
        SetReg_16Bits(0x0ff6);
    }
    else
    {
        SetReg_16Bits(jmbAddr);
    }
    IHIL_TCLK();
    SetReg_16Bits(0x3ffd);
    IHIL_Tclk(0);
    if (altRomAddressForCpuRead)
    {
        cntrl_sig_16bit();
        SetReg_16Bits(0x0501);
    }
    data_capture();
    IHIL_Tclk(1);
    Rx_l = SetReg_16Bits(0);
    IHIL_TCLK();
    Rx_h = SetReg_16Bits(0);
    IHIL_TCLK();
    IHIL_TCLK();
    IHIL_TCLK();
    if (!altRomAddressForCpuRead)
    {
        cntrl_sig_16bit();
        SetReg_16Bits(0x0501);
    }
    IHIL_Tclk(0);
    data_capture();
    IHIL_Tclk(1);
    return((unsigned long)Rx_h<<16) + Rx_l;
}

#pragma inline=forced
void WriteCpuReg(unsigned short reg, unsigned short data)
{
    unsigned short op = 0;
    cntrl_sig_16bit();
    SetReg_16Bits(0x3401);
    data_16bit();
    op = (0x4030 | reg);
    SetReg_16Bits(op);
    IHIL_Tclk(0);
    data_capture();
    IHIL_Tclk(1);
    data_16bit();
    SetReg_16Bits(data);
    IHIL_Tclk(0);
    data_capture();
    IHIL_Tclk(1);
    data_16bit();
    SetReg_16Bits(0x3ffd);
    IHIL_Tclk(0);
    data_capture();
    IHIL_Tclk(1);
    IHIL_Tclk(0);
    cntrl_sig_16bit();
    SetReg_16Bits(0x2401);
    IHIL_Tclk(1);
}

#pragma inline=forced
void WriteCpuRegX(unsigned short mova, unsigned long data)
{
    unsigned short op = 0x0080 | mova | ((data >> 8) & 0x0F00);
    cntrl_sig_high_byte();
    SetReg_8Bits(0x34);
    data_16bit();
    IHIL_Tclk(1);
    SetReg_16Bits(op);
    IHIL_Tclk(0);
    data_capture();
    IHIL_Tclk(1);
    data_16bit();
    IHIL_Tclk(1);
    SetReg_16Bits(data & 0xFFFF);
    IHIL_Tclk(0);
    data_capture();
    IHIL_Tclk(1);
    data_16bit();
    IHIL_Tclk(1);
    SetReg_16Bits(0x3ffd);
    IHIL_Tclk(0);
    data_capture();
    IHIL_Tclk(1);
    IHIL_Tclk(0);
    cntrl_sig_high_byte();
    SetReg_8Bits(0x24);
    IHIL_Tclk(1);
}

// BUGFIX BTT1722 added
#pragma inline=forced
void WriteCpuRegXv2(unsigned short mova, unsigned short data)
{
    IHIL_Tclk(0);
    data_16bit();
    IHIL_Tclk(1);
    SetReg_16Bits(mova);
    cntrl_sig_16bit();
    SetReg_16Bits(0x1401);
    data_16bit();
    IHIL_TCLK();
    SetReg_16Bits(data);
    IHIL_TCLK();
    SetReg_16Bits(0x3ffd);
    IHIL_TCLK();
    IHIL_Tclk(0);
    addr_capture();
    SetReg_20Bits(0x00000);
    IHIL_Tclk(1);
    cntrl_sig_16bit();
    SetReg_16Bits(0x0501);
    IHIL_TCLK();
    IHIL_Tclk(0);
    data_capture();
    IHIL_Tclk(1);
}

#pragma inline=forced
void SetPc(unsigned short pc)
{
    cntrl_sig_16bit();
    SetReg_16Bits(0x3401);
    data_16bit();
    SetReg_16Bits(0x4030);
    IHIL_TCLK();
    SetReg_16Bits(pc);
    IHIL_TCLK();
    addr_capture();
    IHIL_Tclk(0);
    cntrl_sig_16bit();
    SetReg_16Bits(0x2401);
    IHIL_Tclk(1);
}

#pragma inline=forced
void SetPcJtagBug(unsigned short pc)
{
    data_16bit();
    SetReg_16Bits(0x4030);
    IHIL_Tclk(1);
    IHIL_Tclk(0);
    SetReg_16Bits(pc);
    IHIL_Tclk(1);
    IHIL_Tclk(0);
    IHIL_Tclk(1);
    IHIL_Tclk(0);
}

#pragma inline=forced
void SetPcX(unsigned long pc)
{
    unsigned short pc_high = (unsigned short)(0x80 | (((pc)>>8) & 0x0F00));
    unsigned short pc_low  = (unsigned short)((pc) & 0xFFFF);
    cntrl_sig_high_byte();
    SetReg_8Bits(0x34);
    data_16bit();
    SetReg_16Bits(pc_high);
    IHIL_TCLK();
    SetReg_16Bits(pc_low);
    IHIL_TCLK();
    addr_capture();
    IHIL_Tclk(0);
    cntrl_sig_high_byte();
    SetReg_8Bits(0x24);
    IHIL_Tclk(1);
}

#pragma inline=forced
void SetPcXv2(unsigned short Mova, unsigned short pc)
{
    cntrl_sig_capture();
    SetReg_16Bits(0x0000);
    IHIL_Tclk(0);
    data_16bit();
    IHIL_Tclk(1);
    SetReg_16Bits(Mova);
    IHIL_Tclk(0);
    cntrl_sig_16bit();
    SetReg_16Bits(0x1400);
    data_16bit();
    IHIL_Tclk(0);
    IHIL_Tclk(1);
    SetReg_16Bits(pc);
    IHIL_Tclk(0);
    IHIL_Tclk(1);
    SetReg_16Bits(0x4303);
    IHIL_Tclk(0);
    addr_capture();
    SetReg_20Bits(0x00000);
}

#pragma inline=forced
unsigned short SyncJtag()
{
    unsigned short lOut = 0, i = 50;
    cntrl_sig_16bit();
    SetReg_16Bits(0x2401);
    cntrl_sig_capture();
    do
    {
        lOut = SetReg_16Bits(0x0000);
        i--;
    }
    while(((lOut == 0xFFFF) || !(lOut & 0x0200)) && i);
    if(!i)
    {
        return 0;
    }
    return lOut;
}

#pragma inline=forced
void SyncJtagX()
{
    unsigned short lOut = 0, i = 50;
    cntrl_sig_16bit();
    SetReg_16Bits(0x2401);
    cntrl_sig_capture();
    SetReg_16Bits(0x0000);
    IHIL_Tclk(1);
    if (!(lOut & 0x0200))
    {
        cntrl_sig_high_byte();
        SetReg_8Bits(0x34);
        eem_data_exchange32();
        SetReg_32Bits(0x89);
        SetReg_32Bits(0);
        eem_data_exchange32();
        SetReg_32Bits(0x88);
        SetReg_32Bits(lOut|0x40);
        eem_data_exchange32();
        SetReg_32Bits(0x88);
        SetReg_32Bits(lOut);
    }
    cntrl_sig_capture();
    do
    {
        lOut = SetReg_16Bits(0x0000);
        i--;
    }
    while(((lOut == 0xFFFF) || !(lOut & 0x0200)) && i);
}

#pragma inline=forced
void SyncJtagXv2()
{
    unsigned short i = 50, lOut = 0 ;
    cntrl_sig_16bit();
    SetReg_16Bits(0x1501);
    cntrl_sig_capture();
    do
    {
        lOut = SetReg_16Bits(0x0000);
        i--;
    }
    while(((lOut == 0xFFFF) || !(lOut & 0x0200)) && i);
}

#pragma inline=forced
unsigned short wait_for_synch()
{
    unsigned short i = 0;
    cntrl_sig_capture();

    while(!(SetReg_16Bits(0) & 0x0200) && i++ < 50);
    if(i >= 50)
    {
        return 0;
    }
    return 1;
}

#pragma inline=forced
void RestoreTestRegs()
{
    if(devicePowerSettings.powerTestReg3VMask)
    {
        test_reg_3V();
        SetReg_16Bits(devicePowerSettings.powerTestReg3VDefault);
        IHIL_Delay_1ms(20);
    }
    if(devicePowerSettings.powerTestRegMask)
    {
        test_reg();
        SetReg_32Bits(devicePowerSettings.powerTestRegDefault);
        IHIL_Delay_1ms(20);
    }
}

#pragma inline=forced
void EnableLpmx5()
{
    if(devicePowerSettings.powerTestReg3VMask)
    {
        unsigned short reg_3V = 0;
        test_reg_3V();
        reg_3V= SetReg_16Bits(devicePowerSettings.powerTestReg3VDefault);

        SetReg_16Bits(reg_3V & ~devicePowerSettings.powerTestReg3VMask|
                       devicePowerSettings.enableLpmx5TestReg3V);

        IHIL_Delay_1ms(20);
    }

    if(devicePowerSettings.powerTestRegMask)
    {
        unsigned long reg_test = 0;
        test_reg();
        reg_test = SetReg_32Bits(devicePowerSettings.powerTestRegDefault);

        SetReg_32Bits(reg_test & ~devicePowerSettings.powerTestRegMask|
        devicePowerSettings.enableLpmx5TestReg);

        IHIL_Delay_1ms(20);
    }
}

#pragma inline=forced
void DisableLpmx5()
{
    if(devicePowerSettings.powerTestReg3VMask)
    {
        unsigned short reg_3V = 0;
        test_reg_3V();
        reg_3V = SetReg_16Bits(devicePowerSettings.powerTestReg3VDefault);

        SetReg_16Bits(reg_3V & ~devicePowerSettings.powerTestReg3VMask|
            devicePowerSettings.disableLpmx5TestReg3V);
        IHIL_Delay_1ms(20);
    }

    if(devicePowerSettings.powerTestRegMask)
    {
        unsigned long reg_test = 0;
        test_reg();
        SetReg_32Bits(devicePowerSettings.powerTestRegDefault);

        SetReg_32Bits(reg_test & ~devicePowerSettings.powerTestRegMask|
            devicePowerSettings.disableLpmx5TestReg);

        IHIL_Delay_1ms(20);
    }
}

unsigned long i_ReadJmbOut();
unsigned short i_ReadJmbOut16();

short i_WriteJmbIn(unsigned short data);
short i_WriteJmbIn32(unsigned short dataA, unsigned short dataB);

short checkWakeup();
short powerUpArm();


#pragma inline=forced
// -----------------------------------------------------------------------------
void JTAG_PsaSetup(unsigned long StartAddr)
{
    data_16bit();
    IHIL_Tclk(1);
    SetReg_16Bits(MOV_IMM_PC);
    IHIL_Tclk(0);
    IHIL_Tclk(1);
    SetReg_16Bits(StartAddr - 2);
    IHIL_Tclk(0);
    IHIL_Tclk(1);
    IHIL_Tclk(0);
    IHIL_Tclk(1);
    IHIL_Tclk(0);
    addr_capture();
    SetReg_16Bits(0x0000);
}

#pragma inline=forced
// -----------------------------------------------------------------------------
void JTAG_EnhancedPsaSetup(unsigned long StartAddr)
{
    halt_cpu();
    IHIL_Tclk(0);
    data_16bit();
    SetReg_16Bits(StartAddr - 2);
}

#pragma inline=forced
// -----------------------------------------------------------------------------
void JTAG_PsaEnd(void)
{
    // Intentionally does nothing
}

#pragma inline=forced
// -----------------------------------------------------------------------------
void JTAG_EnhancedPsaEnd(void)
{
    release_cpu();
    isInstrLoad();
}

#endif

