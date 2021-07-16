/*
 * \ingroup MODULHIL
 *
 * \file hil432.c
 *
 * \brief Hardware Independent Layer Interface
 *
 *
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
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

#include "arch.h"
#include "hw_compiler_specific.h"
#include "edt.h"
#include "hilDelays.h"
#include "HalGlobalVars.h"
#include "stream.h"

unsigned short gprotocol_id;
unsigned short jtagReleased;


// function prototypes for map initialization
// common HIL configuration methods
short _hil_Init( void );
short _hil_SetProtocol(unsigned short protocol_id);
void  _hil_SetPsaSetup(unsigned short enhanced){return;}
void  _hil_SetPsaTCLK(unsigned short tclkValue){return;}
void  _hil_EntrySequences(unsigned char states){return;}
void _hil_ConfigureSetPc(unsigned short x){return;}
short _hil_Close( void );
short _hil_Open( unsigned char state);

void _hil_SetReset(unsigned char);
void _hil_SetTest(unsigned char);
void _hil_SetTMS(unsigned char);
void _hil_SetTCK(unsigned char);
void _hil_SetTDI(unsigned char);
void _hil_initTimerB0(void){return;}
void _hil_BSL_EntrySequence(unsigned short dummy){return;}
void _hil_SwitchVccFET(unsigned short SwitchVccFET){return;}

short hil_Write_Read_Ap_Jtag(unsigned long address ,unsigned long *data, unsigned short rnw);
short hil_Write_Read_Dp_Jtag(unsigned char address ,unsigned long *data, unsigned short rnw);
short hil_Write_Read_Mem_Ap_Jtag(unsigned short ap_sel, unsigned long address, unsigned long *data, unsigned short rnw);
unsigned long hil_Jtag_read_idcode();

unsigned char _hil_dummy_Instr_4(unsigned char Data){return 0;}
short _hil_dummy_Write_Read_Ap(unsigned long address ,unsigned long *data, unsigned short rnw){return -1;};
short _hil_dummy_Write_Read_Dp(unsigned char address ,unsigned long *data, unsigned short rnw){return -1;};
short _hil_dummy_Write_Read_Mem_Ap(unsigned short ap_sel, unsigned long address, unsigned long *data, unsigned short rnw){return -1;};
unsigned long _hil_dummy_read_idcoe(){return -1;}

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
};
// init by HIL init function
edt_distinct_methods_t _edt_Distinct_Methods;

static struct jtag _Jtag;

// protocol specific methods
// (must be implemeted in 4-wire JTAG )
extern short hil_4w_432_TapReset(void);
extern unsigned char hil_4w_432_SetReg_XBits08(unsigned char Data);
extern unsigned short hil_4w_432_SetReg_XBits16(unsigned short Data);
extern unsigned long hil_4w_432_SetReg_XBits32(unsigned long Data);
extern unsigned long long hil_4w_432_SetReg_35Bits( unsigned long long *Data);
extern unsigned char hil_4w_432_Instr_4(unsigned char Instruction);
extern unsigned long long hil_4w_432_SetReg_XBits64(unsigned long long Data);
extern void hil_4w_432_InitJtag(struct jtag tmp);
extern void hil_4w_432_Seq(unsigned short length, unsigned char *sequence);

extern void TMSset1();
extern void TMSset0();
extern void TDIset1();
extern void TDIset0();
extern void TCKset1();
extern void TCKset0();

// dummy function pointers
short hil_dummy_TapReset(void) {return 0;}
short hil_dummy_CheckJtagFuse(void){return 0;}
unsigned char hil_dummy_Instr(unsigned char Instruction){return 0;}
unsigned char hil_dummy_SetReg_XBits08(unsigned char Data){return 0;}
unsigned short hil_dummy_SetReg_XBits16(unsigned short Data){return 0;}
unsigned long hil_dummy_SetReg_XBits20(unsigned long Data){return 0;}
unsigned long hil_dummy_SetReg_XBits32(unsigned long Data){return 0;}
unsigned long long hil_dummy_SetReg_XBits64(unsigned long long Data){return 0;}
unsigned long long hil_dummy_SetReg_XBits8_64(unsigned long long Data, unsigned short loopCount, unsigned short JStateVersion){return 0;}
void hil_dummy_Tclk(unsigned char state){return;}
void hil_dummy_StepPsa(unsigned long length){return;}
short hil_dummy_BlowFuse(unsigned char targetHasTestVpp){return 0;}
unsigned char hil_dummy_GetPrevInstruction(){return 0;}
unsigned long long hil_dummy_SetReg_35Bits (unsigned long long *Data) {return 0;}
unsigned long long hil_dummy_SetReg_XBits (unsigned long long *Data, unsigned short count) {return 0;}

#pragma inline=forced
void RSTset1()
{
    { (*_Jtag.RST_PORT) |= _Jtag.RST; }
    _hil_Delay_1ms(5);
}
#pragma inline=forced
void RSTset0()
{
    { (*_Jtag.RST_PORT) &= ~_Jtag.RST; }
     _hil_Delay_1ms(5);
}

#pragma inline=forced
void TSTset0()
{
    { (*_Jtag.TSTCTRL_PORT) |= _Jtag._TST;}
    _hil_Delay_1ms(5);
}
#pragma inline=forced
void TSTset1()
{
    {(*_Jtag.TSTCTRL_PORT) &= ~_Jtag._TST; }
    _hil_Delay_1ms(5);
}
#pragma inline=forced
void TSTset0NoDelay()
{
    { (*_Jtag.TSTCTRL_PORT) |= _Jtag._TST;}
}
#pragma inline=forced
void TSTset1NoDelay()
{
    {(*_Jtag.TSTCTRL_PORT) &= ~_Jtag._TST; }
}

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
    DMA2SZ = 4;                       // JTAG test/idle sequence is always 4 transisions
    // set default debug protocol to JTAG
    gprotocol_id = JTAG_432;
    // initialize function pointers to distinct functions
    _hil_SetProtocol(gprotocol_id);
    jtagReleased = 1;

    return 0;
}

// -----------------------------------------------------------------------------
short _hil_SetProtocol(unsigned short protocol_id)
{
    short ret_value = 0;

    if(protocol_id == JTAG_432 || protocol_id == SWD_432)
    {
        gprotocol_id = protocol_id;
        _Jtag = _Jtag_Target;
        hil_4w_432_InitJtag(_Jtag);
        DMA1DA =  (unsigned int)_Jtag.Out;
        DMA2DA =  (unsigned int)_Jtag.Out;
    }
    else
    {
        ret_value = -1;
    }
    if (protocol_id == JTAG_432 || protocol_id == SWD_432)
    {
        // load dummies for 430
        _edt_Distinct_Methods.CheckJtagFuse = hil_dummy_CheckJtagFuse;
        _edt_Distinct_Methods.Instr = hil_dummy_Instr;
        _edt_Distinct_Methods.SetReg_XBits20 = hil_dummy_SetReg_XBits20;
        _edt_Distinct_Methods.GetPrevInstruction =  hil_dummy_GetPrevInstruction;
        _edt_Distinct_Methods.Tclk = hil_dummy_Tclk;
        _edt_Distinct_Methods.SetReg_XBits8_64 = hil_dummy_SetReg_XBits8_64;
        _edt_Distinct_Methods.StepPsa = hil_dummy_StepPsa;
        _edt_Distinct_Methods.BlowFuse = hil_dummy_BlowFuse;

        _edt_Distinct_Methods.TapReset = hil_4w_432_TapReset;
        _edt_Distinct_Methods.SetReg_XBits08 = hil_4w_432_SetReg_XBits08;
        _edt_Distinct_Methods.SetReg_XBits16 = hil_4w_432_SetReg_XBits16;
        _edt_Distinct_Methods.SetReg_XBits32 = hil_4w_432_SetReg_XBits32;
        _edt_Distinct_Methods.SetReg_XBits64 = hil_4w_432_SetReg_XBits64;
        _edt_Distinct_Methods.SetReg_XBits35 = hil_4w_432_SetReg_35Bits;
        _edt_Distinct_Methods.SetReg_XBits = hil_dummy_SetReg_XBits;
        _edt_Distinct_Methods.Instr04 = hil_4w_432_Instr_4;
        _edt_Distinct_Methods.write_read_Dp = hil_Write_Read_Dp_Jtag;
        _edt_Distinct_Methods.write_read_Ap = hil_Write_Read_Ap_Jtag;
        _edt_Distinct_Methods.write_read_mem_Ap = hil_Write_Read_Mem_Ap_Jtag;
        _edt_Distinct_Methods.GetJtagIdCode = hil_Jtag_read_idcode;
    }
    else
    {
        _edt_Distinct_Methods.TapReset  = hil_dummy_TapReset;
        _edt_Distinct_Methods.CheckJtagFuse = hil_dummy_CheckJtagFuse;
        _edt_Distinct_Methods.Instr = hil_dummy_Instr;
        _edt_Distinct_Methods.SetReg_XBits08 = hil_dummy_SetReg_XBits08;
        _edt_Distinct_Methods.SetReg_XBits16 = hil_dummy_SetReg_XBits16;
        _edt_Distinct_Methods.SetReg_XBits20 = hil_dummy_SetReg_XBits20;
        _edt_Distinct_Methods.SetReg_XBits32 = hil_dummy_SetReg_XBits32;
        _edt_Distinct_Methods.SetReg_XBits64 = hil_dummy_SetReg_XBits64;
        _edt_Distinct_Methods.SetReg_XBits8_64 = hil_dummy_SetReg_XBits8_64;
        _edt_Distinct_Methods.SetReg_XBits35 = hil_dummy_SetReg_35Bits;
        _edt_Distinct_Methods.SetReg_XBits = hil_dummy_SetReg_XBits;
        _edt_Distinct_Methods.Tclk = hil_dummy_Tclk;
        _edt_Distinct_Methods.GetPrevInstruction = hil_dummy_GetPrevInstruction;
        _edt_Distinct_Methods.StepPsa = hil_dummy_StepPsa;
        _edt_Distinct_Methods.BlowFuse = hil_dummy_BlowFuse;
        _edt_Distinct_Methods.Instr04 =             _hil_dummy_Instr_4;
        _edt_Distinct_Methods.write_read_Dp =       _hil_dummy_Write_Read_Dp;
        _edt_Distinct_Methods.write_read_Ap =       _hil_dummy_Write_Read_Ap;
        _edt_Distinct_Methods.write_read_mem_Ap =   _hil_dummy_Write_Read_Mem_Ap;
        _edt_Distinct_Methods.GetJtagIdCode =       _hil_dummy_read_idcoe;
    }
    return(ret_value);
}

static void _hil_Release(void)
{
    if(!jtagReleased)
    {
        if(gprotocol_id == JTAG_432 || gprotocol_id == SWD_432)
        {
            // hands off from target RST pin  -> will be pulled up
            (*_Jtag.DIRECTION) |= (_Jtag.TDO);
            (*_Jtag.TSTCTRL_PORT) |= (_Jtag._ENI2O);   // disable TDI2TDO reset target TEST pin low
            (*_Jtag.RST_PORT) |= (_Jtag._SELT);
            IHIL_Delay_1ms(1);
            (*_Jtag.TSTCTRL_PORT) |= (_Jtag._TST);// reset target TEST pin low
        }
        _edt_Distinct_Methods.TapReset  = hil_dummy_TapReset;
        _edt_Distinct_Methods.CheckJtagFuse = hil_dummy_CheckJtagFuse;
        _edt_Distinct_Methods.Instr = hil_dummy_Instr;
        _edt_Distinct_Methods.SetReg_XBits08 = hil_dummy_SetReg_XBits08;
        _edt_Distinct_Methods.SetReg_XBits16 = hil_dummy_SetReg_XBits16;
        _edt_Distinct_Methods.SetReg_XBits20 = hil_dummy_SetReg_XBits20;
        _edt_Distinct_Methods.SetReg_XBits32 = hil_dummy_SetReg_XBits32;
        _edt_Distinct_Methods.SetReg_XBits64 = hil_dummy_SetReg_XBits64;
        _edt_Distinct_Methods.SetReg_XBits8_64 = hil_dummy_SetReg_XBits8_64;
        _edt_Distinct_Methods.SetReg_XBits35 = hil_dummy_SetReg_35Bits;
        _edt_Distinct_Methods.SetReg_XBits = hil_dummy_SetReg_XBits;
        _edt_Distinct_Methods.Tclk = hil_dummy_Tclk;
        _edt_Distinct_Methods.GetPrevInstruction = hil_dummy_GetPrevInstruction;
        _edt_Distinct_Methods.StepPsa = hil_dummy_StepPsa;
        _edt_Distinct_Methods.BlowFuse = hil_dummy_BlowFuse;
        _edt_Distinct_Methods.Instr04 =             _hil_dummy_Instr_4;
        _edt_Distinct_Methods.write_read_Dp =       _hil_dummy_Write_Read_Dp;
        _edt_Distinct_Methods.write_read_Ap =       _hil_dummy_Write_Read_Ap;
        _edt_Distinct_Methods.write_read_mem_Ap =   _hil_dummy_Write_Read_Mem_Ap;
        _edt_Distinct_Methods.GetJtagIdCode =       _hil_dummy_read_idcoe;

        jtagReleased = 1;
    }
    IHIL_Delay_1ms(5);
}

#pragma inline=forced
void qDriveJTAG(void)
{
    (*_Jtag.Out) |= (_Jtag.TCK + _Jtag.TMS + _Jtag.TDI);
    (*_Jtag.DIRECTION) |= (_Jtag.TCK + _Jtag.TMS + _Jtag.TDI);
    (*_Jtag.DIRECTION) &= ~(_Jtag.TDO);

    (*_Jtag.TSTCTRL_PORT) |= (_Jtag._ENI2O + _Jtag._TST);//set test 1 and TDI TO TDO -> signals are low active
    (*_Jtag.RST_PORT) |= (_Jtag.RST);
    (*_Jtag.RST_PORT) &= ~(_Jtag._SELT); //set _SELT 1 > signal is low active -> active level shifters
}

// -----------------------------------------------------------------------------

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

unsigned long hil_Jtag_read_idcode()
{
    unsigned long JtagId = 0;

    JtagId = _edt_Distinct_Methods.Instr04(IR4_IDCODE);

    if (JtagId == 0x1)
    {
        JtagId = _edt_Distinct_Methods.SetReg_XBits32(0);
    }
    return JtagId;
}

static void _hil_Connect(unsigned char state)
{
    _hil_Release();
    if(jtagReleased)
    {
        _hil_SetProtocol(gprotocol_id);
    }
    if(gprotocol_id == JTAG_432 || gprotocol_id == SWD_432)
    {
        qDriveJTAG();
        JTAG_Entry();
        RSTset1();
        TSTset1();
    }
    jtagReleased = 0;
}

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
    if(value)
    {
        TMSset1();
    }
    else
    {
        TMSset0();
    }
}

// -----------------------------------------------------------------------------
void _hil_SetTDI(unsigned char value)
{
    if(value)
    {
        TDIset1();
    }
    else
    {
        TDIset0();
    }
}

// -----------------------------------------------------------------------------
void _hil_SetTCK(unsigned char value)
{
    if(value)
    {
        TCKset1();
    }
    else
    {
        TCKset0();
    }
}

// --------------------------------------------ARM defines----------------------
// JTAG Instruction Register definitions
#define ABORT        (0x8) // Used to force an AP abort
#define DPACC        (0xA) // DP IR dpacc
#define APACC        (0xB) // AP IR apacc
#define IDCODE       (0xE) // JTAG-DP TAP identification
#define BYPASS       (0xF) // Bypasses the device, by providing a direct path between DBGTDI and DBGTDO.

// DP_SELECT Register bits
#define DP_SELECT_APBANKSEL_MASK (0x000000F0)  // APBANKSEL Mask
#define DP_SELECT_APSEL_MASK     (0xFF000000)  // APBANKSEL Mask

// --------------------------------------------ARM defines end------------------

short hil_Write_Read_Dp_Jtag(unsigned char address ,unsigned long *data, unsigned short rnw)
{
    unsigned long long retVal;
    unsigned char i = 0;
    unsigned long long dataAlig = 0ull;
    dataAlig = ((unsigned long long)*data) << 3;

    dataAlig |= (((address & 0xC) >> 1) + (rnw & 0x1));

    if(_edt_Distinct_Methods.Instr04(DPACC) != 0x1)
    {
        return -1;
    }

    for (i = 0; i < MAX_RETRY; i++)
    {
        retVal = _edt_Distinct_Methods.SetReg_XBits35(&dataAlig);
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
short hil_Write_Read_Ap_Jtag(unsigned long address, unsigned long *data, unsigned short rnw)
{
    unsigned long long retVal;
    unsigned char i = 0;
    unsigned long long dataAlig = 0ull;

    dataAlig = ((unsigned long long)*data) << 3;

    unsigned long apsel_bank_sel = address & DP_SELECT_APSEL_MASK;
    apsel_bank_sel |= address & DP_SELECT_APBANKSEL_MASK;

    if (_edt_Distinct_Methods.write_read_Dp(DP_SELECT, &apsel_bank_sel, WRITE) != ACK)
    {
        return -1;
    }

    dataAlig |= (((address & 0xC) >> 1) + (rnw & 0x1));

    if(_edt_Distinct_Methods.Instr04(APACC) != 0x1)
    {
        return -1;
    }

    do
    {
        retVal = _edt_Distinct_Methods.SetReg_XBits35((unsigned long long*) &dataAlig);
    } while((retVal & 0x3 != ACK) && (++i < MAX_RETRY));

    if(i == MAX_RETRY)
    {
        return -1;
    }
    else
    {
        if(rnw == READ)
        {
            return _edt_Distinct_Methods.write_read_Dp(DP_RDBUFF, data, READ);
        }
        else
        {
            return ACK;
        }
    }
}

// -----------------------------------------------------------------------------
// Currently only supports single 32-bit transfers
short hil_Write_Read_Mem_Ap_Jtag(unsigned short ap_sel, unsigned long address, unsigned long *data, unsigned short rnw)
{
    // First Read current CSW value
    unsigned long tmp = 0;
    unsigned long apsel = (((unsigned long )ap_sel) << 24);
    if (_edt_Distinct_Methods.write_read_Ap(AP_CSW | apsel, &tmp, READ) != ACK)
    {
        return -1;
    }

    // Configure the transfer
    tmp |= AP_CSW_ADDRINC_OFF | AP_CSW_SIZE_32BIT;
    if (_edt_Distinct_Methods.write_read_Ap(AP_CSW | apsel, &tmp, WRITE) != ACK)
    {
        return -1;
    }

    // Write the address
    tmp = address;
    if (_edt_Distinct_Methods.write_read_Ap(AP_TAR | apsel, &tmp, WRITE) != ACK)
    {
        return -1;
    }
    // Write/Read data
    return _edt_Distinct_Methods.write_read_Ap(AP_DRW | apsel, data, rnw);
}

