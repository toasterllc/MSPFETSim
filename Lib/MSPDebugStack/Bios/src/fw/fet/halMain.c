/*
 * halMain.c
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

//! \ingroup MODULHAL
//! \file halMain.c
//! \brief
//!
//! <b>files</b>\n
//! \li startup.s43
//! \li halMain.c
//! \li hal.c
//! \li halGlobalVars.c

#include "hw_compiler_specific.h"
#include "stream.h"
#include "hal.h"
#include "edt.h"
#include "stdlib.h"
#include "string.h"
#include "HalGlobalVars.h"
#include "../fw/fet/FetVersion.h"
#include "arch.h"

#define HAL_SIGNATURE 0xBEEFBEEFul

extern void globalVarsInit(void);

REQUIRED(_edt_Common_Methods)
edt_common_methods_t  _edt_Common_Methods;

REQUIRED(_edt_Distinct_Methods)
edt_distinct_methods_t _edt_Distinct_Methods;

extern short _hil_Init( void );
extern unsigned long _hal_mclkCntrl0;

// prototypes for core/HAL interface
void *ResetFirmware(void *stream_adr, unsigned long device_flags, unsigned char v3opHilCrcOk, unsigned char v3opDcdcCcOk);

CONST_AT(HAL_INFOS hal_infos_, 0x1904) =
{
  ResetFirmware,                // _initTask
  (VERSION_MAJOR - 1) << 14 |
  (VERSION_MINOR << 8) |
  VERSION_PATCH,
  VERSION_BUILD
};
REQUIRED(hal_infos_)

const unsigned long hal_Signature_ @ "HALSIGNATURE" = HAL_SIGNATURE;
#pragma required = hal_Signature_

//! \brief Pointer to HAL IRQ vector table
RO_PLACEMENT_NO_INIT volatile const unsigned short hil_Start_UP_ @ "HILINIT";
RO_PLACEMENT_NO_INIT volatile const unsigned long  hil_signature_ @ "HILSIGNATURE";
RO_PLACEMENT_NO_INIT volatile const unsigned short  hil_version_ @ "HILVERSION";
RO_PLACEMENT_NO_INIT volatile const unsigned short  hil_versionCmp_ @ "HILVERSIONCMP";


short _dummy_Init( void ){return 0;};
short _dummy_SetVcc(unsigned short Vcc){return 0;};
void _dummy_SwitchVccFET(unsigned short state) {return;};
short _dummy_GetVcc(double* Vcc, double* ExtVcc){return 0;};

short _dummy_SetProtocol(unsigned short protocol_id){return 0;};
void  _dummy_SetPsaSetup(unsigned short enhanced){return;};
void  _dummy_SetPsaTCLK(unsigned short tclkValue){return;};

short _dummy_Open( unsigned char state ){return 0;};
short _dummy_Close( void ){return 0;};
short _dummy_IccMonitor_Process(unsigned short flags){return 0;}; // flags: to be compatible with HIL calls
void  _dummy_EntrySequences(unsigned char states){return;};

void _dummy_SetReset(unsigned char value){return;};
void _dummy_SetTest(unsigned char value){return;};
void _dummy_SetTMS(unsigned char value){return;};
void _dummy_SetTDI(unsigned char value){return;};
void _dummy_SetTCK(unsigned char value){return;};

void _dummy_SetJtagSpeed(unsigned short jtagSpeed, unsigned short sbwSpeed){return;};
void _dummy_ConfigureSetPc (unsigned short PCclockBeforeCapture){return;};

void _dummy_initDelayTimer(void) {return;};
short _dummy_regulateVcc(void) {return 0;};
unsigned short _dummy_getFpgaVersion(void) {return 0;};

void _dummy(unsigned short dummy){return;};

short _dummy_TapReset_Dma(void){return 0;}
short _dummy_CheckJtagFuse_Dma(void){return 0;}
unsigned char _dummy_Instr_Dma(unsigned char Instruction){return 0;}
unsigned char _dummy_SetReg_XBits08_Dma(unsigned char Data){return 0;}
unsigned short _dummy_SetReg_XBits16_Dma(unsigned short Data){return 0;}
unsigned long _dummy_SetReg_XBits20_Dma(unsigned long Data){return 0;}
unsigned long _dummy_SetReg_XBits32_Dma(unsigned long Data){return 0;}
unsigned long long _dummy_SetReg_XBits64_Dma(unsigned long long Data){return 0;}
unsigned long long _dummy_SetReg_XBits8_64_Dma(unsigned long long Data, unsigned short loopCount, unsigned short Pg){return 0;}
void  _dummy_Tclk_Dma(unsigned char state){return;}
void  _dummy_StepPsa_Dma(unsigned long length){return;}
void  _dummy_StepPsaTclkHigh_Dma(unsigned long length){return;}
short _dummy_BlowFuse_Dma(unsigned char targetHasTestVpp){return 0;}
void  _dummy_ConfigureSpeed_Dma(unsigned short speed){return;}
void  _dummy_initTimerB0(void){return;}
void  _dummy_BSL_EntrySequence(unsigned short switchBypassOff){return;}
unsigned char _dummy_GetPrevInstruction (void){return 0;}
void  _dummy_ReadADC12(void){return;}
void _dummy_ConfigFpgaIoMode(unsigned short mode){return;}

unsigned long long _dummy_SetReg_XBits(unsigned long long *data, unsigned short count){return 0;}
unsigned char _dummy_Instr_4(unsigned char Data){return 0;}

short _dummy_write_read_Dp(unsigned char address, unsigned long *data, unsigned short rnw) {return -1;}
short _dummy_write_read_Ap(unsigned long address, unsigned long *data, unsigned short rnw) {return -1;}
short _dummy_write_read_mem_Ap(unsigned short ap_sel, unsigned long address, unsigned long *data, unsigned short rnw) {return -1;}

void _dummy_setFpgaTimeOut(unsigned short state) {return;};

struct stream_funcs *_stream_Funcs;

HAL_INFOS hal_infos_in_ram_;

// called via cstartup form BIOS_HalInterfaceInit
REQUIRED(ResetFirmware)
void *ResetFirmware(void *stream_adr, unsigned long device_flags, unsigned char v3opHilCrcOk, unsigned char v3opDcdcCcOk)
{
    globalVarsInit();
    memcpy((HAL_INFOS*)&hal_infos_in_ram_,&hal_infos_, sizeof(hal_infos_));
    // save address of stream funcs, located in core
     _stream_Funcs=(struct stream_funcs *)stream_adr;
    _init_Hal();
    hal_infos_in_ram_.swCmp_0 = (VERSION_MAJOR_CMP - 1) << 14 | (VERSION_MINOR_CMP << 8) | VERSION_PATCH_CMP;
    hal_infos_in_ram_.swCmp_1 = VERSION_BUILD_CMP;

    {
       // check if we have a valid hil layer programmed into our tool
        if(hil_signature_ == 0xF00DF00D && hil_Start_UP_ != 0xFFFF && v3opHilCrcOk)
        {
            // INIT HIL layer
            HilInitGetEdtCommenFunc hilEdtCom = NULL;
            // set pointer to edt commen functions
            hilEdtCom = (HilInitGetEdtCommenFunc)0x18A0;
            hilEdtCom(&_edt_Common_Methods);
            hal_infos_in_ram_.hil_version = hil_version_;
            hal_infos_in_ram_.hil_versionCmp = hil_versionCmp_;
#ifdef eZ_FET
            if(v3opDcdcCcOk)
            {
                _edt_Common_Methods.SwitchVccFET(LDO_ON);
            }
#endif


#ifdef MSP_FET
            // Force to not update for Beta1
            hal_infos_in_ram_.fpga_version = _edt_Common_Methods.getFpgaVersion();
            // power up VCC as FET power supply
            if(v3opDcdcCcOk)
            {
                _edt_Common_Methods.SetVcc(3300);
                _edt_Common_Methods.SwitchVccFET(1);
            }
#else
            hal_infos_in_ram_.fpga_version = 0;
#endif
        }
        else
        {
            // if hil layer is not valid or no startup address was found
            hal_infos_in_ram_.hil_version = 0;
            hal_infos_in_ram_.fpga_version = 0;

            _edt_Common_Methods.Init =              _dummy_Init;
            _edt_Common_Methods.SetVcc =            _dummy_SetVcc;
            _edt_Common_Methods.GetVcc =            _dummy_GetVcc;
            _edt_Common_Methods.SetProtocol =       _dummy_SetProtocol;
            _edt_Common_Methods.SetPsaTCLK =        _dummy_SetPsaTCLK;
            _edt_Common_Methods.Open =              _dummy_Open;
            _edt_Common_Methods.Close =             _dummy_Close;
            _edt_Common_Methods.Delay_1us =         _dummy;
            _edt_Common_Methods.Delay_1ms =         _dummy;
            _edt_Common_Methods.Loop =              _dummy_IccMonitor_Process;
            _edt_Common_Methods.EntrySequences =    _dummy_EntrySequences;
            _edt_Common_Methods.SetReset =          _dummy_SetReset;
            _edt_Common_Methods.SetTest =           _dummy_SetTest;
            _edt_Common_Methods.SetTMS =            _dummy_SetTMS;
            _edt_Common_Methods.SetTDI =            _dummy_SetTDI;
            _edt_Common_Methods.SetTCK =            _dummy_SetTCK;
            _edt_Common_Methods.SetJtagSpeed =      _dummy_SetJtagSpeed;
            _edt_Common_Methods.ConfigureSetPc =    _dummy_ConfigureSetPc;
            _edt_Common_Methods.initDelayTimer =    _dummy_initDelayTimer;
            _edt_Common_Methods.regulateVcc =       _dummy_regulateVcc;
            _edt_Common_Methods.getFpgaVersion =    _dummy_getFpgaVersion;
            _edt_Common_Methods.BSL_EntrySequence = _dummy_BSL_EntrySequence;
            _edt_Common_Methods.SwitchVccFET      = _dummy_SwitchVccFET;
            _edt_Common_Methods.setFpgaTimeOut    = _dummy_setFpgaTimeOut;
            _edt_Common_Methods.regulateVcc       = _dummy_regulateVcc;
            _edt_Common_Methods.ReadADC12         = _dummy_ReadADC12;
            _edt_Common_Methods.ConfigFpgaIoMode  = _dummy_ConfigFpgaIoMode;

            _edt_Distinct_Methods.TapReset =            _dummy_TapReset_Dma;
            _edt_Distinct_Methods.CheckJtagFuse =       _dummy_CheckJtagFuse_Dma;
            _edt_Distinct_Methods.Instr =               _dummy_Instr_Dma;
            _edt_Distinct_Methods.SetReg_XBits08 =      _dummy_SetReg_XBits08_Dma;
            _edt_Distinct_Methods.SetReg_XBits16 =      _dummy_SetReg_XBits16_Dma;
            _edt_Distinct_Methods.SetReg_XBits20 =      _dummy_SetReg_XBits20_Dma;
            _edt_Distinct_Methods.SetReg_XBits32 =      _dummy_SetReg_XBits32_Dma;
            _edt_Distinct_Methods.SetReg_XBits64 =      _dummy_SetReg_XBits64_Dma;
            _edt_Distinct_Methods.SetReg_XBits8_64 =    _dummy_SetReg_XBits8_64_Dma;
            _edt_Distinct_Methods.Tclk =                _dummy_Tclk_Dma;
            _edt_Distinct_Methods.StepPsa =             _dummy_StepPsaTclkHigh_Dma;
            _edt_Distinct_Methods.BlowFuse =            _dummy_BlowFuse_Dma;
            _edt_Distinct_Methods.GetPrevInstruction  = _dummy_GetPrevInstruction;
            _edt_Distinct_Methods.SetReg_XBits =        _dummy_SetReg_XBits;
            _edt_Distinct_Methods.Instr04 =             _dummy_Instr_4;
            _edt_Distinct_Methods.write_read_Dp =       _dummy_write_read_Dp;
            _edt_Distinct_Methods.write_read_Ap =       _dummy_write_read_Ap;
            _edt_Distinct_Methods.write_read_mem_Ap =   _dummy_write_read_mem_Ap;
        }
    }

    _hal_mclkCntrl0=0x040f;
    hal_infos_in_ram_.hal_size = sizeof(hal_functions_)/sizeof(HalRec);
    hal_infos_in_ram_.hal_list_ptr = hal_functions_;
    return((void*)&hal_infos_in_ram_); // return software infos
}

unsigned short hal_GetHilVersion()
{
    return 0;
}
