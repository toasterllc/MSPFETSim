/*
 * hal.c
 *
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
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
//!  \file hal.c
//!

#include "hw_compiler_specific.h"
#include "HalGlobalVars.h"
#include "edt.h"
#include "hal.h"
#include "hal_ref.h"
#include "stream.h"
#include "string.h"


// global variables to handle JTAG chain
unsigned short activeDevice;
REQUIRED(activeDevice)

unsigned char numOfDevices;
REQUIRED(numOfDevices)

unsigned short TCE;
REQUIRED(TCE)

DevicePowerSettings devicePowerSettings;
REQUIRED(devicePowerSettings)

DeviceSettings deviceSettings;
REQUIRED(deviceSettings)

// global variables to handle Emulation clock control
unsigned long _hal_mclkCntrl0;
REQUIRED(_hal_mclkCntrl0)

HalRec hal_functions_[HAL_FUNCTIONS_SIZE];

unsigned short setPCclockBeforeCapture = 0;
REQUIRED(setPCclockBeforeCapture)

unsigned short altRomAddressForCpuRead;
REQUIRED(altRomAddressForCpuRead)

unsigned short wdtctlAddress5xx = 0x15C;
REQUIRED(wdtctlAddress5xx)

unsigned short enhancedPsa = 0;
REQUIRED(enhancedPsa)

char traceBuffer[256];              ///< Trace buffer used to print out JTAG IR and DR shofts to debug port

// global variables use for JTAG/SWD DAP
unsigned long cswValues[4];
REQUIRED(cswValues)

ARMConfigSettings armConfigSettings;
REQUIRED(armConfigSettings)

#ifndef __data20
#define __data20
#endif

#define MACRO(x)  {ID_##x, (void __data20 *)_hal_##x },
HalRec hal_functions_default_[HAL_FUNCTIONS_DEFAULT_SIZE] =
{
    MACRO(Zero)
    MACRO_LIST
};
#undef MACRO

REQUIRED(_init_Hal)


void _init_Hal(void)
{
    unsigned char i;

    for(i=0; i < (sizeof(hal_functions_)/sizeof(HalRec)); i++)
    {
        if(i < (sizeof(hal_functions_default_)/sizeof(HalRec)))
        {
            hal_functions_[i].id = hal_functions_default_[i].id;
            hal_functions_[i].function = hal_functions_default_[i].function;
        }
        else
        {
            hal_functions_[i].id = 0xFFFF;
            hal_functions_[i].function = NULL;
        }
    }
}

HAL_FUNCTION(_hal_Zero)
{
    return 0;
}

HAL_FUNCTION(_hal_Init)
{
    //EDT_Init();
    return 0;
}

extern unsigned int LPMx5_DEVICE_STATE;
extern int intstate;
extern unsigned long long prevJState;
extern unsigned short lastTraceWritePos;


HAL_FUNCTION(_hal_ResetStaticGlobalVars)
{
    LPMx5_DEVICE_STATE = 1;
    intstate = 0;
    prevJState = 0x0000000000000000;
    lastTraceWritePos = 0;
    return 0;
}

HAL_FUNCTION(_hal_SetVcc)
{
    unsigned short vcc;
    STREAM_get_word(&vcc);
    IHIL_SetVcc(vcc);
    return 0;
}


HAL_FUNCTION(_hal_SwitchMosfet)
{
#ifdef  MSP_FET
    unsigned short on = 0;
    STREAM_get_word(&on);
    if (on)
    {
        IHIL_SwitchVccFET(1);
    }
    else
    {
        IHIL_SwitchVccFET(0);
    }
    return 0;
#else
    return -1;
#endif
}


HAL_FUNCTION(_hal_GetVcc)
{
    double vcc = 0;
    double ext_vcc = 0;

    IHIL_GetVcc(&vcc, &ext_vcc);
    STREAM_put_word((unsigned short)vcc);
    STREAM_put_word((unsigned short)ext_vcc);
    return 0;
}

HAL_FUNCTION(_hal_GetFuses)
{
    config_fuses();
    STREAM_put_byte((unsigned char)SetReg_8Bits(0));
    return 0;
}

HAL_FUNCTION(_hal_DummyMacro)
{
    return -1;
}
/* EOF */
