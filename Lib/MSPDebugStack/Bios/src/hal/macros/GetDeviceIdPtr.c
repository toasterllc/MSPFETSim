/**
* \ingroup MODULMACROS
*
* \file GetDeviceIdPtr.c
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

#include "arch.h"
#include "edt.h"
#include "hal.h"
#include "stream.h"
#include "JTAG_defs.h"
#include "error_def.h"
#include "../include/hw_compiler_specific.h"
#include "JtagId.h"

/**
  GetJtagId: required as separate macro beside StartJtag to get the JtagId
             of an individual device in the JTAG chain
*/
HAL_FUNCTION(_hal_GetDeviceIdPtr)
{
    unsigned short JtagId = 0;
    unsigned short CoreIpId = 0;
    long DeviceIpPointer = 0;
    long IdDataAddr = 0;
    unsigned long lOut_long = 0;

    JtagId = cntrl_sig_capture();
    if (!jtagIdIsValid(JtagId))
    {
        return HALERR_UNDEFINED_ERROR;
    }
    else if (jtagIdIsXv2(JtagId))
    {
        // Get Core identification info
        core_ip_pointer();
        CoreIpId = SetReg_16Bits(0);
        // Get device identification pointer
        if(JtagId == JTAGVERSION95)
        {
            IHIL_Delay_1ms(1500);
        }
        device_ip_pointer();
        lOut_long = SetReg_20Bits(0);
        // The ID pointer is an un-scrambled 20bit value
        DeviceIpPointer = ((lOut_long & 0xFFFF) << 4 )  + (lOut_long >> 16 );
        if(DeviceIpPointer)
        {
            IdDataAddr = DeviceIpPointer + 4;
        }
    }
    else
    {
        IdDataAddr = 0xFF0;
    }

    STREAM_put_long(DeviceIpPointer);
    STREAM_put_long(IdDataAddr);
    STREAM_put_word(CoreIpId);
    return 0;
}
