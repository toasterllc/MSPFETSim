/**
* \ingroup MODULMACROS
*
* \file StartJtagActivationCode.c
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

#include "error_def.h"
#include "arch.h"
#include "edt.h"
#include "hal.h"
#include "stream.h"

#ifdef DB_PRINT
#include "debug.h"
#endif

/**
  StartJtag
  Open and reset the physical connection to the target device JTAG interface.
  Implies release of the target from JTAG control.
  inData:  <protocol(8)>
  outData: <jtagId(8)>
  protocol: 0 = 4-wire Jtag
  jtagId: the Jtag identifier value
*/

HAL_FUNCTION(_hal_StartJtagActivationCode)
{
    unsigned char chainLen = 0;
    unsigned char protocol;
    unsigned long ActivationCode;
    short ret_value = HALERR_UNDEFINED_ERROR;
    unsigned char jtagMailboxIn = 0;

    if(STREAM_get_byte(&protocol) < 0)
    {
        ret_value = HALERR_START_JTAG_NO_PROTOCOL;
        return(ret_value);
    }
    STREAM_discard_bytes(1);

    if(STREAM_get_long(&ActivationCode) < 0)
    {
        ret_value = HALERR_START_JTAG_NO_ACTIVATION_CODE;
        return(ret_value);
    }
    // LO92 could just operrate in JTAG mode. Perhaps this must be changed for
    // feature devices
    protocol = 0;
    if(IHIL_SetProtocol(protocol))
    {
        ret_value = HALERR_START_JTAG_PROTOCOL_UNKNOWN;
        return(ret_value);
    }

#if defined(eZ_FET) || defined(MSP_FET)
    {
        HilInitGetEdtDistinctFunc hilEdtDis = (HilInitGetEdtDistinctFunc)0x1880;
        hilEdtDis(&_edt_Distinct_Methods);
    }
#endif

    // check if activation code is valid
    if(ActivationCode == L092_MODE || ActivationCode == C092_MODE)
    {
        // L092 has no JTAG chain support
        IHIL_SetReset(0);
        IHIL_Open(RSTLOW);
        IHIL_TapReset();
        IHIL_SetReset(0);
        chainLen = 1;

        jtagMailboxIn = i_WriteJmbIn32(ActivationCode>>16 , ActivationCode);
        // check for Timeout issue during mailbox request
        IHIL_SetReset(1);
        IHIL_Delay_1ms(3000);
        if(jtagMailboxIn == 1)
        {
            return (HALERR_JTAG_PASSWORD_WRONG);
        }
    }
    STREAM_put_byte((unsigned char)chainLen);
    if (chainLen > 0)
    {
        ret_value = 0;
    }
    return(ret_value);
}

