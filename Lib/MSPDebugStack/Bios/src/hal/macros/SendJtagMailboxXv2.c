/**
* \ingroup MODULMACROSXV2
*
* \file SendJtagMailboxXv2.c
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
#include "error_def.h"
#include "JtagId.h"
#ifdef DB_PRINT
#include "debug.h"
#endif

/**
  WriteMemBytesXv2
  Write bytes to a memory mapped location.
  inData:  <addr(32)> <length(32)> <data(8)>{*}
  outData: -
  addr: the address to start writing to
  length: number of bytes to write
  data: data to write to the given location
*/
HAL_FUNCTION(_hal_SendJtagMailboxXv2)
{
    static unsigned short  MailBoxMode;
    static unsigned short  DATA1 = 0x0, DATA2 = 0x0;
    unsigned char jtagMailboxIn = 0;

    if(STREAM_get_word(&MailBoxMode) != 0)
    {
        return(HALERR_UNDEFINED_ERROR);
    }
    // get length ot be flashed
    if(STREAM_get_word(&DATA1) != 0)
    {
        return(HALERR_UNDEFINED_ERROR);
    }
    if(STREAM_get_word(&DATA2) == -1)
    {
        return(HALERR_UNDEFINED_ERROR);
    }
    IHIL_EntrySequences(RSTLOW);
    IHIL_TapReset();

    if(MailBoxMode == 0x11)// 32Bit Mode
    {
        jtagMailboxIn = i_WriteJmbIn32(DATA1,DATA2);
    }
    else // 16 Bit Mode
    {
        jtagMailboxIn = i_WriteJmbIn(DATA1);
    }
    // check if mailbox input was ok
    if(jtagMailboxIn)
    {
        return(HALERR_UNDEFINED_ERROR);
    }
    IHIL_SetReset(1);
    IHIL_Delay_1ms(40);

    IHIL_EntrySequences(RSTHIGH);
    IHIL_Delay_1ms(200);
    IHIL_TapReset();


    if (jtagIdIsXv2(cntrl_sig_capture()))
    {
        return 0;
    }
    return HALERR_UNDEFINED_ERROR;
}
