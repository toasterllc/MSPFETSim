/**
* \file PollStateReg430I.c
*
* \brief Write words (16bit values) to a memory mapped location
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

//! \ingroup MODULMACROS
//! \file PollJStateReg430I.c
//! \brief

#include "arch.h"
#include "edt.h"
#include "hal.h"
#include "stream.h"

/**
  PollJStateReg430I
  Queries the JSTATE register and reports any changes in the relevant bits
  inData:  <forceSendState(16)>
  outData: <captureFlag(16)> <JStateLow(32)> <JStateHigh(32)>
*/

HAL_FUNCTION(_hal_PollJStateReg430I)
{
    short RetState = -1;
    unsigned short forceSendState = 0;
    unsigned short lOut = 0;

    STREAM_get_word(&forceSendState);

    if(forceSendState)
    {
        lOut = cntrl_sig_capture();

        //To check if the device woke up in the meantime
        //and prevent an unwanted reset/BSL entry sequence
        if (lOut != JTAGVERSION)
        {
            IHIL_Open(RSTHIGH);
            IHIL_TapReset();
            IHIL_CheckJtagFuse();
            lOut = cntrl_sig_capture();
        }

        if(lOut == JTAGVERSION)
        {   // Active
            STREAM_put_word(JSTATE_CAPTURE_FLAG);
            STREAM_put_long(0x00000000);
            STREAM_put_long(0x00000000);
        }
        else
        {   // LPMx.5
            STREAM_put_word(JSTATE_CAPTURE_FLAG);
            STREAM_put_long(0x00000000);
            STREAM_put_long(0xC0000000);
        }
        RetState = 0;
    }
    return RetState;
}
