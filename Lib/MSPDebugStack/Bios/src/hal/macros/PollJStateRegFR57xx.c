/**
* \ingroup MODULMACROS
*
* \file PollJStateRegFR57xx.c
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
#include "hal_ref.h"
#include "stream.h"
#include "error_def.h"
#include "global_variables.h"
#include "JtagId.h"

#define ACTIVE              1
#define LPM5_MODE           2

int intstate = 0;

unsigned char getSystemState()
{
    volatile unsigned char TDO_Value = 0;
    unsigned char state = ACTIVE;

    IHIL_Delay_1ms(5);
    if (jtagIdIsXv2(cntrl_sig_capture()))
    {
        intstate = 0; // Active mode
    }
    else
    {
        intstate++;
        state = LPM5_MODE;
    }

    if (intstate > 1) // device is in LPMx. now
    {
        intstate = 0;
        IHIL_EntrySequences(RSTHIGH);
        IHIL_TapReset();

        TDO_Value = cntrl_sig_capture();
        if ( jtagIdIsXv2(TDO_Value) && (jmb_exchange() == TDO_Value)) // if device wakeup in between check content of jtag mailbox
        {
            state = ACTIVE;
            // check if JTAG mailbox is ready & perform input request
            if(SetReg_16Bits(0x0004) == 0x1207) // OUTRDY  | INRDY JMB_OUTREQ
            {
                if (SetReg_16Bits(0x0000) == 0xA55A)
                {
                    StreamSafe stream_tmp;

                    //Setup values for watchdog control regsiters
                    unsigned char Dummy[8] = {wdtctlAddress5xx & 0xFF,(wdtctlAddress5xx >> 8) & 0xFF,
                                            WDTHOLD_DEF|WDTSSEL_ACLK,WDTPW_DEF,0,0,0,0};

                    STREAM_internal_stream(Dummy, sizeof(Dummy), 0, 0, &stream_tmp);
                    HAL_SyncJtag_Conditional_SaveContextXv2(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG);
                    STREAM_external_stream(&stream_tmp);
                }
                else
                {
                    TDO_Value = 0xFF;
                    state = LPM5_MODE;
                }
            }
        }
    }
    return state;
 }

unsigned int LPMx5_DEVICE_STATE  = ACTIVE;       // Assume device starts in Wake-up mode
/**
  PollJStateReg
  Queries the JSTATE register and reports any changes in the relevant bits
  inData:  <maskLow(32)> <maskHigh(32)> <forceSendState(16)>
  outData: <captureFlag(16)> <JStateLow(32)> <JStateHigh(32)>
*/
HAL_FUNCTION(_hal_PollJStateRegFR57xx)
{
    short RetState = HALERR_UNDEFINED_ERROR;

    volatile unsigned char state = LPM5_MODE;

    unsigned int oldLPMx5_DEVICE_STATE = LPMx5_DEVICE_STATE;   // Initialize to not transition

    unsigned short forceSendState = 0;
    STREAM_get_word(&forceSendState);
    state = getSystemState();

    // this is for querry LPMx.5 just one time
    if(forceSendState)
    {
        if (state == ACTIVE)
        {   // Active
            STREAM_put_word(JSTATE_CAPTURE_FLAG);
            STREAM_put_long(0x00000000);
            STREAM_put_long(0x00000000);
        }
        else
        {   // LPMx.5
            STREAM_put_word(JSTATE_CAPTURE_FLAG);
            STREAM_put_long(0x00000000);
            STREAM_put_long(0x40000000);
        }
        RetState = 0;
    }
    else
    {
        LPMx5_DEVICE_STATE = state;

        if(oldLPMx5_DEVICE_STATE != LPMx5_DEVICE_STATE)
        {
            if( LPMx5_DEVICE_STATE == ACTIVE )
            {

                STREAM_put_word(JSTATE_CAPTURE_FLAG);
                STREAM_put_long(0x00000000);
                STREAM_put_long(0x00000000);
                RetState = 1;
            }
            else
            {
                STREAM_put_word(JSTATE_CAPTURE_FLAG);
                STREAM_put_long(0x00000000);
                STREAM_put_long(0x40000000);
                RetState = 1;
            }
        }
    }
    return RetState;
}
