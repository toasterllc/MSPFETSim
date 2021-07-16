/**
* \ingroup MODULMACROS
*
* \file PollJStateReg.c
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

#include "hw_compiler_specific.h"
#include "HalGlobalVars.h"
#include "arch.h"
#include "edt.h"
#include "hal.h"
#include "hal_ref.h"
#include "stream.h"
#include "stddef.h"
#include "error_def.h"
#include "JTAG_defs.h"
#include "global_variables.h"


#define JSTATE20  0x20
#define JSTATE21  0x21
#define DSTATE_PC 0x00

#define ARM_PCMCTL0 (0x40010000UL)
#define ARM_PCSR    (0xE000101CUL)

//-----------------------------------------------------------------------------
#if defined(eZ_FET) || defined(MSP_FET)
//-----------------------------------------------------------------------------

#pragma pack(1)
typedef struct DState432
{
    unsigned long pc;
    unsigned long pstate;
} DState432_t;
#pragma pack()

#pragma pack(1)
typedef struct DState430
{
    unsigned long long JState;
} DState430_t;
#pragma pack()

#pragma pack(1)
typedef struct EnergyTraceRecordEt7
{
    unsigned char eventID;
    unsigned long TimeStamp;
    union
    {
        DState430_t dstate430;
        DState432_t dstate432;
    };
    unsigned long currentTicks;
    unsigned int voltage;
} EnergyTraceRecordEt7_t;
#pragma pack()

#pragma pack(1)
typedef struct EnergyTraceRecordEt8
{
    unsigned char eventID;
    unsigned long TimeStamp;
    unsigned long currentTicks;
    unsigned int voltage;
} EnergyTraceRecordEt8_t;
#pragma pack()

#define NUM_RECORDS_TO_CAPTURE 5

typedef enum ETMode
{
    ET_OFF_MODE = 0,
} ETMode_t;


unsigned long getTimeStamp();
unsigned long getIMeasure();

#pragma inline=forced
unsigned long getTimeStamp()
{
    unsigned long TimeStamp = 0;
    unsigned short currentTA0R = TA0R;
    unsigned short currentTA0R2 = TA0R;
    unsigned short * TimeTick = 0;
    unsigned short testa = 0;

    if (currentTA0R2 < currentTA0R)
    {
        currentTA0R = currentTA0R2;
    }

    STREAM_getSharedVariable(ID_SHARED_MEMORY_TYPE_TIME_TICK, &TimeTick);
    testa = *(unsigned short*)TimeTick;

    TimeStamp = (((unsigned long)testa) << 16) + (unsigned long)(currentTA0R & 0xFFFF);
    return TimeStamp;
}

#pragma inline=forced
unsigned long getIMeasure()
{
    unsigned long IMeasure = 0;
#ifdef MSP_FET
    unsigned short currentIClocks = TB0R;
    unsigned short currentIClocks2 = TB0R;
#else
    unsigned short currentIClocks = TA2R;
    unsigned short currentIClocks2 = TA2R;
#endif
    unsigned short* ITick = 0;
    unsigned short testt = 0;

    if (currentIClocks2 < currentIClocks)
    {
        currentIClocks = currentIClocks2;
    }

    STREAM_getSharedVariable(ID_SHARED_MEMORY_TYPE_I_TICK, &ITick);
    testt = *(unsigned short*)ITick;

    IMeasure = (((unsigned long)testt) << 16) + (unsigned long)(currentIClocks & 0xFFFF);
    return IMeasure;
}
//-----------------------------------------------------------------Event type 8 only analog ------------------------------------------------------------------

HAL_FUNCTION(_hal_PollJStateRegEt8)
{
    static EnergyTraceRecordEt8_t buffer[NUM_RECORDS_TO_CAPTURE] = {0};
    static unsigned short currentIndex = 0;
    double vcc = 0;
    double extVcc= 0 ;
    unsigned short mEtGatedMode = 0;
    unsigned short* syncWithRunVarAddress = 0;

    if(STREAM_discard_bytes(8) == -1)
    {
            return -1;
    }
    // request shared var from bios to sync with RestoreContextRun function

    STREAM_get_word(&mEtGatedMode);

    if(mEtGatedMode)
    {
        syncWithRunVarAddress = getTargetRunningVar();
        if(syncWithRunVarAddress)
        {
             if(!(*syncWithRunVarAddress))
             {
                currentIndex = 0;
                return 2;
             }
        }
        else
        {
            return -1;
        }
    }

    buffer[currentIndex].eventID = 8;

    while(TA0R > 0xFFA0 || TA0R  < 2)
    {
        IHIL_Delay_1us(3);
    }

    _DINT_FET();

    buffer[currentIndex].TimeStamp = getTimeStamp();
    buffer[currentIndex].currentTicks = getIMeasure();

    IHIL_GetVcc(&vcc, &extVcc);

    _EINT_FET();
    buffer[currentIndex].voltage = (unsigned int)(vcc ? vcc : extVcc);

    // Energy trace data is available send it first -> don't check LPMx.5 or Breakpoint hit
    if(++currentIndex == NUM_RECORDS_TO_CAPTURE)
    {
        currentIndex = 0;
        STREAM_put_word(ENERGYTRACE_INFO);                  // Event ID
        STREAM_put_byte(NUM_RECORDS_TO_CAPTURE);            // Number of records that is sent
        STREAM_put_byte(sizeof(EnergyTraceRecordEt8_t));    // Size of Record
        STREAM_put_bytes((void *)buffer, sizeof(EnergyTraceRecordEt8_t) * NUM_RECORDS_TO_CAPTURE);
        return 1;
    }

    return 2;
}
//-----------------------------------------------------------------------------
#endif
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
#ifdef MSP430_UIF
//-----------------------------------------------------------------------------
unsigned short targetIsRunning = 0;

HAL_FUNCTION(_hal_PollJStateRegEt8)
{
    return -1;
}
//-----------------------------------------------------------------------------
#endif
//-----------------------------------------------------------------------------

#pragma inline=forced
short PollforBpHit(unsigned long long JStateValue)
{
    unsigned short* syncWithRunVarAddress = 0;
    if((!(JStateValue & LPMX5_MASK_J)) && (JStateValue & BP_HIT_MASK_J))
    {
        STREAM_put_word(BP_HIT_FLAG);
        STREAM_put_long(JStateValue & 0xFFFFFFFF);
        STREAM_put_long(JStateValue >> 32);

        // Stop polling in next loop run --> Device CPU is stoped due to BP Hit
        syncWithRunVarAddress = getTargetRunningVar();
        if(syncWithRunVarAddress)
        {
             *syncWithRunVarAddress = 0x0000;
        }
        return  1;
    }
    return 2;
}

unsigned long long prevJState = 0x0000000000000000;

#pragma inline=forced
short PollforLPMx5(unsigned long long JStateValue, unsigned short forceSendState)
{
    StreamSafe stream_tmp;
    short RetState = -1, LPMx5Wakeup = 0;

    unsigned short* syncWithRunVarAddress = 0;

    // if we enter an LPM4 and the PC indicates taht the device wakes up form
    // LPMx.5 scan mailbox for Bootcode pattern
    if (!(JStateValue & LPMX5_MASK_J) &&
      (((JStateValue & LPM4_1MASK_J) == LPM4_1MASK_J) || ((JStateValue & LPM4_2MASK_J) == LPM4_2MASK_J)))
    {
        if (jmb_exchange() == JTAGVERSION99)
        {
            // check if JTAG mailbox is ready & perform input request
            if (SetReg_16Bits(0x0004) == 0x1207)
            {
                if (SetReg_16Bits(0x0000) == 0xA55A)
                {
                    LPMx5Wakeup = 1;
                }
            }
        }
    }

    if(forceSendState || ((prevJState & LPMX5_MASK_J) != (JStateValue & LPMX5_MASK_J)) || LPMx5Wakeup)
    {
        STREAM_put_word(JSTATE_CAPTURE_FLAG);
        STREAM_put_long(JStateValue & 0xFFFFFFFF);
        STREAM_put_long(JStateValue >> 32);

        if(((!(JStateValue & LPMX5_MASK_J)) && !forceSendState) || (LPMx5Wakeup && !forceSendState))
        {   //Device woke up on its own, so sync again
            //Setup values for watchdog control regsiters
            unsigned char DummyIn[8] = {wdtctlAddress5xx & 0xFF,(wdtctlAddress5xx >> 8) & 0xFF,
                                     WDTHOLD_DEF,WDTPW_DEF,0,0,0,0};
            STREAM_internal_stream(DummyIn, sizeof(DummyIn), NULL, 0, &stream_tmp);
            HAL_SyncJtag_Conditional_SaveContextXv2(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG);
            STREAM_external_stream(&stream_tmp);

             // Stop polling in next loop run --> Device CPU is stoped due to BP Hit
            syncWithRunVarAddress = getTargetRunningVar();
            if(syncWithRunVarAddress)
            {
                *syncWithRunVarAddress = 0x0000;
            }
        }
        RetState = 1;
    }
    else
    {
        RetState = 2;
    }
    if(forceSendState)
    {
        RetState = 0;
    }
    else
    {
        prevJState = JStateValue;
    }
    return RetState;
}

//-----------------------------------------------------------------Event type 7----------------------------------------------------------------------

/**
  PollJStateReg
  Queries the JSTATE register and reports any changes in the relevant bits
  inData:  <forceSendState(16)> <pollBreakpoints(16)> <pollLPM(16)> <doEnergyTrace(16)> <gateMode(16)>
  outData: <eventFlag(16)> [<JStateLow(32)> <JStateHigh(32)> / <numEnergyTraceRecords> <energyTrace records>]
*/
short PollJStateReg(unsigned short JStateVersion);

HAL_FUNCTION(_hal_PollJStateReg20)
{
    return PollJStateReg(JSTATE20);
}

HAL_FUNCTION(_hal_PollJStateReg)
{
    return PollJStateReg(JSTATE21);
}

HAL_FUNCTION(_hal_PollDStatePCRegEt)
{
    return PollJStateReg(DSTATE_PC);
}

short PollJStateReg(unsigned short JStateVersion)
{
    unsigned long long lOut_long_long = 0;
#if defined(MSP_FET)
    unsigned long lOut_long = 0;
#endif
    short RetState = -1;
    unsigned short forceSendState = 0;

    unsigned short mBreakpointPollingActive = 0;
    unsigned short mLpmPollingActive = 0;
    unsigned short mEtActive = 0;
    unsigned short* syncWithRunVarAddress = 0;

    STREAM_get_word(&forceSendState);
    STREAM_get_word(&mBreakpointPollingActive);
    STREAM_get_word(&mLpmPollingActive);
    STREAM_get_word(&mEtActive);

    if(STREAM_discard_bytes(2) == -1)
    {
        return -1;
    }

    syncWithRunVarAddress = getTargetRunningVar();
    if(!syncWithRunVarAddress)
    {
        return -1;
    }

    if(IHIL_GetPrevInstruction() != IR_JSTATE_ID)
    {
        // Query new JSTATE
        jstate_read();
    }

#if defined(eZ_FET) || defined(MSP_FET)
    // check if Energy Trace should be enabled
    if(*syncWithRunVarAddress && mEtActive)
    {
        static EnergyTraceRecordEt7_t buffer[NUM_RECORDS_TO_CAPTURE] = {0};
        static unsigned short currentIndex = 0;
        double vcc = 0;
        double extVcc = 0;

        buffer[currentIndex].eventID = 7;

        while(TA0R > 0xFFA0 || TA0R  < 2)
        {
            IHIL_Delay_1us(3);
        }

        _DINT_FET();

        buffer[currentIndex].TimeStamp = getTimeStamp();
        buffer[currentIndex].currentTicks = getIMeasure();

        IHIL_GetVcc(&vcc, &extVcc);

        _EINT_FET();

#if defined(MSP_FET)
        if(JStateVersion == DSTATE_PC)
        {
            // read out PC sampling register
            IHIL_Write_Read_Mem_Ap(0, ARM_PCSR, &lOut_long, READ);
            buffer[currentIndex].dstate432.pc = lOut_long;
            // get PCMCTL0 register
            IHIL_Write_Read_Mem_Ap(0, ARM_PCMCTL0, &lOut_long, READ);
            buffer[currentIndex].dstate432.pstate = lOut_long;
        }
        else
        {
#endif
            lOut_long_long = SetReg8_64Bits(0xFFFFFFFFFFFFFFFF,30,JStateVersion);
            buffer[currentIndex].dstate430.JState = lOut_long_long;
#if defined(MSP_FET)
        }
#endif
        buffer[currentIndex].voltage = (unsigned short)(vcc ? vcc : extVcc);

        // Energy trace data is available send it first -> don't check LPMx.5 or Breakpoint hit
        if(++currentIndex == NUM_RECORDS_TO_CAPTURE)
        {
            currentIndex = 0;
            STREAM_put_word(ENERGYTRACE_INFO);                  // Event ID
            STREAM_put_byte(NUM_RECORDS_TO_CAPTURE);            // Number of records that is sent
            STREAM_put_byte(sizeof(EnergyTraceRecordEt7_t));    // Size of Record
            STREAM_put_bytes((void *)buffer, sizeof(EnergyTraceRecordEt7_t) * NUM_RECORDS_TO_CAPTURE);
            return 1;
        }
    }
    else
#endif
    {
        if(!(*syncWithRunVarAddress))
        {
            if(forceSendState)
            {
                STREAM_put_word(JSTATE_CAPTURE_FLAG);
                STREAM_put_long(0);
                STREAM_put_long(0);
                return 0;
            }
            else
            {
                return 2;
            }
        }
        lOut_long_long = SetReg_64Bits(0xFFFFFFFFFFFFFFFF);
    }

    if(*syncWithRunVarAddress && mBreakpointPollingActive)
    {
        RetState = PollforBpHit(lOut_long_long);
        if(RetState == 1)
        {
            return RetState;
        }
    }

    if((*syncWithRunVarAddress || forceSendState) && mLpmPollingActive && (lOut_long_long>>56 & 0xC0) != 0x40)
    {
        RetState = PollforLPMx5(lOut_long_long, forceSendState);
        if(RetState == 1 || RetState == 0)
        {
            return RetState;
        }
    }
    return 2;
}
