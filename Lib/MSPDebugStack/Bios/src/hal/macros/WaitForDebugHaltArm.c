/*
 * \ingroup MODULMACROS
 *
 * \file WaitForDebugHaltArm.c
 *
 * \brief <FILEBRIEF>
 *
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

#include "arch.h"
#include "edt.h"
#include "hal.h"
#include "stream.h"
#include "error_def.h"
#include "global_variables.h"

/**
  WaitForDebugHalt
  Test the DHCSR register for specific bits and
  return if one of them is set.
  This is best used with the ExecLoop command type.
  inData:  <Mask(16)>
  outData: <Ctrl(16)>
          Mask: the bits to look for
          Ctrl: the read DHCSR register
*/
#if defined(MSP430_UIF) || defined(MSP_FET)
extern ARMConfigSettings armConfigSettings;
#endif

short checkDhcsrRegForBbHit()
{
    unsigned long dhcsrValue = 0;
    unsigned short* syncWithRunVarAddress = getTargetRunningVar();

    if(syncWithRunVarAddress)
    {
         if(!(*syncWithRunVarAddress))
         {
            return 2;
         }
    }

    if(IHIL_Write_Read_Mem_Ap(0, DHCSR, &dhcsrValue, READ) == -1)
    {
        return HALERR_UNDEFINED_ERROR;
    }
    if(dhcsrValue & S_HALT)
    {

        STREAM_put_long(dhcsrValue);
        if(syncWithRunVarAddress)
        {
            *syncWithRunVarAddress = 0x0000;
        }
        return 1;
    }
    return 2;
}


HAL_FUNCTION(_hal_WaitForDebugHaltArm)
{
    short RetState = HALERR_UNDEFINED_ERROR;
#if defined(MSP430_UIF) || defined(MSP_FET)
    unsigned long val = 0;
    if(armConfigSettings.ulpDebug)
    {
        IHIL_Write_Read_Dp(DP_CTRL_STAT, &val, READ);

        if(val & DP_CTRL_STAT_CDBGPWRUPACK )
        {
            if( powerUpArm() != 0)
            {
                return RetState = 2;
            }
            if(checkWakeup() != 0)
            {
                return RetState = 2;
            }
            RetState =  checkDhcsrRegForBbHit();
        }
    }
    else // high power force
    {
        RetState = checkDhcsrRegForBbHit();
    }
#endif
    return RetState;
}
