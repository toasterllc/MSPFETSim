/*
 * \ingroup MODULMACROS
 *
 * \file RunArm.c
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
#include "JTAG_defs.h"
#include "error_def.h"
#include "global_variables.h"
#include "error_def.h"

/**
  Run: Runs the Core Debug
*/
#define DISABLE_INTERRUPTS_RUN 0x02
#define DISABLE_INTERRUPTS_UPDATE_PRIMSK 0x80

#if defined(MSP430_UIF) || defined(MSP_FET)
extern ARMConfigSettings armConfigSettings;
extern short writeCpuRegsArm(unsigned long data, unsigned long Rx);
extern short ReadCpurRegArm(unsigned long *data, unsigned long Rx);
#endif

short confSystemHighPower()
{
    // Set system and debug power up if cleared
    unsigned long data = 0;
    unsigned short* syncWithRunVarAddress = getTargetRunningVar();

    //force high system power
    if( powerUpArm() != 0)
    {
        return -1;
    }
    // Clear sleep on ISR exit if it was set by user code
    if(IHIL_Write_Read_Mem_Ap(0, SCB_SCR, &data, READ) == -1)
    {
        return -1;
    }
    if(data & SCB_SCR_SLEEPONEXIT_Msk)
    {
        data &= ~SCB_SCR_SLEEPONEXIT_Msk;
        if(IHIL_Write_Read_Mem_Ap(0, SCB_SCR, &data, WRITE) == -1)
        {
            return - 1;
        }
    }
    if(syncWithRunVarAddress)
    {
          *syncWithRunVarAddress = 0x0001;
    }
    // Do the actual run
    data = DBGKEY | C_DEBUGEN;
    if(IHIL_Write_Read_Mem_Ap(0, DHCSR, &data, WRITE) == -1)
    {
        return -1;
    }
    return 0;
}

short confSystemLowPower()
{
    // Set system and debug power up if cleared
    unsigned long val = 0;
    unsigned long data = 0;
    unsigned short* syncWithRunVarAddress = getTargetRunningVar();

    IHIL_Write_Read_Dp(DP_CTRL_STAT, &val, READ);

    // Request system power state tansitiopn

    // deassert CSYSPWRUPREQ
    val &=  ~DP_CTRL_STAT_CSYSPWRUPREQ;
    // keep CDBGPWRUPREQ asserted
    val |= DP_CTRL_STAT_CDBGPWRUPREQ;
    IHIL_Write_Read_Dp(DP_CTRL_STAT, &val, WRITE);

    if(syncWithRunVarAddress)
    {
        *syncWithRunVarAddress = 0x0001;
    }
    // Do the actual run
    data = DBGKEY | C_DEBUGEN;
    if(IHIL_Write_Read_Mem_Ap(0, DHCSR, &data, WRITE) == -1)
    {
        return -1;
    }
    // deassert CDBGPWRUPREQ
    val  &=  ~DP_CTRL_STAT_CDBGPWRUPREQ;
    IHIL_Write_Read_Dp(DP_CTRL_STAT, &val, WRITE);

    // wait some time for ACK to settle
    IHIL_Delay_1ms(200);

    return 0;
}

HAL_FUNCTION(_hal_RunArm)
{
#if defined(MSP430_UIF) || defined(MSP_FET)
    unsigned long data = 0;
    unsigned short releaseJtag = 0;
    unsigned short retry = MAX_RETRY;

    STREAM_get_word(&releaseJtag);

    // First Disable all breakpoints
    data = KEY;
    if(IHIL_Write_Read_Mem_Ap(0, FP_CTRL, &data, WRITE) == -1)
    {
        return HALERR_UNDEFINED_ERROR;
    }

    if (armConfigSettings.interruptOptions & DISABLE_INTERRUPTS_UPDATE_PRIMSK)
    {
      if (armConfigSettings.interruptOptions & DISABLE_INTERRUPTS_RUN)
      {
        // Making PRIMASK be effective requires one single step before
        ReadCpurRegArm(&data, spec_REG);
        data |= 0x01;
        writeCpuRegsArm(data, spec_REG);
      }
      else
      {
        // Making PRIMASK be effective requires one single step before
        ReadCpurRegArm(&data, spec_REG);
        data &= ~0x01;
        writeCpuRegsArm(data, spec_REG);
      }

      armConfigSettings.interruptOptions &= ~DISABLE_INTERRUPTS_UPDATE_PRIMSK;
    }

    // Perform single step with interrupts disabled to step over current hw breakpoint
    data = DBGKEY | C_DEBUGEN | C_MASKINTS | C_HALT;
    if(IHIL_Write_Read_Mem_Ap(0, DHCSR, &data, WRITE ) == -1)
    {
        return HALERR_UNDEFINED_ERROR;
    }

    data &= ~(C_HALT);
    data |= C_STEP;
    if(IHIL_Write_Read_Mem_Ap(0, DHCSR, &data, WRITE ) == -1)
    {
        return HALERR_UNDEFINED_ERROR;
    }

    // Wait for step to finish
    do
    {
        if(IHIL_Write_Read_Mem_Ap(0, DHCSR, &data, READ) == -1)
        {
            return HALERR_UNDEFINED_ERROR;
        }
    } while(!(data & S_HALT) && --retry);

    // disbale masking of interrupts
    data = DBGKEY | C_DEBUGEN | C_HALT;
    if(IHIL_Write_Read_Mem_Ap(0, DHCSR, &data, WRITE ) == -1)
    {
        return HALERR_UNDEFINED_ERROR;
    }

    // Re-enable all breakpoints
    data = KEY | ENABLE;
    if(IHIL_Write_Read_Mem_Ap(0, FP_CTRL, &data, WRITE) == -1)
    {
        return HALERR_UNDEFINED_ERROR;
    }

    if(armConfigSettings.ulpDebug)
    {
        if(confSystemLowPower() != 0)
        {
            return HALERR_UNDEFINED_ERROR;
        }
    }
    else
    {
        if(confSystemHighPower() != 0)
        {
            return HALERR_UNDEFINED_ERROR;
        }
    }

    if(releaseJtag)
    {
        IHIL_Close(); // release JTAG on go
    }
#endif
    return 0;
}
