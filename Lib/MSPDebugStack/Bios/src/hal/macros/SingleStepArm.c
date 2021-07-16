/*
 * \ingroup MODULMACROS
 *
 * \file SingleStepArm.c
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

/**
  SingleStepMSP432: Single Steps the Core Debug
*/

#define DISABLE_INTERRUPTS_SINGLE_STEP 0x01

#if defined(MSP430_UIF) || defined(MSP_FET)
extern ARMConfigSettings armConfigSettings;
#endif

HAL_FUNCTION(_hal_SingleStepArm)
{
#if defined(MSP430_UIF) || defined(MSP_FET)
    unsigned long data;
    unsigned short retry = MAX_RETRY;

    // First Disable all breakpoints
    data = KEY;
    if(IHIL_Write_Read_Mem_Ap(0, FP_CTRL, &data, WRITE) == -1)
    {
        return HALERR_UNDEFINED_ERROR;
    }

    // Perform actual single step with interrupts disabled
    data = DBGKEY | C_DEBUGEN | C_HALT;
    
    if (armConfigSettings.interruptOptions & DISABLE_INTERRUPTS_SINGLE_STEP)
    {
      data |= C_MASKINTS;
    }
    
    if(IHIL_Write_Read_Mem_Ap(0, DHCSR, &data, WRITE) == -1)
    {
        return HALERR_UNDEFINED_ERROR;
    }
    
    data &= ~(C_HALT);
    data |= C_STEP;
    if(IHIL_Write_Read_Mem_Ap(0, DHCSR, &data, WRITE) == -1)
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
    if(!retry)
    {
        return HALERR_UNDEFINED_ERROR;
    }
#endif

    return 0;
}
