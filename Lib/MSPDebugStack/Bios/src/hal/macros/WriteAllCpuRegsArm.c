/*
 * \ingroup MODULMACROSMSP432
 *
 * \file ReadAllCpuRegsArm.c
 *
 * \brief Read CPU register values, except R0 and R2
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

/**
  WriteAllCpuRegsMSP432
  Write CPU register values
  inData: R0-R12, SP, LR, PC, SR, XPRS, SpecialRegs, MSP_SP, PSP_SP
  outData: -
*/
#if defined(MSP430_UIF) || defined(MSP_FET)
extern ARMConfigSettings armConfigSettings;
#endif

short writeCpuRegsArm(unsigned long data, unsigned long Rx)
{
    unsigned char retry = MAX_RETRY;
    unsigned long returnVal = 0;
    IHIL_Write_Read_Mem_Ap(0, DCRDR, &data, WRITE); // Write value
    data = Rx | REG_WnR;
    IHIL_Write_Read_Mem_Ap(0, DCRSR, &data, WRITE); // Request Write

    // check if wirte was sucessfully
    do
    {
        IHIL_Write_Read_Mem_Ap(0, DHCSR, &returnVal, READ);
    } while(--retry && !(returnVal & S_REGRDY));

    if(!retry)
    {
        return 0;
    }
    return 1;
}

HAL_FUNCTION(_hal_WriteAllCpuRegsArm)
{
#if defined(MSP430_UIF) || defined(MSP_FET)
    unsigned long data = 0;
    // Read the General Purpose registers.
    for (unsigned long Rx = 0; Rx < 16; ++Rx)
    {
        STREAM_get_long(&data);
        if(!writeCpuRegsArm(data, Rx))
        {
            return HALERR_UNDEFINED_ERROR;
        }
    }
    STREAM_get_long(&data);
    if(!writeCpuRegsArm(data, xPSR_REG))
    {
        return HALERR_UNDEFINED_ERROR;
    }

    STREAM_get_long(&data);
    if(!writeCpuRegsArm(data, spec_REG))
    {
        return HALERR_UNDEFINED_ERROR;
    }
    STREAM_get_long(&data);
    if(!writeCpuRegsArm(data, MSP_SP))
    {
        return HALERR_UNDEFINED_ERROR;
    }
    STREAM_get_long(&data);
    if(!writeCpuRegsArm(data, PSP_SP))
    {
        return HALERR_UNDEFINED_ERROR;
    }
#endif
    return 0;
}