/*
 * \ingroup MODULMACROS
 *
 * \file GetCpuIdArm.c
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
  GetJtagIdCode: get the JEP idcode from the debug port
*/
HAL_FUNCTION(_hal_GetCpuIdArm)
{
#ifdef MSP_FET
    unsigned long cpuId = 0, revision = 0;
    unsigned short timeout = 20;

    do
    {
        short  result1 = IHIL_Write_Read_Mem_Ap(0, 0x0020100C, &cpuId, READ);
        short  result2 =IHIL_Write_Read_Mem_Ap(0, 0x00201010, &revision, READ);
        if(!result2 || !result1)
        {
            return HALERR_UNDEFINED_ERROR;
        }
    }
    while((!cpuId || !revision ) && --timeout );

    if (cpuId && revision)
    {
        STREAM_put_long(cpuId);
        STREAM_put_long(revision);
        return 0;
    }
#endif
    return HALERR_UNDEFINED_ERROR;
}

HAL_FUNCTION(_hal_CheckDapLockArm)
{
#ifdef MSP_FET
    unsigned long lock = 0;
    unsigned long pw = 0x0000695A;

    IHIL_Write_Read_Mem_Ap(0, 0xE0044000, &pw, WRITE);
    IHIL_Write_Read_Mem_Ap(0, 0xE0044020, &lock, READ);
    STREAM_put_long(lock);
#endif
    return 0;
}

HAL_FUNCTION(_hal_UnlockDap)
{
#ifdef MSP_FET
    unsigned long pw = 0x0000695A;

    unsigned long fa1 = 0x1;
    unsigned long fa2 = 0x0;
    unsigned long fa4 = 0x6902;

    IHIL_Write_Read_Mem_Ap(0, 0xE0044000, &pw, WRITE);

    IHIL_Write_Read_Mem_Ap(0, 0xE0044004, &fa1, WRITE);
    IHIL_Write_Read_Mem_Ap(0, 0xE0044008, &fa2, WRITE);
    IHIL_Write_Read_Mem_Ap(0, 0xE0044010, &fa4, WRITE);

#endif
    return 0;
}
