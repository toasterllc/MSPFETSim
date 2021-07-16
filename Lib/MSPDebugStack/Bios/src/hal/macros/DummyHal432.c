/**
 * \ingroup MODULMACROS
 *
 * \file DummyHal432.c
 *
 * \brief Set certain parameters to spcified values
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

#include "edt.h"
#include "hal.h"

#ifdef MSP430_UIF
    HAL_FUNCTION(_hal_WriteAllCpuRegsArm)          {return  -1;}
    HAL_FUNCTION(_hal_WaitForDebugHaltArm)         {return  -1;}
    HAL_FUNCTION(_hal_SingleStepArm)               {return  -1;}
    HAL_FUNCTION(_hal_ScanApArm)                   {return  -1;}
    HAL_FUNCTION(_hal_RunArm)                      {return  -1;}
    HAL_FUNCTION(_hal_ResetArm)                    {return  -1;}
    HAL_FUNCTION(_hal_ReadAllCpuRegsArm)           {return  -1;}
    HAL_FUNCTION(_hal_MemApTransactionArm)         {return  -1;}
    HAL_FUNCTION(_hal_HaltArm)                     {return  -1;}
    HAL_FUNCTION(_hal_GetJtagIdCodeArm)            {return  -1;}
    HAL_FUNCTION(_hal_EnableDebugArm)              {return  -1;}
    HAL_FUNCTION(_hal_DisableDebugArm)             {return  -1;}
    HAL_FUNCTION(_hal_MemApTransactionArmSwd)      {return  -1;}
    HAL_FUNCTION(_hal_GetInterfaceModeArm)         {return  -1;}
    HAL_FUNCTION(_hal_GetCpuIdArm)                 {return  -1;}
    HAL_FUNCTION(_hal_CheckDapLockArm)             {return  -1;}
    HAL_FUNCTION(_hal_UnlockDap)                   {return  -1;}
#endif
