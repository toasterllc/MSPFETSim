/**
* \ingroup MODULMACROS
*
* \file Configure.c
*
* \brief Set certain parameters to spcified values
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

#include "error_def.h"
#include "arch.h"
#include "edt.h"
#include "hal.h"
#include "stream.h"
#include "ConfigureParameters.h"
#ifdef DB_PRINT
#include "debug.h"
#endif

/**
  Configure
  Configures specified parameter to the specified value
  inData:  <configureParameter(32) value(32)>
  outData: <>
  configureParameter: contains the parameter to be modified
  value: specifies the vlaue to which to set the parameter
*/

#if defined(MSP430_UIF)
    #ifdef ARCH_MSP432
        extern ARMConfigSettings armConfigSettings;
    #endif
#endif

#if defined(MSP_FET)
        extern ARMConfigSettings armConfigSettings;
#endif

extern ARMConfigSettings armConfigSettings;

extern DeviceSettings deviceSettings;
extern DevicePowerSettings devicePowerSettings;
extern unsigned short altRomAddressForCpuRead;
extern unsigned short wdtctlAddress5xx;
extern unsigned short assertBslValidBit;
extern unsigned short enhancedPsa;
unsigned long JTAGLock5xx = 0xCACACACA;

HAL_FUNCTION(_hal_Configure)
{
    unsigned long configureParameter;
    unsigned long value;

    // Retrieve parameters from stream
    if(STREAM_get_long(&configureParameter) < 0)
    {
        return (HALERR_CONFIG_NO_PARAMETER);
    }

    if(STREAM_get_long(&value) < 0)
    {
        return (HALERR_CONFIG_NO_VALUE);
    }

    // Decode and parameter and set value
    switch(configureParameter)
    {
    case (CONFIG_PARAM_ENHANCED_PSA):
        enhancedPsa = value;
        break;

    case (CONFIG_PARAM_PSA_TCKL_HIGH):
        IHIL_SetPsaTCLK(value);
        break;

    case (CONFIG_PARAM_CLK_CONTROL_TYPE):
        deviceSettings.clockControlType = value;
        break;

    case (CONFIG_PARAM_DEFAULT_CLK_CONTROL):
        break;

    case (CONFIG_PARAM_POWER_TESTREG_MASK):
        devicePowerSettings.powerTestRegMask = value;
        break;

    case (CONFIG_PARAM_POWER_TESTREG_DEFAULT):
        devicePowerSettings.powerTestRegDefault = value;
        break;

    case (CONFIG_PARAM_TESTREG_ENABLE_LPMX5):
        devicePowerSettings.enableLpmx5TestReg = value;
        break;

    case (CONFIG_PARAM_TESTREG_DISABLE_LPMX5):
        devicePowerSettings.disableLpmx5TestReg = value;
        break;

    case (CONFIG_PARAM_POWER_TESTREG3V_MASK):
        devicePowerSettings.powerTestReg3VMask = value;
        break;

    case (CONFIG_PARAM_POWER_TESTREG3V_DEFAULT):
        devicePowerSettings.powerTestReg3VDefault = value;
        break;

    case (CONFIG_PARAM_TESTREG3V_ENABLE_LPMX5):
        devicePowerSettings.enableLpmx5TestReg3V = value;
        break;

    case (CONFIG_PARAM_TESTREG3V_DISABLE_LPMX5):
        devicePowerSettings.disableLpmx5TestReg3V = value;
        break;

    case (CONFIG_PARAM_JTAG_SPEED):
    {
        unsigned long sbwValue = 0;
        if(STREAM_get_long(&sbwValue) < 0)
        {
            return (HALERR_CONFIG_NO_VALUE);
        }
        IHIL_SetJtagSpeed(value, sbwValue);
    break;
    }

    case (CONFIG_PARAM_SFLLDEH):
        deviceSettings.stopFLL = value;
        break;

    case (CONFIG_ALT_ROM_ADDR_FOR_CPU_READ):
        altRomAddressForCpuRead = value;
        break;

    case (CONFIG_WDT_ADDRESS_5XX):
        wdtctlAddress5xx = value;
        break;

    case (CONFIG_ASSERT_BSL_VALID_BIT):
        deviceSettings.assertBslValidBit = value;
        break;

    case (CONFIG_PARAM_SCS_BASE_ADDRESS):
#if defined(MSP430_UIF) || defined(MSP_FET)
    #ifdef ARCH_MSP432
        armConfigSettings.scsBase = value;
    #endif
#endif
        break;

    case (CONFIG_PARAM_FPB_BASE_ADDRESS):
#if defined(MSP430_UIF) || defined(MSP_FET)
    #ifdef ARCH_MSP432
        armConfigSettings.fpbBase = value;
    #endif
#endif
        break;

    case (CONFIG_PARAM_INTERRUPT_OPTIONS):
#if defined(MSP430_UIF) || defined(MSP_FET)
  #ifdef ARCH_MSP432
        armConfigSettings.interruptOptions = value | 0x80;
  #endif
#endif
        break;

    case (CONFIG_PARAM_ULP_MSP432):
#if defined(MSP430_UIF) || defined(MSP_FET)
    #ifdef ARCH_MSP432
        armConfigSettings.ulpDebug = value;
    #endif
#endif
        break;

    case (CONFIG_PARAM_JTAG_LOCK_5XX):
        JTAGLock5xx = value;
        break;
        
    default:
        return (CONFIG_PARAM_UNKNOWN_PARAMETER);
    }

    return(0);
}

