/*
 * \ingroup MODULMACROS
 *
 * \file ScanAPArm.c
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
  ScanAP: Scan access port and provide back nil terminated list
*/
#if defined(MSP430_UIF) || defined(MSP_FET)
    extern unsigned long cswValues[4];
#endif

HAL_FUNCTION(_hal_ScanApArm)
{
#if defined(MSP430_UIF) || defined(MSP_FET)
    unsigned long val = 0;
    unsigned long apsel = 0;

    // Set system and debug power up if cleared
    if( powerUpArm() != 0)
    {
        return HALERR_UNDEFINED_ERROR;
    }

    IHIL_Write_Read_Ap(AP_IDR | apsel, &val, READ);
    if(val)
    {
        STREAM_put_long(val); // IDR value

        IHIL_Write_Read_Ap(AP_BASE | apsel, &val, READ);
        STREAM_put_long(val); // BASE value

        IHIL_Write_Read_Ap(AP_CFG | apsel, &val, READ);
        STREAM_put_long(val); // CFG value

        STREAM_put_byte(apsel >> 24); // Port number

        // Read and save CSW value
        IHIL_Write_Read_Ap(AP_CSW | apsel, &val, READ);
        cswValues[(apsel >> 24) & 0x3] = val & ~(AP_CSW_SIZE_MASK | AP_CSW_ADDRINC_MASK);
    }
    STREAM_put_long(val);
#endif
    return 0;
}
