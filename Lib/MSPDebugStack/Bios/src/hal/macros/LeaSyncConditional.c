/**
* \ingroup MODULMACROSXV2
*
* \file LeaSyncConditional.c
*
* \brief <FILEBRIEF>
*
*/
/*
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
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
#include "JTAG_defs.h"
#include "EEM_defs.h"
#include "global_variables.h"

#define LEASCCNF1 0x08
#define LEASCBSY  0x01

HAL_FUNCTION(_hal_LeaSyncConditional)
{
    unsigned long baseAddres = 0;
    unsigned short tclkCycles = 20000;
    volatile unsigned long currentPcAddress = 0;
    unsigned short leaReg = 0;

    STREAM_get_long(&baseAddres);

    leaReg = ReadMemWordXv2(baseAddres | LEASCCNF1);
    if(!(leaReg & LEASCBSY))
    {
        STREAM_put_word((leaReg & LEASCBSY));
        return 0;
    }

    // Set PC to "safe" address
    SetPcXv2(0x0080, SAFE_PC_ADDRESS);
    cntrl_sig_16bit();
    SetReg_16Bits(0x0501);
    IHIL_Tclk(1);
    addr_capture();

    cntrl_sig_16bit();
    SetReg_16Bits(0x1501);

    while(tclkCycles-- > 0)
    {
        IHIL_TCLK();
    }
    addr_capture();
    currentPcAddress = SetReg_20Bits(0x0);

    if((currentPcAddress != SAFE_PC_ADDRESS) && (currentPcAddress != SAFE_PC_ADDRESS+2))
    {
        return -1;
    }

    leaReg = ReadMemWordXv2(baseAddres | LEASCCNF1);
    STREAM_put_word((leaReg & LEASCBSY));
    return 0;
}
