/*
 * Copyright (C) 2013 - 2014 Texas Instruments Incorporated - http://www.ti.com/
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

#include "dpuser.h"
#include "dpalg.h"
#include "dpdef.h"
#include "FpgaVersion.h"
#include "fpga_top.h"
#include "arch.h"

unsigned char UpdateFpga(void);
void configureJtagPort(void);

const unsigned short fpga_Version_ @ "FPGAVERSION" = FPGA_VERSION;
#pragma required=fpga_Version_
const unsigned long fpga_Signature_ @ "HALSIGNATURE" = FPGA_SIGNATURE;
#pragma required=fpga_Signature_

#pragma required=UpdateFpga
unsigned char UpdateFpga(void)
{
    DPUCHAR status;

    // Configure JTAG port for communication with FPGA
    configureJtagPort();

    // Select FPGA programming through GPIO pins
    hardware_interface = GPIO_SEL;
    enable_mss_support = FALSE;

    // Set the fpga bitstream as image for programming via DirectC
    image_buffer = (DPUCHAR*)top;

    // Start programming
    // At first, verify whether the FPGA already contains the correct bitstream
    Action_code = DP_VERIFY_ACTION_CODE;
    status = dp_top();

    if(status != DPE_SUCCESS)  // If verification fails, the FPGA needs to be reprogrammed
    {

        // Erase FPGA memory
        Action_code = DP_ERASE_ACTION_CODE;
        status = dp_top();
        if(status != DPE_SUCCESS)
        {
            return status;
        }

        // Program FPGA bitstream
        Action_code = DP_PROGRAM_ACTION_CODE;
        status = dp_top();
        if(status != DPE_SUCCESS)
        {
            return status;
        }

        // Verify FPGA memory
        Action_code = DP_VERIFY_ACTION_CODE;
        status = dp_top();
        if(status != DPE_SUCCESS)
        {
            return status;
        }
    }

    // Reset the FPGA
    P9OUT |= BIT4;
    __delay_cycles(100);
    P9OUT &= ~BIT4;

    return status;
}

void configureJtagPort(void)
{
    *(_Jtag_FPGA.DIRECTION) = _Jtag_FPGA.TCK |
                              _Jtag_FPGA.TMS |
                              _Jtag_FPGA.TDI |
                              _Jtag_FPGA.TST;
}
