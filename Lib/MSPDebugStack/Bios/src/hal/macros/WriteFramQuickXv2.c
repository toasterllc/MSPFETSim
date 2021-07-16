/**
* \ingroup MODULMACROSXV2
*
* \file WriteFramQuickXv2.c
*
* \brief <FILEBRIEF>
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

#include "arch.h"
#include "edt.h"
#include "hal.h"
#include "stream.h"
#include "error_def.h"
#ifdef DB_PRINT
#include "debug.h"
#endif

/**
  WriteMemBytesXv2
  Write bytes to a memory mapped location.
  inData:  <addr(32)> <length(32)> <data(8)>{*}
  outData: -
  addr: the address to start writing to
  length: number of bytes to write
  data: data to write to the given location
*/

HAL_FUNCTION(_hal_WriteFramQuickXv2)
{

    static unsigned long  lLen;
    static unsigned long Addr = 0x0 ;
    static unsigned short Mova;
    static unsigned short ret_len = 0;
    static unsigned short id;
    short ret_value = 0;
    unsigned short *pBuf;
    unsigned short sizeOfBuf;
    // fist message--------------------------------------------
    if(flags & MESSAGE_NEW_MSG)
    {
        if(STREAM_get_long(&Addr) != 0)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_ADDRESS);
        }
        // get length ot be flashed
        if(STREAM_get_long(&lLen) != 0)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_LENGTH);
        }

        id = cntrl_sig_capture();

        // i_SetPcRel
        Mova  = 0x0080;
        Mova += (unsigned short)((Addr>>8) & 0x00000F00);
        Addr  = (unsigned short)((Addr & 0xFFFF));

        SetPcXv2(Mova, Addr);

        cntrl_sig_16bit();
        SetReg_16Bits(0x0500);
        IHIL_Tclk(1);

        data_quick();
    }
    if(!(flags & MESSAGE_LAST_MSG ))
    {
      STREAM_out_change_type(RESPTYP_ACKNOWLEDGE);
      STREAM_flush();
    }
    
    // not first and not last message
    STREAM_get_buffer((void **)&pBuf,&sizeOfBuf);
    for(; lLen && sizeOfBuf; lLen--)
    {
         IHIL_Tclk(1);
         SetReg_16Bits(*pBuf++);
         sizeOfBuf -= 2;
         IHIL_Tclk(0);
    }
    
    // last message
    if(flags & MESSAGE_LAST_MSG )
    {
        cntrl_sig_16bit();
        SetReg_16Bits(0x0501);
        IHIL_Tclk(1);
        if(id == JTAGVERSION91 || id == JTAGVERSION99 || id == JTAGVERSION98)
        {
            SetPcXv2(0x80, SAFE_PC_ADDRESS); // Set PC to "safe" JMP $ address
            cntrl_sig_16bit();
            SetReg_16Bits(0x0501);
            IHIL_Tclk(1);
            addr_capture();
        }

        STREAM_put_word(ret_len);
    }
    else if(lLen != 0)
    {
        STREAM_out_change_type(RESPTYP_ACKNOWLEDGE);
        ret_value = 0;
    }
    else
    {
        cntrl_sig_16bit();
        SetReg_16Bits(0x0501);
        IHIL_Tclk(1);
        if(id == JTAGVERSION91 || id == JTAGVERSION99 || id == JTAGVERSION98)
        {
            SetPcXv2(0x80, SAFE_PC_ADDRESS); // Set PC to "safe" JMP $ address
            cntrl_sig_16bit();
            SetReg_16Bits(0x0501);
            IHIL_Tclk(1);
            addr_capture();
        }

        ret_value = HALERR_EXECUTE_FUNCLET_EXECUTION_ERROR;
    }
    return(ret_value);
}
