/**
* \file Reset430I.c
*
* \brief Write words (16bit values) to a memory mapped location
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

//! \ingroup MODULMACROS
//! \file Reset430I.c
//! \brief

#include "arch.h"
#include "edt.h"
#include "hal.h"
#include "hal_ref.h"
#include "stream.h"
#include "../include/hw_compiler_specific.h"

/**
   _hal_Reset430I
   this macro will apply a BSL sequence to reset the
   device and keep it in LPM4 afterwards

*/

HAL_FUNCTION(_hal_Reset430I)
{
        volatile unsigned short id=0;

        /* Apply BSL Entry Sequence to Stop device execution */
        IHIL_BSL_EntrySequence(1);
        /* Now the device should be in LPM4 */

        IHIL_Delay_1ms(500);

        IHIL_Open(RSTHIGH);
        IHIL_TapReset();
        IHIL_CheckJtagFuse();

        id = cntrl_sig_capture();
        if (id == JTAGVERSION)
        {
            STREAM_put_byte((unsigned char)0x1);
            STREAM_put_byte((unsigned char)0x89);
            STREAM_put_byte((unsigned char)SPYBIWIREJTAG);
            return 0;
        }
        return -1;
}
