/**
* \ingroup MODULMACROSXV2
*
* \file RestoreContext_ReleaseJtagXv2.c
*
* \brief Restore the CPU context and releast Jtag control
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
#include "hal_ref.h"
#include "stream.h"
#include "global_variables.h"
#include "../include/hw_compiler_specific.h"
#include "EEM_defs.h"

#define CPUOFF              (0x0010u)
#define DBGJTAGON           (0x0080u)

/**
  RestoreContext_ReleaseJtagXv2
  Restore the CPU context and releast Jtag control.
  inData:  <wdtAddr(16)> <wdtCtrl(16)> <PC(32)> <SR(16)><eemCtrl(16)> <mdb(16)>
  outData: -
  wdtAddr: watchdog timer address
  wdtCtrl: watchdog timer control value
  PC: program counter register (R0)
  SR: status register (R2)
  eemCtrl: EEM control bits (EEM_EN and optional EEM_CLK_EN)
  mdb: value to be put on the Memory Data Bus before release (in case !0)
*/

extern DevicePowerSettings devicePowerSettings;

HAL_FUNCTION(_hal_RestoreContext_ReleaseJtagXv2)
{
    unsigned short wdt_addr;
    unsigned short wdt_value;
    unsigned short Sr;
    unsigned short control_mask = 0;
    unsigned long  lPc;
    unsigned short Mova;
    unsigned short Pc_l;
    unsigned short Mdb;
    unsigned short releaseJtag = 0;
    unsigned short ulpDebug = 0;
    unsigned short* syncWithRunVarAddress = 0;


    STREAM_get_word(&wdt_addr);  // 0
    STREAM_get_word(&wdt_value);  // 2
    STREAM_get_long(&lPc);       // 4-7
    STREAM_get_word(&Sr);       // 8
    STREAM_get_word(&control_mask); // 10
    STREAM_get_word(&Mdb);
    STREAM_get_word(&releaseJtag);
    STREAM_get_word(&ulpDebug);
    // Write back Status Register
    WriteCpuRegXv2(0x0082, Sr);

    // Restore Watchdog Control Register
    WriteMemWordXv2(wdt_addr,wdt_value);

    /* Note 1411 */
    /* check if CPU is OFF, and decrement PC = PC-2 */
    if(Sr & CPUOFF)
    {
        lPc -= 2;
    }
    /* End of note 1411 */

    Mova  = 0x0080;
    Mova += (unsigned short)((lPc>>8) & 0x00000F00);
    Pc_l  = (unsigned short)((lPc & 0xFFFF));

    // restore Program Counter
    SetPcXv2(Mova, Pc_l);

    // prepare release
    if(Mdb)
    {
        cntrl_sig_16bit();
        SetReg_16Bits(0x0401);
        // preload the MDB before releasing the CPU
        data_16bit();
        IHIL_Tclk(1);
        SetReg_16Bits(Mdb);
        IHIL_Tclk(0);
        // release the busses - otherwise modules/flash can't control the MDB
        // = get out of DATA_* instruction
        addr_capture();
        // here one more clock cycle is required to advance the CPU
        // otherwise enabling the EEM can stop the device again
        {
            cntrl_sig_capture();
            if(SetReg_16Bits(0) & CNTRL_SIG_HALT)
            {
                cntrl_sig_16bit();
                SetReg_16Bits(0x0403);
            }
        }
        IHIL_Tclk(1);
    }
    else
    {
        IHIL_Tclk(1);
        cntrl_sig_16bit();
        SetReg_16Bits(0x0401);
        addr_capture();
        // here one more clock cycle is required to advance the CPU
        // otherwise enabling the EEM would stop the device again, because
        {
            cntrl_sig_capture();
            IHIL_Tclk(0);

            // shift out current control signal register value
            if(SetReg_16Bits(0) & CNTRL_SIG_HALT)
            {
                cntrl_sig_16bit();
                SetReg_16Bits(0x0403);
            }
            IHIL_Tclk(1);
        }
    }
    // now we can enable the EEM
    // --> this should work asynchronously
    eem_write_control();
    SetReg_16Bits(control_mask);

    // -------------------------Power mode handling start ----------------------
    if(ulpDebug)
    {
        // Set LPMx.5 settings
        EnableLpmx5();
    }
    else
    {
        unsigned short id = cntrl_sig_capture();
        DisableLpmx5();
        //Manually disable DBGJTAGON bit
        if(id == JTAGVERSION99)
        {
            test_reg_3V();
            SetReg_16Bits(SetReg_16Bits(0x0000) & ~DBGJTAGON);
        }
    }
    // -------------------------Power mode handling end ------------------------

    syncWithRunVarAddress = getTargetRunningVar();
    if(syncWithRunVarAddress)
    {
          *syncWithRunVarAddress = 0x0001;
    }
    // release target device from JTAG control
    cntrl_sig_release();

    if(releaseJtag)
    {
        IHIL_Close(); // release JTAG on go
    }
    return 0;
}
