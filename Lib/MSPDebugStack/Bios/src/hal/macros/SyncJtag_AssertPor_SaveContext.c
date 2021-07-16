/**
* \ingroup MODULMACROS
*
* \file SyncJtag_AssertPor_SaveContext.c
*
* \brief Sync with device, assert a Power On Reset and save processor context
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
#include "hal_ref.h"
#include "stream.h"
#include "EEM_defs.h"
#include "global_variables.h"

extern DeviceSettings deviceSettings;

/**
  SyncJtag_AssertPor_SaveContext
  inData:  <wdtAddr(16)> <wdtCtrl(16)>
  outData: <wdtCtrl(16)> <PC(32)> <SR(16)>
  wdtAddr: watchdog timer address
  wdtCtrl: watchdog timer control value
  PC: program counter register (R0)
  SR: status register (R2)
*/
HAL_FUNCTION(_hal_SyncJtag_AssertPor_SaveContext)
{
    unsigned short i = 0, lOut = 0, ctl_sync = 0;
    unsigned short MyOut[4];    //! Output
    unsigned short address;
    unsigned short wdtVal;
    unsigned short* syncWithRunVarAddress = 0;

    // Check input parameters before we proceed
    if(STREAM_get_word(&address) != 0)
    {
        return (HALERR_SYNC_JTAG_ASSERT_POR_NO_WDT_ADDRESS);
    }
    if(STREAM_get_word(&wdtVal) != 0)
    {
        return (HALERR_SYNC_JTAG_ASSERT_POR_NO_WDT_VALUE);
    }

    syncWithRunVarAddress = getTargetRunningVar();
    if(syncWithRunVarAddress)
    {
        *syncWithRunVarAddress = 0x0000;
    }

    // Sync the JTAG
    cntrl_sig_16bit();
    SetReg_16Bits(0x2401);

    if(cntrl_sig_capture() != JTAGVERSION)
    {
        return (HALERR_JTAG_VERSION_MISMATCH);
    }

    cntrl_sig_capture();
    lOut = SetReg_16Bits(0x0000);    // read control register once

    IHIL_Tclk(1);

    if(!(lOut & CNTRL_SIG_TCE))
    {
        // If the JTAG and CPU are not already synchronized ...
        // Initiate Jtag and CPU synchronization. Read/Write is under CPU control. Source TCLK via TDI.
        // Do not effect bits used by DTC (CPU_HALT, MCLKON).
        cntrl_sig_high_byte();
        SetReg_8Bits((CNTRL_SIG_TAGFUNCSAT | CNTRL_SIG_TCE1 | CNTRL_SIG_CPU) >> 8); // initiate CPU synchronization but release low byte of CNTRL sig register to CPU control

        // Address Force Sync special handling
        eem_data_exchange();               // read access to EEM General Clock Control Register (GCLKCTRL)
        SetReg_16Bits(MX_GCLKCTRL + MX_READ);
        lOut = SetReg_16Bits(0);                 // read the content of GCLKCNTRL into lOUt
        // Set Force Jtag Synchronization bit in Emex General Clock Control register.
        lOut |=  0x0040;                 // 0x0040 = FORCE_SYN in DLLv2
        eem_data_exchange();                // Stability improvement: should be possible to remove this, required only once at the beginning
        SetReg_16Bits(MX_GCLKCTRL + MX_WRITE);   // write access to EEM General Clock Control Register (GCLKCTRL)
        lOut = SetReg_16Bits(lOut);              // write into GCLKCNTRL
        // Reset Force Jtag Synchronization bit in Emex General Clock Control register.
        lOut &= ~0x0040;
        eem_data_exchange();                // Stability improvement: should be possible to remove this, required only once at the beginning
        SetReg_16Bits(MX_GCLKCTRL + MX_WRITE);   // write access to EEM General Clock Control Register (GCLKCTRL)
        lOut = SetReg_16Bits(lOut);             // write into GCLKCNTRL

        ctl_sync =  SyncJtag();

        if(!(ctl_sync & CNTRL_SIG_CPU_HALT))
        { // Synchronization failed!
            return (HALERR_SYNC_JTAG_ASSERT_POR_JTAG_TIMEOUT);
        }
    }// end of if(!(lOut & CNTRL_SIG_TCE))

    if(ctl_sync & CNTRL_SIG_CPU_HALT)
    {
        IHIL_Tclk(0);
        cntrl_sig_16bit();
        SetReg_16Bits(0x2401);
        IHIL_Tclk(1);
    }
    else
    {
        cntrl_sig_16bit();
        SetReg_16Bits(0x2401);
    }

    if (deviceSettings.assertBslValidBit)
    {
        // here we add bit de assert bit 7 in JTAG test reg to enalbe clocks again
        test_reg();
        lOut = SetReg_8Bits(0x00);
        lOut |= 0x80; //DE_ASSERT_BSL_VALID;
        test_reg();
        SetReg_8Bits(lOut); // Bit 7 is de asserted now
    }

    // execute a dummy instruction here
    data_16bit();
    IHIL_Tclk(1);                   // Stability improvement: should be possible to remove this TCLK is already 1
    SetReg_16Bits(0x4303);         // 0x4303 = NOP
    IHIL_Tclk(0);
    data_capture();
    IHIL_Tclk(1);

    // step until next instruction load boundary if not being already there
    if(instrLoad() != 0)
    {
        return (HALERR_INSTRUCTION_BOUNDARY_ERROR);
    }

    if (deviceSettings.clockControlType == GCC_EXTENDED)
    {
        // Perform the POR
        eem_data_exchange();
        SetReg_16Bits(MX_GENCNTRL + MX_WRITE);               // write access to EEM General Control Register (MX_GENCNTRL)
        SetReg_16Bits(EMU_FEAT_EN | EMU_CLK_EN | CLEAR_STOP | EEM_EN);   // write into MX_GENCNTRL

        eem_data_exchange(); // Stability improvement: should be possible to remove this, required only once at the beginning
        SetReg_16Bits(MX_GENCNTRL + MX_WRITE);               // write access to EEM General Control Register (MX_GENCNTRL)
        SetReg_16Bits(EMU_FEAT_EN | EMU_CLK_EN);         // write into MX_GENCNTRL
    }
    if (deviceSettings.clockControlType == GCC_STANDARD_I)
    {
        eem_data_exchange();
        SetReg_16Bits(MX_GENCNTRL + MX_WRITE);  // write access to EEM General Control Register (MX_GENCNTRL)
        SetReg_16Bits(EMU_FEAT_EN);             // write into MX_GENCNTRL
    }

    IHIL_Tclk(0);
    cntrl_sig_16bit();
    SetReg_16Bits(CNTRL_SIG_READ | CNTRL_SIG_TCE1 | CNTRL_SIG_PUC | CNTRL_SIG_TAGFUNCSAT); // Assert PUC
    IHIL_Tclk(1);
    cntrl_sig_16bit();
    SetReg_16Bits(CNTRL_SIG_READ | CNTRL_SIG_TCE1 | CNTRL_SIG_TAGFUNCSAT); // Negate PUC

    IHIL_Tclk(0);
    cntrl_sig_16bit();
    SetReg_16Bits(CNTRL_SIG_READ | CNTRL_SIG_TCE1 | CNTRL_SIG_PUC | CNTRL_SIG_TAGFUNCSAT); // Assert PUC
    IHIL_Tclk(1);
    cntrl_sig_16bit();
    SetReg_16Bits(CNTRL_SIG_READ | CNTRL_SIG_TCE1 | CNTRL_SIG_TAGFUNCSAT); // Negate PUC

    // Explicitly set TMR
    SetReg_16Bits(CNTRL_SIG_READ | CNTRL_SIG_TCE1); // Enable access to Flash registers

    flash_16bit_update();               // Disable flash test mode
    SetReg_16Bits(FLASH_SESEL1);     // Pulse TMR
    SetReg_16Bits(FLASH_SESEL1 | FLASH_TMR);
    SetReg_16Bits(FLASH_SESEL1);
    SetReg_16Bits(FLASH_SESEL1 | FLASH_TMR); // Set TMR to user mode

    cntrl_sig_high_byte();
    SetReg_8Bits((CNTRL_SIG_TAGFUNCSAT | CNTRL_SIG_TCE1) >> 8); // Disable access to Flash register

    // step until an appropriate instruction load boundary
    for(i = 0; i < 10; i++)
    {
      addr_capture();
        lOut = SetReg_16Bits(0x0000);
        if(lOut == 0xFFFE || lOut == 0x0F00)
        {
            break;
        }
        IHIL_TCLK();
    }
    if(i == 10)
    {
        return (HALERR_INSTRUCTION_BOUNDARY_ERROR);
    }
    IHIL_TCLK();
    IHIL_TCLK();

    IHIL_Tclk(0);
    addr_capture();
    SetReg_16Bits(0x0000);
    IHIL_Tclk(1);

    // step until next instruction load boundary if not being already there

    if(instrLoad() != 0)
    {
        return (HALERR_INSTRUCTION_BOUNDARY_ERROR);
    }

    // Hold Watchdog
    MyOut[0] = ReadMemWord(address); // safe WDT value
    wdtVal |= (MyOut[0] & 0xFF); // set original bits in addition to stop bit
    WriteMemWord(address, wdtVal);

    // read MAB = PC here
    addr_capture();
    MyOut[1] = SetReg_16Bits(0);
    MyOut[2] = 0; // high PC always 0 for MSP430 architecture

    // set PC to a save address pointing to ROM to avoid RAM corruption on certain devices
    SetPc(ROM_ADDR);

    // read status register
    MyOut[3] = ReadCpuReg(2);

    // return output
    STREAM_put_bytes((unsigned char*)MyOut,8);

    return(0);
}

