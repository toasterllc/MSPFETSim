/**
* \ingroup MODULMACROSXV2
*
* \file SyncJtag_AssertPor_SaveContextXv2.c
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

#include "arch.h"
#include "edt.h"
#include "hal.h"
#include "modules.h"
#include "stream.h"
#include "EEM_defs.h"
#include "hw_compiler_specific.h"
#include "HalGlobalVars.h"
#include "error_def.h"


extern unsigned long _hal_mclkCntrl0;
unsigned char mclk_modules[32];

/**
  SyncJtag_AssertPor_SaveContextXv2
  inData:  <wdtAddr(16)> <wdtCtrl(16)>
  outData: <wdtCtrl(16)> <PC(32)> <SR(16)>
  wdtAddr: watchdog timer address
  wdtCtrl: watchdog timer control value
  PC: program counter register (R0)
  SR: status register (R2)
*/

extern DevicePowerSettings devicePowerSettings;

HAL_FUNCTION(_hal_SyncJtag_AssertPor_SaveContextXv2)
{
    unsigned short MyOut[4];
    int i;
    unsigned short address;
    unsigned short wdtVal;
    unsigned short id = cntrl_sig_capture();
    unsigned long lOut_long = 0;

    // -------------------------Power mode handling start ----------------------
    //Disable Lpmx5 and power gaiting settings
    if( id == JTAGVERSION99)
    {
        test_reg_3V();
        SetReg_16Bits(0x40A0);
        test_reg();
        SetReg_32Bits(0x00010000);
    }
    // -------------------------Power mode handling end ------------------------
    // enable clock control before sync
    // switch all functional clocks to JCK = 1 and stop them
    eem_data_exchange32();
    SetReg_32Bits(GENCLKCTRL + WRITE);
    SetReg_32Bits(MCLK_SEL3 + SMCLK_SEL3 + ACLK_SEL3 + STOP_MCLK + STOP_SMCLK + STOP_ACLK);
    // enable Emualtion Clocks
    eem_write_control();
    SetReg_16Bits(EMU_CLK_EN + EEM_EN);

    cntrl_sig_16bit();
    // release RW and BYTE control signals in low byte, set TCE1 & CPUSUSP(!!) & RW
    SetReg_16Bits(0x1501);

    if(wait_for_synch())
    {
        // provide one more clock to empty the pipe
        IHIL_TCLK();

        cntrl_sig_16bit();
        // release CPUFLUSH(=CPUSUSP) signal and apply POR signal
        SetReg_16Bits(0x0C01);
        IHIL_Delay_1ms(40);

        // release POR signal again
        SetReg_16Bits(0x0401); // disable fetch of CPU // changed from 401 to 501

        if(id == JTAGVERSION91 || id == JTAGVERSION99 || id == JTAGVERSION98)
        {   // Force PC so safe value memory location, JMP $
            data_16bit();
            IHIL_TCLK();
            IHIL_TCLK();
            SetReg_16Bits(SAFE_PC_ADDRESS);
            // drive safe address into pc end
            IHIL_TCLK();

            if(id == JTAGVERSION91)
            {
                IHIL_TCLK();
            }

            data_capture();
        }
        else
        {
            IHIL_TCLK();
            IHIL_TCLK();
            IHIL_TCLK();
        }

        // TWO more to release CPU internal POR delay signals
        IHIL_TCLK();
        IHIL_TCLK();

        // set CPUFLUSH signal
        cntrl_sig_16bit();
        SetReg_16Bits(0x0501);
        IHIL_TCLK();

        // set EEM FEATURE enable now!!!
        eem_write_control();
        SetReg_16Bits(EMU_FEAT_EN + EMU_CLK_EN + CLEAR_STOP);

        // Check that sequence exits on Init State
        cntrl_sig_capture();
        SetReg_16Bits(0x0000);
    //    lout == 0x0301,0x3fd3
        // hold Watchdog Timer
        STREAM_get_word(&address);
        STREAM_get_word(&wdtVal);

        MyOut[0] = ReadMemWordXv2(address);
        wdtVal |= (MyOut[0] & 0xFF); // set original bits in addition to stop bit
        WriteMemWordXv2(address, wdtVal);

        // Capture MAB - the actual PC value is (MAB - 4)
        addr_capture();
        lOut_long = SetReg_20Bits(0);
        /*****************************************/
        /* Note 1495, 1637 special handling      */
        /*****************************************/
        if(((lOut_long & 0xFFFF) == 0xFFFE) || (id == JTAGVERSION91) || (id == JTAGVERSION99) || (id == JTAGVERSION98))
        {
          MyOut[1] = ReadMemWordXv2(0xFFFE);
          MyOut[2] = 0;
        }
        /*********************************/
        else
        {
          lOut_long -= 4;
          MyOut[1] = (unsigned short)(lOut_long & 0xFFFF);
          MyOut[2] = (unsigned short)(lOut_long >>16);
        }
        // Status Register should be always 0 after a POR
        MyOut[3] = 0;
        STREAM_discard_bytes(1);
        for(i=0; i<32; i++)
        {
          unsigned short v;
          STREAM_get_byte(&mclk_modules[i]);

          if(mclk_modules[i] != 0)
          {
            // check if module clock control is enabled for corresponding module
            if(_hal_mclkCntrl0 & ((unsigned long) 0x00000001 << i))
              v = 1;
            else
              v = 0;
            WriteMemWordXv2(ETKEYSEL, ETKEY + mclk_modules[i]);
            WriteMemWordXv2(ETCLKSEL, v);
          }
        }
        // switch back system clocks to original clock source but keep them stopped
        eem_data_exchange32();
        SetReg_32Bits(GENCLKCTRL + WRITE);
        SetReg_32Bits(MCLK_SEL0 + SMCLK_SEL0 + ACLK_SEL0 + STOP_MCLK + STOP_SMCLK + STOP_ACLK);

        // reset Vacant Memory Interrupt Flag inside SFRIFG1
        if(id == JTAGVERSION91)
        {
            volatile unsigned short specialFunc = ReadMemWordXv2(0x0102);
            if(specialFunc & 0x8)
            {
                SetPcXv2(0x80, SAFE_PC_ADDRESS);
                cntrl_sig_16bit();
                SetReg_16Bits(0x0501);
                IHIL_Tclk(1);
                addr_capture();

                specialFunc &= ~0x8;
                WriteMemWordXv2(0x0102, specialFunc);
            }
        }

        STREAM_put_bytes((unsigned char*)MyOut,8);

        return 0; // retrun status OK
    }
    else
    {
        return HALERR_UNDEFINED_ERROR; // return errer in case if syc fails
    }
}
