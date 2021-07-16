/**
* \ingroup MODULMACROS
*
* \file MagicPattern.c
*
* \brief Get control over the device
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
#include "JtagId.h"
#ifdef DB_PRINT
#include "debug.h"
#endif

/**
  MagicPattern
  This function will try to get control over the device. It will try to reset and
  stop the device.
  It will automatically switch between the different JTAG protocols also it handles
  the LPMx.5

  inData:  void
  outData: <chainLen(8)>
  outData: <jtagId(8)>

  protocol: 0 = 4-wire Jtag
  jtagId: the Jtag identifier value
*/

#define BOOT_DATA_CRC_WRONG 0xC3C3

#ifdef uController_uif
extern void JSBW_EntrySequences(unsigned char states);
extern void JSBW_TapReset(void);
extern void JSBW_MagicPattern(void);
extern void JSBW_JtagUnlock(void);
extern void jRelease(void);
#endif

#define BOOT_DATA_CRC_WRONG 0xC3C3

unsigned short magicPattern = 0;

#ifdef MSP430_UIF
    short magicPatternJsbw2();
#endif

#pragma optimize = medium
HAL_FUNCTION(_hal_MagicPattern)
{
    unsigned short id = 0;
    unsigned char chainLen = 1;
    unsigned short protocol = SPYBIWIRE;

    STREAM_get_word(&protocol);

#ifdef eZ_FET
    if(protocol !=  SPYBIWIRE)
    {
       return HALERR_MAGIC_PATTERN;
    }
#endif

#ifdef MSP430_UIF
    if(protocol == SPYBIWIRE_MSP_FET)
    {
        return magicPatternJsbw2();
    }
#endif

    IHIL_SetProtocol(protocol);
#if defined(eZ_FET) || defined(MSP_FET)
    {
        HilInitGetEdtDistinctFunc hilEdtDis = (HilInitGetEdtDistinctFunc)0x1880;
        hilEdtDis(&_edt_Distinct_Methods);
    }
#endif
    // run entry sequnce but pull rst low during the sequence to wake-up the device

    IHIL_Close();

    _edt_Common_Methods.regulateVcc();
    
    IHIL_Open(RSTLOW);
    IHIL_TapReset();
    IHIL_CheckJtagFuse();
    // put in magic pattern to stop user code execution

    if(i_WriteJmbIn(MAGIC_PATTERN) == 1)
    {
        return (HALERR_JTAG_MAILBOX_IN_TIMOUT);
    }

    // run entry sequnce but pull rst high during the sequence
    IHIL_Open(RSTHIGH);
    IHIL_TapReset();
    IHIL_CheckJtagFuse();

    // Corrupted CRC handling - start --------------------------------------
    // read JTAG mailbox to see if CRC of Bootdata is damaged.
    if(i_ReadJmbOut16() == BOOT_DATA_CRC_WRONG)
    {
        return (HALERR_MAGIC_PATTERN_BOOT_DATA_CRC_WRONG);
    }
    // Corrupted CRC handling - end ----------------------------------------

    id = cntrl_sig_capture() ;
    if (jtagIdIsXv2(id))
    {
        //Disable Lpmx5 and power gaiting settings
        if(id == JTAGVERSION98)
        {
            // just disable JTAG io lock
            test_reg_3V();
            SetReg_16Bits(0x4020);
        }
        if(id == JTAGVERSION99)
        {
            test_reg_3V();
            SetReg_16Bits(0x40A0);
            test_reg();
            SetReg_32Bits(0x00010000);
        }
        STREAM_put_byte((unsigned char)chainLen);
        STREAM_put_byte((unsigned char)id);

#ifdef MSP_FET
        if (protocol == SPYBIWIRE_MSP_FET)
        {
            HilInitGetEdtDistinctFunc hilEdtDis = (HilInitGetEdtDistinctFunc)0x1880;
            protocol = SPYBIWIREJTAG;
            IHIL_SetProtocol(protocol);
            hilEdtDis(&_edt_Distinct_Methods);
            IHIL_Open(RSTHIGH);
            IHIL_TapReset();
            IHIL_CheckJtagFuse();
        }
#endif
        STREAM_put_byte((unsigned char)protocol);

        return 0;
    }
    return (HALERR_MAGIC_PATTERN);
}

#ifdef MSP430_UIF

// Force wakeup in SBW4 mode for LPMx.5 because JTAG pins are locked by LPMx.5
// This case happens if the device is woken up, but goes into LPMx.5 very quickly
// and the device wasn't under JTAG control from a previous debug session
short magicPatternJsbw2()
{
    unsigned short i = 0;
    unsigned short id = 0;
    unsigned char chainLen = 1;
    unsigned short protocol = SPYBIWIREJTAG;
    IHIL_SetProtocol(protocol);

    while( i < 3)
    {
        //  feet in JSBW magic pattern Use real RST and TST pin of TOOL
        JSBW_EntrySequences(0);
        JSBW_TapReset();
        JSBW_MagicPattern();
        IHIL_Delay_1ms(10);
        jRelease();

        // disable JTAG lock
        JSBW_EntrySequences(2);
        JSBW_TapReset();
        JSBW_JtagUnlock();
        IHIL_Delay_1ms(10);

        // reconnect in normal JTAG 4 wire mode
        IHIL_Open(RSTHIGH);
        IHIL_TapReset();
        IHIL_CheckJtagFuse();

        id = cntrl_sig_capture();
        if (jtagIdIsXv2(id))
        {
            //Disable Lpmx5 and power gaiting settings
            if(id == JTAGVERSION98)
            {
                // just disable JTAG i lock
                test_reg_3V();
                SetReg_16Bits(0x4020);
            }
            if(id == JTAGVERSION99)
            {
                test_reg_3V();
                SetReg_16Bits(0x40A0);
                test_reg();
                SetReg_32Bits(0x00010000);
            }
            STREAM_put_byte((unsigned char)chainLen);
            STREAM_put_byte((unsigned char)id);
            STREAM_put_byte((unsigned char)protocol);

            return 0;
        }
        i++;
    }
    return (HALERR_MAGIC_PATTERN);
}
#endif
