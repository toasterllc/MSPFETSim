/**
* \ingroup MODULHAL
*
* \file edt.c
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

#include "edt.h"

unsigned long i_ReadJmbOut()
{
    unsigned short sJMBOUT0 = 0, sJMBOUT1 = 0, sJMBINCTL = 0;
    jmb_exchange();
    if(SetReg_16Bits(sJMBINCTL) & OUT1RDY)
    {
        sJMBINCTL |= JMB32B + OUTREQ;
        SetReg_16Bits(sJMBINCTL);
        sJMBOUT0 = SetReg_16Bits(0);
        sJMBOUT1 = SetReg_16Bits(0);
        return ((unsigned long)sJMBOUT1<<16) + sJMBOUT0;
    }
    return 0;
}

unsigned short i_ReadJmbOut16()
{
    unsigned short sJMBINCTL = 0;
    jmb_exchange();
    if(SetReg_16Bits(sJMBINCTL) & OUT0RDY)
    {
        sJMBINCTL |=  OUTREQ;
        SetReg_16Bits(sJMBINCTL);
        return SetReg_16Bits(0);
    }
    return 0;
}

short i_WriteJmbIn(unsigned short data)
{
    volatile unsigned short sJMBINCTL = INREQ, sJMBIN0 = (unsigned short)(data & 0x0000FFFF);
    volatile unsigned short lOut = 0;
    volatile unsigned long Timeout = 0;

    jmb_exchange();
    do
    {
        lOut = SetReg_16Bits(0x0000);
        if(Timeout++ >= 3000)
        {
            return 1;
        }
    }
    while(!(lOut & IN0RDY) && Timeout < 3000);

    if(Timeout < 3000)
    {
        SetReg_16Bits(sJMBINCTL);
        SetReg_16Bits(sJMBIN0);
    }
    return 0;
}

short i_WriteJmbIn32(unsigned short dataA, unsigned short dataB)
{
    volatile unsigned short sJMBINCTL =  JMB32B | INREQ;
    volatile unsigned short sJMBIN0 = 0 ,sJMBIN1 = 0, lOut = 0;
    volatile unsigned long Timeout = 0;

    sJMBIN0 = (unsigned short)(dataA & 0x0000FFFF);
    sJMBIN1 = (unsigned short)(dataB & 0x0000FFFF);

    jmb_exchange();
    do
    {
        lOut = SetReg_16Bits(0x0000);
        if(Timeout++ >= 3000)
        {
            return 1;
        }
    }
    while(!(lOut & IN1RDY) && Timeout < 3000);
    if(Timeout < 3000)
    {
        sJMBINCTL = 0x11;
        SetReg_16Bits(sJMBINCTL);
        SetReg_16Bits(sJMBIN0);
        SetReg_16Bits(sJMBIN1);
    }
    return 0;
}

short checkWakeup()
{
    unsigned long UNLOCK_ADDRESS = 0xE0044000;
    unsigned long SYS_SYSTEM_STAT = 0xE0044020;
    unsigned long PWD = 0x695A;
    unsigned short DBG_SEC_ACT_MASK = 0x8;

    unsigned long val = 0;
    unsigned short  timeout = 100;

    //unlock Sys master
    IHIL_Write_Read_Mem_Ap(0, UNLOCK_ADDRESS, &PWD, WRITE);

    do
    {
        IHIL_Write_Read_Mem_Ap(0, SYS_SYSTEM_STAT, &val, READ);
        timeout--;
    }while((val & DBG_SEC_ACT_MASK) && --timeout);

    if(!timeout)
    {
        return -1;
    }
    return 0;
}

short powerUpArm()
{
    unsigned long val = 0;
    unsigned short  timeout = 200;

    IHIL_Write_Read_Dp(DP_CTRL_STAT, &val, READ);
    // Request system and debug power up
    val |= DP_CTRL_STAT_CSYSPWRUPREQ | DP_CTRL_STAT_CDBGPWRUPREQ;
    IHIL_Write_Read_Dp(DP_CTRL_STAT, &val, WRITE);
    // Wait for acknowledge
    do
    {
        IHIL_Write_Read_Dp(DP_CTRL_STAT, &val, READ);
        timeout--;
    } while(((val & (DP_CTRL_STAT_CSYSPWRUPACK | DP_CTRL_STAT_CDBGPWRUPACK)) !=
                   (DP_CTRL_STAT_CSYSPWRUPACK | DP_CTRL_STAT_CDBGPWRUPACK)) && timeout);

    if(!timeout)
    {
        return -1;
    }
    return 0;
}
