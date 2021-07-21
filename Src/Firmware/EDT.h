#pragma once
#include "HIL.h"

uint32_t i_ReadJmbOut()
{
    uint16_t sJMBOUT0 = 0, sJMBOUT1 = 0, sJMBINCTL = 0;
    jmb_exchange();
    if(SetReg_16Bits(sJMBINCTL) & OUT1RDY)
    {
        sJMBINCTL |= JMB32B + OUTREQ;
        SetReg_16Bits(sJMBINCTL);
        sJMBOUT0 = SetReg_16Bits(0);
        sJMBOUT1 = SetReg_16Bits(0);
        return ((uint32_t)sJMBOUT1<<16) + sJMBOUT0;
    }
    return 0;
}

uint16_t i_ReadJmbOut16()
{
    uint16_t sJMBINCTL = 0;
    jmb_exchange();
    if(SetReg_16Bits(sJMBINCTL) & OUT0RDY)
    {
        sJMBINCTL |=  OUTREQ;
        SetReg_16Bits(sJMBINCTL);
        return SetReg_16Bits(0);
    }
    return 0;
}

int16_t i_WriteJmbIn(uint16_t data)
{
    volatile uint16_t sJMBINCTL = INREQ, sJMBIN0 = (uint16_t)(data & 0x0000FFFF);
    volatile uint16_t lOut = 0;
    volatile uint32_t Timeout = 0;

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

int16_t i_WriteJmbIn32(uint16_t dataA, uint16_t dataB)
{
    volatile uint16_t sJMBINCTL =  JMB32B | INREQ;
    volatile uint16_t sJMBIN0 = 0 ,sJMBIN1 = 0, lOut = 0;
    volatile uint32_t Timeout = 0;

    sJMBIN0 = (uint16_t)(dataA & 0x0000FFFF);
    sJMBIN1 = (uint16_t)(dataB & 0x0000FFFF);

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

int16_t checkWakeup()
{
    uint32_t UNLOCK_ADDRESS = 0xE0044000;
    uint32_t SYS_SYSTEM_STAT = 0xE0044020;
    uint32_t PWD = 0x695A;
    uint16_t DBG_SEC_ACT_MASK = 0x8;

    uint32_t val = 0;
    uint16_t  timeout = 100;

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

int16_t powerUpArm()
{
    uint32_t val = 0;
    uint16_t  timeout = 200;

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
