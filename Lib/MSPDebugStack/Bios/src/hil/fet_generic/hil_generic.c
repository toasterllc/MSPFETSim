/*
 * hil_generic.c
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
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

#include "hilFpgaAccess.h"
#include "archFpga.h"
#include "arch.h"
#include "JTAG_defs.h"
#include "hilDelays.h"

static unsigned short prevInstruction = 0;

#pragma optimize=low
void _hil_generic_ConfigureSpeed(unsigned short speed)
{
    unsigned short tckFreq, tclkFreq;

    switch(speed)
        {
            case SBW100KHz:
                {
                    tckFreq = 298;
                    tclkFreq = 75;
                    break;
                }
            case SBW200KHz:
                {
                    tckFreq = 350;
                    tclkFreq = 75;
                    break;
                }
            case SBW400KHz:
                {
                    tckFreq = 137;
                    tclkFreq = 30;
                    break;
                }
            case SBW600KHz:
                {
                    tckFreq = 88;
                    tclkFreq = 21;
                    break;
                }
            case SBW1200KHz:
                {
                    tckFreq = 48;
                    tclkFreq = 12;
                    break;
                }
            case JTAG250KHz:
                {
                    tckFreq = 238;
                    tclkFreq = 238;
                    break;
                }
            case JTAG500KHz:
                {
                    tckFreq = 118;
                    tclkFreq = 28;
                    break;
                }
             case JTAG750KHz:
                {
                    tckFreq = 88;
                    tclkFreq = 21;
                    break;
                }
            case JTAG1MHz:
                {
                    tckFreq = 58;
                    tclkFreq = 14;
                    break;
                }
            case JTAG2MHz:
                {
                    tckFreq = 28;
                    tclkFreq = 7;
                    break;
                }
            case JTAG4MHz:
                {
                    tckFreq = 13;
                    tclkFreq = 4;
                    break;
                }
            case JTAG8MHz:
                {   // 7,5 MHz
                    tckFreq = 5;
                    tclkFreq = 2;
                    break;
                }
            case JTAG15MHz:
                {
                    tckFreq = 3;
                    tclkFreq = 3;
                    break;
                }
            default:
                {
                    // 500 kHz, but this should never happen
                    tckFreq = 118;
                    tclkFreq = 118;
                    break;
                }
        }
    hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_TEST_CLK_FREQUENCY, tckFreq);
    hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_TCLK_CLK_FREQUENCY, tclkFreq);
}

void initGeneric()
{
    prevInstruction = 0;
}

unsigned char _hil_generic_GetPrevInstruction()
{
    return prevInstruction;
}

unsigned char _hil_generic_Instr(unsigned char instruction)
{
    unsigned short retVal;

    hil_fpga_write_cmd_data0(FPGA_CMD_IR8_RD, instruction);
    hil_fpga_read_data1(1, &retVal);
    prevInstruction = instruction;

    return ((unsigned char)retVal);
}

unsigned char _hil_generic_SetReg_XBits08(unsigned char data)
{
    unsigned short retVal;

    hil_fpga_write_cmd_data0(FPGA_CMD_DR8_RD, data);
    hil_fpga_read_data1(1, &retVal);

    return ((unsigned char)retVal);
}
unsigned short _hil_generic_SetReg_XBits16(unsigned short data)
{
    unsigned short retVal;

    hil_fpga_write_cmd_data0_data1(FPGA_CMD_DRX_RD, 16, data);
    hil_fpga_read_data1(1, &retVal);

    return retVal;
}

unsigned long _hil_generic_SetReg_XBits20(unsigned long data)
{
    unsigned long retVal;

    hil_fpga_write_cmd_data0_data1_count(FPGA_CMD_DRX_RD, 20, (unsigned short*)&data, 2);
    hil_fpga_read_data1(2, (unsigned short*)&retVal);

    return (retVal >> 4) | ((retVal & 0xF) << 16);
}

// -----------------------------------------------------------------------------
unsigned long _hil_generic_SetReg_XBits32(unsigned long data)
{
    unsigned long retVal;

    hil_fpga_write_cmd_data0_data1_count(FPGA_CMD_DRX_RD, 32, (unsigned short*)&data, 2);
    hil_fpga_read_data1(2, (unsigned short*)&retVal);
    return retVal;
}

// -----------------------------------------------------------------------------
unsigned long long _hil_generic_SetReg_XBits64(unsigned long long data)
{
    unsigned long long retVal;

    hil_fpga_write_cmd_data0_data1_count(FPGA_CMD_DRX_RD, 64, (unsigned short*)&data, 4);
    hil_fpga_read_data1(4, (unsigned short*)&retVal);

    return retVal;
}

//! \brief This function executes an SBW2 64BIT Data SHIFT (DR-SHIFT) in the case,
//! that the first 8 bits show a valid data capture from the JSTATE register. In case
//! of no valid capture the shift is ended after the first 8 bits. Timeout could be
//! used to set the function run count.
//! \param[in]  Data to be shifted into target device
//! \return Value shifted out of target device JTAG module
unsigned long long _hil_generic_XBits8_64(unsigned long long Data, unsigned short loopCount,unsigned short PG)
{
   unsigned long long TDOvalue = 0x0;
    unsigned short timeout = loopCount;
    unsigned char DataState = 0;
    unsigned short currentDeviceState = 0;
    unsigned short syncBrokenCount = loopCount/2;

    do
    {
        TDOvalue = (_hil_generic_SetReg_XBits64(Data));
        // Mask out all not needed bits for device state detection
        currentDeviceState = ((TDOvalue >> 56) & JSTATE_FLOW_CONTROL_BITS);

        // check, if BIT[63, 62, 61, 60, 59  58, 57, 56, ] & 0x4 (BP_HIT) == TRUE
        if(currentDeviceState & JSTATE_BP_HIT)
        {
            // reload Jstate IR
            _hil_generic_Instr(IR_JSTATE_ID);
            DataState = VALID_DATA;
            return(TDOvalue);
        }

        // check, if BIT[63, 62, 61, 60, 59  58, 57, 56] (AM Sync. ongoing) == 0x83
        else if(currentDeviceState  == JSTATE_SYNC_ONGOING)
        {
            DataState = SYNC_ONGOING;
        }

        //check, if BIT[63, 62, 61, 60, 59  58, 57, 56] & 0x40 (Locked State) == 0x40
        else if((currentDeviceState & JSTATE_LOCKED_STATE) == JSTATE_LOCKED_STATE &&
                (currentDeviceState & JSTATE_LPM_X_FIVE) != JSTATE_LPM_X_FIVE)
        {
            DataState = JTAG_LOCKED;
            return(TDOvalue);
        }

        //check, if BIT[63, 62, 61, 60, 59  58, 57, 56] ( Invalid LPM) == 0x81)
        else if(currentDeviceState == JSTATE_INVALID_STATE)
        {
            // reload Jstate IR
            _hil_generic_Instr(IR_JSTATE_ID);
            DataState = INVALID_DATA;
        }
        /*PG2.0 && PG2.1 frozen Sync detection*/
       else if (((currentDeviceState & JSTATE_SYNC_BROKEN_MASK) ==  JSTATE_SYNC_BROKEN_MCLK)
                    ||((currentDeviceState & JSTATE_SYNC_BROKEN_MASK) ==  JSTATE_SYNC_BROKEN_MCLK_PGACT)
                    ||((currentDeviceState & JSTATE_SYNC_BROKEN_MASK) ==  JSTATE_SYNC_BROKEN_PGACT))
        {
           if(syncBrokenCount > 0 && PG >= 0x21)

           { // Only working for PG2.1, do not harm if executed on PG2.0 ? but will not create any effect only consume time
                unsigned long current3VtestReg = 0;

                // read current 3VtestReg value
                _hil_generic_Instr(IR_TEST_REG);
                current3VtestReg = _hil_generic_SetReg_XBits32(0);
                // set bit 25 high to rest sync
                current3VtestReg |= 0x2000000;
                _hil_generic_SetReg_XBits32(current3VtestReg);
                // set bit 25 low reset sync done
                current3VtestReg &= ~0x2000000;
                _hil_generic_SetReg_XBits32(current3VtestReg);

                _hil_generic_Instr(IR_JSTATE_ID);
                syncBrokenCount --;
                timeout = timeout + 5;
            }
            else
            {
                DataState = VALID_DATA;
                return(TDOvalue);
            }
        }
        // device is not in LPMx or AC read out mode just restart the shift but do not reload the JState IR
        else if(currentDeviceState != JSTATE_VALID_CAPTURE
                &&  currentDeviceState !=   JSTATE_LPM_ONE_TWO
                &&  currentDeviceState !=   JSTATE_LPM_THREE_FOUR
                && (currentDeviceState &    JSTATE_LPM_X_FIVE) != JSTATE_LPM_X_FIVE)
        {
            DataState = INVALID_DATA;
        }
        else
        {
            DataState = VALID_DATA;
        }

        if(!timeout)
        {
            return TDOvalue;
        }
        timeout--;
    }
    while(DataState == INVALID_DATA || DataState == SYNC_ONGOING);

    if(      currentDeviceState == JSTATE_LPM_ONE_TWO
        ||   currentDeviceState == JSTATE_LPM_THREE_FOUR
        ||   currentDeviceState == 0x00
        ||   currentDeviceState == 0x02)
    {
        // reload Jstate IR
        _hil_generic_Instr(IR_JSTATE_ID);
    }
    return(TDOvalue);
}


// -----------------------------------------------------------------------------
void _hil_generic_Tclk(unsigned char state)
{
    if(state)
    {
        hil_fpga_write_cmd_data0(FPGA_CMD_CFG, REG_TCLKset1);
    }
    else
    {
        hil_fpga_write_cmd_data0(FPGA_CMD_CFG, REG_TCLKset0);
    }
}
