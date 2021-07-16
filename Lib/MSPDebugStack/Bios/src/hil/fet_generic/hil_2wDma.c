/*
 * hil_2wDma.c
 *
 * <FILEBRIEF>
 *
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

//! \ingroup MODULHIL
//! \file hil_2wDma.c
//! \brief
//!

#include "hw_compiler_specific.h"
#include "arch.h"
#include "hilDelays.h"
#include "JTAG_defs.h"

#ifdef MSP_FET
    #include "hilFpgaAccess.h"
#endif

static struct jtag _Jtag = {0};
unsigned char tdo_bitDma = 0;

#ifdef MSP_FET
    unsigned short protocol_id = 0;
#endif

#define SBW_DELAY   { _NOP();_NOP();_NOP();/*_NOP();_NOP();_NOP();*/}

extern void testVpp(unsigned char mode);
extern void setVpp(long voltage);
extern void TCLKset1();
extern void TCLKset0();
unsigned long _hil_2w_SetReg_XBits32_Dma(unsigned long data);

//#pragma inline=forced
#pragma optimize = low
void TMSH_DMA()
{
    /*_DINT();*/ (*_Jtag.Out) |=  _Jtag.RST; (*_Jtag.Out) &= ~_Jtag.TST; SBW_DELAY; (*_Jtag.Out) |= _Jtag.TST; /*_EINT();*/ // TMS = 1
}

#pragma optimize = low
void TMSL_DMA()
{
    /*_DINT();*/ (*_Jtag.Out) &= ~_Jtag.RST;(*_Jtag.Out) &= ~_Jtag.TST; SBW_DELAY; (*_Jtag.Out) |= _Jtag.TST; /*_EINT();*/  // TMS = 0
}

#pragma optimize = low
void TMSLDH_DMA()
{
    /*_DINT();*/ (*_Jtag.Out) &= ~_Jtag.RST;(*_Jtag.Out) &= ~_Jtag.TST; SBW_DELAY; (*_Jtag.Out) |= _Jtag.RST; (*_Jtag.Out) |= _Jtag.TST; /*_EINT();*/ // TMS = 0, then TCLK immediately = 1
}

#pragma optimize = low
void TDIH_DMA()
{
  /*_DINT();*/ (*_Jtag.Out) |=  _Jtag.RST; (*_Jtag.Out) &= ~_Jtag.TST; SBW_DELAY; (*_Jtag.Out) |= _Jtag.TST;  /*_EINT();*/ // TDI = 1
}

#pragma optimize = low
void TDIL_DMA()
{
    /*_DINT();*/ (*_Jtag.Out) &= ~_Jtag.RST; (*_Jtag.Out) &= ~_Jtag.TST; SBW_DELAY; (*_Jtag.Out) |= _Jtag.TST; /*_EINT();*/
}

#pragma inline=forced
void TDOsbwDma()
{
    _DINT_FET(); (*_Jtag.DIRECTION) &= ~_Jtag.RST; (*_Jtag.Out) &= ~_Jtag.TST; SBW_DELAY; (*_Jtag.Out) |= _Jtag.TST; SBW_DELAY;SBW_DELAY;(*_Jtag.DIRECTION) |= _Jtag.RST;_EINT_FET();// TDO cycle without reading TDO
}
#pragma inline=forced
void TDOsbwFuse()
{
   (*_Jtag.DIRECTION) &= ~_Jtag.RST; (*_Jtag.Out) &= ~_Jtag.TST; SBW_DELAY; (*_Jtag.Out) |= _Jtag.TST; SBW_DELAY;SBW_DELAY;SBW_DELAY;(*_Jtag.DIRECTION) |= _Jtag.RST;// TDO cycle without reading TDO
}
#pragma inline=forced
void TDO_RD_FUSE()
{
    (*_Jtag.DIRECTION) &= ~_Jtag.RST; (*_Jtag.Out) &= ~_Jtag.TST; SBW_DELAY; tdo_bitDma = (*_Jtag.In);  SBW_DELAY;SBW_DELAY;SBW_DELAY;(*_Jtag.Out) |= _Jtag.TST; ; (*_Jtag.DIRECTION) |= _Jtag.RST;  // TDO cycle with TDO read
}
#pragma inline=forced
void TDO_RD_DMA()
{
    _DINT_FET(); (*_Jtag.DIRECTION) &= ~_Jtag.RST; (*_Jtag.Out) &= ~_Jtag.TST; SBW_DELAY; tdo_bitDma = (*_Jtag.In);  SBW_DELAY;SBW_DELAY;(*_Jtag.Out) |= _Jtag.TST; ; (*_Jtag.DIRECTION) |= _Jtag.RST; _EINT_FET();  // TDO cycle with TDO read
}

unsigned char DMA_TMSH_TDIH[84] = {0};

unsigned char DMA_TMSH_TDIL[84] = {0};

unsigned char DMA_TMSL_TDIH[84] = {0};

unsigned char DMA_TMSL_TDIL[84] = {0};

unsigned char TCLK_savedDma;
unsigned char current_Instr;
unsigned char prevInstruction;
void _hil_2w_ConfigureSpeed_Dma(unsigned short speed);


void initJtagSbw2Dma(struct jtag tmp)
{
    _Jtag = tmp;
    TCLK_savedDma =0;
    current_Instr = 0;
    prevInstruction = 0;

#ifdef MSP_FET
    protocol_id = 0;
#endif
}

#ifdef MSP_FET
void setProtocolSbw2Dma(unsigned short id)
{
    protocol_id = id;
}
#endif

unsigned char _hil_2w_GetPrevInstruction_Dma()
{
    return prevInstruction;
}

#pragma inline=forced
void DMA1sbw(void)
{
    DMA1CTL |= DMAEN;
    DMA1CTL |= DMAREQ;    // DMA1 FIRE!!!
    TDOsbwDma();
}

#pragma inline=forced
void DMA2sbw(void)
{
    DMA2CTL |= DMAEN;
    DMA2CTL |= DMAREQ;    // DMA2 FIRE!!!
    TDOsbwDma();
}

#pragma inline=forced
void restoreTCLK(void)
{
    if (TCLK_savedDma & _Jtag.RST)
    {
        DMA1SA = (unsigned char*)DMA_TMSH_TDIH;
        DMA1sbw();
        DMA1SA = (unsigned char*) DMA_TMSL_TDIH;
        DMA1sbw();
    }
    else
    {
        DMA1SA = (unsigned char*)DMA_TMSH_TDIL;
        DMA1sbw();
        // TMSL_TDIL is preloaded;
        DMA2sbw();
    }
}

#pragma inline=forced
unsigned long long sbw_ShiftDma(unsigned long long Data, unsigned short Bits)
{
    unsigned long long TDOvalue = 0x0000000000000000;
    unsigned long long MSB = 0x0000000000000000;

    switch(Bits)
    {
        case F_BYTE: MSB = 0x00000080;
            break;
        case F_WORD: MSB = 0x00008000;
            break;
        case F_ADDR: MSB = 0x00080000;
            break;
        case F_LONG: MSB = 0x80000000;
            break;
        case F_LONG_LONG: MSB = 0x8000000000000000;
            break;
        default: // this is an unsupported format, function will just return 0
            return TDOvalue;
    }
    do
    {
        if (MSB & 1)                       // Last bit requires TMS=1
        {
            if(Data & MSB)
            {
                DMA1SA = (unsigned char*)DMA_TMSH_TDIH;
            }
            else
            {
                DMA1SA = (unsigned char*)DMA_TMSH_TDIL;
            }
            DMA1CTL |= DMAEN;
            DMA1CTL |= DMAREQ;    // DMA1 FIRE!!!
        }
        else
        {
            if(Data & MSB)
            {
                // TMSL_TDIH is preloaded;
                DMA1CTL |= DMAEN;
                DMA1CTL |= DMAREQ;    // DMA1 FIRE!!!
            }
            else
            {
                // TMSL_TDIL is preloaded;
                DMA2CTL |= DMAEN;
                DMA2CTL |= DMAREQ;    // DMA2 FIRE!!!
            }
        }
        TDO_RD_DMA();
        TDOvalue <<= 1;                    // TDO could be any port pin
        TDOvalue |= (tdo_bitDma & _Jtag.RST) > 0;
    }
    while(MSB >>= 1);
    restoreTCLK();
    return(TDOvalue);
}

// -----------------------------------------------------------------------------
short _hil_2w_TapReset_Dma(void)
{
    unsigned short i;

    DMA1SA = (unsigned char*)DMA_TMSH_TDIH;

#ifdef MSP_FET
    if (protocol_id != SPYBIWIRE_SUBMCU)
    {
        hil_fpga_enable_bypass();
    }
#endif

    // Reset JTAG FSM
    for (i = 0; i < 6; i++)      // 6 is nominal
    {
        // TMSH_TDIH is preloaded
        DMA1sbw();
    }
    DMA1SA = (unsigned char*)DMA_TMSL_TDIH;
    DMA1sbw();

#ifdef MSP_FET
    if (protocol_id != SPYBIWIRE_SUBMCU)
    {
        hil_fpga_disable_bypass();
    }
#endif
    return 0;
}

// -----------------------------------------------------------------------------
short _hil_2w_CheckJtagFuse_Dma(void)
{
    unsigned short * dma2_tmp = DMA2SA;

    DMA1SA = (unsigned char*)DMA_TMSH_TDIH;
    DMA2SA = (unsigned char*)DMA_TMSL_TDIH;

#ifdef MSP_FET
    if (protocol_id != SPYBIWIRE_SUBMCU)
    {
        hil_fpga_enable_bypass();
    }
#endif

    // TMSL_TDIH is preloaded                 // now in Run/Test Idle
    DMA2sbw();
    // Fuse check
    // TMSH_TDIH is preloaded
    DMA1sbw();
    // TMSL_TDIH is preloaded
    DMA2sbw();
    // TMSH_TDIH is preloaded
    DMA1sbw();
    // TMSL_TDIH is preloaded
    DMA2sbw();
    // TMSH_TDIH is preloaded
    DMA1sbw();
    // In every TDI slot a TCK for the JTAG machine is generated.
    // Thus we need to get TAP in Run/Test Idle state back again.
    // TMSH_TDIH is preloaded
    DMA1sbw();
    // TMSL_TDIH is preloaded
    DMA2sbw();

    DMA2SA = dma2_tmp;

#ifdef MSP_FET
    if (protocol_id != SPYBIWIRE_SUBMCU)
    {
        hil_fpga_disable_bypass();
    }
#endif

    return 0;
}

// -----------------------------------------------------------------------------
unsigned char _hil_2w_Instr_Dma(unsigned char Instruction)
{
    prevInstruction = Instruction;

    // JTAG FSM state = Run-Test/Idle
    if (TCLK_savedDma & _Jtag.RST) //PrepTCLK
    {
        DMA1SA = (unsigned char*)DMA_TMSH_TDIH;
        DMA1sbw();
    }
    else
    {
        DMA1SA = (unsigned char*)DMA_TMSH_TDIL;
        DMA1sbw();
        DMA1SA = (unsigned char*)DMA_TMSH_TDIH;
    }

    // JTAG FSM state = Select DR-Scan
    // TMSH_TDIH loaded in previous if/else
    DMA1sbw();
    // JTAG FSM state = Select IR-Scan
    DMA1SA = (unsigned char*)DMA_TMSL_TDIH;
    DMA1sbw();
    // JTAG FSM state = Capture-IR
    DMA1sbw();
    // JTAG FSM state = Shift-IR, Shiftin TDI (8 bit)
    return sbw_ShiftDma(Instruction,F_BYTE); // JTAG FSM state = Run-Test/Idle
}


#pragma inline=forced
void hil_2w_SetReg_XBits8_64_Entry_DMA()
{
    // JTAG FSM state = Run-Test/Idle
    if (TCLK_savedDma & _Jtag.RST) //PrepTCLK
    {
        DMA1SA = (unsigned char*)DMA_TMSH_TDIH;
    }
    else
    {
        DMA1SA = (unsigned char*)DMA_TMSH_TDIL;
    }
    DMA1sbw();

    DMA1SA = (unsigned char*)DMA_TMSL_TDIH;
    // JTAG FSM state = Select DR-Scan
    DMA1sbw();
    // JTAG FSM state = Capture-DR
    DMA1sbw();
}

#pragma inline=forced
void hil_2w_SetReg_XBits8_64_Exit_DMA()
{
        // TMS & TDI slot
        DMA1SA = (unsigned char*)DMA_TMSH_TDIH;
        DMA1sbw();
        restoreTCLK();
}

//! \brief This function executes an SBW2 64BIT Data SHIFT (DR-SHIFT) in the case,
//! that the first 8 bits show a valid data capture from the JSTATE register. In case
//! of no valid capture the shift is ended after the first 8 bits.
//! \param[in]  Data to be shifted into target device
//! \param[out]  Data State - shows state of shifted out data
//! \return Value shifted out of target device JTAG module
#pragma inline=forced
unsigned long long hil_2w_SetReg_XBits8_64_Run_Dma(unsigned long long Data, unsigned char * DataState, unsigned short JStateVersion)
{
    unsigned long long      TDOvalue = 0x00000000;
    unsigned long long      MSB = 0x8000000000000000;
    unsigned char           currentDeviceState = 0;

    hil_2w_SetReg_XBits8_64_Entry_DMA();

    do
    {
        if (MSB & 1)                       // Last bit requires TMS=1
        {
            if(Data & MSB)
            {
                DMA1SA = (unsigned char*)DMA_TMSH_TDIH;
            }
            else
            {
                DMA1SA = (unsigned char*)DMA_TMSH_TDIL;
            }
            DMA1CTL |= DMAEN;
            DMA1CTL |= DMAREQ;    // DMA1 FIRE!!!
        }
        else
        {
            if(Data & MSB)
            {
                // TMSL_TDIH is preloaded;
                DMA1CTL |= DMAEN;
                DMA1CTL |= DMAREQ;    // DMA1 FIRE!!!
            }
            else
            {
                // TMSL_TDIL is preloaded;
                DMA2CTL |= DMAEN;
                DMA2CTL |= DMAREQ;    // DMA2 FIRE!!!
            }
        }
        TDO_RD_DMA();
        TDOvalue <<= 1;                    // TDO could be any port pin
        TDOvalue |= (tdo_bitDma & _Jtag.RST) > 0;

        // first 8 bits have been shifted out now go and evaluate it
        if(MSB & EIGHT_JSTATE_BITS)
        {
            // Mask out all not needed bits for device state detection
            currentDeviceState = TDOvalue & JSTATE_FLOW_CONTROL_BITS;

            // check, if BIT[63, 62, 61, 60, 59  58, 57, 56, ] & 0x4 (BP_HIT) == TRUE
            if(currentDeviceState & JSTATE_BP_HIT)
            {
                // exit DR shift state
                hil_2w_SetReg_XBits8_64_Exit_DMA();
                // reload Jstate IR
                _hil_2w_Instr_Dma(IR_JSTATE_ID);

                *DataState = VALID_DATA;
                // retrun BP_HIT BIT
                return(TDOvalue << 56);
            }

            // check, if BIT[63, 62, 61, 60, 59  58, 57, 56] (AM Sync. ongoing) == 0x83
            else if(currentDeviceState  == JSTATE_SYNC_ONGOING)
            {
                // exit DR shift state
                hil_2w_SetReg_XBits8_64_Exit_DMA();

                *DataState = SYNC_ONGOING;
                return 0;
            }
            //check, if BIT[63, 62, 61, 60, 59  58, 57, 56] & 0x40 (Locked State) == 0x40
            else if((currentDeviceState & JSTATE_LOCKED_STATE) == JSTATE_LOCKED_STATE &&
                    (currentDeviceState & JSTATE_LPM_X_FIVE) != JSTATE_LPM_X_FIVE)
            {
                // exit DR shift state
                hil_2w_SetReg_XBits8_64_Exit_DMA();
                *DataState = JTAG_LOCKED;
                return(TDOvalue << 56);
            }

            //check, if BIT[63, 62, 61, 60, 59  58, 57, 56] ( Invalid LPM) == 0x81)
            else if(currentDeviceState == JSTATE_INVALID_STATE)
            {
                // exit DR shift state
                hil_2w_SetReg_XBits8_64_Exit_DMA();
                // reload Jstate IR
                _hil_2w_Instr_Dma(IR_JSTATE_ID);

                *DataState = INVALID_DATA;
                return 0;
            }
             /*PG2.0 && PG2.1 frozen Sync detection*/
            else if (((currentDeviceState & JSTATE_SYNC_BROKEN_MASK) ==  JSTATE_SYNC_BROKEN_MCLK)
                    ||((currentDeviceState & JSTATE_SYNC_BROKEN_MASK) ==  JSTATE_SYNC_BROKEN_MCLK_PGACT)
                    ||((currentDeviceState & JSTATE_SYNC_BROKEN_MASK) ==  JSTATE_SYNC_BROKEN_PGACT))
            {
                hil_2w_SetReg_XBits8_64_Exit_DMA();

                if(JStateVersion >= 0x21)
                {
                    *DataState = SYNC_BROKEN;
                    return(0);
                }
                else // PG2.0
                {
                    *DataState = VALID_DATA;
                    return(TDOvalue << 56);
                }
            }
            // device is not in LPMx or AC read out mode just restart the shift but do not reload the JState IR
            else if(currentDeviceState != JSTATE_VALID_CAPTURE
                    &&  (currentDeviceState != JSTATE_LPM_ONE_TWO)
                    &&  (currentDeviceState != JSTATE_LPM_THREE_FOUR)
                    &&  ((currentDeviceState & JSTATE_LPM_X_FIVE) != JSTATE_LPM_X_FIVE))
            {
                // exit DR shift state
                hil_2w_SetReg_XBits8_64_Exit_DMA();
                *DataState = INVALID_DATA;
                return 0;
            }
            /*
            else
            {
                do not break, continue shift of valid data
            }
            */
        }

    }
    while(MSB >>= 1);
    restoreTCLK();

    if(     currentDeviceState == JSTATE_LPM_ONE_TWO
       ||   currentDeviceState == JSTATE_LPM_THREE_FOUR
       ||   currentDeviceState == 0x00
       ||   currentDeviceState == 0x02)
    {
        // reload Jstate IR
        _hil_2w_Instr_Dma(IR_JSTATE_ID);
    }

    *DataState = VALID_DATA;
    return(TDOvalue);
}

//! \brief This function executes an SBW2 64BIT Data SHIFT (DR-SHIFT) in the case,
//! that the first 8 bits show a valid data capture from the JSTATE register. In case
//! of no valid capture the shift is ended after the first 8 bits. Timeout could be
//! used to set the function run count.
//! \param[in]  Data to be shifted into target device
//! \return Value shifted out of target device JTAG module
unsigned long long _hil_2w_SetReg_XBits8_64_Dma(unsigned long long Data, unsigned short loopCount, unsigned short JStateVersion)
{
    unsigned long long TDOvalue = 0x00000000;
    unsigned short timeout = loopCount, syncBorkenCount = loopCount/2;
    unsigned char DataState = 0;
    do
    {
        TDOvalue = hil_2w_SetReg_XBits8_64_Run_Dma(Data, &DataState, JStateVersion);
        if(!timeout)
        {
            return TDOvalue;
        }
        timeout--;

        if(DataState == SYNC_BROKEN && syncBorkenCount > 0)
        {
            syncBorkenCount--;
            if(JStateVersion >= 0x21)
            { // Only working for PG2.1, do not harm if executed on PG2.0 – but will not create any effect only consume time
                unsigned long current3VtestReg = 0;

                // read current 3VtestReg value
                _hil_2w_Instr_Dma(IR_TEST_REG);
                current3VtestReg = _hil_2w_SetReg_XBits32_Dma(0);
                // set bit 25 high to rest sync
                current3VtestReg |= 0x2000000;
                _hil_2w_SetReg_XBits32_Dma(current3VtestReg);
                // set bit 25 low reset sync done
                current3VtestReg &= ~0x2000000;
                _hil_2w_SetReg_XBits32_Dma(current3VtestReg);
                _hil_2w_Instr_Dma(IR_JSTATE_ID);

            }
            else
            {
                return(TDOvalue);
            }
        }
    }
    while(DataState == INVALID_DATA || DataState == SYNC_ONGOING);
    return(TDOvalue);
}

unsigned char _hil_2w_SetReg_XBits08_Dma(unsigned char data)
{
    // JTAG FSM state = Run-Test/Idle
    if (TCLK_savedDma & _Jtag.RST) //PrepTCLK
    {
        DMA1SA = (unsigned char*)DMA_TMSH_TDIH;
    }
    else
    {
        DMA1SA = (unsigned char*)DMA_TMSH_TDIL;
    }
    DMA1sbw();

    DMA1SA = (unsigned char*)DMA_TMSL_TDIH;
    // JTAG FSM state = Select DR-Scan
    DMA1sbw();
    // JTAG FSM state = Capture-DR
    DMA1sbw();
    // JTAG FSM state = Shift-DR, Shiftin TDI (16 bit)
    return (sbw_ShiftDma(data,F_BYTE));
    // JTAG FSM state = Run-Test/Idle
}



unsigned short _hil_2w_SetReg_XBits16_Dma(unsigned short data)
{
    // JTAG FSM state = Run-Test/Idle
    if (TCLK_savedDma & _Jtag.RST) //PrepTCLK
    {
        DMA1SA = (unsigned char*)DMA_TMSH_TDIH;
    }
    else
    {
        DMA1SA = (unsigned char*)DMA_TMSH_TDIL;
    }
    DMA1sbw();

    DMA1SA = (unsigned char*)DMA_TMSL_TDIH;
    // JTAG FSM state = Select DR-Scan
    DMA1sbw();
    // JTAG FSM state = Capture-DR
    DMA1sbw();
    // JTAG FSM state = Shift-DR, Shiftin TDI (16 bit)
    return (sbw_ShiftDma(data,F_WORD));
    // JTAG FSM state = Run-Test/Idle
}



unsigned long _hil_2w_SetReg_XBits20_Dma(unsigned long data)
{
    unsigned long tmp;

    // JTAG FSM state = Run-Test/Idle
    if (TCLK_savedDma & _Jtag.RST) //PrepTCLK
    {
        DMA1SA = (unsigned char*)DMA_TMSH_TDIH;
    }
    else
    {
        DMA1SA = (unsigned char*)DMA_TMSH_TDIL;
    }
    DMA1sbw();

    DMA1SA = (unsigned char*)DMA_TMSL_TDIH;
    // JTAG FSM state = Select DR-Scan
    DMA1sbw();
    // JTAG FSM state = Capture-DR
    DMA1sbw();
    // JTAG FSM state = Shift-DR, Shiftin TDI (16 bit)
    // de-scramble upper 4 bits if it was a 20bit shift
    tmp = sbw_ShiftDma(data, F_ADDR);
    tmp = ((tmp >> 4) | (tmp << 16)) & 0x000FFFFF;
    return (tmp);
    // JTAG FSM state = Run-Test/Idle
}



unsigned long _hil_2w_SetReg_XBits32_Dma(unsigned long data)
{
    // JTAG FSM state = Run-Test/Idle
    if (TCLK_savedDma & _Jtag.RST) //PrepTCLK
    {
        DMA1SA = (unsigned char*)DMA_TMSH_TDIH;
    }
    else
    {
        DMA1SA = (unsigned char*)DMA_TMSH_TDIL;
    }
    DMA1sbw();

    DMA1SA = (unsigned char*)DMA_TMSL_TDIH;
    // JTAG FSM state = Select DR-Scan
    DMA1sbw();
    // JTAG FSM state = Capture-DR
    DMA1sbw();
    // JTAG FSM state = Shift-DR, Shiftin TDI (32 bit)
    return (sbw_ShiftDma(data, F_LONG));
    // JTAG FSM state = Run-Test/Idle
}


unsigned long long _hil_2w_SetReg_XBits64_Dma(unsigned long long data)
{
    // JTAG FSM state = Run-Test/Idle
    if (TCLK_savedDma & _Jtag.RST) //PrepTCLK
    {
        DMA1SA = (unsigned char*)DMA_TMSH_TDIH;
    }
    else
    {
        DMA1SA = (unsigned char*)DMA_TMSH_TDIL;
    }
    DMA1sbw();

    DMA1SA = (unsigned char*)DMA_TMSL_TDIH;
    // JTAG FSM state = Select DR-Scan
    DMA1sbw();
    // JTAG FSM state = Capture-DR
    DMA1sbw();
    // JTAG FSM state = Shift-DR, Shiftin TDI 64 bit)
    return (sbw_ShiftDma(data, F_LONG_LONG));
    // JTAG FSM state = Run-Test/Idle
}


//#pragma optimize = low
// -----------------------------------------------------------------------------
void _hil_2w_Tclk_Dma(unsigned char state)
{
    _DINT_FET();
    if (TCLK_savedDma & _Jtag.RST) //PrepTCLK
    {
        TMSLDH_DMA();
    }
    else
    {
        TMSL_DMA();
    }

    if(state)
    {
        (*_Jtag.Out) |= _Jtag.RST;
        TDIH_DMA(); TDOsbwDma();    // ExitTCLK
        TCLK_savedDma = _Jtag.RST;
    }
    else
    {
         (*_Jtag.Out) &= ~_Jtag.RST;// original
        TDIL_DMA(); TDOsbwDma();    // ExitTCLK
        TCLK_savedDma = ~_Jtag.RST;
    }
    _EINT_FET();
}

// -----------------------------------------------------------------------------
void _hil_2w_StepPsa_Dma_Xv2(unsigned long length)
{
    unsigned short * dma2_tmp = DMA2SA;

    DMA1SA = (unsigned char*)DMA_TMSH_TDIL;
    DMA2SA = (unsigned char*)DMA_TMSL_TDIL;

#ifdef MSP_FET
    if (protocol_id != SPYBIWIRE_SUBMCU)
    {
        hil_fpga_enable_bypass();
    }
#endif

    while(length--)
    {
        _hil_2w_Tclk_Dma(0);
        // TMSH_TDIL preloaded
        DMA1sbw();
        // TMSL_TDIL  preloaded
        DMA2sbw();
        // TMSL_TDIL  preloaded
        DMA2sbw();
        // TMSH_TDIL  preloaded
        DMA1sbw();
        // TMSH_TDIL  preloaded
        DMA1sbw();
        // TMSL_TDIL  preloaded
        DMA2sbw();
        _hil_2w_Tclk_Dma(1);
    }

    // Restore the prvious DMA2SA value
    DMA2SA = dma2_tmp;

#ifdef MSP_FET
    if (protocol_id != SPYBIWIRE_SUBMCU)
    {
        hil_fpga_disable_bypass();
    }
#endif
}

// -----------------------------------------------------------------------------
void _hil_2w_StepPsa_Dma(unsigned long length)
{
    unsigned short * dma2_tmp = DMA2SA;

    DMA1SA = (unsigned char*)DMA_TMSH_TDIL;
    DMA2SA = (unsigned char*)DMA_TMSL_TDIL;

#ifdef MSP_FET
    if (protocol_id != SPYBIWIRE_SUBMCU)
    {
        hil_fpga_enable_bypass();
    }
#endif
    while(length--)
    {
        _hil_2w_Tclk_Dma(1);
        _hil_2w_Tclk_Dma(0);
        // TMSH_TDIH preloaded
        DMA1sbw();
        // TMSL_TDIH  preloaded
        DMA2sbw();
        // TMSL_TDIH  preloaded
        DMA2sbw();
        // TMSH_TDIH  preloaded
        DMA1sbw();
        // TMSH_TDIH  preloaded
        DMA1sbw();
        // TMSL_TDIH  preloaded
        DMA2sbw();
    }
    // Restore the prvious DMA2SA value
    DMA2SA = dma2_tmp;

#ifdef MSP_FET
    if (protocol_id != SPYBIWIRE_SUBMCU)
    {
        hil_fpga_disable_bypass();
    }
#endif
}

// -----------------------------------------------------------------------------
void _hil_2w_StepPsaTclkHigh_Dma(unsigned long length)
{
    unsigned short * dma2_tmp = DMA2SA;

    DMA1SA = (unsigned char*)DMA_TMSH_TDIH;
    DMA2SA = (unsigned char*)DMA_TMSL_TDIH;

#ifdef MSP_FET
    if (protocol_id != SPYBIWIRE_SUBMCU)
    {
        hil_fpga_enable_bypass();
    }
#endif

    while(length--)
    {
        _hil_2w_Tclk_Dma(1);
        // TMSH_TDIH preloaded
        DMA1sbw();
        // TMSL_TDIH  preloaded
        DMA2sbw();
        // TMSL_TDIH  preloaded
        DMA2sbw();
        // TMSH_TDIH  preloaded
        DMA1sbw();
        // TMSH_TDIH  preloaded
        DMA1sbw();
        // TMSL_TDIH  preloaded
        DMA2sbw();
        _hil_2w_Tclk_Dma(0);
    }

    // Restore the prvious DMA2SA value
    DMA2SA = dma2_tmp;

#ifdef MSP_FET
    if (protocol_id != SPYBIWIRE_SUBMCU)
    {
        hil_fpga_disable_bypass();
    }
#endif
}

void _hil_2w_ConfigureSpeed_Dma(unsigned short speed)
{
    switch(speed)
    {   //----------------------------------------------------------------------
        case SBW600KHz: // fastes SBW2 speed
        {
            DMA1SZ = 8;                         // load DMA1 with size
            DMA2SZ = 8;                         // load DMA1 with size
            DMA_TMSH_TDIH[0] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[1] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[2] = /*TMS Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[3] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;

            DMA_TMSH_TDIH[4] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[5] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[6] = /*TDI Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[7] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;

            DMA_TMSH_TDIL[0] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[1] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[2] = /*TMS Slot*/  _Jtag.RST;
            DMA_TMSH_TDIL[3] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;

            DMA_TMSH_TDIL[4] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[5] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[6] = /*TDI Slot*/  0;
            DMA_TMSH_TDIL[7] = /*TDI Slot*/  _Jtag.TST;

            DMA_TMSL_TDIH[0] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[1] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[2] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIH[3] = /*TMS Slot*/ _Jtag.TST;

            DMA_TMSL_TDIH[4] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[5] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[6] = /*TDI SLot*/ _Jtag.RST;
            DMA_TMSL_TDIH[7] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;

            DMA_TMSL_TDIL[0] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[1] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[2] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIL[3] = /*TMS Slot*/ _Jtag.TST;

            DMA_TMSL_TDIL[4] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[5] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[6] = /*TDI Slot*/ 0;
            DMA_TMSL_TDIL[7] = /*TDI Slot*/ _Jtag.TST;
            break;
        }//---------------------------------------------------------------------
        case SBW400KHz: // fastes SBW2 speed
        {
            DMA1SZ = 24;                         // load DMA1 with size
            DMA2SZ = 24;                         // load DMA1 with size

            DMA_TMSH_TDIH[0] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[1] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[2] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[3] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[4] = /*TMS Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[5] = /*TMS Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[6] = /*TMS Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[7] = /*TMS Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[8] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[9] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[10] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[11] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;

            DMA_TMSH_TDIH[12] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[13] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[14] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[15] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[16] = /*TDI Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[17] = /*TDI Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[18] = /*TDI Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[19] = /*TDI Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[20] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[21] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[22] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[23] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;

            DMA_TMSH_TDIL[0] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[1] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[2] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[3] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[4] = /*TMS Slot*/  _Jtag.RST;
            DMA_TMSH_TDIL[5] = /*TMS Slot*/  _Jtag.RST;
            DMA_TMSH_TDIL[6] = /*TMS Slot*/  _Jtag.RST;
            DMA_TMSH_TDIL[7] = /*TMS Slot*/  _Jtag.RST;
            DMA_TMSH_TDIL[8] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[9] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[10] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[11] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;

            DMA_TMSH_TDIL[12] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[13] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[14] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[15] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[16] = /*TDI Slot*/  0;
            DMA_TMSH_TDIL[17] = /*TDI Slot*/  0;
            DMA_TMSH_TDIL[18] = /*TDI Slot*/  0;
            DMA_TMSH_TDIL[19] = /*TDI Slot*/  0;
            DMA_TMSH_TDIL[20] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[21] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[22] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[23] = /*TDI Slot*/  _Jtag.TST;

            DMA_TMSL_TDIH[0] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[1] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[2] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[3] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[4] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIH[5] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIH[6] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIH[7] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIH[8] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[9] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[10] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[11] = /*TMS Slot*/ _Jtag.TST;

            DMA_TMSL_TDIH[12] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[13] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[14] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[15] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[16] = /*TDI SLot*/ _Jtag.RST;
            DMA_TMSL_TDIH[17] = /*TDI SLot*/ _Jtag.RST;
            DMA_TMSL_TDIH[18] = /*TDI SLot*/ _Jtag.RST;
            DMA_TMSL_TDIH[19] = /*TDI SLot*/ _Jtag.RST;
            DMA_TMSL_TDIH[20] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[21] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[22] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[23] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;

            DMA_TMSL_TDIL[0] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[1] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[2] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[3] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[4] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIL[5] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIL[6] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIL[7] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIL[8] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[9] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[10] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[11] = /*TMS Slot*/ _Jtag.TST;

            DMA_TMSL_TDIL[12] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[13] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[14] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[15] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[16] = /*TDI Slot*/ 0;
            DMA_TMSL_TDIL[17] = /*TDI Slot*/ 0;
            DMA_TMSL_TDIL[18] = /*TDI Slot*/ 0;
            DMA_TMSL_TDIL[19] = /*TDI Slot*/ 0;
            DMA_TMSL_TDIL[20] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[21] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[22] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[23] = /*TDI Slot*/ _Jtag.TST;
            break;
        }//---------------------------------------------------------------------
        case SBW200KHz: // fastes SBW2 speed
        {//---------------------------------------------------------------------
            DMA1SZ = 48;                         // load DMA1 with size
            DMA2SZ = 48;                         // load DMA1 with size

            DMA_TMSH_TDIH[0] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[1] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[2] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[3] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[4] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[5] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[6] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[7] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[8] = /*TMS Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[9] = /*TMS Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[10] = /*TMS Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[11] = /*TMS Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[12] = /*TMS Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[13] = /*TMS Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[14] = /*TMS Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[15] = /*TMS Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[16] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[17] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[18] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[19] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[20] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[21] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[22] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[23] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;

            DMA_TMSH_TDIH[24] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[25] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[26] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[27] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[28] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[29] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[30] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[31] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[32] = /*TDI Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[33] = /*TDI Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[34] = /*TDI Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[35] = /*TDI Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[36] = /*TDI Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[37] = /*TDI Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[38] = /*TDI Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[39] = /*TDI Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[40] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[41] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[42] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[43] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[44] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[45] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[46] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[47] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;

            DMA_TMSH_TDIL[0] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[1] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[2] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[3] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[4] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[5] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[6] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[7] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[8] = /*TMS Slot*/  _Jtag.RST;
            DMA_TMSH_TDIL[9] = /*TMS Slot*/  _Jtag.RST;
            DMA_TMSH_TDIL[10] = /*TMS Slot*/  _Jtag.RST;
            DMA_TMSH_TDIL[11] = /*TMS Slot*/  _Jtag.RST;
            DMA_TMSH_TDIL[12] = /*TMS Slot*/  _Jtag.RST;
            DMA_TMSH_TDIL[13] = /*TMS Slot*/  _Jtag.RST;
            DMA_TMSH_TDIL[14] = /*TMS Slot*/  _Jtag.RST;
            DMA_TMSH_TDIL[15] = /*TMS Slot*/  _Jtag.RST;
            DMA_TMSH_TDIL[16] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[17] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[18] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[19] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[20] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[21] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[22] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[23] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;

            DMA_TMSH_TDIL[24] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[25] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[26] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[27] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[28] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[29] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[30] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[31] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[32] = /*TDI Slot*/  0;
            DMA_TMSH_TDIL[33] = /*TDI Slot*/  0;
            DMA_TMSH_TDIL[34] = /*TDI Slot*/  0;
            DMA_TMSH_TDIL[35] = /*TDI Slot*/  0;
            DMA_TMSH_TDIL[36] = /*TDI Slot*/  0;
            DMA_TMSH_TDIL[37] = /*TDI Slot*/  0;
            DMA_TMSH_TDIL[38] = /*TDI Slot*/  0;
            DMA_TMSH_TDIL[39] = /*TDI Slot*/  0;
            DMA_TMSH_TDIL[40] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[41] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[42] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[43] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[44] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[45] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[46] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[47] = /*TDI Slot*/  _Jtag.TST;

            DMA_TMSL_TDIH[0] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[1] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[2] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[3] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[4] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[5] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[6] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[7] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[8] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIH[9] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIH[10] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIH[11] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIH[12] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIH[13] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIH[14] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIH[15] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIH[16] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[17] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[18] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[19] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[20] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[21] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[22] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[23] = /*TMS Slot*/ _Jtag.TST;

            DMA_TMSL_TDIH[24] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[25] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[26] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[27] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[28] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[29] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[30] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[31] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[32] = /*TDI SLot*/ _Jtag.RST;
            DMA_TMSL_TDIH[33] = /*TDI SLot*/ _Jtag.RST;
            DMA_TMSL_TDIH[34] = /*TDI SLot*/ _Jtag.RST;
            DMA_TMSL_TDIH[35] = /*TDI SLot*/ _Jtag.RST;
            DMA_TMSL_TDIH[36] = /*TDI SLot*/ _Jtag.RST;
            DMA_TMSL_TDIH[37] = /*TDI SLot*/ _Jtag.RST;
            DMA_TMSL_TDIH[38] = /*TDI SLot*/ _Jtag.RST;
            DMA_TMSL_TDIH[39] = /*TDI SLot*/ _Jtag.RST;
            DMA_TMSL_TDIH[40] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[41] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[42] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[43] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[44] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[45] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[46] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[47] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;

            DMA_TMSL_TDIL[0] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[1] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[2] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[3] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[4] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[5] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[6] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[7] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[8] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIL[9] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIL[10] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIL[11] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIL[12] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIL[13] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIL[14] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIL[15] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIL[16] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[17] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[18]= /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[19] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[20] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[21] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[22] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[23] = /*TMS Slot*/ _Jtag.TST;

            DMA_TMSL_TDIL[24] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[25] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[26] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[27] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[28] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[29] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[30] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[31] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[32] = /*TDI Slot*/ 0;
            DMA_TMSL_TDIL[33] = /*TDI Slot*/ 0;
            DMA_TMSL_TDIL[34] = /*TDI Slot*/ 0;
            DMA_TMSL_TDIL[35] = /*TDI Slot*/ 0;
            DMA_TMSL_TDIL[36] = /*TDI Slot*/ 0;
            DMA_TMSL_TDIL[37] = /*TDI Slot*/ 0;
            DMA_TMSL_TDIL[38] = /*TDI Slot*/ 0;
            DMA_TMSL_TDIL[39] = /*TDI Slot*/ 0;
            DMA_TMSL_TDIL[40] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[41] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[42] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[43] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[44] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[45] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[46] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[47] = /*TDI Slot*/ _Jtag.TST;
            break;
        }//---------------------------------------------------------------------
        case SBW100KHz: // fastes SBW2 speed
        {//---------------------------------------------------------------------
            DMA1SZ = 84;                         // load DMA1 with size
            DMA2SZ = 84;                         // load DMA1 with size

            DMA_TMSH_TDIH[0] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[1] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[2] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[3] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[4] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[5] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[6] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[7] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[8] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[9] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[10] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[11] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[12] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[13] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[14] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[15] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[16] = /*TMS Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[17] = /*TMS Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[18] = /*TMS Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[19] = /*TMS Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[20] = /*TMS Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[21] = /*TMS Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[22] = /*TMS Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[23] = /*TMS Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[24] = /*TMS Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[25] = /*TMS Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[26] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[27] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[28] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[29] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[30] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[31] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[32] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[33] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[34] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[35] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[36] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[37] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[38] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[39] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[40] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[41] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;

            DMA_TMSH_TDIH[42] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[43] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[44] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[45] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[46] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[47] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[48] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[49] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[50] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[51] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[52] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[53] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[54] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[55] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[56] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[57] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[58] = /*TDI Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[59] = /*TDI Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[60] = /*TDI Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[61] = /*TDI Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[62] = /*TDI Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[63] = /*TDI Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[64] = /*TDI Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[65] = /*TDI Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[66] = /*TDI Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[67] = /*TDI Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[68] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[69] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[70] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[71] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[72] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[73] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[74] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[75] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[76] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[77] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[78] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[79] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[80] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[81] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[82] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[83] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;

            DMA_TMSH_TDIL[0] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[1] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[2] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[3] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[4] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[5] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[6] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[7] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[8] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[9] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[10] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[11] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[12] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[13] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[14] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[15] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[16] = /*TMS Slot*/  _Jtag.RST;
            DMA_TMSH_TDIL[17] = /*TMS Slot*/  _Jtag.RST;
            DMA_TMSH_TDIL[18] = /*TMS Slot*/  _Jtag.RST;
            DMA_TMSH_TDIL[19] = /*TMS Slot*/  _Jtag.RST;
            DMA_TMSH_TDIL[20] = /*TMS Slot*/  _Jtag.RST;
            DMA_TMSH_TDIL[21] = /*TMS Slot*/  _Jtag.RST;
            DMA_TMSH_TDIL[22] = /*TMS Slot*/  _Jtag.RST;
            DMA_TMSH_TDIL[23] = /*TMS Slot*/  _Jtag.RST;
            DMA_TMSH_TDIL[24] = /*TMS Slot*/  _Jtag.RST;
            DMA_TMSH_TDIL[25] = /*TMS Slot*/  _Jtag.RST;
            DMA_TMSH_TDIL[26] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[27] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[28] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[29] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[30] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[31] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[32] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[33] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[34] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[35] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[36] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[37] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[38] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[39] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[40] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[41] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;

            DMA_TMSH_TDIL[42] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[43] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[44] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[45] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[46] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[47] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[48] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[49] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[50] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[51] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[52] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[53] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[54] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[55] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[56] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[57] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[58] = /*TDI Slot*/  0;
            DMA_TMSH_TDIL[59] = /*TDI Slot*/  0;
            DMA_TMSH_TDIL[60] = /*TDI Slot*/  0;
            DMA_TMSH_TDIL[61] = /*TDI Slot*/  0;
            DMA_TMSH_TDIL[62] = /*TDI Slot*/  0;
            DMA_TMSH_TDIL[63] = /*TDI Slot*/  0;
            DMA_TMSH_TDIL[64] = /*TDI Slot*/  0;
            DMA_TMSH_TDIL[65] = /*TDI Slot*/  0;
            DMA_TMSH_TDIL[66] = /*TDI Slot*/  0;
            DMA_TMSH_TDIL[67] = /*TDI Slot*/  0;
            DMA_TMSH_TDIL[68] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[69] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[70] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[71] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[72] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[73] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[74] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[75] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[76] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[77] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[78] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[79] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[80] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[81] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[82] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[83] = /*TDI Slot*/  _Jtag.TST;

            DMA_TMSL_TDIH[0] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[1] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[2] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[3] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[4] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[5] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[6] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[7] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[8] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[9] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[10] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[11] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[12] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[13] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[14] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[15] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[16] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIH[17] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIH[18] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIH[19] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIH[20] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIH[21] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIH[22] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIH[23] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIH[24] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIH[25] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIH[26] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[27] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[28] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[29] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[30] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[31] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[32] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[33] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[34] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[35] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[36] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[37] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[38] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[39] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[40] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[41] = /*TMS Slot*/ _Jtag.TST;

            DMA_TMSL_TDIH[42] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[43] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[44] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[45] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[46] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[47] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[48] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[49] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[50] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[51] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[52] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[53] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[54] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[55] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[56] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[57] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[58] = /*TDI SLot*/ _Jtag.RST;
            DMA_TMSL_TDIH[59] = /*TDI SLot*/ _Jtag.RST;
            DMA_TMSL_TDIH[60] = /*TDI SLot*/ _Jtag.RST;
            DMA_TMSL_TDIH[61] = /*TDI SLot*/ _Jtag.RST;
            DMA_TMSL_TDIH[62] = /*TDI SLot*/ _Jtag.RST;
            DMA_TMSL_TDIH[63] = /*TDI SLot*/ _Jtag.RST;
            DMA_TMSL_TDIH[64] = /*TDI SLot*/ _Jtag.RST;
            DMA_TMSL_TDIH[65] = /*TDI SLot*/ _Jtag.RST;
            DMA_TMSL_TDIH[66] = /*TDI SLot*/ _Jtag.RST;
            DMA_TMSL_TDIH[67] = /*TDI SLot*/ _Jtag.RST;
            DMA_TMSL_TDIH[68] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[69] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[70] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[71] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[72] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[73] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[74] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[75] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[76] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[77] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[78] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[79] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[80] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[81] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[82] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[83] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;

            DMA_TMSL_TDIL[0] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[1] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[2] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[3] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[4] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[5] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[6] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[7] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[8] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[9] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[10] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[11] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[12] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[13] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[14] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[15] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[16] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIL[17] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIL[18] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIL[19] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIL[20] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIL[21] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIL[22] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIL[23] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIL[24] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIL[25] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIL[26] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[27] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[28]= /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[29] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[30] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[31] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[32] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[33] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[34] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[35] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[36]= /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[37] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[38] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[39] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[40] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[41] = /*TMS Slot*/ _Jtag.TST;

            DMA_TMSL_TDIL[42] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[43] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[44] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[45] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[46] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[47] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[48] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[49] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[50] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[51] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[52] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[53] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[54] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[55] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[56] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[57] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[58] = /*TDI Slot*/ 0;
            DMA_TMSL_TDIL[59] = /*TDI Slot*/ 0;
            DMA_TMSL_TDIL[60] = /*TDI Slot*/ 0;
            DMA_TMSL_TDIL[61] = /*TDI Slot*/ 0;
            DMA_TMSL_TDIL[62] = /*TDI Slot*/ 0;
            DMA_TMSL_TDIL[63] = /*TDI Slot*/ 0;
            DMA_TMSL_TDIL[64] = /*TDI Slot*/ 0;
            DMA_TMSL_TDIL[65] = /*TDI Slot*/ 0;
            DMA_TMSL_TDIL[66] = /*TDI Slot*/ 0;
            DMA_TMSL_TDIL[67] = /*TDI Slot*/ 0;
            DMA_TMSL_TDIL[68] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[69] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[70] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[71] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[72] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[73] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[74] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[75] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[76] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[77] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[78] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[79] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[80] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[81] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[82] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[83] = /*TDI Slot*/ _Jtag.TST;
            break;
        }//---------------------------------------------------------------------
        default:// fastes SBW2 speed
        {
            DMA1SZ = 8;                         // load DMA1 with size
            DMA2SZ = 8;                         // load DMA1 with size
            DMA_TMSH_TDIH[0] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[1] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[2] = /*TMS Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[3] = /*TMS Slot*/ _Jtag.RST|_Jtag.TST;

            DMA_TMSH_TDIH[4] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[5] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIH[6] = /*TDI Slot*/ _Jtag.RST;
            DMA_TMSH_TDIH[7] = /*TDI Slot*/ _Jtag.RST|_Jtag.TST;

            DMA_TMSH_TDIL[0] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[1] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;
            DMA_TMSH_TDIL[2] = /*TMS Slot*/  _Jtag.RST;
            DMA_TMSH_TDIL[3] = /*TMS Slot*/  _Jtag.RST|_Jtag.TST;

            DMA_TMSH_TDIL[4] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[5] = /*TDI Slot*/  _Jtag.TST;
            DMA_TMSH_TDIL[6] = /*TDI Slot*/  0;
            DMA_TMSH_TDIL[7] = /*TDI Slot*/  _Jtag.TST;

            DMA_TMSL_TDIH[0] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[1] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIH[2] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIH[3] = /*TMS Slot*/ _Jtag.TST;

            DMA_TMSL_TDIH[4] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[5] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;
            DMA_TMSL_TDIH[6] = /*TDI SLot*/ _Jtag.RST;
            DMA_TMSL_TDIH[7] = /*TDI SLot*/ _Jtag.RST|_Jtag.TST;

            DMA_TMSL_TDIL[0] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[1] = /*TMS Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[2] = /*TMS Slot*/ 0;
            DMA_TMSL_TDIL[3] = /*TMS Slot*/ _Jtag.TST;

            DMA_TMSL_TDIL[4] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[5] = /*TDI Slot*/ _Jtag.TST;
            DMA_TMSL_TDIL[6] = /*TDI Slot*/ 0;
            DMA_TMSL_TDIL[7] = /*TDI Slot*/ _Jtag.TST;
            break;
        }//---------------------------------------------------------------------
    }
}

#ifdef MSP_FET
extern unsigned char _hil_generic_Instr(unsigned char Instruction);
extern unsigned char _hil_generic_SetReg_XBits08(unsigned char Data);
extern unsigned short _hil_generic_SetReg_XBits16(unsigned short Data);
extern unsigned long _hil_generic_SetReg_XBits20(unsigned long Data);
extern unsigned long _hil_generic_SetReg_XBits32(unsigned long Data);

extern unsigned char lastTestState;
extern unsigned char lastResetState;
extern short _hil_SetVcc(unsigned short Vcc);

#endif

#pragma optimize = medium
// -----------------------------------------------------------------------------
short _hil_2w_BlowFuse_Dma(unsigned char targetHasTestVpp)
{
#ifdef MSP_FET
    if (protocol_id != SPYBIWIRE_SUBMCU)
    {
        unsigned char MSB = 0x80;;
        unsigned char Data = IR_EX_BLOW, i = 0;

        //_hil_SetVcc(2500);
        _hil_generic_Instr(IR_PREPARE_BLOW);

        _DINT_FET();
        lastResetState = 1;
        lastTestState = 1;
        hil_fpga_enable_bypass();

        // JTAG FSM state = Run-Test/Idle
        TMSH_DMA(); TDIH_DMA(); TDOsbwFuse();

        // JTAG FSM state = Select DR-Scan
        TMSH_DMA(); TDIH_DMA(); TDOsbwFuse();

        // JTAG FSM state = Select IR-Scan
        TMSL_DMA(); TDIH_DMA(); TDOsbwFuse();

        // JTAG FSM state = Capture-IR
        TMSL_DMA(); TDIH_DMA(); TDOsbwFuse();

        for (i = 8; i > 1; i--)
        {
            if((Data & MSB) == 0)
            {
                TMSL_DMA();  TDIL_DMA(); TDO_RD_FUSE();
            }
            else
            {
                TMSL_DMA(); TDIH_DMA(); TDO_RD_FUSE();
            }
            Data <<= 1;
        }
        // last bit requires TMS=1; TDO one bit before TDI
        if((Data & MSB) == 0)
        {
            TMSH_DMA();  TDIL_DMA();  TDO_RD_FUSE();
        }
        else
        {
            TMSH_DMA();  TDIH_DMA();  TDO_RD_FUSE();
        }
        // SBWTDIO must be low on exit!
        TMSH_DMA(); TDIL_DMA(); TDOsbwFuse();

        TMSL_DMA(); TDIL_DMA(); TDOsbwFuse();
        // instruction shift done!

        // After the IR_EX_BLOW instruction is shifted in via SBW, one more TMS_SLOT must be performed
        // create TMSL slot

        (*_Jtag.Out) &= ~_Jtag.RST;
        SBW_DELAY; SBW_DELAY;
        SBW_DELAY; SBW_DELAY;

        (*_Jtag.Out) &= ~_Jtag.TST;
        SBW_DELAY; SBW_DELAY;
        SBW_DELAY; SBW_DELAY;

        (*_Jtag.Out) |= _Jtag.TST;
        _hil_Delay_1ms(1);

        // Apply fuse blow voltage
        setVpp(1);

        // Taking SBWTDIO high as soon as Vpp has been settled blows the fuse
        (*_Jtag.Out) |=  _Jtag.RST;

        _hil_Delay_1ms(1);

        setVpp(0);                                       // switch VPP off

        hil_fpga_disable_bypass();

        // now perform a BOR via JTAG - we loose control of the device then...
        _hil_generic_Instr(IR_TEST_REG);
        _hil_generic_SetReg_XBits32(0x00000200);

        _EINT_FET();
    }
#endif
    // not suppored by eZ-FET
    return 0;
}
/* EOF */
