/* Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
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

#pragma once

struct jtag _Jtag = {0};
uint8_t tdo_bitDma = 0;

#ifdef MSP_FET
    uint16_t protocol_id = 0;
#endif

#define SBW_DELAY   { _NOP();_NOP();_NOP();/*_NOP();_NOP();_NOP();*/}

//extern void testVpp(uint8_t mode);
//extern void setVpp(int32_t voltage);
//extern void TCLKset1();
//extern void TCLKset0();
//uint32_t _hil_2w_SetReg_XBits32_Dma(uint32_t data);

//#pragma inline=forced
//#pragma optimize = low
//void TMSH_DMA()
//{
//    // TMS = 1
//    /*_DINT();*/
//    (*_Jtag.Out) |=  _Jtag.RST;
//    (*_Jtag.Out) &= ~_Jtag.TST;
//    SBW_DELAY;
//    (*_Jtag.Out) |= _Jtag.TST;
//    /*_EINT();*/
//}
//
//#pragma optimize = low
//void TMSL_DMA()
//{
//    // TMS = 0
//    /*_DINT();*/
//    (*_Jtag.Out) &= ~_Jtag.RST;
//    (*_Jtag.Out) &= ~_Jtag.TST;
//    SBW_DELAY;
//    (*_Jtag.Out) |= _Jtag.TST;
//    /*_EINT();*/
//}
//
//#pragma optimize = low
//void TMSLDH_DMA()
//{
//    // TMS = 0, then TCLK immediately = 1
//    /*_DINT();*/
//    (*_Jtag.Out) &= ~_Jtag.RST;
//    (*_Jtag.Out) &= ~_Jtag.TST;
//    SBW_DELAY;
//    (*_Jtag.Out) |= _Jtag.RST;
//    (*_Jtag.Out) |= _Jtag.TST;
//    /*_EINT();*/
//}
//
//#pragma optimize = low
//void TDIH_DMA()
//{
//    // TDI = 1
//    /*_DINT();*/
//    (*_Jtag.Out) |=  _Jtag.RST;
//    (*_Jtag.Out) &= ~_Jtag.TST;
//    SBW_DELAY;
//    (*_Jtag.Out) |= _Jtag.TST;
//    /*_EINT();*/
//}
//
//#pragma optimize = low
//void TDIL_DMA()
//{
//    // TDI = 0
//    /*_DINT();*/
//    (*_Jtag.Out) &= ~_Jtag.RST;
//    (*_Jtag.Out) &= ~_Jtag.TST;
//    SBW_DELAY;
//    (*_Jtag.Out) |= _Jtag.TST;
//    /*_EINT();*/
//}
//
//#pragma inline=forced
//void TDOsbwDma()
//{
//    // TDO cycle without reading TDO
//    _DINT_FET();
//    (*_Jtag.DIRECTION) &= ~_Jtag.RST;
//    (*_Jtag.Out) &= ~_Jtag.TST;
//    SBW_DELAY;
//    (*_Jtag.Out) |= _Jtag.TST;
//    SBW_DELAY;
//    SBW_DELAY;
//    (*_Jtag.DIRECTION) |= _Jtag.RST;
//    _EINT_FET();
//}
//#pragma inline=forced
//void TDOsbwFuse()
//{
//    // TDO cycle without reading TDO
//   (*_Jtag.DIRECTION) &= ~_Jtag.RST;
//   (*_Jtag.Out) &= ~_Jtag.TST;
//   SBW_DELAY;
//   (*_Jtag.Out) |= _Jtag.TST;
//   SBW_DELAY;SBW_DELAY;SBW_DELAY;(*_Jtag.DIRECTION) |= _Jtag.RST;
//}
//#pragma inline=forced
//void TDO_RD_FUSE()
//{
//    // TDO cycle with TDO read
//    (*_Jtag.DIRECTION) &= ~_Jtag.RST;
//    (*_Jtag.Out) &= ~_Jtag.TST;
//    SBW_DELAY;
//    tdo_bitDma = (*_Jtag.In);
//    SBW_DELAY;
//    SBW_DELAY;
//    SBW_DELAY;
//    (*_Jtag.Out) |= _Jtag.TST;
//    (*_Jtag.DIRECTION) |= _Jtag.RST;
//}
//#pragma inline=forced
//void TDO_RD_DMA()
//{
//    // TDO cycle with TDO read
//    _DINT_FET();
//    (*_Jtag.DIRECTION) &= ~_Jtag.RST;
//    (*_Jtag.Out) &= ~_Jtag.TST;
//    SBW_DELAY;
//    tdo_bitDma = (*_Jtag.In);
//    SBW_DELAY;
//    SBW_DELAY;
//    (*_Jtag.Out) |= _Jtag.TST;
//    (*_Jtag.DIRECTION) |= _Jtag.RST;
//    _EINT_FET();
//}

uint8_t DMA_TMSH_TDIH[84] = {0};

uint8_t DMA_TMSH_TDIL[84] = {0};

uint8_t DMA_TMSL_TDIH[84] = {0};

uint8_t DMA_TMSL_TDIL[84] = {0};

void* DMA1SA = nullptr;
void* DMA2SA = nullptr;

uint8_t TCLK_savedDma = 0;
uint8_t current_Instr = 0;
uint8_t prevInstruction = 0;
//void _hil_2w_ConfigureSpeed_Dma(uint16_t speed);


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
void setProtocolSbw2Dma(uint16_t id)
{
    protocol_id = id;
}
#endif

uint8_t _hil_2w_GetPrevInstruction_Dma()
{
    return prevInstruction;
}

//#pragma inline=forced
void DMA1sbw(void)
{
    const bool tms = (DMA1SA==DMA_TMSH_TDIH || DMA1SA==DMA_TMSH_TDIL);
    const bool tdi = (DMA1SA==DMA_TMSH_TDIH || DMA1SA==DMA_TMSL_TDIH);
    _sbwio(tms, tdi);
    
//    DMA1CTL |= DMAEN;
//    DMA1CTL |= DMAREQ;    // DMA1 FIRE!!!
//    TDOsbwDma();
}

//#pragma inline=forced
void DMA2sbw(void)
{
    const bool tms = (DMA2SA==DMA_TMSH_TDIH || DMA2SA==DMA_TMSH_TDIL);
    const bool tdi = (DMA2SA==DMA_TMSH_TDIH || DMA2SA==DMA_TMSL_TDIH);
    _sbwio(tms, tdi);
    
//    DMA2CTL |= DMAEN;
//    DMA2CTL |= DMAREQ;    // DMA2 FIRE!!!
//    TDOsbwDma();
}

//#pragma inline=forced
void DMA1sbw_r(void)
{
    const bool tms = (DMA1SA==DMA_TMSH_TDIH || DMA1SA==DMA_TMSH_TDIL);
    const bool tdi = (DMA1SA==DMA_TMSH_TDIH || DMA1SA==DMA_TMSL_TDIH);
    _sbwio_r(tms, tdi);
}

//#pragma inline=forced
void DMA2sbw_r(void)
{
    const bool tms = (DMA2SA==DMA_TMSH_TDIH || DMA2SA==DMA_TMSH_TDIL);
    const bool tdi = (DMA2SA==DMA_TMSH_TDIH || DMA2SA==DMA_TMSL_TDIH);
    _sbwio_r(tms, tdi);
}




//#pragma inline=forced
void restoreTCLK(void)
{
    if (TCLK_savedDma)
    {
        DMA1SA = (uint8_t*)DMA_TMSH_TDIH;
        DMA1sbw();
        DMA1SA = (uint8_t*) DMA_TMSL_TDIH;
        DMA1sbw();
    }
    else
    {
        DMA1SA = (uint8_t*)DMA_TMSH_TDIL;
        DMA1sbw();
        // TMSL_TDIL is preloaded;
        DMA2sbw();
    }
}

//#pragma inline=forced
//uint64_t sbw_ShiftDma(uint64_t Data, uint16_t Bits)
//{
//    uint64_t TDOvalue = 0x0000000000000000;
//    uint64_t MSB = 0x0000000000000000;
//
//    switch(Bits)
//    {
//        case F_BYTE: MSB = 0x00000080;
//            break;
//        case F_WORD: MSB = 0x00008000;
//            break;
//        case F_ADDR: MSB = 0x00080000;
//            break;
//        case F_LONG: MSB = 0x80000000;
//            break;
//        case F_LONG_LONG: MSB = 0x8000000000000000;
//            break;
//        default: // this is an unsupported format, function will just return 0
//            return TDOvalue;
//    }
//    do
//    {
//        if (MSB & 1)                       // Last bit requires TMS=1
//        {
//            if(Data & MSB)
//            {
//                DMA1SA = (uint8_t*)DMA_TMSH_TDIH;
//            }
//            else
//            {
//                DMA1SA = (uint8_t*)DMA_TMSH_TDIL;
//            }
//            DMA1CTL |= DMAEN;
//            DMA1CTL |= DMAREQ;    // DMA1 FIRE!!!
//        }
//        else
//        {
//            if(Data & MSB)
//            {
//                // TMSL_TDIH is preloaded;
//                DMA1CTL |= DMAEN;
//                DMA1CTL |= DMAREQ;    // DMA1 FIRE!!!
//            }
//            else
//            {
//                // TMSL_TDIL is preloaded;
//                DMA2CTL |= DMAEN;
//                DMA2CTL |= DMAREQ;    // DMA2 FIRE!!!
//            }
//        }
//        TDO_RD_DMA();
//        TDOvalue <<= 1;                    // TDO could be any port pin
//        TDOvalue |= (tdo_bitDma & _Jtag.RST) > 0;
//    }
//    while(MSB >>= 1);
//    restoreTCLK();
//    return(TDOvalue);
//}

// -----------------------------------------------------------------------------
int16_t _hil_2w_TapReset_Dma(void)
{
    uint16_t i;

    DMA1SA = (uint8_t*)DMA_TMSH_TDIH;

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
    DMA1SA = (uint8_t*)DMA_TMSL_TDIH;
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
int16_t _hil_2w_CheckJtagFuse_Dma(void)
{
    void* dma2_tmp = DMA2SA;

    DMA1SA = (uint8_t*)DMA_TMSH_TDIH;
    DMA2SA = (uint8_t*)DMA_TMSL_TDIH;

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
SBWShiftProxy<uint8_t> _hil_2w_Instr_Dma(uint8_t Instruction)
{
    prevInstruction = Instruction;

    // JTAG FSM state = Run-Test/Idle
    if (TCLK_savedDma) //PrepTCLK
    {
        DMA1SA = (uint8_t*)DMA_TMSH_TDIH;
        DMA1sbw();
    }
    else
    {
        DMA1SA = (uint8_t*)DMA_TMSH_TDIL;
        DMA1sbw();
        DMA1SA = (uint8_t*)DMA_TMSH_TDIH;
    }

    // JTAG FSM state = Select DR-Scan
    // TMSH_TDIH loaded in previous if/else
    DMA1sbw();
    // JTAG FSM state = Select IR-Scan
    DMA1SA = (uint8_t*)DMA_TMSL_TDIH;
    DMA1sbw();
    // JTAG FSM state = Capture-IR
    DMA1sbw();
    // JTAG FSM state = Shift-IR, Shiftin TDI (8 bit)
    return SBWShiftProxy<uint8_t>(this, Instruction);
}


//#pragma inline=forced
void hil_2w_SetReg_XBits8_64_Entry_DMA()
{
    // JTAG FSM state = Run-Test/Idle
    if (TCLK_savedDma) //PrepTCLK
    {
        DMA1SA = (uint8_t*)DMA_TMSH_TDIH;
    }
    else
    {
        DMA1SA = (uint8_t*)DMA_TMSH_TDIL;
    }
    DMA1sbw();

    DMA1SA = (uint8_t*)DMA_TMSL_TDIH;
    // JTAG FSM state = Select DR-Scan
    DMA1sbw();
    // JTAG FSM state = Capture-DR
    DMA1sbw();
}

//#pragma inline=forced
void hil_2w_SetReg_XBits8_64_Exit_DMA()
{
        // TMS & TDI slot
        DMA1SA = (uint8_t*)DMA_TMSH_TDIH;
        DMA1sbw();
        restoreTCLK();
}

//! \brief This function executes an SBW2 64BIT Data SHIFT (DR-SHIFT) in the case,
//! that the first 8 bits show a valid data capture from the JSTATE register. In case
//! of no valid capture the shift is ended after the first 8 bits.
//! \param[in]  Data to be shifted into target device
//! \param[out]  Data State - shows state of shifted out data
//! \return Value shifted out of target device JTAG module

#warning we should try optimizing this function if its called a lot. perhaps just do the full shift?

//#pragma inline=forced
uint64_t hil_2w_SetReg_XBits8_64_Run_Dma(uint64_t Data, uint8_t * DataState, uint16_t JStateVersion)
{
    printf("MEOWMIX hil_2w_SetReg_XBits8_64_Run_Dma CALLED \n");
    uint64_t      TDOvalue = 0x00000000;
    uint64_t      MSB = 0x8000000000000000;
    uint8_t           currentDeviceState = 0;
    uint8_t counter = 0;

    hil_2w_SetReg_XBits8_64_Entry_DMA();

    do
    {
        if (MSB & 1)                       // Last bit requires TMS=1
        {
            if(Data & MSB)
            {
                DMA1SA = (uint8_t*)DMA_TMSH_TDIH;
            }
            else
            {
                DMA1SA = (uint8_t*)DMA_TMSH_TDIL;
            }
            
            DMA1sbw_r();
//            DMA1CTL |= DMAEN;
//            DMA1CTL |= DMAREQ;    // DMA1 FIRE!!!
        }
        else
        {
            if(Data & MSB)
            {
                // TMSL_TDIH is preloaded;
                DMA1sbw_r();
//                DMA1CTL |= DMAEN;
//                DMA1CTL |= DMAREQ;    // DMA1 FIRE!!!
            }
            else
            {
                // TMSL_TDIL is preloaded;
//                DMA2CTL |= DMAEN;
//                DMA2CTL |= DMAREQ;    // DMA2 FIRE!!!
                DMA2sbw_r();
            }
        }
        
        // Every 8 bits, read another byte
        if (counter == 7) {
            TDOvalue <<= 8;
            TDOvalue |= _read(1);
            counter = 0;
        
        } else {
            counter++;
        }
//        TDO_RD_DMA();
//        TDOvalue <<= 1;                    // TDO could be any port pin
//        TDOvalue |= (tdo_bitDma & _Jtag.RST) > 0;

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
uint64_t _hil_2w_SetReg_XBits8_64_Dma(uint64_t Data, uint16_t loopCount, uint16_t JStateVersion)
{
    uint64_t TDOvalue = 0x00000000;
    uint16_t timeout = loopCount, syncBorkenCount = loopCount/2;
    uint8_t DataState = 0;
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
            { // Only working for PG2.1, do not harm if executed on PG2.0 ñ but will not create any effect only consume time
                uint32_t current3VtestReg = 0;

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

SBWShiftProxy<uint8_t> _hil_2w_SetReg_XBits08_Dma(uint8_t data)
{
    // JTAG FSM state = Run-Test/Idle
    if (TCLK_savedDma) //PrepTCLK
    {
        DMA1SA = (uint8_t*)DMA_TMSH_TDIH;
    }
    else
    {
        DMA1SA = (uint8_t*)DMA_TMSH_TDIL;
    }
    DMA1sbw();

    DMA1SA = (uint8_t*)DMA_TMSL_TDIH;
    // JTAG FSM state = Select DR-Scan
    DMA1sbw();
    // JTAG FSM state = Capture-DR
    DMA1sbw();
    // JTAG FSM state = Shift-DR, Shiftin TDI (16 bit)
    return SBWShiftProxy<uint8_t>(this, data);
    // JTAG FSM state = Run-Test/Idle
}



SBWShiftProxy<uint16_t> _hil_2w_SetReg_XBits16_Dma(uint16_t data)
{
    // JTAG FSM state = Run-Test/Idle
    if (TCLK_savedDma) //PrepTCLK
    {
        DMA1SA = (uint8_t*)DMA_TMSH_TDIH;
    }
    else
    {
        DMA1SA = (uint8_t*)DMA_TMSH_TDIL;
    }
    DMA1sbw();

    DMA1SA = (uint8_t*)DMA_TMSL_TDIH;
    // JTAG FSM state = Select DR-Scan
    DMA1sbw();
    // JTAG FSM state = Capture-DR
    DMA1sbw();
    // JTAG FSM state = Shift-DR, Shiftin TDI (16 bit)
    return SBWShiftProxy<uint16_t>(this, data);
    // JTAG FSM state = Run-Test/Idle
}



SBWShiftProxy<uint32_t,20> _hil_2w_SetReg_XBits20_Dma(uint32_t data)
{
    // JTAG FSM state = Run-Test/Idle
    if (TCLK_savedDma) //PrepTCLK
    {
        DMA1SA = (uint8_t*)DMA_TMSH_TDIH;
    }
    else
    {
        DMA1SA = (uint8_t*)DMA_TMSH_TDIL;
    }
    DMA1sbw();

    DMA1SA = (uint8_t*)DMA_TMSL_TDIH;
    // JTAG FSM state = Select DR-Scan
    DMA1sbw();
    // JTAG FSM state = Capture-DR
    DMA1sbw();
    // JTAG FSM state = Shift-DR, Shiftin TDI (16 bit)
    return SBWShiftProxy<uint32_t,20>(this, data);
    // JTAG FSM state = Run-Test/Idle
}



SBWShiftProxy<uint32_t> _hil_2w_SetReg_XBits32_Dma(uint32_t data)
{
    // JTAG FSM state = Run-Test/Idle
    if (TCLK_savedDma) //PrepTCLK
    {
        DMA1SA = (uint8_t*)DMA_TMSH_TDIH;
    }
    else
    {
        DMA1SA = (uint8_t*)DMA_TMSH_TDIL;
    }
    DMA1sbw();

    DMA1SA = (uint8_t*)DMA_TMSL_TDIH;
    // JTAG FSM state = Select DR-Scan
    DMA1sbw();
    // JTAG FSM state = Capture-DR
    DMA1sbw();
    // JTAG FSM state = Shift-DR, Shiftin TDI (32 bit)
    return SBWShiftProxy<uint32_t>(this, data);
    // JTAG FSM state = Run-Test/Idle
}


SBWShiftProxy<uint64_t> _hil_2w_SetReg_XBits64_Dma(uint64_t data)
{
    // JTAG FSM state = Run-Test/Idle
    if (TCLK_savedDma) //PrepTCLK
    {
        DMA1SA = (uint8_t*)DMA_TMSH_TDIH;
    }
    else
    {
        DMA1SA = (uint8_t*)DMA_TMSH_TDIL;
    }
    DMA1sbw();

    DMA1SA = (uint8_t*)DMA_TMSL_TDIH;
    // JTAG FSM state = Select DR-Scan
    DMA1sbw();
    // JTAG FSM state = Capture-DR
    DMA1sbw();
    // JTAG FSM state = Shift-DR, Shiftin TDI 64 bit)
    return SBWShiftProxy<uint64_t>(this, data);
    // JTAG FSM state = Run-Test/Idle
}


//#pragma optimize = low
// -----------------------------------------------------------------------------
void _hil_2w_Tclk_Dma(uint8_t state)
{
    _sbwioTclk(0, state);
    
//    _DINT_FET();
//    if (TCLK_savedDma) //PrepTCLK
//    {
//        TMSLDH_DMA();
//    }
//    else
//    {
//        TMSL_DMA();
//    }
//
//    if(state)
//    {
//        (*_Jtag.Out) |= _Jtag.RST;
//        TDIH_DMA();
//        TDOsbwDma();    // ExitTCLK
//        TCLK_savedDma = true;
//    }
//    else
//    {
//        (*_Jtag.Out) &= ~_Jtag.RST;// original
//        TDIL_DMA();
//        TDOsbwDma();    // ExitTCLK
//        TCLK_savedDma = false;
//    }
//    _EINT_FET();
}

// -----------------------------------------------------------------------------
void _hil_2w_StepPsa_Dma_Xv2(uint32_t length)
{
    void* dma2_tmp = DMA2SA;

    DMA1SA = (uint8_t*)DMA_TMSH_TDIL;
    DMA2SA = (uint8_t*)DMA_TMSL_TDIL;

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
void _hil_2w_StepPsa_Dma(uint32_t length)
{
    void* dma2_tmp = DMA2SA;

    DMA1SA = (uint8_t*)DMA_TMSH_TDIL;
    DMA2SA = (uint8_t*)DMA_TMSL_TDIL;

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
void _hil_2w_StepPsaTclkHigh_Dma(uint32_t length)
{
    void* dma2_tmp = DMA2SA;

    DMA1SA = (uint8_t*)DMA_TMSH_TDIH;
    DMA2SA = (uint8_t*)DMA_TMSL_TDIH;

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

void _hil_2w_ConfigureSpeed_Dma(uint16_t speed)
{
}

#ifdef MSP_FET
//extern uint8_t _hil_generic_Instr(uint8_t Instruction);
//extern uint8_t _hil_generic_SetReg_XBits08(uint8_t Data);
//extern uint16_t _hil_generic_SetReg_XBits16(uint16_t Data);
//extern uint32_t _hil_generic_SetReg_XBits20(uint32_t Data);
//extern uint32_t _hil_generic_SetReg_XBits32(uint32_t Data);
//
//extern uint8_t lastTestState;
//extern uint8_t lastResetState;
//extern int16_t _hil_SetVcc(uint16_t Vcc);

#endif

//#pragma optimize = medium
// -----------------------------------------------------------------------------
int16_t _hil_2w_BlowFuse_Dma(uint8_t targetHasTestVpp)
{
#ifdef MSP_FET
    if (protocol_id != SPYBIWIRE_SUBMCU)
    {
        uint8_t MSB = 0x80;;
        uint8_t Data = IR_EX_BLOW, i = 0;

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










void _sbwio(bool tms, bool tdi) {
    // With no `tclk` specified, use the value for TMS, so that the line stays constant
    // between registering the TMS value and outputting the TDI value
    _msp.sbwIO(tms, tms, tdi, false);
}

void _sbwio_r(bool tms, bool tdi) {
    // With no `tclk` specified, use the value for TMS, so that the line stays constant
    // between registering the TMS value and outputting the TDI value
    _msp.sbwIO(tms, tms, tdi, true);
}

void _sbwioTclk(bool tms, bool tclk) {
    _msp.sbwIO(tms, TCLK_savedDma, tclk, false);
    TCLK_savedDma = tclk;
}

uint64_t _read(uint8_t w) {
    uint8_t b[4];
    assert(w <= sizeof(b));
    _msp.sbwRead(b, w);
    
         if (w == 1)    return                                 b[0]<<0  ;
    else if (w == 2)    return                       b[0]<<8 | b[1]<<0  ;
    else if (w == 3)    return            b[0]<<16 | b[1]<<8 | b[2]<<0  ;
    else if (w == 4)    return b[0]<<24 | b[1]<<16 | b[2]<<8 | b[3]<<0  ;
    else                abort(); // invalid width
}

void sbw_Shift(uint64_t data, uint16_t width) {
    // MSPFETSim: custom implementation
    // <- Shift-DR
    for (uint64_t msb=(UINT64_C(1)<<(width-1)); msb; msb>>=1) {
        const bool tms = (msb & 1); // Last bit requires TMS=1
        const bool tdi = data&msb;
        _sbwio(tms, tdi);
    }
    // <- Exit1-DR
    
    // Return to Run-Test/Idle state
    _sbwio(1, TCLK_savedDma);
    _sbwio(0, TCLK_savedDma);
}

uint64_t sbw_Shift_R(uint64_t data, uint16_t width) {
    // MSPFETSim: custom implementation
    // <- Shift-DR
    for (uint64_t msb=(UINT64_C(1)<<(width-1)); msb; msb>>=1) {
        const bool tms = (msb & 1); // Last bit requires TMS=1
        const bool tdi = data&msb;
        _sbwio_r(tms, tdi);
    }
    // <- Exit1-DR
    
    // Return to Run-Test/Idle state
    // If we're reading a 20-bit address, read 4 extra dummy bits so we have
    // 3 full bytes of data enqueued (24 bits instead of 20 bits)
    switch (width) {
    case F_ADDR:
        _sbwio_r(1, TCLK_savedDma);
        _sbwio_r(0, TCLK_savedDma);
        // <- Run-Test/Idle
        // 2 extra cycles in the Run-Test/Idle state to read 2 more bits
        _sbwio_r(0, TCLK_savedDma);
        _sbwio_r(0, TCLK_savedDma);
        break;
    default:
        _sbwio(1, TCLK_savedDma);
        _sbwio(0, TCLK_savedDma);
        // <- Run-Test/Idle
        break;
    }
    
    uint64_t tdo = _read((width+7)/8);
    if (width == F_ADDR) {
        // For 20-bit addresses, scramble the result because the rest of
        // the TI codebase expects addresses to be scrambled for some
        // reason. When the caller unscrambles our result, they get the
        // value we started with, [1b]:
        // 
        //      [1a]        [1b]        [2]         [3]
        //   0xABCDEF -> 0x0ABCDE -> 0x0EABCD -> 0x0ABCDE
        // 
        // As shown above, we read [1a], remove the bottom 4 dummy bits
        // to get [1b] (see comment above), scramble and return [2], and
        // the caller de-scrambles to get [3].
        tdo = ((tdo&0x0000F0)<<12) | ((tdo&0xFFFF00)>>8);
    }
    return tdo;
}
