/**
* \ingroup MODULHIL
*
* \file hil_2w.c
*
* \brief 2 wire implementation of hil interface
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

#include "hw_compiler_specific.h"
#include "HalGlobalVars.h"
#include "arch.h"
#include "edt.h"


#define  PIN_TMS            BIT0
#define  PIN_TDI            BIT1
#define  PIN_TDO            BIT2
#define  PIN_TCK            BIT3

#define  CLEAR_INSTR        0
#define  EXEC_FUSE_BLOW     1

const unsigned char DMA_TMSH_TDIH[] = {/*TMS Slot*/ PIN_TDI|PIN_TCK, PIN_TDI, PIN_TDI|PIN_TCK,
                                       /*TDI Slot*/ PIN_TDI|PIN_TCK, PIN_TDI, PIN_TDI|PIN_TCK};

const unsigned char DMA_TMSH_TDIL[] = {/*TMS Slot*/ PIN_TDI|PIN_TCK, PIN_TDI, PIN_TDI|PIN_TCK,
                                       /*TDI Slot*/ PIN_TCK,               0, PIN_TCK};

const unsigned char DMA_TMSL_TDIH[] = {/*TMS Slot*/ PIN_TCK,               0, PIN_TCK,
                                       /*TDI SLot*/ PIN_TDI|PIN_TCK, PIN_TDI, PIN_TDI|PIN_TCK};

const unsigned char DMA_TMSL_TDIL[] = {/*TMS Slot*/ PIN_TCK,               0, PIN_TCK,
                                       /*TDI Slot*/ PIN_TCK,               0, PIN_TCK};

unsigned char TCLK_saved;
unsigned char current_Instr;

// defined in hil.c
extern unsigned char *tstctrl_port_;
extern unsigned char entdi2tdo_;
extern void SetVpp(long voltage);
unsigned char prevInstruction;

INLINE(forced)
void DMA1sbw(void)
{
    DMA1CTL |= DMAEN;
    DMA1CTL |= DMAREQ;    // DMA1 FIRE!!!
    TDOsbw
}

INLINE(forced)
void DMA2sbw(void)
{
    DMA2CTL |= DMAEN;
    DMA2CTL |= DMAREQ;    // DMA2 FIRE!!!
    TDOsbw
}

unsigned char _hil_2w_GetPrevInstruction()
{
    return prevInstruction;
}

INLINE(forced)
void restoreTCLK(void)
{
    if (TCLK_saved & sbwdato)
    {
        DMA1SA = (unsigned short)DMA_TMSH_TDIH;
        DMA1sbw();
        DMA1SA = (unsigned short)DMA_TMSL_TDIH;
        DMA1sbw();
    }
    else
    {
        DMA1SA = (unsigned short)DMA_TMSH_TDIL;
        DMA1sbw();
        // TMSL_TDIL is preloaded;
        DMA2sbw();
    }
}

INLINE(forced)
unsigned long long sbw_Shift(unsigned long long Data, unsigned long long Bits)
{
    unsigned long long  TDOvalue = 0;
    unsigned long long MSB = 0x0000000000000000;

    if(Bits == 0)
    {
        // 20 bit
        MSB = 0x000080000ull;
    }
    else if(Bits == 1)
    {
        // 32 bit
        MSB = 0x80000000ull;
    }
    else
    {
        // 64 bit
        MSB = 0x8000000000000000ull;
    }
    do
    {
        if (MSB & 1)                       // Last bit requires TMS=1
        {
            if(Data & MSB)
            {
                DMA1SA = (unsigned int)DMA_TMSH_TDIH;
            }
            else
            {
                DMA1SA = (unsigned int)DMA_TMSH_TDIL;
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
        TDO_RD
    }
    while(MSB >>= 1);
    restoreTCLK();
    return(TDOvalue);
}



INLINE(forced)
unsigned short sbw_Shift16(unsigned short Data)
{
    unsigned short TDOvalue = 0;
    unsigned short MSB = 0x8000;

    do
    {
        if ((MSB & 1) == 1)                       // Last bit requires TMS=1
        {
            if(Data & MSB)
            {
                DMA1SA = (unsigned int)DMA_TMSH_TDIH;
            }
            else
            {
                DMA1SA = (unsigned int)DMA_TMSH_TDIL;
            }
            DMA1CTL |= DMAEN;
            DMA1CTL |= DMAREQ;    // DMA1 FIRE!!!
        }
        else
        {
            if(Data & MSB)
            {
                // TMSL_TDIH is preloaded
                DMA1CTL |= DMAEN;
                DMA1CTL |= DMAREQ;    // DMA1 FIRE!!!
            }
            else
            {
                // TMSL_TDIL is preloaded;
                DMA2CTL |= DMAEN;
                DMA2CTL |= DMAREQ;    // DMA1 FIRE!!!
            }
        }
        TDO_RD
    }
    while(MSB >>= 1);
    restoreTCLK();
    return(TDOvalue);
}



INLINE(forced)
unsigned char sbw_Shift8(unsigned char Data)
{
    unsigned char TDOvalue = 0;
    unsigned char  MSB = 0x80;

    do
    {
        if ((MSB & 1) == 1)                       // Last bit requires TMS=1
        {
            if(Data & MSB)
            {
                DMA1SA = (unsigned int)DMA_TMSH_TDIH;
            }
            else
            {
                DMA1SA = (unsigned int)DMA_TMSH_TDIL;
            }
            DMA1CTL |= DMAEN;
            DMA1CTL |= DMAREQ;    // DMA1 FIRE!!!
        }
        else
        {
            if(Data & MSB)
            {
                // TMSL_TDIH is preloaded
                DMA1CTL |= DMAEN;
                DMA1CTL |= DMAREQ;    // DMA1 FIRE!!!
            }
            else
            {
                // TMSL_TDIL is preloaded
                DMA2CTL |= DMAEN;
                DMA2CTL |= DMAREQ;    // DMA1 FIRE!!!
            }
        }
        TDO_RD
    }
    while(MSB >>= 1);

    // after the execute fuse blow instruction is shifted into the target, the SBWTDIO
    // pin must be low until Vpp is settled and a high transition of SBWTDIO burns the fuse
    if (current_Instr == EXEC_FUSE_BLOW)
    {
      TCLK_saved = 0;
      current_Instr = CLEAR_INSTR;
    }
    restoreTCLK();
    return(TDOvalue);
}



// -----------------------------------------------------------------------------
short _hil_2w_TapReset(void)
{
    unsigned short i;

    DMA1SA = (unsigned short)DMA_TMSH_TDIH;
    // Reset JTAG FSM
    for (i = 0; i < 6; i++)      // 6 is nominal
    {
        // TMSH_TDIH is preloaded
        DMA1sbw();
    }
    DMA1SA = (unsigned short)DMA_TMSL_TDIH;
    DMA1sbw();
    return 0;
}

// -----------------------------------------------------------------------------
short _hil_2w_CheckJtagFuse(void)
{
    unsigned short dma2_tmp = DMA2SA;

    DMA1SA = (unsigned short)DMA_TMSH_TDIH;
    DMA2SA = (unsigned short)DMA_TMSL_TDIH;

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
    return 0;
}

// -----------------------------------------------------------------------------
unsigned char _hil_2w_Instr(unsigned char Instruction)
{
    prevInstruction = Instruction;
    // JTAG FSM state = Run-Test/Idle
    if (TCLK_saved & sbwdato) //PrepTCLK
    {
        DMA1SA = (unsigned int)DMA_TMSH_TDIH;
        DMA1sbw();
    }
    else
    {
        DMA1SA = (unsigned int)DMA_TMSH_TDIL;
        DMA1sbw();
        DMA1SA = (unsigned int)DMA_TMSH_TDIH;
    }

    // JTAG FSM state = Select DR-Scan
    // TMSH_TDIH loaded in previous if/else
    DMA1sbw();
    // JTAG FSM state = Select IR-Scan
    DMA1SA = (unsigned int)DMA_TMSL_TDIH;
    DMA1sbw();
    // JTAG FSM state = Capture-IR
    DMA1sbw();
    // JTAG FSM state = Shift-IR, Shiftin TDI (8 bit)
    return(sbw_Shift8(Instruction));  // JTAG FSM state = Run-Test/Idle
}



unsigned char _hil_2w_SetReg_XBits08(unsigned char data)
{
    // JTAG FSM state = Run-Test/Idle
    if (TCLK_saved & sbwdato) //PrepTCLK
    {
        DMA1SA = (unsigned int)DMA_TMSH_TDIH;
    }
    else
    {
        DMA1SA = (unsigned int)DMA_TMSH_TDIL;
    }
    DMA1sbw();

    DMA1SA = (unsigned int)DMA_TMSL_TDIH;
    // JTAG FSM state = Select DR-Scan
    DMA1sbw();
    // JTAG FSM state = Capture-DR
    DMA1sbw();
    // JTAG FSM state = Shift-DR, Shiftin TDI (16 bit)
    return (sbw_Shift8(data));
    // JTAG FSM state = Run-Test/Idle
}



unsigned short _hil_2w_SetReg_XBits16(unsigned short data)
{
    // JTAG FSM state = Run-Test/Idle
    if (TCLK_saved & sbwdato) //PrepTCLK
    {
        DMA1SA = (unsigned int)DMA_TMSH_TDIH;
    }
    else
    {
        DMA1SA = (unsigned int)DMA_TMSH_TDIL;
    }
    DMA1sbw();

    DMA1SA = (unsigned int)DMA_TMSL_TDIH;
    // JTAG FSM state = Select DR-Scan
    DMA1sbw();
    // JTAG FSM state = Capture-DR
    DMA1sbw();
    // JTAG FSM state = Shift-DR, Shiftin TDI (16 bit)
    return (sbw_Shift16(data));
    // JTAG FSM state = Run-Test/Idle
}



unsigned long _hil_2w_SetReg_XBits20(unsigned long data)
{
    unsigned long tmp;

    // JTAG FSM state = Run-Test/Idle
    if (TCLK_saved & sbwdato) //PrepTCLK
    {
        DMA1SA = (unsigned int)DMA_TMSH_TDIH;
    }
    else
    {
        DMA1SA = (unsigned int)DMA_TMSH_TDIL;
    }
    DMA1sbw();

    DMA1SA = (unsigned int)DMA_TMSL_TDIH;
    // JTAG FSM state = Select DR-Scan
    DMA1sbw();
    // JTAG FSM state = Capture-DR
    DMA1sbw();
    // JTAG FSM state = Shift-DR, Shiftin TDI (16 bit)
    // de-scramble upper 4 bits if it was a 20bit shift
    tmp = sbw_Shift(data, 0);
    tmp = ((tmp >> 4) | (tmp << 16)) & 0x000FFFFF;
    return (tmp);
    // JTAG FSM state = Run-Test/Idle
}



unsigned long _hil_2w_SetReg_XBits32(unsigned long data)
{
    // JTAG FSM state = Run-Test/Idle
    if (TCLK_saved & sbwdato) //PrepTCLK
    {
        DMA1SA = (unsigned int)DMA_TMSH_TDIH;
    }
    else
    {
        DMA1SA = (unsigned int)DMA_TMSH_TDIL;
    }
    DMA1sbw();

    DMA1SA = (unsigned int)DMA_TMSL_TDIH;
    // JTAG FSM state = Select DR-Scan
    DMA1sbw();
    // JTAG FSM state = Capture-DR
    DMA1sbw();
    // JTAG FSM state = Shift-DR, Shiftin TDI (16 bit)
    return (sbw_Shift(data, 1));
    // JTAG FSM state = Run-Test/Idle
}


unsigned long long _hil_2w_SetReg_XBits64(unsigned long long data)
{
    // JTAG FSM state = Run-Test/Idle
    if (TCLK_saved & sbwdato) //PrepTCLK
    {
        DMA1SA = (unsigned int)DMA_TMSH_TDIH;
    }
    else
    {
        DMA1SA = (unsigned int)DMA_TMSH_TDIL;
    }
    DMA1sbw();

    DMA1SA = (unsigned int)DMA_TMSL_TDIH;
    // JTAG FSM state = Select DR-Scan
    DMA1sbw();
    // JTAG FSM state = Capture-DR
    DMA1sbw();
    // JTAG FSM state = Shift-DR, Shiftin TDI (16 bit)
    return (sbw_Shift(data, 2));
    // JTAG FSM state = Run-Test/Idle
}

//! \brief This function executes an SBW2 64BIT Data SHIFT (DR-SHIFT) in the case,
//! that the first 8 bits show a valid data capture from the JSTATE register. In case
//! of no valid capture the shift is ended after the first 8 bits. Timeout could be
//! used to set the function run count.
//! \param[in]  Data to be shifted into target device
//! \return Value shifted out of target device JTAG module
unsigned long long _hil_2w_SetReg_XBits8_64(unsigned long long Data, unsigned short loopCount, unsigned short JStateVersion)
{
    unsigned long long TDOvalue = 0x0;
    unsigned short timeout = loopCount;
    unsigned char DataState = 0;
    unsigned short currentDeviceState = 0;
    unsigned short syncBrokenCount = loopCount%2;

    do
    {
        TDOvalue = (_hil_2w_SetReg_XBits64(Data));
        // Mask out all not needed bits for device state detection
        currentDeviceState = ((TDOvalue >> 56) & JSTATE_FLOW_CONTROL_BITS);

        // check, if BIT[63, 62, 61, 60, 59  58, 57, 56, ] & 0x4 (BP_HIT) == TRUE
        if(currentDeviceState & JSTATE_BP_HIT)
        {
            // reload Jstate IR
            _hil_2w_Instr(IR_JSTATE_ID);
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
            _hil_2w_Instr(IR_JSTATE_ID);
            DataState = INVALID_DATA;
        }
        /*PG2.0 && PG2.1 frozen Sync detection*/
       else if (((currentDeviceState & JSTATE_SYNC_BROKEN_MASK) ==  JSTATE_SYNC_BROKEN_MCLK)
                    ||((currentDeviceState & JSTATE_SYNC_BROKEN_MASK) ==  JSTATE_SYNC_BROKEN_MCLK_PGACT)
                    ||((currentDeviceState & JSTATE_SYNC_BROKEN_MASK) ==  JSTATE_SYNC_BROKEN_PGACT))
        {
           if(syncBrokenCount > 0 && JStateVersion >= 0x21)

           { // Only working for PG2.1, do not harm if executed on PG2.0 – but will not create any effect only consume time
                unsigned long current3VtestReg = 0;

                // read current 3VtestReg value
                _hil_2w_Instr(IR_TEST_REG);
                current3VtestReg = _hil_2w_SetReg_XBits32(0);
                // set bit 25 high to rest sync
                current3VtestReg |= 0x2000000;
                _hil_2w_SetReg_XBits32(current3VtestReg);
                // set bit 25 low reset sync done
                current3VtestReg &= ~0x2000000;
                _hil_2w_SetReg_XBits32(current3VtestReg);

                _hil_2w_Instr(IR_JSTATE_ID);
                syncBrokenCount --;
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
                && (currentDeviceState &    JSTATE_LPM_X_FIVE) != JSTATE_LPM_X_FIVE
                &&  currentDeviceState !=   0x00
                &&  currentDeviceState !=   0x02)
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
        _hil_2w_Instr(IR_JSTATE_ID);
    }
    return(TDOvalue);
}

// -----------------------------------------------------------------------------
void _hil_2w_Tclk(unsigned char state)
{
    __disable_interrupt();
    if (TCLK_saved & sbwdato) //PrepTCLK
    {
        TMSLDH
    }
    else
    {
        TMSL
    }

    if(state)
    {
        JTAGOUT |= sbwdato;
        TDIH TDOsbw    // ExitTCLK
        TCLK_saved = sbwdato;
    }
    else
    {
        JTAGOUT &= ~sbwdato;// original
        TDIL TDOsbw    // ExitTCLK
        TCLK_saved = ~sbwdato;
    }
    //  __enable_interrupt() are called in TDOsbw
}

void _hil_2w_StepPsa_Xv2(unsigned long length)
{
    unsigned short dma2_tmp = DMA2SA;

    DMA1SA = (unsigned short)DMA_TMSH_TDIL;
    DMA2SA = (unsigned short)DMA_TMSL_TDIL;

    while(length--)
    {
        TCLKset0
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
        TCLKset1
    }
    // Restore the prvious DMA2SA value
    DMA2SA = dma2_tmp;
}

// -----------------------------------------------------------------------------
void _hil_2w_StepPsa(unsigned long length)
{
    unsigned short dma2_tmp = DMA2SA;

    DMA1SA = (unsigned short)DMA_TMSH_TDIL;
    DMA2SA = (unsigned short)DMA_TMSL_TDIL;

    while(length--)
    {
        TCLKset1
        TCLKset0
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
    }
    // Restore the prvious DMA2SA value
    DMA2SA = dma2_tmp;
}

// -----------------------------------------------------------------------------
void _hil_2w_StepPsaTclkHigh(unsigned long length)
{
    unsigned short dma2_tmp = DMA2SA;

    DMA1SA = (unsigned short)DMA_TMSH_TDIH;
    DMA2SA = (unsigned short)DMA_TMSL_TDIH;

    while(length--)
    {
        TCLKset1
        TCLKset0
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
    TCLKset1
}

// -----------------------------------------------------------------------------
short _hil_2w_BlowFuse(unsigned char targetHasTestVpp)
{
    /// \ref SLAU320A July 2010 - Revised November 2010

    prepare_blow(); // initialize fuse blowing
    IHIL_Delay_1ms(1);

    // set current instruction to ensure that the SBWTDIO pin does not go high after
    // the instruction is shifted into the target
    current_Instr = EXEC_FUSE_BLOW;
    ex_blow();      // send fuse blow instruction

    // After the IR_EX_BLOW instruction is shifted in via SBW, one more TMS_SLOT must be performed
    TMSL

    // apply fuse blow voltage
    SetVpp(1);

    // Taking SBWTDIO high as soon as VPP has been settled blows the fuse
    JTAGOUT |= sbwdato;
    IHIL_Delay_1ms(3);
    SetVpp(0);

    // now perform a BOR via JTAG - we loose control of the device then...
    test_reg();
    SetReg_32Bits(0x00000200);

    return 0;
}

/* EOF */
