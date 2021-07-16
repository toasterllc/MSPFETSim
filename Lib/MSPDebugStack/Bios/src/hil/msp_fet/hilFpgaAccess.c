/*
 * Copyright (C) 2007 - 2014 Texas Instruments Incorporated - http://www.ti.com/
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

#include "archFpga.h"
#include "arch.h"
#include "hilDelays.h"
#include "hilFpgaAccess.h"
#include "hw_compiler_specific.h"

// structs representing ports for FPGA communication
s_FPGA_PA FPGA_PORTA;
s_FPGA_PB FPGA_PORTB;

unsigned char lastTestState = 0;
unsigned char lastResetState= 0;

static struct jtag _Jtag = {0};

extern unsigned short gprotocol_id;
unsigned short static volatile TimeOut = FET_FALSE;

void hil_fpga_init(void)
{
    // Port1
    //  P1.0 -> DATA0[0]
    //  P1.1 -> DATA0[1]
    //  P1.2 -> DATA0[2]
    //  P1.3 -> DATA0[3]
    //  P1.4 -> DATA0[4]
    //  P1.5 -> DATA0[5]
    //  P1.6 -> DATA0[6]
    //  P1.7 -> DATA0[7]
    P1DIR |= (BIT0+BIT1+BIT2+BIT3+BIT4+BIT5+BIT6+BIT7);	// set pins to output direction
    P1SEL &= ~(BIT0+BIT1+BIT2+BIT3+BIT4+BIT5+BIT6+BIT7);

    // Port2
    //  P2.0 <- DCDC_PULSE
    //  P2.1 <- UART_RTS
    //  P2.2 -> CMD[0]
    //  P2.3 -> CMD[1]
    //  P2.4 -> CMD[2]
    //  P2.5 -> CMD[3]
    //  P2.6 -> WR_TRIG
    //  P2.7 -> SYS_CLK
    P2DIR |= (BIT2+BIT3+BIT4+BIT5+BIT6+BIT7); // set pins to output direction
    P2SEL &= ~(BIT2+BIT3+BIT4+BIT5+BIT6+BIT7);

    // Port3
    //  P3.0 <-> DATA1[0]
    //  P3.1 <-> DATA1[1]
    //  P3.2 <-> DATA1[2]
    //  P3.3 <-> DATA1[3]
    //  P3.4 <-> DATA1[4]
    //  P3.5 <-> DATA1[5]
    //  P3.6 <-> DATA1[6]
    //  P3.7 <-> DATA1[7]
    P3DIR |= (BIT0+BIT1+BIT2+BIT3+BIT4+BIT5+BIT6+BIT7);	// set pins initially to output direction
    P3SEL &= ~(BIT0+BIT1+BIT2+BIT3+BIT4+BIT5+BIT6+BIT7);

    // Port4
    //  P4.0 <-> DATA1[8]
    //  P4.1 <-> DATA1[9]
    //  P4.2 <-> DATA1[10]
    //  P4.3 <-> DATA1[11]
    //  P4.4 <-> DATA1[12]
    //  P4.5 <-> DATA1[13]
    //  P4.6 <-> DATA1[14]
    //  P4.7 <-> DATA1[15]
    P4DIR |= (BIT0+BIT1+BIT2+BIT3+BIT4+BIT5+BIT6+BIT7);	// set pins initially to output direction
    P4SEL &= ~(BIT0+BIT1+BIT2+BIT3+BIT4+BIT5+BIT6+BIT7);

    // Port5
    //  P5.0 -> VREF+ (output of reference voltage to ADC)
    //  P5.1 -> FPGA_RESET
    //  P5.2 -> VF2TEST_CTRL
    //  P5.3 -> LED1
    //  P5.4 -> VF2TDI_CTRL
    //  P5.5 -> TDIOFF_CTRL (0 = turns off TDI, after that ok to select VF for fuse blowing)
    //  P5.6 <- MCU_DMAE0 / RD_TRIG (DMA trigger input)
    //  P5.7 -> DCDC_IO1
    P5DIR |= (BIT1+BIT2+BIT4+BIT5);	// set pins initially to output direction
    P5DIR &= ~(BIT6);
    P5OUT |= (BIT5);
    P5OUT &= ~(BIT2+BIT4);

    // Port8
    //  P8.0 -> VCC_DT2TRGT_CTRL (control signal to switches to provide debug signals to target via JTAG.x)
    //  P8.1 -> IO_DIR
    //  P8.2 -> UART_TXD
    //  P8.3 <- UART_RXD
    //  P8.4 <- DCDC_IO0
    //  P8.5 -> HOST_SDA
    //  P8.6 -> HOST_SCL
    //  P8.7 -> VCC_SUPPLY2TRGT_CTRL (DCDC VCC to target VCC)
    P8OUT |= (BIT1);		    // set pins to '1'
    P8DIR |= (BIT1);	        // set pins initially to output direction

    // Port9
    //  P9.0 -> FPGA_TCK
    //  P9.1 -> FPGA_TDI
    //  P9.2 -> FPGA_TMS
    //  P9.3 <- FPGA_TDO
    //  P9.4 -> FPGA_TRST
    //  P9.5 -> UART_CTS
    //  P9.6 -> VCC_DT2SUPPLY_CTRL (DT voltage to target VCC)
    //  P9.7 -> PWM_SETVF (to fuse blow circuit)
    P9OUT |= BIT0+BIT1+BIT2+BIT4;
    P9OUT &= ~(BIT3);
    P9DIR |= BIT0+BIT1+BIT2+BIT4+BIT7; // TDO is in, all others out
    P9DIR &= ~(BIT3);
    P9REN |= BIT3;  // TDO with pulldown


    // SYS_CLK: Output 20MHz MCLK on P2.7
    _DINT_FET();                                // Disable Interrupts before altering Port Mapping registers
    PMAPPWD = 0x02D52;                        // Enable Write-access to modify port mapping registers
    PMAPCTL = PMAPRECFG;                      // Allow reconfiguration during runtime
    P2MAP7 = PM_MCLK;
    PMAPPWD = 0;                              // Disable Write-Access to modify port mapping registers
    _EINT_FET();                     // Re-enable all interrupts

    // Reset FPGA
    FPGA_RESET_DEASSERT;
    __delay_cycles(100);
    FPGA_RESET_ASSERT;

    // Reset the internal structure
    FPGA_PORTA.all = 0;
    FPGA_PORTA.bit.CMD = 0xF;
    FPGA_CMD_DATA0_PORT_OUT = FPGA_PORTA.all;

    // Enable SYS_CLK
    __delay_cycles(100);
    FPGA_SYS_CLK_START;

    __delay_cycles(10000);
    FPGA_RESET_DEASSERT;

    lastTestState = 0;
    lastResetState= 1;

    TimeOut = FET_FALSE;
}

static void startWdtTimeOutCounter(unsigned long timeout)
{
    TimeOut = FET_FALSE;
    // Start WDT timer use SMCLK 20 MHz, timer mode, and clear timer, timout is 6.4 s
    WDTCTL = WDTPW + WDTSSEL__SMCLK + WDTCNTCL + WDTTMSEL + timeout;
    SFRIE1 |= WDTIE;
    SFRIFG1 &= ~WDTIFG;
}

static void stopWdtTimeOutCounter()
{
   // Stop watchdog timer
    WDTCTL = WDTPW + WDTHOLD;
    SFRIE1 &= ~WDTIE;
    SFRIFG1 &= ~WDTIFG;
}

void _hil_FpgaAccess_setTimeOut(unsigned short state)
{
    TimeOut = state;

    // Reset FPGA
    FPGA_RESET_DEASSERT;
    __delay_cycles(100);
    FPGA_RESET_ASSERT;

    // Enable SYS_CLK
    __delay_cycles(100);
    FPGA_SYS_CLK_START;

    __delay_cycles(10000);
    FPGA_RESET_DEASSERT;

    lastTestState = 0;
    lastResetState= 1;
}

void initJtagBypass(struct jtag tmp)
{
    _Jtag = tmp;
}

//enable bypass
void hil_fpga_enable_bypass()
{
    FPGA_BYPASS_DIR_CTRL_PORT_DIR = FPGA_BYPASS_DIR_CTRL_TEST_BIT + FPGA_BYPASS_DIR_CTRL_RST_BIT + FPGA_BYPASS_DIR_CTRL_TDO_BIT  + FPGA_BYPASS_DIR_CTRL_TDI_BIT + FPGA_BYPASS_DIR_CTRL_TMS_BIT + FPGA_BYPASS_DIR_CTRL_TCK_BIT;

    /*------------------------ test pin state handling ----------------------*/
    if (lastTestState)
    {
        (*_Jtag.Out) |= _Jtag.TST;
    }
    else
    {
        (*_Jtag.Out) &= ~_Jtag.TST;
    }

    /*------------------------ RST pin state handling ----------------------*/
    if(lastResetState)
    {
        (*_Jtag.Out) |= _Jtag.RST;
    }
    else
    {
        (*_Jtag.Out) &= ~_Jtag.RST;
    }

    if( gprotocol_id == SPYBIWIRE || gprotocol_id == SPYBIWIRE_MSP_FET)
    {
        (*_Jtag.DIRECTION) = _Jtag.TST + _Jtag.RST;
        FPGA_BYPASS_DIR_CTRL_PORT_OUT = _Jtag.TST + _Jtag.RST;
    }
    if( gprotocol_id == SPYBIWIREJTAG || gprotocol_id == JTAG || gprotocol_id == JTAG_432 || gprotocol_id == SWD_432)
    {
        (*_Jtag.Out) |= (_Jtag.TCK + _Jtag.TDI);
        (*_Jtag.DIRECTION) = _Jtag.TST + _Jtag.RST + _Jtag.TDI + _Jtag.TMS + _Jtag.TCK;
        FPGA_BYPASS_DIR_CTRL_PORT_OUT = _Jtag.TST + _Jtag.RST + _Jtag.TDI + _Jtag.TMS + _Jtag.TCK;
    }

    // Enable bypass mode
    hil_fpga_write_cmd_data0(FPGA_CMD_BYPASS, 1);

    FPGA_IO_DIR_PORT_OUT &= ~FPGA_IO_DIR_BIT;

    _hil_Delay_1ms(10);
}


// Leave bypass
void hil_fpga_disable_bypass(void)
{
    if((*_Jtag.Out) & _Jtag.TST)
    {
        lastTestState  = 1;
    }
    else
    {
        lastTestState  = 0;
    }
    if((*_Jtag.Out) & _Jtag.RST)
    {
        lastResetState = 1;
    }
    else
    {
        lastResetState = 0;
    }

    FPGA_IO_DIR_PORT_OUT |= FPGA_IO_DIR_BIT;
    // Disable bypass mode
    hil_fpga_write_cmd_data0(FPGA_CMD_BYPASS, 0);
    _hil_Delay_1us(10);
}

void hil_fpga_write_cmd_data0(unsigned char cmd, unsigned char data0)
{
    // Assemble PORTA data
    FPGA_PORTA.bit.CMD      = cmd;
    FPGA_PORTA.bit.DATA0    = data0;
    FPGA_PORTA.bit.WR_TRIG ^= 1;

    startWdtTimeOutCounter(WDTIS__128M);
    // Wait until RD_TRIG = 1 and FPGA is ready to accept data
    while ((!FPGA_RD_TRIG_IS_ASSERTED))
    {
         if(TimeOut){break;}
    }
    stopWdtTimeOutCounter();

    // Write to FPGA
    FPGA_CMD_DATA0_PORT_OUT = FPGA_PORTA.all;
}


void hil_fpga_write_cmd_data0_432(unsigned char cmd, unsigned char data0)
{
    // Assemble PORTA data
    FPGA_PORTA.bit.CMD      = cmd;
    FPGA_PORTA.bit.DATA0    = data0;
    FPGA_PORTA.bit.WR_TRIG ^= 1;

    // Wait until RD_TRIG = 1 and FPGA is ready to accept data
    while ((!FPGA_RD_TRIG_IS_ASSERTED));

    // Write to FPGA
    FPGA_CMD_DATA0_PORT_OUT = FPGA_PORTA.all;
}

// Write CMD, DATA0 and DATA1 to FPGA
void hil_fpga_write_cmd_data0_data1(unsigned char cmd, unsigned char data0, unsigned short data1)
{
    // Set DATA1 bus to output direction
    FPGA_IO_DIR_PORT_OUT |= FPGA_IO_DIR_BIT;
    FPGA_DATA1_PORT_DIR = 0xFFFF;

    // Assemble PORTA data
    FPGA_PORTA.bit.CMD      = cmd;
    FPGA_PORTA.bit.DATA0    = data0;
    FPGA_PORTA.bit.WR_TRIG ^= 1;

    startWdtTimeOutCounter(WDTIS__128M);
    // Wait until RD_TRIG = 1 and FPGA is ready to accept data
    while (!(FPGA_RD_TRIG_PORT_IN & FPGA_RD_TRIG_BIT))
    {
        if(TimeOut){break;}
    }
    stopWdtTimeOutCounter();

    // Write to FPGA
    FPGA_DATA1_PORT_OUT     = data1;
    FPGA_CMD_DATA0_PORT_OUT = FPGA_PORTA.all;
}

// Write CMD, DATA0 and DATA1 to FPGA
void hil_fpga_write_cmd_data0_data1_count_432(unsigned char cmd, unsigned char data0, unsigned short data1Buf[], unsigned char count)
{
    // Set DATA1 bus to output direction
    FPGA_IO_DIR_PORT_OUT |= FPGA_IO_DIR_BIT;
    FPGA_DATA1_PORT_DIR = 0xFFFF;

    // Assemble PORTA data
    FPGA_PORTA.bit.CMD      = cmd;
    FPGA_PORTA.bit.DATA0    = data0;

    unsigned char i =0;
    while(i <  count)
    {
        // Write to FPGA
        FPGA_DATA1_PORT_OUT     = data1Buf[i++];
        FPGA_PORTA.bit.WR_TRIG ^= 1;
        FPGA_CMD_DATA0_PORT_OUT = FPGA_PORTA.all;
    }
}

// Read data from FPGA
void hil_fpga_read_data_432(unsigned char count, unsigned short *buf)
{
    // Set DATA1 bus to input direction
    FPGA_DATA1_PORT_DIR = 0x0000;
    FPGA_IO_DIR_PORT_OUT &= ~FPGA_IO_DIR_BIT;

    while (count)
    {
        // Wait for word on DATA1
        while (!(FPGA_RD_TRIG_PORT_IN & FPGA_RD_TRIG_BIT));

        // Store word
        *(buf+(--count)) = FPGA_DATA1_PORT_IN;
        // Request next word on DATA1
        if (count)
        {
            FPGA_IO_DIR_PORT_OUT ^= FPGA_IO_DIR_BIT;
            FPGA_IO_DIR_PORT_OUT ^= FPGA_IO_DIR_BIT;
        }
    }
    // Set DATA1 to output
    FPGA_IO_DIR_PORT_OUT |= FPGA_IO_DIR_BIT;
    FPGA_DATA1_PORT_DIR = 0xFFFF;
}


// Write CMD, DATA0 and DATA1 to FPGA
void hil_fpga_write_cmd_data0_data1_count(unsigned char cmd, unsigned char data0, unsigned short data1Buf[], unsigned char count)
{
    //unsigned short i = 0;
    // Set DATA1 bus to output direction
    FPGA_IO_DIR_PORT_OUT |= FPGA_IO_DIR_BIT;
    FPGA_DATA1_PORT_DIR = 0xFFFF;

    // Assemble PORTA data
    FPGA_PORTA.bit.CMD      = cmd;
    FPGA_PORTA.bit.DATA0    = data0;

    // Wait until RD_TRIG = 1 and FPGA is ready to accept data
    startWdtTimeOutCounter(WDTIS__128M);
    while (!(FPGA_RD_TRIG_PORT_IN & FPGA_RD_TRIG_BIT))
    {
        if(TimeOut){break;}
    }
    stopWdtTimeOutCounter();

    do
    {
        // Write to FPGA
        FPGA_DATA1_PORT_OUT     = data1Buf[count - 1];
        FPGA_PORTA.bit.WR_TRIG ^= 1;
        FPGA_CMD_DATA0_PORT_OUT = FPGA_PORTA.all;
        count--;
    } while(count);
}


// Read data from FPGA
void hil_fpga_read_data1(unsigned char count, unsigned short *buf)
{
    // Set DATA1 bus to input direction
    FPGA_DATA1_PORT_DIR = 0x0000;
    FPGA_IO_DIR_PORT_OUT &= ~FPGA_IO_DIR_BIT;

    while (count)
    {
        startWdtTimeOutCounter(WDTIS__128M);

        // Wait for word on DATA1
        while (!(FPGA_RD_TRIG_PORT_IN & FPGA_RD_TRIG_BIT))
        {
            if(TimeOut){break;}
        }
		stopWdtTimeOutCounter();

        // Store word
        *(buf+(count-1)) = FPGA_DATA1_PORT_IN;
        count--;
        // Request next word on DATA1
        if (count)
        {
            FPGA_IO_DIR_PORT_OUT ^= FPGA_IO_DIR_BIT;
            FPGA_IO_DIR_PORT_OUT ^= FPGA_IO_DIR_BIT;
        }
    }
    // Set DATA1 to output
    FPGA_IO_DIR_PORT_OUT |= FPGA_IO_DIR_BIT;
    FPGA_DATA1_PORT_DIR = 0xFFFF;
}

// Hi-Z JTAG signals to target, except TEST and RST (actively drive 0)
void hil_fpga_power_up_target(void)
{
    // Enable VCC_SUPPLY and VCC_DT
    P8DIR |= (BIT0 + BIT7);
    P8OUT |= (BIT0 + BIT7);
    _hil_Delay_1ms(1);
}

unsigned short hil_fpga_get_version(void)
{
    unsigned short version = 0;

    // This is a bit complicated, due to the configuration register returning a value
    hil_fpga_write_cmd_data0(FPGA_CMD_VERSION, 0);

    // Set DATA1 bus to input direction
    FPGA_DATA1_PORT_DIR = 0x0000;
    FPGA_IO_DIR_PORT_OUT &= ~FPGA_IO_DIR_BIT;

    // Wait for data valid
    while (!FPGA_RD_TRIG_IS_ASSERTED);

    // Store word
    version = FPGA_DATA1_PORT_IN;

    // Remove VERSION command
    FPGA_PORTA.bit.CMD      = 0;
    FPGA_PORTA.bit.DATA0    = 0;
    FPGA_CMD_DATA0_PORT_OUT = FPGA_PORTA.all;

    // Set DATA1 to output
    FPGA_IO_DIR_PORT_OUT |= FPGA_IO_DIR_BIT;
    FPGA_DATA1_PORT_DIR = 0xFFFF;

    return version;
}
