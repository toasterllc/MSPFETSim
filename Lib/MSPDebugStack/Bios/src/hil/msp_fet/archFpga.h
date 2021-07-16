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

#include "msp430.h"
#define uController_uif

#ifndef _FPGA_ARCH_H_
#define _FPGA_ARCH_H_
// ------------------------------------------------------------------------------
// FPGA

// RESET
#define FPGA_RESET_BIT			    ( BIT1 )
#define FPGA_RESET_PORT_DIR			( P5DIR )
#define FPGA_RESET_PORT_OUT			( P5OUT )
#define FPGA_RESET_ASSERT           { FPGA_RESET_PORT_OUT &= ~FPGA_RESET_BIT; }
#define FPGA_RESET_DEASSERT         { FPGA_RESET_PORT_OUT |= FPGA_RESET_BIT; }

#define TRIGGER_LA                  { P6OUT |= BIT7; __delay_cycles(10); P6OUT &= ~BIT7; }
#define TRIGGER_ONCE                { P6OUT ^= BIT7; }

// DCDC output to target VCC
#define DCDC_VCC_BIT				( BIT7 )
#define DCDC_VCC_PORT_OUT			( P8OUT )
#define DCDC_VCC_ENABLE				{ DCDC_VCC_PORT_OUT |= DCDC_VCC_BIT; }
#define DCDC_VCC_DISABLE			{ DCDC_VCC_PORT_OUT &= ~DCDC_VCC_BIT; }

// DT voltage to target VCC
#define DT_VCC_BIT				    ( BIT6 )
#define DT_VCC_PORT_OUT				( P9OUT )
#define DT_VCC_ENABLE				{ DT_VCC_PORT_OUT |= DT_VCC_BIT; }
#define DT_VCC_DISABLE				{ DT_VCC_PORT_OUT &= ~DT_VCC_BIT; }

// DT voltage to level shifters
#define DT_SIGNALS_BIT			    ( BIT0 )
#define DT_SIGNALS_PORT_OUT			( P8OUT )
#define DT_SIGNALS_ENABLE			{ DT_SIGNALS_PORT_OUT |= DT_SIGNALS_BIT; }
#define DT_SIGNALS_DISABLE			{ DT_SIGNALS_PORT_OUT &= ~DT_SIGNALS_BIT; }

// SYS_CLK
#define FPGA_SYS_CLK_BIT		    ( BIT7 )
#define FPGA_SYS_CLK_PORT_DIR		( P2DIR )
#define FPGA_SYS_CLK_PORT_SEL		( P2SEL )
#define FPGA_SYS_CLK_START			{ FPGA_SYS_CLK_PORT_SEL |= FPGA_SYS_CLK_BIT; }
#define FPGA_SYS_CLK_STOP			{ FPGA_SYS_CLK_PORT_SEL &= ~FPGA_SYS_CLK_BIT; }

// WR_TRIG
#define FPGA_WR_TRIG_BIT		    ( BIT6 )
#define FPGA_WR_TRIG_PORT_DIR		( P2DIR )
#define FPGA_WR_TRIG_PORT_OUT		( P2OUT )

// RD_TRIG
#define FPGA_RD_TRIG_BIT			( BIT6 )
#define FPGA_RD_TRIG_PORT_DIR		( P5DIR )
#define FPGA_RD_TRIG_PORT_IN		( P5IN )
#define FPGA_RD_TRIG_IS_ASSERTED 	( FPGA_RD_TRIG_PORT_IN & FPGA_RD_TRIG_BIT )

// CMD
#define FPGA_CMD_BITS			    ( BIT2 + BIT3 + BIT4 + BIT5 )
#define FPGA_CMD_PORT_DIR			( P2DIR )
#define FPGA_CMD_PORT_OUT			( P2OUT )

// DATA0
#define FPGA_DATA0_BITS		        ( 0xFF )
#define FPGA_DATA0_PORT_DIR			( P1DIR )
#define FPGA_DATA0_PORT_IN			( P1IN )
#define FPGA_DATA0_PORT_OUT			( P1OUT )

// CMD, DATA0, WR_TRIG
#define FPGA_CMD_DATA0_PORT_OUT     ( PAOUT )

// DATA1
#define FPGA_DATA1_BITS		        ( 0xFFFF )
#define FPGA_DATA1_PORT_DIR			( PBDIR )
#define FPGA_DATA1_PORT_IN			( PBIN )
#define FPGA_DATA1_PORT_OUT			( PBOUT )

// IO_DIR
#define FPGA_IO_DIR_BIT				( BIT1 )
#define FPGA_IO_DIR_PORT_DIR		( P8SEL )
#define FPGA_IO_DIR_PORT_OUT		( P8OUT )

// UART_TXD / UART_TXD (MSP-FET RXD input, JTAG.12)
#define FPGA_UART_TXD_BIT			( BIT3 )
#define FPGA_UART_TXD_PORT_SEL		( P8SEL )

// UART_RXD / UART_RXD (MSP-FET TXD output, JTAG.14)
#define FPGA_UART_RXD_BIT			( BIT2 )
#define FPGA_UART_RXD_PORT_SEL		( P8SEL )
#define FPGA_UART_RXD_PORT_OUT		( P8OUT )
#define FPGA_UART_RXD_HIGH			{ FPGA_UART_RXD_PORT_OUT |= FPGA_UART_RXD_BIT; }
#define FPGA_UART_RXD_LOW			{ FPGA_UART_RXD_PORT_OUT &= ~FPGA_UART_RXD_BIT; }

// UART_CTS / UART_CTS (MSP-FET "clear to send" output, JTAG.10)
#define FPGA_UART_CTS_BIT			( BIT5 )
#define FPGA_UART_CTS_PORT_DIR		( P9DIR )
#define FPGA_UART_CTS_PORT_OUT		( P9OUT )
#define FPGA_UART_CTS_HIGH			{ FPGA_UART_CTS_PORT_OUT |= FPGA_UART_CTS_BIT; }
#define FPGA_UART_CTS_LOW			{ FPGA_UART_CTS_PORT_OUT &= ~FPGA_UART_CTS_BIT; }

// UART_RTR / UART_RTR (MSP-FET "ready to receive" input, JTAG.13)
#define FPGA_UART_RTR_BIT			( BIT1 )
#define FPGA_UART_RTR_PORT_DIR		( P2DIR )
#define FPGA_UART_RTR_PORT_IN		( P2IN )
#define FPGA_UART_RTR_IS_HIGH		( FPGA_UART_RTR_PORT_IN & FPGA_UART_RTR_BIT )

// JTAG signals in bypass mode
#define FPGA_BYPASS_DIR_CTRL_TEST_BIT		( BIT5 )
#define FPGA_BYPASS_DIR_CTRL_RST_BIT		( BIT4 )
#define FPGA_BYPASS_DIR_CTRL_TDO_BIT		( BIT3 )
#define FPGA_BYPASS_DIR_CTRL_TDI_BIT		( BIT2 )
#define FPGA_BYPASS_DIR_CTRL_TMS_BIT		( BIT1 )
#define FPGA_BYPASS_DIR_CTRL_TCK_BIT		( BIT0 )
#define FPGA_BYPASS_DIR_CTRL_PORT_DIR		( P4DIR )
#define FPGA_BYPASS_DIR_CTRL_PORT_OUT		( P4OUT )
#define FPGA_BYPASS_DIR_CTRL_PORT_IN		( P4IN )

#define FPGA_BYPASS_TEST_BIT		( BIT5 )
#define FPGA_BYPASS_RST_BIT			( BIT4 )
#define FPGA_BYPASS_TDO_BIT			( BIT3 )
#define FPGA_BYPASS_TDI_BIT			( BIT2 )
#define FPGA_BYPASS_TMS_BIT			( BIT1 )
#define FPGA_BYPASS_TCK_BIT			( BIT0 )
#define FPGA_BYPASS_JTAG_PORT_DIR	( P3DIR )
#define FPGA_BYPASS_JTAG_PORT_OUT	( P3OUT )
#define FPGA_BYPASS_JTAG_PORT_IN	( P3IN )

#define FPGA_BYPASS_TEST_HIGH		{ FPGA_BYPASS_JTAG_SBW_PORT_OUT |= FPGA_BYPASS_TEST_BIT; }
#define FPGA_BYPASS_TEST_LOW		{ FPGA_BYPASS_JTAG_SBW_PORT_OUT &= ~FPGA_BYPASS_TEST_BIT; }
#define FPGA_BYPASS_RST_HIGH		{ FPGA_BYPASS_JTAG_SBW_PORT_OUT |= FPGA_BYPASS_RST_BIT; }
#define FPGA_BYPASS_RST_LOW			{ FPGA_BYPASS_JTAG_SBW_PORT_OUT &= ~FPGA_BYPASS_RST_BIT; }
#define FPGA_BYPASS_TDI_HIGH		{ FPGA_BYPASS_JTAG_SBW_PORT_OUT |= FPGA_BYPASS_TDI_BIT; }
#define FPGA_BYPASS_TDI_LOW			{ FPGA_BYPASS_JTAG_SBW_PORT_OUT &= ~FPGA_BYPASS_TDI_BIT; }
#define FPGA_BYPASS_TMS_HIGH		{ FPGA_BYPASS_JTAG_SBW_PORT_OUT |= FPGA_BYPASS_TMS_BIT; }
#define FPGA_BYPASS_TMS_LOW			{ FPGA_BYPASS_JTAG_SBW_PORT_OUT &= ~FPGA_BYPASS_TMS_BIT; }
#define FPGA_BYPASS_TCK_HIGH		{ FPGA_BYPASS_JTAG_SBW_PORT_OUT |= FPGA_BYPASS_TCK_BIT; }
#define FPGA_BYPASS_TCK_LOW			{ FPGA_BYPASS_JTAG_SBW_PORT_OUT &= ~FPGA_BYPASS_TCK_BIT; }
#define FPGA_BYPASS_TDO_IS_ASSERTED	( FPGA_BYPASS_JTAG_SBW_PORT_IN & FPGA_BYPASS_TDO_BIT )

// UART direction signals in bypass mode
#define FPGA_BYPASS_DIR_CTRL_UART_RTR_BIT	( BIT7 )
#define FPGA_BYPASS_DIR_CTRL_UART_CTS_BIT	( BIT6 )
#define FPGA_BYPASS_DIR_CTRL_UART_RXD_BIT	( BIT5 )
#define FPGA_BYPASS_DIR_CTRL_UART_TXD_BIT	( BIT4 )
//#define FPGA_BYPASS_DIR_CTRL_PORT_OUT	      ( P1OUT )

// UART data signals in bypass mode
#define FPGA_BYPASS_UART_RTR_BIT		( BIT1 )
#define FPGA_BYPASS_UART_RTR_PORT_DIR	( P2DIR)
#define FPGA_BYPASS_UART_RTR_PORT_OUT	( P2OUT )
#define FPGA_BYPASS_UART_CTS_BIT		( BIT5 )
#define FPGA_BYPASS_UART_CTS_PORT_DIR	( P9DIR )
#define FPGA_BYPASS_UART_CTS_PORT_OUT	( P9OUT )
#define FPGA_BYPASS_UART_RXD_BIT		( BIT2 )
#define FPGA_BYPASS_UART_RXD_PORT_DIR	( P8DIR )
#define FPGA_BYPASS_UART_RXD_PORT_OUT	( P8OUT )
#define FPGA_BYPASS_UART_TXD_BIT		( BIT3 )
#define FPGA_BYPASS_UART_TXD_PORT_DIR	( P8DIR )
#define FPGA_BYPASS_UART_TXD_PORT_OUT	( P8OUT )

// FPGA commands
#define FPGA_CMD_CFG        0x00
#define FPGA_CMD_IR8_RD     0x01
#define FPGA_CMD_IR8        0x02
#define FPGA_CMD_IR4_RD     0x03
#define FPGA_CMD_IR4        0x04
#define FPGA_CMD_DR8_RD     0x05
#define FPGA_CMD_DR8        0x06
#define FPGA_CMD_DRX_RD     0x07
#define FPGA_CMD_DRX        0x08
#define FPGA_CMD_DRX0_RD    0x09
#define FPGA_CMD_BYPASS     0x0A
#define FPGA_CMD_RESET      0x0B
#define FPGA_CMD_ABORT      0x0C
#define FPGA_CMD_VERSION    0x0D
#define FPGA_CMD_CJTAG      0x0E

// FPGA cJTAG commands
#define FPGA_CJTAG_ZBS      0x00
#define FPGA_CJTAG_TCKIDLE  0x01

// Config Parameters
//FPGA registers
#define REG_RESPONSE_HANDSHAKE_OFF  0x00
#define REG_PROTOCOL		        0x01
#define REG_IRSCAN_PREAMBLE         0x02
#define REG_IRSCAN_POSTAMBLE	    0x03
#define REG_DRSCAN_PREAMBLE	        0x04
#define REG_DRSCAN_POSTAMBLE	    0x05
#define REG_TEST_CLK_FREQUENCY	    0x06
#define REG_TARGET_IO_CONFIGURATION 0x07
#define REG_TCLKset0		        0x08
#define REG_TCLKset1		        0x09
#define REG_TCLK_CLK_FREQUENCY	    0x0a
#define REG_START_FIFO		        0x0b
#define REG_STOP_FIFO		        0x0c
#define REG_JTAG_4_WIRE_FPGA_432    0x0D

#define JTAG_4_WIRE_FPGA            0
#define SBW_2_MSP_FET_FPGA          2
#define SBW_2_BACK_FPGA             1
#define TDI_TO_TDO_FPGA             3
#define TRI_STATE_FPGA_JTAG         4
#define TRI_STATE_FPGA_SBW          5


// PORTA bits
typedef union
{
  // Bit control
  struct
  {
    unsigned char DATA0		: 8;
    unsigned char UCA0TXD	: 1;
    unsigned char UCA0RXD	: 1;
    unsigned char CMD		: 4;
    unsigned char WR_TRIG	: 1;
    unsigned char RESERVED1	: 1;
  } bit;

  // Bypass control
  struct
  {
    unsigned char IO_GPIO0        : 1;
    unsigned char IO_GPIO1        : 1;
    unsigned char RESERVED0	      : 2;
    unsigned char DIR_CTRL_GPIO0  : 1;
    unsigned char DIR_CTRL_GPIO1  : 1;
    unsigned char DIR_CTRL_GPIO2  : 1;
    unsigned char DIR_CTRL_GPIO3  : 1;
    unsigned char IO_GPIO2        : 1;
    unsigned char IO_GPIO3        : 1;
    unsigned char RESERVED1	      : 6;
  } byp;

  // 16-bit access
  unsigned int all;

} s_FPGA_PA;

// PORTB bits
typedef union
{
   // Bypass control
  struct
  {
    unsigned char IO_TCK          : 1;
    unsigned char IO_TMS          : 1;
    unsigned char IO_TDI	      : 1;
    unsigned char IO_TDO          : 1;
    unsigned char IO_RST          : 1;
    unsigned char IO_TEST         : 1;
    unsigned char RESERVED0	      : 2;
    unsigned char DIR_CTRL_TCK    : 1;
    unsigned char DIR_CTRL_TMS    : 1;
    unsigned char DIR_CTRL_TDI    : 1;
    unsigned char DIR_CTRL_TDO    : 1;
    unsigned char DIR_CTRL_RST    : 1;
    unsigned char DIR_CTRL_TEST   : 1;
    unsigned char RESERVED1	      : 2;
  } byp;

  // 16-bit access
  unsigned int all;

} s_FPGA_PB;

#endif
