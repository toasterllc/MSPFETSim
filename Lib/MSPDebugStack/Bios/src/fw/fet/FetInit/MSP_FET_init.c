/*
 * MSP_FET_init.c
 *
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
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


#include "init.h"
#include "hw_compiler_specific.h"

void init_BiosPorts(void)
{
	// Ports all back to reset state
    P1DIR = 0;  P2DIR = 0;  P3DIR = 0;    // Reset all ports direction to be inputs
    P4DIR = 0;  P5DIR = 0;  P6DIR = 0;
    P7DIR = 0;  P8DIR = 0;  P9DIR = 0;
    P1SEL = 0;  P2SEL = 0;  P3SEL = 0;    // Reset all ports alternate function
    P4SEL = 0;  P5SEL = 0;  P6SEL = 0;
    P7SEL = 0;  P8SEL = 0;  P9SEL = 0;
    P1OUT = 0;  P2OUT = 0;  P3OUT = 0;    // Reset all port output registers
    P4OUT = 0;  P5OUT = 0;  P6OUT = 0;
    P7OUT = 0;  P8OUT = 0;  P9OUT = 0;

    // Port2
    //  P2.0 <- DCDC_PULSE
    //  P2.1 <- UART_RTS
    //  P2.2 -> CMD[0]
    //  P2.3 -> CMD[1]
    //  P2.4 -> CMD[2]
    //  P2.5 -> CMD[3]
    //  P2.6 -> WR_TRIG
    //  P2.7 -> SYS_CLK
    //P2SEL = BIT0;

    // Port5
    //  P5.0 -> VREF+ (output of reference voltage to ADC)
    //  P5.1 -> FPGA_RESET
    //  P5.2 -> VF2TEST_CTRL
    //  P5.3 -> LED1
    //  P5.4 -> VF2TDI_CTRL
    //  P5.5 -> TDIOFF_CTRL (0 = turns off TDI, after that ok to select VF for fuse blowing)
    //  P5.6 <- MCU_DMAE0 / RD_TRIG (DMA trigger input)
    //  P5.7 -> DCDC_IO1
    P5DIR |= (BIT3 | BIT4 | BIT5 | BIT7);	// set pins initially to output direction
    //P5SEL = (BIT0);
    P5OUT = (BIT5);
    P5OUT &= ~BIT7;


      // Port6
      //  P6.0 <- A_VBUS5 (input to ADC channel A0)
      //  P6.1 <- A_VCC_SUPPLY
      //  P6.2 <- A_VF
      //  P6.3 <- A_VCC_SENSE0_TRGT
      //  P6.4 <- A_VCC_DT
      //  P6.5 <- A_VCC_DT_BSR
      //  P6.6 -> VCC_JTAGLDO2TRGT_CTRL
      //  P6.7 -> LED0
      P6DIR |= (BIT7);	// set pins initially to output direction
      //P6SEL = (BIT0+BIT1+BIT2+BIT3+BIT4+BIT5);

      // Port7
      //  P7.0 -> n/c
      //  P7.1 -> n/c
      //  P7.2 <- XT2IN
      //  P7.3 -> XT2OUT
      //  P7.4 -> DCDC_TEST
      //  P7.5 -> DCDC_RST (0 = reset)
      //  P7.6 -> VCC_DT_REF
      //  P7.7 -> VCC_DCDC_REF
      P7DIR |= (BIT0+BIT1);	        // set pins initially to output direction
      P7SEL |= (BIT6+BIT7);	        // set pins to alternate port function BIT2+BIT3

      // Port8
      //  P8.0 -> VCC_DT2TRGT_CTRL (control signal to switches to provide debug signals to target via JTAG.x)
      //  P8.1 -> IO_CTRL
      //  P8.2 -> UART_TXD
      P8DIR |= BIT3; // set UART_TXD to output direction\r
      //  P8.3 <- UART_RXD
    //  P8.4 <- DCDC_IO0
      //  P8.5 -> HOST_SDA
      //  P8.6 -> HOST_SCL
      //  P8.7 -> VCC_SUPPLY2TRGT_CTRL (DCDC VCC to target VCC)
      P8DIR |= (BIT0+BIT7);	        // set pins initially to output direction
}
