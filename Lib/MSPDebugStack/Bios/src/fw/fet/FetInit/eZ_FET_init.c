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
	//P7DIR = 0; 	P8DIR = 0;
	P1SEL = 0;  P2SEL = 0;  P3SEL = 0;    // Reset all ports alternate function
	P4SEL = 0;  P5SEL = 0;  P6SEL = 0;
	//P7SEL = 0; 	P8SEL = 0;
	P1OUT = 0;  P2OUT = 0;  P3OUT = 0;    // Reset all port output registers
	P4OUT = 0;  P5OUT = 0;  P6OUT = 0;
	//P7OUT = 0;	P8OUT = 0;

    //Port1
    //LED's
    P1DIR = (BIT2+BIT3+BIT4);

	// Port5
	P5DIR = (BIT0);	// set pins initially to output direction
    P5SEL = (BIT0+BIT6);

	// Port6
	P6DIR = (BIT5);	// set pins initially to output direction
	P6SEL = (BIT0+BIT1+BIT2);

    // set PORT6.4 to input for SUB mcu current reculation
    P6DIR &= ~BIT4;

    P6OUT &= ~(BIT6); // set Reset to 1 -> drive device reset
    __delay_cycles(20000);
    P6OUT = (BIT6);  // release Reset start Sub MCU
    P2REN |= BIT6;   // enable pull down for RTS uart line
}
