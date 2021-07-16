/*
 * hilDelays.c
 *
 * <FILE_BRIEF>
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

#include "MSP430F1612.h"
#include "hilDelays.h"

static void _hil_DelayStartTimer()
{
    TACTL =  0;                                          // STOP Timer
    TACTL &= ~TAIFG;                                     // Clear the interrupt flag
    TACTL =  TASSEL_2;                                   // Timer is runnig at 1 Mhz
    TACCR0 = 0x2E9;                                      // Load CCR0 with delay... (1ms delay)
    TAR = 0;
    TACTL |= TACLR + MC_1;                               // Start Timer
}

static void _hil_DelayStopTimer()
{
    TACTL =  0;                                          // STOP Timer
    TACTL &= ~CCIFG;                                     // Clear the interrupt flag
}

void _hil_Delay_1ms(unsigned short millisec)			// inner loop generates 1 ms
{
    unsigned short i;
    for(i = millisec; i > 0; i--)
    {
        _hil_DelayStartTimer();
        while ((TACTL & TAIFG) == 0);	                    // Wait until the Timer elapses
        _hil_DelayStopTimer();
    }
}

void _hil_Delay_1us(unsigned short  microsec_1)		    // inner loop generates 10 us
{
    while((--microsec_1) > 0)
    {
        __delay_cycles(4);
    }
}
