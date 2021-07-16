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

//******************************************************************************
// Filename:                       hilDacControl.c
//
// Description:
//
//******************************************************************************
#include "msp430.h"
#include "hilDacControl.h"
#include "arch.h"
//------------------------------------------------------------------------------
void hil_DacControlInitDAC0(void)
{
    // Set reference voltage to 2.5 V
    REFCTL0 |= REFMSTR;
    REFCTL0 |= REFVSEL1;
    REFCTL0 |= REFON;

    // Vref is used as reference, calibration on, DAC on
    DAC12_0CTL0 =  DAC12SREF_0 + DAC12AMP_5 + DAC12CALON;
    DAC12_0CTL0 |= DAC12IR;
    // set 12-bit resolution
    DAC12_0CTL0 |= DAC12OPS;          // DAC12 out on P7.6
    DAC12_0CTL0 |= DAC12ENC;          // Enable DAC12
}

void hil_DacControlSetDAC0(unsigned short dacVal)
{
     DAC12_0DAT = dacVal;
}

void hil_DacControlInitDAC1(void)
{
    // Set reference voltage to 2.5 V
    REFCTL0 |= REFMSTR;
    REFCTL0 |= REFVSEL1;
    REFCTL0 |= REFON;

    // Vref is used as reference, calibration on, DAC on
    DAC12_1CTL0 =  DAC12SREF_0 + DAC12AMP_5 + DAC12CALON;
    DAC12_1CTL0 |= DAC12IR;

    // set 12-bit resolution
    DAC12_1CTL0 |= DAC12OPS;          // DAC12 out on P7.7
    DAC12_1CTL0 |= DAC12ENC;          // Enable DAC12
    DAC12_1DAT = 0;                   // 0V
}

void hil_DacControlSetDAC1(unsigned short dacVal)
{
     DAC12_1DAT = dacVal;
}
