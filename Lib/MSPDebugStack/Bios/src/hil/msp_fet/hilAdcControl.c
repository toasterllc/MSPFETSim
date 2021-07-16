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
// Filename:                       hilAdcControl.c
//
// Description:
//
//******************************************************************************
#include "msp430.h"
#include "hilAdcControl.h"
#include "arch.h"

static unsigned short runningAverageBufferSpVcc[8] = {0,0,0,0,0,0,0,0};
static unsigned short runningBufferVCCDtSense = 0;
static unsigned short runningAverageBufferDtVcc[8] = {0,0,0,0,0,0,0,0};
static unsigned short runningAverageBufferExtVcc[8] = {0,0,0,0,0,0,0,0};
static unsigned short isVCC = 0, dtVCC = 0, iexVCC = 0;

/*  In normal operation mode, the  read function is triggered by the ADC12
    interrupt when a sample sequence has been completed.
    During Hil module init (FET startup)  and Set VCC the ADC12 interrupts are
    disabled and the readout is triggered manually by the calling function
*/

#define CAL_ADC_GAIN_FACTOR  *((unsigned short *)0x1A16)
#define CAL_ADC_OFFSET  *((short *)0x1A18)

void hil_AdcControlRead()
{
    volatile long notCalibratedAdc = 0;
    while ((ADC12IFG & BIT3) == 0);

    notCalibratedAdc = ADC12MEM0;
    notCalibratedAdc = ((notCalibratedAdc * CAL_ADC_GAIN_FACTOR) / 32768) + (long)CAL_ADC_OFFSET;
    runningAverageBufferSpVcc[isVCC] = (unsigned short)(((((notCalibratedAdc * 2500) / 4096) * (220 + 220)) / 220));
    isVCC++;
    if(isVCC == 8) isVCC = 0;

    notCalibratedAdc = ADC12MEM1;
    notCalibratedAdc = ((notCalibratedAdc * CAL_ADC_GAIN_FACTOR) / 32768) + (long)CAL_ADC_OFFSET;
    if(notCalibratedAdc > 0)
    {
        runningAverageBufferExtVcc[iexVCC] = (unsigned short)(((((notCalibratedAdc * 2500) / 4096) * (150 + 150)) / 150));
    }
    else
    {
        runningAverageBufferExtVcc[iexVCC] = 0;
    }
    iexVCC++;
    if(iexVCC == 8) iexVCC = 0;

    notCalibratedAdc = ADC12MEM2;
    notCalibratedAdc = ((notCalibratedAdc * CAL_ADC_GAIN_FACTOR) / 32768) + (long)CAL_ADC_OFFSET;
    runningAverageBufferDtVcc[dtVCC] = (unsigned short)(((((notCalibratedAdc * 2500) / 4096) * (150 + 150)) / 150));
    dtVCC++;
    if(dtVCC == 8) dtVCC = 0;

    runningBufferVCCDtSense = (unsigned short)(((float)ADC12MEM3 * 1000 * A_VREFPLUS) / ADC_CONV_RANGE);
}

/* Init ADC, enable sequence conversion on A1, A3, A4 and A6 – Sequence is
    triggered once during main loop execution, only when HIL module is loaded and
    valid
*/
void hil_AdcControlInitADC()
{
    isVCC = 0;
    dtVCC = 0;
    iexVCC = 0;
    runningBufferVCCDtSense = 0;

    // Enable Vref=2.5V for ADC --> Already done in Dac init

    // A0=P6.0, A1=P6.1
    P6OUT &= ~(BIT1+BIT3+BIT4+BIT6);
    P6SEL |= (BIT1+BIT3+BIT4+BIT6);

    // Configure ADC12
    // select channel and do conversion
    ADC12CTL0 &= ~ADC12ENC;                      // Disable conversion
    ADC12CTL0 = ADC12ON + ADC12SHT0_9 + ADC12REF2_5V + ADC12REFON + ADC12MSC;         // Turn on ADC12, set sampling time
    ADC12CTL1 = ADC12SHP + ADC12CONSEQ_1 + ADC12SSEL_3 + ADC12DIV_7;                   // Use sampling timer

    ADC12MCTL0 = ADC12SREF_1 + ADC12INCH_1;             //A_VCCSUPPLY;
    ADC12MCTL1 = ADC12SREF_1 + ADC12INCH_3;             //A_VCCSUPPLY_SENSE;
    ADC12MCTL2 = ADC12SREF_1 + ADC12INCH_4;             //A_VCCDT;
    ADC12MCTL3 = ADC12SREF_1 + ADC12INCH_6 + ADC12EOS;; //A_VCCDT_SENSE;

    __delay_cycles((15*90));
    ADC12CTL0 |= ADC12ENC;                       // Enable conversion
}

/*----------------------------------------------------------------------------
   This function performs a single AD conversion on the selected pin.
   Uses internal reference 2500mV (VR+ = VREF+).
    Only used during Fuse Blow to measure VCC Fuse
   Arguments: word pinmask (bit position of selected analog input Ax)
   Result:    word (12 bit ADC conversion result)
*/
float ConvertAD(short channel)
{
    ADC12CTL0  &= ~ADC12ENC;                                        // Disable conversion, write controls
    ADC12MCTL0  = ADC12SREF_1 + channel;                               // select Vref and analog channel Ax
    ADC12CTL0  |= ADC12ENC;                                         // Enable conversions
    ADC12CTL0  |= ADC12SC;                                          // Start conversions
    while ((ADC12IFG & BIT0) == 0);                                 // wait until conversion is done
    return(((float)ADC12MEM0* A_VREFPLUS) / ADC_CONV_RANGE);        // return conversion result from MEM0 value in mv
}

/* Only used during Fuse Blow to measure VCC Fuse  */
void hil_AdcControlInitADCvFuse()
{
    ADC12IE = 0;
    ADC12MCTL0 = 0;
    ADC12MCTL1 = 0;
    ADC12MCTL2 = 0;
    ADC12MCTL3 = 0;

    // A0=P6.0, A1=P6.1
    P6OUT &= ~(BIT0+BIT1+BIT4+BIT6);
    P6SEL |= BIT0+BIT1+BIT4+BIT6;

    // Configure ADC12
    // select channel and do conversion
    ADC12CTL0 &= ~ADC12ENC;                      // Disable conversion
    ADC12CTL0 = ADC12ON + ADC12SHT0_2 + ADC12REF2_5V + ADC12REFON;                  // Turn on ADC12, set sampling time
    ADC12CTL1 = ADC12SHP + ADC12SSEL_1;                                             // Use sampling timer
    __delay_cycles((15*90));
    ADC12CTL0 |= ADC12ENC;                       // Enable conversion
}

// Get 1 sample from A6
unsigned short hil_AdcControlGetDtVccSense(void)
{
    return runningBufferVCCDtSense;
}

// Get 1 sample from A3
unsigned short hil_AdcControlGetExternalVcc(void)
{
    return ((runningAverageBufferExtVcc[0] + runningAverageBufferExtVcc [1] +
        runningAverageBufferExtVcc[2] + runningAverageBufferExtVcc[3] +
        runningAverageBufferExtVcc[4] + runningAverageBufferExtVcc[5] +
        runningAverageBufferExtVcc[6] + runningAverageBufferExtVcc[7]) >> 3);
}
// Get 1 sample from A1
unsigned short hil_AdcControlGetSupplyVcc(void)
{
    return (((runningAverageBufferSpVcc[0] + runningAverageBufferSpVcc [1] +
        runningAverageBufferSpVcc[2] + runningAverageBufferSpVcc[3] +
        runningAverageBufferSpVcc[4] + runningAverageBufferSpVcc[5] +
        runningAverageBufferSpVcc[6] + runningAverageBufferSpVcc[7]  ) >> 3) - 20);
}
// Get 1 sample from A2
unsigned short hil_AdcControlGetVFuse(void)
{
    float VFuseTmp = (((ConvertAD(A_VF) * (270 + 150)) / 150) * 1000);
        return (unsigned short)VFuseTmp;
}
// Get 1 sample from A4
unsigned short hil_AdcControlGetVccDt(void)
{
    return (((runningAverageBufferDtVcc[0] + runningAverageBufferDtVcc [1] +
        runningAverageBufferDtVcc[2] + runningAverageBufferDtVcc[3] +
        runningAverageBufferDtVcc[4] + runningAverageBufferDtVcc[5] +
        runningAverageBufferDtVcc[6] + runningAverageBufferDtVcc[7]) >> 3)-20);
}
