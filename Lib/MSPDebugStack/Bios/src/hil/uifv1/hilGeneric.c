/*
 * \ingroup MODULHIL
 *
 * \file hilGeneric.c
 *
 * \brief Hardware Independent Layer Interface
 *
 *
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
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

#include "arch.h"
#include "hw_compiler_specific.h"
#include "edt.h"
#include "hilDelays.h"
#include "HalGlobalVars.h"
#include "stream.h"

volatile unsigned char VFSettleFlag = 0;
unsigned short bVccOn;
extern unsigned char *tstctrl_port_;

const unsigned short ADC_MV[4] = { 5905, 5905, 6934, 5905 };  // input voltages included the scale by resitors
const unsigned short ADC_CONV_RANGE = 4096;                   // 1 Bit error, because div is with 4096 faster
const unsigned short ADC_AVERAGES = 4;
const unsigned short DAC_CONV_RANGE = 4095;

unsigned short _hilGeneric_ConvertAD(unsigned short channel)
{
    unsigned long tmp;
    unsigned short ret_mv = 0;

    if(channel < 4 )
    {
        tmp = *((unsigned short*)&ADC12MEM0+channel) +
        *((unsigned short*)&ADC12MEM4+channel) +
        *((unsigned short*)&ADC12MEM8+channel) +
        *((unsigned short*)&ADC12MEM12+channel);
        tmp *= ADC_MV[channel];
        tmp /= ADC_CONV_RANGE * ADC_AVERAGES;
        ret_mv = tmp;
    }
    return(ret_mv);
}

/*----------------------------------------------------------------------------
   This function performs a single AD conversion on the selected pin.
   Uses internal reference 2.5V (VR+ = VREF+, VR- = AVSS).
   Only used for SetTargetVcc() and Selftest().
   Arguments: word pinmask (bit position of selected analog input Ax)
   Result:    word (12 bit ADC conversion result)
*/
short  _hilGeneric_ConvertAD_(short channel)
{
// select channel and do conversion
  ADC12CTL0  &= ~ENC;                   // Disable conversion, write controls
  ADC12MCTL0  = 0x10 + channel;         // select Vref and analog channel Ax
  ADC12CTL0  |= ENC;                    // Enable conversions
  ADC12CTL0  |= ADC12SC;                // Start conversions
  while ((ADC12IFG & BIT0) == 0);       // wait until conversion is done
  return(ADC12MEM0);                    // return conversion result from MEM0
}

unsigned short last_vcc;

#pragma optimize = none
void  _hilGeneric_SetDac(unsigned short vcc)
{
    signed short nomDac;
    signed short corr;

    if(last_vcc != vcc)
    {
        nomDac = (unsigned short)(ConvRange - (vcc - minVCCT));      // calculate approximated value
        nomDac += (85 - (vcc/20));                 // error reduction of approximated value
        if(nomDac > ConvRange)
        {
            nomDac = ConvRange;
        }
        if(nomDac < 0)
        {
            nomDac = 0;
        }
        DAC12_0DAT = nomDac;                  // set voltage
        DAC12_0CTL |= DAC12ENC;               // enable DAC12
        last_vcc = vcc;
    }
    else
    {
        corr =  _hilGeneric_ConvertAD(VCCTCHN);
        if(corr > (vcc+10))
        {
            if(DAC12_0DAT < 4095)
            {
                DAC12_0DAT++;
            }
        }
        else if(corr < (vcc-10))
        {
            if(DAC12_0DAT)
            {
                DAC12_0DAT--;
            }
        }
    }
}

// -----------------------------------------------------------------------------
void _hilGeneric_Init( void )
{
    // Setup ADC12
       // Setup ADC12
    ADC12CTL0  &= ~ENC;					  // Disable conversions, write controls
    ADC12CTL0  = ADC12ON | REFON | REF2_5V | MSC | SHT0_2 | SHT1_2;  // Turn on ADC12, VREF = 2.5v, set samp. time
    ADC12CTL1  = SHP | CONSEQ_3;					  // Use sampling timer, CstartAddr = MEM0
    // four time averaging on each channel
    ADC12MCTL0  = 0x10 | 0;           // select Vref and cannel
    ADC12MCTL1  = 0x10 | 1;           // select Vref and cannel
    ADC12MCTL2  = 0x10 | 2;           // select Vref and cannel
    ADC12MCTL3  = 0x10 | 3;           // select Vref and cannel
    ADC12MCTL4  = 0x10 | 0;           // select Vref and cannel
    ADC12MCTL5  = 0x10 | 1;           // select Vref and cannel
    ADC12MCTL6  = 0x10 | 2;           // select Vref and cannel
    ADC12MCTL7  = 0x10 | 3;           // select Vref and cannel
    ADC12MCTL8  = 0x10 | 0;           // select Vref and cannel
    ADC12MCTL9  = 0x10 | 1;           // select Vref and cannel
    ADC12MCTL10 = 0x10 | 2;           // select Vref and cannel
    ADC12MCTL11 = 0x10 | 3;           // select Vref and cannel
    ADC12MCTL12 = 0x10 | 0;           // select Vref and cannel
    ADC12MCTL13 = 0x10 | 1;           // select Vref and cannel
    ADC12MCTL14 = 0x10 | 2;           // select Vref and cannel
    ADC12MCTL15 = 0x10 | 3;           // select Vref and cannel

    ADC12CTL0  |= ENC;                    // Enable conversions
    // ADCs run free
    ADC12CTL0  |= ADC12SC;                // Start conversions

    DAC12_0DAT = VCCTmin;                           // set voltage to MIN ~0
    DAC12_0CTL = DAC12CALON + DAC12IR + DAC12AMP_5; // Internal ref gain 1

    // Init SPI for JTAG
    UCTL1 |= SWRST;
    UCTL1 = CHAR | SYNC | MM | SWRST;
    UTCTL1 = CKPL | SSEL0 | STC | TXEPT;
    UBR01  = 2;                          // Load SPI frequency divider 2
    UBR11  = 0x00;
    UMCTL1 = 0x00;						// Clear Modulation Register
    UCTL1 &= ~SWRST;
    ME2   |= USPIE1;					// Enable SPI module
}

void _hilGeneric_SetJtagSpeed(unsigned short speed, unsigned short sbwSpeed)
{
    if(speed)
    {
        UBR01  = 2;				        // Load SPI frequency divider
    }
}

// -----------------------------------------------------------------------------
short _hilGeneric_GetVcc(double* Vcc, double* ExtVcc)
{
    if(P4OUT & BIT3)
    {
        *Vcc = _hilGeneric_ConvertAD(VCCTCHN);
    }
    else
    {
        *Vcc = 0;
    }
    *ExtVcc = _hilGeneric_ConvertAD(VCCICHN);
    return 0;
}

#ifdef ARCH_MSP432
void VCCTon()
{
    P4OUT |= BIT3;
}
void VCCToff()
{
    P4OUT &= ~BIT3;
}
#endif

// -----------------------------------------------------------------------------
#pragma optimize = medium
short _hilGeneric_SetVcc(unsigned short Vcc)
{
    static unsigned char bPowerUp = 1;

    if(Vcc)
    {
        //This workaround is necessary to set TCK to a defined low level before
        //applying Vcc to the target device
        //otherwise the target will not power up correctly and the UIF can't
        //establish a SBW communication
        if(bPowerUp)
        {
            P5OUT &= ~BIT3;
            P2OUT &= ~BIT5; //(*_Jtag.RST_PORT) &= ~(_Jtag._SELT);
            bVccOn = Vcc;
            _hilGeneric_SetDac(Vcc+50);
            VCCTon();
            _hil_Delay_1ms(1);
            bPowerUp = 0;
        }
        else
        {
            bVccOn = Vcc;
            _hilGeneric_SetDac(Vcc+50);
            VCCTon();
        }
    }
    else
    {
        VCCToff();
        IHIL_Close();
        bVccOn = 0;
        bPowerUp = 1;
    }
    return 0;
}


// *****************************************************************************
// Icc Monitor
char bIccMonitorOn;                 // Initialization in HalGlobalVars.c
char bHighCurrent;                  // Initialization in HalGlobalVars.c
signed short last_ext_vcc;          // Initialization in HalGlobalVars.c
unsigned short over_current_count;   // Initialization in HalGlobalVars.c

short IccMonitor_Process(unsigned short flags)
{
    signed short ext_vcc;
    signed short int_vcc;
    signed short rint_vcc;

    ext_vcc = _hilGeneric_ConvertAD(VCCICHN);          // convert channel
    int_vcc = _hilGeneric_ConvertAD(VCCTCHN);          // convert channel
    rint_vcc = _hilGeneric_ConvertAD(VCCRCHN);

    if(ext_vcc >= ExtLimit) // external voltage in the range 1.7V >=
    {
        if((ext_vcc < (last_ext_vcc-10)) || ext_vcc > (last_ext_vcc+10))
        {
            last_ext_vcc = ext_vcc;
        }
        _hilGeneric_SetDac(last_ext_vcc);               // let's hace a look at the external supply pin
    }
    else
    {
        if(bVccOn != 0)
        {
          _hilGeneric_SetVcc(bVccOn);
        }
    }

    if(bVccOn && bIccMonitorOn)
    {
        if((rint_vcc - int_vcc) > 150) // eqivalent to > 100mA
        {
            over_current_count++;
            if(over_current_count > 400)
            {
                bHighCurrent = 1;
                IHIL_Close();
                P4OUT &= ~BIT3;                             // turn off target VCC
                bVccOn = 0;
                STREAM_biosLedOff(1);
            }
        }
        else
        {
            over_current_count = 0;
        }
    }
    else
    {
        over_current_count = 0;
    }
    return 0;
}

// -----------------------------------------------------------------------------
#pragma vector=TIMERA0_VECTOR
__interrupt void TA_CCR0_ISR (void)
{
    static unsigned char cnt  = 0;

    short x;
    cnt++;
    if(cnt == 10)
    {
        cnt   = 0;     // reset pulse counter
        TACTL = 0;     // stop Timer A
        x = _hilGeneric_ConvertAD_(VFCHN);
        if(x < 0xFFF)   // 12-bit A/D thus value must exceed maximum measureable value (+-6.6V)
        {
            TACTL = (TASSEL_1 + TACLR + MC_1);      // start Timer A again and produce pulses
        }
        else
        {
            VFSettleFlag = 0;        // send message to main program
        }
    }
}

