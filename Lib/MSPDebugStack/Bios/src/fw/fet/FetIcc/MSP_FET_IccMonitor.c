/*
 * MSP_IccMonitor.c
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

#include "Bios.h"
#include "Fet_IccMonitor.h"
#include "stream.h"

static unsigned short voltageSupervsion;
static edt_common_methods_t edt_Common_Methods;

void IccMonitor_setHilIterface(edt_common_methods_t *edt_c)
{
    edt_Common_Methods = *edt_c;
}
void IccMonitor_StartVoltageSupervision()
{
    voltageSupervsion = 1;
}
void IccMonitor_StopVoltageSupervision()
{
    voltageSupervsion = 0;
}
void IccMonitor_TriggerAdc()
{
    ADC12CTL0 |= ADC12SC;
}
void IccMonitor_StopDcdcOvercurrentDetection()
{
    P5DIR |= BIT7;
    P5OUT |= BIT7;
}
void IccMonitor_StartDcdcOvercurrentDetection()
{
    P5DIR |= BIT7;
    P5OUT &= ~BIT7;
}
void IccMonitor_SendOverCurrentEvent(unsigned short event)
{
    static unsigned long counter = 0;
    if(counter == 0)
    {
        if(STREAM_out_init(0, RESPTYP_STATUS) != EXCEPTION_TX_NO_BUFFER)
        {
            STREAM_put_word(event);
            STREAM_flush();
        }
    }
    if(counter++ > 100000)
    {
        counter = 0;
    }
}

short IccMonitor_Process(unsigned short flags)
{
    static short overcurrent = 0;
    static short VccDtSampling = 0;
    static short AdcTrigger = 1000;
    short error = 0;

    AdcTrigger--;
    if(AdcTrigger == 0)
    {
        IccMonitor_TriggerAdc();
        AdcTrigger = 1000;
    }

    if(!voltageSupervsion)
    {
        return 0;
    }
    if(VccDtSampling++ > 50 || overcurrent)
    {
        //call VCC regulation algorithm
        error = edt_Common_Methods.regulateVcc();
        IccMonitor_TriggerAdc();
        VccDtSampling = 0;
        if(error)
        {
            if(overcurrent == 0)
            {
                IccMonitor_SendOverCurrentEvent(FET_OVERCURRENT);
            }
            overcurrent = 1;
            BIOS_LedOff(BIOS_LED_POWER);
            BIOS_LedOn(BIOS_LED_MODE);
            return 0;
        }
    }
    if((P8IN & BIT4) == BIT4)
    {
       //overcurrent detected -> Sub MCU swith off VCC -> Biso switch on READ LED -> switch of GREEN LED
        BIOS_LedOff(BIOS_LED_POWER);
        BIOS_LedOn(BIOS_LED_MODE);
        if(overcurrent == 0)
        {
            IccMonitor_SendOverCurrentEvent(FET_OVERCURRENT); // callback
        }
        overcurrent = 1;
        return 0;
    }
    else
    {
        // switch on LEDs.
        BIOS_LedOn(BIOS_LED_POWER);
        if(overcurrent)
        {
            overcurrent = 0;
            BIOS_LedOff(BIOS_LED_MODE);
        }
    }
    return 0;
}

#pragma vector=ADC12_VECTOR
__interrupt void ADC12ISR (void)
{
  switch(__even_in_range(ADC12IV,34))
  {
      case  0: break;                           // Vector  0:  No interrupt
      case  2: break;                           // Vector  2:  ADC overflow
      case  4: break;                           // Vector  4:  ADC timing overflow
      case  6: break;                           // Vector  6:  ADC12IFG0
      case  8: break;                           // Vector  8:  ADC12IFG1
      case 10: break;                           // Vector 10:  ADC12IFG2
      case 12:                                  // Vector 14:  ADC12IFG3
        edt_Common_Methods.ReadADC12();
        ADC12CTL0 &= ~ADC12SC;
        break;
      case 14: break;                           // Vector 14:  ADC12IFG4
      case 16: break;                           // Vector 16:  ADC12IFG5
      case 18: break;                           // Vector 18:  ADC12IFG6
      case 20: break;                           // Vector 20:  ADC12IFG7
      case 22: break;                           // Vector 22:  ADC12IFG8
      case 24: break;                           // Vector 24:  ADC12IFG9
      case 26: break;                           // Vector 26:  ADC12IFG10
      case 28: break;                           // Vector 28:  ADC12IFG11
      case 30: break;                           // Vector 30:  ADC12IFG12
      case 32: break;                           // Vector 32:  ADC12IFG13
      case 34: break;                           // Vector 34:  ADC12IFG14
      default: break;
  }
}
