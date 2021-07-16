/*
 * eZ_Fet_IccMonitor.c
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

void IccMonitor_setHilIterface(edt_common_methods_t *edt_c)
{
    return;
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

void IccMonitor_StopDcdcOvercurrentDetection()
{
    // Switch off over-current detection
    P6DIR |= BIT5;
    P6OUT |= BIT5;
}
void IccMonitor_StartDcdcOvercurrentDetection()
{
    P6DIR |= BIT5;
    P6OUT &= ~BIT5;
}

short IccMonitor_Process(unsigned short flags)
{
    static short overcurrent = 0;

    if(!voltageSupervsion)
    {
        return 0;
    }
    if(((P6IN & BIT4) == BIT4) && Bios_getTool_id() == eZ_FET_WITH_DCDC
       || ((P6IN & BIT4) == BIT4) && Bios_getTool_id() == eZ_FET_WITH_DCDC_NO_FLOWCTL
         || ((P6IN & BIT4) == BIT4) && Bios_getTool_id() == eZ_FET_WITH_DCDC_V2x)//eZ-FET with DCDC
    {
       //overcurrent detected -> Sub MCU swith off VCC -> Biso switch on READ LED -> switch of GREEN LED
        BIOS_LedOff(BIOS_LED_POWER);
        BIOS_LedOn(BIOS_LED_MODE);
        if(overcurrent == 0)
        {
            IccMonitor_SendOverCurrentEvent(FET_OVERCURRENT);
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
