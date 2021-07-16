/*
 * usbMain.c
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
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
#include "USB_API/USB_Common/device.h"
#include "USB_API/USB_Common/types.h"          // Basic Type declarations
#include "USB_config/descriptors.h"
#include "USB_API/USB_Common/usb.h"         // USB-specific functions
#include "USB_API/USB_CDC_API/UsbCdc.h"
#include "usbMain.h"
#include "usbConstructs.h"

COM_INFOS_t com_ins;

void UsbMain_setComIterface(COM_INFOS_t *ui_c)
{
    com_ins = *ui_c;
}
void UsbMain_ComConfig(unsigned long Baudrate, unsigned long MCLK_Frequency, unsigned short toolId)
{
    com_ins.comConfig(Baudrate,MCLK_Frequency, toolId);
}

void USB_MainLoop(void)
{
    com_ins.comLoop();
}

#ifdef eZ_FET
// Interrupt service routines for UART handling
#pragma vector=USCI_A0_VECTOR
__interrupt void UART_RxIsr(void)
{
    com_ins.comReceive(UCA0RXBUF);
}

#pragma vector=PORT2_VECTOR
__interrupt void UART_RtsIsr(void)
{
    if((P2IFG & BIT6) == BIT6)
    {
        com_RtsIfgClear();
        com_ins.comSetRts();
    }
}
#endif

#ifdef MSP_FET
    #pragma vector=USCI_A1_VECTOR
    __interrupt void UART_RxIsr(void)
    {
        com_ins.comReceive(UCA1RXBUF);
    }

    #pragma vector=PORT2_VECTOR
    __interrupt void UART_RtsIsr(void)
    {
        if((P2IFG & BIT1) == BIT1)
        {
            com_RtsIfgClear();
            com_ins.comSetRts();
        }
    }
#endif
