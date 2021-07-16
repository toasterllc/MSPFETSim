/*
* FetUart.c
*
* Base class for all memory classes.
*
* Copyright (C) 2007 - 2011 Texas Instruments Incorporated - http://www.ti.com/
*
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
*
* Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the
* distribution.
*
* Neither the name of Texas Instruments Incorporated nor the names of
* its contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
* A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
* OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*----- FetBSL.c -----*/
#include "hw_compiler_specific.h"

#include "msp430.h"
#include "..\FetCom.h"
#include "FetBsl.h"
#include "USB_API/USB_Common/types.h"          // Basic Type declarations

static FET_USB_INFOS_t UsbPointers_;

static unsigned long UartPreviousBaudrate = 0;

// init function for UART port pins
// reset all portpins
void FetBsl_PortsHds()         // with handshake
{
    uart_CtsOut();          // CTS output
    uart_ClearCtsBit();     // clear CTS
    uart_RtsIn();           // RTS input
    uart_TxdSel();
    uart_RxdSel();
}

// initilaize FetBSL_functions
void FetBsl_Init()
{
    // Ports configuration
    FetBsl_PortsHds();
    UCA_IE_REGISTER |= UCRXIE;        // enable USCIAx interrupt
}

void FetBsl_SetUsb(FET_USB_INFOS_t* usb_Pointers)
{
    UsbPointers_ = *usb_Pointers;
}

void FetBsl_Close()
{
  TX_RX_PORT_SEL &= ~( TX_PORT_BIT | RX_PORT_BIT ); // deassign IO-Ports from UCAxTXD, UCAxRxD
  TX_RX_PORT_DIR &= ~( TX_PORT_BIT | RX_PORT_BIT );
  // set RTS as input
  RTS_PORT_OUT &= ~RTS_PORT_BIT;
  RTS_PORT_DIR &= ~RTS_PORT_BIT;
  // set CTS as input
  CTS_PORT_OUT &= ~CTS_PORT_BIT;
  CTS_PORT_DIR &= ~CTS_PORT_BIT;
  CTS_PORT_REN &= ~CTS_PORT_BIT;
  com_RtsIfgClear();
}

// configuration function for UART settings
// UART will be configure automatically about COM channel configuration
short FetBsl_Config(unsigned long UartBaudrate)
{
    uart_ClearCtsBit();
    if(UartBaudrate != UartPreviousBaudrate)    // baudrate is changed ?
    {
        UCA_CTRL_REGISTER |= UCSWRST;         // Put state maschine reset
        UCA_CTRL_REGISTER |= UCSSEL__SMCLK;   // clock source: SMCLK

        if(UartBaudrate == 9600)
        {
            UCA1BR0 = 0x8F;
            UCA1BR1 = 0x0;
            UCA1MCTL = 0xE0 + UCOS16;
        }
        else if(UartBaudrate == 19200)
        {
            UCA1BR0 = 0x47;
            UCA1BR1 = 0x0;
            UCA1MCTL = 0xF0 + UCOS16;
        }
        else if(UartBaudrate == 28800)
        {
            UCA1BR0 = 0x2B;
            UCA1BR1 = 0x0;
            UCA1MCTL = 0x50 + UCOS16;
        }
        else if(UartBaudrate == 38400)
        {
            UCA1BR0 = 0x23;
            UCA1BR1 = 0x0;
            UCA1MCTL = 0x80 + UCOS16;
        }
        else if(UartBaudrate == 56000)
        {
            UCA1BR0 = 0x18;
            UCA1BR1 = 0x0;
            UCA1MCTL = 0x40 + UCOS16;
        }
        else if(UartBaudrate == 57600)
        {
            UCA1BR0 = 0x17;
            UCA1BR1 = 0x0;
            UCA1MCTL = 0xA0 + UCOS16;
        }
        else if(UartBaudrate == 115200)
        {
            UCA1BR0 = 0xB;
            UCA1BR1 = 0x0;
            UCA1MCTL = 0xD0 + UCOS16;
        }
        else if(UartBaudrate == 256000)
        {
            UCA1BR0 = 0x5;
            UCA1BR1 = 0x0;
            UCA1MCTL = 0xE0 + UCOS16;
        }

        UCA1CTL0 =  UCPEN +  UCPAR;       //  parety even
        UCA_CTRL_REGISTER &=~ UCSWRST;    // Initialize USCI state machine
        UCA_IE_REGISTER |= UCRXIE;        // enable USCIAx interrupt

        UartPreviousBaudrate = UartBaudrate;

    }
    return (0);
}

short FetBsl_NotBusy()
{
    if((UCA_STATUS_REGISTER & UCBUSY) == UCBUSY)   // when busy:
    {
        return (0);                     // return 0
    }
    return (1);                         // else return 1
}

short FetBsl_TxEmpty()
{
    // UCTXIFG is set when UCAxTXBUF empty
    if(UCTXIFG == (UCTXIFG & UCA_INTERRUPT_FLAG_REGISTER))
    {
        return (1);     // empty
    }
    return (0);         // not empty
}

// functions copies byte by byte from USB buffer to UART buffer
short FetBsl_Transmit(unsigned char uartDataToTxBuffer)
{
    UCA_TX_BUF = uartDataToTxBuffer;
    return (0);
}

short FetBsl_Send()
{
      // Start USB to UART bridge communication ----------------------------------
    BYTE BytesInUsbBuffer = UsbPointers_.FetUSB_bytesInUSBBuffer(CDC0_DATA_INTERFACE);
    if(BytesInUsbBuffer)
    {
        unsigned char i = 0;
        BYTE uartDataToSend[260] = {0};
        UsbPointers_.FetUSB_receiveData(uartDataToSend, BytesInUsbBuffer , CDC0_DATA_INTERFACE);
        while(BytesInUsbBuffer)
        {
            if(FetBsl_NotBusy() && FetBsl_TxEmpty())
            {
                FetBsl_Transmit(uartDataToSend[i++]);
                BytesInUsbBuffer--;
            }
        }
    }
    return 0;
}
