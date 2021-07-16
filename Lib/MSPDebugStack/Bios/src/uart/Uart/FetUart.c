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

/*----- FetUart.c -----*/
#include "hw_compiler_specific.h"

#include "msp430.h"
#include "../FetCom.h"
#include "USB_API/USB_Common/types.h"          // Basic Type declarations

static unsigned char RtsReady = 0;
static unsigned short HandshakeOn = 0;

static FET_USB_INFOS_t UsbPointers_;
static unsigned long UartPreviousBaudrate = 0;

static unsigned short *Semaphore_Add = (unsigned short*)COM_SEMAPHOREADRESS;

static unsigned short parityEven = 0;

short Uart_SetCts()
{
    uart_SetCtsBit();
    *Semaphore_Add = 1;
    return (1);
}

short Uart_ClearCts()
{
    uart_ClearCtsBit();     // clear CTS
    *Semaphore_Add = 0;
    return (0);
}

// init function for UART port pins
// reset all portpins
void Uart_PortsHds()         // with handshake
{
    uart_CtsOut();          // CTS output
    Uart_ClearCts();     // clear CTS
    uart_RtsIn();           // RTS input3

    //uart_RtsRen();          // enable pull up resistor on RTS
    uart_TxdSel();
    uart_RxdSel();
}

void Uart_EnableRtsPullUp()
{
  RTS_PULLUP_REN |= RTS_PULLUP_BIT;
  RTS_PULLUP_OUT |= RTS_PULLUP_BIT;
  rtsSetPullUpDir();
}

void UART_DisableHandshake()
{
    HandshakeOn = 0;
    RtsReady = 1;
    com_RtsInterruptDisable();
    Uart_EnableRtsPullUp();
}

void UART_EnableHandshake()
{
    HandshakeOn = 1;
    RtsReady = 0;
    com_RtsIes();          // RTS Interrupt edge select
    com_RtsIe();           // RTS Interrupt enable
    com_RtsIfgClear();     // RTS Interrupt flag clear
    com_RtsIfgSet();       // RTS Interrupt flag set (required for sending first byte)
}

// initilaize UART functions
void Uart_Init(unsigned short mode, unsigned short parity)
{
    if(mode == UART_NO_HANDSHAKE)
    {
        UART_DisableHandshake();
    }
    if(mode == UART_HANDSHAKE)
    {
        UART_EnableHandshake();
    }
    // Ports configuration
    Uart_PortsHds();
    UCA_IE_REGISTER |= UCRXIE;        // enable USCIAx interrupt

    parityEven = parity;
}

void Uart_SetUsb(FET_USB_INFOS_t* usb_Pointers)
{
    UsbPointers_ = *usb_Pointers;
}

void Uart_Close()
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

void UART_SetRts()
{
    RtsReady = 1;
}

// configuration function for UART settings
// UART will be configure automatically about COM channel configuration
short Uart_Config(unsigned long UartBaudrate)
{
    Uart_ClearCts();
    if(UartBaudrate != UartPreviousBaudrate)   // baudrate is changed ?
    {
        UCA_CTRL_REGISTER |= UCSWRST;         // Put state maschine reset
        UCA_CTRL_REGISTER |= UCSSEL__SMCLK;   // clock source: SMCLK

        if(UartBaudrate == 4800)
        {
            #ifdef eZ_FET
                UCA_BR0 = 0x45;
                UCA_BR1 = 0x1;
                UCA_MOD_CTRL_REGISTER = 0x80 + UCOS16;
            #endif
            #ifdef MSP_FET
                UCA_BR0 = 0x1e;
                UCA_BR1 = 0x1;
                UCA_MOD_CTRL_REGISTER = 0xC0 + UCOS16;
            #endif
        }
        if(UartBaudrate == 9600)
        {
            #ifdef eZ_FET
                UCA_BR0 = 0xA2;
                UCA_BR1 = 0x0;
                UCA_MOD_CTRL_REGISTER = 0xC0 + UCOS16;
            #endif
            #ifdef MSP_FET
                UCA_BR0 = 0x8F;
                UCA_BR1 = 0x0;
                UCA_MOD_CTRL_REGISTER = 0xE0 + UCOS16;
            #endif
        }
        else if(UartBaudrate == 14400)
        {
            #ifdef eZ_FET
                UCA_BR0 = 0x6C;
                UCA_BR1 = 0x0;
                UCA_MOD_CTRL_REGISTER = 0x80 + UCOS16;
            #endif
            #ifdef MSP_FET
                UCA_BR0 = 0x5F;
                UCA_BR1 = 0x0;
                UCA_MOD_CTRL_REGISTER =  0x90 + UCOS16;
            #endif
        }
        else if(UartBaudrate == 19200)
        {
            #ifdef eZ_FET
                UCA_BR0 = 0x51;
                UCA_BR1 = 0x0;
                UCA_MOD_CTRL_REGISTER = 0x60 + UCOS16;
            #endif
            #ifdef MSP_FET
                UCA_BR0 = 0x47;
                UCA_BR1 = 0x0;
                UCA_MOD_CTRL_REGISTER = 0xF0 + UCOS16;
            #endif
        }
        else if(UartBaudrate == 28800)
        {
            #ifdef eZ_FET
                UCA_BR0 = 0x36;
                UCA_BR1 = 0x0;
                UCA_MOD_CTRL_REGISTER = 0x40 + UCOS16;
            #endif
            #ifdef MSP_FET
                UCA_BR0 = 0x2F;
                UCA_BR1 = 0x0;
                UCA_MOD_CTRL_REGISTER = 0x50 + UCOS16;
            #endif
        }
        else if(UartBaudrate == 38400)
        {
            #ifdef eZ_FET
                UCA_BR0 = 0x28;
                UCA_BR1 = 0x0;
                UCA_MOD_CTRL_REGISTER = 0xB0 + UCOS16;
            #endif
            #ifdef MSP_FET
                UCA_BR0 = 0x23;
                UCA_BR1 = 0x0;
                UCA_MOD_CTRL_REGISTER = 0x80 + UCOS16;
            #endif
        }
        else if(UartBaudrate == 56000)
        {
            #ifdef eZ_FET
                UCA_BR0 =0x1B;
                UCA_BR1 = 0x0;
                UCA_MOD_CTRL_REGISTER = 0xE0 + UCOS16;
            #endif
            #ifdef MSP_FET
                UCA_BR0 = 0x18;
                UCA_BR1 = 0x0;
                UCA_MOD_CTRL_REGISTER = 0x40 + UCOS16;
            #endif
        }
        else if(UartBaudrate == 57600)
        {
            #ifdef eZ_FET
                UCA_BR0 = 0x1B;
                UCA_BR1 = 0x0;
                UCA_MOD_CTRL_REGISTER = 0x20  + UCOS16;
            #endif
            #ifdef MSP_FET
                UCA_BR0 = 0x17;
                UCA_BR1 = 0x0;
                UCA_MOD_CTRL_REGISTER = 0xA0 + UCOS16;
            #endif
        }
        else if(UartBaudrate == 115200)
        {
            #ifdef eZ_FET
                UCA_BR0 = 0xD;
                UCA_BR1 = 0x0;
                UCA_MOD_CTRL_REGISTER = 0x90  + UCOS16;
            #endif
            #ifdef MSP_FET
                UCA_BR0 = 0xB;
                UCA_BR1 = 0x0;
                UCA_MOD_CTRL_REGISTER = 0xD0 + UCOS16;
            #endif
        }
        else if(UartBaudrate == 256000)
        {
           #ifdef eZ_FET
                UCA_BR0 = 0x6;
                UCA_BR1 = 0x0;
                UCA_MOD_CTRL_REGISTER = 0x4 + UCOS16;
            #endif
            #ifdef MSP_FET
                UCA_BR0 = 0x5;
                UCA_BR1 = 0x0;
                UCA_MOD_CTRL_REGISTER = 0xE0 + UCOS16;
            #endif
        }

        if(parityEven == PARITY_EVEN)
        {
            UCA_CTRL0_REGISTER |= UCPEN | UCPAR;
        }
        else
        {
            UCA_CTRL0_REGISTER &= ~(UCPEN | UCPAR);
            UCA_CTRL_REGISTER = 0;
            UCA_CTRL_REGISTER |= UCSSEL__SMCLK;   // clock source: SMCLK
        }

        UCA_CTRL_REGISTER &=~ UCSWRST;    // Initialize USCI state machine
        UCA_IE_REGISTER |= UCRXIE;        // enable USCIAx interrupt

        UartPreviousBaudrate = UartBaudrate;
    }
    Uart_SetCts();
    return (0);
}

short Uart_GetRts()
{
    if(uart_RtsStatus())    // get RTS status from Target
    {
        return (1);
    }
    else
    {
        return (0);
    }
}

short Uart_NotBusy()
{
    if((UCA_STATUS_REGISTER & UCBUSY) == UCBUSY)   // when busy:
    {
        return (0);                     // return 0
    }
    return (1);                         // else return 1
}

short Uart_TxEmpty()
{
    // UCTXIFG is set when UCAxTXBUF empty
    if(UCTXIFG == (UCTXIFG & UCA_INTERRUPT_FLAG_REGISTER))
    {
        return (1);     // empty
    }
    return (0);         // not empty
}

short Uart_Send()
{
    BYTE BytesInUsbBuffer = UsbPointers_.FetUSB_bytesInUSBBuffer(CDC0_DATA_INTERFACE);
    if(BytesInUsbBuffer)
    {
        if(Uart_NotBusy() && Uart_TxEmpty() && RtsReady)
        {
            if(HandshakeOn == 1)
            {
                // just check for toggle of RTS line if Handshake is used.
                RtsReady = 0;
            }
            // Start USB to UART bridge communication ----------------------------------
            BYTE uartDataToSend = 0;
            UsbPointers_.FetUSB_receiveData(&uartDataToSend, 1 , CDC0_DATA_INTERFACE);
            UCA_TX_BUF = uartDataToSend;

        }
    }
    return 0;
}
