/*
* FetCom.h
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

/*----- FetUart.h -----*/
#ifndef FETCOM_H
#define FETCOM_H

#include "hil_Structs.h"
#include "FetDcdc.h"
#include "FetUsb.h"

#ifdef eZ_FET

    // UART port direction
    #define uart_CtsOut()       {P2DIR |= BIT7;}    // CTS output
    #define uart_RtsIn()        {P2DIR &=~ BIT6;}   // RTS input

    // UART port
    #define uart_SetCtsBit()    {P2OUT |= BIT7;}    // Output
    #define uart_ClearCtsBit()  {P2OUT &=~ BIT7;}   // Output clear
    #define uart_RtsRen()       {P2REN |= BIT6;}    // RTS resistor
    #define uart_CtsRen()       {P2REN |= BIT7;}    // CTS resistor

    #define com_RtsIfgSet()            {P2IFG |= BIT6;}    // RTS Interrupt flag set (required for sending first byte)
    #define com_RtsIfgClear()          {P2IFG &=~ BIT6;}   // RTS Interrupt flag clear
    #define com_RtsIe()                {P2IE  |= BIT6;}    // RTS Interrupt enable
    #define com_RtsInterruptDisable()  {P2IE &= ~BIT6;}    // RTS Interrupt disable
    #define com_RtsIes()               {P2IES &=~ BIT6;}   // RTS Interrupt edge select (is set when a low-high transition)

    // UART status bits
    #define uart_RtsStatus()    ((P2IN & BIT6)==BIT6)

    // UART port select
    #define uart_TxdSel()       {P3SEL |= BIT3;}    // assign P3.3 to UCA0TXD
    #define uart_RxdSel()       {P3SEL |= BIT4;}    // assign P3.4 to UCA0RXD


    // UART Register and bit abstractions
    #define TX_RX_PORT_SEL    ( P3SEL )
    #define TX_RX_PORT_DIR    ( P3DIR )
    #define TX_PORT_BIT       ( BIT3 )
    #define RX_PORT_BIT       ( BIT4 )
    #define UCA_CTRL_REGISTER ( UCA0CTL1 )
    #define UCA_CTRL0_REGISTER ( UCA0CTL0 )
    #define UCA_BAUD_CTRL_REGISTER ( UCA0BRW )
    #define UCA_MOD_CTRL_REGISTER  ( UCA0MCTL )
    #define UCA_IE_REGISTER   ( UCA0IE )
    #define UCA_STATUS_REGISTER ( UCA0STAT )
    #define UCA_INTERRUPT_FLAG_REGISTER  ( UCA0IFG )
    #define UCA_TX_BUF          ( UCA0TXBUF )
    #define UCA_RX_BUF          ( UCA0RXBUF )

    #define UCA_BR0         ( UCA0BR0 )
    #define UCA_BR1         ( UCA0BR1 )

    #define RTS_PORT_OUT ( P2OUT )
    #define RTS_PORT_DIR ( P2DIR )
    #define RTS_PORT_BIT ( BIT7 )

    #define RTS_PULLDOWN_OUT ( P2OUT )
    #define RTS_PULLDOWN_REN ( P2REN )
    #define RTS_PULLDOWN_BIT ( BIT6 )
    #define rtsSetPullDownDir() (P2DIR &= ~BIT6)
// CHECKME: Bit is cleared in both cases
    #define rtsClearPullDownDir() (P2DIR &= ~BIT6)
    #define RTS_PULLUP_REN ( P2REN )
    #define RTS_PULLUP_OUT ( P2OUT )
    #define RTS_PULLUP_BIT ( BIT6 )
    #define rtsSetPullUpDir() (P2DIR &= ~BIT6)

    #define CTS_PORT_OUT ( P2OUT )
    #define CTS_PORT_DIR ( P2DIR )
    #define CTS_PORT_REN ( P2REN )
    #define CTS_PORT_BIT ( BIT6 )
#endif

#ifdef MSP_FET

    // UART port direction
    #define uart_CtsOut()       {P9DIR |= BIT5;}    // CTS output
    #define uart_RtsIn()        {P2DIR &=~ BIT1;}   // "RTS" input (no RTS on MSPFET)


    // UART port
    #define uart_SetCtsBit()    {P9OUT |= BIT5;}    // Output
    #define uart_ClearCtsBit()  {P9OUT &=~ BIT5;}   // Output clear
    #define uart_RtsRen()       {P2REN |= BIT1;}    // RTS resistor
    #define uart_CtsRen()       {P9REN |= BIT5;}    // CTS resistor

    #define com_RtsIfgSet()            {P2IFG = 0x00; P2IFG |= BIT1;}    // RTS Interrupt flag set (required for sending first byte)
    #define com_RtsIfgClear()          {P2IFG = 0x00; /*P2IFG &=~ BIT1;*/}   // RTS Interrupt flag clear
    #define com_RtsIe()                {P2IE  = 0x00; P2IE  |= BIT1;}    // RTS Interrupt enable
    #define com_RtsInterruptDisable()  {P2IE &= ~BIT1;}    // RTS Interrupt disable
    #define com_RtsIes()               {P2IES = 0x00; P2IES &=~ BIT1;}   // RTS Interrupt edge select (is set when a low-high transition)

    // UART status bits
    #define uart_RtsStatus()    ((P2IN & BIT1)==BIT1)

    // UART port select
    #define uart_TxdSel()       {P8SEL |= BIT3;}    // assign P3.3 to UCA0TXD
    #define uart_RxdSel()       {P8SEL |= BIT2;}    // assign P3.4 to UCA0RXD

    // UART Register and bit abstractions
    #define TX_RX_PORT_SEL    ( P8SEL )
    #define TX_RX_PORT_DIR    ( P8DIR )
    #define TX_PORT_BIT       ( BIT2 )
    #define RX_PORT_BIT       ( BIT3 )
    #define UCA_CTRL_REGISTER ( UCA1CTL1 )
    #define UCA_CTRL0_REGISTER ( UCA1CTL0 )
    #define UCA_BAUD_CTRL_REGISTER ( UCA1BRW )
    #define UCA_MOD_CTRL_REGISTER ( UCA1MCTL )
    #define UCA_IE_REGISTER   ( UCA1IE )
    #define UCA_STATUS_REGISTER ( UCA1STAT )
    #define UCA_INTERRUPT_FLAG_REGISTER  ( UCA1IFG )
    #define UCA_TX_BUF          ( UCA1TXBUF )
    #define UCA_RX_BUF          ( UCA1RXBUF )

    #define UCA_BR0         ( UCA1BR0 )
    #define UCA_BR1         ( UCA1BR1 )

    #define RTS_PORT_OUT ( P2OUT )
    #define RTS_PORT_DIR ( P2DIR )
    #define RTS_PORT_BIT ( BIT1 )

    #define RTS_PULLDOWN_OUT ( P2OUT )
    #define RTS_PULLDOWN_REN ( P2REN )
    #define RTS_PULLDOWN_BIT ( BIT1 )
    #define rtsSetPullDownDir() (P2DIR |=  BIT1)
// CHECKME: Bit is set in both cases
    #define rtsClearPullDownDir() (P2DIR |=  BIT1)
    #define RTS_PULLUP_REN ( P2REN )
    #define RTS_PULLUP_OUT ( P2OUT )
    #define RTS_PULLUP_BIT ( BIT1 )
    #define rtsSetPullUpDir() (P2DIR |=  BIT1)

    #define CTS_PORT_OUT ( P9OUT )
    #define CTS_PORT_DIR ( P9DIR )
    #define CTS_PORT_REN ( P9REN )
    #define CTS_PORT_BIT ( BIT5 )
#endif

#define IO_CONFIG_HIGH_Z_UART           0x0
#define IO_CONFIG_UART              	0x1
#define IO_CONFIG_I2C               	0x2

#define COM_SIGNATURE     0xACDCACDCul

#ifdef MSP_FET
    #define COM_FIFOSIZE      256ul
#endif

#ifdef eZ_FET
    #define COM_FIFOSIZE      64ul
#endif

//UART commands
#define COM_CLOSE 9620ul
#define UART_NO_HANDSHAKE 9621ul
#define UART_NO_HANDSHAKE_PARITY_EVEN 9625ul
#define UART_HANDSHAKE 9622ul
#define COM_POWER_UP 9623ul
#define COM_POWER_DOWN 9624ul

#define PARITY_EVEN 1
#define PARITY_NONE 0

// BSL Commands
#define BSL_UART_INVOKE_SEQUENCE 9601ul
#define BSL_UART_SEQUENCE 9602ul
#define BSL_DISABLE 8001ul
#define BSL_MAX_DATA_LENGTH  (260*2)
#define BSL_I2C_INVOKE_SEQUENCE1 100000ul
#define BSL_I2C_INVOKE_SEQUENCE2 400000ul
#define BSL_I2C_INVOKE_SEQUENCE3 100001ul
#define BSL_I2C_INVOKE_SEQUENCE4 400001ul
#define BSL_I2C_SEQUENCE1 100002ul
#define BSL_I2C_SEQUENCE2 400002ul

struct COM_INFOS
{
    short (*comGetLayerVersion)(void);
    short (*comConfig)(unsigned long Baudrate, unsigned long MCLK_Frequency, unsigned short);
    short (*comTransmit)(void);
    short (*comReceive)(unsigned char character);
    void  (*comClose)(void);
    void  (*comSetHil)(edt_common_methods_t*);
    void  (*comSetDcdc)(DCDC_INFOS_t*);
    void  (*comSetUSB)(FET_USB_INFOS_t*);
    void  (*comLoop)(void);
    short (*comConfigMode)(unsigned long Baudrate);
    short (*comSetCts)(void);
    short (*comClearCts)(void);
    void  (*comSetRts)(void);
    short (*comGetLayerVersionCmp)(void);
};
typedef struct COM_INFOS COM_INFOS_t;

short COM_BASE_GetLayerVersion();
short COM_BASE_GetLayerVersionCmp();
short COM_BASE_Config(unsigned long Baudrate, unsigned long MCLK_Frequency,
                      unsigned short fetType);
short COM_BASE_Receive(unsigned char character);
void COM_BASE_Init(COM_INFOS_t* uartInfos_Pointer);
void COM_BASE_SetHil(edt_common_methods_t* hil_Pointers);
void COM_BASE_SetDcdc(DCDC_INFOS_t* dcdc_Pointers);
void COM_BASE_SetUsb(FET_USB_INFOS_t* usb_Pointers);
void COM_BASE_Close();
void COM_BASE_Loop();

#endif
