/*
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
#ifndef BSL_PI_H
#define BSL_PI_H

#define RX_PACKET_ONGOING     0x00
#define DATA_RECEIVED         BIT0
#define RX_ERROR_RECOVERABLE  BIT1
#define RX_ERROR_REINIT       BIT2
#define RX_ERROR_FATAL        BIT3
#define PI_DATA_RECEIVED      BIT4

#define PI_COMMAND_UPPER 0x50

//PI version Signifiers
#define BSL430_TIMER_A_PI   0x00
#define BSL430_USB_PI       0x30
#define BSL430_USCIA_PI     0x50
#define BSL430_eUSCIA_PI    0x70
#define BSL430_USCIB_I2C_PI 0x090
#define BSL430_eUSCIB_I2C_PI 0x0A0
#define BSL430_eUSCI_I2C_UART_PI 0xB0
/*******************************************************************************
* *Function:    init
* *Description: Initialize the peripheral and ports to begin TX/RX
*******************************************************************************/
void PI_init();

/*******************************************************************************
* *Function:    PI_receivePacket
* *Description: Reads an entire packet, verifies it, and sends it to the core to be interpreted
* *Returns:
*             DATA_RECEIVED         A packet has been received and can be processed
*             RX_ERROR_RECOVERABLE  An error has occured, the function can be called again to
*                                   receive a new packet
*******************************************************************************/
char PI_receivePacket();

/*******************************************************************************
* *Function:    PI_sendData
* *Description: Sends the data in the data buffer
* *Parameters:
*             int bufSize - the number of bytes to send
*******************************************************************************/
void PI_sendData(int bufSize);

/*******************************************************************************
* *Function:    PI_getBufferSize
* *Description: Returns the max Data Buffer Size for this PI
* *Returns:     max buffer size
*******************************************************************************/
int PI_getBufferSize();

#endif
