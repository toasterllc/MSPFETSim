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

/*----- FetBslI2c.c -----*/
#include "hw_compiler_specific.h"

#include "msp430.h"
#include "..\FetCom.h"
#include "I2C\I2C.h"
#include "USB_API/USB_Common/types.h"          // Basic Type declarations
#include "FetBsl.h"

static FET_USB_INFOS_t UsbPointers_;
static edt_common_methods_t hilPointers_;

// initilaize FetBSL_functions
void FetBslI2c_Init()
{
    com_RtsInterruptDisable();
    P8SEL &= ~BIT3;    // de-assign P3.3 from UCA0TXD to GPIO
    P8SEL &= ~BIT2;    // de-assign P3.4 from UCA0RXD to GPIO
    I2C_init();
}

void FetBslI2c_SetUsb(FET_USB_INFOS_t* usb_Pointers)
{
    UsbPointers_ = *usb_Pointers;
}

void FetBslI2c_SetHil(edt_common_methods_t* hil_Pointers)
{
    hilPointers_ = *hil_Pointers;
}

short FetBslI2c_Send()
{
    _DINT_FET()
    static short isFirstMessage = 1, isLastMessage = 0;
    static unsigned short dataLength = 0;
      // Start USB to I2C bridge communication ----------------------------------
    BYTE BytesInUsbBuffer = UsbPointers_.FetUSB_bytesInUSBBuffer(CDC0_DATA_INTERFACE);
    if(BytesInUsbBuffer)
    {
        BYTE i2cDataToSend[BSL_MAX_DATA_LENGTH] = {0};
        BYTE returnData[BSL_MAX_DATA_LENGTH] = {0};
        unsigned short readLength = 0;

        UsbPointers_.FetUSB_receiveData(i2cDataToSend, BytesInUsbBuffer , 1);
        if(isFirstMessage)
        {
            dataLength = (i2cDataToSend[2] << 8 |  i2cDataToSend[1]);
            isFirstMessage = 0;
            dataLength = dataLength + OFFSET_BSL_HEADER + OFFSET_BSL_CRC - BytesInUsbBuffer;
            I2C_writeData(I2C_SLAVE_ADDRESS ,i2cDataToSend, BytesInUsbBuffer, I2C_NO_STOP, I2C_START); // start write
        }
        else
        {
            dataLength = dataLength - BytesInUsbBuffer;
            I2C_writeData(I2C_SLAVE_ADDRESS,i2cDataToSend, BytesInUsbBuffer, I2C_NO_STOP, I2C_NO_START);// continue write
        }

        if(dataLength ==  0)
        {
            isLastMessage = 1;
        }

        if(isLastMessage)
        {
            unsigned int i = 0;
            hilPointers_.Delay_1ms(2);
            I2C_readData(I2C_SLAVE_ADDRESS, returnData, &readLength, I2C_REPEATED_START);
            for(i = 0;  i < readLength; i++)
            {
                COM_BASE_Receive(returnData[i]);
            }
            isFirstMessage = 1;
            isLastMessage = 0;
        }
    }
    _EINT_FET()
    return 0;
}
