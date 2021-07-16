/*
* Com_Base.c
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
#include "../FetCom.h"
#include "USB_API/USB_Common/types.h"          // Basic Type declarations
#include "../BSL/FetBsl.h"
#include "../Uart/Uart.h"
#include "../../fw/fet/FetVersion.h"


#define USB_TIMEOUT 10
#define EJECT 1500

static unsigned short RxFifoPosition;
static unsigned char *RxActiveBuffer;
static unsigned char RxBufferFront[COM_FIFOSIZE]={0};
static unsigned char RxBufferBack[COM_FIFOSIZE]={0};
static unsigned char FrontBufferUsed = 1;
static unsigned long PreviousBaudrate = 0;

static unsigned short *Semaphore_Add = (unsigned short*)COM_SEMAPHOREADRESS;

static const unsigned long com_Signature_ @ "UARTSIGNATURE" = COM_SIGNATURE;
#pragma required = com_Signature_

static const unsigned int com_LayerVersion_ @ "UARTLAYERVERSION" = COM_LAYER_VERSION;
static const unsigned int com_LayerVersionCmp_ @ "UARTLAYERVERSIONCMP" = COM_LAYER_VERSION_CMP;


#pragma required = com_LayerVersion_
#pragma required = com_LayerVersionCmp_

static COM_INFOS_t comInfos_;
static edt_common_methods_t hilPointers_;
static DCDC_INFOS_t dcdcPointers_;

static FET_USB_INFOS_t UsbPointers_;

static unsigned short iBytesReceived = 0;
static unsigned short EjectCounter = 0;

// Dummy functions loaded by default
#ifdef MSP_FET
    static short COM_BASE_ConfigModeDummy(unsigned long UartBaudrate){return 0;}
    static void COM_BASE_SetRtsDummy(){return;}
#endif
static short COM_BASE_SetCtsDummy(){return 0;}


// initilaize UART functions
void COM_BASE_Init(COM_INFOS_t* comInfos_Pointer)
{
    // map function pointers in FET UART layer
    // get Infos about UART-module
    comInfos_.comGetLayerVersion      = COM_BASE_GetLayerVersion;
    comInfos_.comConfig               = COM_BASE_Config;
    comInfos_.comReceive              = COM_BASE_Receive;
    comInfos_.comSetHil               = COM_BASE_SetHil;
    comInfos_.comSetDcdc              = COM_BASE_SetDcdc;
    comInfos_.comSetUSB               = COM_BASE_SetUsb;
    comInfos_.comLoop                 = COM_BASE_Loop;
    comInfos_.comClose                = COM_BASE_Close;

    // set CTS to dummy - only needed for UART mode with handshake
    comInfos_.comSetCts               = COM_BASE_SetCtsDummy;
    comInfos_.comClearCts             = COM_BASE_SetCtsDummy;
    comInfos_.comSetRts               = UART_SetRts;

    // UART without handshake is configured  by default
    comInfos_.comTransmit             = Uart_Send;
    comInfos_.comConfigMode           = Uart_Config;

    comInfos_.comGetLayerVersionCmp   = COM_BASE_GetLayerVersionCmp;

    Uart_Init(UART_NO_HANDSHAKE, PARITY_NONE);

     // init Rx FIFO pointer
    RxFifoPosition = 0;
    RxActiveBuffer = RxBufferFront;
    FrontBufferUsed = 1;

    // now copy getLayerVersion and return it to uper core layer
    *comInfos_Pointer =  comInfos_;
}

void COM_BASE_SetHil(edt_common_methods_t* hil_Pointers)
{
    hilPointers_ = *hil_Pointers;

#ifdef MSP_FET
    FetBslI2c_SetHil(hil_Pointers);
#endif
}

void COM_BASE_SetDcdc(DCDC_INFOS_t* dcdc_Pointers)
{
    dcdcPointers_ = *dcdc_Pointers;
}

void COM_BASE_SetUsb(FET_USB_INFOS_t* usb_Pointers)
{
    UsbPointers_ = *usb_Pointers;
    Uart_SetUsb(usb_Pointers);

#ifdef MSP_FET
    FetBslI2c_SetUsb(usb_Pointers);
    FetBsl_SetUsb(usb_Pointers);
#endif
}

void COM_BASE_Close()
{
    RxFifoPosition = 0;
    RxActiveBuffer = RxBufferFront;
    FrontBufferUsed = 1;
    Uart_Close();

#ifdef MSP_FET
    FetBsl_Close();
#endif
}

// return UART layer version
short COM_BASE_GetLayerVersion()
{
    return (COM_LAYER_VERSION);
}

short COM_BASE_GetLayerVersionCmp()
{
    return (COM_LAYER_VERSION_CMP);
}
unsigned short fetTypeInt = 0;

#pragma optimize = low
void COM_BASE_EvaluateBaudRateCommand(unsigned long Baudrate, unsigned short fetType)
{
    *Semaphore_Add = 0;
    fetTypeInt = fetType;
    switch(Baudrate)
    {
        case COM_CLOSE:
        case BSL_DISABLE:
        {
#ifdef MSP_FET
            hilPointers_.ConfigFpgaIoMode(IO_CONFIG_HIGH_Z_UART);
#endif
            COM_BASE_Close();
            break;
        }
        case COM_POWER_DOWN:
        {
            hilPointers_.SetVcc(0);
#ifdef MSP_FET
            hilPointers_.SwitchVccFET(0);
#endif
            break;
        }
        case COM_POWER_UP:
        {
            dcdcPointers_.dcdcSetVcc(3300);
            hilPointers_.SetVcc(3300);
#ifdef MSP_FET
            hilPointers_.SwitchVccFET(1);
#endif
            break;
        }
        case UART_NO_HANDSHAKE:
        case UART_NO_HANDSHAKE_PARITY_EVEN:
        {
            // UART
#ifdef MSP_FET
            hilPointers_.ConfigFpgaIoMode(IO_CONFIG_UART);
#endif
            comInfos_.comTransmit = Uart_Send;
            comInfos_.comConfigMode = Uart_Config;
            //configure FET  to work without handshake - reset to defaults
            comInfos_.comSetCts = COM_BASE_SetCtsDummy;
            comInfos_.comClearCts = COM_BASE_SetCtsDummy;
            if(Baudrate == UART_NO_HANDSHAKE_PARITY_EVEN)
            {
                Uart_Init(UART_NO_HANDSHAKE, PARITY_EVEN);
            }
            else
            {
                Uart_Init(UART_NO_HANDSHAKE, PARITY_NONE);
            }
            break;
        }
        case UART_HANDSHAKE:
        {
#ifdef MSP_FET
            hilPointers_.ConfigFpgaIoMode(IO_CONFIG_UART);
#endif
            comInfos_.comTransmit = Uart_Send;
            comInfos_.comConfigMode = Uart_Config;

            if(fetTypeInt == eZ_FET_WITH_DCDC_NO_FLOWCTL || fetTypeInt == eZ_FET_WITH_DCDC_V2x)
            {
                //configure FET  to work without handshake - reset to defaults
                comInfos_.comSetCts = COM_BASE_SetCtsDummy;
                comInfos_.comClearCts = COM_BASE_SetCtsDummy;
                Uart_Init(UART_NO_HANDSHAKE, PARITY_NONE);
            }
            else
            {
                //configure FET UART to work with handshake
                comInfos_.comSetCts = Uart_SetCts;
                comInfos_.comClearCts  = Uart_ClearCts;
                Uart_Init(UART_HANDSHAKE, PARITY_NONE);
                *Semaphore_Add = comInfos_.comSetCts();
            }
            break;
        }

#ifdef MSP_FET
        case BSL_UART_INVOKE_SEQUENCE:
        case BSL_UART_SEQUENCE:
        {
            //configure FET  to work without handshake - reset to defaults
            comInfos_.comSetCts = COM_BASE_SetCtsDummy;
            comInfos_.comClearCts = COM_BASE_SetCtsDummy;
            comInfos_.comSetRts = COM_BASE_SetRtsDummy;

            FetBsl_Init();
            // Ports configuration
            dcdcPointers_.dcdcSetVcc(3300);
            hilPointers_.SetVcc(3300);
            hilPointers_.SwitchVccFET(1);
            hilPointers_.SetProtocol(2);
            // UART
            hilPointers_.ConfigFpgaIoMode(IO_CONFIG_UART);
            if(BSL_UART_SEQUENCE != Baudrate)
            {
              hilPointers_.BSL_EntrySequence(0);
            }
            // prtocol specific functions
            comInfos_.comTransmit = FetBsl_Send;
            comInfos_.comConfigMode = FetBsl_Config;
            break;
        }
        case BSL_I2C_INVOKE_SEQUENCE1:
        case BSL_I2C_INVOKE_SEQUENCE2:
        case BSL_I2C_INVOKE_SEQUENCE3:
        case BSL_I2C_INVOKE_SEQUENCE4:
        case BSL_I2C_SEQUENCE1:
        case BSL_I2C_SEQUENCE2:
        {
            //configure FET  to work without handshake - reset to defaults
            comInfos_.comSetCts = COM_BASE_SetCtsDummy;
            comInfos_.comClearCts = COM_BASE_SetCtsDummy;
            comInfos_.comSetRts = COM_BASE_SetRtsDummy;

            // Ports configuration
            dcdcPointers_.dcdcSetVcc(3300);
            hilPointers_.SetVcc(3300);
            hilPointers_.SwitchVccFET(1);
            hilPointers_.SetProtocol(2);
            //I2C config
            hilPointers_.ConfigFpgaIoMode(IO_CONFIG_HIGH_Z_UART);
            hilPointers_.ConfigFpgaIoMode(IO_CONFIG_I2C);
            if(BSL_I2C_SEQUENCE1 != Baudrate && BSL_I2C_SEQUENCE2 != Baudrate)
            {
              hilPointers_.BSL_EntrySequence(0);
            }
            FetBslI2c_Init();
            // prtocol specific functions
            comInfos_.comTransmit = FetBslI2c_Send;
            comInfos_.comConfigMode = COM_BASE_ConfigModeDummy;
            break;
        }
#endif
        default:
        {
            if(Baudrate != BSL_I2C_INVOKE_SEQUENCE2 && Baudrate != BSL_I2C_INVOKE_SEQUENCE1 &&
               Baudrate != BSL_UART_INVOKE_SEQUENCE && Baudrate != UART_HANDSHAKE &&
               Baudrate != UART_NO_HANDSHAKE && Baudrate !=  COM_POWER_UP &&
               Baudrate != COM_CLOSE && Baudrate != COM_POWER_DOWN &&
               Baudrate != BSL_I2C_INVOKE_SEQUENCE3 &&
               Baudrate != BSL_I2C_INVOKE_SEQUENCE4 && Baudrate != BSL_I2C_SEQUENCE1 &&
               Baudrate != BSL_I2C_SEQUENCE2 && Baudrate != BSL_UART_SEQUENCE )
            {
                // Uart without handshake is set as default
            #ifdef MSP_FET
                hilPointers_.ConfigFpgaIoMode(IO_CONFIG_UART);
            #endif
            }
        }
    }
}

// configuration function for UART settings
// UART will be configure automatically about COM channel configuration
short COM_BASE_Config(unsigned long Baudrate, unsigned long MCLK_Frequency,
                      unsigned short fetType)
{
    if(Baudrate != PreviousBaudrate)    // baudrate changed ?
    {
       COM_BASE_EvaluateBaudRateCommand(Baudrate, fetType);
       comInfos_.comConfigMode(Baudrate);
       PreviousBaudrate = Baudrate;
    }
    return (0);
}

void COM_BASE_SwitchBuffer()
{
    RxFifoPosition = 0;
    if(FrontBufferUsed == 1)
    {
          RxActiveBuffer = RxBufferBack;
          FrontBufferUsed = 0;
    }
    else
    {
          RxActiveBuffer = RxBufferFront;
          FrontBufferUsed = 1;
    }
}

unsigned char *COM_BASE_GetBuffer()
{
    unsigned char *activePtr = 0;
    if(FrontBufferUsed == 0)
    {
          activePtr = RxBufferFront;
    }
    else
    {
          activePtr = RxBufferBack;
    }
    return activePtr;
}

short COM_BASE_Receive(unsigned char character)
{
    *Semaphore_Add = comInfos_.comClearCts();
    if(RxFifoPosition != COM_FIFOSIZE)
    {
        RxActiveBuffer[RxFifoPosition++] = character;
        iBytesReceived++;
    }
    else
    {
        COM_BASE_SwitchBuffer();
        RxActiveBuffer[RxFifoPosition++] = character;
        iBytesReceived = 1;
        EjectCounter = 0;
        UsbPointers_.FetUsb_CdcSendDataInBackground(COM_BASE_GetBuffer(), COM_FIFOSIZE, CDC0_DATA_INTERFACE, USB_TIMEOUT);
    }
    *Semaphore_Add = comInfos_.comSetCts();
    return 0;
}

void COM_BASE_Loop()
{
    comInfos_.comTransmit();
    // Start USB to Host communication (Uart)-----------------------------------
    if(iBytesReceived > 0)
    {
        EjectCounter++;
        if(EjectCounter >= EJECT)
        {
            *Semaphore_Add = comInfos_.comClearCts();
            UCA_IE_REGISTER &= ~UCRXIE;        // disable USCIAx interrupt
            unsigned short tmpbyte = iBytesReceived;
            iBytesReceived = 0;
            COM_BASE_SwitchBuffer();
            BYTE sendResult = UsbPointers_.FetUsb_CdcSendDataInBackground(COM_BASE_GetBuffer(), tmpbyte, CDC0_DATA_INTERFACE, 10);
            if( sendResult == 0)
            {
                EjectCounter = 0;
            }
            UCA_IE_REGISTER |= UCRXIE;        // enable USCIAx interrupt
            *Semaphore_Add = comInfos_.comSetCts();
        }
    }
    // End USB to Host communication (Uart)-------------------------------------
}
