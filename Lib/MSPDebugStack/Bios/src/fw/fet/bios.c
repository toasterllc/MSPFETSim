/*
 * bios.c
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

#include "hw_compiler_specific.h"
#include "bios.h"
#include <string.h>

#include "USB_API/USB_Common/types.h"
#include "USB_API/USB_CDC_API/UsbCdc.h"
#include "USB_config/descriptors.h"
#include "usbConstructs.h"
#include "USB_API/USB_Common/usb.h"
#include "communicationDefs.h"
#include "protocol.h"
#include "FetVersion.h"

//! \brief count of LED from uif
#define BIOS_LED_COUNT 2

//! \brief pin (from TUSB) for hardware handshake for TX cannel
//! \details also used as TRUE value for bios_xoff_
#define BIOS_HARD_RTS_BIT 3

//! \brief version of bios code
const unsigned short core_version_ @ "COREVERSION" = CORE_VERSION;
#pragma required=core_version_

unsigned short tool_id_;
#pragma required=tool_id_

//! \brief chars for flow control
const unsigned char BIOS_DLE_CHAR  = 0x10;
//! \brief chars for flow control
const unsigned char BIOS_XON_CHAR  = 0x11;
//! \brief chars for flow control
const unsigned char BIOS_XOFF_CHAR = 0x13;

//**************************** Internal function prototypes

//! \brief clears the TX software buffer
void  bios_UsbTxClear(void);

//! \brief dummy to catch all irq
//! \details biosHalInterfaceClear set all by bios unused irqs to dummy
INTERRUPT_PROTO void dummyIsr (void);

//! \brief receive char form UART
//! \details get chars form hardware RX buffer and stor them in the RX software buffer(s).
//! Messages are checked on crc. and ack/exceptions processed in this function.
void usbRxIsr (unsigned char receivedByte);

//! \brief 10ms timer interrupt
//! \details count all 10ms the timer for RX and TX timeouts.
//! Also the timer for LED drive.
INTERRUPT_PROTO void timerB1Isr (void);

//! \brief function is called by hardware interrupt if TX buffer goes empty
//! \details move a char from the TX software buffer to the hardware TX buffer.
void BIOS_usbTxData (void);

//! \brief Buffer for the RX cannel
//! \details actual two buffers used, one for working, the second to catch char which receivend before
//! the cts signal have an effect.
volatile BiosRxRecord bios_rx_record_ = {{0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, 0, {0,0,0,0}, 0, 0, 0, {0,0,0,0}};
//! \brief Buffer for the TX cannel
//! \details only one buffer in uif. There is no need for a second. The dll play ping pong with the uif
volatile BiosTxRecord bios_tx_record_ = {0, 0, {0}, {0}, {0}, {0}, {0}, NULL, 0};

//! \brief Timer for RX and TX timeouts
//! \details timebase 10ms
volatile BiosGlobalTimer bios_global_timer_[BIOS_TIMER_COUNT];
//! \brief uif is under control of the eg. workbench
//! \details \li 0, not under control, reset by EXCEPTION NULL
//! \li 1, set in v3opRx with the first valid command
char bios_wb_control_ = 0;

//! \brief values for (re)set LEDs
const unsigned char BIOS_LED_OFF   = 0x00;
//! \brief values for (re)set LEDs
const unsigned char BIOS_LED_ON    = 0x01;
//! \brief values for (re)set LEDs
const unsigned char BIOS_LED_BLINK = 0x02;

volatile SystemEventMsp system_event_msp_;

//! \brief for LED driving
struct _BiosLedTimer_
{
  unsigned char  mode;
  unsigned short load;
  unsigned short counter;
  unsigned char  *addr;
  unsigned char bit;
};
typedef struct _BiosLedTimer_ BiosLedTimer;

//! \brief for LED driving
volatile BiosLedTimer bios_leds_[BIOS_LED_COUNT];

//! \brief the LED from the EASY :-)
//! \details The EASY haven't LEDs, this var is a replacment for the port to catch
//! the LED driving
unsigned char bios_led_dummy_;

//! \brief Hold the crystal frequency
//! \details will set in biosInitSystem depended for the detected hardware
//! will used eg. from timers and UART
unsigned long crystal_;

//! \brief modulation register value
//! \details will set in biosInitSystem depended for the detected hardware
unsigned char baudmod_ = 0;

//! \brief flags for hardware features
//! \details will set in biosInitSystem depended for the detected hardware
//! \li bit 0 -> handshake xon/xoff = 1, hw handshake = 0
//! \li bit 1 -> 4 wire
//! \li bit 2 -> 2 wire
unsigned long bios_device_flags_ = 0;

//! \brief hardware version
//! \details will set in biosInitSystem depended for the detected hardware
unsigned short bios_info_hw_0_;

//! \brief hardware version
//! \details will set in biosInitSystem depended for the detected hardware
unsigned short bios_info_hw_1_;

//! \brief gets the received char
unsigned char bios_rx_char_;

//! \brief dummy to switch of XON/XOFF in easy
const unsigned char bios_rx_char_always_0 = 0;

//! \brief points to bios_rx_char_ if XON/XOFF is active
//! \details on EASY it points to bios_rx_char_always_0 to disable XON/XOFF functionality
//! will set in biosInitSystem depended for the detected hardware
unsigned char *bios_rx_char_ptr_;

//! \brief Flag for XON/XOFF status
//! \li 0, TX can send
//! \li 1, TX must not be send
//! \details between receiving xoff and sending chars is a delay (by program flow). This
//! is not a problem, because the TUSB has enougth reserve in the buffer.
unsigned char bios_xoff_ = 0;

//! \brief pointer to active TX flow control
//! \details \li on EASY it points to the PORT with the handshakesignals
//! \li on uif it points to the (software)flag bios_xoff_
//! will set in biosInitSystem depended for the detected hardware
unsigned char *bios_xoff_ptr_ = &bios_xoff_;

//! \brief Flag for error indication
//! \details \li 1, an error message is to send. Set by every function which can generate an error
//! \li 0, reset in biosmainLoop, if error message was send
volatile char  bios_rx_err_set_ = 0;

//! \brief Message ID from received message which provoke the error.
volatile unsigned char bios_rx_err_id_ = 0;

//! \brief error code
//! \li EXCEPTION_NOT_IMPLEMENT_ERR 	0x8001
//! \li EXCEPTION_MSGID_ERR	        0x8002
//! \li EXCEPTION_CRC_ERR	        0x8003
//! \li EXCEPTION_RX_TIMEOUT_ERR	0x8004
//! \li EXCEPTION_TX_TIMEOUT_ERR	0x8005
//! \li EXCEPTION_RX_OVERFLOW_ERR     	0x8006
//! \li EXCEPTION_TX_NO_BUFFER	        0x8007
//! \li EXCEPTION_COM_RESET	        0x8008
//! \li EXCEPTION_RX_NO_BUFFER	        0x8009
//! \li EXCEPTION_RX_TO_SMALL_BUFFER	0x800A
//! \li EXCEPTION_RX_LENGTH	        0x800B
//! \li HAL specific exceptions	0xFFFF to 0xFF00 (-1 to -255)
volatile BIOS_RxError_t BIOS_RxEr;

//! \brief Pointer to additional information
//! \details if an error provide more information about the error
volatile unsigned short *bios_rx_err_payload_ = NULL;

//! \brief target for dummyread form UART
volatile unsigned char usb_read_dummy_;

volatile unsigned short CTS_LINE = 0;

//**************************** Module Implementation ******************/
void BIOS_InitSystem (void)
{    //Init Timer A1 (currently used for TX and RX timeouts and LED's)
    //generates interrupt every 10ms
    TA1CCR0 = USB_MCLK_FREQ / (8 * 100) - 1;
    TA1CTL = TASSEL__SMCLK | ID__8 | MC__UP/*MC__CONTINOUS*/ | TACLR | TAIE;

#ifdef eZ_FET
    // LED assignments for eZ-FET
    bios_leds_[0].addr = (unsigned char*)&P1OUT;
    bios_leds_[0].bit = BIT2;
    bios_leds_[1].addr = (unsigned char*)&P1OUT;
    bios_leds_[1].bit = BIT3;
#endif

#ifdef MSP_FET
    // LED assignments for MSP-FET
    bios_leds_[0].addr = (unsigned char*)&P6OUT;
    bios_leds_[0].bit = BIT7;
    bios_leds_[1].addr = (unsigned char*)&P5OUT;
    bios_leds_[1].bit = BIT3;
#endif

    /// is detected!!
    // char set to rx input for xon/xoff
    bios_rx_char_ptr_ = &bios_rx_char_;

    // disalbe BSL protection
    SYSBSLC &= ~SYSBSLPE;
    // get tool id stored in BSL area
    tool_id_ = *(unsigned short*)0x100e;

    // enable BSL protection again
    SYSBSLC |= SYSBSLPE;

    bios_info_hw_0_ = tool_id_;
    bios_info_hw_1_ = tool_id_;
}

void BIOS_SuspendSystem(void)
{}

void BIOS_ResumeSystem(void)
{}

void BIOS_SetCts()
{
    CTS_LINE = FALSE;  bios_global_timer_[0].state |= BIOS_TIMER_BREAK;
}
void BIOS_ResetCts()
{
    CTS_LINE = TRUE; bios_global_timer_[0].state &= ~BIOS_TIMER_BREAK;
}

unsigned short Bios_getCore_version()
{
    return CORE_VERSION;
}
unsigned short Bios_getTool_id()
{
    return tool_id_;
}
unsigned short Bios_getInfo_hw_0()
{
    return bios_info_hw_0_;
}
unsigned short Bios_getInfo_hw_1()
{
    return bios_info_hw_1_;
}

//*************************************
short BIOS_LedOn(unsigned char no)
{
    if(no < BIOS_LED_COUNT)
    {
        bios_leds_[no].mode = BIOS_LED_ON;
        bios_leds_[no].load = 0;
        bios_leds_[no].counter = 0;
    }
    return(0);
}

//************************************
short BIOS_LedOff(unsigned char no)
{
    if(no < BIOS_LED_COUNT)
    {
        bios_leds_[no].mode = BIOS_LED_OFF;
        bios_leds_[no].load = 0;
        bios_leds_[no].counter = 0;
    }
    return(0);
}

//************************************
short BIOS_LedBlink(unsigned char no,unsigned short time)
{
    if(no < BIOS_LED_COUNT)
    {
        bios_leds_[no].load = time * 2;
        bios_leds_[no].counter = time * 2;
        bios_leds_[no].mode = BIOS_LED_BLINK;
    }
    return(0);
}

//************************************
short BIOS_LedFlash(unsigned char no,unsigned short time)
{
    if((no < BIOS_LED_COUNT) && !bios_leds_[no].counter)
    {
        bios_leds_[no].load = time * 2;
        bios_leds_[no].counter = time * 2;
        bios_leds_[no].mode &= ~BIOS_LED_BLINK;
    }
    return(0);
}

//************************************
short BIOS_LedAlternate(unsigned short time)
{
    bios_leds_[0].load = time * 2;
    bios_leds_[0].counter = time;
    bios_leds_[1].load = time * 2;
    bios_leds_[1].counter = time;
    if(time)
    {
        bios_leds_[0].mode = BIOS_LED_ON | BIOS_LED_BLINK;
        bios_leds_[1].mode = BIOS_LED_OFF | BIOS_LED_BLINK;
    }
    else
    {
        bios_leds_[0].mode = BIOS_LED_OFF;
        bios_leds_[1].mode = BIOS_LED_OFF;
    }
    return(0);
    }

//************************************
void BIOS_GlobalError(void)
{
    BIOS_LedOn(BIOS_LED_MODE);
    BIOS_LedOff(BIOS_LED_POWER);
    while(1);
}

//INTERRUPT(TIMER1_A1_VECTOR) void timerB1Isr (void)
//***********************************************
INTERRUPT(TIMER1_A1_VECTOR) void timerB1Isr (void)
{
    if(TA1IV == TA1IV_TA1IFG)
    {
        // process global timers
		if((bios_global_timer_[BIOS_TIMER_RX].count > 1) && !(bios_global_timer_[BIOS_TIMER_RX].state & BIOS_TIMER_BREAK))
        {
            bios_global_timer_[BIOS_TIMER_RX].count--;
        }
        else if(bios_global_timer_[BIOS_TIMER_RX].count == 1)
        {
            bios_global_timer_[0].count = 0;
            BIOS_UsbRxError(EXCEPTION_RX_TIMEOUT_ERR);
        }
        if((bios_global_timer_[BIOS_TIMER_TX].count > 1) && !(bios_xoff_))
        {
            bios_global_timer_[BIOS_TIMER_TX].count--;
        }
        // LED mode
        DIAG_SUPPRESS(Pa082)
        if((bios_leds_[BIOS_LED_MODE].mode & BIOS_LED_ON) ^ (bios_leds_[BIOS_LED_MODE].counter > (bios_leds_[BIOS_LED_MODE].load/2)))
        {
            *bios_leds_[BIOS_LED_MODE].addr |= bios_leds_[BIOS_LED_MODE].bit;
        }
        else
        {
            *bios_leds_[BIOS_LED_MODE].addr &= ~bios_leds_[BIOS_LED_MODE].bit;
        }
        DIAG_DEFAULT(Pa082)
        if(bios_leds_[BIOS_LED_MODE].counter > 0)
        {
            bios_leds_[BIOS_LED_MODE].counter--;
        }
        else if(bios_leds_[BIOS_LED_MODE].mode & BIOS_LED_BLINK)
        {
            bios_leds_[BIOS_LED_MODE].counter = bios_leds_[BIOS_LED_MODE].load;
        }

        // LED power
        DIAG_SUPPRESS(Pa082)
        if((bios_leds_[BIOS_LED_POWER].mode & BIOS_LED_ON) ^ (bios_leds_[BIOS_LED_POWER].counter > (bios_leds_[BIOS_LED_POWER].load/2)))
        {
            *bios_leds_[BIOS_LED_POWER].addr |= bios_leds_[BIOS_LED_POWER].bit;
        }
        else
        {
            *bios_leds_[BIOS_LED_POWER].addr &= ~bios_leds_[BIOS_LED_POWER].bit;
        }
        DIAG_DEFAULT(Pa082)
        if(bios_leds_[BIOS_LED_POWER].counter > 0)
        {
            bios_leds_[BIOS_LED_POWER].counter--;
        }
        else if(bios_leds_[BIOS_LED_POWER].mode & BIOS_LED_BLINK)
        {
            bios_leds_[BIOS_LED_POWER].counter = bios_leds_[BIOS_LED_POWER].load;
        }
    }
}

//**********************************************************
INTERRUPT_PROTO void dummyIsr (void)
{
    ;
}
const volatile unsigned short *bios_intvec_ = (unsigned short*)0xFF80;

//! \brief Pointer to HAL IRQ vector table
RO_PLACEMENT_NO_INIT volatile const unsigned long   hal_intvec_    @ "HALINTVEC";
RO_PLACEMENT_NO_INIT volatile const unsigned long   hal_signature_ @ "HALSIGNATURE";

RO_PLACEMENT_NO_INIT volatile const unsigned long   dcdc_intvec_   @ "DCDCIVEC";
RO_PLACEMENT_NO_INIT volatile const unsigned long   dcdc_signature_ @ "DCDCSIGNATURE";

RO_PLACEMENT_NO_INIT volatile const unsigned long   hil_signature_ @ "HILSIGNATURE";
RO_PLACEMENT_NO_INIT volatile const unsigned short  hil_Start_UP_ @ "HILINIT";

RO_PLACEMENT_NO_INIT volatile const unsigned long   com_intvec_   @ "COMINTVEC";
RO_PLACEMENT_NO_INIT volatile const unsigned long   com_signature_ @ "COMSIGNATURE";
RO_PLACEMENT_NO_INIT volatile const long  com_semaphore_ @ "COM_SEMAPHOREADRESS";

unsigned long Bios_getHal_intvec()
{
   return hal_intvec_;
}
unsigned short Bios_getHil_intvec()
{
   return hil_Start_UP_;
}
unsigned long Bios_getDcdc_intvec()
{
   return dcdc_intvec_;
}
unsigned long Bios_getCom_intvec()
{
   return com_intvec_;
}
unsigned long Bios_getDcdc_signature()
{
   return dcdc_signature_;
}
unsigned long Bios_getHal_signature()
{
    return hal_signature_;
}
unsigned long Bios_getHil_signature()
{
    return hil_signature_;
}
unsigned long Bios_getCom_signature()
{
    return com_signature_;
}
//***************************************
unsigned short tempInDataRx[4][BIOS_RX_SIZE]= {0x0,0x0};

//***************************************
void BIOS_InitCom(void)
{
    unsigned char i;

    for(i = 0; i < BIOS_RX_QUEUS; i++)
    {
        bios_rx_record_.datas[i] = tempInDataRx[i];
        if(bios_rx_record_.datas[i] == NULL)
        {
            BIOS_GlobalError();
        }
        bios_rx_record_.data[i] = (unsigned char*)bios_rx_record_.datas[i];
    }
    for(i = 0; i < BIOS_TX_QUEUS; i++)
    {
        bios_tx_record_.data[i] = (unsigned char*)bios_tx_record_.datas[i];
    }
}
//*****************************************
void BIOS_PrepareTx(unsigned int size)
{
    bios_tx_record_.send_counter[bios_tx_record_.cannel_to_send] = 0;
    DIAG_SUPPRESS(Pa082)
    bios_tx_record_.count[bios_tx_record_.active] = size;
    bios_tx_record_.state[bios_tx_record_.active] |= BIOS_TX_TO_SEND;
    DIAG_DEFAULT(Pa082)
}

//********************************************
void BIOS_StartTx(void)
{
    bios_global_timer_[BIOS_TIMER_TX].count = BIOS_TX_TIMEOUT;
    BIOS_usbTxData();
}

//*********************************************
void BIOS_UsbTxError(void)
{
    BIOS_UsbTxClear();
}

//*********************************************
void BIOS_UsbTxClear(void)
{
    unsigned char i;

    memset((unsigned char*)&bios_tx_record_, 0, sizeof(bios_tx_record_));
    bios_global_timer_[BIOS_TIMER_TX].count = 0;
    bios_rx_char_ = 0;
    for(i = 0; i < BIOS_TX_QUEUS; i++)
    {
        bios_tx_record_.data[i] = (unsigned char*)bios_tx_record_.datas[i];
        bios_tx_record_.state[i] = 0;
    }
}

//***************************************
void BIOS_UsbRxClear(void)
{
    unsigned char i;

    bios_rx_record_.active = 0;
    for(i = 0; i < BIOS_RX_QUEUS; i++)
    {
        bios_rx_record_.state[i] = 0;
        bios_rx_record_.last_cmd_typ = 0;
        bios_rx_record_.last_msg_id = 0;
        bios_rx_record_.data[i][0] = 0;
        bios_rx_record_.crc[i] = 0;
        bios_rx_record_.count[i] = 0;
        bios_rx_record_.size[i] = 0;
    }
    bios_global_timer_[0].count = 0;
}

short BIOS_IsUsbRxError()
{
    if(BIOS_RxEr.bios_rx_err_set_)
    {
        return TRUE;
    }
    return FALSE;
}

void BIOS_ClearUsbRxError()
{
    BIOS_RxEr.bios_rx_err_code_ = 0;
    BIOS_RxEr.bios_rx_err_id_ = 0;
    BIOS_RxEr.bios_rx_err_payload_ = NULL;
    BIOS_RxEr.bios_rx_err_set_ = 0;
}

BIOS_RxError_t BIOS_getRxError()
{
    return BIOS_RxEr;
}
//********************************************
void BIOS_UsbRxError(unsigned short code)
{
    BIOS_UsbRxClear();
    BIOS_RxEr.bios_rx_err_code_ = code;
    BIOS_RxEr.bios_rx_err_id_ = 0;
    BIOS_RxEr.bios_rx_err_payload_ = NULL;
    BIOS_RxEr.bios_rx_err_set_ = 1;
}
//************************************************************
void BIOS_usbTxData (void)
{
    unsigned char first_cannel;
    // some thing to send in active send buffer?
    first_cannel = bios_tx_record_.cannel_to_send;
    while(!(bios_tx_record_.state[bios_tx_record_.cannel_to_send] & BIOS_TX_TO_SEND))
    {
      
        bios_tx_record_.cannel_to_send++;
        if(bios_tx_record_.cannel_to_send >= BIOS_TX_QUEUS)
        {
            bios_tx_record_.cannel_to_send = 0;
        }
      
        if(bios_tx_record_.cannel_to_send == first_cannel)
        {
            return;
        }
    }
    // test on chars in software TX buffer
    DIAG_SUPPRESS(Pa082)
    if(bios_tx_record_.send_counter[bios_tx_record_.cannel_to_send] < bios_tx_record_.count[bios_tx_record_.cannel_to_send])
    {
        WORD x;

        // send char from software buffer
        if(cdcSendDataInBackground((BYTE*)&bios_tx_record_.data[bios_tx_record_.cannel_to_send][0],bios_tx_record_.count[bios_tx_record_.cannel_to_send],0,10)) //Send the response over USB
        {
            USBCDC_abortSend(&x,0);                                         // It failed for some reason; abort and leave the main loop
        }
        // no char to send ***anymore*** in actual TX software buffer
        // we don't clear TX software buffer, reset only flags. If need we can
        // send the buffer again. (resending in not implemented)
        bios_tx_record_.state[bios_tx_record_.cannel_to_send] &= ~BIOS_TX_TO_SEND;
        bios_tx_record_.send_counter[bios_tx_record_.cannel_to_send] = 0;
        first_cannel = bios_tx_record_.cannel_to_send;
        if(!(bios_tx_record_.state[bios_tx_record_.cannel_to_send] & BIOS_TX_WAIT_ON_ACK))
        {
            bios_global_timer_[BIOS_TIMER_TX].count = 0;
        }
        // search in TX software buffer for data to send. If no data found, it was the
        // last TX buffer empty interrupt.
        do
        {
          
            bios_tx_record_.cannel_to_send++;
            if(bios_tx_record_.cannel_to_send >= BIOS_TX_QUEUS)
            {
                bios_tx_record_.cannel_to_send = 0;
            }
          
            if(bios_tx_record_.state[bios_tx_record_.cannel_to_send] & BIOS_TX_TO_SEND)
            {
                if(bios_tx_record_.send_counter[bios_tx_record_.cannel_to_send] <= ((unsigned short)bios_tx_record_.data[bios_tx_record_.cannel_to_send][0])) // add left side +1, if crc active
                {
                    bios_global_timer_[BIOS_TIMER_TX].count = BIOS_TX_TIMEOUT;
                }
            }
        }
        while(!(bios_tx_record_.state[bios_tx_record_.cannel_to_send] & BIOS_TX_TO_SEND) && (bios_tx_record_.cannel_to_send != first_cannel));
    }
    DIAG_DEFAULT(Pa082)
}

short BIOS_UsbRxData()
{
        BYTE count = 0;
        BYTE usbDataPointer = 0;
        BYTE sizeToCopy = 0;
        short returnError = 0;

        BIOS_SetCts();
        count = USBCDC_bytesInUSBBuffer(DEBUG_CHANNEL);
        if(count)
        {
            #pragma data_alignment=4
            static BYTE recivedData[260] = {0};
            
            USBCDC_receiveData(recivedData, count , DEBUG_CHANNEL);
            
            while(count)
            {
                // get char from hardwarebuffer
                bios_rx_char_ = recivedData[usbDataPointer];
                // test for xon/xoff flow control char

                // test for free buffer
                if(bios_rx_record_.state[bios_rx_record_.active] & BIOS_RX_RDY)
                {
                    BIOS_UsbRxError(EXCEPTION_RX_NO_BUFFER);
                    returnError = EXCEPTION_RX_NO_BUFFER;
                    goto usbRxIsrExit;
                }
                // test for first char in a message
                if(!bios_rx_record_.count[bios_rx_record_.active])
                {
                    // a message must received in 100ms completely
                    // if not a timeout error generate by timerB1Isr
                    bios_global_timer_[0].count = 50; // start timeout timer 500ms
                    bios_rx_record_.crc[1] = 0;
                    // length of messages must be even
                    if(bios_rx_char_ & 0x01)
                    {
                        bios_rx_record_.size[bios_rx_record_.active] = bios_rx_char_ + 0;
                    }
                    else
                    {
                        bios_rx_record_.size[bios_rx_record_.active] = bios_rx_char_ + 1;
                    }
                }
                // test for buffer size
                if(bios_rx_record_.size[bios_rx_record_.active] < (BIOS_RX_SIZE + 2))
                {
                    DIAG_SUPPRESS(Pa082)
                    sizeToCopy = bios_rx_record_.size[bios_rx_record_.active] + 1 - bios_rx_record_.count[bios_rx_record_.active];
                    if(sizeToCopy > count)
                    {
                      sizeToCopy = count;
                    }
                    BYTE numWords = sizeToCopy/sizeof(unsigned int);
                    unsigned int *pDest = (unsigned int*) &bios_rx_record_.data[bios_rx_record_.active][bios_rx_record_.count[bios_rx_record_.active]];
                    unsigned int *pSrc = (unsigned int*) &recivedData[usbDataPointer];
                    for(unsigned char i = 0; i < numWords; ++i)
                    {
                      *pDest++ = *pSrc++;
                    }
                    for(unsigned char i = numWords*sizeof(unsigned int); i < sizeToCopy; ++i)
                    {
                      bios_rx_record_.data[bios_rx_record_.active][bios_rx_record_.count[bios_rx_record_.active]+i] = recivedData[usbDataPointer+i];
                    }
                    bios_rx_record_.count[bios_rx_record_.active] += sizeToCopy;
                    usbDataPointer += sizeToCopy;
                    count -= sizeToCopy;
                    DIAG_DEFAULT(Pa082)
                }
                else
                {
                    BIOS_UsbRxError(EXCEPTION_RX_TO_SMALL_BUFFER);
                    returnError = EXCEPTION_RX_TO_SMALL_BUFFER;
                    goto usbRxIsrExit;
                }

                // test for completly message
                DIAG_SUPPRESS(Pa082)
                if(bios_rx_record_.count[bios_rx_record_.active] > (bios_rx_record_.size[bios_rx_record_.active]))
                DIAG_DEFAULT(Pa082)
                {
                    bios_global_timer_[0].count = 0;  // stop timeout timer

                    if(bios_rx_record_.data[bios_rx_record_.active][MESSAGE_CMDTYP_POS] == RESPTYP_ACKNOWLEDGE)
                    {
                        // search for message which are waiting on a ack
                        for(unsigned char i = 0; i < BIOS_TX_QUEUS; i++)
                        {
                            DIAG_SUPPRESS(Pa082)
                            // waits a transmited message for a ACK?
                            if((bios_tx_record_.state[i] & BIOS_TX_WAIT_ON_ACK) &&
                                ((bios_tx_record_.data[i][MESSAGE_MSG_ID_POS] & 0x3F) == bios_rx_record_.data[bios_rx_record_.active][MESSAGE_MSG_ID_POS]))
                            {
                                // if the transmited message are a member of a multi package message, the ack provide the package number
                                if( bios_tx_record_.data[i][3] || (bios_rx_record_.data[bios_rx_record_.active][0] > 3) )
                                {
                                    // package number are in byte 4
                                    if(bios_tx_record_.data[i][3] == bios_rx_record_.data[bios_rx_record_.active][4])
                                    DIAG_DEFAULT(Pa082)
                                    {
                                        bios_tx_record_.state[i] &= ~(BIOS_TX_WAIT_ON_ACK | BIOS_TX_TO_SEND | BIOS_TX_NO_SEND);
                                        bios_global_timer_[BIOS_TIMER_TX].count = 0;
                                        break;
                                    }
                                }
                                else
                                {
                                    bios_tx_record_.state[i] &= ~(BIOS_TX_WAIT_ON_ACK | BIOS_TX_TO_SEND | BIOS_TX_NO_SEND);
                                    break;
                                }
                            }
                        }
                    }
                    else if(bios_rx_record_.data[bios_rx_record_.active][MESSAGE_CMDTYP_POS] ==  RESPTYP_EXCEPTION)
                    {
                        // exception with message ID = 0 always reset the communication
                        // also all user macros in the loop are terminated
                        if(bios_rx_record_.data[bios_rx_record_.active][MESSAGE_MSG_ID_POS] == 0)
                        {
                            BIOS_LedOn(BIOS_LED_POWER);
                            BIOS_LedOff(BIOS_LED_MODE);
                            bios_wb_control_ = 0;
                            returnError = RESPTYP_EXCEPTION;

                        }
                        else
                        {
                            // search for message which are waiting on a ack
                            for(unsigned char i = 0; i < BIOS_TX_QUEUS; i++)
                            {
                                DIAG_SUPPRESS(Pa082)
                                if(((bios_tx_record_.state[i] & (BIOS_TX_WAIT_ON_ACK | BIOS_TX_TO_SEND)) == (BIOS_TX_WAIT_ON_ACK | BIOS_TX_TO_SEND)) &&
                                    (bios_tx_record_.data[i][MESSAGE_MSG_ID_POS] == bios_rx_record_.data[bios_rx_record_.active][MESSAGE_MSG_ID_POS]) &&
                                    (bios_tx_record_.data[i][3] == bios_rx_record_.data[bios_rx_record_.active][4]))
                                DIAG_DEFAULT(Pa082)
                                {
                                    bios_tx_record_.state[i] &= ~BIOS_TX_WAIT_ON_ACK;
                                    break;
                                }
                            }
                        }
                    }
                    else if((bios_rx_record_.data[bios_rx_record_.active][MESSAGE_MSG_ID_POS] == 0) && (bios_rx_record_.data[bios_rx_record_.active][MESSAGE_CMDTYP_POS] ==CMDTYP_COMRESET))
                    {
                        // clear RX cannel
                        BIOS_UsbRxClear();
                        returnError = CMDTYP_COMRESET;
                        goto usbRxIsrExit;
                    }
                    else
                    {
                        // mark message as ready
                        // BIOS_MainLoop pass the message to v3opRx for macro execution
                        DIAG_SUPPRESS(Pa082)
                        bios_rx_record_.state[bios_rx_record_.active] |= BIOS_RX_RDY;
                        DIAG_DEFAULT(Pa082)
                        // switch to the next Rx buffer
                          
                        bios_rx_record_.active++;
                        if(bios_rx_record_.active >= BIOS_RX_QUEUS)
                        {
                            bios_rx_record_.active = 0;
                        }
                    }
                // no size error
                bios_rx_record_.count[bios_rx_record_.active] = 0;
                bios_rx_record_.crc[1] = 0;
                }
                usbRxIsrExit:
                _NOP();
            }
        }
        BIOS_ResetCts(); // release TUSB RX
        return returnError;
}

// MSP-FET specifc bios ISR
#ifdef MSP_FET

// Dummy ISR
#pragma optimize = low
#pragma vector=TIMER2_A1_VECTOR
__interrupt void TIMER2_A1_ISR(void)
{
   switch (__even_in_range(TA2IV, TA2IV_TA2IFG))
   {
     case TA2IV_TA2IFG:
            _NOP();
            break;
     default:
            _NOP();
            break;
   }
}
#pragma vector=TIMER0_B0_VECTOR
__interrupt void TIMER0_B0_ISR_(void)
{
    _NOP();
}
#endif

// Dummy ISR
#pragma optimize = low
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void)
{
    _NOP();
}
/****************************************
 * \brief Interrupt function for NMI events
*/
#pragma vector = UNMI_VECTOR
__interrupt VOID UNMI_ISR(VOID)
{
    _DINT_FET();
    switch (__even_in_range(SYSUNIV, SYSUNIV_BUSIFG))
    {
    case SYSUNIV_NONE:
      __no_operation();
      break;
    case SYSUNIV_NMIIFG:
      __no_operation();
      break;
    case SYSUNIV_OFIFG:
        UCSCTL7 &= ~(DCOFFG + XT1LFOFFG + XT2OFFG);         //Clear OSC flaut Flags fault flags
        SFRIFG1 &= ~OFIFG;                                  //Clear OFIFG fault flag
        BIOS_LedOn(BIOS_LED_POWER);
        BIOS_LedOn(BIOS_LED_MODE);
        break;
    case SYSUNIV_ACCVIFG:
      __no_operation();
      break;
    case SYSUNIV_BUSIFG:
      SYSBERRIV = 0;            // clear bus error flag
      USB_disable();            // Disable
      BIOS_LedOn(BIOS_LED_POWER);
      BIOS_LedOn(BIOS_LED_MODE);
      break;
    }
}

#pragma vector=SYSNMI_VECTOR
__interrupt void SysNmiHandler(void)
{
    switch (__even_in_range(SYSUNIV, SYSUNIV_BUSIFG))
    {
    case SYSUNIV_NONE:
        __no_operation();
        break;
    case SYSUNIV_NMIIFG:
        __no_operation();
        break;
    case SYSUNIV_OFIFG:
        UCSCTL7 &= ~(DCOFFG + XT1LFOFFG + XT2OFFG);         //Clear OSC flaut Flags fault flags
        SFRIFG1 &= ~OFIFG;                                  //Clear OFIFG fault flag
        BIOS_LedOn(BIOS_LED_POWER);
        BIOS_LedOn(BIOS_LED_MODE);
        break;
    case SYSUNIV_ACCVIFG:
        __no_operation();
        break;
    case SYSUNIV_BUSIFG:                                      //USB is required.
        SYSBERRIV = 0;                                      //clear bus error flag
        USB_disable();                                      //Disable
        BIOS_LedOn(BIOS_LED_POWER);
        BIOS_LedOn(BIOS_LED_MODE);
        break;
    }
}
