#pragma once
#include "Types.h"

#define BIOS_LED_MODE       0
#define BIOS_LED_POWER      1

#define BIOS_RX_RDY         0x01
#define BIOS_RX_CRC_ERROR   0x02
#define BIOS_RX_SIZE_ERROR  0x04
#define BIOS_TX_TO_SEND     0x01
#define BIOS_TX_WAIT_ON_ACK 0x02
#define BIOS_TX_NO_SEND     0x04
#define BIOS_TX_BUSY        0x08

#define BIOS_TX_TIMEOUT     50

#define BIOS_TIMER_BREAK    0x01
#define BIOS_TIMER_COUNT    2
#define BIOS_TIMER_RX       0
#define BIOS_TIMER_TX       1

#define BIOS_LED_COUNT 2

#define _NOP()

typedef struct _BiosRxRecord_
{   
    uint8_t  *data[BIOS_RX_QUEUS];
    uint16_t *datas[BIOS_RX_QUEUS];
    uint16_t count[BIOS_RX_QUEUS];
    uint16_t size[BIOS_RX_QUEUS];
    uint16_t last_msg_call;
    uint16_t crc[BIOS_RX_QUEUS];
    uint8_t  active;
    uint8_t  last_cmd_typ;
    uint8_t  last_msg_id;
    uint8_t  state[BIOS_RX_QUEUS];
} BiosRxRecord;

typedef struct _BiosTxRecord_
{
    uint8_t  active;
    uint8_t  cannel_to_send;
    uint16_t send_counter[BIOS_TX_QUEUS];
    uint16_t ack_timeout[BIOS_TX_QUEUS];
    uint8_t  state[BIOS_TX_QUEUS];
    uint16_t count[BIOS_TX_QUEUS];
    uint16_t datas[BIOS_TX_QUEUS][BIOS_TX_SIZE/sizeof(uint16_t)];
    uint8_t  *data[BIOS_TX_QUEUS];
    uint16_t  *ext_data;
    uint16_t ext_size;
    uint16_t ext_counter;
} BiosTxRecord;

//! \struct Global timer used for timeouts
struct _BiosGlobalTimer_
{
    uint16_t count;
    uint8_t state;
};
typedef struct _BiosGlobalTimer_ BiosGlobalTimer;

struct BIOS_RxError
{
    uint16_t bios_rx_err_code_;
    uint16_t *bios_rx_err_payload_;
    uint16_t bios_rx_err_id_;
    uint16_t bios_rx_err_set_;
};
typedef struct BIOS_RxError BIOS_RxError_t;

//! \brief values for (re)set LEDs
const uint8_t BIOS_LED_OFF   = 0x00;
//! \brief values for (re)set LEDs
const uint8_t BIOS_LED_ON    = 0x01;
//! \brief values for (re)set LEDs
const uint8_t BIOS_LED_BLINK = 0x02;

volatile BiosRxRecord bios_rx_record_ = {{0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, 0, {0,0,0,0}, 0, 0, 0, {0,0,0,0}};
volatile BiosTxRecord bios_tx_record_ = {0, 0, {0}, {0}, {0}, {0}, {0}, NULL, 0};
volatile BiosGlobalTimer bios_global_timer_[BIOS_TIMER_COUNT];

uint16_t tempInDataRx[4][BIOS_RX_SIZE]= {0x0,0x0};

BIOS_RxError_t BIOS_RxEr;

int8_t bios_wb_control_ = 0;
uint8_t bios_rx_char_ = 0;

void BIOS_SetCts() {
}

void BIOS_ResetCts() {
}

int16_t BIOS_LedOn(uint8_t no) {
    return 0;
}

int16_t BIOS_LedOff(uint8_t no) {
    return 0;
}

int16_t BIOS_LedBlink(uint8_t no,uint16_t time) {
    return 0;
}

int16_t BIOS_LedFlash(uint8_t no,uint16_t time) {
    return 0;
}

int16_t BIOS_LedAlternate(uint16_t time) {
    return 0;
}

void BIOS_GlobalError(void)
{
    BIOS_LedOn(BIOS_LED_MODE);
    BIOS_LedOff(BIOS_LED_POWER);
    while(1);
}

void BIOS_InitCom(void)
{
    uint8_t i;

    for(i = 0; i < BIOS_RX_QUEUS; i++)
    {
        bios_rx_record_.datas[i] = tempInDataRx[i];
        if(bios_rx_record_.datas[i] == NULL)
        {
            BIOS_GlobalError();
        }
        bios_rx_record_.data[i] = (uint8_t*)bios_rx_record_.datas[i];
    }
    for(i = 0; i < BIOS_TX_QUEUS; i++)
    {
        bios_tx_record_.data[i] = (uint8_t*)bios_tx_record_.datas[i];
    }
}

int16_t BIOS_IsUsbRxError()
{
    if(BIOS_RxEr.bios_rx_err_set_)
    {
        return TRUE;
    }
    return FALSE;
}

void BIOS_UsbTxError(void) {
    // Unimplemented
    abort();
    return;
}

void BIOS_UsbRxClear(void)
{
    uint8_t i;

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

void BIOS_UsbRxError(uint16_t code)
{
    BIOS_UsbRxClear();
    BIOS_RxEr.bios_rx_err_code_ = code;
    BIOS_RxEr.bios_rx_err_id_ = 0;
    BIOS_RxEr.bios_rx_err_payload_ = NULL;
    BIOS_RxEr.bios_rx_err_set_ = 1;
}

void BIOS_UsbTxClear(void)
{
    uint8_t i;

    memset((uint8_t*)&bios_tx_record_, 0, sizeof(bios_tx_record_));
    bios_global_timer_[BIOS_TIMER_TX].count = 0;
    bios_rx_char_ = 0;
    for(i = 0; i < BIOS_TX_QUEUS; i++)
    {
        bios_tx_record_.data[i] = (uint8_t*)bios_tx_record_.datas[i];
        bios_tx_record_.state[i] = 0;
    }
}

BIOS_RxError_t BIOS_getRxError()
{
    return BIOS_RxEr;
}

void BIOS_ClearUsbRxError()
{
    BIOS_RxEr.bios_rx_err_code_ = 0;
    BIOS_RxEr.bios_rx_err_id_ = 0;
    BIOS_RxEr.bios_rx_err_payload_ = NULL;
    BIOS_RxEr.bios_rx_err_set_ = 0;
}

void BIOS_PrepareTx(uint16_t size)
{
    bios_tx_record_.send_counter[bios_tx_record_.cannel_to_send] = 0;
    bios_tx_record_.count[bios_tx_record_.active] = size;
    bios_tx_record_.state[bios_tx_record_.active] |= BIOS_TX_TO_SEND;
}

//********************************************
void BIOS_StartTx(void)
{
    bios_global_timer_[BIOS_TIMER_TX].count = BIOS_TX_TIMEOUT;
    BIOS_usbTxData();
}

void BIOS_usbTxData (void)
{
    uint8_t first_cannel;
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
    if(bios_tx_record_.send_counter[bios_tx_record_.cannel_to_send] < bios_tx_record_.count[bios_tx_record_.cannel_to_send])
    {
        uint16_t x;

        // send char from software buffer
        if(cdcSendDataInBackground((uint8_t*)&bios_tx_record_.data[bios_tx_record_.cannel_to_send][0],bios_tx_record_.count[bios_tx_record_.cannel_to_send],0,10)) //Send the response over USB
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
                if(bios_tx_record_.send_counter[bios_tx_record_.cannel_to_send] <= ((uint16_t)bios_tx_record_.data[bios_tx_record_.cannel_to_send][0])) // add left side +1, if crc active
                {
                    bios_global_timer_[BIOS_TIMER_TX].count = BIOS_TX_TIMEOUT;
                }
            }
        }
        while(!(bios_tx_record_.state[bios_tx_record_.cannel_to_send] & BIOS_TX_TO_SEND) && (bios_tx_record_.cannel_to_send != first_cannel));
    }
}

int16_t BIOS_UsbRxData()
{
    uint8_t count = 0;
    uint8_t usbDataPointer = 0;
    uint8_t sizeToCopy = 0;
    int16_t returnError = 0;

    BIOS_SetCts();
    count = USBCDC_bytesInUSBBuffer(DEBUG_CHANNEL);
    if(count)
    {
        uint8_t recivedData[260] __attribute__((aligned(4))) = {0};
        
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
                sizeToCopy = bios_rx_record_.size[bios_rx_record_.active] + 1 - bios_rx_record_.count[bios_rx_record_.active];
                if(sizeToCopy > count)
                {
                  sizeToCopy = count;
                }
                uint8_t numWords = sizeToCopy/sizeof(uint16_t);
                uint16_t *pDest = (uint16_t*) &bios_rx_record_.data[bios_rx_record_.active][bios_rx_record_.count[bios_rx_record_.active]];
                uint16_t *pSrc = (uint16_t*) &recivedData[usbDataPointer];
                for(uint8_t i = 0; i < numWords; ++i)
                {
                  *pDest++ = *pSrc++;
                }
                for(uint8_t i = numWords*sizeof(uint16_t); i < sizeToCopy; ++i)
                {
                  bios_rx_record_.data[bios_rx_record_.active][bios_rx_record_.count[bios_rx_record_.active]+i] = recivedData[usbDataPointer+i];
                }
                bios_rx_record_.count[bios_rx_record_.active] += sizeToCopy;
                usbDataPointer += sizeToCopy;
                count -= sizeToCopy;
            }
            else
            {
                BIOS_UsbRxError(EXCEPTION_RX_TO_SMALL_BUFFER);
                returnError = EXCEPTION_RX_TO_SMALL_BUFFER;
                goto usbRxIsrExit;
            }

            // test for completly message
            if(bios_rx_record_.count[bios_rx_record_.active] > (bios_rx_record_.size[bios_rx_record_.active]))
            {
                bios_global_timer_[0].count = 0;  // stop timeout timer

                if(bios_rx_record_.data[bios_rx_record_.active][MESSAGE_CMDTYP_POS] == RESPTYP_ACKNOWLEDGE)
                {
                    // search for message which are waiting on a ack
                    for(uint8_t i = 0; i < BIOS_TX_QUEUS; i++)
                    {
                        // waits a transmited message for a ACK?
                        if((bios_tx_record_.state[i] & BIOS_TX_WAIT_ON_ACK) &&
                            ((bios_tx_record_.data[i][MESSAGE_MSG_ID_POS] & 0x3F) == bios_rx_record_.data[bios_rx_record_.active][MESSAGE_MSG_ID_POS]))
                        {
                            // if the transmited message are a member of a multi package message, the ack provide the package number
                            if( bios_tx_record_.data[i][3] || (bios_rx_record_.data[bios_rx_record_.active][0] > 3) )
                            {
                                // package number are in byte 4
                                if(bios_tx_record_.data[i][3] == bios_rx_record_.data[bios_rx_record_.active][4])
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
                        for(uint8_t i = 0; i < BIOS_TX_QUEUS; i++)
                        {
                            if(((bios_tx_record_.state[i] & (BIOS_TX_WAIT_ON_ACK | BIOS_TX_TO_SEND)) == (BIOS_TX_WAIT_ON_ACK | BIOS_TX_TO_SEND)) &&
                                (bios_tx_record_.data[i][MESSAGE_MSG_ID_POS] == bios_rx_record_.data[bios_rx_record_.active][MESSAGE_MSG_ID_POS]) &&
                                (bios_tx_record_.data[i][3] == bios_rx_record_.data[bios_rx_record_.active][4]))
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
                    bios_rx_record_.state[bios_rx_record_.active] |= BIOS_RX_RDY;
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
