#pragma once
#include "Types.h"
#include "BIOS.h"
#include "V3OP.h"

#define kUSBCDC_sendStarted         0x01
#define kUSBCDC_sendComplete        0x02
#define kUSBCDC_intfBusyError       0x03
#define kUSBCDC_receiveStarted      0x04
#define kUSBCDC_receiveCompleted    0x05
#define kUSBCDC_receiveInProgress   0x06
#define kUSBCDC_generalError        0x07
#define kUSBCDC_busNotAvailable     0x08

#define kUSBCDC_waitingForSend      0x01
#define kUSBCDC_waitingForReceive   0x02
#define kUSBCDC_dataWaiting         0x04
#define kUSBCDC_busNotAvailable     0x08
#define kUSB_allCdcEvents           0xFF

using CHAR      = int8_t;
using UCHAR     = uint8_t;
using INT       = int16_t;
using UINT      = uint16_t;
using SHORT     = int16_t;
using USHORT    = uint16_t;
using LONG      = int32_t;
using ULONG     = uint32_t;
using VOID      = void;
using HANDLE    = uint32_t;
using PSTR      = int8_t*;
using BOOL      = int16_t;
using DOUBLE    = double;
using BYTE      = uint8_t;
using PBYTE     = uint8_t*;
using WORD      = uint16_t;
using DWORD     = uint32_t;
using PDWORD    = uint32_t*;

BYTE USBCDC_intfStatus (BYTE intfNum, WORD* bytesSent, WORD* bytesReceived) {
    return 0;
}

BYTE USBCDC_abortSend (WORD* size, BYTE intfNum) {
    // Unimplemented
    abort();
    return 0;
}

BYTE cdcSendDataInBackground (BYTE* dataBuf, WORD size, BYTE intfNum, ULONG ulTimeout) {
    ULONG sendCounter = 0;
    WORD bytesSent, bytesReceived;

    while (USBCDC_intfStatus(intfNum,&bytesSent, &bytesReceived) & kUSBCDC_waitingForSend)
    {
        if (ulTimeout && ((sendCounter++) > ulTimeout))
        {    /* A send operation is underway; incr counter & try again */
            return ( 1) ;                                   /* Timed out                */
        }
    }

    /* The interface is now clear.  Call sendData().   */
    switch (USBCDC_sendData(dataBuf,size,intfNum))
    {
        case kUSBCDC_sendStarted:
            return ( 0) ;
        case kUSBCDC_busNotAvailable:
            return ( 2) ;
        default:
            return ( 4) ;
    }
}

uint8_t USBCDC_bytesInUSBBuffer(BYTE intfNum) {
    assert(!_state.msgs.queue.empty());
    const Msg& msg = *_state.msgs.queue.front();
    assert(msg.len <= std::numeric_limits<uint8_t>::max());
    return msg.len;
}

BYTE USBCDC_sendData (const BYTE* data, WORD size, BYTE intfNum) {
    printf("[FW] SENDING DATA LEN: %zu\n", (size_t)size);
    
    MsgPtr respPtr = std::make_unique<Msg>();
    Msg& resp = *respPtr;
    
    assert(size <= sizeof(resp.data));
    memcpy(resp.data, data, size);
    resp.len = size;
    
    // Add the message to the queue
    _state.resps.queue.push_back(std::move(respPtr));
    _state.resps.signal.notify_all();
    
    return (kUSBCDC_sendStarted);
}

BYTE USBCDC_receiveData(BYTE* data, WORD size, BYTE intfNum) {
    const Msg& msg = *_state.msgs.queue.front();
    assert(size <= msg.len);
    memcpy(data, msg.data, size);
    _state.msgs.queue.pop_front();
    return kUSBCDC_receiveCompleted;
}

BYTE USBCDC_handleDataReceived (BYTE intfNum)
{
    if (intfNum == DEBUG_CHANNEL) //this is our debug channel
    {
        if(BIOS_UsbRxData() == RESPTYP_EXCEPTION )
        {
            V3OP_KillLoop(0);
            BIOS_UsbRxClear();
            BIOS_UsbTxClear();
            BIOS_InitCom();
         }
    }
    return (TRUE);
}
