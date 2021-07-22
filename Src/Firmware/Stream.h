#pragma once
#include "Types.h"
#include "BIOS.h"

#define HAL_STREAM

#define MESSAGE_NEW_MSG 0x0001
#define MESSAGE_LAST_MSG 0x0002
#define MESSAGE_OUT_TO_DLL (uint8_t*)0x0001
#define MESSAGE_NO_OUT (uint8_t*)0x0000

#define ID_SHARED_MEMORY_TYPE_SYNC_RUN 0xAA01
#define ID_SHARED_MEMORY_TYPE_TIMER_TICK 0xAA02
#define ID_SHARED_MEMORY_TYPE_I_TICK 0xAA03
#define ID_SHARED_MEMORY_TYPE_TIME_TICK 0xAA04

struct _StreamSafe_
{
    uint8_t rx[12];
    uint16_t *ext_data;
    uint16_t ext_size;
    uint16_t ext_counter;
};
typedef struct _StreamSafe_ StreamSafe;

struct stream_funcs
{
    int16_t (MSPProbeSim::*out_init)(uint8_t, uint8_t);
    int16_t (MSPProbeSim::*flush)(void);
    int16_t (MSPProbeSim::*put_byte)(uint8_t);
    int16_t (MSPProbeSim::*put_bytes)(void*, uint16_t);
    int16_t (MSPProbeSim::*put_word)(uint16_t);
    int16_t (MSPProbeSim::*put_long)(uint32_t);
    int16_t (MSPProbeSim::*in_init)(void *, uint8_t);
    int16_t (MSPProbeSim::*internal_stream)(uint8_t *, uint16_t, uint8_t *, uint16_t, StreamSafe *);
    int16_t (MSPProbeSim::*external_stream)(StreamSafe *);
    uint8_t (MSPProbeSim::*change_type)(uint8_t);
    int16_t (MSPProbeSim::*get_byte)(uint8_t *);
    int16_t (MSPProbeSim::*get_word)(uint16_t *);
    int16_t (MSPProbeSim::*get_long)(uint32_t *);
    int16_t (MSPProbeSim::*get_buffer)(void **, uint16_t *);
    int16_t (MSPProbeSim::*discard_bytes)(uint16_t);
    int16_t (MSPProbeSim::*memcpy)(uint8_t*, uint8_t*, uint16_t);
    int16_t (MSPProbeSim::*biosLedOn)(uint8_t);
    int16_t (MSPProbeSim::*biosLedOff)(uint8_t);
    int16_t (MSPProbeSim::*biosLedBlink)(uint8_t, uint16_t);
    int16_t (MSPProbeSim::*biosLedFlash)(uint8_t, uint16_t);
    int16_t (MSPProbeSim::*biosLedAlternate)(uint16_t);
    int16_t (MSPProbeSim::*sendDebug)(int8_t *, uint16_t size);
    int16_t (MSPProbeSim::*getSharedVariable)(uint16_t IdMemoryType, uint16_t** Address);
    int16_t (MSPProbeSim::*deleteSharedVariable)(uint16_t IdMemoryType);
};

void STREAM_resetSharedVariables();

#ifdef HAL_STREAM

//extern struct stream_funcs * _stream_Funcs;

// add definitions for external use
// functions are located in core module

/**
 * STREAM Function Pointer - these are the STREAM Exports
 */
#define STREAM_out_init             (CALL_MEMBER_FN_PTR(_stream_Funcs->out_init))
#define STREAM_flush                (CALL_MEMBER_FN_PTR(_stream_Funcs->flush))
#define STREAM_put_byte             (CALL_MEMBER_FN_PTR(_stream_Funcs->put_byte))
#define STREAM_put_bytes            (CALL_MEMBER_FN_PTR(_stream_Funcs->put_bytes))
#define STREAM_put_word             (CALL_MEMBER_FN_PTR(_stream_Funcs->put_word))
#define STREAM_put_long             (CALL_MEMBER_FN_PTR(_stream_Funcs->put_long))
#define STREAM_get_buffer           (CALL_MEMBER_FN_PTR(_stream_Funcs->get_buffer))
#define STREAM_in_init              (CALL_MEMBER_FN_PTR(_stream_Funcs->in_init))
#define STREAM_internal_stream      (CALL_MEMBER_FN_PTR(_stream_Funcs->internal_stream))
#define STREAM_external_stream      (CALL_MEMBER_FN_PTR(_stream_Funcs->external_stream))
#define STREAM_out_change_type      (CALL_MEMBER_FN_PTR(_stream_Funcs->change_type))
#define STREAM_get_byte             (CALL_MEMBER_FN_PTR(_stream_Funcs->get_byte))
#define STREAM_get_word             (CALL_MEMBER_FN_PTR(_stream_Funcs->get_word))
#define STREAM_get_long             (CALL_MEMBER_FN_PTR(_stream_Funcs->get_long))
#define STREAM_discard_bytes        (CALL_MEMBER_FN_PTR(_stream_Funcs->discard_bytes))
#define STREAM_memcpy               (CALL_MEMBER_FN_PTR(_stream_Funcs->memcpy))
#define STREAM_biosLedOn            (CALL_MEMBER_FN_PTR(_stream_Funcs->biosLedOn))
#define STREAM_biosLedOff           (CALL_MEMBER_FN_PTR(_stream_Funcs->biosLedOff))
#define STREAM_biosLedBlink         (CALL_MEMBER_FN_PTR(_stream_Funcs->biosLedBlink))
#define STREAM_biosLedFlash         (CALL_MEMBER_FN_PTR(_stream_Funcs->biosLedFlash))
#define STREAM_biosLedAlternate     (CALL_MEMBER_FN_PTR(_stream_Funcs->biosLedAlternate))
#define STREAM_getSharedVariable    (CALL_MEMBER_FN_PTR(_stream_Funcs->getSharedVariable))
#define STREAM_deleteSharedVariable (CALL_MEMBER_FN_PTR(_stream_Funcs->deleteSharedVariable))



/**** Debug & Trace Functionality ****/
#define STREAM_sendDebug        (*_stream_Funcs->sendDebug)
/**** Debug & Trace Functionality ****/

#else // HAL_STREAM
//extern struct stream_funcs _stream_Funcs;
/**
 * STREAM Function Pointer - these are the STREAM Exports
 */
#define STREAM_out_init             (CALL_MEMBER_FN_PTR(_stream_Funcs.out_init))
#define STREAM_flush                (CALL_MEMBER_FN_PTR(_stream_Funcs.flush))
#define STREAM_put_byte             (CALL_MEMBER_FN_PTR(_stream_Funcs.put_byte))
#define STREAM_put_bytes            (CALL_MEMBER_FN_PTR(_stream_Funcs.put_bytes))
#define STREAM_put_word             (CALL_MEMBER_FN_PTR(_stream_Funcs.put_word))
#define STREAM_put_long             (CALL_MEMBER_FN_PTR(_stream_Funcs.put_long))
#define STREAM_in_init              (CALL_MEMBER_FN_PTR(_stream_Funcs.in_init))
#define STREAM_internal_stream      (CALL_MEMBER_FN_PTR(_stream_Funcs.internal_stream))
#define STREAM_external_stream      (CALL_MEMBER_FN_PTR(_stream_Funcs.external_stream))
#define STREAM_out_change_type      (CALL_MEMBER_FN_PTR(_stream_Funcs.change_type))
#define STREAM_get_byte             (CALL_MEMBER_FN_PTR(_stream_Funcs.get_byte))
#define STREAM_get_word             (CALL_MEMBER_FN_PTR(_stream_Funcs.get_word))
#define STREAM_get_long             (CALL_MEMBER_FN_PTR(_stream_Funcs.get_long))
#define STREAM_get_buffer           (CALL_MEMBER_FN_PTR(_stream_Funcs->get_buffer))
#define STREAM_discard_bytes        (CALL_MEMBER_FN_PTR(_stream_Funcs.discard_bytes))
#define STREAM_memcpy               (CALL_MEMBER_FN_PTR(_stream_Funcs.memcpy))
#define STREAM_biosLedOn            (CALL_MEMBER_FN_PTR(_stream_Funcs.biosLedOn))
#define STREAM_biosLedOff           (CALL_MEMBER_FN_PTR(_stream_Funcs.biosLedOff))
#define STREAM_biosLedBlink         (CALL_MEMBER_FN_PTR(_stream_Funcs.biosLedBlink))
#define STREAM_biosLedFlash         (CALL_MEMBER_FN_PTR(_stream_Funcs.biosLedFlash))
#define STREAM_biosLedAlternate     (CALL_MEMBER_FN_PTR(_stream_Funcs.biosLedAlternate))
#define STREAM_sendDebug            (CALL_MEMBER_FN_PTR(_stream_Funcs.sendDebug))
#define STREAM_getSharedVariable    (CALL_MEMBER_FN_PTR(_stream_Funcs.getSharedVariable))
#define STREAM_deleteSharedVariable (CALL_MEMBER_FN_PTR(_stream_Funcs.deleteSharedVariable))

#define STREAM_CORE_ZERO_VERSION    0x00
#define STREAM_CORE_ZERO_MACRO_SIZE 0x01
#define STREAM_CORE_ZERO_MACRO_ADDR 0x02
#define STREAM_CORE_ZERO_PUC_RESET  0x03
// function numbers in "zero function(s) in HAL modul" must start at 0xFF and count downward
#endif // HAL_STREAM

typedef int16_t (MSPProbeSim::*FuncInOut)  (uint16_t id);

// MSPProbeSim: Moved to Types.h
//#ifndef HAL_REC
//#define HAL_REC
//struct _HalRec_
//{
//    uint16_t id;
//    void  *function;
//};
//typedef struct _HalRec_ HalRec;
//#endif

#define DEVICE_FLAG_XONOFF  0x00000001
#define DEVICE_FLAG_SBW4    0x00000002
#define DEVICE_FLAG_SBW2    0x00000004
#define DEVICE_FLAG_EASY    0x00000008

#define DEVICE_FLAG_MSPFET  0x00000010
#define DEVICE_FLAG_EZFET   0x00000020
