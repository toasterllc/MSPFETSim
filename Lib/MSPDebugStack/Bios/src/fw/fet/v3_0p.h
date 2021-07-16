/*
 * v3_0p.h
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

//! \ingroup MODULBIOS
//! \file v3_0p.h
//! \brief

#ifndef _V3_0P_HEADER_
#define _V3_0P_HEADER_

#include "stream.h"

#define V3OP_LOOP_WAIT_FLAG   0x01
#define V3OP_LOOP_ARRAY_COUNT 4

struct _V3opLoopArray_
{
  unsigned short addr;
  unsigned char  *indata;
  unsigned char  flags;
  unsigned char  msg_id;
  unsigned char  msg_type;
  unsigned char  active;
};
typedef struct _V3opLoopArray_ V3opLoopArray;
typedef HalRec (*HAL_REC_ARRAY)[];

#define HAL_FUNCTION_PTR(x) short (*x)(unsigned short)
typedef HAL_FUNCTION_PTR(HalFuncInOut);

// functions
short V3OP_Rx(unsigned char *str);
short V3OP_SendException(unsigned char msg_id, unsigned short code, unsigned short *payload);
short V3OP_SetLoop(unsigned char *payload_incl_addr, unsigned char flags);
short V3OP_KillLoop(unsigned char msg_id);
short V3OP_PauseLoop(unsigned char msg_id);
short V3OP_ResumeLoop(unsigned char msg_id);
void  V3OP_KillAllLoops(void);

void V3OP_Scheduler(void);

short V3OP_HalInterfaceClear(void);
short V3OP_HalInterfaceInit(void);

void V3OP_DcdcInterfaceClear(void);
short V3OP_DcdcInterfaceInit(void);

short V3OP_ComInterfaceInit(void);
void V3OP_ComInterfaceClear(void);

short V3OP_HalFpgaUpdate(void);
void V3OP_HwReset(void);

short V3OP_CoreFlashFunctionInit(unsigned char *payload);

#endif // _V3_0P_HEADER_
