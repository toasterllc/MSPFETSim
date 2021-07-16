/*
 * v3_0p_hw_nguif.h
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
//! \file v3_0p_hw_uif.h
//! \brief
#ifndef V3_0P_HW_FET_H_
#define V3_0P_HW_FET_H_

unsigned long V3OP_GetSegmentType(unsigned long addr);
unsigned char V3OP_pWriteAllowed(unsigned short addr);
unsigned char V3OP_EraseAllowed(unsigned short addr);

short V3OP_CoreFlashFunctionErase(unsigned char *payload);
short V3OP_CoreFlashFunctionWrite(unsigned char *payload,unsigned short v30p_stream_flags_);
short V3OP_CoreFlashFunctionRead(unsigned char *payload);
void V3OP_UpCore(void);

unsigned short V3OP_GetHilCrc();
unsigned short V3OP_GetHalCrc();
unsigned short V3OP_GetCoreCrc();
unsigned short V3OP_GetDcdcCrc();
unsigned short V3OP_GetComChannelCrc();
unsigned short V3OP_GetHalFpgaCrc();

unsigned char V3OP_HilCrcOk();
unsigned char V3OP_HalCrcOk();
unsigned char V3OP_CoreCrcOk();
unsigned char V3OP_DcdcCrcOk();
unsigned char V3OP_ComChannelCrcOk();
unsigned char V3OP_HalFpgaCrcOk();

unsigned short V3OP_SystemOk(void);

#endif /* V3_0P_HW_FET_H_ */
