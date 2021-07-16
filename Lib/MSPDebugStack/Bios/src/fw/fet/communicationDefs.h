/*
 * communicationDefs.h
 *
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef communicationDefs_H_
#define communicationDefs_H_

#define MESSAGE_SIZE_POS              0
#define MESSAGE_CMDTYP_POS            1
#define MESSAGE_MSG_ID_POS            2
#define MESSAGE_INSIZE_POS            3
#define MESSAGE_PAYLOAD_POS           4
#define MESSAGE_EXECUTE_CALL_ADDR_POS 4
#define MESSAGE_EXECUTE_ZERO_ADDR_POS 6
#define MESSAGE_EXECUTE_PAYLOAD_POS   6

#define MESSAGE_NO_RESPONSE           0x8000
#define EXCEPTION_NOT_IMPLEMENT_ERR   0x8001
#define EXCEPTION_MSGID_ERR           0x8002
#define EXCEPTION_CRC_ERR             0x8003
#define EXCEPTION_RX_TIMEOUT_ERR      0x8004
#define EXCEPTION_TX_TIMEOUT_ERR      0x8005
#define EXCEPTION_RX_OVERFLOW_ERR     0x8006
#define EXCEPTION_TX_NO_BUFFER        0x8007
#define EXCEPTION_COM_RESET           0x8008
#define EXCEPTION_RX_NO_BUFFER        0x8009
#define EXCEPTION_RX_TO_SMALL_BUFFER  0x800A
#define EXCEPTION_RX_LENGTH           0x800B

#endif /* communicationDefs_H_ */
