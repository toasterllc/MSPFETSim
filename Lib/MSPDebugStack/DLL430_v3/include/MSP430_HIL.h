/*
 * MSP430_HIL.h
 *
 * API for low level HIL access.
 *
 * Copyright (c) 2004 - 2016 Texas Instruments Incorporated - http://www.ti.com/
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

/**
\file MSP430_HIL.h

 \brief       This file contains the DLL function headers and definitions
              for low level HIL access.

 \par         Project:
              MSP-FET430UIF (TI USB FET) HIL API

 \par         Developed using:
              MS Visual C++ 2003/2010

 \par         Supported API calls:
              - MSP430_HIL_Open()
			  - MSP430_HIL_Configure();
			  - MSP430_HIL_Connect()
			  -	MSP430_HIL_Bsl()
			  -	MSP430_HIL_Close()

			  -	MSP430_HIL_JTAG_IR()
			  -	MSP430_HIL_JTAG_DR()
			  -	MSP430_HIL_TST()
			  -	MSP430_HIL_TCK()
			  -	MSP430_HIL_TMS()
			  -	MSP430_HIL_TDI()
			  -	MSP430_HIL_RST()
			  -	MSP430_HIL_ResetJtagTap()
*/


#ifndef MSP430_HIL_H
#define MSP430_HIL_H

#include "MSP430.h"

#if defined(__cplusplus)
extern "C" {
#endif

/// Configurations values for CONFIG_MODE INTERFACE_MODE
typedef enum HIL_ENTRY_STATE {
	HIL_ENTRY_RSTHIGH = 0, /**< RST high during entry sequence*/
	HIL_ENTRY_RSTLOW = 1, /**< RST low during entry sequence*/
} IHIL_ENTRY_STATE_t;


// Functions. -----------------------------------------------------------------
DLL430_SYMBOL STATUS_T WINAPI MSP430_HIL_Open(void);
DLL430_SYMBOL STATUS_T WINAPI MSP430_HIL_Configure(int32_t mode, int32_t value);
DLL430_SYMBOL STATUS_T WINAPI MSP430_HIL_Connect();
DLL430_SYMBOL STATUS_T WINAPI MSP430_HIL_Connect_Entry_State(int32_t value);
DLL430_SYMBOL STATUS_T WINAPI MSP430_HIL_Bsl(void);
DLL430_SYMBOL STATUS_T WINAPI MSP430_HIL_Close(int32_t vccOff);

DLL430_SYMBOL int32_t WINAPI MSP430_HIL_JTAG_IR(int32_t instruction);
DLL430_SYMBOL int32_t WINAPI MSP430_HIL_JTAG_IR4(int32_t instruction);
DLL430_SYMBOL int32_t WINAPI MSP430_HIL_JTAG_DR(int32_t data, int32_t bits);
DLL430_SYMBOL int64_t WINAPI MSP430_HIL_JTAG_DRX(int64_t data, int32_t bits);
DLL430_SYMBOL int64_t WINAPI MSP430_HIL_JTAG_IR_DRX(int32_t instruction, int64_t data, int32_t bits);
DLL430_SYMBOL STATUS_T WINAPI MSP430_HIL_TST(int32_t state);
DLL430_SYMBOL STATUS_T WINAPI MSP430_HIL_TCK(int32_t state);
DLL430_SYMBOL STATUS_T WINAPI MSP430_HIL_TMS(int32_t state);
DLL430_SYMBOL STATUS_T WINAPI MSP430_HIL_TDI(int32_t state);
DLL430_SYMBOL STATUS_T WINAPI MSP430_HIL_RST(int32_t state);
DLL430_SYMBOL void WINAPI MSP430_HIL_ResetJtagTap(void);
DLL430_SYMBOL void WINAPI MSP430_HIL_FuseCheck(void);
DLL430_SYMBOL STATUS_T WINAPI MSP430_HIL_DPACC(uint8_t address, uint32_t *data, uint16_t RdnWr);
DLL430_SYMBOL STATUS_T WINAPI MSP430_HIL_APACC(uint8_t portNum, uint8_t address, uint32_t *data, uint16_t RdnWr);
DLL430_SYMBOL STATUS_T WINAPI MSP430_HIL_MEMAP(uint8_t portNum, uint32_t address, uint32_t *data, uint16_t RdnWr);
DLL430_SYMBOL STATUS_T WINAPI MSP430_HIL_TCLK_V3(uint8_t value);

#if defined(__cplusplus)
}
#endif

#endif // MSP430_HIL_H
