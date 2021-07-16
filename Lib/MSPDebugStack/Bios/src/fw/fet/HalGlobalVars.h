/*
 * HalGlobalVars.h
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

//! \ingroup MODULHAL
//! \file HalGlobalVars.h
//! \brief
//!
//! \author  Detlef Fink (03/10/2011)

// var addresses
// unsigned char

//#define HAL_ADDR_VAR_MCLK_MODULES     0x2402
// union (char) 32 Bytes
//#define HAL_ADDR_VAR_CHAIN_CONFIGURATION 0x2424
// unsigned short
//#define HAL_ADDR_VAR_ACTIVE_DEVICE 0x2444
// unsigned char
//#define HAL_ADDR_VAR_NUM_OF_DEVICES 0x2446
// unsigned short
//#define HAL_ADDR_VAR_TCE 0x2448
// unsigned short
//#define HALL_ADDR_VAR_HAL_MCLK_CNTRL0 0x244C


// 512
//#define HAL_ADDR_VAR_HAL_FUNCTION 0x00244Eul
// *to_struc
//#define HAL_ADDR_VAR_STREAM_FUNCS 0x0025FEul
// 18 bytes
//#define HAL_ADDR_CONST_ETD_COMMON_METHODS 0x002602ul
// 12 bytes
//#define HAL_ADDR_VAR_EDT_DISTINCT_METHODS 0x002652ul
// struct (10 Bytes)
//#define HAL_ADDR_VAR_HAL_INFOS_IN_RAM 0x00268Aul


// unsigned short
//#define HAL_ADDR_VAR_QPROTOCOL_ID 0x2698
// unsigned short
//#define HAL_ADDR_VAR_B_VCC_ON 0x269A
// unsigned char*
//#define HL_ADDR_JTAG_PORT_OUT 0x269C
// unsigned long
//#define HAL_ADDR_VAR_DEVICE_FLAGS 0x269E
// unsigned char *
//#define HAL_ADDR_TSTCTRL_PORT 0x26A0
// unsigned short
//#define HAL_ADDR_VAR_DELAY_LOOP_US 0x26A2
// unsigned short
//#define HAL_ADDR_VAR_DELAY_LOOP_MS 0x26A4

//#define HAL_ADDR_VAR_DEVICE_POWER_SETTINGS 0x26A6ul

// const addresses
// short[5]
//#define HAL_ADDR_CONST_HAL_INFOS 0x0D020ul//0x0026CAul
// 512 bytes
//#define HAL_ADDR_CONST_HAL_FUNCTION_DEFAULTS 0x0E02Eul//0x0D02Eul//0x0026D8ul
