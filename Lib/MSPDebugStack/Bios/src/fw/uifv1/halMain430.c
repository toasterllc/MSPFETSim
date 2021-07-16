/*
 * halMain430.c
 *
 * <FILE_BRIEF>
 *
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
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
#include "stream.h"
#include "hal.h"
#include "edt.h"
#include "stdlib.h"
#include "string.h"
#include "HalGlobalVars.h"
#include "../fw/uifv1/uifVersion.h"

extern void globalVarsInit(void);

extern short _hil_Init( void );
extern unsigned long _hal_mclkCntrl0;

// prototypes for core/HAL interface
void *ResetFirmware(void *stream_adr, unsigned long device_flags);

CONST_AT(HAL_INFOS hal_infos_, HAL_ADDR_CONST_HAL_INFOS) =
{
  ResetFirmware,          // _initTask
  (VERSION_MAJOR - 1) << 14 |
  (VERSION_MINOR << 8) |
  VERSION_PATCH,
  VERSION_BUILD
};
REQUIRED(hal_infos_)

//HAL signature, indicates HAL part is valid
const unsigned short hal_signature @ "HALSIG" = 0x5137;
REQUIRED(hal_signature)

struct stream_funcs *_stream_Funcs;
HAL_INFOS hal_infos_in_ram_;
unsigned short hal_delay_loop_us_;
unsigned short hal_delay_loop_ms_;

// called via cstartup form biosHalInterfaceInit
REQUIRED(ResetFirmware)
void *ResetFirmware(void *stream_adr, unsigned long device_flags)
{
    globalVarsInit();

    hal_delay_loop_us_ = 1;
    hal_delay_loop_ms_ = 0x7CF;

    memcpy((HAL_INFOS*)&hal_infos_in_ram_,&hal_infos_, sizeof(hal_infos_));
    _stream_Funcs=(struct stream_funcs *)stream_adr;
    _init_Hal();
    if((hal_functions_[sizeof(hal_functions_)/sizeof(HalRec)-1].function == NULL))
    {
        hal_functions_[sizeof(hal_functions_)/sizeof(HalRec)-1].id = 0xFFFE;
        hal_functions_[sizeof(hal_functions_)/sizeof(HalRec)-1].function = (void*)_edt_Common_Methods.Loop;
    }
    _hil_Init();
    _hal_mclkCntrl0=0;

    hal_infos_in_ram_.hal_size = sizeof(hal_functions_)/sizeof(HalRec);
    hal_infos_in_ram_.hal_list_ptr = hal_functions_;
    hal_infos_in_ram_.arch =  T_ARCH_MSP430;

    return((void*)&hal_infos_in_ram_); // return software infos
}
