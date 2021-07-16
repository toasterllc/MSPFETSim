/*
 * Copyright (C) 2007 - 2014 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef _HIL_FPGA_ACCESS_H_
#define _HIL_FPGA_ACCESS_H_

#include "arch.h"

void hil_fpga_init(void);
void initJtagBypass(struct jtag);
void hil_fpga_enable_bypass();
void hil_fpga_disable_bypass(void);

void hil_fpga_write_cmd_data0(unsigned char cmd, unsigned char data0);
void hil_fpga_write_cmd_data0_data1(unsigned char cmd, unsigned char data0, unsigned short data1);
void hil_fpga_write_cmd_data0_data1_count(unsigned char cmd, unsigned char data0, unsigned short data1Buf[], unsigned char count);
void hil_fpga_read_data1(unsigned char count, unsigned short *buf);
void hil_fpga_power_up_target(void);

//432 only
void hil_fpga_write_cmd_data0_data1_count_432(unsigned char cmd, unsigned char data0, unsigned short data1Buf[], unsigned char count);
void hil_fpga_read_data_432(unsigned char count, unsigned short *buf);
void hil_fpga_write_cmd_data0_432(unsigned char cmd, unsigned char data0);

#endif /*_HIL_FPGA_ACCESS_H_*/
