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

#include "msp430.h"

#ifndef _HIL_ADC_CONTROL_H_
#define _HIL_ADC_CONTROL_H_

#define  A_VBUS             0 /* ADC12 Input Channel Select Bit 0 */
#define  A_VCCSUPPLY        1 /* ADC12 Input Channel Select Bit 1 target vcc*/
#define  A_VF               2 /* ADC12 Input Channel Select Bit 2 Fuse blow voltage */
#define  A_VCCSUPPLY_SENSE  3 /* ADC12 Input Channel Select Bit 3 external target vcc*/
#define  A_VCCDT            4 /* ADC12 Input Channel Select Bit 4 JTAG line vcc*/
#define  A_VCCDT_SENSE      6 /* ADC12 Input Channel Select Bit 6 short detection on Jtag lines*/

void            hil_AdcControlInitADC(void);
unsigned short  hil_AdcControlGetExternalVcc(void);
unsigned short  hil_AdcControlGetSupplyVcc(void);
unsigned short  hil_AdcControlGetVccDt(void);
unsigned short  hil_AdcControlGetVFuse(void);
unsigned short  hil_AdcControlGetDtVccSense(void);
void            hil_AdcControlRead(void);
void            hil_AdcControlInitADCvFuse(void);
#endif /*_HIL_DAC_CONTROL_H_*/
