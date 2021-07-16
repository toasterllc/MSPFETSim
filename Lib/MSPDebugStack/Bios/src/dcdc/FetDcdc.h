/*
 * FetDcdc.h
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

//! \ingroup MODULBIOS
//! \file FetDcdc.h
//! \brief
//!
//! \author  Berenbrinker Florian 02/01/2012)
#ifndef _FetDcdc_H_
#define _FetDcdc_H_

#define CMD_CONFIGURE               0x01
#define CMD_SETPWM                  0x02
#define CMD_CALLOAD                 0x03
#define CMD_POWERDOWN               0x04

// Defines
#define SDIO_SLAVE_ADDRESS        (0x8200)
#define SDIO_POWERDOWN_KEY        (0x5a5a)
#define SDIO_DELAY_AFTER_SENDING  (1000u)

struct calibrationResistors
{
    unsigned long time;
    unsigned long ticks;
    unsigned short resistor;
};
typedef struct calibrationResistors calibrationResistors_t;

struct calibrationValues
{
    unsigned short vcc;
    calibrationResistors_t resValues[5];
    unsigned short valid;
};
typedef struct calibrationValues calibrationValues_t;

struct DCDC_INFOS
{
    short (*getSubMcuVersion)(void);
    short (*getLayerVersion)(void);
    short (*dcdcCalibrate)(unsigned short resistor[4], unsigned short resCount, unsigned short vcc);
    short (*dcdcPowerDown)(void);
    short (*dcdcSetVcc)(unsigned short vcc);
    short (*dcdcRestart)(unsigned short fetType_);
    void  (*dcdc_getCalibrationValues)(unsigned short vcc, unsigned short resistor,  unsigned short resCount, unsigned long *ticks, unsigned long *time);
    short (*getLayerVersionCmp)(void);
};
typedef struct DCDC_INFOS DCDC_INFOS_t;

void dcdc_Init(DCDC_INFOS_t* dcdcInfos_Pointer);
short dcdc_Calibrate(unsigned short resistor[5], unsigned short resCount, unsigned short vcc);
short dcdc_PowerDown();
short dcdc_SetVcc(unsigned short vcc);
short dcdc_getSubMcuVersion();
short dcdc_getLayerVersion();
short dcdc_getLayerVersionCmp();
short dcdc_Restart(unsigned short fetType_);
short dcdc_Send(unsigned int cmd, unsigned int data);
short dcdc_Receive(unsigned int *data_read);
void dcdc_getCalibrationValues(unsigned short vcc, unsigned short resistor, unsigned short resCount, unsigned long *ticks, unsigned long *time);
#endif
