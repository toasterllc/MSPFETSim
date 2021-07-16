/*
 * UsbIoChannel.cpp
 *
 * Base class for USB IOChannel devices, e.g. CDC or HID.
 *
 * Copyright (C) 2007 - 2011 Texas Instruments Incorporated - http://www.ti.com/
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

#include <pch.h>
#include "UsbIoChannel.h"
#include "HalExecCommand.h"
#include "HalExecElement.h"
#include "FetControl.h"
#include <assert.h>

using namespace TI::DLL430;


UsbIoChannel::UsbIoChannel(const PortInfo& portInfo)
 : portInfo(portInfo)
{
}

UsbIoChannel::~UsbIoChannel ()
{
}

uint16_t UsbIoChannel::createCrc(const uint8_t * buf)
{
	uint16_t crc = 0x0000;
	uint8_t count = ((buf[0]+1) >> 1);

	if (!(buf[0] & 0x01))
	{
		count++;
	}

	for (int i = 0; i < count; ++i)
	{
		const uint16_t val = (buf[(i*2)+1] << 8) + buf[i*2];
		crc ^= val;
	}
	crc ^= 0xFFFF;

	return crc;
}
