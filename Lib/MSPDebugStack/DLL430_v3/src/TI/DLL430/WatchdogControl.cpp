/*
 * WatchdogControl.cpp
 *
 * Handles control over watchdog.
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
#include "WatchdogControl.h"

using namespace TI::DLL430;

WatchdogControl::WatchdogControl(uint16_t addr)
 : addr(addr)
 , value(0)
{
}

bool WatchdogControl::checkRead(uint16_t wdtCtrl)
{
	// watchdog password value: 0x69 for reading
	return ((wdtCtrl >> 8) & 0xFF) == 0x69;
}

void WatchdogControl::set(uint16_t wdtCtrl)
{
	this->value = (wdtCtrl & 0xFF);
}

uint16_t WatchdogControl::get() const
{
	return 0x6900 | this->value;
}

void WatchdogControl::addHoldParamsTo(HalExecElement* el) const
{
	el->appendInputData16(this->addr);
	el->appendInputData16(0x5A80);
}

void WatchdogControl::addRestoreParamsTo(HalExecElement* el) const
{
	el->appendInputData16(this->addr);
	el->appendInputData16(0x5A00 | this->value);
}

uint16_t WatchdogControl::getAddress() const
{
	return this->addr;
}
