/*
 * PinSequence.cpp
 *
 * Wrapper to send JTAG pin sequences
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

#include "PinSequence.h"
#include "FetHandle.h"
#include "HalExecElement.h"
#include "HalExecCommand.h"

using namespace std;
using namespace TI::DLL430;


PinState::PinState(JTAG_PIN pin, bool set, uint16_t delay)
	: mask(1<<pin), states(0), delay(delay)
{
	if (set)
		states |= (1 << pin);
}


PinState& PinState::operator()(JTAG_PIN pin, bool set)
{
	mask |= (1 << pin);

	if (set)
		states |= (1 << pin);
	else
		states &= ~(1 << pin);

	return *this;
}


PinState& PinState::setDelay(uint16_t ms)
{
	delay = ms;
	return *this;
}


bool TI::DLL430::sendPinSequence(const list<PinState>& pinStates, FetHandle* handle)
{
	HalExecElement* el = new HalExecElement(ID_BitSequence);
	el->appendInputData8( (uint8_t)pinStates.size() );

	for (const auto& it : pinStates)
	{
		el->appendInputData16( it.states );
		el->appendInputData16( it.mask );
		el->appendInputData16( it.delay );
	}

	HalExecCommand cmd;
	cmd.elements.emplace_back(el);
	return handle->send(cmd);
}
