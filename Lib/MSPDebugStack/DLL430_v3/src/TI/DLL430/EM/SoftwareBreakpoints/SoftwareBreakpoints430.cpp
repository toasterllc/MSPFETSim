/*
 * SoftwareBreakpoints430.cpp
 *
 * Software breakpoint functionality for MSP430.
 *
 * Copyright (C) 2007 - 2013 Texas Instruments Incorporated - http://www.ti.com/
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

#include "SoftwareBreakpoints430.h"
#include "../Exceptions/Exceptions.h"
#include "../TriggerCondition/DataValueCondition430.h"

using namespace TI::DLL430;

static const uint16_t swbpInstruction = 0x4343;

SoftwareBreakpoints430::SoftwareBreakpoints430(TriggerManager430Ptr triggerManager)
	: mTriggerManager(triggerManager)
	, mSwbpManager(std::make_shared<SoftwareBreakpointManager>(swbpInstruction))
{
}


void SoftwareBreakpoints430::enable()
{
	if (!mSwbpInstructionTrigger)
	{
		if (mTriggerManager->numAvailableBusTriggers() < 1)
			throw EM_TriggerResourceException();

		mSwbpInstructionTrigger = std::make_shared<DataValueCondition430>(mTriggerManager, swbpInstruction, 0xffff);
		mSwbpInstructionTrigger->addReaction(TR_BREAK);
	}
}


void SoftwareBreakpoints430::disable()
{
	mSwbpManager->clearSoftwareTriggers();
	mSwbpInstructionTrigger.reset();
}

bool SoftwareBreakpoints430::isEnabled() const
{
	return static_cast<bool>(mSwbpInstructionTrigger);
}

SoftwareBreakpointManagerPtr SoftwareBreakpoints430::getSwbpManager() const
{
	if (!mSwbpManager)
		throw EM_NoSoftwareBreakpointsException();

	return mSwbpManager;
}
