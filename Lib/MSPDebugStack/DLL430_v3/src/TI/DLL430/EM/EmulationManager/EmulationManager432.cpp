/*
 * EmulationManager432.cpp
 *
 * Emulation manager holds and manages the different modules
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
#include "EmulationManager432.h"
#include "../BreakpointManager/IBreakpointManager.h"
#include "../TriggerManager/TriggerManager432.h"
#include "../TriggerCondition/TriggerConditionManager432.h"
#include "../Exceptions/Exceptions.h"

#include "../EemRegisters/EemRegisterAccess432.h"


using namespace TI::DLL430;


TriggerConditionManagerPtr EmulationManager432::getTriggerConditionManager() const
{
	if (!mTriggerConditionManager)
		throw EM_NoTriggerConditionManagerException();

	return mTriggerConditionManager;
}


BreakpointManagerPtr EmulationManager432::getBreakpointManager() const
{
	if (!mBreakpointManager)
		throw EM_NoBreakpointManagerException();

	return mBreakpointManager;
}


ClockControlPtr EmulationManager432::getClockControl() const
{
	throw EM_NoClockControlException();
}


CycleCounterPtr EmulationManager432::getCycleCounter() const
{
	throw EM_NoCycleCounterException();
}


SequencerPtr EmulationManager432::getSequencer() const
{
	throw EM_NoSequencerException();
}


TracePtr EmulationManager432::getTrace() const
{
	throw EM_NoTraceException();
}


VariableWatchPtr EmulationManager432::getVariableWatch() const
{
	throw EM_NoVariableWatchException();
}


SoftwareBreakpointsPtr EmulationManager432::getSoftwareBreakpoints() const
{
	throw EM_NoSoftwareBreakpointsException();
}


bool EmulationManager432::hasTriggerConditionManager() const
{
	return static_cast<bool>(mTriggerConditionManager);
}


bool EmulationManager432::hasBreakpointManager() const
{
	return static_cast<bool>(mBreakpointManager);
}


bool EmulationManager432::hasClockControl() const
{
	return false;
}


bool EmulationManager432::hasCycleCounter() const
{
	return false;
}


bool EmulationManager432::hasSequencer() const
{
	return false;
}


bool EmulationManager432::hasTrace() const
{
	return false;
}


bool EmulationManager432::hasVariableWatch() const
{
	return false;
}


bool EmulationManager432::hasSoftwareBreakpoints() const
{
	return false;
}


void EmulationManager432::writeConfiguration() const
{
	if (mTriggerManager)
	{
		mTriggerManager->writeAllTriggers();
	}
}


void EmulationManager432::rewriteConfiguration() const
{
	writeConfiguration();
}


void EmulationManager432::onEvent(MessageDataPtr messageData)  const
{
}


void EmulationManager432::writeRegister(uint32_t reg, uint32_t value) const
{
	writeEemRegister432(reg, value);
}


uint32_t EmulationManager432::readRegister(uint32_t reg) const
{
	return readEemRegister432(reg);
}

void EmulationManager432::fillEMInfo(DEVICE_T *deviceData)
{
	deviceData->nDataWatchpoints = this->mTriggerManager->numAvailableDataAddressTriggers();
	deviceData->nDataWatchpointsValueMatch = this->mTriggerManager->numAvailableDataValueTriggers();
}