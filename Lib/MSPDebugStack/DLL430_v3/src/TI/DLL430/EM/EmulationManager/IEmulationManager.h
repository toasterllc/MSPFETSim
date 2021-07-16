/*
 * IEmulationManager.h
 *
 * Interface for emulation manager
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

#pragma once

#include <MessageData.h>
#include "../TriggerCondition/ITriggerConditionManager.h"
#include "../SoftwareBreakpoints/ISoftwareBreakpoints.h"

namespace TI { namespace DLL430 {

class IEmulationManager;
class IBreakpointManager;
class IClockControl;
class ICycleCounter;
class ISequencer;
class ITrace;
class IVariableWatch;


typedef std::shared_ptr<IEmulationManager> EmulationManagerPtr;
typedef std::shared_ptr<IBreakpointManager> BreakpointManagerPtr;
typedef std::shared_ptr<IClockControl> ClockControlPtr;
typedef std::shared_ptr<ICycleCounter> CycleCounterPtr;
typedef std::shared_ptr<ISequencer> SequencerPtr;
typedef std::shared_ptr<ITrace> TracePtr;
typedef std::shared_ptr<IVariableWatch> VariableWatchPtr;


//Encapsulating the entire emulation module and managing its components
class IEmulationManager
{
public:
	virtual ~IEmulationManager() {}

	//Get components (will throw if component does not exist/is not supported)
	virtual TriggerConditionManagerPtr getTriggerConditionManager() const = 0;
	virtual BreakpointManagerPtr getBreakpointManager() const = 0;
	virtual ClockControlPtr getClockControl() const = 0;
	virtual CycleCounterPtr getCycleCounter() const = 0;
	virtual SequencerPtr getSequencer() const = 0;
	virtual TracePtr getTrace() const = 0;
	virtual VariableWatchPtr getVariableWatch() const = 0;
	virtual SoftwareBreakpointsPtr getSoftwareBreakpoints() const = 0;

	//Check if a specific component is supported
	virtual bool hasTriggerConditionManager() const = 0;
	virtual bool hasBreakpointManager() const = 0;
	virtual bool hasClockControl() const = 0;
	virtual bool hasCycleCounter() const = 0;
	virtual bool hasSequencer() const = 0;
	virtual bool hasTrace() const = 0;
	virtual bool hasVariableWatch() const = 0;
	virtual bool hasSoftwareBreakpoints() const = 0;

	//Write current configuration for all components to target device
	virtual void writeConfiguration() const = 0;

	//Rewrite the current configuration (ex. after a reset)
	virtual void rewriteConfiguration() const = 0;

	//Process asynchronous events reported by the UIF
	virtual void onEvent(MessageDataPtr) const = 0;

	//Reset to default values, clear cache and triggers
	virtual void reset() = 0;

	virtual void fillEMInfo(DEVICE_T *deviceData) = 0;
	//These functions will be removed after reimplementation of clock control and cycle counter
	virtual void writeRegister(uint32_t reg, uint32_t value) const = 0;
	virtual uint32_t readRegister(uint32_t reg) const = 0;
};

}}
