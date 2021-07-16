/*
 * ICycleCounter.h
 *
 * Interface for cycle counter module
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

#include "../TriggerCondition/ITriggerConditionManager.h"

namespace TI { namespace DLL430 {

enum CounterCountMode
{
	CC_COUNT_STOPPED		= 0x0,
	CC_COUNT_ON_REACTION	= 0x1,
	CC_COUNT_ON_IFCLK		= 0x2,
	CC_COUNT_ON_FETCH		= 0x4,
	CC_COUNT_ON_ALL_BUS		= 0x5,
	CC_COUNT_ON_CPU_BUS		= 0x6,
	CC_COUNT_ON_DMA_BUS		= 0x7,
};

enum CounterStartMode
{
	CC_START_ON_RELEASE		= 0x0,
	CC_START_ON_REACTION	= 0x1,
	CC_START_ON_COUNTER		= 0x2,
	CC_START_IMMEDIATELY	= 0x3
};

enum CounterStopMode
{
	CC_STOP_ON_DBG_HALT	= 0x0,
	CC_STOP_ON_REACTION	= 0x1,
	CC_STOP_ON_COUNTER	= 0x2,
	CC_STOP_NO_EVENT	= 0x3
};

enum CounterClearMode
{
	CC_CLEAR_NO_EVENT		= 0x0,
	CC_CLEAR_ON_REACTION	= 0x1,
	CC_CLEAR_ON_COUNTER		= 0x2
};


class ICycleCounter
{
public:
	virtual ~ICycleCounter() {}

	virtual void addTriggerCondition(TriggerConditionPtr condition) = 0;
	virtual void clearTriggerConditions() = 0;

	virtual uint64_t getCounterValue(size_t counter) const = 0;
	virtual void setCounterValue(size_t counter, uint64_t value) = 0;

	virtual void setCountMode(size_t counter, CounterCountMode mode) = 0;
	virtual void setStartMode(size_t counter, CounterStartMode mode) = 0;
	virtual void setStopMode(size_t counter, CounterStopMode mode) = 0;
	virtual void setClearMode(size_t counter, CounterClearMode mode) = 0;

	virtual void resetCounter(size_t counter) = 0;

	virtual void readCounter(size_t counter) = 0;
	virtual void writeCounter(size_t counter) = 0;

	//Only used by EmulationManager
	virtual void writeConfiguration() = 0;
};

}}
