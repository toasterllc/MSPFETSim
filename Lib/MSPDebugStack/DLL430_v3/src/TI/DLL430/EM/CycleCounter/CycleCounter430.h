/*
 * CycleCounter430.h
 *
 * Cycle counter functionality for MSP430.
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

#include "ICycleCounter.h"

namespace TI { namespace DLL430 {

class CycleCounter430 : public ICycleCounter
{
public:
	explicit CycleCounter430(size_t numCounters);

	virtual void writeConfiguration();

	virtual void addTriggerCondition(TriggerConditionPtr condition);
	virtual void clearTriggerConditions();

	virtual uint64_t getCounterValue(size_t counter) const;
	virtual void setCounterValue(size_t counter, uint64_t value);

	virtual void setCountMode(size_t counter, CounterCountMode mode);
	virtual void setStartMode(size_t counter, CounterStartMode mode) { setMode(counter, mode, 8); }
	virtual void setStopMode(size_t counter, CounterStopMode mode)   { setMode(counter, mode, 10); }
	virtual void setClearMode(size_t counter, CounterClearMode mode) { setMode(counter, mode, 12); }

	virtual void resetCounter(size_t counter);

	virtual void readCounter(size_t counter);
	virtual void writeCounter(size_t counter);

private:
	struct Counter
	{
		Counter() : cycleCounterCntrl(0), value(0) {}

		uint16_t cycleCounterCntrl;
		uint64_t value;
	};

	void setMode(size_t counter, uint32_t mode, uint32_t shift);

	Counter& getCounter(size_t counter);
	const Counter& getCounter(size_t counter) const;

	std::vector<Counter> mCounters;
	std::vector<TriggerConditionPtr> triggerConditions_;
};

}}
