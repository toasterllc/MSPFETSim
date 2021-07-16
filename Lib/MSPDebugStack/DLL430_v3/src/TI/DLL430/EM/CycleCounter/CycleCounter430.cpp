/*
 * CycleCounter430.cpp
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

#include <pch.h>
#include "CycleCounter430.h"
#include "../TriggerCondition/ITriggerCondition.h"
#include "../EemRegisters/EemRegisterAccess430.h"
#include "../Exceptions/Exceptions.h"

using namespace TI::DLL430;

namespace {
	enum CycleCounterRegister
	{
		CCNT0CTL   = 0xb0,
		CCNT0L	   = 0xb2,
		CCNT0H	   = 0xb4,
		CCNT1CTL   = 0xb8,
		CCNT1L	   = 0xba,
		CCNT1H	   = 0xbc
	};

	enum CycleCounterControlBits
	{
		RESET = 0x40
	};

	const CycleCounterRegister cycleCounterRegisters[2][3] = { {CCNT0CTL, CCNT0L, CCNT0H},
															   {CCNT1CTL, CCNT1L, CCNT1H} };

	struct LfsrRegister
	{
		explicit LfsrRegister(uint64_t value = 0) : value(value) {}

		union
		{
			struct { uint32_t low, high; };
			uint64_t value;
		};
	};
}


uint64_t toLFSR(uint64_t value)
{
	const uint64_t hex2lfsr[15] = { 0x0, 0x1, 0x2, 0x5, 0xa, 0x4, 0x9, 0x3, 0x6, 0xd, 0xb, 0x7, 0xe, 0xc, 0x8 };

	uint64_t lfsr = 0;

	for (int i = 0; i < 40; i += 4)
	{
		lfsr |= hex2lfsr[value % 15] << i;
		value /= 15;
	}
	return lfsr;
}


uint64_t fromLFSR(uint64_t lfsr)
{
	const uint32_t lfsr2hex[16] = {0x0, 0x1, 0x2, 0x7, 0x5, 0x3, 0x8, 0xb, 0xe, 0x6, 0x4, 0xa, 0xd, 0x9, 0xc, 0};

	uint64_t value = 0;

	for (int i = 36; i >= 0; i -= 4)
	{
		value *= 15;
		value += lfsr2hex[(lfsr >> i) & 0xf];
	}
	return value;
}


CycleCounter430::CycleCounter430(size_t numCounters)
	: mCounters(numCounters)
{
	for (size_t i = 0; i < mCounters.size(); ++i)
	{
		//Start/stop with Jtag control, count CPU bus cycles
		mCounters[i].cycleCounterCntrl = CC_COUNT_ON_CPU_BUS;
	}
}


void CycleCounter430::writeConfiguration()
{
	for (size_t i = 0; i < mCounters.size(); ++i)
	{
		writeCounter(i);
	}
}


void CycleCounter430::addTriggerCondition(TriggerConditionPtr triggerCondition)
{
	if (triggerCondition)
	{
		triggerConditions_.push_back(triggerCondition);
		triggerCondition->addReaction(TR_CYCLE_COUNTER);
	}
}


void CycleCounter430::clearTriggerConditions()
{
	for (TriggerConditionPtr& condition : triggerConditions_)
	{
		condition->removeReaction(TR_CYCLE_COUNTER);
	}
	triggerConditions_.clear();
}


uint64_t CycleCounter430::getCounterValue(size_t counter) const
{
	return getCounter(counter).value;
}


void CycleCounter430::setCounterValue(size_t counter, uint64_t value)
{
	getCounter(counter).value = value;
}


void CycleCounter430::setCountMode(size_t counter, CounterCountMode mode)
{
	if (counter == 0 && mode == CC_COUNT_ON_REACTION)
		throw EM_Exception(PARAMETER_ERR, "Invalid mode for counter 0");

	getCounter(counter).cycleCounterCntrl &= ~0xF;
	getCounter(counter).cycleCounterCntrl |= mode;
}


void CycleCounter430::setMode(size_t counter, uint32_t mode, uint32_t shift)
{
	if (counter == 0 && mode == 0x1)
		throw EM_Exception(PARAMETER_ERR, "Invalid mode for counter 0");

	if (mCounters.size() < 2 && mode == 0x2)
		throw EM_Exception(PARAMETER_ERR, "Invalid mode with only one counter");

	getCounter(counter).cycleCounterCntrl &= ~(0x3 << shift);
	getCounter(counter).cycleCounterCntrl |= (mode << shift);
}


void CycleCounter430::resetCounter(size_t counter)
{
	getCounter(counter).cycleCounterCntrl |= RESET;
}


void CycleCounter430::readCounter(size_t counter)
{
	Counter& cnt = getCounter(counter);
	cnt.cycleCounterCntrl = readEemRegister430(cycleCounterRegisters[counter][0]);

	LfsrRegister lfsrRegister;
	lfsrRegister.low = readEemRegister430(cycleCounterRegisters[counter][1]);
	lfsrRegister.high = readEemRegister430(cycleCounterRegisters[counter][2]);

	cnt.value = fromLFSR(lfsrRegister.value);
}


void CycleCounter430::writeCounter(size_t counter)
{
	Counter& cnt = getCounter(counter);
	const bool isReset = (cnt.cycleCounterCntrl & RESET) != 0;

	writeEemRegister430(cycleCounterRegisters[counter][0], cnt.cycleCounterCntrl, isReset);
	cnt.cycleCounterCntrl &= ~RESET;

	LfsrRegister lfsrRegister( toLFSR(cnt.value) );
	writeEemRegister430(cycleCounterRegisters[counter][1], lfsrRegister.low);
	writeEemRegister430(cycleCounterRegisters[counter][2], lfsrRegister.high);
}


CycleCounter430::Counter& CycleCounter430::getCounter(size_t counter)
{
	if (counter >= mCounters.size())
		throw EM_Exception(PARAMETER_ERR, "Invalid counter id");

	return mCounters[counter];
}

const CycleCounter430::Counter& CycleCounter430::getCounter(size_t counter) const
{
	if (counter >= mCounters.size())
		throw EM_Exception(PARAMETER_ERR, "Invalid counter id");

	return mCounters[counter];
}
