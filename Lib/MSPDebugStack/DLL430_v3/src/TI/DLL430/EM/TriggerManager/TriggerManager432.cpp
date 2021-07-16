/*
 * TriggerManager432.cpp
 *
 * Handles trigger resources on 432
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

#include "../Trigger/Trigger432.h"
#include "../Trigger/DataAddressTrigger432.h"

#include "TriggerManager432.h"

#include "../Exceptions/Exceptions.h"

using namespace TI::DLL430;

TriggerManager432::TriggerManager432(std::vector<unsigned int>& vecCodeTriggers, std::vector<unsigned int>& vecLiteralTriggers, std::vector<unsigned int>& vecDataAddressTriggers, std::vector<unsigned int>& vecDataValueTriggers)
{
	for (const unsigned int codeTrigger : vecCodeTriggers)
	{
		mCodeTriggers.push_back(Trigger432(Trigger432::CODE_TRIGGER, codeTrigger));
	}

	for (const unsigned int literalTrigger : vecLiteralTriggers)
	{
		mLiteralTriggers.push_back(Trigger432(Trigger432::LITERAL_TRIGGER, literalTrigger));
	}

	for (const unsigned int dataAddressTrigger : vecDataAddressTriggers)
	{
		mDataAddressTriggers.push_back(DataAddressTrigger432(DataTrigger432::DATA_TRIGGER, dataAddressTrigger));
	}

	for (const unsigned int dataValueTrigger : vecDataValueTriggers)
	{
		mDataValueTriggers.push_back(DataValueTrigger432(DataTrigger432::DATA_TRIGGER, dataValueTrigger));
	}
	//Configure always available options
	DataTrigger432::accessTypeBits.clear();
	DataTrigger432::accessTypeBits[AT_NO_FETCH] = 0x8;
	DataTrigger432::accessTypeBits[AT_READ] = 0x5;
	DataTrigger432::accessTypeBits[AT_WRITE] = 0x6;
	DataTrigger432::accessTypeBits[AT_READ_WRITE] = 0x7;
}


Trigger432* TriggerManager432::getCodeTrigger()
{
	for (Trigger432& trigger : mCodeTriggers)
	{
		if (!trigger.isInUse())
		{
			trigger.isInUse(true);
			return &trigger;
		}
	}
	return nullptr;
}


Trigger432* TriggerManager432::getLiteralTrigger()
{
	for (Trigger432& trigger : mLiteralTriggers)
	{
		if (!trigger.isInUse())
		{
			trigger.isInUse(true);
			return &trigger;
		}
	}
	return nullptr;
}


DataTrigger432* TriggerManager432::getDataAddressTrigger()
{
	for (DataAddressTrigger432& trigger : mDataAddressTriggers)
	{
		if (!trigger.isInUse())
		{
			trigger.isInUse(true);
			return &trigger;
		}
	}
	return nullptr;
}

DataTrigger432* TriggerManager432::getDataValueTrigger()
{
	for (DataValueTrigger432& trigger : mDataValueTriggers)
	{
		if (!trigger.isInUse())
		{
			/* DataValueTrigger always requires a DataAddressTrigger */
			DataTrigger432 *dependencyTrigger = getDataAddressTrigger();
			if (dependencyTrigger == nullptr)
				break;
			trigger.isInUse(true);
			trigger.setDependencyTrigger(dependencyTrigger);
			return &trigger;
		}
	}
	return nullptr;
}


void TriggerManager432::releaseTrigger(Trigger432* trigger)
{
	trigger->reset();
}

void TriggerManager432::releaseDataTrigger432(DataTrigger432* trigger)
{
	trigger->reset();
}


int TriggerManager432::numAvailableCodeTriggers() const
{
	int count = 0;

	for (const Trigger432& trigger : mCodeTriggers)
	{
		if (!trigger.isInUse())
		{
			++count;
		}
	}
	return count;
}

int TriggerManager432::numAvailableDataAddressTriggers() const
{
	int count = 0;

	for (const DataAddressTrigger432& trigger : mDataAddressTriggers)
	{
		if (!trigger.isInUse())
		{
			++count;
		}
	}
	return count;
}

int TriggerManager432::numAvailableDataValueTriggers() const
{
	int count = 0;

	for (const DataValueTrigger432& trigger : mDataValueTriggers)
	{
		if (!trigger.isInUse())
		{
			++count;
		}
	}
	return count;
}

int TriggerManager432::numAvailableLiteralTriggers() const
{
	int count = 0;

	for (const Trigger432& trigger : mLiteralTriggers)
	{
		if (!trigger.isInUse())
		{
			++count;
		}
	}
	return count;
}

void TriggerManager432::writeAllTriggers() const
{
	for (const Trigger432& trigger : mCodeTriggers)
	{
		trigger.write();
	}
	for (const Trigger432& trigger : mLiteralTriggers)
	{
		trigger.write();
	}
	for (const DataAddressTrigger432& trigger : mDataAddressTriggers)
	{
		trigger.write();
	}

	for (const DataValueTrigger432& trigger : mDataValueTriggers)
	{
		trigger.write();
	}
}
