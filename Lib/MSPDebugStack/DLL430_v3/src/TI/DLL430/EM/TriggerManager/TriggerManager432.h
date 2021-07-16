/*
 * TriggerManager432.h
 *
 * Handles trigger resources on 430
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

#include "../Trigger/Trigger432.h"
#include "../Trigger/DataAddressTrigger432.h"
#include "../Trigger/DataValueTrigger432.h"

namespace TI { namespace DLL430 {

class TriggerManager432
{
public:
	//TriggerManager432(int numCodeTriggers, int numLiteralTriggers, int numDataAddressTriggers, int numDataValueTriggers);
	TriggerManager432(std::vector<unsigned int>& vecCodeTriggers, std::vector<unsigned int>& vecLiteralTriggers, std::vector<unsigned int>& vecDataAddressTriggers, std::vector<unsigned int>& vecDataValueTriggers);

	Trigger432* getCodeTrigger();
	Trigger432* getLiteralTrigger();
	DataTrigger432* getDataAddressTrigger();
	DataTrigger432* getDataValueTrigger();
	void releaseTrigger(Trigger432* trigger);
	void releaseDataTrigger432(DataTrigger432* trigger);

	int numAvailableCodeTriggers() const;
	int numAvailableDataAddressTriggers() const;
	int numAvailableDataValueTriggers() const;
	int numAvailableLiteralTriggers() const;

	void writeAllTriggers() const;

private:
	std::deque<Trigger432> mCodeTriggers;
	std::deque<Trigger432> mLiteralTriggers;
	std::deque<DataAddressTrigger432> mDataAddressTriggers;
	std::deque<DataValueTrigger432> mDataValueTriggers;
};

typedef std::shared_ptr<TriggerManager432> TriggerManager432Ptr;

}}
