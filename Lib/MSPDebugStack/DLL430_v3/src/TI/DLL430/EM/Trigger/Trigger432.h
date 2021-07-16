/*
 * Trigger432.h
 *
 * Common implementation for triggers on 432
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

#include <stdint.h>
#include "../TriggerCondition/TriggerDefinitions.h"

namespace TI { namespace DLL430 {

class Trigger432
{
public:
	enum TYPE { CODE_TRIGGER, LITERAL_TRIGGER};

	Trigger432(TYPE type, uint32_t id);
	virtual ~Trigger432() {}

	virtual uint32_t getId() const;

	virtual void read();
	virtual void write() const;
	virtual void reset();

	virtual void setValue(uint32_t value);

	virtual bool isInUse() const;
	virtual void isInUse(bool inUse);

	virtual bool isEnabled() const;
	virtual void isEnabled(bool enabled);

	virtual void setAccessType(AccessType accessType);
	virtual void setMask(uint32_t mask);

	static std::map<AccessType, uint16_t> accessTypeBits;

private:
	TYPE type_;

	uint32_t id_;

	bool isInUse_;
	bool isEnabled_;

	uint32_t comparatorRegister_;
};

}}
