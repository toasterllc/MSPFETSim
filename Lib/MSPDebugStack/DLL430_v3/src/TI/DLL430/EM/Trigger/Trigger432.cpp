/*
 * Trigger432.cpp
 *
 * Common implementation for triggers on 430
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

#include "Trigger432.h"
#include "../EemRegisters/EemRegisterAccess432.h"
#include "../Exceptions/Exceptions.h"

using namespace std;
using namespace TI::DLL430;

Trigger432::Trigger432(TYPE type, uint32_t id)
	: type_(type)
	, id_(id)
	, isInUse_(false)
	, isEnabled_(false)
	, comparatorRegister_(0)
{
}

uint32_t Trigger432::getId() const
{
	return id_;
}

void Trigger432::read()
{
}

void Trigger432::write() const
{
		writeEemRegister432(0x2008 + id_ * 4, comparatorRegister_ | (isEnabled_ ? 1 : 0));
}

void Trigger432::reset()
{
	isInUse_ = false;
	isEnabled_ = false;
}

void Trigger432::setValue(uint32_t value)
{
	comparatorRegister_ = (value & 2) ? 0x80000000 : 0x40000000;
	comparatorRegister_ |= (value & 0xFFFFFFFC);
}

void Trigger432::setAccessType(AccessType accessType)
{

}

void Trigger432::setMask(uint32_t mask)
{

}

bool Trigger432::isInUse() const
{
	return isInUse_;
}

void Trigger432::isInUse(bool inUse)
{
	isInUse_ = inUse;
	isEnabled_ = inUse;
}

bool Trigger432::isEnabled() const
{
	return isEnabled_;
}

void Trigger432::isEnabled(bool enabled)
{
	isEnabled_ = enabled;
}
