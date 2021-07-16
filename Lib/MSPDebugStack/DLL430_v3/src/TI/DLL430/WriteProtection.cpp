/*
 * WriteProtection.cpp
 *
 * Copyright (C) 2007 - 2014 Texas Instruments Incorporated - http://www.ti.com/
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

#include "WriteProtection.h"
#include "IMemoryManager.h"

using namespace TI::DLL430;

WriteProtection::WriteProtection(
	IMemoryManager* mm,
	uint32_t Register,
	uint16_t Bits,
	uint16_t Mask,
	uint16_t Pwd
)
	: mm(mm)
	, Register(Register), Bits(Bits), Mask(Mask), Pwd(Pwd)
	, registerValue(0), registerBackup(0)
{}

bool WriteProtection::disableIfEnabled()
{
	//Must not read again without restoring first
	if (registerBackup != registerValue)
	{
		return false;
	}
	if (!readSettings())
	{
		return false;
	}
	return (registerValue & Bits) ? disable() : true;
}

void WriteProtection::restore()
{
	if (registerBackup != registerValue)
	{
		if (MemoryArea* peripheral = mm->getMemoryArea(MemoryArea::Peripheral16bit, 0))
		{
			if (peripheral->write(Register - peripheral->getStart(), registerBackup) && peripheral->sync())
			{
				registerValue = registerBackup;
			}
		}
	}
}

bool WriteProtection::isEnabled()
{
	readSettings();
	return (registerValue & Bits) ? true : false;
}

bool WriteProtection::readSettings()
{
	bool success = false;

	if (MemoryArea* peripheral = mm->getMemoryArea(MemoryArea::Peripheral16bit, 0))
	{
		uint8_t buffer[2] = { 0 };
		success = peripheral->read(Register - peripheral->getStart(), buffer, 2) && peripheral->sync();
		if (success)
		{
			registerValue = (buffer[0] + (buffer[1] << 8)) & 0xFFFF;

			//Replace pwd read value with write value
			if (Pwd != 0)
			{
				registerValue = (registerValue & Mask) | Pwd;
			}
			registerBackup = registerValue;
		}
	}
	return success;
}

bool WriteProtection::disable()
{
	bool success = false;

	if (MemoryArea* peripheral = mm->getMemoryArea(MemoryArea::Peripheral16bit, 0))
	{
		const uint16_t newRegisterValue = (registerValue & ~Bits);
		success = peripheral->write(Register - peripheral->getStart(), newRegisterValue) && peripheral->sync();
		if (success)
		{
			registerValue = newRegisterValue;
		}
	}
	return success;
}

WriteProtection432::WriteProtection432(
	IMemoryManager* mm,
	uint32_t Register,
	uint16_t Bits,
	uint16_t Mask,
	uint16_t Pwd,
	uint32_t unlockAddr
	)
	: mm(mm)
	, Register(Register), Bits(Bits), Mask(Mask), Pwd(Pwd), Value(0), unlockAddress(unlockAddr)
{

}

bool WriteProtection432::readSettings()
{
	bool success = false;

	if (MemoryArea* eem = mm->getMemoryArea(MemoryArea::Eem, 0))
	{
		success = eem->write(unlockAddress - eem->getStart(), Pwd) && eem->sync();
		if (!success)
		{
			return false;
		}

		uint8_t buffer[2] = { 0 };
		success = eem->read(Register - eem->getStart(), buffer, 2) && eem->sync();
		if (success)
		{
			Value = (buffer[0] + (buffer[1] << 8)) & Mask;
		}
	}
	return success;
}

bool WriteProtection432::isEnabled()
{
	if (!readSettings())
		return false;

	return (Value & Bits) ? true : false;
}
