/*
 * RegisterAccessBase.cpp
 *
 * Base class for all memory classes which access some type of register.
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
#include "RegisterAccessBase.h"
#include "HalExecCommand.h"
#include "IDeviceHandle.h"

using namespace TI::DLL430;

RegisterAccess::RegisterAccess (
	MemoryArea::Name name,
	IDeviceHandle* devHandle,
	uint32_t start,
	uint32_t end,
	uint32_t seg,
	uint32_t banks,
	bool mapped,
	uint8_t bits,
	const std::vector<uint8_t>& mask
)
 : MemoryAreaBase(name, devHandle, start, end, seg, banks, mapped, false/*isProtected*/, 0xff)
 , bits(bits), mask(mask)
{
}


bool RegisterAccess::doRead(uint32_t address, uint8_t* buffer, size_t count)
{
	const hal_id readMacro = (bits == 8) ? ID_ReadMemBytes : ID_ReadMemWords;
	defaultRead(readMacro, nullptr, address, buffer, count);

	//Patch in the watchdog value the device would have while running
	const int wdtOffset = devHandle->getWatchdogControl()->getAddress() - getStart() - address;

	if (wdtOffset >= 0 && wdtOffset < (int)count)
	{
		//Force sync immediately, instead of trying to patch watchdog in postSync
		if (!sync())
		{
			return false;
		}

		buffer[wdtOffset] = devHandle->getWatchdogControl()->get() & 0xFF;
	}
	return true;
}

bool RegisterAccess::doWrite(uint32_t address, uint32_t value)
{
	const size_t numBytes = bits / 8;
	const uint8_t *buf = reinterpret_cast<uint8_t*>(&value);
	return this->doWrite(address, buf, numBytes);
}

/** each buffer element contains one _byte_ */
bool RegisterAccess::doWrite(uint32_t address, const uint8_t* buffer, size_t count)
{
	//If watchdog is written, set password and stop bit
	//and store value the watchdog will have while running
	const uint8_t WDT_STOP_BIT = 0x80;
	const uint8_t WDT_PW = 0x5A;

	const int wdtOffset = devHandle->getWatchdogControl()->getAddress() - getStart() - address;

	uint8_t start_val = 0;
	const uint32_t end_adr = address + static_cast<uint32_t>(count);
	size_t paddedCount = count;

	if (address & 1)
	{
		++paddedCount;
		if (!doRead(address - 1, &start_val, 1) || !sync())
		{
			return false;
		}
	}

	uint8_t end_val = 0;
	if (end_adr & 1)
	{
		++paddedCount;
		if (!doRead(end_adr, &end_val, 1) || !sync())
		{
			return false;
		}
	}

	const hal_id writeMacro = (bits == 8) ? ID_WriteMemBytes : ID_WriteMemWords;

	HalExecElement* el = new HalExecElement(devHandle->checkHalId(writeMacro));
	address += getStart();

	el->appendInputData32(address & 0xfffffffe);
	el->appendInputData32(static_cast<uint32_t>(paddedCount / 2));

	if (address & 1)
	{
		el->appendInputData8(start_val);
	}
	for (size_t i = 0; i < count; ++i)
	{
		uint8_t val = buffer[i];
		if (i == wdtOffset)
		{
			devHandle->getWatchdogControl()->set(buffer[i]);
			val |= WDT_STOP_BIT;
		}
		else if (i == wdtOffset + 1)
		{
			val = WDT_PW;
		}
		el->appendInputData8(val);
	}
	if (end_adr & 1)
	{
		el->appendInputData16(end_val);
	}

	this->elements.emplace_back(el);

	return true;
}

bool RegisterAccess::postSync(const HalExecCommand& cmd)
{
	ReadElement_map readElements = this->readMap;

	const bool success = MemoryAreaBase::postSync(cmd);

	if (success)
	{
		for (const ReadElement_map::value_type& e : readElements)
		{
			if (this->readMap.find(e.first) == this->readMap.end())
			{
				const size_t address = e.second.offset + (e.second.omitFirst ? 1 : 0);
				const size_t size = e.second.size - (e.second.omitFirst ? 1 : 0) - (e.second.omitLast ? 1 : 0);
				for (size_t i = 0; i < size; ++i)
				{
					e.second.v_buffer[i] &= (address + i < mask.size()) ? mask[address + i] : 0xff;
				}
			}
		}
	}
	return success;
}


bool RegisterAccess5xx::doRead(uint32_t address, uint8_t* buffer, size_t count)
{
	const uint32_t sfrIfgRegister = 0x102;
	uint8_t sfrIfgBackup[2] = {0};

	//Save and restore state of VMAIFG
	return RegisterAccess::doRead(sfrIfgRegister - this->getStart(), sfrIfgBackup, 2) && sync() &&
		   RegisterAccess::doRead(address, buffer, count) &&
		   doWrite(sfrIfgRegister - this->getStart(), sfrIfgBackup, 2);
}
