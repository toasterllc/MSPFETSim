/*
 * TinyRandomMemoryAccess.cpp
 *
 * Memory class for accessing RAM.
 *
 * Copyright (C) 2007 - 2016 Texas Instruments Incorporated - http://www.ti.com/
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
#include "TinyRandomMemoryAccess.h"
#include "HalExecCommand.h"
#include "IDeviceHandle.h"

using namespace TI::DLL430;

TinyRandomMemoryAccess::TinyRandomMemoryAccess(
				MemoryArea::Name name,
				IDeviceHandle* devHandle,
				uint32_t start,
				uint32_t end,
				uint32_t seg,
				uint32_t banks,
				bool mapped,
				const bool isProtected,
				IMemoryManager* mm,
				uint8_t psa)
 : MemoryAreaBase(name, devHandle, start, end, seg, banks, mapped, isProtected, psa)
 , mm(mm)
{}

bool TinyRandomMemoryAccess::doWrite(uint32_t address, const uint8_t* buffer, size_t count)
{
	if (count + address > this->getSize())
	{
		return false;
	}
	return this->writeBytes(address, buffer, count);
}


bool TinyRandomMemoryAccess::read(uint32_t address, uint8_t* buffer, size_t count)
{
	if (address+count > tinyRamCache.size())
	{
		return false;
	}
	if (buffer)
	{
		for (size_t i = 0; i < count; ++i)
		{
			buffer[i] = this->tinyRamCache[i+address];
		}
	}
	return true;
}


bool TinyRandomMemoryAccess::writeBytes(uint32_t address, const uint8_t* buffer, size_t count)
{
	for (uint8_t i = 0; i < count; ++i)
	{
		tinyRamCache[i+address] = buffer[i];
	}
	return true;
}

bool TinyRandomMemoryAccess::doWrite(uint32_t address, uint32_t value)
{
	return this->writeBytes(address, (uint8_t*)&value, 1);
}

bool TinyRandomMemoryAccess::fillCache()
{
	tinyRamCache.clear();
	tinyRamCache.resize(getSize());

	// backup tiny ram fist
	HalExecElement* readTinyRam = new HalExecElement(this->devHandle->checkHalId(ID_ReadMemWords));
	readTinyRam->appendInputData32(getStart());
	readTinyRam->appendInputData32(getSize()/2);

	HalExecCommand cmdRead;
	cmdRead.elements.emplace_back(readTinyRam);

	if (!this->devHandle->send(cmdRead))
	{
		return false;
	}
	for (uint8_t i = 0; i < tinyRamCache.size(); ++i)
	{
		tinyRamCache[i] = cmdRead.elements[0]->getOutputAt8(i);
	}

	// then write save pattern to tiny ram
	HalExecElement* writeTinyRam = new HalExecElement(this->devHandle->checkHalId(ID_WriteMemWords));
	writeTinyRam->appendInputData32(getStart());
	writeTinyRam->appendInputData32(getSize()/2);
	
	for (uint8_t i = 0; i < tinyRamCache.size()/2; ++i)
	{
		writeTinyRam->appendInputData16(0x3FFF);
	}
	HalExecCommand cmdWrite;
	cmdWrite.elements.emplace_back(writeTinyRam);

	if (!this->devHandle->send(cmdWrite))
	{
		return false;
	}
	return true;
}

bool TinyRandomMemoryAccess::flushCache() const
{
	HalExecElement* writeTinyRam = new HalExecElement(this->devHandle->checkHalId(ID_WriteMemWords));
	writeTinyRam->appendInputData32(getStart());
	writeTinyRam->appendInputData32(getSize()/2);

	for (uint8_t i = 0; i < tinyRamCache.size(); ++i)
	{
		writeTinyRam->appendInputData8(tinyRamCache[i]);
	}

	HalExecCommand cmd;
	cmd.elements.emplace_back(writeTinyRam);

	if (!this->devHandle->send(cmd))
	{
		return false;
	}
	return true;
}

bool TinyRandomMemoryAccess::verify(uint32_t address, const uint8_t* buffer, size_t count)
{
	bool status = true;

	if (count + address > tinyRamCache.size())
	{
		return false;
	}
	
	for (uint8_t i = 0; i < count; ++i)
	{
		if (tinyRamCache[i + address] != buffer[i])
		{
			status = false;
			break;
		}
	}

	return status;
}
