/*
 * ArmCpuMemoryAccess.cpp
 *
 * Implementaion for access of CPU registers.
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
#include "ArmCpuMemoryAccess.h"
#include "HalExecCommand.h"
#include "FetControl.h"
#include "WatchdogControl.h"
#include "IDeviceHandle.h"

using namespace TI::DLL430;

ArmCpuMemoryAccess::ArmCpuMemoryAccess (MemoryArea::Name name, IDeviceHandle* devHandle,
	uint32_t start,
	uint32_t size,
	uint32_t seg,
	uint32_t banks,
	bool mapped,
	uint8_t bits)
 : size(size)
 , localCache(size, 0)
 , devHandle(devHandle)
{
}

size_t ArmCpuMemoryAccess::getSize() const
{
	return size;
}


void ArmCpuMemoryAccess::pushCache()
{
	this->backupCache = this->localCache;
}
void ArmCpuMemoryAccess::popCache()
{
	this->localCache = this->backupCache;
	flushCache();
	fillCache(0, 18);
}

bool ArmCpuMemoryAccess::switchContext(uint32_t pc, uint32_t sp)
{
	if (!write(REGISTER_SP, sp))
	{
		return false;
	}
	if (!write(REGISTER_PC, pc))
	{
		return false;
	}
	if (!write(REGISTER_XPSR, 0x01000000))
	{
		return false;
	}
	if (!write(REGISTER_LR, 0xffffffff))
	{
		return false;
	}
	return true;
}

bool ArmCpuMemoryAccess::disableInterrupts()
{
	uint32_t specialRegister = 0x00;
	if (!read(REGISTER_SPECIAL, &specialRegister, 1))
	{
	return false;
	}
	if (!write(REGISTER_SPECIAL, specialRegister | (1<<0) | (1<<16)))
	{
		return false;
	}
	return true;
}

bool ArmCpuMemoryAccess::read(uint32_t Register, uint32_t* buffer, size_t count)
{
	if (Register + count > localCache.size())
	{
		return false;
	}
	if (buffer)
	{
		for (size_t i = 0; i < count; ++i)
		{
			buffer[i] = this->localCache[Register++];
		}
	}
	return true;
}

bool ArmCpuMemoryAccess::write(uint32_t Register, const uint32_t* buffer, size_t count)
{
	if (Register + count > localCache.size())
	{
		return false;
	}
	while (count--)
	{
		this->localCache[Register++] = *(buffer++);
	}
	return true;
}

bool ArmCpuMemoryAccess::write(uint32_t Register, uint32_t value)
{
	if (Register > localCache.size())
		return false;

	this->localCache[Register] = value;
	return true;
}

bool ArmCpuMemoryAccess::fillCache(uint32_t Register, size_t count)
{
	if ((Register + count) > localCache.size())
	{
		return false;
	}

	HalExecElement* el = new HalExecElement(this->devHandle->checkHalId(ID_ReadAllCpuRegs));

	HalExecCommand cmd;
	cmd.elements.emplace_back(el);

	if (!this->devHandle->send(cmd))
	{
		return false;
	}

	for (uint8_t i = 0; i < this->localCache.size(); ++i)
	{
		localCache[i] = cmd.elements[0]->getOutputAt32(i * 4);
	}
	return true;
}

bool ArmCpuMemoryAccess::flushCache()
{
	HalExecCommand cmd;
	HalExecElement* el = new HalExecElement(this->devHandle->checkHalId(ID_WriteAllCpuRegs));

	for (size_t i = 0; i < this->localCache.size(); i++)
	{
		el->appendInputData32(localCache[i]);
	}

	cmd.elements.emplace_back(el);

	if (!this->devHandle->send(cmd))
	{
		return false;
	}
	return true;
}

void ArmCpuMemoryAccess::clearCache(uint32_t Register, size_t count)
{
	if (Register + count > localCache.size())
	{
		return;
	}

	std::fill(localCache.begin(), localCache.begin() + count, 0);
}
