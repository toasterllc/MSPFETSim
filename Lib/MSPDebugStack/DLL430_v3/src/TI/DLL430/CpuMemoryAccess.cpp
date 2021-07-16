/*
 * CpuMemoryAccess.cpp
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
#include "CpuMemoryAccess.h"
#include "HalExecCommand.h"
#include "FetControl.h"
#include "WatchdogControl.h"
#include "IDeviceHandle.h"

using namespace TI::DLL430;

CpuMemoryAccess::CpuMemoryAccess (MemoryArea::Name name, IDeviceHandle* devHandle,
	uint32_t start,
	uint32_t size,
	uint32_t seg,
	uint32_t banks,
	bool mapped,
	uint8_t bits)
 : size(size)
 , bytes((bits + 7) / 8) //number of bytes that bits fit in
 , localCache(16, 0)
 , devHandle(devHandle)
{
}

size_t CpuMemoryAccess::getSize() const
{
	return size;
}

bool CpuMemoryAccess::read(uint32_t Register, uint32_t* buffer, size_t count)
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

bool CpuMemoryAccess::write(uint32_t Register, const uint32_t* buffer, size_t count)
{
	if (Register + count > localCache.size())
	{
		return false;
	}
	uint32_t value = 0;
	while (count--)
	{
		value = *(buffer++);
		if (Register == 1)
		{
			value &= 0xfffffffe;
		}
		this->localCache[Register++] = value;
	}
	return true;
}

bool CpuMemoryAccess::write(uint32_t Register, uint32_t value)
{
	if (Register > localCache.size())
		return false;

	// remove bit 0 of SP silently if set
	if (Register==1)
	{
		value&=0xfffffffe;
	}
	this->localCache[Register] = value;
	return true;
}

bool CpuMemoryAccess::fillCache(uint32_t Register, size_t count)
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
	int pos = 0;
	for (uint8_t i = 0; i < this->localCache.size(); ++i)
	{
		if ((1<<i) & 0xFFF2)
		{
			this->localCache[i]=0;
			for (int j = 0; j < this->bytes; ++j)
			{
				this->localCache[i] |= el->getOutputAt8(pos) << (j*8);
				++pos;
			}
		}
	}
	return true;
}

bool CpuMemoryAccess::flushCache()
{
	HalExecCommand cmd;
	HalExecElement* el = new HalExecElement(this->devHandle->checkHalId(ID_WriteAllCpuRegs));

	for (size_t i=0;i<this->localCache.size();i++)
	{
		if ((1<<i)&0xFFF2)
		{
			for (int j=0;j<this->bytes;j++)
			{
				el->appendInputData8((this->localCache[i] >> (j * 8)) & 0xFF);
			}
		}
	}
	cmd.elements.emplace_back(el);
	if (!this->devHandle->send(cmd))
	{
		return false;
	}
	return true;
}

void CpuMemoryAccess::clearCache(uint32_t Register, size_t count)
{
	for (uint8_t k = Register; k < Register+count; ++k)
	{
		localCache[k] = 0x0;
	}
}
