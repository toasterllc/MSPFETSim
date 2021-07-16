/*
 * MemoryAreaFactory.cpp
 *
 * Abstract Factory for creating classes of interface type MemoryAreaBase.
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
#include "MemoryAreaFactory.h"
#include "EemMemoryAccess.h"
#include "CpuMemoryAccess.h"
#include "ArmCpuMemoryAccess.h"
#include "ReadonlyMemoryAccess.h"
#include "RegisterAccessBase.h"
#include "FlashMemoryAccessBase.h"
#include "DeviceInfo.h"
#include "IDeviceHandle.h"

using namespace TI::DLL430;


MemoryAreaFactory::MemoryAreaFactory (IDeviceHandle* devHandle, PsaType psaType, uint8_t bits)
 : devHandle(devHandle)
 , psa(psaType)
 , defaultBits(bits)
{
}

CpuRegisters* MemoryAreaFactory::createCpuRegisters(const MemoryInfo& memInfo)
{
	const uint8_t bits = (memInfo.bits != 0) ? memInfo.bits : defaultBits;
	if (memInfo.name == MemoryArea::Cpu)
	{
		if (memInfo.bits != 32)
		{
			return new CpuMemoryAccess(memInfo.name,
				this->devHandle,
				memInfo.start,
				memInfo.size,
				memInfo.segmentSize,
				memInfo.banks,
				memInfo.mapped,
				bits);
		}
		else
		{
			return new ArmCpuMemoryAccess(memInfo.name,
				this->devHandle,
				memInfo.start,
				memInfo.size,
				memInfo.segmentSize,
				memInfo.banks,
				memInfo.mapped,
				bits);
		}
	}
	return nullptr;
}

MemoryAreaBase* MemoryAreaFactory::createMemoryArea(IMemoryManager* mm, const MemoryInfo& memInfo)
{
	const uint8_t bits = (memInfo.bits != 0) ? memInfo.bits : defaultBits;

	if (memInfo.memoryCreatorPtr)
	{
		return (*memInfo.memoryCreatorPtr)(memInfo.name,
			this->devHandle,
			memInfo.start,
			memInfo.size,
			memInfo.segmentSize,
			memInfo.banks,
			memInfo.mapped,
			memInfo.protectable,
			mm,
			static_cast<uint8_t>(this->psa));
	}
	else
	{
		if (memInfo.name == MemoryArea::Eem)
		{
			return new EemMemoryAccess(memInfo.name, this->devHandle,
				memInfo.start,
				memInfo.size,
				memInfo.segmentSize,
				memInfo.banks,
				memInfo.mapped,
				bits);
		}

		switch (memInfo.type)
		{
		case MemoryType::Flash:
			return new FlashMemoryAccessBase(memInfo.name,
				this->devHandle,
				memInfo.start,
				memInfo.size,
				memInfo.segmentSize,
				memInfo.banks,
				memInfo.mapped,
				memInfo.protectable,
				mm,
				static_cast<uint8_t>(this->psa));
			break;

		case MemoryType::Rom:
			return new ReadonlyMemoryAccess(
				memInfo.name,
				this->devHandle,
				memInfo.start,
				memInfo.size,
				memInfo.segmentSize,
				memInfo.banks,
				memInfo.mapped,
				memInfo.protectable,
				mm,
				static_cast<uint8_t>(this->psa));
			break;

		case MemoryType::Ram:
			return new RandomMemoryAccess(
				memInfo.name,
				this->devHandle,
				memInfo.start,
				memInfo.size,
				memInfo.segmentSize,
				memInfo.banks,
				memInfo.mapped,
				memInfo.protectable,
				mm,
				static_cast<uint8_t>(this->psa));
			break;

		case MemoryType::Register:
			//Handles 8 and 16bit register access
			return new RegisterAccess(memInfo.name,
				this->devHandle,
				memInfo.start,
				memInfo.size,
				memInfo.segmentSize,
				memInfo.banks,
				memInfo.mapped,
				memInfo.bits,
				memInfo.mask);
			break;
		}
	}

	return nullptr;
}
