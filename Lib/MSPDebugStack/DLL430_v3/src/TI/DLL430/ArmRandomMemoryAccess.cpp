/*
 * ArmRandomMemoryAccess.cpp
 *
 * Memory class for accessing RAM.
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
#include "ArmRandomMemoryAccess.h"
#include "HalExecCommand.h"
#include "IDeviceHandle.h"
#include "MSP432_FlashLib.h"

namespace MSP432_DataWidth
{
	static const uint8_t DATA_WITH_8 = 0x0;
	static const uint8_t DATA_WITH_16 = 0x1;
	static const uint8_t DATA_WITH_32 = 0x2;

	static const uint8_t AP_READ = 0x1;
	static const uint8_t AP_WRITE = 0x0;
}

using namespace TI::DLL430;

ArmRandomMemoryAccess::ArmRandomMemoryAccess(
	MemoryArea::Name name,
	IDeviceHandle* devHandle,
	uint32_t start,
	uint32_t size,
	uint32_t seg,
	uint32_t banks,
	bool mapped,
	const bool isProtected,
	IMemoryManager* mm,
	uint8_t psa)
	: MemoryAreaBase(name, devHandle, start, size, seg, banks, mapped, isProtected, psa), mm(mm)
{
}

ArmRandomMemoryAccess::~ArmRandomMemoryAccess()
{
}

bool ArmRandomMemoryAccess::doWrite(uint32_t address, const uint8_t* buffer, size_t count)
{
	HalExecElement *el;
	if (this->devHandle->getInterfaceMode() == SWD_MSP432)
	{
		el = new HalExecElement(ID_MemApTransactionArmSwd);
	}
	else
	{
		el = new HalExecElement(ID_MemApTransactionArm);
	}


	el->appendInputData16(0);                // APSEL
	el->appendInputData16(MSP432_DataWidth::AP_WRITE); 

	if ((address & 0x1) || ((address + count) & 01))
	{
		el->appendInputData16(MSP432_DataWidth::DATA_WITH_8);
	}
	else if ((address & 0x2) || ((address + count) & 02))
	{
		el->appendInputData16(MSP432_DataWidth::DATA_WITH_16);
	}
	else
	{
		if (name == MemoryArea::Peripheral16bit)
		{// Force 16-bit transactions to avoid bus errors when accessing 16-bit peripherals
			el->appendInputData16(MSP432_DataWidth::DATA_WITH_16);
		}
		else
		{
			el->appendInputData16(MSP432_DataWidth::DATA_WITH_32);
		}
	}

	el->appendInputData32(address + this->getStart());// address
	el->appendInputData32(static_cast<uint32_t>(count));                    // size in bytes

	for (size_t i = 0; i < count; ++i)
	{
		el->appendInputData8(static_cast<uint8_t>(buffer[i]));
	}

	this->elements.emplace_back(el);

	return true;
}

bool ArmRandomMemoryAccess::doRead(uint32_t address, uint8_t* buffer, size_t count)
{
	HalExecElement *el;
	if (this->devHandle->getInterfaceMode() == SWD_MSP432)
	{
		el = new HalExecElement(ID_MemApTransactionArmSwd);
	}
	else
	{
		el = new HalExecElement(ID_MemApTransactionArm);
	}

	el->appendInputData16(0);                // APSEL
	el->appendInputData16(MSP432_DataWidth::AP_READ); 

	if ((address & 0x1) || ((address + count) & 01))
	{
		el->appendInputData16(MSP432_DataWidth::DATA_WITH_8);
	}
	else if ((address & 0x2) || ((address + count) & 02))
	{
		el->appendInputData16(MSP432_DataWidth::DATA_WITH_16);
	}
	else
	{
		if (name == MemoryArea::Peripheral16bit)
		{// Force 16-bit transactions to avoid bus errors when accessing 16-bit peripherals
			el->appendInputData16(MSP432_DataWidth::DATA_WITH_16);
		}
		else
		{
			el->appendInputData16(MSP432_DataWidth::DATA_WITH_32);
		}
	}

	el->appendInputData32(address + this->getStart());// address
	el->appendInputData32(static_cast<uint32_t>(count));                    // size in bytes

	ReadElement r(buffer, count, 0, 0, 0);
	this->readMap[this->elements.size()] = r;
	this->elements.emplace_back(el);

	return true;
}

bool ArmRandomMemoryAccess::doWrite(uint32_t address, uint32_t value)
{
	return doWrite(address, (uint8_t*)&value, sizeof(value));
}

bool ArmRandomMemoryAccess::verify(uint32_t address, const uint8_t* buffer, size_t count)
{	
	std::vector<uint8_t> tmpBuffer(count);

	if (!doRead(address, tmpBuffer.data(), count) ||!sync())
	{
		return false;
	}
	return memcmp(buffer ? buffer : std::vector<uint8_t>(count, 0xFF).data(), tmpBuffer.data(), count) == 0;
}

bool ArmRandomMemoryAccess::postSync(const HalExecCommand& cmd)
{
	return MemoryAreaBase::postSync(cmd);
}

bool ArmRandomMemoryAccess::uploadFunclet()
{
	bool success = false;

	const FuncletCode& funclet = devHandle->getFunclet(FuncletCode::Type::ERASE);

	if (!mm)
	{
		return false;
	}

	MemoryArea* ram = mm->getMemoryArea(MemoryArea::Ram, 0);

	if (funclet.maxPayloadSize() <= ram->getSize())
	{
		if (mm->getRamPreserveMode())
		{
			const size_t backupSize = std::min((size_t)ram->getSize(), funclet.maxPayloadSize());
			ramBackup.resize(backupSize);
			if (!ram->read(0, &ramBackup[0], ramBackup.size()) || !ram->sync())
				return false;
		}
		else
		{
			ramBackup.clear();
		}
	}
	const uint8_t* code = (uint8_t*)funclet.codeNoOffset();
	const size_t count = funclet.codeSizeNoCheck();
	success = ram->write(0, code, count) && ram->sync();
	return success;
}

void ArmRandomMemoryAccess::restoreRam()
{
	if (!ramBackup.empty())
	{
		if (MemoryArea* ram = mm->getMemoryArea(MemoryArea::Ram, 1))
		{
			ram->write(0, &ramBackup[0], ramBackup.size()) && ram->sync();
		}
		ramBackup.clear();
	}
}