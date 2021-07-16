/*
 * FramMemoryAccessBase.cpp
 *
 * Handles access to fram memory.
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

#include "FramMemoryAccessBase.h"
#include "ClockCalibration.h"
#include "HalExecCommand.h"
#include "IDeviceHandle.h"
#include "MpuFRx.h"
#include "WriteProtection.h"


using namespace TI::DLL430;

FramMemoryAccessBase::FramMemoryAccessBase
(
	MemoryArea::Name name,
	IDeviceHandle* devHandle,
	uint32_t start,
	uint32_t end,
	uint32_t seg,
	uint32_t banks,
	bool mapped,
	const bool isProtected,
	IMemoryManager* mm,
	uint8_t psa,
	IWriteProtection* writeProt,
	IMpu* mpu
)
: MainMemoryAccessBase(name, devHandle, start, end, seg, banks, mapped, isProtected, mm, psa)
, writeProtection(writeProt)
, mpu(mpu)
, mustUnlockMpu(false)
{
}

bool FramMemoryAccessBase::doWrite(uint32_t address, const uint8_t* buffer, size_t count)
{
	if (count > this->getSize())
	{
		return false;
	}

	address += this->getStart();

	MemoryArea* ram = mm->getMemoryArea(MemoryArea::Ram);
	if (ram == nullptr)
	{
		return false;
	}

	//32bit alignment
	const uint32_t alignedAddress = address & 0xfffffffc;
	const int frontPadding = address - alignedAddress;
	const int stubble = (address + static_cast<uint32_t>(count)) % 4;
	const int backPadding = (4 - stubble) % 4;

	HalExecElement* el = new HalExecElement(this->devHandle->checkHalId(ID_WriteFramQuickXv2));
	el->appendInputData32(alignedAddress);
	el->appendInputData32((static_cast<uint32_t>(count)+frontPadding + backPadding) / 2);

	std::vector<uint8_t> frontBuffer(frontPadding);
	std::vector<uint8_t> backBuffer(backPadding);

	if (frontPadding != 0)
	{
		mm->read(alignedAddress, &frontBuffer[0], frontPadding);
		mm->sync();
	}

	if (backPadding != 0)
	{
		mm->read(address + static_cast<uint32_t>(count), &backBuffer[0], backPadding);
		mm->sync();
	}

	if (!frontBuffer.empty())
	{
		el->appendInputData8(&frontBuffer[0], frontBuffer.size());
	}

	el->appendInputData8(buffer, count);

	if (!backBuffer.empty())
	{
		el->appendInputData8(&backBuffer[0], backBuffer.size());
	}

	this->elements.emplace_back(el);

	mustUnlockMpu = true;
	return true;
}

bool FramMemoryAccessBase::erase(uint32_t start, uint32_t end, uint32_t block_size, int type, bool forceUnlock)
{
	// check if valid erase type is used
	if ((type != ERASE_SEGMENT) && (type != ERASE_MAIN))
	{
		return false;
	}

	if (!writeProtection->disableIfEnabled())
	{
		return false;
	}
	// if the  MPU is enabled, disable it to enable memory erase
	if (!mpu->disableIfEnabled((type == ERASE_MAIN) || forceUnlock))
	{
		writeProtection->restore();
		return false;
	}

	// get Device RAM parameters for funclet upload
	MemoryArea* ram = mm->getMemoryArea(MemoryArea::Ram, 0);
	if (!ram)
	{
		return false;
	}

	if (!uploadFunclet(FuncletCode::ERASE))
	{
		return false;
	}

	std::shared_ptr<void> restoreRamOnExit(static_cast<void*>(0),
			std::bind(&FramMemoryAccessBase::restoreRam, this));

	//Note the erase on an FRAM device is just a dummy write with 0xFFFF to the device FRAM
	int32_t erase_address = start;

	const FuncletCode& funclet = devHandle->getFunclet(FuncletCode::ERASE);

	const uint32_t eraseType = 0;
	const uint32_t eraseLength = end - start + 1;
	const uint16_t flags = 0x0;
	const uint16_t programStartAddress = ram->getStart() + funclet.programStartOffset();

	HalExecCommand cmd;
	cmd.setTimeout(10000);	// overwrite 3 sec default with 10 sec

	HalExecElement* el = new HalExecElement(this->devHandle->checkHalId(ID_ExecuteFunclet));
	el->appendInputData16(static_cast<uint16_t>(ram->getStart() & 0xFFFF));
	el->appendInputData16(static_cast<uint16_t>(ram->getSize() & 0xFFFF));
	el->appendInputData16(programStartAddress);
	el->appendInputData32(static_cast<uint32_t>(erase_address));
	el->appendInputData32(eraseLength);
	el->appendInputData16(eraseType);
	el->appendInputData16(flags);
	el->appendInputData16(devHandle->getClockCalibration()->getCal0());
	el->appendInputData16(devHandle->getClockCalibration()->getCal1());

	//Dummy data to trigger execution of erase funclet
	el->appendInputData32(0xDEADBEEF);

	cmd.elements.emplace_back(el);

	bool success = this->devHandle->send(cmd);
	this->mpu->restore();
	this->writeProtection->restore();
	return success;
}

bool FramMemoryAccessBase::preSync()
{
	if (mustUnlockMpu)
	{
		mustUnlockMpu = false;
		if (!writeProtection->disableIfEnabled())
		{
			return false;
		}
		if (!mpu->disableIfEnabled(false))
		{
			writeProtection->restore();
			return false;
		}
	}
	return MainMemoryAccessBase::preSync();
}

bool FramMemoryAccessBase::postSync(const HalExecCommand& cmd)
{
	mpu->restore();
	writeProtection->restore();
	return MainMemoryAccessBase::postSync(cmd);
}
