/*
 * FlashMemoryAccessBase.cpp
 *
 * Memory class for accessing flash memory.
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
#include "FlashMemoryAccessBase.h"
#include "HalExecCommand.h"
#include "IDeviceHandle.h"
#include "FetHandle.h"
#include "ClockCalibration.h"
#include "CpuRegisters.h"
#include "warnings/Warnings.h"


using namespace TI::DLL430;
using namespace std;

FlashMemoryAccessBase::FlashMemoryAccessBase (
				MemoryArea::Name name,
				IDeviceHandle* devHandle,
				uint32_t start,
				uint32_t end,
				uint32_t seg,
				uint32_t banks,
				bool mapped,
				const bool isProtected,
				IMemoryManager* mm,
				uint8_t psa
)
 : MainMemoryAccessBase(name, devHandle, start, end, seg, banks, mapped, isProtected, mm, psa)
 , funcletToUpload(FuncletCode::NONE)
{
}


bool FlashMemoryAccessBase::erase(uint32_t start, uint32_t end, uint32_t block_size, int type, bool forceUnlock)
{
	// check if valid erase type is used
	if ((type != ERASE_SEGMENT) && (type != ERASE_MAIN))
	{
		return false;
	}
	if (block_size < 1 || !mm)
	{
		return false;
	}

	// get Device RAM parameters for funclet upload
	MemoryArea* ram = mm->getMemoryArea(MemoryArea::Ram, 0);
	if (!ram)
	{
		return false;
	}

	if ( !mm->checkMinFlashVoltage() )
	{
		WarningFactory::instance()->message(MESSAGE_LEVEL_T::MSPDS_MESSAGE_LEVEL_WARNING, WarningCode::WARNING_FLASH_VCC);
		return false;
	}

	ClockCalibration* calibration = devHandle->getClockCalibration();

	if ( !calibration->backupSettings() )
	{
		return false;
	}

	shared_ptr<void> restoreFrequencyExit(static_cast<void*>(0),
									  bind(&ClockCalibration::restoreSettings, calibration));

	if ( !calibration->determineSettings() )
	{
		return false;
	}

	if ( !calibration->makeSettings() )
	{
		return false;
	}

	if (getFlags() & LOCK_INFO_A_FLAG)
	{
		if (!backupInfo())
		{
			return false;
		}
	}

	if ( !uploadFunclet(FuncletCode::ERASE) )
	{
		return false;
	}

	shared_ptr<void> restoreRamOnExit(static_cast<void*>(0),
						bind(&FlashMemoryAccessBase::restoreRam, this));

	if (mm)
	{
		/* FLASH31 workaround */
		CpuRegisters* cpu = mm->getCpuRegisters();
		if (!cpu)
		{
			return false;
		}
		cpu->fillCache(2, 1);

		uint32_t sr = 0;
		cpu->read(2, &sr, 1);

		// disable GIE on target
		sr &= ~ (0x08);

		cpu->write(2, &sr, 1);
		cpu->flushCache();
	}

	//Erase main in reverse, using end of bank as address (5xxx issue)
	//First bank is erased twice (with end and start of bank as address)
	int32_t address = (type != ERASE_MAIN) ? start : end - 1;
	const int32_t increment = (type != ERASE_MAIN) ? block_size : -static_cast<int32_t>(block_size);
	int count_commands=0;
	bool done = false;

	const FuncletCode& funclet = devHandle->getFunclet(FuncletCode::ERASE);

	uint32_t eraseType = 0;
	if ( type == ERASE_SEGMENT) eraseType = 0xA502;
	if ( type == ERASE_MAIN)	eraseType = 0xA504;

	const uint16_t flags = ( getFlags() & LOCK_INFO_A_FLAG ) ? 0xA548 : 0xA508;
	const size_t availableRam = min(funclet.maxPayloadSize(), ram->getSize() - funclet.codeSize());
	const uint16_t programStartAddress = ram->getStart() + funclet.programStartOffset();

	const int32_t smallerSegmentSize = (type != ERASE_MAIN) ? (getSize() % getSegmentSize()) : 0;
	const int32_t regularSegmentsStart = getStart() + smallerSegmentSize;

	do
	{
		HalExecCommand cmd;
		cmd.setTimeout(10000);	// overwrite 3 sec default with 10 sec

		do
		{
			if (address+2 == (int32_t)start)
				address += 2;

			HalExecElement* el = new HalExecElement(this->devHandle->checkHalId(ID_ExecuteFunclet));
			el->appendInputData16(static_cast<uint16_t>(ram->getStart() & 0xFFFF));
			el->appendInputData16(static_cast<uint16_t>(availableRam & 0xFFFF));
			el->appendInputData16(programStartAddress);
			el->appendInputData32(static_cast<uint32_t>(address));
			el->appendInputData32(0x2); //Payload size (32bit for erase)
			el->appendInputData16(eraseType);
			el->appendInputData16(flags);
			el->appendInputData16(devHandle->getClockCalibration()->getCal0());
			el->appendInputData16(devHandle->getClockCalibration()->getCal1());

			//Dummy data to trigger execution of erase funclet
			el->appendInputData32(0xDEADBEEF);

			cmd.elements.emplace_back(el);

			if (address < regularSegmentsStart)
				address += smallerSegmentSize;
			else
				address += increment;

			count_commands++;
			done = (address >= (int32_t)end) || (address+2 < (int32_t)start);
		}while (!done && (count_commands < 4));

		count_commands=0;

		if (!this->devHandle->send(cmd))
			return false;

	} while (!done);

	if (getFlags() & LOCK_INFO_A_FLAG)
	{
		restoreInfo();
	}
	return true;
}

bool FlashMemoryAccessBase::doOverwrite(uint32_t address, const uint8_t* data, size_t size)
{
	const uint32_t startAddress = getStart() + address;
	const uint32_t endAddress = startAddress + static_cast<uint32_t>(size);
	const uint32_t segSize = getSegmentSize();

	const uint32_t firstSegmentStart = max(getStart(), (startAddress/segSize)*segSize);
	const uint32_t lastSegmentEnd = ((endAddress+segSize-1) / segSize) * segSize;

	const uint32_t bufferOffset = startAddress - firstSegmentStart;
	const uint32_t totalSize = lastSegmentEnd - firstSegmentStart;

	vector<uint8_t> memBuffer(totalSize);

	if (startAddress > firstSegmentStart)
	{
		if (!doRead(firstSegmentStart - getStart(), &memBuffer[0], bufferOffset) || !sync())
			return false;
	}

	copy(data, data+size, memBuffer.begin() + bufferOffset);

	if (endAddress < lastSegmentEnd)
	{
		if (!doRead(endAddress - getStart(), &memBuffer[bufferOffset + size], lastSegmentEnd - endAddress) || !sync())
			return false;
	}

	if (!MainMemoryAccessBase::erase(firstSegmentStart, lastSegmentEnd))
		return false;

	if (!doWrite(firstSegmentStart - getStart(), &memBuffer[0], memBuffer.size()))
		return false;

	return true;
}

bool FlashMemoryAccessBase::doWrite(uint32_t address, const uint8_t* buffer, size_t count)
{
	if (count > this->getSize() || !mm)
	{
		return false;
	}

	address += this->getStart();

	MemoryArea* ram = mm->getMemoryArea(MemoryArea::Ram);
	if (ram == nullptr)
	{
		return false;
	}

	if (!mm->checkMinFlashVoltage())
	{
		WarningFactory::instance()->message(MESSAGE_LEVEL_T::MSPDS_MESSAGE_LEVEL_WARNING, WarningCode::WARNING_FLASH_VCC);
		return false;
	}

	if (mm)
	{
		/* FLASH31 workaround */
		CpuRegisters* cpu = mm->getCpuRegisters();
		if (!cpu)
		{
			return false;
		}
		cpu->fillCache(2, 1);

		uint32_t sr = 0;
		cpu->read(2, &sr, 1);

		// disable GIE on target
		sr &= ~(0x08);

		cpu->write(2, &sr, 1);
		cpu->flushCache();
	}

	const FuncletCode& funclet = devHandle->getFunclet(FuncletCode::WRITE);

	const uint16_t programStartAddress = ram->getStart() + funclet.programStartOffset();

	Alignment alignedData = alignData(address, static_cast<uint32_t>(count));

	const uint16_t flags = ( getFlags() & LOCK_INFO_A_FLAG ) ? 0xA548 : 0xA508;
	const size_t availableRam = min(funclet.maxPayloadSize(), ram->getSize() - funclet.codeSize());

	HalExecElement* el = new HalExecElement(this->devHandle->checkHalId(ID_ExecuteFunclet));
	el->appendInputData16(static_cast<uint16_t>(ram->getStart() & 0xFFFF));
	el->appendInputData16(static_cast<uint16_t>(availableRam & 0xFFFF));
	el->appendInputData16(programStartAddress);
	el->appendInputData32(alignedData.alignedAddress);
	el->appendInputData32( (static_cast<uint32_t>(count) + alignedData.frontPadding + alignedData.backPadding)/2 );
	el->appendInputData16(0);
	el->appendInputData16(flags);
	el->appendInputData16(devHandle->getClockCalibration()->getCal0());
	el->appendInputData16(devHandle->getClockCalibration()->getCal1());

	for (int i = 0; i < alignedData.frontPadding; ++i)
	{
		el->appendInputData8(0xff);
	}

	for (size_t i = 0; i < count; ++i)
	{
		el->appendInputData8(static_cast<uint8_t>(buffer[i]));
	}

	for (int i = 0; i < alignedData.backPadding; ++i)
	{
		el->appendInputData8(0xff);
	}

	this->elements.emplace_back(el);

	funcletToUpload = FuncletCode::WRITE;

	return true;
}


bool FlashMemoryAccessBase::preSync()
{
	bool success = true;

	if (funcletToUpload != FuncletCode::NONE)
	{
		if ( !devHandle->getClockCalibration()->backupSettings() )
			return false;

		if ( !devHandle->getClockCalibration()->determineSettings() )
			return false;

		if ( !devHandle->getClockCalibration()->makeSettings() )
			return false;

		success = uploadFunclet(funcletToUpload);
		funcletToUpload = FuncletCode::NONE;
	}

	return success;
}


bool FlashMemoryAccessBase::postSync(const HalExecCommand& cmd)
{
	devHandle->getClockCalibration()->restoreSettings();

	return MainMemoryAccessBase::postSync(cmd);
}


MemoryAreaBase::Alignment FlashMemoryAccessBase::alignData(uint32_t address, uint32_t count) const
{
	const uint32_t alignedAddress = address & 0xfffffffc;
	const int frontPadding = address - alignedAddress;
	const int stubble = (address + count) % 4;
	const int backPadding = (4 - stubble) % 4;

	return Alignment(alignedAddress, frontPadding, backPadding);
}


FlashMemoryAccess2ByteAligned::FlashMemoryAccess2ByteAligned(
				MemoryArea::Name name,
				IDeviceHandle* devHandle,
				uint32_t start,
				uint32_t end,
				uint32_t seg,
				uint32_t banks,
				bool mapped,
				const bool isProtected,
				IMemoryManager* mm,
				uint8_t psa
)
: FlashMemoryAccessBase(name, devHandle, start, end, seg, banks, mapped, isProtected, mm, psa)
{
}


MemoryAreaBase::Alignment FlashMemoryAccess2ByteAligned::alignData(uint32_t address, uint32_t count) const
{
	const uint32_t alignedAddress = address & 0xfffffffe;
	const int frontPadding = address - alignedAddress;
	const int stubble = (address + count) % 2;
	const int backPadding = (stubble > 0) ? 1 : 0;

	return Alignment(alignedAddress, frontPadding, backPadding);
}
