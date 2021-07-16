/*
 * LockableRamMemoryAccess.cpp
 *
 * Handles access to lockable ram memory.
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
#include "LockableRamMemoryAccess.h"
#include "CpuRegisters.h"
#include "HalExecCommand.h"
#include "IDeviceHandle.h"

using namespace TI::DLL430;
using std::bind;
using std::shared_ptr;

LockableRamMemoryAccess::LockableRamMemoryAccess
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
				uint8_t psa
)
 : RandomMemoryAccess(name, devHandle, start, end, seg, banks, mapped, isProtected, mm, psa)
 , unlockBeforeSync(false), lockState(2, 0)
{
}

bool LockableRamMemoryAccess::erase(uint32_t start, uint32_t end, uint32_t block_size, int type)
{
	const size_t size = end - start + 1;
	const std::vector<uint8_t> eraseDummyBuffer(size, 0xFF);

	return write(start - getStart(), &eraseDummyBuffer[0], size) && sync();
}

bool LockableRamMemoryAccess::doRead(uint32_t address, uint8_t* buffer, size_t count)
{
	const hal_id readMacro = devHandle->supportsQuickMemRead() ? ID_ReadMemQuick : ID_ReadMemWords;
	return defaultRead(readMacro, mm, address, buffer, count);
}

bool LockableRamMemoryAccess::doWrite(uint32_t address, const uint8_t* buffer, size_t count)
{
	const bool success = RandomMemoryAccess::doWrite(address, buffer, count);

	if (success)
	{
		unlockBeforeSync = true;
	}
	return success;
}

bool LockableRamMemoryAccess::doWrite(uint32_t address, uint32_t value)
{
	return this->doWrite(address, (uint8_t*)&value, 1);
}

bool LockableRamMemoryAccess::erase(uint32_t start, uint32_t end, bool forceUnlock)
{
	return erase(start, end, this->getSegmentSize(), 0);
}

bool LockableRamMemoryAccess::erase()
{
	uint32_t bank_size=this->getSize()/this->getBanks();

	return erase(this->getStart(), this->getEnd(), bank_size, 1);
}

bool LockableRamMemoryAccess::preSync()
{
	bool success = true;

	if ( unlockBeforeSync )
	{
		success = false;
		int16_t index = mm->getMemoryAreaIndex(MemoryArea::Peripheral16bit, 0x0190, 0x2);
		if (index < 0)
		{
			return false;
		}
		MemoryArea* peripheral16bit = mm->getMemoryArea(MemoryArea::Peripheral16bit, index);
		if (peripheral16bit &&
			peripheral16bit->read(0x0190 - peripheral16bit->getStart(), &lockState[0], 2) &&
			peripheral16bit->sync())
		{
			const uint8_t tmp[2] = { 0, static_cast<uint8_t>(lockState[1] & 0x08) };
			success = peripheral16bit->write(0x0190 - peripheral16bit->getStart(), tmp, 2) && peripheral16bit->sync();
		}
	}
	return success;
}


bool LockableRamMemoryAccess::postSync (const HalExecCommand& cmd)
{
	if ( unlockBeforeSync )
	{
		MemoryArea* peripheral16bit = mm->getMemoryArea(MemoryArea::Peripheral16bit);
		peripheral16bit->write(0x0190 - peripheral16bit->getStart(), &lockState[0], 2);
		peripheral16bit->sync();
		unlockBeforeSync = false;
	}

	return MemoryAreaBase::postSync(cmd);
}
