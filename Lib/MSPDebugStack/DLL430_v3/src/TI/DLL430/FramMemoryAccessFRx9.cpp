/*
 * FlashMemoryAccessFRx9.cpp
 *
 * Memory class for accessing fram memory on FR59xx/FR69xx.
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
#include "FramMemoryAccessFRx9.h"
#include "HalExecCommand.h"
#include "IDeviceHandle.h"
#include "IConfigManager.h"
#include "MpuFRx.h"
#include "EM/EmulationManager/IEmulationManager.h"


using namespace TI::DLL430;

FramMemoryAccessFRx9::FramMemoryAccessFRx9
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
:FramMemoryAccessBase(name, devHandle, start, end, seg, banks, mapped, isProtected, mm, psa, writeProt, mpu)
{}

bool FramMemoryAccessFRx9::erase(uint32_t start, uint32_t end, uint32_t block_size, int type, bool forceUnlock)
{
	MemoryArea* main = this->mm->getMemoryArea(MemoryArea::Main, 0);
	if (!main)
	{
		return false;
	}

	if ((type == ERASE_MAIN) && (start == main->getStart()))
	{
		HalExecCommand cmd;
		cmd.setTimeout(10000);	// overwrite 3 sec default with 10 sec

		HalExecElement* el = new HalExecElement(this->devHandle->checkHalId(ID_SendJtagMailboxXv2));
		el->appendInputData16(LONG_MAILBOX_MODE);	// Mailbox Mode
		el->appendInputData16(STOP_DEVICE);			// Data 1 Mailbox
		el->appendInputData16(MAIN_ERASE_MODE);		// Data 2 Mailbox
		cmd.elements.emplace_back(el);

		if (!this->devHandle->send(cmd))
		{
			return false;
		}

		this->devHandle->getEmulationManager()->rewriteConfiguration();

		return this->devHandle->reset();
	}

	return FramMemoryAccessBase::erase(start, end, block_size, type, forceUnlock);
}