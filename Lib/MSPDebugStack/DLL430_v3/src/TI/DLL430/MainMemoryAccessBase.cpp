/*
 * MainMemoryAccessBase.cpp
 *
 * Base class for Flash and FRAM memory access
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
#include "MainMemoryAccessBase.h"
#include "HalExecCommand.h"
#include "IDeviceHandle.h"
#include "CpuRegisters.h"

using namespace TI::DLL430;

MainMemoryAccessBase::MainMemoryAccessBase (
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
 : MemoryAreaBase(name, devHandle, start, end, seg, banks, mapped, isProtected, psa)
 , mm(mm)
{
}

bool MainMemoryAccessBase::doRead(uint32_t address, uint8_t* buffer, size_t count)
{
	const hal_id readMacro = devHandle->supportsQuickMemRead() ? ID_ReadMemQuick : ID_ReadMemWords;
	return defaultRead(readMacro, mm, address, buffer, count);
}


bool MainMemoryAccessBase::doWrite(uint32_t address, uint32_t value)
{
	return this->doWrite(address, (uint8_t*)&value, 1);
}


bool MainMemoryAccessBase::erase(uint32_t start, uint32_t end, bool forceUnlock)
{
	return erase(start, end, this->getSegmentSize(), ERASE_SEGMENT, forceUnlock);
}


bool MainMemoryAccessBase::erase()
{
	const uint32_t bank_size = this->getSize() / this->getBanks();
	return erase(this->getStart(), this->getEnd(), bank_size, ERASE_MAIN, true);
}


bool MainMemoryAccessBase::uploadFunclet(FuncletCode::Type type)
{
	bool success = false;

	const FuncletCode& funclet = devHandle->getFunclet(type);

	if ( !funclet.code() )
	{
		success = true;
		ramBackup.clear();
	}
	else if ( MemoryArea* ram = mm ? mm->getMemoryArea(MemoryArea::Ram, 0) : 0 )
	{
		if ( funclet.codeSize() <= ram->getSize() )
		{
			if ( mm && mm->getRamPreserveMode() )
			{
				const size_t backupSize = std::min((size_t)ram->getSize(),
										   funclet.codeSize() + funclet.maxPayloadSize());

				ramBackup.resize( backupSize );
				if (!ram->read(0, &ramBackup[0], ramBackup.size()) || !ram->sync())
					return false;
			}
			else
			{
				ramBackup.clear();
			}

			const uint8_t* code = (uint8_t*)funclet.code();
			const size_t count = funclet.codeSize();
			success = ram->write(0, code, count) && ram->sync();
		}
	}
	return success;
}


void MainMemoryAccessBase::restoreRam()
{
	if (!ramBackup.empty())
	{
		if ( MemoryArea* ram = mm->getMemoryArea(MemoryArea::Ram, 0) )
		{
			ram->write(0, &ramBackup[0], ramBackup.size()) && ram->sync();
		}
		ramBackup.clear();
	}
}

void MainMemoryAccessBase::restoreInfo()
{
	if (MemoryArea* info = mm ? mm->getMemoryArea(MemoryArea::Info, 0) : 0)
	{
		std::vector<uint8_t> cmpInfo;
		const size_t segmentSize = info->getSegmentSize();

		cmpInfo.resize(segmentSize);

		info->read(segmentSize * (info->getBanks() - 1), &cmpInfo[0], cmpInfo.size());
		info->sync();

		//Compare if the Info memroy content has changed during the erase. 
		//If yes restore the saved content
		if (!(std::equal(infoBackup.begin(), infoBackup.end(), cmpInfo.begin())))
		{
			// 0 is the offset to the start of the info memory of the current connected SoC. 
			info->write(segmentSize * (info->getBanks() - 1), &infoBackup[0], segmentSize);
		}
		cmpInfo.clear();
	}	
	clearBackupInfo();
}

void MainMemoryAccessBase::clearBackupInfo()
{
	infoBackup.clear();
}

bool MainMemoryAccessBase::backupInfo()
{	
	clearBackupInfo();

	if (mm)
	{
		if (MemoryArea* info = mm ? mm->getMemoryArea(MemoryArea::Info, 0) : 0)
		{
			//only backup the last Info segment that holds the TLV data
			const size_t segmentSize = info->getSegmentSize();
			infoBackup.resize(segmentSize);
			if (!info->read(segmentSize * (info->getBanks() -1), &infoBackup[0], segmentSize) || !info->sync())
			{
				return false;
			}
		}
	}
	return true;
}

bool MainMemoryAccessBase::postSync(const HalExecCommand& cmd)
{
	restoreRam();
	return MemoryAreaBase::postSync(cmd);
}
