/*
 * ArmSpecialMemoryAccess.cpp
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
#include "ArmFlashMemoryAccess.h"
#include "ArmSpecialFlashMemoryAccess.h"
#include "HalExecCommand.h"
#include "IDeviceHandle.h"
#include "FetHandle.h"
#include "ClockCalibration.h"
#include "MSP432_FlashLib.h"
#include "CpuRegisters.h"

using namespace TI::DLL430;
using std::vector;
using std::bind;
using std::shared_ptr;

ArmInfoFlashMemoryAccess::ArmInfoFlashMemoryAccess(
	MemoryArea::Name name,
	IDeviceHandle* devHandle,
	uint32_t start,
	uint32_t size,
	uint32_t seg,
	uint32_t banks,
	bool mapped,
	const bool isProtected,
	IMemoryManager* mm,
	uint8_t psa
	)
	: ArmFlashMemoryAccess(name, devHandle, start, size, seg, banks, mapped, isProtected, mm, psa, static_cast<IWriteProtection*>(new NoWriteProtection(mm)))
	, start(start)
	, end(start + size - 1)
{}

ArmInfoFlashMemoryAccess::~ArmInfoFlashMemoryAccess() {};


bool ArmInfoFlashMemoryAccess::erase()
{
	return ArmFlashMemoryAccess::erase(start, end);
}

ArmBslFlashMemoryAccess::ArmBslFlashMemoryAccess(
				MemoryArea::Name name,
				IDeviceHandle* devHandle,
				uint32_t start,
				uint32_t size,
				uint32_t seg,
				uint32_t banks,
				bool mapped,
				const bool isProtected,
				IMemoryManager* mm,
				uint8_t psa
			)
			: ArmFlashMemoryAccess(name, devHandle, start, size, seg, banks, mapped, isProtected, mm, psa, static_cast<IWriteProtection*>(new NoWriteProtection(mm)))
			, start(start)
			, end(start + size - 1)
{}

ArmBslFlashMemoryAccess::~ArmBslFlashMemoryAccess()
{}


bool ArmBslFlashMemoryAccess::erase()
{
	if (!isLocked())
	{
		return ArmFlashMemoryAccess::flashErase(ArmBslFlashMemoryAccess::start, ArmBslFlashMemoryAccess::end,
			true, MSP432_FlashLib::FLASH_SECTOR_ERASE);
	}
	return true;
}

bool ArmBslFlashMemoryAccess::erase(uint32_t start, uint32_t end, bool forceUnlock)
{	
	if (!isLocked())
	{
		return ArmFlashMemoryAccess::flashErase(start, end, true, MSP432_FlashLib::FLASH_SECTOR_ERASE);
	}
	return true;
}

bool ArmBslFlashMemoryAccess::doWrite(uint32_t address, const uint8_t* buffer, size_t count)
{
	if (!isLocked())
	{
		return ArmFlashMemoryAccess::flashWrite(address, buffer, count, true);
	}
	return true;
}

ArmFlashMemoryAccess2M::ArmFlashMemoryAccess2M(
	MemoryArea::Name name,
	IDeviceHandle* devHandle,
	uint32_t start,
	uint32_t size,
	uint32_t seg,
	uint32_t banks,
	bool mapped,
	const bool isProtected,
	IMemoryManager* mm,
	uint8_t psa,
	IWriteProtection *writeProt
	)
	: ArmFlashMemoryAccess(name, devHandle, start, size, seg, banks, mapped, isProtected, mm, psa, writeProt)
{}

ArmFlashMemoryAccess2M::~ArmFlashMemoryAccess2M()
{}


bool ArmFlashMemoryAccess2M::wakeup()
{
	uint8_t timeout = 6;

	uint32_t pcmCtlData = 0;
	uint8_t tmpPcm[4] = { 0 };

	IDebugManager *dbm = devHandle->getDebugManager();
	if (!dbm)
	{
		return false;
	}

	mm->read(MSP432_FlashLib::PCM_CTL, tmpPcm, sizeof(pcmCtlData));
	mm->sync();
	pcmCtlData = tmpPcm[0] | (tmpPcm[1] << 8) | (tmpPcm[2] << 16) | (tmpPcm[3] << 24);

	pcmCtlData = MSP432_FlashLib::PCM_CTL0_KEY | ((pcmCtlData & 0xFFFF) & ~(MSP432_FlashLib::PCM_CTL1_LOCKLPM5 + MSP432_FlashLib::PCM_CTL1_LOCKBKUP));

	mm->write(MSP432_FlashLib::PCM_CTL, pcmCtlData);
	mm->sync();
		
	uint32_t sramEn = 0;
	uint8_t sramEnTmp[4] = { 0 };
	do
	{
		mm->read(MSP432_FlashLib::SYS_SRAM_STAT, sramEnTmp, sizeof(sramEn));
		mm->sync();
		sramEn = sramEnTmp[0] | (sramEnTmp[1] << 8) | (sramEnTmp[2] << 16) | (sramEnTmp[3] << 24);
	} while (!(sramEn & MSP432_FlashLib::BNKEN_RDY) && --timeout); 

	if (!timeout)
	{
		return false;
	}

	uint32_t sramBanks = 0;
	uint8_t sramBanksTmp[4] = { 0 };

	mm->read(MSP432_FlashLib::SYS_SRAM_NUMBANKS, sramBanksTmp, sizeof(sramBanks));
	mm->sync();
	sramBanks = sramBanksTmp[0] | (sramBanksTmp[1] << 8) | (sramBanksTmp[2] << 16) | (sramBanksTmp[3] << 24);

	if (sramBanks == 4)
	{
		mm->write(MSP432_FlashLib::SYS_SRAM_BNKEN_CTL0, MSP432_FlashLib::BNK3_EN);
		mm->sync();
	}
	else if (sramBanks == 2)
	{
		mm->write(MSP432_FlashLib::SYS_SRAM_BNKEN_CTL0, MSP432_FlashLib::BNK1_EN);
		mm->sync();
	}
	else
	{
		return false;
	}
	return true;
}

ArmBslFlashMemoryAccess2M::ArmBslFlashMemoryAccess2M(
	MemoryArea::Name name,
	IDeviceHandle* devHandle,
	uint32_t start,
	uint32_t size,
	uint32_t seg,
	uint32_t banks,
	bool mapped,
	const bool isProtected,
	IMemoryManager* mm,
	uint8_t psa
	)
	: ArmFlashMemoryAccess2M(name, devHandle, start, size, seg, banks, mapped, isProtected, mm, psa, static_cast<IWriteProtection*>(new NoWriteProtection(mm))),
	start(start),
	end(start + size - 1)
{}

ArmBslFlashMemoryAccess2M::~ArmBslFlashMemoryAccess2M()
{}

bool ArmBslFlashMemoryAccess2M::erase()
{
	if (!isLocked())
	{
		return ArmFlashMemoryAccess2M::flashErase(ArmBslFlashMemoryAccess2M::start, ArmBslFlashMemoryAccess2M::end,
			true, MSP432_FlashLib::FLASH_SECTOR_ERASE);
	}
	return true;
}

bool ArmBslFlashMemoryAccess2M::erase(uint32_t start, uint32_t end, bool forceUnlock)
{
	if (!isLocked())
	{
		return ArmFlashMemoryAccess2M::flashErase(start, end, true, MSP432_FlashLib::FLASH_SECTOR_ERASE);
	}
	return true;
}

bool ArmBslFlashMemoryAccess2M::doWrite(uint32_t address, const uint8_t* buffer, size_t count)
{
	if (!isLocked())
	{
		return ArmFlashMemoryAccess2M::flashWrite(address, buffer, count, true);
	}
	return true;
}

ArmInfoFlashMemoryAccess2M::ArmInfoFlashMemoryAccess2M(
	MemoryArea::Name name,
	IDeviceHandle* devHandle,
	uint32_t start,
	uint32_t size,
	uint32_t seg,
	uint32_t banks,
	bool mapped,
	const bool isProtected,
	IMemoryManager* mm,
	uint8_t psa
	)
	: ArmFlashMemoryAccess2M(name, devHandle, start, size, seg, banks, mapped, isProtected, mm, psa, static_cast<IWriteProtection*>(new NoWriteProtection(mm)))
	, start(start)
	, end(start + size - 1)
{}

ArmInfoFlashMemoryAccess2M::~ArmInfoFlashMemoryAccess2M() {};


bool ArmInfoFlashMemoryAccess2M::erase()
{
	return ArmFlashMemoryAccess2M::erase(start, end);
}



