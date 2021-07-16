/*
 * ArmFlashMemoryAccess.cpp
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
#include "HalExecCommand.h"
#include "IDeviceHandle.h"
#include "FetHandle.h"
#include "ClockCalibration.h"
#include "MSP432_FlashLib.h"
#include "CpuRegisters.h"
#include "warnings/Warnings.h"


using namespace TI::DLL430;
using std::vector;
using std::bind;
using std::shared_ptr;

ArmFlashMemoryAccess::ArmFlashMemoryAccess (
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
 : ArmRandomMemoryAccess(name, devHandle, start, size, seg, banks, mapped, isProtected, mm, psa),
 mSegmentSize(seg),
 mWriteProt(writeProt)
{
}

ArmFlashMemoryAccess::~ArmFlashMemoryAccess()
{
}


bool ArmFlashMemoryAccess::flashExit()
{
	CpuRegisters* cpu = mm->getCpuRegisters();
	IDebugManager *dbm = devHandle->getDebugManager();
	if (!cpu || !dbm)
	{
		return false;
	}
	mm->write(MSP432_FlashLib::FLASH_FUNCTION_ADDRESS, MSP432_FlashLib::FLASH_EXIT);
	mm->sync();

	uint32_t status = 0;
	do
	{
		uint8_t tmp[4] = { 0 };
		mm->read(MSP432_FlashLib::RETURN_CODE_ADDRESS, tmp, sizeof(status)) && mm->sync();
		status = tmp[0] | (tmp[1] << 8) | (tmp[2] << 16) | (tmp[3] << 24);
	} while (status == MSP432_FlashLib::FLASH_BUSY);

	// halt CPU
	if (!dbm->stop())
	{
		return false;
	}

	restoreRam();
	cpu->popCache();

	if (dbm->getLpmDebugging())
	{
		devHandle->getFetHandle()->getConfigManager()->setUlpDebug(true);
	}

	return (status == MSP432_FlashLib::FLASH_SUCCESS);
}

bool ArmFlashMemoryAccess::wakeup()
{
	uint32_t pcmCtlData = 0;
	uint8_t tmpPcm[4] = { 0 };
	uint8_t timeout = 6;

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
		mm->read(MSP432_FlashLib::SYS_SRAM_BANKEN, sramEnTmp, sizeof(sramEn));
		mm->sync();
		sramEn = sramEnTmp[0] | (sramEnTmp[1] << 8) | (sramEnTmp[2] << 16) | (sramEnTmp[3] << 24);
	} while (!(sramEn & MSP432_FlashLib::SRAM_RDY) && --timeout);

	if (!timeout)
	{
		return false;
	}

	if (!(sramEn & MSP432_FlashLib::SRAM_EN))
	{
		mm->write(MSP432_FlashLib::SYS_SRAM_BANKEN, MSP432_FlashLib::SRAM_EN);
		mm->sync();
	}
	return true;
}

bool ArmFlashMemoryAccess::isProtectionEnabled()
{
	return mWriteProt->isEnabled();
}

bool ArmFlashMemoryAccess::flashInit()
{
	CpuRegisters* cpu = mm->getCpuRegisters();
	IDebugManager *dbm = devHandle->getDebugManager();
	if (!cpu || !dbm)
	{
		return false;
	}

	cpu->pushCache();
	cpu->disableInterrupts();
		
	devHandle->getFetHandle()->getConfigManager()->setUlpDebug(false);

	cpu->switchContext(MSP432_FlashLib::RAM_LOADER_MAIN, MSP432_FlashLib::RAM_LOADER_STACK);
	/* making PREMASK effective requires a single step */
	if (!dbm->singleStep())
	{
		return false;
	}

	mm->write(MSP432_FlashLib::FLASH_FUNCTION_ADDRESS, MSP432_FlashLib::FLASH_INIT);
	cpu->switchContext(MSP432_FlashLib::RAM_LOADER_MAIN, MSP432_FlashLib::RAM_LOADER_STACK);
	mm->sync();

	if (!dbm->run(0))
	{
		return false;
	}

	uint32_t status = 0;
	do
	{
		uint8_t tmp[4] = { 0 };
		mm->read(MSP432_FlashLib::RETURN_CODE_ADDRESS, tmp, sizeof(status)) && mm->sync();
		status = tmp[0] | (tmp[1] << 8) | (tmp[2] << 16) | (tmp[3] << 24);
	} while (status == MSP432_FlashLib::FLASH_BUSY);

	// clear RETURN_CODE_ADDRESS value
	mm->write(MSP432_FlashLib::RETURN_CODE_ADDRESS, 0x0000);
	mm->sync();

	return (status == MSP432_FlashLib::FLASH_SUCCESS);
}


bool ArmFlashMemoryAccess::flashErase(uint32_t start, uint32_t end, bool unlockBsl, uint32_t mode)
{
	CpuRegisters* cpu = mm->getCpuRegisters();
	IDebugManager *dbm = devHandle->getDebugManager();
	if (!cpu || !dbm)
	{
		return false;
	}
	if (!mm->checkMinFlashVoltage())
	{
		WarningFactory::instance()->message(MESSAGE_LEVEL_T::MSPDS_MESSAGE_LEVEL_WARNING, WarningCode::WARNING_FLASH_VCC);
		return false;
	}
	// halt CPU
	if (!dbm->stop())
	{
		return false;
	}
	if (!wakeup())
	{
		return false;
	}

	if (!uploadFunclet())
	{
		return false;
	}
	if (!flashInit())
	{
		return false;
	}
	if (unlockBsl)
	{
		mm->write(MSP432_FlashLib::UNLOCK_BSL_ADDRESS, MSP432_FlashLib::UNLOCK_BSL_KEY);
	}

	uint32_t status = 0;

	if (mode == MSP432_FlashLib::FLASH_MASS_ERASE)
	{		
		mm->write(MSP432_FlashLib::ERASE_PARAM_ADDRESS, MSP432_FlashLib::FLASH_MASS_ERASE);
		mm->write(MSP432_FlashLib::FLASH_FUNCTION_ADDRESS, MSP432_FlashLib::FLASH_MASS_ERASE);
		mm->sync();

		status = 0;
		do
		{
			uint8_t tmp[4] = { 0 };
			mm->read(MSP432_FlashLib::RETURN_CODE_ADDRESS, tmp, sizeof(status)) && mm->sync();
			status = tmp[0] | (tmp[1] << 8) | (tmp[2] << 16) | (tmp[3] << 24);
		} while (status == MSP432_FlashLib::FLASH_BUSY);

	}
	if (mode == MSP432_FlashLib::FLASH_SECTOR_ERASE)
	{
		for (uint32_t sectorStartAddr = (start & 0xFFFFF000); sectorStartAddr <= end; )
		{
			mm->write(MSP432_FlashLib::DST_ADDRESS, sectorStartAddr);
			mm->write(MSP432_FlashLib::FLASH_FUNCTION_ADDRESS, MSP432_FlashLib::FLASH_SECTOR_ERASE);
			mm->sync();

			status = 0;
			do
			{
				uint8_t tmp[4] = { 0 };
				mm->read(MSP432_FlashLib::RETURN_CODE_ADDRESS, tmp, sizeof(status)) && mm->sync();
				status = tmp[0] | (tmp[1] << 8) | (tmp[2] << 16) | (tmp[3] << 24);
			} while (status == MSP432_FlashLib::FLASH_BUSY);

			sectorStartAddr += getSegmentSize();
		}
	}

	if (!flashExit())
	{
		return false;
	}
	return (status == MSP432_FlashLib::FLASH_SUCCESS);
}

bool ArmFlashMemoryAccess::flashWrite(uint32_t address, const uint8_t* buffer, size_t count, bool unlockBsl)
{
	CpuRegisters* cpu = mm->getCpuRegisters();
	IDebugManager *dbm = devHandle->getDebugManager();

	address += this->getStart();

	if (!cpu || !dbm)
	{
		return false;
	}
	if (!mm->checkMinFlashVoltage())
	{
		WarningFactory::instance()->message(MESSAGE_LEVEL_T::MSPDS_MESSAGE_LEVEL_WARNING, WarningCode::WARNING_FLASH_VCC);
		return false;
	}

	if (!dbm->stop())
	{
		return false;
	}

	if (!wakeup())
	{
		return false;
	}

	if (!uploadFunclet())
	{
		return false;
	}

	if (!flashInit())
	{
		return false;
	}

	volatile uint32_t bytesToWrite = static_cast<uint32_t>(count);
	volatile uint32_t bytesIndex = 0;
	volatile uint32_t status = MSP432_FlashLib::FLASH_SUCCESS;
	volatile uint32_t bCtlRegister1 = 0, bCtlRegister2 = 0;

	if (unlockBsl)
	{
		mm->write(MSP432_FlashLib::UNLOCK_BSL_ADDRESS, MSP432_FlashLib::UNLOCK_BSL_KEY);
	}
	uint8_t tmp[4] = { 0 };
	uint8_t tmp1[4] = { 0 };

	mm->write(MSP432_FlashLib::DST_ADDRESS, address);
	mm->write(MSP432_FlashLib::SRC_LENGTH_ADDRESS, bytesToWrite);
	mm->write(MSP432_FlashLib::FLASH_FUNCTION_ADDRESS, MSP432_FlashLib::FLASH_CONTIOUS_PROGRAM);

	while ((status == MSP432_FlashLib::FLASH_SUCCESS) && (bytesIndex < static_cast<uint32_t>(count)))
	{
		mm->read(MSP432_FlashLib::BUFFER1_STATUS_REGISTER, tmp, sizeof(bCtlRegister1));		
		mm->read(MSP432_FlashLib::BUFFER2_STATUS_REGISTER, tmp1, sizeof(bCtlRegister2)) && mm->sync();

		bCtlRegister1 = tmp[0] | (tmp[1] << 8) | (tmp[2] << 16) | (tmp[3] << 24);		
		bCtlRegister2 = tmp1[0] | (tmp1[1] << 8) | (tmp1[2] << 16) | (tmp1[3] << 24);

		if (!(bCtlRegister1 & MSP432_FlashLib::BUFFER_ACTIVE) && (bytesIndex < static_cast<uint32_t>(count)))
		{
			//Use Buffer 1
			// Do not copy more than SRC_LENGTH_MAX bytes to RAM loader buffer
			if (static_cast<uint32_t>(count) - bytesIndex > MSP432_FlashLib::SRC_LENGTH_MAX)
			{
				bytesToWrite = MSP432_FlashLib::SRC_LENGTH_MAX;
			}
			else
			{
				bytesToWrite = static_cast<uint32_t>(count) - bytesIndex;
			}		

			mm->write(MSP432_FlashLib::RAM_LOADER_BUFFER1, buffer + bytesIndex, bytesToWrite);
			mm->write(MSP432_FlashLib::BUFFER1_STATUS_REGISTER, MSP432_FlashLib::BUFFER_DATA_READY);
			mm->sync();

			bytesIndex += bytesToWrite;
		}
		
		if (!(bCtlRegister2 & MSP432_FlashLib::BUFFER_ACTIVE) && (bytesIndex < static_cast<uint32_t>(count)))
		{
			//Use Buffer 2

			// Do not copy more than SRC_LENGTH_MAX bytes to RAM loader buffer
			if (static_cast<uint32_t>(count) - bytesIndex > MSP432_FlashLib::SRC_LENGTH_MAX)
			{
				bytesToWrite = MSP432_FlashLib::SRC_LENGTH_MAX;
			}
			else
			{
				bytesToWrite = static_cast<uint32_t>(count) - bytesIndex;
			}
			
			mm->write(MSP432_FlashLib::RAM_LOADER_BUFFER2, buffer + bytesIndex, bytesToWrite);
			mm->write(MSP432_FlashLib::BUFFER2_STATUS_REGISTER, MSP432_FlashLib::BUFFER_DATA_READY);
			mm->sync();
			
			bytesIndex += bytesToWrite;
		}
	}
	if (!flashExit())
	{
		return false;
	}
	if (status == MSP432_FlashLib::FLASH_SUCCESS)
	{
		return true;
	}
	return false;
}

bool ArmFlashMemoryAccess::erase()
{
	return flashErase(0, 0, false, MSP432_FlashLib::FLASH_MASS_ERASE);
}

bool ArmFlashMemoryAccess::erase(uint32_t start, uint32_t end, bool forceUnlock)
{
	 return flashErase(start, end, false, MSP432_FlashLib::FLASH_SECTOR_ERASE);
}

bool ArmFlashMemoryAccess::doWrite(uint32_t address, const uint8_t* buffer, size_t count)
{
	return flashWrite(address, buffer, count, false);
}
