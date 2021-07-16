/*
 * MpuFRx.cpp
 *
 * Functionality for configuring MPU.
 *
 * Copyright (C) 2007 - 2014 Texas Instruments Incorporated - http://www.ti.com/
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
#include "ConfigManager.h"
#include "IDeviceHandle.h"
#include "FetHandle.h"
#include "MpuFRx.h"

using namespace TI::DLL430;

MpuFRx::MpuFRx(IDeviceHandle* devHandle, IMemoryManager* mm)
	: devHandle(devHandle)
	, mm(mm)
	, MPUCTL0(0)
	, MPUCTL0_original(0)
{
}


bool MpuFRx::disableIfEnabled(bool killLockBit)
{
	if (MPUCTL0_original != MPUCTL0)
	{
		return false;
	}
	if (!readSettings())
	{
		return false;
	}
	return (MPUCTL0 & 0x1) ? disable(killLockBit) : true;
}


bool MpuFRx::readSettings()
{
	// read out  mpu settings

	int16_t index = mm->getMemoryAreaIndex(MemoryArea::Peripheral16bit, MPUCTL0_Address, 0x2);
	if (index < 0)
	{
		return false;
	}
	MemoryArea* peripheral = mm->getMemoryArea(MemoryArea::Peripheral16bit, index);
	if (!peripheral)
	{
		return false;
	}
	uint8_t tmp[8] = {0};
	if (!peripheral->read(MPUCTL0_Address - peripheral->getStart(), tmp, 8) || !peripheral->sync())
	{
		if (!peripheral->read(MPUCTL0_Address - peripheral->getStart(), tmp, 8) || !peripheral->sync())
		{
			return false;
		}
	}

	this->MPUCTL0 = framCtlKey | tmp[0];
	MPUCTL0_original = MPUCTL0;

	return true;
}

bool MpuFRx::disable(bool killLockBit)
{
	// Check if lock bit of MPU is set
	if ((MPUCTL0 & 0x2) == 0x2)
	{
		if (!killLockBit)
		{
			//Don't do anything
			return true;
		}

		if (!removeLockBit())
		{
			return false;
		}
	}
	int16_t index = mm->getMemoryAreaIndex(MemoryArea::Peripheral16bit, MPUCTL0_Address, 0x2);
	if (index < 0)
	{
		return false;
	}
	MemoryArea* peripheral = mm->getMemoryArea(MemoryArea::Peripheral16bit, index);
	if (!peripheral)
	{
		return false;
	}
	// mpu ist not locked (anymore), disable it set Mpuena = 0
	// write also Fram MPUCTL0 key also into register to gain access
	if (!peripheral->write(MPUCTL0_Address - peripheral->getStart(), framCtlKey) || !peripheral->sync())
	{
		return false;
	}

	const uint16_t backup = MPUCTL0_original;

	this->readSettings();

	MPUCTL0_original = backup;

	return (MPUCTL0 & 0x1) == 0;
}

bool MpuFRx::removeLockBit()
{
	IConfigManager* cm = this->devHandle->getFetHandle()->getConfigManager();

	// assert hard RST/NMI and feed in magic pattern to stop device execution
	// thr RST/NMI will remove the register protection
	if (!cm->reset(false, true, devHandle->getJtagId(), devHandle->checkHalId(ID_ResetXv2)))
	{
		return false;
	}
	// restart jtag connection and if needed feed in JTAG password
	if (cm->start() != 0x1)
	{
		return false;
	}
	return devHandle->reset();
}

void MpuFRx::restore()
{
	if (MPUCTL0_original != MPUCTL0)
	{
		int16_t index = mm->getMemoryAreaIndex(MemoryArea::Peripheral16bit, MPUCTL0_Address, 0x2);
		if (index >= 0)
		{
			if (MemoryArea* peripheral = mm->getMemoryArea(MemoryArea::Peripheral16bit, index))
			{
				if (peripheral->write(MPUCTL0_Address - peripheral->getStart(), MPUCTL0_original) && peripheral->sync())
				{
					MPUCTL0 = MPUCTL0_original;
				}
			}
		}
	}
}
