/*
 * DebugManagerArm.cpp
 *
 * Functionality for debugging target device.
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
#include "DebugManagerArm.h"
#include "DeviceHandleArm.h"
#include "HalExecCommand.h"
#include "PollingManager.h"
#include "CpuRegisters.h"

using namespace TI::DLL430;

/** \brief manage debug actions on the target device */
DebugManagerArm::DebugManagerArm(IDeviceHandle* deviceHandle, const DeviceInfo& devInfo)
	: mDeviceHandle(deviceHandle),
	mPollingManager(nullptr),
	mCallback(nullptr),
	ulpDebug(false)
{
}

DebugManagerArm::~DebugManagerArm()
{
	if (mDeviceHandle->getFetHandle() && mPollingManager)
	{
		mPollingManager->stopBreakpointPolling(*this->mDeviceHandle);
		mPollingManager->setBreakpointCallback(PollingManager::Callback());

		mPollingManager->stopStateStoragePolling(*this->mDeviceHandle);
		mPollingManager->setStateStorageCallback(PollingManager::Callback());

		mPollingManager->stopLpmPolling(*this->mDeviceHandle);
		mPollingManager->setLpmCallback(PollingManager::Callback());
	}
}

bool DebugManagerArm::reconnectJTAG()
{
	return false;
}

bool DebugManagerArm::run(uint16_t controlType, DebugEventTarget* target, bool releaseJTAG)
{
	bool retVal = false;

	mCallback = target;

	IMemoryManager *mm = mDeviceHandle->getMemoryManager();

	CpuRegisters* cpu = mm->getCpuRegisters();
	cpu->flushCache();

	HalExecCommand cmd;

	HalExecElement* el = new HalExecElement(this->mDeviceHandle->checkHalId(ID_RunArm));
	el->appendInputData16(releaseJTAG ? 1 : 0);
	cmd.elements.emplace_back(el);

	if (this->mDeviceHandle->send(cmd))
	{
		if (releaseJTAG)
		{
			pausePolling();
		}
		else
		{
			this->resumePolling();
		}

		if (controlType)
		{
			retVal = mPollingManager && mPollingManager->startBreakpointPolling(*this->mDeviceHandle);
		}
		else
		{
			retVal = mPollingManager && mPollingManager->stopBreakpointPolling(*this->mDeviceHandle);
		}
	}

	return retVal;
}

bool DebugManagerArm::stop(bool jtagWasReleased)
{
	IMemoryManager *mm = mDeviceHandle->getMemoryManager();
	CpuRegisters *cpu = mm->getCpuRegisters();

	// Pause the polling loop while we try to wake up the device and sync again
	this->pausePolling();


	HalExecCommand cmd;
	cmd.elements.emplace_back(new HalExecElement(this->mDeviceHandle->checkHalId(ID_HaltArm)));
	

	if (mDeviceHandle->send(cmd))
	{
		return cpu->fillCache(0, 18);
	}

	return false;
}

bool DebugManagerArm::singleStep(uint32_t* cycles)
{
	IMemoryManager *mm = mDeviceHandle->getMemoryManager();
	CpuRegisters *cpu = mm->getCpuRegisters();

	cpu->flushCache();

	HalExecCommand cmd;
	cmd.elements.emplace_back(new HalExecElement(this->mDeviceHandle->checkHalId(ID_SingleStepArm)));

	if (mDeviceHandle->send(cmd))
	{
		return cpu->fillCache(0, 18);
	}

	return false;
}

uint8_t DebugManagerArm::getClockControl() const
{
	return 0;
}

uint16_t DebugManagerArm::getClockControlSetting() const
{
	return 0;
}

void DebugManagerArm::setClockControlSetting(uint16_t clkcntrl)
{
}

uint16_t DebugManagerArm::getGeneralClockDefaultSetting() const
{
	return 0;
}

uint32_t DebugManagerArm::getClockModuleDefaultSetting() const
{
	return 0;
}

uint32_t DebugManagerArm::getClockModuleSetting() const
{
	return 0;
}

void DebugManagerArm::setClockModuleSetting(uint32_t modules)
{
}

char** DebugManagerArm::getModuleStrings(uint32_t* n) const
{
	*n = 0;
	return nullptr;
}

char** DebugManagerArm::getClockStrings(uint32_t* n) const
{
	*n = 0;
	return nullptr;
}

bool DebugManagerArm::initEemRegister()
{
	return false;
}

bool DebugManagerArm::activatePolling(uint16_t mask)
{
	return false;
}

bool DebugManagerArm::activateJStatePolling(DebugEventTarget* cb)
{
	return false;
}

bool DebugManagerArm::queryIsInLpm5State()
{
	return false;
}

void DebugManagerArm::setOpcode(uint16_t value)
{
}

bool DebugManagerArm::saveContext()
{
	return false;
}

void DebugManagerArm::setLpmDebugging(bool enable)
{
	ulpDebug = enable;
}

bool DebugManagerArm::getLpmDebugging()
{
	return ulpDebug;
}

void DebugManagerArm::pausePolling()
{
	if (mPollingManager)
	{
		mPollingManager->pausePolling();
	}
}

void DebugManagerArm::resumePolling()
{
	if (mPollingManager)
	{
		mPollingManager->resumePolling();
	}
}

bool DebugManagerArm::syncDeviceAfterLPMx5()
{
	return false;
}

uint64_t DebugManagerArm::getCycleCounterValue()
{
	return 0;
}

void DebugManagerArm::resetCycleCounterValue()
{
}

bool DebugManagerArm::startStoragePolling()
{
	return false;
}

bool DebugManagerArm::stopStoragePolling()
{
	return false;
}

void DebugManagerArm::setPollingManager(PollingManager* pollingManager)
{
	this->mPollingManager = pollingManager;
	pollingManager->setBreakpointCallback(std::bind(&DebugManagerArm::runEvent, this, std::placeholders::_1));
}

void DebugManagerArm::enableLegacyCycleCounter(bool enable)
{
}

bool DebugManagerArm::legacyCycleCounterEnabled() const
{
	return false;
}

void DebugManagerArm::runEvent(MessageDataPtr messageData)
{
	uint32_t DHCSR = 0;
	(*messageData) >> DHCSR;

	if (DHCSR & 0x00020000)
	{ // Breakpoint hit

		IMemoryManager *mm = mDeviceHandle->getMemoryManager();
		CpuRegisters *cpu = mm->getCpuRegisters();

		cpu->fillCache(0, 18);

		if (mCallback)
		{
			mCallback->event(DebugEventTarget::BreakpointHit);
		}
	}
}
