/*
 * DebugManagerMSP430.cpp
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

#include "EemMemoryAccess.h"
#include "CpuRegisters.h"
#include "DebugManagerMSP430.h"
#include "IDeviceHandle.h"
#include "FetHandle.h"
#include "HalExecCommand.h"
#include "HalResponse.h"
#include "HalResponseHandler.h"
#include "FetControl.h"
#include "DeviceInfo.h"
#include "EM/EmulationManager/IEmulationManager.h"
#include "EM/Trace/ITrace.h"
#include "EM/SoftwareBreakpoints/ISoftwareBreakpoints.h"
#include "EM/Exceptions/Exceptions.h"
#include "MessageData.h"
#include "PinSequence.h"

using namespace TI::DLL430;

/// State modes.
enum INTERNAL_STATE_MODES {
	D_STOPPED = 0, /**< The device is stopped */
	D_RUNNING = 1, /**< The device is running or is being single stepped */
	D_SINGLE_STEP_COMPLETE = 2, /**< The device is stopped after the single step operation is complete */
	D_BREAKPOINT_HIT = 3, /**< The device is stopped as a result of hitting an enabled breakpoint */
	D_LPMX5_MODE = 4, /**< The device is in LPMx.5 low power mode  */
	D_LPMX5_WAKEUP = 5, /**<	The device woke up from LPMx.5 low power mode */
};

#define J_STATE_LPMX_MASK_HIGH 0x40000000ull

DebugManagerMSP430::DebugManagerMSP430(IDeviceHandle* parent, const DeviceInfo& devInfo)
	: parent(parent)
 	, clockControl(devInfo.clockInfo.clockControl)
	, genclkcntrl(DefaultClkCntrl)
 	, mclkcntrl0(devInfo.clockInfo.mclkCntrl0)
	, defaultGenClkCntrl(DefaultClkCntrl)
    , defaultMclkcntrl0(devInfo.clockInfo.mclkCntrl0)
	, emulationLevel(devInfo.eem)
	, moduleStrings(0)
	, nModuleStrings(0)
	, clockStrings(0)
	, nClockStrings(0)
	, lpmDebuggingEnabled(false)
	, lpm5WakeupDetected(false)
	, mdbPatchValue(0)
	, cbx(0)
 	, cycleCounter_(devInfo.eem, devInfo.bits)
	, storagePollingActive(false)
	, mPollingManager(nullptr)
	, irRequest(0)
{
	//Need to set TCE_SMCLK/TCE_SMCLK_MCLK for devices with standard clock control
	//to prevent a running SMCLK to be sourced from TCLK under debug (default)
	if (clockControl == GCC_STANDARD || clockControl == GCC_STANDARD_I)
	{
		defaultGenClkCntrl |= 0x1;
		genclkcntrl = defaultGenClkCntrl;
	}

	createModuleStrings(devInfo.clockInfo.eemTimers);
	createClockStrings(devInfo.clockInfo.eemClocks);
}

DebugManagerMSP430::~DebugManagerMSP430 ()
{
	if (parent->getFetHandle() && mPollingManager)
	{
		mPollingManager->stopBreakpointPolling(*this->parent);
		mPollingManager->setBreakpointCallback(PollingManager::Callback());

		mPollingManager->stopStateStoragePolling(*this->parent);
		mPollingManager->setStateStorageCallback(PollingManager::Callback());

		mPollingManager->stopLpmPolling(*this->parent);
		mPollingManager->setLpmCallback(PollingManager::Callback());
	}

	if (nullptr != moduleStrings)
	{
		for (uint32_t i = 0; i < nModuleStrings; ++i)
		{
			delete[] moduleStrings[i];
			moduleStrings[i] = nullptr;
		}
		delete[] moduleStrings;
		moduleStrings = nullptr;
	}

	if (nullptr != clockStrings)
	{
		for (uint32_t i = 0; i < nClockStrings; ++i)
		{
			delete[] clockStrings[i];
			clockStrings[i] = nullptr;
		}
		delete[] clockStrings;
		clockStrings = nullptr;
	}
}

void DebugManagerMSP430::createModuleStrings(const ClockMapping& clockMapping)
{
	nModuleStrings = static_cast<uint32_t>(clockMapping.size());
	moduleStrings = new char*[nModuleStrings];
	for (uint32_t i = 0; i < nModuleStrings; ++i)
	{
		size_t size = clockMapping[i].name.size() + 1;
		moduleStrings[i] = new char[size];
		memset(moduleStrings[i], 0, size);
		strncpy(moduleStrings[i], clockMapping[i].name.c_str(), size - 1);
	}
}

void DebugManagerMSP430::createClockStrings(const ClockNames& clockNames)
{
	nClockStrings = static_cast<uint32_t>(clockNames.size());
	clockStrings = new char*[nClockStrings];
	for (uint32_t i = 0; i < nClockStrings; ++i)
	{
		size_t size = clockNames[i].size() + 1;
		clockStrings[i] = new char[size];
		memset(clockStrings[i], 0, size);
		strncpy(clockStrings[i], clockNames[i].c_str(), size - 1);
	}
}

void DebugManagerMSP430::setPollingManager(PollingManager* pollingManager)
{
	this->mPollingManager = pollingManager;
	pollingManager->setBreakpointCallback(std::bind(&DebugManagerMSP430::runEvent, this, std::placeholders::_1));
	pollingManager->setLpmCallback(std::bind(&DebugManagerMSP430::runEvent, this, std::placeholders::_1));
	pollingManager->setStateStorageCallback(std::bind(&DebugManagerMSP430::runEvent, this, std::placeholders::_1));
}


bool DebugManagerMSP430::activatePolling(uint16_t mask)
{
	if (!mPollingManager)
	{
		return false;
	}
	return mPollingManager->startBreakpointPolling(*this->parent);
}


bool DebugManagerMSP430::activateJStatePolling(DebugEventTarget * cb)
{
	if (!mPollingManager)
	{
		return false;
	}
	mPollingManager->resumePolling();
	return mPollingManager->startLpmPolling(*this->parent);
}


void DebugManagerMSP430::enableLegacyCycleCounter(bool enable)
{
	const bool wasEnabled = cycleCounter_.isEnabled();
	cycleCounter_.enable(enable);

	if (enable && !wasEnabled)
	{
		cycleCounter_.configure();
	}
}


bool DebugManagerMSP430::startStoragePolling()
{
	if (!mPollingManager)
	{
		return false;
	}
	storagePollingActive = mPollingManager->startStateStoragePolling(*parent);
	return storagePollingActive;
}


bool DebugManagerMSP430::stopStoragePolling()
{
	if (!mPollingManager)
	{
		return false;
	}
	storagePollingActive = !mPollingManager->stopStateStoragePolling(*parent);
	return !storagePollingActive;
}


bool DebugManagerMSP430::reconnectJTAG()
{
	bool success = false;

	if ( IFetHandle* fetHandle = parent->getFetHandle() )
	{
		if ( IConfigManager* cm = fetHandle->getConfigManager() )
			success = (cm->start() > 0);

		if (mPollingManager)
		{
			mPollingManager->resumePolling();
		}
	}
	return success;
}

bool DebugManagerMSP430::run(uint16_t controlMask, DebugEventTarget * cb, bool releaseJtag)
{
	IMemoryManager* mm = this->parent->getMemoryManager();
	CpuRegisters* cpu = mm->getCpuRegisters();
	if (!cpu)
	{
		return false;
	}

	lpm5WakeupDetected = false;

	if (cb!=0)
	{
		cbx=cb;
	}

	uint32_t pc, sr;
	cpu->read(0, &pc);
	cpu->read(2, &sr);
	
	if (!restoreTinyRam())
	{
		return false;
	}

	if (!cpu->flushCache())
	{
		return false;
	}

	cycleCounter_.reset();
	IConfigManager *cm = parent->getFetHandle()->getConfigManager();

	const uint16_t mdb = parent->getEmulationManager()->getSoftwareBreakpoints()->getSwbpManager()->getInstructionAt(pc);
	if (mdb != 0)
	{
		mdbPatchValue = mdb;
	}

	HalExecElement* el = new HalExecElement(this->parent->checkHalId(ID_RestoreContext_ReleaseJtag));
	this->parent->getWatchdogControl()->addRestoreParamsTo(el);
	el->appendInputData32(pc);
	el->appendInputData16(sr);
	el->appendInputData16(controlMask!=0? 0x0007: 0x0006);	// eem control bits
	el->appendInputData16(mdbPatchValue);		// mdb
	el->appendInputData16(releaseJtag ? 1 : 0);
	el->appendInputData16(cm->ulpDebugEnabled() ? 1 : 0);

	mdbPatchValue = 0;

	HalExecCommand cmd;
	cmd.elements.emplace_back(el);

	if (!this->parent->send(cmd))
	{
		return false;
	}

	// handle lpmx5 polling
	if (releaseJtag)
	{
		pausePolling();
	}
	else
	{
		this->resumePolling();
	}

	if (controlMask != 0 && !releaseJtag && !activatePolling(controlMask))
	{
		return false;
	}

	return true;
}


void DebugManagerMSP430::runEvent(MessageDataPtr messageData)
{
	if (EmulationManagerPtr emManager = this->parent->getEmulationManager())
	{
		emManager->onEvent(messageData);
	}

	messageData->reset();

	uint16_t eventMask = 0;
	(*messageData) >> eventMask;

	if (eventMask & JSTATE_CAPTURE_FLAG)
	{
		uint32_t maskLow = 0;
		uint32_t maskHigh = 0;
		(*messageData) >> maskLow >> maskHigh;

		if (maskHigh & J_STATE_LPMX_MASK_HIGH)
		{
			if (cbx)
			{
				cbx->event(DebugEventTarget::Lpm5Sleep);
			}
		}
		else
		{
			if (cbx)
			{
				cbx->event(DebugEventTarget::Lpm5Wakeup);
			}
		}
	}

	if (eventMask & BP_HIT_FLAG)
	{
		saveContext();

		IMemoryManager* mm = this->parent->getMemoryManager();
		if (CpuRegisters* cpu = mm ? mm->getCpuRegisters() : 0)
		{
			uint32_t pc = 0;
			cpu->read(0, &pc);

			try
			{
				//If there is a SWBP 2 byte before the PC, we must adjust the PC accordingly and patch the MDB
				const uint16_t mdb = parent->getEmulationManager()->getSoftwareBreakpoints()->getSwbpManager()->getInstructionAt(pc-2);
				if (mdb != 0)
				{
					cpu->write(0, pc-2);
				}
			}
			catch (const EM_Exception&) { /*ignore*/ }
		}

		if (cbx)
		{
			cbx->event(DebugEventTarget::BreakpointHit);
		}
	}

	if (eventMask & STATE_STORAGE_FLAG)
	{
		uint16_t numEntries = 0;
		(*messageData) >> numEntries;

		if (cbx)
		{
			try
			{
				const size_t bufferSize = parent->getEmulationManager()->getTrace()->getTraceData().size();
				const uint16_t bufferFull = (bufferSize == 8) ? 1 : 0;
				cbx->event(DebugEventTarget::Storage, numEntries, bufferFull);
			}
			catch (const EM_Exception&) { /*ignore*/ }
		}
	}

	if (eventMask & VARIABLE_WATCH_FLAG)
	{
		if (cbx)
		{
			cbx->event(DebugEventTarget::VariableWatch);
		}
	}
}


bool DebugManagerMSP430::wakeupDevice()
{
	if (parent->getDeviceCode() == 0x20404020)
	{
		IConfigManager *cm = parent->getFetHandle()->getConfigManager();
		return cm && (cm->MSP430I_MagicPattern((uint16_t)AUTOMATIC_IF) != -1);
	}

	std::list<PinState> stateChanges;
	stateChanges.push_back( PinState(JTAG_PIN_TST, true, 5) );
	stateChanges.push_back( PinState(JTAG_PIN_RST, true, 5) );

	stateChanges.push_back( PinState(JTAG_PIN_TST, false, 20) );

	stateChanges.push_back( PinState(JTAG_PIN_RST, false, 10) );
	stateChanges.push_back( PinState(JTAG_PIN_RST, true, 5) );

	stateChanges.push_back( PinState(JTAG_PIN_TST, true, 5) );

	bool isSleeping = true;

	if (FetHandle* fetHandle = parent->getFetHandle())
	{
		int i = 0;
		while (isSleeping && i++ < 5)
		{
			// Power down JTAG LDO to enable RST NMI wakeup
			if (parent->getJtagId() == 0x99)
			{
				fetHandle->sendJtagShift(HIL_CMD_JTAG_IR, 0x2F);
				fetHandle->sendJtagShift(HIL_CMD_JTAG_DR, 0xC020, 16);
			}
			sendPinSequence(stateChanges, fetHandle);

			IConfigManager *cm = fetHandle->getConfigManager();

			cm->start();

			isSleeping = queryIsInLpm5State();
		}
	}

	return !isSleeping;
}

/* sync Jtag */
bool DebugManagerMSP430::stop(bool jtagWasReleased)
{
	bool success = false;
	int attemptsLeft = 3;
	bool isSleeping = false;
	bool wasInLpmx5 = false;

	// Pause the polling loop while we try to wake up the device and sync again
	this->pausePolling();

	do
	{
		if (queryIsInLpm5State())
		{
			isSleeping = !wakeupDevice();
			wasInLpmx5 = true;
			if (!jtagWasReleased)
			{
				this->resumePolling();
				std::this_thread::sleep_for(std::chrono::milliseconds(500));
				this->pausePolling();
			}
		}
		// save context when wakeup was detected
		if (lpm5WakeupDetected)
		{
			wasInLpmx5 = true;
			success = saveContext();
		}
	}
	while ( isSleeping && --attemptsLeft );

	// just save context when no wakeup was detected before
	if (!lpm5WakeupDetected)
	{
		success = saveContext();
	}

	success = success && !isSleeping;

	//If the device was in LPMx5, read reset vector and set PC
	if ( success && wasInLpmx5)
	{
		IMemoryManager* mm = parent->getMemoryManager();
		if ( CpuRegisters* cpu = mm->getCpuRegisters() )
		{
			uint8_t buffer[2];
			if (mm->read(0xFFFE, buffer, 2) && mm->sync() && mm->read(0xFFFE, buffer, 2) && mm->sync())
			{
				cpu->write(0, buffer[0] | (buffer[1] << 8));
				cpu->write(2, 0); // BTT2001 status register bugfix
			}
		}
	}

	// If the JTAG was released, we could not know that the device went into LPMx.5,
	// thus manually generate the event in this case
	if (jtagWasReleased && wasInLpmx5)
	{
		MessageDataPtr data(new MessageData);
		(*data) << (uint16_t)JSTATE_CAPTURE_FLAG;
		if (mPollingManager)
		{
			mPollingManager->queueEvent(data);
		}
	}

	return success;
}

bool DebugManagerMSP430::syncDeviceAfterLPMx5()
{
	lpm5WakeupDetected = true;
	return true;
}

bool DebugManagerMSP430::singleStep (uint32_t* cycles)
{
	IMemoryManager* mm = this->parent->getMemoryManager();
	CpuRegisters* cpu = mm->getCpuRegisters();
	if (!cpu)
		return false;

	lpm5WakeupDetected = false;

	uint32_t pc, sr;
	cpu->read(0, &pc);
	cpu->read(2, &sr);

	/* flush and clear all caches */
	if (!cpu->flushCache())
	{
		return false;
	}
	if (!restoreTinyRam())
	{
		return false;
	}

	HalExecCommand cmd;

	HalExecElement* instructionRead = 0; 
	if (cycles && emulationLevel < EMEX_EXTRA_SMALL_5XX)
	{
		//we run that inline instead of through the IMemoryManager
		instructionRead = new HalExecElement(this->parent->checkHalId(ID_ReadMemWords));
		instructionRead->appendInputData32(pc);
		instructionRead->appendInputData32(2);
		cmd.elements.emplace_back(instructionRead);
	}

	IConfigManager *cm = parent->getFetHandle()->getConfigManager();
	const uint16_t mdb = parent->getEmulationManager()->getSoftwareBreakpoints()->getSwbpManager()->getInstructionAt(pc);
	if (mdb != 0)
	{
		mdbPatchValue = mdb;
	}
	std::shared_ptr<WatchdogControl> wdt = this->parent->getWatchdogControl();
	HalExecElement* el = new HalExecElement(this->parent->checkHalId(ID_SingleStep));
	//Parameters for RestoreContext_ReleaseJtag
	wdt->addRestoreParamsTo(el);
	el->appendInputData32(pc);
	el->appendInputData16(static_cast<uint16_t>(sr));
	el->appendInputData16(7);
	el->appendInputData16(mdbPatchValue);
	//releaseJtag
	el->appendInputData16(0);
	el->appendInputData16(cm->ulpDebugEnabled() ? 1 : 0);
	//Parameters for SingleStep
	el->appendInputData16(irRequest);
	//Parameters for SyncJtag_Conditional_SaveContext
	wdt->addHoldParamsTo(el);

	mdbPatchValue = 0;

	cmd.elements.emplace_back(el);

	mPollingManager->resumeStateStoragePolling(*parent);
	if (!this->parent->send(cmd))
	{
		return false;
	}
	mPollingManager->pauseStateStoragePolling(*parent);

	// Exit if device stepped into LPMx.5 mode
	if (queryIsInLpm5State())
	{
		return true;
	}

	// get watchdog from single step macro
	uint16_t wdtCtrl = el->getOutputAt16(0);
	if (!WatchdogControl::checkRead(wdtCtrl))
	{
		return false;
	}
	// store watchdog values
	wdt->set(wdtCtrl);

	pc = el->getOutputAt32(2);
	sr = el->getOutputAt16(6);

	irRequest = el->getOutputAt16(8);

	// write PC and SR to IMemoryManager to signal that _NO_ additional
	// SyncJTAG is necessary
	cpu->write(0, pc);
	cpu->write(2, sr);
	cpu->fillCache(0, 16);

	setLeaAccessibility();
	backUpTinyRam();
	if (checkUssIsBusy())
	{
		return false;
	}

	if (cycles)
	{
		if (emulationLevel < EMEX_EXTRA_SMALL_5XX)
		{
			const uint16_t steppedIntoInterrupt = el->getOutputAt16(8);
			uint32_t instruction = instructionRead->getOutputAt32(0);
			cycleCounter_.countInstruction(instruction, steppedIntoInterrupt != 0);
		}
		*cycles = static_cast<uint32_t>(cycleCounter_.read());
	}

	return true;
}


bool DebugManagerMSP430::initEemRegister()
{
	bool success = false;

	EmulationManagerPtr emManager = parent->getEmulationManager();

	emManager->writeRegister(GENCTRL, 0x0);
	emManager->writeRegister(GENCLKCTRL, defaultGenClkCntrl);
	emManager->writeRegister(MODCLKCTRL0, defaultMclkcntrl0);

	cycleCounter_.configure();

	return success;
}


uint8_t DebugManagerMSP430::getClockControl() const
{
	return clockControl;
}

uint16_t DebugManagerMSP430::getClockControlSetting() const
{
	return genclkcntrl;
}

uint16_t DebugManagerMSP430::getGeneralClockDefaultSetting() const
{
	return defaultGenClkCntrl;
}

uint32_t DebugManagerMSP430::getClockModuleDefaultSetting() const
{
	return defaultMclkcntrl0;
}

uint32_t DebugManagerMSP430::getClockModuleSetting() const
{
	return mclkcntrl0;
}

char ** DebugManagerMSP430::getModuleStrings(uint32_t * n) const
{
	*n=nModuleStrings;
	return moduleStrings;
}

char ** DebugManagerMSP430::getClockStrings(uint32_t * n) const
{
	*n=nClockStrings;
	return clockStrings;
}

void DebugManagerMSP430::setOpcode(uint16_t value)
{
	mdbPatchValue = value;
}

bool DebugManagerMSP430::saveContext()
{
	IMemoryManager* mm = this->parent->getMemoryManager();

	CpuRegisters* cpu = mm->getCpuRegisters();
	if (!cpu)
		return false;

	HalExecElement* el0 = new HalExecElement(this->parent->checkHalId(ID_SyncJtag_Conditional_SaveContext));
	this->parent->getWatchdogControl()->addHoldParamsTo(el0);

	HalExecCommand cmd;
	cmd.elements.emplace_back(el0);

	if (!this->parent->send(cmd))
	{
		return false;
	}

	uint16_t wdtCtrl = el0->getOutputAt16(0);

	if (!WatchdogControl::checkRead(wdtCtrl))
		return false;
	// sync watchdog after stopping CPU
	this->parent->getWatchdogControl()->set(wdtCtrl);

	// get register values
	uint32_t pc = el0->getOutputAt32(2);
	uint32_t sr = el0->getOutputAt16(6);
	irRequest = el0->getOutputAt16(8);

	cpu->write(0, pc);
	cpu->write(2, sr);
	cpu->fillCache(0, 16);

	cycleCounter_.read();

	backUpTinyRam();
	setLeaAccessibility();
	if (checkUssIsBusy())
	{
		return false;
	}
	return true;
}

void DebugManagerMSP430::setLpmDebugging(bool enable)
{
	lpmDebuggingEnabled = enable;
}

bool DebugManagerMSP430::getLpmDebugging()
{
	return lpmDebuggingEnabled;
}

bool DebugManagerMSP430::queryIsInLpm5State()
{
	if (!getLpmDebugging() && (parent->getDeviceCode() != 0x20404020))
	{
		return false;
	}

	const uint64_t bit = 1;

	HalExecElement* el = new HalExecElement(parent->checkHalId(ID_PollJStateReg));
	el->appendInputData16(1); //Query once, force send
	el->appendInputData16(0); //Breakpoint polling
	el->appendInputData16(1); //LPM polling
	el->appendInputData16(0); //Energy Trace
	el->appendInputData16(0); //Gated Mode

	HalExecCommand cmd;
	cmd.elements.emplace_back(el);

	uint64_t jstate = 0;

	if (this->parent->send(cmd))
	{
		jstate  = el->getOutputAt32(2);
		jstate |= (uint64_t)el->getOutputAt32(6) << 32;
	}
	return (( jstate & (bit << 62) ) != 0);
}

void DebugManagerMSP430::pausePolling()
{
	if (mPollingManager)
	{
		mPollingManager->pausePolling();
	}
}

void DebugManagerMSP430::resumePolling()
{
	if (mPollingManager)
	{
		mPollingManager->resumePolling();
	}
}

bool DebugManagerMSP430::backUpTinyRam()
{
	IMemoryManager* mm = this->parent->getMemoryManager();
	if (MemoryArea* tinyRam = mm ? mm->getMemoryArea(MemoryArea::TinyRam) : nullptr)
	{
		return tinyRam->fillCache();
	}
	// in case the device has no tinyRam
	return true;
}

bool DebugManagerMSP430::restoreTinyRam()
{
	IMemoryManager* mm = this->parent->getMemoryManager();
	if (MemoryArea* tinyRam = mm ? mm->getMemoryArea(MemoryArea::TinyRam) : nullptr)
	{
		return tinyRam->flushCache();
	}
	// in case the device has no tinyRam
	return true;
}

void DebugManagerMSP430::setLeaAccessibility()
{
	IMemoryManager* mm = this->parent->getMemoryManager();
	if (MemoryArea* lea_registers = mm ? mm->getMemoryArea(MemoryArea::LeaPeripheral) : nullptr)
	{
		HalExecElement* el = new HalExecElement(ID_LeaSyncConditional);
		el->appendInputData32(lea_registers->getStart());

		HalExecCommand cmd;
		cmd.elements.emplace_back(el);

		if (this->parent->send(cmd))
		{
			const bool leaBusy = el->getOutputAt16(0) != 0;

			lea_registers->setAccessible(!leaBusy);
			if (MemoryArea* lea_ram = mm->getMemoryArea(MemoryArea::LeaRam))
			{
				lea_ram->setAccessible(!leaBusy);
			}
		}
	}
}
bool DebugManagerMSP430::checkUssIsBusy()
{
	IMemoryManager* mm = this->parent->getMemoryManager();
	if (MemoryArea* uss_registers = mm ? mm->getMemoryArea(MemoryArea::UssPeripheral) : nullptr)
	{
		HalExecElement* el = new HalExecElement(ID_UssSyncConditional);
		el->appendInputData32(uss_registers->getStart());
		HalExecCommand cmd;
		cmd.elements.emplace_back(el);
		if (this->parent->send(cmd))
		{
			const bool ussBusy = el->getOutputAt16(1) != 0;
			return (ussBusy);
		}
	}
	return false;
}
bool DebugManagerMSP430::setPCtoSafeLocation()
{
	uint32_t pc = 0x4;
	uint32_t sr = 0;
	IMemoryManager * mm = parent->getMemoryManager();
	if (!mm)
	{
		return false;
	}
	CpuRegisters* cpu = mm->getCpuRegisters();
	if (!cpu)
	{
		return false;
	}
	cpu->write(0, pc);
	cpu->write(2, sr);	
	cpu->flushCache();
	return true;
}
