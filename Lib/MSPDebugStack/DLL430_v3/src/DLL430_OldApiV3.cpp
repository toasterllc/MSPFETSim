/*
 * DLL430_OldApi.cpp
 *
 * Old API interface for IAR - V3 implementation.
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

#include "DLL430_OldApiV3.h"
#include "DeviceInfo.h"
#include "DeviceDb/Database.h"
#include "EemMemoryAccess.h"
#include "CpuRegisters.h"
#include "IUpdateManager.h"
#include "HidUpdateManager.h"
#include "IDeviceHandle.h"
#include "../version.h"
#include "FetHandleManagerImpl.h"

#include "EM/EmulationManager/IEmulationManager.h"
#include "EM/Trace/ITrace.h"
#include "EM/VariableWatch/IVariableWatch.h"
#include "EM/Sequencer/ISequencer.h"
#include "EM/CycleCounter/ICycleCounter.h"
#include "EM/SoftwareBreakpoints/ISoftwareBreakpoints.h"
#include "EM/Exceptions/Exceptions.h"
#include "../../Bios/include/error_def.h"
#include "TI/DLL430/warnings/Warnings.h"

#include <boost/thread.hpp>


#include "JtagId.h"
#include "JtagShifts.h"
#include "MemoryContent.h"
#include "FileReader.h"
#include "FileWriter.h"

#ifdef UNIX
#include <boost/filesystem.hpp>
using namespace boost::filesystem;
#endif

using namespace TI::DLL430;
using namespace std;
using namespace std::placeholders;

#define OLD_API_USB_TYPE "CDC"

#define ERROR_DEF(errorEnum, errorString) errorString,
const char* const errorStrings[] = { ERROR_DEFINITIONS };
#undef ERROR_DEF


DLL430_OldApiV3::DLL430_OldApiV3()
	: varWatch_state(VW_DISABLE)
	, manager(new FetHandleManagerImpl)
	, handle(0)
	, errNum(0)
	, singleDevice(0)
	, selectedJtagMode(AUTOMATIC_IF)
	, devInfoFilled(false)
	, devInfo()
	, devCode(0)
	, notifyCallback(0)
	, clientHandle(0)
	, mPollingManager(0)
	, mEnergyTraceManager(0)
	, devArchitecture(MSP430)
	, lastCycleCount(0)
{
	varWatch_state = VW_DISABLE;

	memset(&mPdSetup, 0, sizeof(mPdSetup));
	memset(&mPdCallbacks, 0, sizeof(mPdCallbacks));

	trace_storage.trAction=TR_FETCH;
	trace_storage.trControl=TR_DISABLE;
	trace_storage.trMode=TR_HISTORY;

	this->debug.state = STOPPED;
	this->debug.jtagReleased = true;
	this->debug.cb.func = 0;

	const uint16_t DEFAULT_CLKCNTRL = 0x0026;
	const uint16_t DEVICE_DEFAULT_MCLKCNTRL0 = 0x2407;

	clock_control.ccControl		= CC_DISABLE;
	clock_control.ccGeneralCLK	= DEFAULT_CLKCNTRL;
	clock_control.ccModule		= DEVICE_DEFAULT_MCLKCNTRL0;

	memset(&moduleNameBuffer, 0, sizeof(moduleNameBuffer));

	memset(&sequencer_control, 0, sizeof(sequencer_control));
}

DLL430_OldApiV3::~DLL430_OldApiV3 ()
{
	//This is called from the detach handler, be VERY careful with what you do here
	//Do NOT try to use mutexes or join threads. Do as much as possible in Close(), not here.
}

void DLL430_OldApiV3::event(DebugEventTarget::EventType e,  uint32_t lParam, uint16_t wParam)
{
	boost::lock_guard<boost::recursive_mutex> lock(apiMutex);
	boost::lock_guard<boost::mutex> callbackLock(callbackMutex);
	switch (e)
	{
	case DebugEventTarget::EnergyTraceData:
	{
		if (mPdCallbacks.pPushDataFn)
		{
			const void* buffer = mEnergyTraceManager->getEnergyTraceBuffer();
			const size_t size = mEnergyTraceManager->getEnergyTraceBufferSize();

			// Push the data to the IDE
			if (buffer)
			{
				mPdCallbacks.pPushDataFn(mPdCallbacks.pContext, (const uint8_t*)buffer, static_cast<uint32_t>(size));
			}
		}
		break;

	}
	case DebugEventTarget::Lpm5Sleep:
	{
		execNotifyCallback(DEVICE_IN_LPM5_MODE);
		debug.state = LPMX5_MODE;
		break;
	}

	case DebugEventTarget::Lpm5Wakeup:
	{
		try
		{
			resetEM();
		}
		catch (const EM_Exception&) {}

		if (IDebugManager* db_man = singleDevice->getDebugManager())
		{
			db_man->syncDeviceAfterLPMx5();
			int32_t dummyState = 0;
			int32_t dummyCycles = 0;
			State(&dummyState, 1, &dummyCycles);
		}

		execNotifyCallback(DEVICE_WAKEUP_LPM5_MODE);
		debug.state = LPMX5_WAKEUP;
		break;
	}
	case DebugEventTarget::BreakpointHit:
	{
		if (debug.state == LPMX5_MODE)
		{
			break;
		}
		if (IDebugManager* db_man = singleDevice->getDebugManager())
		{
			//With trace/variable watch enabled, give storage events on same trigger time to be reported
			if (trace_storage.trControl == TR_ENABLE)
			{
				this_thread::sleep_for(chrono::milliseconds(100));
			}
			db_man->pausePolling();
		}
		this->debug.state = BREAKPOINT_HIT;

		if (this->debug.cb.func)
		{
			(*this->debug.cb.func)(

				this->debug.cb.ids.uiMsgIdBreakpoint,
				0,
				0,
				this->debug.cb.clientHandle
				);
		}
		break;
	}
	case DebugEventTarget::Storage:
	{
		if (this->debug.cb.func)
		{
			(*this->debug.cb.func)(
				this->debug.cb.ids.uiMsgIdStorage,
				lParam,		// # new entries
				wParam,
				this->debug.cb.clientHandle
				);
		}
		break;
	}

	case DebugEventTarget::VariableWatch:
	{
		watchedVariablesMutex.lock();
		for (map<uint16_t, WatchedVariablePtr>::const_iterator it = watchedVariables.begin(); it != watchedVariables.end(); ++it)
		{
			if (it->second->isValid() && this->debug.cb.func)
			{
				(*this->debug.cb.func)(
					this->debug.cb.ids.uiMsgIdStorage,
					it->first,
					it->second->value(),
					this->debug.cb.clientHandle
					);
			}
		}
		watchedVariablesMutex.unlock();
		break;
		}
	}
}


void DLL430_OldApiV3::log(TI::DLL430::LogTarget::Severity sev, unsigned int id, const char* message)
{
	switch (sev)
	{
	case LogTarget::ERR:
	case LogTarget::FATAL:
		this->errNum = id;
		break;

	default:
		break;
	}
}

bool DLL430_OldApiV3::GetNumberOfUsbIfs(int32_t* Number)
{
	assert(this->manager);

	//Make sure there is no active communication when we clear the port list
	this->Close(0);

	// API does not support different USB types here
	// using default
	this->manager->createPortList(OLD_API_USB_TYPE, true, false);
	if (nullptr != Number)
	{
		*Number = static_cast<int32_t>(this->manager->getPortNumber());
		return true;
	}

	log(LogTarget::ERR, PARAMETER_ERR, "");
	return false;
}

bool DLL430_OldApiV3::GetNameOfUsbIf(int32_t Idx, char** Name, int32_t* Status)
{
	if (PortInfo *portinfo = this->manager->getPortElement(Idx))
	{
		// Copy string to persistent buffer that won't become invalid after init
		port_names.resize( max((int32_t)port_names.size(), Idx+1) );
		strncpy(port_names[Idx].name, portinfo->name.c_str(), sizeof(port_names[Idx].name)-1);

		*Name = port_names[Idx].name;
		*Status = portinfo->status;
		return true;
	}

	log(LogTarget::ERR, USB_FET_NOT_FOUND_ERR, "");
	return false;
}

bool DLL430_OldApiV3::SetTargetArchitecture(TARGET_ARCHITECTURE_t architecture)
{	
	if (architecture != MSP430 && architecture != MSP432_M4)
	{
		log(LogTarget::ERR, WRONG_TARGET_ARCHITECTURE, "");
		return false;
	}
	devArchitecture = architecture;

	return true;
}

bool DLL430_OldApiV3::loadDeviceDb(const char* file)
{
	try
	{
		#ifndef NDEBUG
			if (file)
			{
				// load external database only for in debug build
				TI::DLL430::DeviceDb::Database().loadDevices(file, true);
			}
			else
			{
		#endif
				// laod internal database
				TI::DLL430::DeviceDb::Database().loadDevices("", false);
		#ifndef NDEBUG
			}
		#endif
	}
	catch (const std::runtime_error& e)
	{
#ifndef NDEBUG
		ofstream("xml_error.log") << e.what() << endl;
#endif
		log(LogTarget::ERR, DEVICE_DB_ERR, e.what());
		return false;
	}
	return true;
}

bool DLL430_OldApiV3::DumpDeviceDb(const char* file)
{
	try
	{
		if (file)
		{
			// load external database only for in debug build
			TI::DLL430::DeviceDb::Database().dump(file);
		}
		else
		{
			// laod internal database
			TI::DLL430::DeviceDb::Database().dump("database.xml");
		}
	}
	catch (const std::runtime_error& e)
	{
#ifndef NDEBUG
		ofstream("xml_error.log") << e.what() << endl;
#endif
		log(LogTarget::ERR, DEVICE_DB_ERR, e.what());
		return false;
	}
	return true;
}

bool DLL430_OldApiV3::clearDeviceDb()
{
	TI::DLL430::DeviceDb::Database().clearDevices();
	return true;
}

bool DLL430_OldApiV3::Initialize(const char* port, int32_t* version)
{
	// if ports are still open, exec a cleanup here
	this->Close(0);

	PortInfo *portInfo = this->manager->getPortElement(std::string(port));

	if (!portInfo)
	{
		manager->createPortList(OLD_API_USB_TYPE, true, false);
		portInfo = this->manager->getPortElement(std::string(port));
	}

#ifdef UNIX
	if (!portInfo)
	{
		try
		{
			portInfo = this->manager->getPortElement(boost::filesystem::canonical(port).filename().string());
		}
		catch (...) {}
	}
#endif

	if (nullptr == portInfo)
	{
		log(LogTarget::ERR, USB_FET_NOT_FOUND_ERR, "");
		return false;
	}
	else if (portInfo->status == PortInfo::inUseByAnotherInstance)
	{
		log(LogTarget::ERR, USB_FET_BUSY_ERR, "");
		return false;
	}

	if (portInfo->type == PortInfo::BSL)
	{
		*version = -2;
		return true;
	}

	this->handle = this->manager->createFetHandle(*portInfo, devArchitecture);
	if (!this->handle)
	{
		log(LogTarget::ERR, COMM_ERR, "");
		this->Close(0);
		return false;
	}

	// Everything was OK so create an instance of the EnergyTraceManager
	mPollingManager =  new PollingManager(dynamic_cast<FetHandle*>(handle));
	mEnergyTraceManager = new EnergyTraceManager(dynamic_cast<FetHandle*>(handle), mPollingManager);
	this->handle->getConfigManager()->setEnergyTraceManager(mEnergyTraceManager);

	if (notifyCallback!=nullptr)
	{
		handle->addSystemNotifyCallback(bind(&DLL430_OldApiV3::iNotifyCallback, this, std::placeholders::_1));
	}

	if (version)		// pointer not 0
	{
		FetHandle* fetHandle = dynamic_cast<FetHandle*>(handle);
		if (fetHandle->getControl()->getDcdcSubMcuVersion() == 0xAA)
		{
			log(LogTarget::ERR, POSSIBLE_OVERCURRENT, "");
			*version = -1;
			return false;
		}
		if (this->handle->getConfigManager()->isUpdateRequired(devArchitecture))
		{
			*version = -1;
		}
		else
		{
			*version = VERSION_MAJOR * 10000000 +
						VERSION_MINOR * 100000 +
						VERSION_PATCH * 1000 +
						VERSION_BUILD;

			handle->resetState();
		}
	}

	errNum = NO_ERR;
	return true;
}

bool DLL430_OldApiV3::SetSystemNotfyCallback(SYSTEM_NOTIFY_CALLBACK parSystemNotifyCallback)
{

	boost::lock_guard<boost::mutex> lock(callbackMutex);
	notifyCallback=parSystemNotifyCallback;

	if (handle)
	{
		handle->addSystemNotifyCallback(bind(&DLL430_OldApiV3::iNotifyCallback, this, std::placeholders::_1));
	}

	return true;
}

bool DLL430_OldApiV3::OpenDevice(const char* Device, const char* Password, int32_t PwLength, int32_t DeviceCode, int32_t setId)
{
	string tmpName(Device);
	if (tmpName.find("MSP430C09") == 0)
	{
		DeviceCode = 0xDEADBABE;
	}

	if (tmpName.find("MSP430I") == 0 && DeviceCode == 0)
	{
		DeviceCode = 0x20404020;
	}

	if (tmpName.find("MSP430L09") == 0 || tmpName.find("MSP430C09") == 0)
	{
		this->Configure(INTERFACE_MODE, JTAG_IF);
		devCode = DeviceCode;
	}
	FetHandle* fetHandle = dynamic_cast<FetHandle*>(handle);

	if (fetHandle && fetHandle->getControl()->getFetToolId() == MSP_FET430 && this->devArchitecture == TARGET_ARCHITECTURE_t::MSP432_M4)
	{
		log(LogTarget::ERR, INTERFACE_SUPPORT_ERR_MSP432, "");
		return false;
	}
	if (this->Identify(devInfo.buffer, sizeof(DEVICE_T), setId, Password, PwLength, DeviceCode))
	{
		devInfoFilled = true;
		debug.jtagReleased = false;

		IMemoryManager *mm = singleDevice->getMemoryManager();
		if (mm)
		{
			MemoryArea *main = mm->getMemoryArea(MemoryArea::Main);
			if (main && main->isProtectionEnabled())
			{
				WarningFactory::instance()->message(MESSAGE_LEVEL_T::MSPDS_MESSAGE_LEVEL_WARNING, WarningCode::WARNING_IP_PROTECTION);
			}
		}

		return true;
	}
	return false;
}

bool DLL430_OldApiV3::GetFoundDevice(uint8_t* FoundDevice, int32_t count)
{
	count = min((int32_t)sizeof(DEVICE_T), count);
	memcpy(FoundDevice, &devInfo, count);
	return true;
}

bool DLL430_OldApiV3::Close(int32_t vccOff)
{
	if (!this->handle)
	{
		return true;
	}

	SetSystemNotfyCallback(NULL);
	RegisterMessageCallback(nullptr);
	this->debug.cb.func = 0;
		
	selectedJtagMode = AUTOMATIC_IF;
	if (singleDevice)
	{
		if (IDebugManager* db_man = singleDevice->getDebugManager())
		{
			db_man->pausePolling();
		}
	}

	//Stop polling and wait for potentially active callbacks to return
	if (mPollingManager)
	{
		mPollingManager->shutdown();
	}

	bool success = disableSoftwareBreakpointsOnClose();

	if (singleDevice != nullptr)
	{
		const bool wasRunning = deviceIsRunning();

		int32_t state = 0, cpuCycles = 0;
		State(&state, true, &cpuCycles);

		singleDevice->disableHaltOnWakeup();

		if (wasRunning)
		{
			// just start device execution, JTAG test register will be set to default values by Stop JTAG.
			Run(FREE_RUN, false);
		}
		else
		{ 
			if (IDebugManager* db_man = singleDevice->getDebugManager())
			{
				db_man->setPCtoSafeLocation();
			}
		}
	}

	if ( IConfigManager* cm = this->handle->getConfigManager() )
	{
		cm->stop();

		if (vccOff)
		{
			if (!cm->setDeviceVcc(0))
			{
				log(LogTarget::ERR, VCC_ERR, "");
				success = false;
			}
		}
	}

	this->handle->shutdown();

	traceTriggers.clear();
	breakpoints.clear();
	triggerCombinations.clear();
	watchedVariables.clear();
	varWatch_state = VW_DISABLE;

	if (singleDevice!=nullptr)
	{
		IDeviceHandleManager* dhm = handle->getDeviceHandleManager();
		dhm->destroyDeviceHandle(singleDevice);
		singleDevice=nullptr;
	}

	delete mEnergyTraceManager;
	mEnergyTraceManager = nullptr;

	delete mPollingManager;
	mPollingManager = nullptr;

	if (manager != nullptr)
	{
		this->manager->destroyFetHandle(this->handle);
		this->handle = 0;

		selectedJtagMode = AUTOMATIC_IF;
		manager->clearPortList();

		config_settings.clear();
	}

	return success;
}


bool DLL430_OldApiV3::disableSoftwareBreakpointsOnClose()
{
	bool success = true;

	if (singleDevice && devArchitecture == MSP430)
	{
		try
		{
			SoftwareBreakpointManagerPtr swbpMan = singleDevice->getEmulationManager()->getSoftwareBreakpoints()->getSwbpManager();
			if (swbpMan->numberOfActiveSoftwareTriggers() > 0)
			{
				const STATE_MODES previousState = debug.state;
				const bool jtagReleased = debug.jtagReleased;

				int32_t state = 0, cpuCycles = 0;
				State(&state, true, &cpuCycles);

				if (!Configure(SOFTWARE_BREAKPOINTS, DISABLE))
				{
					success = false;
				}

				if (previousState == RUNNING)
				{
					Run(FREE_RUN, jtagReleased);
				}
			}
		}
		catch (const EM_Exception&)
		{
			log(LogTarget::ERR, REMOVE_SOFTWARE_BREAKPOINT_ERR, "");
		}
	}
	return success;
}


bool DLL430_OldApiV3::Configure(enum CONFIG_MODE mode, int32_t value)
{
	if (handle == nullptr)
	{
		log(LogTarget::ERR, INTERFACE_SUPPORT_ERR, "");
		return false;
	}

	// Save the value for later use
	if ( mode != WRITE_EXTERNAL_MEMORY && mode != SET_MDB_BEFORE_RUN && mode != TOTAL_ERASE_DEVICE )
	{
		config_settings[mode] = value;
	}

	switch (mode)
	{
		case INTERFACE_MODE:
		{
			IConfigManager* cm = this->handle->getConfigManager();
			if (!cm)
			{
				log(LogTarget::ERR, INTERNAL_ERR, "");
				return false;
			}
			enum INTERFACE_TYPE type = (enum INTERFACE_TYPE)value;
			switch (type)
			{
				case JTAG_IF:
					selectedJtagMode = JTAG_IF;
					cm->setJtagMode(JTAG_IF);
					break;
				case SPYBIWIRE_IF:
					selectedJtagMode = SPYBIWIRE_IF;
					cm->setJtagMode(SPYBIWIRE_IF);
					break;
				case SPYBIWIREJTAG_IF:
					selectedJtagMode = SPYBIWIREJTAG_IF;
					cm->setJtagMode(SPYBIWIREJTAG_IF);
					break;
				case SPYBIWIRE_MSP_FET_IF:
					selectedJtagMode = SPYBIWIRE_MSP_FET_IF;
					cm->setJtagMode(SPYBIWIRE_MSP_FET_IF);
					break;
				case JTAG_MSP432:
					selectedJtagMode = JTAG_MSP432;
					cm->setJtagMode(JTAG_MSP432);
					break;
				case SWD_MSP432:
					selectedJtagMode = SWD_MSP432;
					cm->setJtagMode(SWD_MSP432);
					break;
				case AUTOMATIC_IF:
					selectedJtagMode = AUTOMATIC_IF;
					break;
				default:
					log(LogTarget::ERR, PARAMETER_ERR, "");
					return false;
			}
		}
		break;

	case EMULATION_MODE:
	/*	try
		{
			DeviceDb::Database().dump();
		}
		catch (const runtime_error& e)
		{
			e.what();
		}*/
		break;

	case SET_MDB_BEFORE_RUN:
		if (singleDevice == nullptr)
		{
			log(LogTarget::ERR, INTERNAL_ERR, "");
			return false;
		}

		try
		{
			if (singleDevice->getEmulationManager()->getSoftwareBreakpoints()->isEnabled())
			{
				log(LogTarget::ERR, INCOMPATIBLE_WITH_SW_BREAKPOINT_API, "");
				return false;
			}
		}
		catch (const EM_Exception& e)
		{
			log(LogTarget::ERR, e.errorCode(), e.what());
			return false;
		}

		if (IDebugManager* db_man = singleDevice->getDebugManager())
		{
			db_man->setOpcode((uint16_t)value);
		}
		break;

	case WRITE_EXTERNAL_MEMORY:
		if (value != 0)
		{
			try
			{
				map<uint16_t, BpParameter_t> breakpointsBackup = bp_storage;
				clearSoftwareTriggers();

				if (!writeToExternalMemory())
				{
					log(LogTarget::ERR, WRITE_MEMORY_ERR, "");
					return false;
				}

				resetEM();
				restoreSoftwareTriggers(breakpointsBackup);
			}
			catch (const EM_Exception& e)
			{
				log(LogTarget::ERR, e.errorCode(), e.what());
				return false;
			}

		}
		break;

	case TOTAL_ERASE_DEVICE:
		return Erase(ERASE_TOTAL, 0, 0);

	case UNLOCK_BSL_MODE:
		lockMemory(MemoryArea::Bsl, value != ENABLE);
		break;

	case LOCKED_FLASH_ACCESS:
		lockMemory(MemoryArea::Info, value != ENABLE);
		break;

	case RAM_PRESERVE_MODE:
		if ( singleDevice )
		{
			if ( IMemoryManager* mm = singleDevice->getMemoryManager() ){
				mm->setRamPreserveMode( value == ENABLE );
			}
		}
		break;
	case DEBUG_LPM_X:		
		if (singleDevice)
		{
			IConfigManager* cm = this->handle->getConfigManager();
			if (!cm)
			{
				log(LogTarget::ERR, INTERNAL_ERR, "");
				return false;
			}
			if (IDebugManager* dbgManager = singleDevice->getDebugManager())
			{
				dbgManager->setLpmDebugging(value == ENABLE);
				cm->setUlpDebug(value == ENABLE);
			}
		}
		break;

	case SET_INTERFACE_SPEED:
		if (IConfigManager* cm = this->handle->getConfigManager())
		{
			if (!cm->configureJtagSpeed(value))
			{
				log(LogTarget::ERR, SPEED_CONFIG_ERR, "");
				return false;
			}
		}
		else
		{
			log(LogTarget::ERR, INTERNAL_ERR, "");
			return false;
		}
		break;

	case ET_CURRENTDRIVE_FINE:
		if (IConfigManager* cm = this->handle->getConfigManager())
		{
			cm->setCurrentDrive(value);
		}
		else
		{
			log(LogTarget::ERR, INTERNAL_ERR, "");
			return false;
		}
		break;

	case SOFTWARE_BREAKPOINTS:
		if (!singleDevice)
		{
			log(LogTarget::ERR, INTERNAL_ERR, "");
			return false;
		}
		try
		{
			SoftwareBreakpointsPtr swbp = singleDevice->getEmulationManager()->getSoftwareBreakpoints();
			if (value == ENABLE)
			{
				swbp->enable();
			}
			else
			{
				swbp->disable();
				clearSoftwareTriggers();
			}

			singleDevice->getEmulationManager()->writeConfiguration();
		}
		catch (const EM_Exception& e)
		{
			log(LogTarget::ERR, e.errorCode(), e.what());
			return false;
		}
		break;

	case DISABLE_INTERRUPTS:
		if (IConfigManager* cm = this->handle->getConfigManager())
		{
			cm->setDisableInterruptsMode(value);
		}
		else
		{
			log(LogTarget::ERR, INTERNAL_ERR, "");
			return false;
		}
		break;

	case CONFIG_JTAG_LOCK_5XX:
		if (IConfigManager* cm = this->handle->getConfigManager())
		{
			cm->setJTAGLock5xx(value);
		}
		else
		{
			log(LogTarget::ERR, INTERNAL_ERR, "");
			return false;
		}
		break;

	default:
		log(LogTarget::ERR, PARAMETER_ERR, "");
		return false;
	}
	return true;
}

bool DLL430_OldApiV3::lockMemory(MemoryArea::Name memoryName, bool lock)
{
	if (singleDevice != nullptr)
	{
		IMemoryManager* mm = singleDevice->getMemoryManager();
		if (mm != nullptr)
		{
			const bool success = mm->lock(memoryName, lock);
			if (!success)
			{
				log(LogTarget::ERR, UNLOCK_BSL_ERR, "");
			}

			return success;
		}
	}
	return false;
}

int32_t DLL430_OldApiV3::Error_Number(void)
{
	int32_t ret = this->errNum;

	this->errNum = 0;
	return ret;
}

const char* DLL430_OldApiV3::Error_String(int32_t errorNumber)
{
	if ((errorNumber < 0) || (errorNumber >= INVALID_ERR))
	{
		errorNumber = INVALID_ERR;
	}

	return (errorStrings[errorNumber]);
}

bool DLL430_OldApiV3::GetJtagID(int32_t* JtagId)
{
	log(LogTarget::ERR, INTERFACE_SUPPORT_ERR, "");
	return false;
}

bool DLL430_OldApiV3::Identify(uint8_t* buffer, int32_t count, int32_t setId, const char* Password, int32_t PwLength, int32_t code)
{
	errNum=0;
	if (DeviceDb::Database().getMaxId() < 0)
	{
		log(LogTarget::ERR, DEVICE_DB_ERR, "");
		return false;
	}
	if (setId < 0 || setId > DeviceDb::Database().getMaxId())
	{
		log(LogTarget::ERR, PARAMETER_ERR, "");
		return false;
	}
	IConfigManager* cm = this->handle ? this->handle->getConfigManager() : nullptr;
	if (!cm || !handle)
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}
	//--disable overcurrent detection for device init------
	cm->configureOverCurrent(false);

	std::shared_ptr<void> restoreOverCurrent(nullptr,
							bind(&IConfigManager::configureOverCurrent, cm, true));

	//---------------------find Interface-------------------
	INTERFACE_TYPE selectedMode = JTAG_IF;
	if (devArchitecture == MSP432_M4)
	{
		if (selectedJtagMode == AUTOMATIC_IF)
		{
			INTERFACE_TYPE ifMode = cm->getInterfaceMode(devArchitecture);
			cm->setJtagMode(ifMode);
			selectedMode = ifMode;
		}
		else if (selectedJtagMode == SPYBIWIRE_IF ||
			selectedJtagMode == SWD_MSP432)
		{
			cm->setJtagMode(SWD_MSP432);
			selectedMode = SWD_MSP432;
		}
		else
		{
			cm->setJtagMode(JTAG_MSP432);
			selectedMode = JTAG_MSP432;
		}
		code = 0x432;
	}
	else if (selectedJtagMode == AUTOMATIC_IF)
	{
		INTERFACE_TYPE ifMode = cm->getInterfaceMode(devArchitecture);
		if (ifMode == UNDEF_IF)
		{
			//Assume 4-wire with JTAG-bits locked. This will cause the MagicPattern to be sent later
			ifMode = SPYBIWIREJTAG_IF;
		}
		cm->setJtagMode(ifMode);
	}

	//------------------end find Interface-------------------
	// start JTAG
	cm->setDeviceCode(code);
	//Leading "0x" is cut off here
	const int strLen = (Password && PwLength > 0) ? PwLength * 4 : 0;
	const string pwd = (strLen > 0) ? string(Password + 2, strLen) : "";
	cm->setPassword(pwd);

	int16_t num = cm->start();
	if (num < 0)
	{
		if (num == -2)
		{
			log(LogTarget::ERR, WRONG_PASSWORD, "");
		}
		else
		{
			log(LogTarget::ERR, INTERNAL_ERR, "");
		}
		return false;
	}

	IDeviceHandleManager* dhm = handle->getDeviceHandleManager();

	if (singleDevice != nullptr)
	{
		dhm->destroyDeviceHandle(singleDevice);
	}
	// save the device handle to work with
	singleDevice = dhm->createDeviceHandle((uint32_t)code, selectedMode);
	// sanity check
	if (singleDevice == nullptr)
	{
		cm->stop();
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}
	//attach to running target
	if (setId != 0x0)
	{
		if (0 == singleDevice->getJtagId())// if no jtag id was found the device is locked or lpmx.5
		{
			setId = DEVICE_UNKNOWN;
		}
	}

	if (setId == DEVICE_UNKNOWN)
	{
		setId = singleDevice->identifyDevice(code, false);
		if (setId < 0)
		{
		
			bool isFuseBlown = false;
			long errorCode = 0;

			if (devArchitecture == TARGET_ARCHITECTURE::MSP430)
			{
				isFuseBlown = singleDevice->isJtagFuseBlown();
			}
			else // 432 DAP is locked
			{
				if (setId == -2)
				{
					isFuseBlown = true;
					errorCode = -1;
				}
				else
				{
					cm->reset(false, true, 0, 0);
					cm->setJtagMode(SWD_MSP432);
					selectedMode = SWD_MSP432;
					cm->start();
				}
			}
			
			if (devArchitecture == TARGET_ARCHITECTURE::MSP430)
			{				
				// this handles the rst of an MSP430i family device
				errorCode = -1;
				if (code == 0x20404020)
				{
					errorCode = cm->MSP430I_MagicPattern((uint16_t)selectedJtagMode);
				}
				if ((errorCode == -1) && (selectedJtagMode != JTAG_IF) && (singleDevice->getJtagId() != 0x89))
				{
					errorCode = singleDevice->magicPatternSend((uint16_t)selectedJtagMode);
				}
				if (singleDevice->getJtagId() == 0x89)
				{
					const bool success = handle && handle->sendHilCommand(HIL_CMD_BSL_1XX_4XX);
					if (!success)
					{
						errorCode = -1;
					}
					else
					{
						errorCode = 0;
						cm->start();
					}

				}
			}

			// call magic pattern to get device under control
			switch (errorCode)
			{
			case 0: // no error
				break;
			case HALERR_MAGIC_PATTERN_BOOT_DATA_CRC_WRONG:
				log(LogTarget::ERR, DEVICE_CRC_WRONG, "");
				return false;
			case 5:
				log(LogTarget::ERR, MSP432_TOTAL_ERASE, "");
				return false;
			default:
				if (isFuseBlown)
				{
					if (devArchitecture == TARGET_ARCHITECTURE::MSP430)
					{
						log(LogTarget::ERR, FUSE_BLOWN_ERR, "");
					}
					else
					{
						log(LogTarget::ERR, DAP_LOCK_WRONG_PROTOCOL, "");
					}
				}
				else
				{
					log(LogTarget::ERR, DEVICE_UNKNOWN_ERR, "");
				}
				return false;
			}

			// If the device is locked by a password, unlock it
			if (!pwd.empty())
			{
				cm->start();
			}

			// Re-initialize the device handle
			dhm->destroyDeviceHandle(singleDevice);
			singleDevice = dhm->createDeviceHandle((uint32_t)code, selectedMode);

			// try to identify the device again
			setId = singleDevice->identifyDevice(code, true);
		}
		if (singleDevice->isJtagFuseBlown())
		{
			log(LogTarget::ERR, FUSE_BLOWN_ERR, "");
			return false;
		}
		if (setId < 0)
		{
			log(LogTarget::ERR, DEVICE_UNKNOWN_ERR, "");
			return false;
		}
		this->debug.state = STOPPED;
	}
	else // this is Attach to running target
	{
		if (singleDevice->setDeviceId(setId) == -1)
		{
			log(LogTarget::ERR, DEVICE_UNKNOWN_ERR, "");
			return false;
		}
		this->debug.state=RUNNING;
	}

	if (singleDevice->getDescription() == "Legacy")
	{
		log(LogTarget::ERR, LEGACY_DEVICE_ERR, "");
		return false;
	}

	if (IDebugManager* db_man = singleDevice->getDebugManager())
	{
		db_man->setPollingManager(this->mPollingManager);

		clock_control.ccModule = db_man->getClockModuleSetting();
		clock_control.ccGeneralCLK = db_man->getClockControlSetting();
		lastCycleCount = static_cast<uint32_t>(db_man->getCycleCounterValue());
	}

	std::map<enum CONFIG_MODE, int32_t>::const_iterator it = config_settings.begin();

	// Apply all configuration parameters	
	for ( ; it != config_settings.end(); ++it)
	{
		this->Configure(it->first, it->second);
	}

	const DeviceInfo* info = DeviceDb::Database().getDeviceInfo(setId);

	if (info && !info->warning.empty() && WarningFactory::instance())
	{
		WarningFactory::instance()->message(MESSAGE_LEVEL_T::MSPDS_MESSAGE_LEVEL_WARNING, info->warning.c_str());
	}

	return this->Device(setId, buffer, count);
}


bool DLL430_OldApiV3::Device(int32_t localDeviceId, uint8_t* buffer, int32_t count)
{
	if (buffer == 0)
	{
		log(LogTarget::ERR, PARAMETER_ERR, "");
		return false;
	}

	//use smaller value as limit for memcpy
	count = min((int32_t)sizeof(DEVICE_T), count);

	DEVICE_T data;
	::memset(&data, 0, sizeof(data));
	data.endian = 0xaa55;
	data.id = (uint16_t)(localDeviceId & 0xFFFF);

	if (localDeviceId > DeviceDb::Database().getMaxId())
	{
		log(LogTarget::ERR, PARAMETER_ERR, "");
		::memcpy(buffer, data.buffer, count);
		return false;
	}

	const DeviceInfo* info = DeviceDb::Database().getDeviceInfo(localDeviceId);

	if (!info)
	{
		return false;
	}

	::strncpy((char*)data.string, info->description.c_str(), sizeof(data.string) - 1);

	uint16_t usbRamStart = 0;
	uint16_t usbRamEnd = 0;
	int ramAreaCounter = 0;
	int infoAreaCounter = 0;
	data.structSize = count;

	for (const auto& it : info->memoryLayout)
	{
		const MemoryInfo& mem = it.second;
		const unsigned start = mem.start;
		const unsigned end = start + mem.size - 1;

		if (mem.name == MemoryArea::Main)
		{
			data.main32BitStart = start;
			data.main32BitEnd = end;
			data.mainStart = static_cast<uint16_t>(start);
			data.mainEnd = end;			
			data.mainSegmentSize = mem.segmentSize;
		}
		else if (mem.name == MemoryArea::Info && infoAreaCounter == 0)
		{
			data.info32BitStart = start;
			data.info32BitEnd = end;
			data.infoStart = static_cast<uint16_t>(start);
			data.infoEnd = static_cast<uint16_t>(end);

			if (data.info32BitEnd)
			{
				//MSP32 case
				data.info32BitSegmentSize = mem.segmentSize;
			}
			else
			{
				//MSP430 case
				data.info32BitSegmentSize = 0;
			}

			++infoAreaCounter;
		}
		else if (mem.name == MemoryArea::Bsl)
		{	
			data.bsl32BitStart = start;
			data.bsl32BitEnd = end;
			data.bslStart = static_cast<uint16_t>(start);
			data.bslEnd = static_cast<uint16_t>(end);
		}
		else if (mem.name == MemoryArea::Lcd)
		{
			data.lcdStart = static_cast<uint16_t>(start);
			data.lcdEnd = static_cast<uint16_t>(end);
		}
		else if (mem.name == MemoryArea::Ram && ramAreaCounter == 0)
		{

			data.ram32BitStart = start;
			data.ram32BitEnd = end;
			data.ramStart = static_cast<uint16_t>(start);
			data.ramEnd = static_cast<uint16_t>(end);
			++ramAreaCounter;
		}
		else if (mem.name == MemoryArea::Ram && ramAreaCounter == 1)
		{	
			data.ram32BitBandStart = start;
			data.ram32BitBandEnd = end;
			data.ram2Start = static_cast<uint16_t>(start);
			data.ram2End = static_cast<uint16_t>(end);			
			++ramAreaCounter;
		}
		else if (mem.name == MemoryArea::Info && infoAreaCounter == 1)
		{
			data.infoSecond32BitStart = start;
			data.infoSecond32BitEnd = end;
			++infoAreaCounter;
		}

		else if (mem.name == MemoryArea::UsbRam)
		{
			usbRamStart = static_cast<uint16_t>(start);
			usbRamEnd = static_cast<uint16_t>(end);
		}
	}
	if (usbRamStart > 0 && usbRamStart < data.ramStart)
	{
		data.ramStart = usbRamStart;
	}

	if (usbRamEnd > 0 && usbRamEnd > data.ramEnd)
	{
		data.ramEnd = usbRamEnd;
	}
	data.clockControl = info->clockInfo.clockControl;
	if (data.clockControl == GCC_STANDARD_I)
	{
		data.clockControl = GCC_STANDARD;
	}

	data.emulation = info->eem;
	if (data.emulation < 9)
	{
		typedef uint16_t arr[9];
		data.nBreakpoints = arr{0, 2, 3, 8, 2, 3, 5, 8, 6 }[data.emulation];
		data.nRegTrigger = arr{0, 0, 0, 2, 0, 1, 1, 2 , 0 }[data.emulation];
		data.nCombinations = arr{0, 2, 3, 8, 2, 4, 6, 1, 0 }[data.emulation];
		data.nBreakpointsOptions = arr{0, 0, 0, 1, 1, 1, 1, 1, 0 }[data.emulation];
		data.nBreakpointsReadWrite = arr{0, 0, 0, 1, 1, 1, 1, 1, 2 }[data.emulation];
		data.nBreakpointsDma = arr{0, 0, 0, 1, 1, 1, 1, 1,0 }[data.emulation];
		data.TrigerMask = arr{ 0, 0, 0, 1, 0, 0, 0, 1, 0 }[data.emulation];
		data.nRegTriggerOperations = arr{ 0, 0, 0, 1, 0, 1, 1, 1, 1 }[data.emulation];
		data.nStateStorage = arr{ 0, 0, 0, 8, 0, 0, 0, 8,  0}[data.emulation];
		data.nCycleCounter = arr{ 0, 0, 0, 0, 1, 1, 1, 2, 1 }[data.emulation];
		data.nCycleCounterOperations = arr{ 0, 0, 0, 0, 0, 0, 0, 1, 1 }[data.emulation];
		data.nSequencer = arr{ 0, 0, 0, 1, 0, 0, 1, 3, 0 }[data.emulation];
	}
	data.vccMinOp = info->voltageInfo.vccMin;
	data.vccMaxOp = info->voltageInfo.vccMax;
	data.hasTestVpp = info->voltageInfo.testVpp;
	data.HasFramMemroy = info->features.hasFram;

	if (singleDevice)
	{
		data.jtagId=singleDevice->getJtagId();
		data.deviceIdPtr=singleDevice->getDeviceIdPtr();
		data.eemVersion=singleDevice->getEemVersion();

		if (data.jtagId == 0x95)
		{
			data.cpuArch = CPU_ARCH_ORIGINAL;
		}
		else if (jtagIdIsXv2(data.jtagId))
		{
			data.cpuArch = CPU_ARCH_XV2;
		}
		else if (data.jtagId == 0x77)
		{
			data.cpuArch = CPU_ARCH_ARM;
		}
		else
		{
			data.cpuArch = (info->bits == 20) ? CPU_ARCH_X : CPU_ARCH_ORIGINAL;
		}

		// fill device_t structure with EmulationManager specific information
		try
		{
			this->singleDevice->getEmulationManager()->fillEMInfo(&data);
		}
		catch(const EM_Exception&)
		{

		}
	}
	::memcpy(buffer, data.buffer, count);
	return true;
}

bool DLL430_OldApiV3::VCC(int32_t voltage)
{
	if (voltage > 0xFFFF)
	{
		log(LogTarget::ERR, PARAMETER_ERR, "");
		return false;
	}

	if (!this->handle)
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}

	IConfigManager* cm = this->handle->getConfigManager();
	if (!cm)
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}

	if (voltage == 0 || ( voltage >= 1800 && voltage <= 3600 ))
	{
		if (!cm->setDeviceVcc(static_cast<uint16_t>(voltage & 0xFFFF)))
		{
			log(LogTarget::ERR, VCC_ERR, "");
			return false;
		}
	}
	else
	{
		log(LogTarget::ERR, PARAMETER_ERR, "");
		return false;
	}
	return true;
}

bool DLL430_OldApiV3::GetCurVCCT(int32_t* voltage)
{
	if (!this->handle)
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}

	IConfigManager* cm = this->handle->getConfigManager();
	if (!cm)
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}
	if (voltage)
		*voltage = static_cast<int32_t>(cm->getDeviceVcc());
	return true;
}

bool DLL430_OldApiV3::GetExtVoltage(int32_t* voltage, int32_t* state)
{
	if (!this->handle)
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}

	IConfigManager* cm = this->handle->getConfigManager();
	if (!cm)
	{
		log(LogTarget::ERR, VCC_ERR, "");
		return false;
	}

	const uint16_t extVoltage = cm->getExternalVcc();

	if (voltage)
		*voltage = static_cast<int32_t>(cm->getExternalVcc());

	if (state)
	{
		if (extVoltage < 1000)
		{ //1V
			*state = NO_EX_POWER;
		}
		else if (extVoltage < 1700)
		{ //1.7V
			*state = LOW_EX_POWER;
		}
		else if (extVoltage < 5600)
		{ //5.6V
			*state = EX_POWER_OK;
		}
		else
		{
			*state = HIGH_EX_POWER;
		}
	}
	return true;
}

bool DLL430_OldApiV3::Erase(int32_t type, int32_t address, int32_t length)
{
	IConfigManager* cm = this->handle ? this->handle->getConfigManager() : nullptr;
	if (!cm)
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}
	// If an FRAM Device is password protected the Mailbox erase is not functional.
	// Make sure that password and reset vector are erase before BootCode execution.
	if (type == ERASE_ALL || type == ERASE_MAIN || type == ERASE_TOTAL)
	{
		// Do not execute erase when in probeState for FRAM devices
		if (singleDevice && singleDevice->hasFram() && cm->atProbeStateEnabled())
			return true;

		if (singleDevice && singleDevice->hasFram())
		{
			IMemoryManager* mm = singleDevice->getMemoryManager();
			if (!mm->erase(0xFF80, 0xFF80 + 0x80 - 1, true))
			{
				log(LogTarget::ERR, ERASE_ERR, "");
				return false;
			}
			cm->setPassword("");
		}
	}
	if ((type == ERASE_TOTAL || type == ERASE_USER_CODE) && devArchitecture != MSP432_M4)
	{
		const uint16_t eraseKey = (type == ERASE_TOTAL) ? MASS_ERASE_MODE : MAIN_ERASE_MODE;

		// execute total erase in SBW2 Mode
		Configure(INTERFACE_MODE, SPYBIWIRE_IF);
		if (!cm->jtagErase(eraseKey))
		{
			// execute total erase in SBW4 Mode
			Configure(INTERFACE_MODE, SPYBIWIREJTAG_IF);
			if (!cm->jtagErase(eraseKey))
			{
				log(LogTarget::ERR, ERASE_ERR, "");
				return false;
			}
		}
		if (singleDevice)
		{
			try
			{
				singleDevice->getEmulationManager()->rewriteConfiguration();
				if (!singleDevice->reset())
				{
					log(LogTarget::ERR, RESET_ERR, "");
					return false;
				}
			}
			catch (const EM_Exception&) {/*Nothing to do*/}
		}
		return true;
	}

	// MSP432 total erase
	if (type == ERASE_TOTAL  && devArchitecture == MSP432_M4)
	{
		if (singleDevice)
		{
			if (!singleDevice->magicPatternSend(0))
			{
				log(LogTarget::ERR, ERASE_ERR, "");
				return false;
			}
			this_thread::sleep_for(chrono::seconds(1));
			return true;
			
		}
		else
		{
			this->OpenDevice("Device Unknown", "", 0, 0, 0);
			if (singleDevice && !singleDevice->magicPatternSend(0))
			{
				log(LogTarget::ERR, ERASE_ERR, "");
				return false;
			}
			if (singleDevice)
			{
				this_thread::sleep_for(chrono::seconds(1));
				return true;
			}
		}
	}
	

	if (singleDevice==nullptr)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	bool success = false;

	try
	{
		IMemoryManager* mm = singleDevice->getMemoryManager();
		switch (type)
		{
		case ERASE_SEGMENT:
			if (!mm->erase(address, address+length-1))
			{
				log(LogTarget::ERR, ERASE_ERR, "");
				return false;
			}
			break;

		case ERASE_ALL:
			if (!mm->erase())
			{
				log(LogTarget::ERR, ERASE_ERR, "");
				return false;
			}
			break;

		case ERASE_MAIN:
			{
				MemoryArea* area = mm->getMemoryArea(MemoryArea::Main);
				if (!area || !area->erase())
				{
					log(LogTarget::ERR, ERASE_ERR, "");
					return false;
				}
			}
			break;
		}

		success = true;
	}
	catch (const EM_Exception& e)
	{
		log(LogTarget::ERR, e.errorCode(), e.what());
	}

	return success;
}

bool DLL430_OldApiV3::Memory(int32_t address, uint8_t* buf, int32_t count, int32_t rw)
{
	bool status = true;

	if (singleDevice==nullptr)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	IMemoryManager* mm = singleDevice->getMemoryManager();

	if (rw == WRITE)
	{
		try
		{
			SoftwareBreakpointManagerPtr swbpMan = singleDevice->getEmulationManager()->getSoftwareBreakpoints()->getSwbpManager();
			swbpMan->patchMemoryWrite(address, buf, count);
		}
		catch (const EM_Exception&)
		{
			// ignore
		}

		status = mm->write(address, buf, count) && mm->sync();
	}
	else if (rw == READ)
	{
		//Prefill buffer with pattern for vacant memory
		bool oddAddress = ((address % 2) != 0);
		for (int byte = 0; byte < count; ++byte)
		{
			buf[byte] = oddAddress ? 0x3f : 0xff;
			oddAddress = !oddAddress;
		}

		status = mm->read(address, buf, count) && mm->sync();

		try
		{
			SoftwareBreakpointManagerPtr swbpMan = singleDevice->getEmulationManager()->getSoftwareBreakpoints()->getSwbpManager();
			swbpMan->patchMemoryRead(address, buf, count);
		}
		catch (const EM_Exception&)
		{
			//ignore
		}

	}
	else if (rw == OVERWRITE)
	{
		try
		{
			SoftwareBreakpointManagerPtr swbpMan = singleDevice->getEmulationManager()->getSoftwareBreakpoints()->getSwbpManager();
			swbpMan->patchMemoryWrite(address, buf, count);
		}
		catch (const EM_Exception&)
		{
			// ignore
		}

		status = mm->overwrite(address, buf, count) && mm->sync();
	}
	else
	{
		log(LogTarget::ERR, PARAMETER_ERR, "");
		return false;
	}

	if (status==false)
	{
		switch (mm->getLastError())
		{
		case MEMORY_READ_ERROR: log(LogTarget::ERR, READ_MEMORY_ERR, ""); break;
		case MEMORY_WRITE_ERROR: log(LogTarget::ERR, WRITE_MEMORY_ERR, ""); break;
		case MEMORY_LOCKED_ERROR: log(LogTarget::ERR, BSL_MEMORY_LOCKED_ERR, ""); break;
		case MEMORY_UNLOCK_ERROR: log(LogTarget::ERR, UNLOCK_BSL_ERR, ""); break;
		default:
			if (rw == WRITE)
				log(LogTarget::ERR, WRITE_MEMORY_ERR, "");
			else
				log(LogTarget::ERR, READ_MEMORY_ERR, "");
		}
	}
	return status;
}

bool DLL430_OldApiV3::Secure()
{
	bool success = false;
	try
	{
		success = singleDevice && singleDevice->secure();
		if (!success)
		{
			log(LogTarget::ERR, BLOW_FUSE_ERR, "");
		}
	}
	catch (const ERROR_CODE& error)
	{
		log(LogTarget::ERR, error, "");
	}
	return success;
}

bool DLL430_OldApiV3::ReadOutFile(int32_t address, int32_t length, const char* filename, int32_t fileType)
{
	// parameter check
	if ((address < 0) || (length < 1) || (filename == nullptr))
	{
		log(LogTarget::ERR, PARAMETER_ERR, "");
		return false;
	}

	if (singleDevice==nullptr)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	// first: read mem
	IMemoryManager * m_man = singleDevice->getMemoryManager();
	if (!m_man)
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}

	std::vector<uint8_t> buffer(length);

	if (!Memory(address, &buffer[0], length, READ))
	{
		log(LogTarget::ERR, READ_MEMORY_ERR, "");
		return false;
	}

	const FileType type = (fileType == 2) ? INTEL_HEX : TI_TXT;

	try
	{
		const MemoryContent data((uint32_t)address, &buffer[0], buffer.size());
		FileWriter::create(filename, type)->write(data);
	}
	catch (const DLL430_Exception& e)
	{
		log(LogTarget::ERR, e.errorCode(), e.what());
		return false;
	}

	return true;
}

bool DLL430_OldApiV3::ProgramFile(const char* filename, int32_t eraseType, int32_t verifyMem)
{
	try
	{
		if (singleDevice == nullptr)
			throw DLL430_Exception(NO_DEVICE_ERR);

		IMemoryManager* mm = singleDevice->getMemoryManager();
		if (mm == nullptr)
			throw DLL430_Exception(NO_DEVICE_ERR);

		MemoryContent ff;
		FileReader::create(filename)->read(&ff);

		if ((eraseType == ERASE_ALL) || (eraseType == ERASE_MAIN))
		{
			if (!this->Erase(eraseType, 0, 0))
				throw DLL430_Exception(ERASE_ERR);
		}

		for (DataSegment& segment : ff.segments)
		{
			if (!Memory(segment.startAddress, &segment.data[0], static_cast<int32_t>(segment.data.size()), WRITE))
				throw DLL430_Exception(WRITE_MEMORY_ERR);

			if (verifyMem)
			{
				if (!VerifyMem(segment.startAddress, static_cast<int32_t>(segment.data.size()), &segment.data[0]))
					throw DLL430_Exception(VERIFY_ERR);
			}
		}
	}
	catch (const DLL430_Exception& e)
	{
		log(LogTarget::ERR, e.errorCode(), e.what());
		return false;
	}
	return true;
}

bool DLL430_OldApiV3::VerifyFile(const char* filename)
{
	try
	{
		if (singleDevice == nullptr)
			throw DLL430_Exception(NO_DEVICE_ERR);

		MemoryContent ff;
		FileReader::create(filename)->read(&ff);

		for (DataSegment& segment : ff.segments)
		{
			if (!VerifyMem(segment.startAddress, static_cast<int32_t>(segment.data.size()), &segment.data[0]))
				throw DLL430_Exception(VERIFY_ERR);
		}
	}
	catch (const DLL430_Exception& e)
	{
		log(LogTarget::ERR, e.errorCode(), e.what());
		return false;
	}
	return true;
}

bool DLL430_OldApiV3::VerifyMem(int32_t StartAddr, int32_t Length, const uint8_t* DataArray)
{
	bool status = true;

	if (singleDevice==nullptr)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	if (!singleDevice->getMemoryManager()->verify(StartAddr, DataArray, Length))
	{
		log(LogTarget::ERR, VERIFY_ERR, "");
		status = false;
	}
	return status;
}

bool DLL430_OldApiV3::EraseCheck(int32_t StartAddr, int32_t Length)
{
	uint32_t addr = static_cast<uint32_t>(StartAddr);
	size_t len = static_cast<size_t>(Length);

	if (singleDevice==nullptr)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}
	IMemoryManager* mm = singleDevice->getMemoryManager();
	if (!mm)
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}
	if (!mm->verify(addr, 0, len))
	{
		log(LogTarget::ERR, VERIFY_ERR, "");
		return false;
	}
	return true;
}

bool DLL430_OldApiV3::Reset(int32_t method, int32_t execute, int32_t releaseJTAG)
{
	if (!method)
	{
		log(LogTarget::ERR, PARAMETER_ERR, "");
		return false;
	}

	if (singleDevice==nullptr)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	IDebugManager* db_man = singleDevice->getDebugManager();
	if (!db_man)
	{
		log(LogTarget::ERR, RESET_ERR, "");
		return false;
	}

	// Make sure the device is stopped (ie. under our control) before doing a reset
	this->State(0, 1, 0);

	db_man->pausePolling();

	if (this->debug.jtagReleased)
	{
		this->debug.jtagReleased = !db_man->reconnectJTAG();
	}

	bool resetSuccessful = false;
	try
	{
		bool restartJtag = false;

		IConfigManager* cm = handle ? handle->getConfigManager() : 0;

		/*-----------------------------------------------------------------------------*/
		if (!(method & FORCE_RESET) || (method & PUC_RESET))
		{
			resetSuccessful = singleDevice->reset();
		}

		if (!resetSuccessful && ((method & RST_RESET) || (method & VCC_RESET)))
		{
			if (devArchitecture == MSP432_M4)
			{
				resetSuccessful = singleDevice->reset(true);
			}
			else
			{
				resetSuccessful = cm && cm->reset(false, true, this->singleDevice->getJtagId(), this->singleDevice->checkHalId(ID_ResetXv2));
				restartJtag = true;
			}
		}

		/*-----------------get control over the device after RST--------------------*/
		if (restartJtag)
		{
			if (!cm || cm->start()!= 0x1)
			{
				log(LogTarget::ERR, RESET_ERR, "");
				return false;
			}

			resetEM();

			if (!singleDevice->reset())
			{
				log(LogTarget::ERR, RESET_ERR, "");
				return false;
			}
		}

		// if no reset was executed, raise error
		if (!resetSuccessful)
		{
			log(LogTarget::ERR, RESET_ERR, "");
			return false;
		}

		int32_t state;
		int32_t pCPUCycles;

		if (devArchitecture != MSP432_M4)
		{
			db_man->resumePolling();
			this_thread::sleep_for(chrono::milliseconds(500));
			db_man->pausePolling();
		}
		if (!State(&state, true, &pCPUCycles))
		{
			log(LogTarget::ERR, RESET_ERR, "");
			return false;
		}

		db_man->resetCycleCounterValue();
		lastCycleCount = 0;

		if (devCode == 0x5AA55AA5 || devCode == 0xDEADBABE || singleDevice->getJtagId() == 0x99 || devCode == 0xA55AA55A)
		{
			int32_t program_start = 0;
			const int32_t address = (devCode == 0xA55AA55A) ? 0x1C7E : 0xFFFE;

			this->Memory(address, (uint8_t*)&program_start, 2, READ);   // read address of program start
			this->Register(&program_start, PC, WRITE);
		}

		if (execute)
		{
			this->debug.state = RUNNING;

			if (!db_man->run(FreeRun, 0, releaseJTAG != 0))
			{
				log(LogTarget::ERR, RUN_ERR, "");
				return false;
			}

			if ( releaseJTAG != 0 )
			{
				//If we get here, run was successfully releasing JTAG
				debug.jtagReleased = true;
			}
		}
		else if (cm && releaseJTAG != 0 )
		{
			this->debug.state = RUNNING;
			debug.jtagReleased = true;
			cm->stop();
		}
	}
	catch (const EM_Exception& e)
	{
		log(LogTarget::ERR, e.errorCode(), e.what());
		resetSuccessful = false;
	}

	return resetSuccessful;
}


bool DLL430_OldApiV3::ExtRegisters(int32_t address, uint8_t* buffer, int32_t count, int32_t rw)
{
	return false;
}

bool DLL430_OldApiV3::Registers(int32_t* registers, int32_t mask, int32_t rw)
{
	if (singleDevice==nullptr)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	IMemoryManager * mm=singleDevice->getMemoryManager();
	if (mm==nullptr)
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}

	CpuRegisters* cpu = mm->getCpuRegisters();
	if (!cpu)
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}

	for (uint32_t i = 0; i < cpu->getSize(); ++i)
	{
		if ((mask & (1 << i)) != 0)
		{
			DLL430_OldApiV3::Register(&registers[i], i, rw);
		}
	}

	return true;
}

bool DLL430_OldApiV3::Register(int32_t* reg, int32_t regNb, int32_t rw)
{
	if (singleDevice==nullptr)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	CpuRegisters* cpu = singleDevice->getMemoryManager()->getCpuRegisters();
	if (!cpu)
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}

	if (rw == WRITE)
	{
		uint32_t tmp = (uint32_t)(*reg);
		if (!cpu->write(regNb, tmp))
		{
			log(LogTarget::ERR, WRITE_REGISTER_ERR, "");
			return false;
		}
	}
	else
	{
		uint32_t value;
		if (!cpu->read(regNb, &value))
		{
			log(LogTarget::ERR, READ_REGISTER_ERR, "");
			return false;
		}
		*reg = (int32_t)value;
	}

	return true;
}


bool DLL430_OldApiV3::Run(int32_t mode, int32_t releaseJTAG)
{
	if (this->debug.state == RUNNING)
	{
		log(LogTarget::ERR, THREAD_ACTIVE_ERR, "");
		return false;
	}
	if (singleDevice==nullptr)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}
	IDebugManager* db_man = singleDevice->getDebugManager();
	if (!db_man)
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}

	try
	{
		const uint32_t activeSwbps = singleDevice->getEmulationManager()->getSoftwareBreakpoints()->getSwbpManager()->numberOfActiveSoftwareTriggers();
		if (activeSwbps && (mode == FREE_RUN || releaseJTAG))
		{
			log (LogTarget::ERR, PARAMETER_ERR, "");
			return false;
		}
	}
	catch (const EM_Exception&) {}

	uint16_t controlType = FreeRun;

	if (mode==RUN_TO_BREAKPOINT)
	{
		controlType|=Stopped;
	}

	if ( singleDevice->hasLPMx5() && db_man->getLpmDebugging() && notifyCallback != 0 && !releaseJTAG)
	{
		db_man->activateJStatePolling(this);
	}

	if (mEnergyTraceManager && this->debug.state != LPMX5_WAKEUP)
	{
		mEnergyTraceManager->ResetEnergyTrace();
	}

	if (handle)
	{
		IConfigManager* cm = this->handle->getConfigManager();
		if (cm)
		{
			if (!cm->updateDisableInterruptsMode())
			{
				log(LogTarget::ERR, INTERNAL_ERR, "");
				return false;
			}
		}
	}

	switch (mode)
	{
	case FREE_RUN:
	case RUN_TO_BREAKPOINT:

		this->debug.state = RUNNING;
		lastCycleCount = 0;
		if ( !db_man->run(controlType, this, releaseJTAG != 0) )
		{
			log(LogTarget::ERR, RUN_ERR, "");
			return false;
		}

		if ( releaseJTAG != 0 )
		{
			//If we get here, run was successfully releasing JTAG
			debug.jtagReleased = true;
		}

		break;

	case SINGLE_STEP:
		{
			// Query LPMx.5 state: if the processor is in LPMx.5 don't even attempt to singleStep
			if (db_man->queryIsInLpm5State())
			{
				return true;
			}

			uint32_t cycles = 0;
			if (!db_man->singleStep(&cycles))
			{
				log(LogTarget::ERR, STEP_ERR, "");
				return false;
			}

			// Query LPMx.5 state again: if the processor stepped into LPMx.5, do not send the callback
			if (db_man->queryIsInLpm5State())
			{
				db_man->resumePolling();
			}
			else
			{
				this->debug.state = SINGLE_STEP_COMPLETE;
				if (this->debug.cb.func)
				{
					(*this->debug.cb.func)(
						this->debug.cb.ids.uiMsgIdSingleStep,
						0,
						cycles - lastCycleCount,
						this->debug.cb.clientHandle
					);
				}
			}
			break;
		}
	}
	return true;
}

bool DLL430_OldApiV3::State(int32_t* state, int32_t stop, int32_t* pCPUCycles)
{
	if (!stop)
	{
		log(LogTarget::ERR, PARAMETER_ERR, "");
		return false;
	}

	if (handle == nullptr)
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}

	bool jtagWasReleased = false;

	if (this->debug.jtagReleased)
	{
		if (IConfigManager* cm = handle->getConfigManager())
		{
			this->debug.jtagReleased = (cm->start() == 0);
		}
		jtagWasReleased = true;
	}

	if (state)
	{
		if (this->debug.state >= LPMX5_MODE)
		{
			*state = STOPPED;
		}
		else
		{
			*state = this->debug.state;
		}
	}

	if (singleDevice == nullptr)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}


	IDebugManager* db_man = singleDevice->getDebugManager();
	if (!db_man)
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}

	if (stop)
	{
		mPollingManager->pausePolling();

		if ((this->debug.state == RUNNING) || (this->debug.state == LPMX5_MODE) || (this->debug.state == LPMX5_WAKEUP))
		{	// If device is in LPMx.5, we must also perform the stop to re-gain control of the device
			if (!db_man->stop(jtagWasReleased))
			{
				log(LogTarget::ERR, INTERNAL_ERR, "");
				return false;
			}
		}
		this->debug.state = STOPPED;
	}

	if (pCPUCycles)
	{
		*pCPUCycles = (int32_t)db_man->getCycleCounterValue() - lastCycleCount;
	}
	lastCycleCount = (int32_t)db_man->getCycleCounterValue();

	// find condition to avoid stop while stopped
	if (state)
	{
		if (this->debug.state >= LPMX5_MODE)
		{
			*state = RUNNING;
		}
		else
		{
			*state = this->debug.state;
		}
	}
	return true;
}


bool DLL430_OldApiV3::CcGetClockNames(int32_t localDeviceId, EemGclkCtrl_t** CcClockNames)
{
	if (singleDevice==nullptr)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	IDebugManager * dm=singleDevice->getDebugManager();

	uint32_t size;
	char ** list = dm->getClockStrings(&size);

	if ((list == nullptr) || (size != 32))
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}

	*CcClockNames=(EemGclkCtrl_t*)list;

	return true;
}

bool DLL430_OldApiV3::CcGetModuleNames(int32_t localDeviceId, EemMclkCtrl_t** CcModuleNames)
{
	if (singleDevice==nullptr)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	IDebugManager * dm=singleDevice->getDebugManager();

	uint32_t size;
	char ** list = dm->getModuleStrings(&size);

	if ( (list == nullptr) || (size != 32) )
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}
	
	*CcModuleNames=(EemMclkCtrl_t*)list;
	
	return true;
}


bool DLL430_OldApiV3::EEM_Init(
	MSP430_EVENTNOTIFY_FUNC callback,
	int32_t clientHandle,
	const MessageID_t* pMsgIdBuffer
)
{
	if (singleDevice==nullptr)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	bool success = false;

	IDebugManager * dm=singleDevice->getDebugManager();

	this->debug.cb.func = callback;
	this->debug.cb.clientHandle = clientHandle;
	this->debug.cb.ids.uiMsgIdBreakpoint = pMsgIdBuffer->uiMsgIdBreakpoint;
	this->debug.cb.ids.uiMsgIdSingleStep = pMsgIdBuffer->uiMsgIdSingleStep;
	this->debug.cb.ids.uiMsgIdStorage = pMsgIdBuffer->uiMsgIdStorage;
	this->debug.cb.ids.uiMsgIdState = pMsgIdBuffer->uiMsgIdState;
	this->debug.cb.ids.uiMsgIdWarning = pMsgIdBuffer->uiMsgIdWarning;
	this->debug.cb.ids.uiMsgIdCPUStopped = pMsgIdBuffer->uiMsgIdCPUStopped;

	try
	{
		singleDevice->getEmulationManager()->writeConfiguration();
		lastCycleCount = 0;

		dm->initEemRegister();

		success = true;
	}
	catch (const EM_Exception& e)
	{
		log(LogTarget::ERR, e.errorCode(), e.what());
	}

	return success;
}


bool DLL430_OldApiV3::softwareTriggerInRangeExists(uint32_t start, uint32_t end, BpRangeAction_t inOut) const
{
	for (const auto& params : bp_storage)
	{
		const uint32_t address = (uint32_t)params.second.lAddrVal;
		const bool isInside = (address >= start && address <= end);
		const bool isOutside = (address <= start || address >= end);

		if (params.second.bpMode == BP_SOFTWARE && ((inOut == BP_INSIDE && isInside) || (inOut == BP_OUTSIDE && isOutside)))
			return true;
	}
	return false;
}

bool DLL430_OldApiV3::rangeTriggerIncludingAddressExists(uint32_t address) const
{
	for (const auto& params : bp_storage)
	{
		const BpParameter_t& bp = params.second;
		const uint32_t start = (uint32_t)bp.lAddrVal;
		const uint32_t end = (uint32_t)bp.lRangeEndAdVa;
		const bool isInside = (address >= start && address <= end);
		const bool isOutside = (address <= start || address >= end);

		if (bp.bpMode == BP_RANGE && ((bp.bpRangeAction == BP_INSIDE && isInside) || (bp.bpRangeAction == BP_OUTSIDE && isOutside)))
			return true;
	}
	return false;
}

bool DLL430_OldApiV3::hardwareTriggerAtAddressExists(uint32_t address) const
{
	for (const auto& params : bp_storage)
	{
		const BpParameter_t& bp = params.second;
		if (bp.bpAction == BP_BRK && bp.bpMode != BP_SOFTWARE)
		{
			if ((bp.bpType == BP_MAB || bp.bpMode == BP_CODE) && bp.lAddrVal == address)
				return true;

			if (bp.bpMode == BP_RANGE && bp.lRangeEndAdVa == address)
				return true;
		}
	}
	return false;
}

bool DLL430_OldApiV3::softwareTriggerAtAddressExists(uint32_t address) const
{
	for (const auto& params : bp_storage)
	{
		const BpParameter_t& bp = params.second;
		if (bp.bpMode == BP_SOFTWARE && bp.lAddrVal == address)
			return true;
	}
	return false;
}

bool DLL430_OldApiV3::triggerConflictsWithExistingTrigger(const BpParameter_t* bpBuffer) const
{
	const bool breakWithSwbpAtOrPreceding = bpBuffer->bpAction == BP_BRK &&
											(softwareTriggerAtAddressExists(bpBuffer->lAddrVal) ||
											softwareTriggerAtAddressExists(bpBuffer->lAddrVal-2));

	if (bpBuffer->bpMode == BP_SOFTWARE)
	{
		return softwareTriggerAtAddressExists(bpBuffer->lAddrVal) ||
				hardwareTriggerAtAddressExists(bpBuffer->lAddrVal) ||
				hardwareTriggerAtAddressExists(bpBuffer->lAddrVal+2) ||
				rangeTriggerIncludingAddressExists(bpBuffer->lAddrVal);
	}

	if (bpBuffer->bpMode == BP_CODE || bpBuffer->bpType == BP_MAB)
	{
		if (breakWithSwbpAtOrPreceding)
			return true;
	}

	if (bpBuffer->bpMode == BP_RANGE && bpBuffer->bpType == BP_MAB)
	{
		return bpBuffer->bpAction == BP_BRK &&
			   softwareTriggerInRangeExists(bpBuffer->lAddrVal, bpBuffer->lRangeEndAdVa, bpBuffer->bpRangeAction);
	}

	return false;
}

bool DLL430_OldApiV3::criticalRrcmInstructionAt(uint32_t address)
{
	uint8_t buffer[2] = {0};
	Memory(address, buffer, 2, READ);

	const uint16_t value = ((uint16_t)buffer[1] << 8) | buffer[0];

	//pattern is 0x0?40 or 0x0?50
	return (value & 0xf0ef) == 0x40;
}

static bool checkFetchCondition(AccessType bpAccess)
{
	if (static_cast<AccessType>(BP_FETCH) == bpAccess || static_cast<AccessType>(BP_FETCH_HOLD) == bpAccess)
	{
		return true;
	}
	return false;
}

TriggerConditionPtr DLL430_OldApiV3::triggerConditionFromBpParameter(EmulationManagerPtr emuManager, const BpParameter_t* bpBuffer)
{
	TriggerConditionManagerPtr tcManager = emuManager->getTriggerConditionManager();

	TriggerConditionPtr triggerCondition;

	switch (bpBuffer->bpMode)
	{
	case BP_CODE:
		triggerCondition = tcManager->createInstructionAddressCondition(bpBuffer->lAddrVal);
		break;

	case BP_RANGE:
		if (bpBuffer->lAddrVal > bpBuffer->lRangeEndAdVa || bpBuffer->bpType == BP_REGISTER)
			throw EM_TriggerParameterException();

		if (bpBuffer->bpType == BP_MDB)
		{
			triggerCondition = tcManager->createDataRangeCondition(bpBuffer->lAddrVal, bpBuffer->lRangeEndAdVa,
																	0xffffffff, 0xffffffff,
																	(AccessType)bpBuffer->bpAccess,
																	bpBuffer->bpRangeAction == BP_OUTSIDE);
		}
		else
		{
			triggerCondition = tcManager->createAddressRangeCondition(bpBuffer->lAddrVal, bpBuffer->lRangeEndAdVa,
																		0xffffffff, 0xffffffff,
																		(AccessType)bpBuffer->bpAccess,
																		bpBuffer->bpRangeAction == BP_OUTSIDE);
		}
		break;

	case BP_COMPLEX:
		if (bpBuffer->bpType == BP_REGISTER)
		{
			triggerCondition = tcManager->createRegisterCondition((uint8_t)bpBuffer->lReg, bpBuffer->lAddrVal,
																	bpBuffer->lMask, (ComparisonOperation)bpBuffer->bpOperat);
		}
		else
		{
			if (bpBuffer->bpType == BP_MDB)
			{
				if (bpBuffer->bpCondition == BP_NO_COND)
				{
					triggerCondition = tcManager->createDataValueCondition(bpBuffer->lAddrVal, bpBuffer->lMask,
						(AccessType)bpBuffer->bpAccess,
						(ComparisonOperation)bpBuffer->bpOperat);
				}
				else
				{
					triggerCondition = tcManager->createDataAddressValueCondition(bpBuffer->lCondMdbVal,
						bpBuffer->lAddrVal, bpBuffer->lCondMask, bpBuffer->lMask,
						(AccessType)bpBuffer->bpCondAccess, (AccessType)bpBuffer->bpAccess,
						(ComparisonOperation)bpBuffer->bpCondOperat, (ComparisonOperation)bpBuffer->bpOperat, (DataSize)bpBuffer->bpCondValSize);
				}
			}
			else
			{
				if (checkFetchCondition((AccessType)bpBuffer->bpAccess))
				{
					triggerCondition = tcManager->createInstructionAddressCondition(bpBuffer->lAddrVal, bpBuffer->lMask,
						(AccessType)bpBuffer->bpAccess,
						(ComparisonOperation)bpBuffer->bpOperat);
				}
				else
				{
					triggerCondition = tcManager->createDataAddressCondition(bpBuffer->lAddrVal, bpBuffer->lMask,
						(AccessType)bpBuffer->bpAccess,
						(ComparisonOperation)bpBuffer->bpOperat);
				}	
				if (bpBuffer->bpCondition == BP_COND)
				{
					DataValueConditionPtr dataCond = tcManager->createDataValueCondition(bpBuffer->lCondMdbVal, bpBuffer->lCondMask,
						(AccessType)bpBuffer->bpCondAccess,
						(ComparisonOperation)bpBuffer->bpCondOperat);
					triggerCondition->combine(dataCond);
				}
			}
			
		}
		break;

	case BP_SOFTWARE:
		if (jtagIdIsXv2(singleDevice->getJtagId()) && criticalRrcmInstructionAt(bpBuffer->lAddrVal+2))
			throw EM_SwbpCriticalInstruction();

		triggerCondition = tcManager->createSoftwareTriggerCondition(bpBuffer->lAddrVal);
		break;

	default:
		throw EM_TriggerParameterException();
	}

	return triggerCondition;
}


void DLL430_OldApiV3::clearSoftwareTriggers()
{
	map<uint16_t, BpParameter_t>::iterator it = bp_storage.begin();
	while (it != bp_storage.end())
	{
		map<uint16_t, BpParameter_t>::iterator tmp = it++;
		if (tmp->second.bpMode == BP_SOFTWARE)
		{
			triggers.erase(tmp->first);
			bp_storage.erase(tmp);
		}
	}
}


void DLL430_OldApiV3::restoreSoftwareTriggers(map<uint16_t, BpParameter_t>& bpStorage)
{
	for (auto& bp : bpStorage)
	{
		if (bp.second.bpMode == BP_SOFTWARE)
		{
			uint16_t handle = bp.first;
			EEM_SetBreakpoint(&handle, &bp.second);
		}
	}
}


void DLL430_OldApiV3::addBreakpointsAndStorage(EmulationManagerPtr emuManager, TriggerConditionPtr triggerCondition,
								BpAction_t reactions, uint16_t handle)
{
	if (triggerCondition)
	{
		//Collect all triggers (including those without configured reactions)
		triggers[handle].set(triggerCondition);

		if (reactions & BP_BRK)
		{
			breakpoints[handle].set( emuManager->getBreakpointManager()->createBreakpoint(triggerCondition) );
		}

		if (reactions & BP_STO)
		{
			traceTriggers[handle].set(triggerCondition);
		}

		if (reactions & BP_CC)
		{
			counterTriggers[handle].set(triggerCondition);
		}
	}
}


void DLL430_OldApiV3::updateStorageReactions(EmulationManagerPtr emuManager)
{
	if (emuManager->hasTrace())
	{
		TracePtr trace = emuManager->getTrace();
		trace->clearTriggerConditions();

		for (const auto& entry : traceTriggers)
		{
			if (entry.second.inUse())
			{
				trace->addTriggerCondition(entry.second.value());
			}
		}
	}
}


void DLL430_OldApiV3::updateCounterReactions(EmulationManagerPtr emuManager)
{
	if (emuManager->hasCycleCounter())
	{
		CycleCounterPtr counter = emuManager->getCycleCounter();
		counter->clearTriggerConditions();

		for (const auto& entry : counterTriggers)
		{
			if (entry.second.inUse())
			{
				counter->addTriggerCondition(entry.second.value());
			}
		}
	}
}


void DLL430_OldApiV3::resetEM()
{
	if (singleDevice)
	{
		SoftwareBreakpointManagerPtr oldSwbpMan = singleDevice->getEmulationManager()->getSoftwareBreakpoints()->getSwbpManager();
		singleDevice->getEmulationManager()->reset();

		if (IDebugManager* db_man = singleDevice->getDebugManager())
			db_man->initEemRegister();

		singleDevice->getEmulationManager()->getSoftwareBreakpoints()->getSwbpManager()->importInstructionTable(*oldSwbpMan.get());

		//Reenable software breakpoints if previously active
		if (config_settings[SOFTWARE_BREAKPOINTS] == ENABLE)
		{
			try
			{
				singleDevice->getEmulationManager()->getSoftwareBreakpoints()->enable();
				singleDevice->getEmulationManager()->writeConfiguration();
			}
			catch (const EM_Exception&) {} //ignore
		}
	}

	//We keep the software triggers
	map<uint16_t, BpParameter_t>::iterator it = bp_storage.begin();
	while (it != bp_storage.end())
	{
		map<uint16_t, BpParameter_t>::iterator tmp = it++;
		if (tmp->second.bpMode != BP_SOFTWARE)
		{
			triggers.erase(tmp->first);
			bp_storage.erase(tmp->first);
		}
	}

	traceTriggers.clear();
	counterTriggers.clear();
	breakpoints.clear();
	triggerCombinations.clear();
	watchedVariables.clear();
	varWatch_state = VW_DISABLE;
}


bool DLL430_OldApiV3::EEM_SetBreakpoint(uint16_t* bpHandle, const BpParameter_t* bpBuffer)
{
	if (singleDevice==nullptr)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	if ( bpHandle == nullptr || bpBuffer == nullptr ||
		(bpBuffer->bpMode == BP_CLEAR && *bpHandle == 0) ||
		(bpBuffer->bpMode == BP_SOFTWARE && bpBuffer->bpAction != BP_BRK))
	{
		log(LogTarget::ERR, PARAMETER_ERR, "");
		return false;
	}

	const bool modifyingSwbp = (*bpHandle != 0) && (bp_storage[*bpHandle].bpMode == BP_SOFTWARE);
	const bool settingSwbp = (bpBuffer->bpMode == BP_SOFTWARE);

	if (deviceIsRunning() && (modifyingSwbp || settingSwbp))
	{
		log(LogTarget::ERR, TARGET_RUNNING_ERR, "");
		return false;
	}


	bool success = false;

	uint16_t handle = *bpHandle;

	try
	{
		prepareEemAccess();
		EmulationManagerPtr emuManager = singleDevice->getEmulationManager();

		if (handle == 0)
		{
			handle = 1;
			while (triggers[handle].inUse())
				++handle;
		}
		else
		{
			//Special case handling if trigger is part of a combination
			for (auto& combination : triggerCombinations)
			{
				vector<uint16_t> combinedHandles = combination.second;
				if (find(combination.second.begin(), combination.second.end(), handle) != combination.second.end())
				{
					uint16_t combinationHandle = combination.first;

					//If part of a combination is removed, simply break up the combination
					if (bpBuffer->bpMode == BP_CLEAR)
					{
						EEM_SetCombineBreakpoint(CB_CLEAR, static_cast<uint16_t>(combinedHandles.size()), &combinationHandle, &combinedHandles[0]);
					}
					//If the breakpoint is changed, trigger a reconstruction of the combination with new parameters
					else
					{
						combination.second.clear();
						bp_storage[handle] = *bpBuffer;
						return EEM_SetCombineBreakpoint(CB_SET, static_cast<uint16_t>(combinedHandles.size()), &combinationHandle, &combinedHandles[0]);
					}
				}
			}

			triggers[handle].clear();
			breakpoints[handle].clear();
			traceTriggers[handle].clear();
			counterTriggers[handle].clear();
			bp_storage.erase(handle);
		}

		if (bpBuffer->bpMode != BP_CLEAR)
		{
			if (triggerConflictsWithExistingTrigger(bpBuffer))
				throw EM_TriggerConflictException();

  			TriggerConditionPtr triggerCondition = triggerConditionFromBpParameter(emuManager, bpBuffer);

			if (bpBuffer->bpMode != BP_SOFTWARE)
			{
				addBreakpointsAndStorage(emuManager, triggerCondition, bpBuffer->bpAction, handle);
			}
			else
			{
				triggers[handle].set(triggerCondition);
			}

			bp_storage[handle] = *bpBuffer;
			*bpHandle = handle;
		}

		//Reconfigure all trace triggers
		updateStorageReactions(emuManager);
		updateCounterReactions(emuManager);

		emuManager->writeConfiguration();
		lastCycleCount = 0;

		success = true;
	}
	catch (const EM_Exception& e)
	{
		log(LogTarget::ERR, e.errorCode(), e.what());
	}

	return success;
}


bool DLL430_OldApiV3::EEM_GetBreakpoint(uint16_t wBpHandle, BpParameter_t* pBpDestBuffer)
{
	if (pBpDestBuffer==nullptr)
	{
		log(LogTarget::ERR, PARAMETER_ERR, "");
		return false;
	}

	map<uint16_t, BpParameter_t>::iterator it = bp_storage.find(wBpHandle);

	if ( it == bp_storage.end() )
	{
		log(LogTarget::ERR, PARAMETER_ERR, "");
		return false;
	}

	*pBpDestBuffer = it->second;

	return true;
}


bool DLL430_OldApiV3::EEM_SetCombineBreakpoint(CbControl_t control, uint16_t count, uint16_t* cbHandle, const uint16_t* bpHandle)
{
	if (singleDevice == nullptr)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	if (bpHandle == nullptr || cbHandle == nullptr || (control == CB_SET && count < 2))
	{
		log(LogTarget::ERR, PARAMETER_ERR, "");
		return false;
	}

	bool success = false;

	try
	{
		prepareEemAccess();
		EmulationManagerPtr emuManager = singleDevice->getEmulationManager();

		if (control == CB_SET)
		{
			//Check that no combined trigger is a software breakpoint
			for (int i = 0; i < count; ++i)
			{
				if (bp_storage[ bpHandle[i] ].bpMode == BP_SOFTWARE)
				{
					log(LogTarget::ERR, PARAMETER_ERR, "");
					return false;
				}
			}

			//Combination handle is handle of first combined trigger
			*cbHandle = bpHandle[0];
			if (!triggerCombinations[*cbHandle].empty())
				return false;

			//Combine: replace existing breakpoints/watched variables with placeholders, create new one based on bp_storage
			for (int i = 0; i < count; ++i)
			{
				triggers[ bpHandle[i] ].reserveSlot();
				breakpoints[ bpHandle[i] ].reserveSlot();
				traceTriggers[ bpHandle[i] ].reserveSlot();
				counterTriggers[bpHandle[i]].reserveSlot();
			}

			TriggerConditionPtr triggerCondition = triggerConditionFromBpParameter(emuManager, &bp_storage[bpHandle[0]]);

			for (int i = 1; i < count; ++i)
			{
				triggerCondition->combine( triggerConditionFromBpParameter(emuManager, &bp_storage[bpHandle[i]]) );
			}

			addBreakpointsAndStorage(emuManager, triggerCondition, bp_storage[bpHandle[0]].bpAction, bpHandle[0]);

			triggerCombinations[*cbHandle] = vector<uint16_t>(bpHandle, bpHandle + count);
		}

		//Destroy combination, recreate separate breakpoints/watched variables
		if (control == CB_CLEAR)
		{
			vector<uint16_t> combinedHandles = triggerCombinations[*cbHandle];
			triggerCombinations[*cbHandle].clear();

			for (uint16_t handle : combinedHandles)
			{
				BpParameter_t* param = &bp_storage[handle];
				EEM_SetBreakpoint(&handle, param);
			}
		}

		//Reconfigure all storage/counter triggers
		updateStorageReactions(emuManager);
		updateCounterReactions(emuManager);

		emuManager->writeConfiguration();
		lastCycleCount = 0;

		success = true;
	}
	catch (const EM_Exception& e)
	{
		log(LogTarget::ERR, e.errorCode(), e.what());
	}

	return success;
}

bool DLL430_OldApiV3::EEM_GetCombineBreakpoint(uint16_t cbHandle, uint16_t* count, uint16_t* bpHandle)
{
	if (singleDevice == nullptr)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	if ((count == nullptr) || (bpHandle == nullptr))
	{
		log(LogTarget::ERR, PARAMETER_ERR, "");
		return false;
	}

	*count = 0;

	for (int32_t id : triggerCombinations[cbHandle])
	{
		bpHandle[(*count)++] = (uint16_t)id;
	}

	return true;
}

bool DLL430_OldApiV3::EEM_SetTrace(const TrParameter_t* trBuffer)
{
	if (singleDevice==nullptr)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	bool success = false;

	try
	{
		prepareEemAccess();
		EmulationManagerPtr emuManager = singleDevice->getEmulationManager();
		TracePtr trace = emuManager->getTrace();

		trace_storage.trAction = trBuffer->trAction;
		trace_storage.trControl = trBuffer->trControl;
		trace_storage.trMode = trBuffer->trMode;

		switch (trBuffer->trAction)
		{
		case TR_FETCH:
			trace->setStoreOnInstructionFetch();
			break;

		case TR_ALL_CYCLE:
			trace->setStoreOnClock();
			break;

		default: break;
		}


		switch (trBuffer->trMode)
		{
		case TR_HISTORY:
			trace->setStartOnTrigger(false);
			trace->setStopOnTrigger(true);
			trace->setStoreContinuously();
			break;

		case TR_FUTURE:
			trace->setStartOnTrigger(true);
			trace->setStopOnTrigger(false);
			trace->setStoreUntilFull();
			break;

		case TR_SHOT:
			trace->setStartOnTrigger(false);
			trace->setStopOnTrigger(false);
			trace->setStoreUntilFull();
			break;

		case TR_COLLECT:
			trace->setStartOnTrigger(false);
			trace->setStopOnTrigger(false);
			trace->setStoreOnTrigger();
			trace->setStoreUntilFull();
			break;

		default: break;
		}


		switch (trBuffer->trControl)
		{
		case TR_ENABLE:
			trace->enable();
			singleDevice->getDebugManager()->startStoragePolling();
			break;

		case TR_DISABLE:
			trace->disable();
			singleDevice->getDebugManager()->stopStoragePolling();
			break;

		default: break;
		}

		//Changed settings always require a reset
		trace->reset();

		emuManager->writeConfiguration();

		success = true;
	}
	catch (const EM_Exception& e)
	{
		log(LogTarget::ERR, e.errorCode(), e.what());
	}

	return success;
}

bool DLL430_OldApiV3::EEM_GetTrace(TrParameter_t* trDestBuffer)
{
	trDestBuffer->trAction = trace_storage.trAction;
	trDestBuffer->trControl = trace_storage.trControl;
	trDestBuffer->trMode = trace_storage.trMode;
	return true;
}

bool DLL430_OldApiV3::EEM_ReadTraceBuffer(TraceBuffer_t* destTraceBuffer)
{
	uint32_t count = 8;
	return EEM_ReadTraceData(destTraceBuffer, &count);
}

bool DLL430_OldApiV3::EEM_ReadTraceData(TraceBuffer_t* destTraceBuffer, uint32_t *count)
{
	if (destTraceBuffer == nullptr || count == nullptr)
	{
		log(LogTarget::ERR, PARAMETER_ERR, "");
		return false;
	}

	if (singleDevice==nullptr)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	bool success = false;

	try
	{
		const TraceBuffer& traceBuffer = singleDevice->getEmulationManager()->getTrace()->getTraceData();

		TraceBuffer::const_reverse_iterator it = traceBuffer.rbegin();
		*count = min(*count, (uint32_t)traceBuffer.size());

		for (size_t i = 0; (i < *count) && (it != traceBuffer.rend()); ++i, ++it)
		{
			destTraceBuffer[i].lTrBufMAB = it->mab;
			destTraceBuffer[i].lTrBufMDB = it->mdb;
			destTraceBuffer[i].wTrBufCNTRL = it->ctl;
		}
		success = true;
	}
	catch (const EM_Exception& e)
	{
		log(LogTarget::ERR, e.errorCode(), e.what());
	}

	return success;
}


bool DLL430_OldApiV3::EEM_RefreshTraceBuffer(void)
{
	if (singleDevice==nullptr)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	bool success = false;

	try
	{
		prepareEemAccess();
		EmulationManagerPtr emuManager = singleDevice->getEmulationManager();
		emuManager->getTrace()->reset();

		emuManager->writeConfiguration();

		success = true;
	}
	catch (const EM_Exception& e)
	{
		log(LogTarget::ERR, e.errorCode(), e.what());
	}

	return success;
}

bool DLL430_OldApiV3::EEM_SetVariableWatch(VwEnable_t vwEnable)
{
	if (singleDevice==nullptr)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	bool success = false;

	try
	{
		prepareEemAccess();

		EmulationManagerPtr emuManager = singleDevice->getEmulationManager();
		VariableWatchPtr varWatch = emuManager->getVariableWatch();

		if (vwEnable == VW_ENABLE)
		{
			varWatch->enable();
			singleDevice->getDebugManager()->startStoragePolling();
		}
		else
		{
			varWatch->disable();
			singleDevice->getDebugManager()->stopStoragePolling();
			watchedVariables.clear();
			vw_storage.clear();
		}
		varWatch_state = vwEnable;

		emuManager->writeConfiguration();

		success = true;
	}
	catch (const EM_Exception& e)
	{
		log(LogTarget::ERR, e.errorCode(), e.what());
	}

	return success;
}


bool DLL430_OldApiV3::EEM_GetVariableWatch(VwEnable_t* vwEnable, VwResources_t* vwDestBuffer)
{
	memset(vwDestBuffer, 0, 8*sizeof(*vwDestBuffer));

	*vwEnable = varWatch_state;

	std::map<uint16_t, VwResources_t>::const_iterator it = vw_storage.begin();
	for (int i = 0; i < 8 && it != vw_storage.end(); ++i, ++it)
	{
		vwDestBuffer[i].vwHandle = it->second.vwHandle;
		vwDestBuffer[i].lAddr = it->second.lAddr;
		vwDestBuffer[i].vwDataType = it->second.vwDataType;
	}

	return true;
}


bool DLL430_OldApiV3::EEM_SetVariable(uint16_t* vwHandle, const VwParameter_t* vwBuffer)
{
	// Check provided parameter
	if (vwHandle == nullptr || vwBuffer == nullptr)
	{
		log(LogTarget::ERR, PARAMETER_ERR, "");
		return false;
	}

	if (singleDevice==nullptr)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	bool success = false;

	try
	{
		prepareEemAccess();

		EmulationManagerPtr emuManager = singleDevice->getEmulationManager();
		VariableWatchPtr varWatch = emuManager->getVariableWatch();

		if (vwBuffer->vwControl == VW_CLEAR)
		{
			boost::lock_guard<boost::mutex> lock(watchedVariablesMutex);
			watchedVariables.erase(*vwHandle);
			vw_storage.erase(*vwHandle);
		}

		if (vwBuffer->vwControl == VW_SET)
		{
			boost::lock_guard<boost::mutex> lock(watchedVariablesMutex);

			//Start at 0x10, so VW events are easily distinguished from Trace events
			*vwHandle = 0x10;
			while (watchedVariables[*vwHandle])
				++(*vwHandle);

			uint32_t bits = 8;
			if (vwBuffer->vwDataType == VW_16)
				bits = 16;

			if (vwBuffer->vwDataType == VW_32)
				bits = 32;

			TriggerConditionManagerPtr tcManager = emuManager->getTriggerConditionManager();
			watchedVariables[*vwHandle] = varWatch->createWatchedVariable(vwBuffer->lAddr, bits, tcManager);

			VwResources_t resource = {*vwHandle, vwBuffer->lAddr, vwBuffer->vwDataType};
			vw_storage[*vwHandle] = resource;
		}

		emuManager->writeConfiguration();

		success = true;
	}
	catch (const EM_Exception& e)
	{
		log(LogTarget::ERR, e.errorCode(), e.what());
	}

	return success;
}

bool DLL430_OldApiV3::EEM_SetClockControl(const CcParameter2_t* pCcBuffer)
{
	if (pCcBuffer==nullptr)
	{
		log(LogTarget::ERR, PARAMETER_ERR, "");
		return false;
	}
	if (singleDevice==nullptr)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}
	// Check clock control emulation on device
	if ( singleDevice->getDebugManager()->getClockControl() < 1 )
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}

	bool success = false;

	try
	{
		prepareEemAccess();

		uint16_t cmp_general = singleDevice->getDebugManager()->getGeneralClockDefaultSetting();
		uint32_t cmp_module = singleDevice->getDebugManager()->getClockModuleDefaultSetting();

		if (pCcBuffer->ccControl == CC_ENABLE)
		{
			cmp_general = pCcBuffer->ccGeneralCLK;
			cmp_module = pCcBuffer->ccModule;
		}

		const uint8_t clockControlType = singleDevice->getDebugManager()->getClockControl();
		bool needReset = false;
		
		//Only applies to Cpu/CpuX devices
		if ( (clock_control.ccGeneralCLK != cmp_general) && !jtagIdIsXv2(singleDevice->getJtagId()))
		{
			singleDevice->getEmulationManager()->writeRegister(GENCLKCTRL, cmp_general);
			clock_control.ccGeneralCLK = cmp_general;
			needReset = true;
		}

		if ( (clock_control.ccModule != cmp_module) && (clockControlType == GccExtended))
		{
			singleDevice->getEmulationManager()->writeRegister(MODCLKCTRL0, cmp_module);
			clock_control.ccModule = cmp_module;
			needReset = true;
		}

		clock_control.ccControl = pCcBuffer->ccControl;

		// PUC reset
		if (needReset && !singleDevice->reset())
		{
			log(LogTarget::ERR, RESET_ERR, "");
			return false;
		}

		success = true;
	}
	catch (const EM_Exception& e)
	{
		log(LogTarget::ERR, e.errorCode(), e.what());
	}

	return success;
}

bool DLL430_OldApiV3::EEM_GetClockControl(CcParameter2_t* pCcDestBuffer)
{
	if (pCcDestBuffer==nullptr)
	{
		log(LogTarget::ERR, PARAMETER_ERR, "");
		return false;
	}

	memcpy(pCcDestBuffer, &clock_control, sizeof(CcParameter2_t));

	return true;
}


bool DLL430_OldApiV3::EEM_SetSequencer(const SeqParameter_t* seqBuffer)
{
	if (singleDevice==nullptr)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	bool success = false;

	try
	{
		prepareEemAccess();

		EmulationManagerPtr emuManager = singleDevice->getEmulationManager();

		SequencerPtr sequencer = emuManager->getSequencer();

		if (seqBuffer->seqControl == SEQ_DISABLE)
		{
			sequencer->disable();
		}
		else
		{
			sequencer->enable();
		}

		sequencer_control = *seqBuffer;

		sequencer->clearAllTransitions();
		sequencer->clearResetTrigger();
		sequencer->clearReactions();

		for (uint8_t i = 0; i < 4; ++i)
		{
			if (seqBuffer->wHandleStateX[i] != 0)
			{
				sequencer->setTransition(i, 0, seqBuffer->seqNextStateX[i], triggers[ seqBuffer->wHandleStateX[i] ].value());
			}

			if (seqBuffer->wHandleStateY[i] != 0)
			{
				sequencer->setTransition(i, 1, seqBuffer->seqNextStateY[i], triggers[ seqBuffer->wHandleStateY[i] ].value());
			}
		}

		//Configure sequencer to reset on specified trigger
		if (seqBuffer->wHandleRstTrig != 0)
		{
			sequencer->setResetTrigger(triggers[seqBuffer->wHandleRstTrig].value());
		}

		//Set sequencer actions on reaching final state
		if (seqBuffer->bpAction & BP_BRK)
		{
			sequencer->addReaction(TR_BREAK);
		}
		if (seqBuffer->bpAction & BP_STO)
		{
			sequencer->addReaction(TR_STATE_STORAGE);
		}


		emuManager->writeConfiguration();

		success = true;
	}
	catch (const EM_Exception& e)
	{
		log(LogTarget::ERR, e.errorCode(), e.what());
	}

	return success;
}

bool DLL430_OldApiV3::EEM_GetSequencer(SeqParameter_t* seqDestBuffer)
{
	if (seqDestBuffer == nullptr)
	{
		log(LogTarget::ERR, PARAMETER_ERR, "");
		return false;
	}

	*seqDestBuffer = sequencer_control;
	return true;
}

bool DLL430_OldApiV3::EEM_ReadSequencerState(SeqState_t* pSeqState)
{
	if (singleDevice == nullptr)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	if (sequencer_control.seqControl == SEQ_DISABLE)
	{
		log(LogTarget::ERR, SEQ_ENABLE_ERR, "");
		return false;
	}

	bool success = false;

	try
	{
		prepareEemAccess();
		*pSeqState = (SeqState)singleDevice->getEmulationManager()->getSequencer()->readCurrentState();

		success = true;
	}
	catch (const EM_Exception& e)
	{
		log(LogTarget::ERR, e.errorCode(), e.what());
	}

	return success;
}

bool DLL430_OldApiV3::EEM_SetCycleCounterMode(CycleCounterMode_t mode)
{
	if (singleDevice == nullptr)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	bool success = false;

	try {
		if (mode == CYC_MODE_ADVANCED && !singleDevice->getEmulationManager()->hasCycleCounter())
		{
			throw EM_Exception(FEATURE_NOT_SUPPORTED, "Target has no hardware cycle counter");
		}

		if (IDebugManager* dbgMan = singleDevice->getDebugManager())
		{
			dbgMan->enableLegacyCycleCounter(mode == CYC_MODE_BASIC);

			singleDevice->getEmulationManager()->writeConfiguration();
		}

		success = true;
	}
	catch (const EM_Exception& e)
	{
		log(LogTarget::ERR, e.errorCode(), e.what());
	}

	return success;
}

void DLL430_OldApiV3::checkCycleCounterConflict(uint32_t wCounter) const
{
	const IDebugManager* dbgMan = singleDevice->getDebugManager();
	if (dbgMan && dbgMan->legacyCycleCounterEnabled() && wCounter == 0)
	{
		throw EM_Exception(CYCLE_COUNTER_CONFLICT, "");
	}
}

bool DLL430_OldApiV3::EEM_ConfigureCycleCounter(uint32_t wCounter, CycleCounterConfig_t pCycConfig)
{
	if (singleDevice == nullptr)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	bool success = false;

	try
	{
		checkCycleCounterConflict(wCounter);
		prepareEemAccess();

		CycleCounterPtr cc = singleDevice->getEmulationManager()->getCycleCounter();
		cc->setCountMode(wCounter, (CounterCountMode)pCycConfig.countMode);
		cc->setStartMode(wCounter, (CounterStartMode)pCycConfig.startMode);
		cc->setStopMode(wCounter, (CounterStopMode)pCycConfig.stopMode);
		cc->setClearMode(wCounter, (CounterClearMode)pCycConfig.clearMode);

		singleDevice->getEmulationManager()->writeConfiguration();

		success = true;
	}
	catch (const EM_Exception& e)
	{
		log(LogTarget::ERR, e.errorCode(), e.what());
	}

	return success;
}

bool DLL430_OldApiV3::EEM_ReadCycleCounterValue(uint32_t wCounter, uint64_t* value)
{
	if (singleDevice == nullptr)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	if (value == nullptr)
	{
		log(LogTarget::ERR, PARAMETER_ERR, "");
		return false;
	}

	bool success = false;

	try
	{
		checkCycleCounterConflict(wCounter);

		CycleCounterPtr cc = singleDevice->getEmulationManager()->getCycleCounter();
		prepareEemAccess();
		cc->readCounter(wCounter);
		*value = cc->getCounterValue(wCounter);

		success = true;
	}
	catch (const EM_Exception& e)
	{
		log(LogTarget::ERR, e.errorCode(), e.what());
	}

	return success;
}

bool DLL430_OldApiV3::EEM_WriteCycleCounterValue(uint32_t wCounter, uint64_t value)
{
	if (singleDevice == nullptr)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	bool success = false;

	try
	{
		checkCycleCounterConflict(wCounter);
		prepareEemAccess();
		singleDevice->getEmulationManager()->getCycleCounter()->setCounterValue(wCounter, value);
		singleDevice->getEmulationManager()->writeConfiguration();

		success = true;
	}
	catch (const EM_Exception& e)
	{
		log(LogTarget::ERR, e.errorCode(), e.what());
	}

	return success;
}

bool DLL430_OldApiV3::EEM_ResetCycleCounter(uint32_t wCounter)
{
	if (singleDevice == nullptr)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	bool success = false;

	try
	{
		checkCycleCounterConflict(wCounter);
		prepareEemAccess();

		singleDevice->getEmulationManager()->getCycleCounter()->resetCounter(wCounter);
		singleDevice->getEmulationManager()->writeConfiguration();

		success = true;
	}
	catch (const EM_Exception& e)
	{
		log(LogTarget::ERR, e.errorCode(), e.what());
	}

	return success;
}

bool DLL430_OldApiV3::FET_SelfTest(int32_t count, uint8_t* buffer)
{
	return true;
}

bool DLL430_OldApiV3::FET_SetSignals(int32_t SigMask, int32_t SigState)
{
	log(LogTarget::ERR, INTERFACE_SUPPORT_ERR, "");
	return false;
}

bool DLL430_OldApiV3::FET_Reset(void)
{
	log(LogTarget::ERR, INTERFACE_SUPPORT_ERR, "");
	return false;
}

bool DLL430_OldApiV3::FET_I2C(int32_t address, uint8_t* buffer, int32_t count, int32_t rw)
{
	log(LogTarget::ERR, INTERFACE_SUPPORT_ERR, "");
	return false;
}

bool DLL430_OldApiV3::FET_EnterBootloader(void)
{
	log(LogTarget::ERR, INTERFACE_SUPPORT_ERR, "");
	return false;
}

bool DLL430_OldApiV3::FET_ExitBootloader(void)
{
	log(LogTarget::ERR, INTERFACE_SUPPORT_ERR, "");
	return false;
}

bool DLL430_OldApiV3::FET_GetFwVersion(int32_t* version)
{
	if (handle==nullptr)
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}

	if (version)
	{
		*version = this->handle->getConfigManager()->getHalVersion().get();
	}
	return true;
}


bool DLL430_OldApiV3::FET_GetHwVersion(uint8_t** version, int32_t* count)
{
	if (handle==nullptr)
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}

	if ((version==nullptr)||(count==nullptr))
	{
		log(LogTarget::ERR, PARAMETER_ERR, "");
		return false;
	}

	const std::vector<uint8_t>* hwId = this->handle->getHwVersion();

	static uint8_t v[4] = {0, 0, 0, 0};

	const bool valid = (hwId->size() >= 4);
	if (!valid)
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
	}
	else
	{
		v[0]=hwId->at(0);
		v[1]=hwId->at(1);
		v[2]=hwId->at(2);
		v[3]=hwId->at(3);
	}
	*version = v;
	*count = 4;

	return valid;
}

bool DLL430_OldApiV3::FET_FwUpdate(
	const char* lpszFileName,
	DLL430_FET_NOTIFY_FUNC callback,
	int32_t clientHandle)
{
	this->errNum = 0;
	this->clientHandle=clientHandle;

	try
	{
		UpdateNotifyCallback cbFunction;
		if (callback != nullptr)
		{
			cbFunction = bind(callback, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, clientHandle);
		}

		const uint32_t countHidDevices =( HidUpdateManager::countHidDevices(MSPBSL_EZ_FET_USB_PID) + HidUpdateManager::countHidDevices(MSPBSL_MSP_FET_USB_PID));
		// HID FET recovery handling -------------------------------------------------------------------------------
		if (countHidDevices == 1)
		{	// just one HID FET was detected
			bool returnValue = HidUpdateManager().hid_firmWareUpdate(lpszFileName, cbFunction);
			if (!returnValue)
			{
				log(LogTarget::ERR, RECOVERY_FAILED, "");
				return false;
			}
			return true;
		}

		if (countHidDevices > 1)
		{
			// do not start if more than one FET need recovery
			log(LogTarget::ERR, RECOVERY_MULTIPLE_UIF, "");
			return false;
		}
		// HID FET recovery handling  END ---------------------------------------------------------------------

		if (handle==nullptr)
		{
			log(LogTarget::ERR, INTERNAL_ERR, "");
			return false;
		}
		IConfigManager* config = handle->getConfigManager();

		int32_t version = -1;
		const string portName = this->handle->getCurrentPortName();
		vector<char> port(portName.begin(), portName.end());
		port.push_back(0);
		std::string serialNumberFET = this->manager->getPortElement(handle->getCurrentPortName())->serial;

		bool coreUpdate = false;
		bool updateSuccess = config->firmWareUpdate(lpszFileName, cbFunction, &coreUpdate);

		//Perform up to two retries if not core update
		for (int retries = 2; retries > 0 && !updateSuccess && !coreUpdate; --retries)
		{
			//Initialize will invalidate config manager
			Initialize(&port[0], &version);
			config = handle ? handle->getConfigManager() : nullptr;
			updateSuccess = config && config->firmWareUpdate(lpszFileName, cbFunction, &coreUpdate);
		}

		if (!updateSuccess)
		{
			// if core update was not successful
			if (coreUpdate)
			{
				log(LogTarget::ERR, UPDATE_CORE_ERR, "");
				return false;
			}// if normal update was not successfully
			else
			{
				log(LogTarget::ERR, UPDATE_MODULE_ERR, "");
				return false;
			}
		}
		// if core update was successful, run module updates after it.(just eZ-FET & MSP-FET).
		if (coreUpdate)
		{
			if (!this->Initialize(&port[0], &version))
			{
				std::string UpdateLog;
				UpdateLog.append("\n\n\n ------------------------!this->Initialize(&port[0], &version) #1--------------------------- \n");
				UpdateLog.append("\n");
				UpdateLog.append(this->Error_String(this->Error_Number()));
				UpdateLog.append("\n");
				{
#if defined(_WIN32) || defined(_WIN64)
					char binaryPath[256] = { 0 };
					uint32_t pathLength = 0;

					pathLength = GetModuleFileName(0, binaryPath, sizeof(binaryPath));
					while (pathLength > 0 && binaryPath[pathLength - 1] != '\\')
					{
						--pathLength;
					}
					string logfile = string(binaryPath, pathLength) + "Update.log";
#else
					string logfile = "Update.log";
#endif
					UpdateLog.append("\n---------------------Firmware upate end--------------------------\n");

					std::ofstream(logfile.c_str(), std::ios::app | std::ios::out) << UpdateLog;
				}
			}

			if (!handle)
			{
				PortInfo *portinfo = nullptr;
				int timeout = 100;
				while (timeout-- && !portinfo)
				{
					this_thread::sleep_for(chrono::milliseconds(100));
					portinfo = this->manager->getPortElementBySN(serialNumberFET);
				}

				if (!portinfo)
				{
					std::string portname = portinfo->name;
					this->Initialize(portname.c_str(), &version);
				}
				if (!handle)
				{
					std::string UpdateLog;
					UpdateLog.append("\n\n\n ------------------------!this->Initialize(&port[0], &version) #2--------------------------- \n");
					UpdateLog.append("\n");
					UpdateLog.append(this->Error_String(this->Error_Number()));
					UpdateLog.append("\n");
					{
#if defined(_WIN32) || defined(_WIN64)
						char binaryPath[256] = { 0 };
						uint32_t pathLength = 0;

						pathLength = GetModuleFileName(0, binaryPath, sizeof(binaryPath));
						while (pathLength > 0 && binaryPath[pathLength - 1] != '\\')
						{
							--pathLength;
						}
						string logfile = string(binaryPath, pathLength) + "Update.log";
#else
						string logfile = "Update.log";
#endif
						UpdateLog.append("\n---------------------Firmware upate end--------------------------\n");

						std::ofstream(logfile.c_str(), std::ios::app | std::ios::out) << UpdateLog;
					}
				}
			}

			if (handle)
			{
				config = handle->getConfigManager();

				if (!config->firmWareUpdate(lpszFileName, cbFunction, &coreUpdate))
				{
					// if module update was not successful
					log(LogTarget::ERR, UPDATE_MODULE_ERR, "");
					return false;
				}
			}
			else
			{
				// if module update was not successful
				log(LogTarget::ERR, FET_UPDATE_FAILED, "");
				return false;
			}
		}
	}
	catch (const std::runtime_error& e)
	{
		if (string("DUMMY_FW_MANAGER_UPDATE") == e.what())
		{
			log(LogTarget::ERR, HARDWARE_STATE_UNKNOWN, "");
			return false;
		}
	}
	return true;
}

void DLL430_OldApiV3::HIL_ResetJtagTap()
{
	if (handle)
	{
		handle->sendHilCommand(HIL_CMD_RESET_JTAG_TAP);
	}
}

bool DLL430_OldApiV3::HIL_Configure(enum CONFIG_MODE mode, int32_t value)
{
	if (handle == nullptr)
	{
		log(LogTarget::ERR, INTERFACE_SUPPORT_ERR, "");
		return false;
	}

	bool retValue = false;

	switch (mode)
	{
		case INTERFACE_MODE:
		{
			enum INTERFACE_TYPE type = (enum INTERFACE_TYPE)value;
			switch (type)
			{
				case JTAG_IF:
					retValue = handle->sendHilCommand(HIL_CMD_CONFIGURE, JTAG_IF);
					break;
				case SPYBIWIRE_IF:
					retValue = handle->sendHilCommand(HIL_CMD_CONFIGURE, SPYBIWIRE_IF);
					break;
				case SPYBIWIREJTAG_IF:
					retValue = handle->sendHilCommand(HIL_CMD_CONFIGURE, SPYBIWIREJTAG_IF);
					break;
				case JTAG_MSP432:
					retValue = handle->sendHilCommand(HIL_CMD_CONFIGURE, JTAG_MSP432);
					break;
				case SWD_MSP432:
					retValue = handle->sendHilCommand(HIL_CMD_CONFIGURE, SWD_MSP432);
					break;
				default: break;
			}
		}
		break;
		default:
			break;
	}
	return retValue;
}

bool DLL430_OldApiV3::HIL_Open()
{
	if (singleDevice)
	{
		if (IDebugManager* db_man = singleDevice->getDebugManager())
		{
			// Stop polling, since we don't want it to interfere with low-level access
			db_man->pausePolling();
		}
	}

	const bool success = handle && handle->sendHilCommand(HIL_CMD_OPEN);
	if (!success)
		log(LogTarget::ERR, INTERNAL_ERR, "");

	return success;
}

bool DLL430_OldApiV3::HIL_Bsl()
{
	const bool success = handle && handle->sendHilCommand(HIL_CMD_BSL);
	if (!success)
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
	}

	return success;
}

bool DLL430_OldApiV3::HIL_Connect_Entry_State(int32_t value)
{
	const bool success = handle && handle->sendHilCommand(HIL_CMD_CONNECT, (uint32_t)value);
	if (!success)
		log(LogTarget::ERR, INTERNAL_ERR, "");

	return success;
}

void DLL430_OldApiV3::HIL_FuseCheck()
{
	if (handle)
	{
		handle->sendHilCommand(HIL_CMD_FUSE_CHECK, 0);
	}
}

bool DLL430_OldApiV3::HIL_TCLK(uint8_t value)
{
	if (handle)
	{
		return handle->sendHilCommand(HIL_CMD_TCLK, value);
	}
	return false;
}

bool DLL430_OldApiV3::HIL_Connect()
{
	const bool success = handle && handle->sendHilCommand(HIL_CMD_CONNECT, 0);
	if (!success)
		log(LogTarget::ERR, INTERNAL_ERR, "");

	return success;
}

bool DLL430_OldApiV3::HIL_Close(int32_t vccOff)
{
	const bool success = handle && handle->sendHilCommand(HIL_CMD_CLOSE, vccOff);
	if (!success)
		log(LogTarget::ERR, INTERNAL_ERR, "");

	return success;
}

bool DLL430_OldApiV3::HIL_TCK(int32_t state)
{
	const bool success = handle && handle->setJtagPin(JTAG_PIN_TCK, state != 0);
	if (!success)
		log(LogTarget::ERR, INTERNAL_ERR, "");

	return success;
}

bool DLL430_OldApiV3::HIL_TMS(int32_t state)
{
	const bool success = handle && handle->setJtagPin(JTAG_PIN_TMS, state != 0);
	if (!success)
		log(LogTarget::ERR, INTERNAL_ERR, "");

	return success;
}

bool DLL430_OldApiV3::HIL_TDI(int32_t state)
{
	const bool success = handle && handle->setJtagPin(JTAG_PIN_TDI, state != 0);
	if (!success)
		log(LogTarget::ERR, INTERNAL_ERR, "");

	return success;
}

bool DLL430_OldApiV3::HIL_RST(int32_t state)
{
	const bool success = handle && handle->setJtagPin(JTAG_PIN_RST, state != 0);
	if (!success)
		log(LogTarget::ERR, INTERNAL_ERR, "");

	return success;
}

bool DLL430_OldApiV3::HIL_TST(int32_t state)
{
	const bool success = handle && handle->setJtagPin(JTAG_PIN_TST, state != 0);
	if (!success)
		log(LogTarget::ERR, INTERNAL_ERR, "");

	return success;
}

uint64_t DLL430_OldApiV3::HIL_JTAG_IR(int32_t instruction)
{
	uint64_t retVal = (uint64_t)-1;
	if (handle)
		retVal = handle->sendJtagShift(HIL_CMD_JTAG_IR, instruction);

	return retVal;
}

uint64_t DLL430_OldApiV3::HIL_JTAG_IR4(int32_t instruction)
{
	uint64_t retVal = (uint64_t)-1;
	if (handle)
		retVal = handle->sendJtagShift(HIL_CMD_JTAG_IR4, instruction);

	return retVal;
}

uint64_t DLL430_OldApiV3::HIL_JTAG_DR(int64_t data, int32_t bitSize)
{
	uint64_t retVal = (uint64_t)-1;
	if (handle)
		retVal = handle->sendJtagShift(HIL_CMD_JTAG_DR, data, bitSize);

	return retVal;
}

uint64_t DLL430_OldApiV3::HIL_JTAG_IR_DR(uint32_t instruction, uint64_t data, uint32_t bits)
{
	uint64_t resultDR = (uint64_t)-1;
	if (FetHandle* fetHandle = dynamic_cast<FetHandle*>(handle))
	{
		JtagShifts shifts;

		if (fetHandle->send(shifts(HIL_CMD_JTAG_IR, instruction, 8)(HIL_CMD_JTAG_DR, data, bits)))
		{
			//IR shift result at position 0 as 64bit value if needed
			resultDR = shifts.getData().getOutputAt64(8);
		}
	}
	return resultDR;
}

bool DLL430_OldApiV3::HIL_DPACC(uint8_t address, uint32_t *data, uint16_t RdnWr)
{
	HalExecElement *el = new HalExecElement(ID_HilCommand);

	el->appendInputData32((uint32_t)HIL_CMD_DPACC);
	el->appendInputData32((uint32_t)address);
	el->appendInputData32(*data);
	el->appendInputData32((uint32_t)RdnWr);

	HalExecCommand cmd;
	cmd.elements.emplace_back(el);

	FetHandle* fetHandle = dynamic_cast<FetHandle*>(handle);

	bool result = false;
	if (fetHandle)
	{
		result = fetHandle->send(cmd);
		*data  = cmd.elements[0]->getOutputAt32(0);
	}

	return result;
}

bool DLL430_OldApiV3::HIL_APACC(uint8_t portNum, uint8_t address, uint32_t *data, uint16_t RdnWr)
{
	HalExecElement *el = new HalExecElement(ID_HilCommand);

	el->appendInputData32((uint32_t)HIL_CMD_APACC);
	el->appendInputData32((uint32_t)address + (((uint32_t)portNum) << 24));
	el->appendInputData32(*data);
	el->appendInputData32((uint32_t)RdnWr);

	HalExecCommand cmd;
	cmd.elements.emplace_back(el);

	FetHandle* fetHandle = dynamic_cast<FetHandle*>(handle);

	bool result = false;
	if (fetHandle)
	{
		result = fetHandle->send(cmd);
		*data  = cmd.elements[0]->getOutputAt32(0);
	}

	return result;
}

bool DLL430_OldApiV3::HIL_MEMAP(uint8_t portNum, uint32_t address, uint32_t *data, uint16_t RdnWr)
{
	HalExecElement *el = new HalExecElement(ID_HilCommand);

	el->appendInputData32((uint32_t)HIL_CMD_MEMAP);
	el->appendInputData32(address);
	el->appendInputData32(*data);
	el->appendInputData32((uint32_t)RdnWr | ((uint32_t)portNum << 24));

	HalExecCommand cmd;
	cmd.elements.emplace_back(el);

	FetHandle* fetHandle = dynamic_cast<FetHandle*>(handle);

	bool result = false;
	if (fetHandle)
	{
		result = fetHandle->send(cmd);
		*data  = cmd.elements[0]->getOutputAt32(0);
	}

	return result;
}

void DLL430_OldApiV3::execNotifyCallback(SYSTEM_EVENT_MSP event)
{
	// just executes the function, given by the IDE
	if (notifyCallback)
	{
		this->notifyCallback(event);
	}
}

void DLL430_OldApiV3::iNotifyCallback(uint32_t systemEventID)
{
	this->execNotifyCallback((enum SYSTEM_EVENT_MSP)systemEventID);
}

bool DLL430_OldApiV3::writeToExternalMemory()
{
	// PC  = @0xF8A2      Set to start of Generate Image routine
	// R11 = @0x1C7E      Set to addr in RAM Reset vector
	// R12 = RAM_START
	// R13 = 0x0000       Upper addr for start of SPI memory
	// R14 = 0x0002       write image starting at addr 2
	// R15 = RAM_LENGTH   (RAM_END-RAM_START)
	// SP  = 0x23FE       make space for return addr on stack
	// @23FE = @1C7E      make function return to program start
	IMemoryManager* mm = singleDevice ? singleDevice->getMemoryManager() : 0;

	CpuRegisters* cpu = mm ? mm->getCpuRegisters() : 0;

	if (!cpu)
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}

	const uint32_t WRITE_ERROR = 0xFD4C;

	uint32_t saveRegister[16] = {0x0};
	const uint32_t ramStart = 0x1C60;
	const uint32_t ramEnd = 0x2400;
	const uint32_t stackPtr = ramEnd - 2;
	const uint32_t stackReserve = 50;

	cpu->read(0, saveRegister, 16);

	uint8_t tmp[2] = {0};

	if ( !mm->read(0xF8A2, tmp, 2) || !mm->sync() )
	{
		log(LogTarget::ERR, READ_MEMORY_ERR, "");
		return false;
	}

	const uint32_t functionAddress = tmp[0] | (tmp[1] << 8);

	if ( !mm->read(0x1C7E, tmp, 2) || !mm->sync() )
	{
		log(LogTarget::ERR, READ_MEMORY_ERR, "");
		return false;
	}

	const uint32_t programStart = tmp[0] | (tmp[1] << 8);

	uint8_t loaderAddress[2] = {0xFE, 0xBC};
	if ( !mm->write(stackPtr, loaderAddress, 2) || !mm->sync() )
	{
		log(LogTarget::ERR, WRITE_MEMORY_ERR, "");
		return false;
	}

	cpu->write(0, functionAddress );	//Set PC to start of Generate Image routine
	cpu->write(1, stackPtr);
	cpu->write(11, programStart );
	cpu->write(12, ramStart);
	cpu->write(13, 0);
	cpu->write(14, 2);
	cpu->write(15, ramEnd - ramStart - stackReserve);

	this->handle->getConfigManager()->start();

	Run(FREE_RUN, true);					// start execution of loader code

	this_thread::sleep_for(chrono::seconds(15));

	int32_t state;
	int32_t pCPUCycles;
	State(&state, true, &pCPUCycles);

	uint32_t value;
	cpu->read(0, &value);

	cpu->write(0, saveRegister, 16);
	cpu->flushCache(); // write save register values into the msp

	return (value != WRITE_ERROR);
}

bool DLL430_OldApiV3::EnableEnergyTrace(const EnergyTraceSetup* setup, const EnergyTraceCallbacks* callbacks, EnergyTraceHandle* handle)
{
	if (!this->handle)
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}
	if (!this->handle->getConfigManager()->isEnergyTraceSupported())
	{
		log(LogTarget::ERR, ET_NOT_SUPPORTED, "");
		return false;
	}
	if (!handle)
	{
		log(LogTarget::ERR, PARAMETER_ERR, "");
		return false;
	}

	bool EA_Result = false;

	if (mEnergyTraceManager)
	{
		mPdSetup = *setup;

		mPdCallbacks = *callbacks;
		*handle = (EnergyTraceHandle*)this;

		// if ET7
		if (setup->ETMode == ET_PROFILING_ANALOG_DSTATE)
		{
			if (!singleDevice)
			{
				log(LogTarget::ERR, NO_DEVICE_ERR, "");
				return false;
			}
			if (!this->handle->getConfigManager()->ulpDebugEnabled() && singleDevice->getTargetArchitecture() == MSP430)
			{
				log(LogTarget::ERR, ET_NOT_SUPPORTED_ULP_DEBUG, "");
				return false;
			}
			// if ET7 && not Jstate capbile device return error.
			// For ARM devices PC register is used instead of Jstate
			if (!singleDevice->deviceSupportsEnergyTrace())
			{
				log(LogTarget::ERR, ET_NOT_SUPPORTED_DEVICE, "");
				return false;
			}
			// if ET7 && Jstate capbile device start ET
			else
			{
				EA_Result = mEnergyTraceManager->startEnergyTrace(this, setup->ETMode, setup->ETCallback, singleDevice);
			}
		}
		// if ET8 - start on every device
		else if (mPdSetup.ETMode == ET_PROFILING_ANALOG)
		{
			EA_Result = mEnergyTraceManager->startEnergyTrace(this, setup->ETMode, setup->ETCallback, singleDevice);
		}
		// default return error
		else
		{
			log(LogTarget::ERR, ET_NOT_SUPPORTED_DEVICE, "");
			return false;
		}
	}
	if (!EA_Result)
	{
		log(LogTarget::ERR, ET_NOT_SUPPORTED_DEVICE, "");
		return false;
	}
	return true;
}

bool DLL430_OldApiV3::ResetEnergyTrace(const EnergyTraceHandle handle)
{
	return mEnergyTraceManager->ResetEnergyTrace();
}

bool DLL430_OldApiV3::DisableEnergyTrace(const EnergyTraceHandle handle)
{
	bool retVal = false;

	if (mEnergyTraceManager)
	{
		mEnergyTraceManager->stopPolling();
		retVal = true;
	}
	boost::lock_guard<boost::mutex> callbackLock(callbackMutex);
	mPdCallbacks.pContext = 0;
	mPdCallbacks.pErrorOccurredFn = 0;
	mPdCallbacks.pPushDataFn = 0;
	return retVal;
}

void DLL430_OldApiV3::prepareEemAccess() const
{
	if (singleDevice && handle && handle->getConfigManager() &&
		(deviceIsRunning() && !singleDevice->eemAccessibleInLpm() && this->handle->getConfigManager()->ulpDebugEnabled()))
		throw EM_EemNotAccessibleException();
}

bool DLL430_OldApiV3::deviceIsRunning() const
{
	return (debug.state == RUNNING || debug.state == LPMX5_MODE);
}

bool DLL430_OldApiV3::InterfaceType(enum INTERFACE_TYPE* type) const
{
	IConfigManager *cm = this->handle->getConfigManager();
	if (!cm || !singleDevice)
		return false;
	*type = cm->getInterfaceMode(singleDevice->getTargetArchitecture());
	return true;
}

bool DLL430_OldApiV3::RegisterMessageCallback(MessageCallbackFn callback)
{
	return WarningFactory::instance()->RegisterMessageCallback(callback);
}
