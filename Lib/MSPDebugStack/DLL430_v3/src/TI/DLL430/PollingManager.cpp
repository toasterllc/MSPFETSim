/*
 * PollingManager.cpp
 *
 * Manages the various polling macros and their combinations.
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

#include "PollingManager.h"
#include "DeviceHandleMSP430.h"
#include "FetHandle.h"
#include <boost/thread.hpp>


using namespace TI::DLL430;


PollingManager::PollingManager(FetHandle* fetHandle)
	: mFetHandle(fetHandle)
	, mEnergyTraceMode(ET_POLLING_OFF)
	, mEtGatedMode(ET_POLLING_GATED_OFF)
{
	//Set base macros for polling types
	mPollingMacro[PT_BREAKPOINT] = ID_WaitForEem;
	mPollingMacro[PT_LPM] = ID_PollJStateReg;
	mPollingMacro[PT_STATE_STORAGE] = ID_WaitForStorage;

	//Mapping Energy Trace modes to polling macros
	mEtModeToMacro[ET_POLLING_ANALOG] = ID_PollJStateRegEt8;
	mEtModeToMacro[ET_POLLING_ANALOG_DSTATE] = ID_PollJStateReg;
	mEtModeToMacro[ET_POLLING_ANALOG_DSTATE_PC] = ID_PollDStatePCRegEt;

	//Initialize event handling
	mEventNotifier.setEventHandler( std::bind(&PollingManager::runEvent, this, std::placeholders::_1) );
	mEventNotifier.startProcessingEvents();
}

void PollingManager::shutdown()
{
	mEventNotifier.stopProcessingEvents();
}

bool PollingManager::startBreakpointPolling(const IDeviceHandle& devHandle)
{
	const hal_id macro = devHandle.checkHalId(ID_WaitForEem);

	//Breakpoint polling can turn itself off, which will set response id to 0
	//If polling automatically stopped, set to inactive and remove before restarting
	boost::lock_guard<boost::mutex> lock(mMutexActiveMacros);
	MacroTable::iterator it = mActiveMacros.find(macro);
	if (it != mActiveMacros.end() && it->second.cmd->getResponseId() == 0)
	{
		mPollingActive[PT_BREAKPOINT] = false;
		if (!removeMacro(macro))
		{
			return false;
		}
	}

	return startPolling(PT_BREAKPOINT, devHandle);
}

bool PollingManager::stopBreakpointPolling(const IDeviceHandle& devHandle)
{
	return stopPolling(PT_BREAKPOINT, devHandle);
}

bool PollingManager::startLpmPolling(const IDeviceHandle& devHandle)
{
	boost::lock_guard<boost::mutex> lock(mMutexActiveMacros);
	return startPolling(PT_LPM, devHandle);
}

bool PollingManager::stopLpmPolling(const IDeviceHandle& devHandle)
{
	return stopPolling(PT_LPM, devHandle);
}

bool PollingManager::startStateStoragePolling(const IDeviceHandle& devHandle)
{
	boost::lock_guard<boost::mutex> lock(mMutexActiveMacros);
	return startPolling(PT_STATE_STORAGE, devHandle);
}

bool PollingManager::stopStateStoragePolling(const IDeviceHandle& devHandle)
{
	return stopPolling(PT_STATE_STORAGE, devHandle);
}


bool PollingManager::startPolling(POLLING_TYPE type, const IDeviceHandle& devHandle)
{
	//Check if not already active, determine device specific macro and add to loop
	if (!mPollingActive[type])
	{
		const hal_id macro = devHandle.checkHalId(mPollingMacro[type]);
		mPollingActive[type] = true;
		return addMacro(macro);
	}
	return true;
}

bool PollingManager::stopPolling(POLLING_TYPE type, const IDeviceHandle& devHandle)
{
	//If active, determine device specific macro and disable
	boost::lock_guard<boost::mutex> lock(mMutexActiveMacros);
	if (mPollingActive[type])
	{
		const hal_id macro = devHandle.checkHalId(mPollingMacro[type]);
		mPollingActive[type] = false;
		return removeMacro(macro);
	}
	return true;
}


bool PollingManager::startEnergyTracePolling(EtPollingMode mode, EtGatedMode gating)
{
	//If currently off and have macro for polling mode, set configuration and add to loop
	if (mEnergyTraceMode == ET_POLLING_OFF)
	{
		const hal_id macroId = mEtModeToMacro[mode];
		if (macroId != 0)
		{
			mEnergyTraceMode = mode;
			mEtGatedMode = gating;
			return addMacro(macroId);
		}
	}
	//Only return error if trying to set a different mode without disabling first
	return (mEnergyTraceMode == mode);
}

bool PollingManager::stopEnergyTracePolling()
{
	boost::lock_guard<boost::mutex> lock(mMutexActiveMacros);
	const hal_id macroId = mEtModeToMacro[mEnergyTraceMode];
	if (macroId != 0)
	{
		mEnergyTraceMode = ET_POLLING_OFF;
		return removeMacro(macroId);
	}
	return false;
}


bool PollingManager::addMacro(hal_id macroId)
{
	//If already in loop, kill it first, so it will restart with new parameters
	Macro& macro = mActiveMacros[macroId];
	if ((macro.count++ > 0) && (macro.cmd->getResponseId() > 0))
	{
		if (!mFetHandle->kill(*macro.cmd))
		{
			return false;
		}
	}
	return addToLoop(macroId);
}

bool PollingManager::removeMacro(hal_id macroId)
{
	MacroTable::iterator it = mActiveMacros.find(macroId);
	if (it != mActiveMacros.end())
	{
		//Kill and restart with new parameters if still active
		//Otherwise remove from table of active macros
		if (it->second.cmd->getResponseId() > 0)
		{
			if (!mFetHandle->kill(*it->second.cmd))
			{
				return false;
			}
		}

		if (--it->second.count == 0)
		{
			mActiveMacros.erase(it);
		}
		else
		{
			return addToLoop(macroId);
		}
	}
	return true;
}

bool PollingManager::addToLoop(hal_id macroId)
{
	const uint16_t eemBpMask = 0x80;

	HalExecElement* el = nullptr;
	switch (macroId)
	{
	case ID_PollJStateReg:
	case ID_PollJStateReg20:
	case ID_PollJStateRegEt8:
	case ID_PollDStatePCRegEt:
	case ID_PollJStateRegFR57xx:
		//Parameters for jstate polling have been set by calling functions
		el = new HalExecElement(macroId);
		el->appendInputData16(0); //ForceSendState (only for direct execution, not loop)
		el->appendInputData16(mPollingActive[PT_BREAKPOINT]);
		el->appendInputData16(mPollingActive[PT_LPM]);
		el->appendInputData16(mEnergyTraceMode == ET_POLLING_ANALOG_DSTATE || mEnergyTraceMode == ET_POLLING_ANALOG_DSTATE_PC);
		el->appendInputData16((uint16_t)mEtGatedMode);
		break;

	case ID_WaitForEem:
		el = new HalExecElement(macroId);
		el->appendInputData16(eemBpMask);
		break;

	case ID_WaitForStorage:
	case ID_WaitForStorageX:
	case ID_WaitForDebugHaltArm:
		el = new HalExecElement(macroId);
		break;

	default: break;
	}

	if (el)
	{
		//Get/create command, configure and send
		HalExecCommand& cmd = *mActiveMacros[macroId].cmd;
		cmd.setCallBack( std::bind(&PollingManager::queueEvent, this, std::placeholders::_1), 0 );
		cmd.setAsyncMode(!mKillAfterEvent[macroId]);

		cmd.elements.clear();
		cmd.elements.emplace_back(el);
		return mFetHandle->getControl()->send(cmd);
	}
	return false;
}

uint8_t PollingManager::getResponseId(hal_id baseMacroId, const IDeviceHandle& devHandle) const
{
	MacroTable::const_iterator it = mActiveMacros.find( devHandle.checkHalId(baseMacroId) );
	return (it != mActiveMacros.end()) ? it->second.cmd->getResponseId() : 0;
}


void PollingManager::pauseStateStoragePolling(const IDeviceHandle& devHandle)
{
	pausePolling(PT_STATE_STORAGE, devHandle);
}

void PollingManager::resumeStateStoragePolling(const IDeviceHandle& devHandle)
{
	resumePolling(PT_STATE_STORAGE, devHandle);
}

void PollingManager::pausePolling(POLLING_TYPE type, const IDeviceHandle& devHandle)
{
	boost::lock_guard<boost::mutex> lock(mMutexActiveMacros);
	const hal_id macroId = mPollingMacro[type];
	if (const uint8_t id = getResponseId(macroId, devHandle))
	{
		mFetHandle->pauseLoopCmd(id);
	}
}

void PollingManager::resumePolling(POLLING_TYPE type, const IDeviceHandle& devHandle)
{
	boost::lock_guard<boost::mutex> lock(mMutexActiveMacros);
	const hal_id macroId = mPollingMacro[type];
	if (const uint8_t id = getResponseId(macroId, devHandle))
	{
		mFetHandle->resumeLoopCmd(id);
	}
}

void PollingManager::pausePolling()
{
	boost::lock_guard<boost::mutex> lock(mMutexActiveMacros);
	for (const MacroTable::value_type& entry : mActiveMacros)
	{
		//Don't pause analog energy trace macro
		if (entry.first == ID_PollJStateRegEt8)
		{
			continue;
		}

		if (const uint8_t id = entry.second.cmd->getResponseId())
		{
			mFetHandle->pauseLoopCmd(id);
		}
	}
}

void PollingManager::resumePolling()
{
	boost::lock_guard<boost::mutex> lock(mMutexActiveMacros);
	for (const MacroTable::value_type& entry : mActiveMacros)
	{
		if (const uint8_t id = entry.second.cmd->getResponseId())
		{
			mFetHandle->resumeLoopCmd(id);
		}
	}
}

void PollingManager::setBreakpointCallback(const Callback& callback)
{
	mCallbacks[BP_HIT_FLAG] = callback;
}

void PollingManager::setLpmCallback(const Callback& callback)
{
	mCallbacks[JSTATE_CAPTURE_FLAG] = callback;
}

void PollingManager::setStateStorageCallback(const Callback& callback)
{
	mCallbacks[STATE_STORAGE_FLAG] = callback;
	mCallbacks[VARIABLE_WATCH_FLAG] = callback;
}

void PollingManager::setEnergyTraceCallback(const Callback& callback)
{
	mCallbacks[ENERGYTRACE_INFO] = callback;
}

void PollingManager::queueEvent(MessageDataPtr messageData)
{
	mEventNotifier.queueEvent(messageData);
}

void PollingManager::runEvent(MessageDataPtr messageData) const
{
	uint16_t eventMask = 0;
	(*messageData) >> eventMask;

	messageData->reset();

	//Check callback table for matching entries and execute if valid
	for (const CallbackTable::value_type& entry : mCallbacks)
	{
		if ((eventMask & entry.first) && entry.second)
		{
			entry.second(messageData);
		}
	}
}
