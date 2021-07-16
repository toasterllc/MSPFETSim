/*
 * PollingManager.h
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

#pragma once

#include "HalExecCommand.h"
#include "MessageData.h"
#include "EventNotifier.h"
#include <boost/thread/mutex.hpp>

/*
	Polling is encapsulated in this class to transparently handle situations
	where multiple kinds of polling are performed by the same macro. For example,
	ID_PollJStateReg is used to poll for breakpoints, LPMx.5 and Energy Trace.

	This makes it necessary to keep track of what a macro is polling for and which
	macro is used for a particular device (not all devices are polling for
	breakpoints in ID_PollJStateReg).

	Usage:
		-Configure callbacks for different events (ex. breakpoints)
			void breakpointCallback(MessageDataPtr data) {...}

			pollingManager.setBreakpointCallback(breakpointCallback);

			For member functions as callbacks, use std::bind
			pollingManager.setBreakpointCallback( bind(&MyClass::callback, instance, _1) );

		-Start polling (this will add a macro to the polling loop or modify an existing one)
			pollingManager.startBreakpointPolling(deviceHandle);

		-pause/resume polling will pause/resume all polling loops,
		 only State Storage polling can currently be paused separately.
		 Pausing only skips macros in the loop, they are not removed.
			pollingManager.pausePolling();
			pollingManager.resumeStateStoragePolling(deviceHandle);
			pollingManager.resumePolling();

		-Stop polling (remove macro from loop or modify existing one)
			pollingManager.stopBreakpointPolling(deviceHandle);
*/

namespace TI
{
	namespace DLL430
	{
		enum EtPollingMode
		{
			ET_POLLING_OFF,
			ET_POLLING_ANALOG,
			ET_POLLING_ANALOG_DSTATE,
			ET_POLLING_ANALOG_DSTATE_PC,
		};

		enum EtGatedMode
		{
			ET_POLLING_GATED_OFF,
			ET_POLLING_GATED_ON
		};

		class IDeviceHandle;
		class FetHandle;

		class PollingManager
		{
		public:
			typedef std::function<void (MessageDataPtr)> Callback;

			explicit PollingManager(FetHandle* fetHandle);

			PollingManager(const PollingManager&) = delete;
			PollingManager& operator=(const PollingManager&) = delete;

			void shutdown();

			bool startBreakpointPolling(const IDeviceHandle& devHandle);
			bool startLpmPolling(const IDeviceHandle& devHandle);
			bool startStateStoragePolling(const IDeviceHandle& devHandle);
			bool startEnergyTracePolling(EtPollingMode, EtGatedMode);

			bool stopBreakpointPolling(const IDeviceHandle& devHandle);
			bool stopLpmPolling(const IDeviceHandle& devHandle);
			bool stopStateStoragePolling(const IDeviceHandle& devHandle);
			bool stopEnergyTracePolling();

			void pausePolling(); //Pause all polling loops
			void pauseStateStoragePolling(const IDeviceHandle&);

			void resumePolling(); //Resume all polling loops
			void resumeStateStoragePolling(const IDeviceHandle&);

			void setEnergyTraceCallback(const Callback&);
			void setBreakpointCallback(const Callback&);
			void setLpmCallback(const Callback&);
			void setStateStorageCallback(const Callback&);

			void queueEvent(MessageDataPtr messageData);

		private:
			enum POLLING_TYPE {PT_BREAKPOINT, PT_LPM, PT_STATE_STORAGE, PT_ENERGY_TRACE};

			bool startPolling(POLLING_TYPE, const IDeviceHandle&);
			bool stopPolling(POLLING_TYPE, const IDeviceHandle&);

			void pausePolling(POLLING_TYPE, const IDeviceHandle&);
			void resumePolling(POLLING_TYPE, const IDeviceHandle&);

			void runEvent(MessageDataPtr messageData) const;

			bool addMacro(hal_id macroId);
			bool removeMacro(hal_id macroId);

			bool addToLoop(hal_id macroId);

			uint8_t getResponseId(hal_id baseMacroId, const IDeviceHandle&) const;

			FetHandle* mFetHandle;

			std::map<POLLING_TYPE, bool> mPollingActive;
			std::map<POLLING_TYPE, hal_id> mPollingMacro;

			EtPollingMode mEnergyTraceMode;
			EtGatedMode mEtGatedMode;

			struct Macro
			{
				Macro() : count(0), cmd(new HalExecCommand) {}
				int count;
				//Can't be copied due to contained std::mutex
				std::shared_ptr<HalExecCommand> cmd;
			};
			typedef std::map<uint32_t, Macro> MacroTable;
			MacroTable mActiveMacros;

			typedef std::map<uint16_t, Callback> CallbackTable;
			CallbackTable mCallbacks;

			std::map<uint32_t, bool> mKillAfterEvent;
			std::map<EtPollingMode, hal_id> mEtModeToMacro;

			EventNotifier<MessageDataPtr> mEventNotifier;

			boost::mutex mMutexActiveMacros;
		};
	}
}
