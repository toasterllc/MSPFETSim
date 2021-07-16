/*
 * DebugManagerMSP430.h
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

#pragma once

#include "HalExecCommand.h"
#include "DeviceInfo.h"
#include "IDebugManager.h"

#include "EEM/CycleCounter.h"

#include "MessageData.h"
#include "PollingManager.h"


namespace TI
{
	namespace DLL430
	{
		class IDeviceHandle;
		namespace EEM {
			class Trigger;
			class TriggerGroupImpl;
			class SequencerImpl;
		};

		class DebugManagerMSP430 : public IDebugManager
		{
		public:
			DebugManagerMSP430 (IDeviceHandle*, const DeviceInfo&);
			~DebugManagerMSP430 ();

			DebugManagerMSP430(const DebugManagerMSP430&) = delete;
			DebugManagerMSP430& operator=(const DebugManagerMSP430&) = delete;

			bool reconnectJTAG();

			/** free run */
			bool run (uint16_t controlMask, DebugEventTarget* = 0, bool releaseJTAG = false);

			bool clearEemResponse();

			/* sync Jtag */
			bool stop (bool jtagWasReleased = false);

			/* do a single step */
			bool singleStep (uint32_t* cycles = 0);

			uint8_t getClockControl() const;

			uint16_t getClockControlSetting() const;

			uint16_t getGeneralClockDefaultSetting() const;

			uint32_t getClockModuleDefaultSetting() const;

			uint32_t getClockModuleSetting() const;

			char ** getModuleStrings(uint32_t * n) const;

			char ** getClockStrings(uint32_t * n) const;

			bool initEemRegister();

			void setOpcode(uint16_t value);

			bool activatePolling(uint16_t mask);

			bool saveContext();

			void runEvent(MessageDataPtr messageData);

			void setLpmDebugging(bool enable);

			bool getLpmDebugging();

			bool activateJStatePolling(DebugEventTarget* cb);

			bool queryIsInLpm5State();

			bool wakeupDevice();

			void pausePolling();

			void resumePolling();

			bool syncDeviceAfterLPMx5();

			void enableLegacyCycleCounter(bool enable);

			bool legacyCycleCounterEnabled() const { return cycleCounter_.isEnabled(); }

			uint64_t getCycleCounterValue() { return cycleCounter_.read(); }

			void resetCycleCounterValue() { cycleCounter_.reset(); }

			bool startStoragePolling();

			bool stopStoragePolling();

			void setCalibrationValues(double *calibrationValues);

			void setPollingManager(PollingManager* pollingManager);

			bool setPCtoSafeLocation();

		private:
			void createModuleStrings(const ClockMapping& clockMapping);
			void createClockStrings(const ClockNames& clockNames);

			void setLeaAccessibility();
			bool backUpTinyRam();
			bool restoreTinyRam();
			bool checkUssIsBusy();

			IDeviceHandle* parent;

			uint8_t clockControl;
			uint16_t genclkcntrl;
			uint32_t mclkcntrl0;

			uint16_t defaultGenClkCntrl;
			uint32_t defaultMclkcntrl0;

			uint8_t emulationLevel;

			char** moduleStrings;
			uint32_t nModuleStrings;

			char** clockStrings;
			uint32_t nClockStrings;

			uint16_t lpm5WakeupDetected;

			uint16_t irRequest;

			uint16_t mdbPatchValue;

			DebugEventTarget* cbx;

			bool lpmDebuggingEnabled;

			CycleCounter cycleCounter_;
			bool storagePollingActive;

			PollingManager* mPollingManager;
		};

	}
}
