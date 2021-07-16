/*
 * DeviceHandleMSP430.h
 *
 * Communication with target device.
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

#include <pch.h>
#include <MSP430.h>
#include <cmath>

#include "VersionInfo.h"
#include "DeviceHandleMSP430.h"
#include "DeviceDb/Database.h"
#include "HalExecCommand.h"
#include "FetControl.h"
#include "WatchdogControl.h"
#include "FetHandle.h"
#include "MemoryManager.h"
#include "CpuRegisters.h"
#include "DebugManagerMSP430.h"
#include "ClockCalibration.h"
#include "EM/EmulationManager/EmulationManager430.h"
#include "EM/SoftwareBreakpoints/ISoftwareBreakpoints.h"
#include "EM/SoftwareBreakpoints/SoftwareBreakpointManager.h"
#include "EM/EemRegisters/EemRegisterAccess430.h"
#include "EM/Exceptions/Exceptions.h"
#include "EemMemoryAccess.h"
#include "JtagId.h"
#include "JtagShifts.h"
#include "FileReader.h"

#include "../../Bios/include/error_def.h"
#include "../../Bios/include/ConfigureParameters.h"

namespace TI
{
	namespace DLL430
	{
		class HalCommand;
		
		class DeviceHandleMSP430 : public IDeviceHandle
		{
		public:
			DeviceHandleMSP430(FetHandle*, uint32_t deviceCode);
			~DeviceHandleMSP430();

			DeviceHandleMSP430(const DeviceHandleMSP430&) = delete;
			DeviceHandleMSP430& operator=(const DeviceHandleMSP430&) = delete;

			EmulationManagerPtr getEmulationManager();
			MemoryManager* getMemoryManager ();
			DebugManagerMSP430* getDebugManager ();
			FetHandle* getFetHandle () { return parent; }
			ClockCalibration* getClockCalibration() { return clockCalibration; }

			/// \brief Run the boot code writing the the specified command in the mailbox first
			/// \param command[in] Command to be put in the mailbox
			/// \return True if bootcode execution succeeded
			long magicPatternSend(uint16_t ifMode);
			int32_t identifyDevice(uint32_t activationKey, bool afterMagicPattern);
			const std::string& getDescription() const;
			bool secure ();
			bool reset(bool hardReset);

			FetControl* getControl ();

			bool send (HalExecCommand &command);

			void setWatchdogControl (std::shared_ptr<WatchdogControl>);
			std::shared_ptr<WatchdogControl> getWatchdogControl() const;

			uint32_t readJtagId();
			uint32_t getJtagId();
			uint32_t getDeviceIdPtr();
			uint32_t getEemVersion();
			bool isJtagFuseBlown();
			uint32_t getDeviceCode() const;

			int32_t setDeviceId (long id);

			hal_id checkHalId(hal_id base_id) const;

			const FuncletCode& getFunclet(FuncletCode::Type funclet);

			bool supportsQuickMemRead() const;
			uint16_t getMinFlashVcc() const;
			bool hasFram() const;
			bool hasLPMx5() const;

			void disableHaltOnWakeup();

			bool eemAccessibleInLpm() const;

			bool deviceSupportsEnergyTrace() const;

			TARGET_ARCHITECTURE_t getTargetArchitecture() const;

			INTERFACE_TYPE getInterfaceMode(){ return SPYBIWIREJTAG_IF; }

			bool restoreTinyRam();
		protected:

		private:
			FetHandle* parent;
			EmulationManagerPtr emulationManager;
			MemoryManager* memoryManager;
			DebugManagerMSP430* debugManager;
			ClockCalibration* clockCalibration;

			DeviceInfo deviceInfo;
			bool deviceHasLPMx5;

			uint32_t jtagId;
			uint32_t deviceIdPtr;
			uint32_t eemVersion;
			enum INTERFACE_TYPE mode;
			uint32_t deviceCode;


			std::shared_ptr<WatchdogControl> wdt;


			typedef std::array<uint8_t, nrUsedClockModules> EtwCodes;
			EtwCodes etwCodes;

			void configure (const DeviceInfo& info);
			long getDeviceIdentity(uint32_t activationKey, uint32_t* pc, uint32_t* sr, bool afterMagicPattern);

			bool sendDeviceConfiguration(uint32_t parameter, uint32_t value);

			int16_t getSubID(uint32_t info_len, uint32_t deviceIdPtr, uint32_t pc);
		};
	}
}
