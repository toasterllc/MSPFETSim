/*
 * ConfigManager.h
 *
 * Functionality for configuring target device.
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


#include "IConfigManager.h"
#include "HalExecCommand.h"

namespace TI
{
	namespace DLL430
	{
		class FetHandle;
		class IUpdateManager;
		class FetHandleManager;
		class EnergyTraceManager;

		class ConfigManager : public IConfigManager
		{
		public:
			ConfigManager(FetHandle*, FetHandleManager*, TARGET_ARCHITECTURE_t arch);
			~ConfigManager();

			ConfigManager(const ConfigManager&) = delete;
			ConfigManager& operator=(const ConfigManager&) = delete;

			void init();

			VersionInfo getHalVersion() const;

			bool isUpdateRequired(TARGET_ARCHITECTURE_t arch) const;

			bool setDeviceVcc(uint16_t vcc);

			uint16_t getDeviceVcc() const;

			void setJtagMode(enum INTERFACE_TYPE mode);
			INTERFACE_TYPE getInterfaceMode(TARGET_ARCHITECTURE_t arch) const;
			uint16_t getExternalVcc() const;
			int16_t start();
			int16_t start(const std::string& pwd, uint32_t deviceCode);
			bool stop();

			bool reset(bool vcc, bool nmi, uint16_t JtagId, uint32_t rstHalId);

			bool firmWareUpdate(const char * fname, UpdateNotifyCallback callback, bool* coreUpdate);

			void setPassword(const std::string& pwd);
			bool setDeviceCode(uint32_t deviceCode);
			bool setJtagSpeed(JTAG_4WIRE_SPEED speedJtag, JTAG_2WIRE_SPEED speedSbw);
			bool jtagErase(uint16_t eraseKey);
			void setEnergyTraceManager(EnergyTraceManager*);
			bool isEnergyTraceSupported();
			void setCurrentDrive(uint32_t value);
			bool configureJtagSpeed(uint32_t speed);

			bool freqCalibrationEnabled() const;
			bool ulpDebugEnabled() const;
			bool atProbeStateEnabled() const;
			void setUlpDebug(bool ulp);
			long MSP430I_MagicPattern(uint16_t ifMode);

			bool configureOverCurrent(bool state);

			void setDisableInterruptsMode(uint32_t mode);
			uint32_t getDisableInterruptsMode() const;
			bool updateDisableInterruptsMode();
			bool setJTAGLock5xx(uint32_t value);

		private:
			bool setVccEzFet(uint16_t vcc);
			bool setVccMspFetUif(uint16_t vcc);
			bool setVccMspFET(uint16_t vcc);

			FetHandle* parent;
			IUpdateManager* updateManagerFet;
			uint16_t vcc;
			enum INTERFACE_TYPE mode;
			EnergyTraceManager* mEnergyTraceManager;

			HalExecCommand updateCmd;

			std::string password;
			uint32_t deviceCode;
			uint32_t mhighres;
			bool freqCalibration;
			bool ulpDebug;
			bool atProbeState;
			TARGET_ARCHITECTURE_t arch;
			uint32_t disableInterruptsMode;
			uint32_t disableInterruptsModeBackup;
		};
	}
}
