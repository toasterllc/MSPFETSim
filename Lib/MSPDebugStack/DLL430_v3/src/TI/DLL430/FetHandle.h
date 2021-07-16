/*
 * FetHandle.cpp
 *
 * Communication with FET.
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

#include "IFetHandle.h"
#include "VersionInfo.h"


namespace TI
{
	namespace DLL430
	{
		struct PortInfo;
		class FetControl;
		class IoChannel;
		class HalExecCommand;
		class IConfigManager;
		class IMemoryManager;
		class DebugManagerMSP430;
		class DeviceHandleManager;
		class FetHandleManager;

		class FetHandle : public IFetHandle
		{
		public:
			static const char* id;

			FetHandle(const PortInfo&, FetHandleManager*, TARGET_ARCHITECTURE_t arch);
			~FetHandle();

			FetHandle(const FetHandle&) = delete;
			FetHandle& operator=(const FetHandle&) = delete;

			/* public API */
			const VersionInfo& getDllVersion() const { return this->version; };
			IConfigManager* getConfigManager();
			IDeviceHandleManager* getDeviceHandleManager();

			/* internal API */
			FetControl* getControl();

			bool send(HalExecCommand &command);

			bool kill(HalExecCommand &command);
			bool kill(uint8_t respId);
			bool pauseLoopCmd(unsigned long respId);
			bool resumeLoopCmd(unsigned long respId);

			bool hasCommunication() const;

			void addSystemNotifyCallback(const NotifyCallback& notifyCallback);

			void shutdown();

			bool sendHilCommand(HIL_COMMAND command, uint32_t data = 0);
			uint64_t sendJtagShift(HIL_COMMAND shiftType, uint64_t data, int32_t bitSize = 16);
			bool setJtagPin(JTAG_PIN pin, bool state);

			bool resetState();

			const std::vector<uint8_t>* getHwVersion() const;
			const std::vector<uint8_t>* getSwVersion() const;
			std::string getCurrentPortName();

		private:
			VersionInfo version;
			IoChannel* channel;
			FetControl* control;

			IConfigManager* configManager;
			IDeviceHandleManager* deviceHandleManager;

			bool communication;
		};
	}
}
