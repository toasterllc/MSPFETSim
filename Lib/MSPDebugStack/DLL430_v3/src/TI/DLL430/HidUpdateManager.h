/*
 * HidUpdateManager.h
 *
 * Recovery for broken eZ-FETs and MSP-FET Debuggers
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

#include "IUpdateManager.h"

class MSPBSL_Connection5xxUSB;

namespace TI
{
	namespace DLL430
	{
		class MemoryContent;

		inline std::string convertPid(int pid)
		{
			std::stringstream tempPid;
			tempPid << std::hex << pid;
			return tempPid.str();
		}

		class HidUpdateManager
		{
		public:
			static uint32_t countHidDevices(uint16_t productId);

			HidUpdateManager ();
			~HidUpdateManager ();

			bool hid_firmWareUpdate(const char* fname, UpdateNotifyCallback callback);

		private:
			bool hid_updateCore(const MemoryContent &firmware, MSPBSL_Connection5xxUSB* bsl) const;

			MSPBSL_Connection5xxUSB* hid_enumerateBSL(uint16_t currentPid);
			uint16_t hid_getBSLToolId(MSPBSL_Connection5xxUSB* bsl);
		};

	}
}
