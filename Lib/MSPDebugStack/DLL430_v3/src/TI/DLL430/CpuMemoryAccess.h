/*
 * CpuMemoryAccess.h
 *
 * Implementaion for access of CPU registers.
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

#include "CpuRegisters.h"
#include "DeviceInfo.h"

namespace TI
{
	namespace DLL430
	{
		class IDeviceHandle;

		class CpuMemoryAccess : public CpuRegisters
		{
		public:
			CpuMemoryAccess(MemoryArea::Name name, IDeviceHandle* devHandle,
				uint32_t start,
				uint32_t size,
				uint32_t seg,
				uint32_t banks,
				bool mapped,
				uint8_t bits);

			bool read(uint32_t Register, uint32_t* value, size_t count) OVERRIDE;
			bool write(uint32_t Register, const uint32_t* buffer, size_t count) OVERRIDE;
			bool write(uint32_t Register, uint32_t value) OVERRIDE;
			size_t getSize() const OVERRIDE;

			bool flushCache() OVERRIDE;
			bool fillCache(uint32_t Register, size_t count) OVERRIDE;
			void clearCache(uint32_t Register, size_t count) OVERRIDE;

			void pushCache() OVERRIDE {};
			void popCache() OVERRIDE {};
			bool switchContext(uint32_t pc, uint32_t sp) OVERRIDE { return true; }
			bool disableInterrupts()  OVERRIDE{ return true; }


		private:
			virtual bool postSync(const HalExecCommand&) { return true; }

			size_t size;
			uint8_t bytes;

			typedef uint32_t cpuType;
			std::vector<cpuType> localCache;
			IDeviceHandle* devHandle;
		};

	}
}
