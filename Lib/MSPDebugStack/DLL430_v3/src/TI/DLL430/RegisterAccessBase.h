/*
 * RegisterAccessBase.h
 *
 * Base class for accessing registers.
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

#include "MemoryAreaBase.h"

namespace TI
{
	namespace DLL430
	{
		class IDeviceHandle;
		class RegisterAccess : public MemoryAreaBase
		{
		public:
			RegisterAccess (
				MemoryArea::Name name,
				IDeviceHandle* devHandle,
				uint32_t start,
				uint32_t end,
				uint32_t seg,
				uint32_t banks,
				bool mapped,
				uint8_t bits,
				const std::vector<uint8_t>& mask
			);
			virtual ~RegisterAccess() {};

			bool doRead(uint32_t address, uint8_t* buffer, size_t count) OVERRIDE;
			bool doWrite(uint32_t address, const uint8_t* buffer, size_t count) OVERRIDE;
			bool doWrite(uint32_t address, uint32_t value) OVERRIDE;

		protected:
			bool postSync(const HalExecCommand&) OVERRIDE;

			uint8_t bits;
			std::vector<uint8_t> mask;
			std::vector<uint32_t> v_buffer;
		};


		class RegisterAccess5xx : public RegisterAccess
		{
		public:
			RegisterAccess5xx (
				MemoryArea::Name name,
				IDeviceHandle* devHandle,
				uint32_t start,
				uint32_t end,
				uint32_t seg,
				uint32_t banks,
				bool mapped,
				const bool,
				IMemoryManager*,
				uint8_t
				)
				: RegisterAccess(name, devHandle, start, end, seg, banks, mapped, 16, std::vector<uint8_t>())
			{}

			bool doRead(uint32_t address, uint8_t* buffer, size_t count) OVERRIDE;
		};
	}
}
