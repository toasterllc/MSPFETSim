/*
 * EemMemoryAccess.h
 *
 * Memory class for EEM register access.
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
		enum EemRegister
		{
			GENCTRL			= 0x82,		// general debug control register
			GENCLKCTRL		= 0x88,		// general clock control register
			MODCLKCTRL0		= 0x8A,		// clock module control register
		};

		class EemMemoryAccess : public MemoryAreaBase
		{
		public:
			EemMemoryAccess(MemoryArea::Name name, IDeviceHandle* devHandle,
					uint32_t start,
					uint32_t size,
					uint32_t seg,
					uint32_t banks,
					bool mapped,
					uint8_t bits);

			bool doRead(uint32_t address, uint32_t* value);
			bool doWrite(uint32_t address, uint32_t value);

			// write given value to eem register reg
			bool writeEemRegister(EemRegister reg, uint32_t value);
			// read value of eem register reg
			bool readEemRegister(EemRegister reg, uint32_t* buffer);

		private:
			const size_t maxAddress;
			uint8_t words;

			std::vector<uint8_t> queue;
			uint32_t* readPtr;

			bool preSync();
			bool postSync(const HalExecCommand&);
		};

	};
};
