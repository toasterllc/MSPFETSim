/*
 * MemoryContent.h
 *
 * Class holding memory segment data
 *
 * Copyright (C) 2008 - 2014 Texas Instruments Incorporated - http://www.ti.com/
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


namespace TI
{
	namespace DLL430
	{
		struct DataSegment
		{
			uint32_t startAddress;
			std::vector<uint8_t> data;

			explicit DataSegment(uint32_t address = 0) : startAddress(address) {}
			DataSegment(uint32_t address, const uint8_t* data, size_t size) : startAddress(address), data(data, data+size) {}
		};

		class MemoryContent
		{
		public:
			MemoryContent() {}
			MemoryContent(uint32_t address, const uint8_t* data, size_t size);
			MemoryContent(const uint16_t* data, const uint32_t* address, const uint32_t* length, uint32_t sections);

			void fromBuffer(uint32_t address, const uint8_t* data, size_t size);
			void fromSRec(const uint16_t* data, const uint32_t* address, const uint32_t* length, uint32_t sections);

			std::vector<DataSegment> segments;
		};
	}
}
