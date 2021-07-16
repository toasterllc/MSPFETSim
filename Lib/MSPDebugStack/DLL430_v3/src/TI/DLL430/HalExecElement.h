/*
 * HalExecElement.h
 *
 * Buffer for sending.
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

#include <boost/thread/mutex.hpp>

namespace TI
{
	namespace DLL430
	{
		class HalExecCommand;
		class HalExecBuffered;

		class HalExecElement
		{
		public:
			HalExecElement(uint16_t functionId, uint8_t msgType = 0x81);

			void appendInputData64(uint64_t data);
			void appendInputData64(const uint64_t* data, size_t dwords);
			void appendInputData32(uint32_t);
			void appendInputData32(const uint32_t* data, size_t dwords);
			void appendInputData16(uint16_t);
			void appendInputData16(const uint16_t* data, size_t words);
			void appendInputData16(const std::vector<uint16_t>::const_iterator& begin, const std::vector<uint16_t>::const_iterator& end);
			void appendInputData8(uint8_t);
			void appendInputData8(const uint8_t* data, size_t bytes);
			void appendInputData8(const std::vector<uint8_t>::const_iterator& begin, const std::vector<uint8_t>::const_iterator& end);

			void setOutputSize(size_t);
			const std::vector<uint8_t>& getOutput() const;
			uint64_t getOutputAt64(size_t pos) const;
			uint32_t getOutputAt32(size_t pos) const;
			uint16_t getOutputAt16(size_t pos) const;
			uint8_t getOutputAt8(size_t pos) const;

			void setAddrFlag(bool flag);
			bool getAddrFlag() const;

			void addTransaction(uint8_t id);
			uint8_t checkTransaction(uint8_t id, uint8_t flag);
			uint8_t changeTransaction(uint8_t id, uint8_t flag, bool set);

		private:
			friend class TI::DLL430::HalExecCommand;
			friend class TI::DLL430::HalExecBuffered;

			uint16_t functionId;
			uint8_t msgType;
			std::vector<uint8_t> inData;
			std::vector<uint8_t> outData;

			// key is response id and value transaction flags
			std::map<uint8_t, uint8_t> respRef;
			boost::mutex respRefMutex;

			bool hasAddr;
		};

	};
};
