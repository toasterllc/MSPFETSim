/*
 * MemoryAreaBase.h
 *
 * Base class for all memory classes.
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


#include "MemoryArea.h"
#include "HalExecElement.h"
#include <hal.h>

namespace TI
{
	namespace DLL430
	{
		class IDeviceHandle;
		class HalExecCommand;
		class IMemoryManager;

		class MemoryAreaBase : public MemoryArea
		{
		public:
			MemoryAreaBase(MemoryArea::Name, IDeviceHandle* devHandle,
							uint32_t start, uint32_t end,
							uint32_t seg, uint32_t banks,
							bool mapped, const bool protectable,
							uint8_t psa_type);

			MemoryAreaBase(const MemoryAreaBase&) = delete;
			MemoryAreaBase& operator=(const MemoryAreaBase&) = delete;

			MemoryArea::Name getName() const OVERRIDE;
			MemoryError getError();
			virtual bool isReadOnly() const OVERRIDE;

			uint32_t getStart() const OVERRIDE;
			uint32_t getEnd() const OVERRIDE;
			uint32_t getSize() const OVERRIDE;
			uint32_t getSegmentSize() const OVERRIDE;
			uint32_t getBanks() const OVERRIDE;
			bool isMapped() const OVERRIDE;
			bool isLocked() const;
			bool lock();
			bool unlock();

			bool read(uint32_t address, uint8_t* buffer, size_t count) OVERRIDE;
			bool overwrite(uint32_t address, const uint8_t* buffer, size_t count) OVERRIDE;
			bool write(uint32_t address, const uint8_t* buffer, size_t count) OVERRIDE;
			bool write(uint32_t address, uint32_t value) OVERRIDE;

			virtual bool doRead(uint32_t address, uint8_t* buffer, size_t count);
			virtual bool doOverwrite(uint32_t address, const uint8_t* buffer, size_t count);
			virtual bool doWrite(uint32_t address, const uint8_t* buffer, size_t count);
			virtual bool doWrite(uint32_t address, uint32_t value);

			virtual bool sync() OVERRIDE;

			bool send(std::vector< std::unique_ptr<HalExecElement> >* elem, HalExecCommand* cmd);

			virtual bool erase() OVERRIDE;
			bool erase(uint32_t start, uint32_t end, bool forceUnlock = false) OVERRIDE;
			bool verify(uint32_t address, const uint8_t* buffer, size_t count) OVERRIDE;
			static uint16_t psa(uint32_t address, const uint8_t* buffer, size_t count);

			virtual void setAccessible(bool accessible) OVERRIDE;
			virtual bool isAccessible() const OVERRIDE;

			virtual bool fillCache() OVERRIDE { return false; }
			virtual bool flushCache() const OVERRIDE { return false; }

			virtual bool wakeup() OVERRIDE { return false; }
			virtual bool isProtectionEnabled() OVERRIDE { return false; }
		protected:
			struct Alignment
			{
				Alignment(uint32_t alignedAddress, int frontPadding, int backPadding)
					: alignedAddress(alignedAddress), frontPadding(frontPadding), backPadding(backPadding) {}

				const uint32_t alignedAddress;
				const int frontPadding;
				const int backPadding;
			};

			bool defaultRead(hal_id readMacro, IMemoryManager* mm, uint32_t address, uint8_t* buffer, size_t count);

			virtual Alignment alignData(uint32_t address, uint32_t count) const;

			virtual bool preSync() { return true; };
			virtual bool postSync(const HalExecCommand&) = 0;

			virtual unsigned short getFlags() {return 0;}
			MemoryArea::Name name;
			IDeviceHandle* devHandle;

			struct ReadElement
			{
				ReadElement() : v_buffer(0), size(0), omitFirst(false), omitLast(false), offset(0) {}

				ReadElement(uint8_t* buffer, size_t size, bool omitFirst, bool omitLast, size_t offset)
					: v_buffer(buffer), size(size), omitFirst(omitFirst), omitLast(omitLast), offset(offset) {}

				uint8_t* v_buffer;
				size_t size;

				bool omitFirst;
				bool omitLast;
				size_t offset;
			};
			typedef std::map<size_t, ReadElement> ReadElement_map;
			ReadElement_map readMap;
			uint8_t psaType;
			MemoryError err;

		protected:
			std::vector<std::unique_ptr<HalExecElement>> elements;

		private:
			uint32_t start;
			uint32_t end;
			uint32_t segmentSize;
			uint32_t banks;
			bool mapped;
			const bool isProtectable;
			bool locked;
			bool isAccessible_;
		};
	}
}
