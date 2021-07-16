/*
 * MemoryManager.h
 *
 * Manages to which MemoryAreaBases to communicate dependent on address.
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
#include "IMemoryManager.h"
#include "FuncletCode.h"


namespace TI
{
	namespace DLL430
	{
		class IDeviceHandle;
		struct DeviceInfo;

		class MemoryManager : public IMemoryManager
		{
		public:
			MemoryManager (IDeviceHandle*, const DeviceInfo&);

			MemoryAreaBase* getMemoryArea (MemoryArea::Name name, size_t subIndex = 0);
			int16_t getMemoryAreaIndex(MemoryArea::Name type, uint32_t address, uint32_t size);

			MemoryAreaBase* getMemoryArea(size_t index) OVERRIDE;
			CpuRegisters* getCpuRegisters(size_t index = 0) OVERRIDE;
			size_t count() const OVERRIDE;

			bool read(uint32_t address, uint8_t* buffer, size_t count) OVERRIDE;
			bool overwrite(uint32_t address, const uint8_t* buffer, size_t count) OVERRIDE;
			bool write(uint32_t address, const uint8_t* buffer, size_t count) OVERRIDE;
			bool write(uint32_t address, uint32_t value) OVERRIDE;
			bool sync() OVERRIDE;
			bool erase() OVERRIDE;
			bool erase(uint32_t start, uint32_t end, bool forceUnlock = false) OVERRIDE;
			bool verify(uint32_t address, const uint8_t* buffer, size_t count) OVERRIDE;

			bool isReadOnly() const OVERRIDE;
			bool lock(MemoryArea::Name name, bool action) OVERRIDE;

			MemoryError getLastError() OVERRIDE;

			void setRamPreserveMode(bool enabled) OVERRIDE;
			bool getRamPreserveMode() const OVERRIDE;
			
			bool uploadFunclet(FuncletCode::Type type);
			bool checkMinFlashVoltage() const OVERRIDE;

		private:
			typedef bool (MemoryArea::*AccessFunction)(uint32_t, const uint8_t*, size_t);
			bool doForMemoryAreas(uint32_t address, const uint8_t* buffer, size_t count, AccessFunction function);

			typedef std::vector<std::unique_ptr<MemoryAreaBase>> MemoryList;
			typedef std::vector<std::unique_ptr<CpuRegisters>> CpuList;

			IDeviceHandle* parent;
			MemoryList types;
			CpuList cpus;
			MemoryError lastError;
			bool preserveRam;
		};

	}
}
