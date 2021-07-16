/*
 * SpecialMemoryTypes.h
 *
 * Memory Types whith special handling.
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
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

#include <hal.h>

#include <FlashMemoryAccessBase.h>
#include <ReadonlyMemoryAccess.h>
#include <RandomMemoryAccess.h>

namespace TI { namespace DLL430 {
	class IMemoryManager;


	class InformationFlashAccess : public FlashMemoryAccessBase
	{
	public:
		InformationFlashAccess (
					MemoryArea::Name name,
					TI::DLL430::IDeviceHandle* devHandle,
					uint32_t start,
					uint32_t end,
					uint32_t seg,
					uint32_t banks,
					bool mapped,
					const bool protectable,
					IMemoryManager* mm,
					uint8_t psa)
		: FlashMemoryAccessBase(
			name, devHandle,
			start, end, seg, banks,
			mapped, protectable, mm, psa
		) {}

		virtual unsigned short getFlags() {return isLocked() ? LOCK_INFO_A_FLAG : NO_FLAG;}

		virtual bool erase() OVERRIDE;
	};


	//actual Bootcode is flash, but behaves like ROM
	class BootcodeRomAccess : public ReadonlyMemoryAccess
	{
	public:
		BootcodeRomAccess (MemoryArea::Name name,
					TI::DLL430::IDeviceHandle* devHandle,
					uint32_t start,
					uint32_t end,
					uint32_t seg,
					uint32_t banks,
					bool mapped,
					const bool protectable,
					IMemoryManager* mm,
					uint8_t psa)
		: ReadonlyMemoryAccess (name, devHandle,	start, end, seg, banks, mapped, protectable, mm, psa)
		{}

		virtual bool doRead(uint32_t address, uint8_t* buffer, size_t count) OVERRIDE;
	};


	class BslRomAccessGR : public BootcodeRomAccess
	{
	public:
		BslRomAccessGR (MemoryArea::Name name,
				TI::DLL430::IDeviceHandle* devHandle,
				uint32_t start,
				uint32_t end,
				uint32_t seg,
				uint32_t banks,
				bool mapped,
				const bool protectable,
				IMemoryManager* mm,
				uint8_t psa)
		: BootcodeRomAccess(name, devHandle, start, end, seg, banks, mapped, protectable, mm, psa)
		{}

		virtual bool doRead(uint32_t address, uint8_t* buffer, size_t count) OVERRIDE;

	private:
		static const uint32_t sysBslcAddr = 0x142;

		uint16_t readSysbslc() const;
		void writeSysbslc(uint16_t value) const;
	};


	class BslMemoryAccessBase : public MemoryAreaBase
	{
	public:
		virtual bool sync() OVERRIDE { return memoryAccess->sync(); }

		virtual bool doRead(uint32_t address, uint8_t* buffer, size_t count) OVERRIDE;

		virtual bool doWrite(uint32_t address, const uint8_t* buffer, size_t count) OVERRIDE;

		virtual bool doWrite(uint32_t address, uint32_t value) OVERRIDE;

		virtual bool erase() OVERRIDE;

		virtual bool erase(uint32_t start, uint32_t end, bool forceUnlock = false) OVERRIDE;

		bool doUnlockBslMemory();

		bool readBslPe(std::vector<uint8_t>* bslPeBuffer) const;

		bool unlockBslPeAndCheck(uint32_t bslSize);

		bool isDeviceLocked(const std::vector<uint8_t>& bslPeBuffer) const;

		uint32_t getLockedStartAddress() const;

	protected:
		BslMemoryAccessBase (MemoryArea::Name name,
					TI::DLL430::IDeviceHandle* devHandle,
					uint32_t start,
					uint32_t end,
					uint32_t seg,
					uint32_t banks,
					bool mapped,
					const bool protectable,
					uint8_t psa,
					IMemoryManager* mm,
					MemoryAreaBase* memoryAccess)
			: MemoryAreaBase(name, devHandle, start, end, seg, banks, mapped, protectable, psa)
			, mm(mm)
			, memoryAccess(memoryAccess)
		{}

		virtual ~BslMemoryAccessBase()
		{
			if (memoryAccess)
			{
				delete memoryAccess;
			}
			memoryAccess = nullptr;
		}

	private:
		virtual bool postSync(const HalExecCommand&) { return true; }

		IMemoryManager* mm;
		MemoryAreaBase* memoryAccess;

		static const uint16_t mySysBslcSize = 2;
		static const uint16_t mySysBslc = 0x182;
	};

	template<class BaseAccessType>
	class BslMemoryAccess : public BslMemoryAccessBase
	{
	public:
		BslMemoryAccess(MemoryArea::Name name, TI::DLL430::IDeviceHandle* devHandle,
						uint32_t start, uint32_t end, uint32_t seg, uint32_t banks,
						bool mapped, const bool protectable, IMemoryManager* mm,	uint8_t psa)
			: BslMemoryAccessBase(name, devHandle, start, end, seg, banks, mapped, protectable, psa, mm,
					new BaseAccessType(MemoryArea::None, devHandle, start, end, seg, banks, mapped, protectable, mm, psa))
		{}
	};

	typedef BslMemoryAccess<FlashMemoryAccessBase> BslFlashAccess;
	typedef BslMemoryAccess<BootcodeRomAccess> BslRomAccess;
	class UsbRamAccess : public TI::DLL430::RandomMemoryAccess
	{
	public:
		UsbRamAccess(
			MemoryArea::Name name,
			TI::DLL430::IDeviceHandle* devHandle,
			uint32_t start,
			uint32_t end,
			uint32_t seg,
			uint32_t banks,
			bool mapped,
			const bool protectable,
			IMemoryManager* mm,
			uint8_t psa)
			: RandomMemoryAccess(
			name, devHandle,
			start, end, seg, banks,
			mapped, protectable, mm, psa
			) {}
		virtual bool doRead(uint32_t address, uint8_t* buffer, size_t count) OVERRIDE
		{
			return MemoryAreaBase::doRead(address, buffer, count);
		}
	};
}//namespace DLL430
}//namespace TI
