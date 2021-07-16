/*
 * ArmSpecialFlashMemoryAccess.h
 *
 * Memory class for accessing RAM on ARM Cortex devices.
 *
 * Copyright (C) 2007 - 2015 Texas Instruments Incorporated - http://www.ti.com/
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

#include "ArmFlashMemoryAccess.h"

namespace TI
{
	namespace DLL430
	{
		class IDeviceHandle;
		class ArmBslFlashMemoryAccess : public ArmFlashMemoryAccess
		{
		public:
			ArmBslFlashMemoryAccess(
				MemoryArea::Name name,
				IDeviceHandle* devHandle,
				uint32_t start,
				uint32_t size,
				uint32_t seg,
				uint32_t banks,
				bool mapped,
				const bool isProtected,
				IMemoryManager* mm,
				uint8_t psa);
			virtual ~ArmBslFlashMemoryAccess();

			virtual bool doWrite(uint32_t address, const uint8_t* buffer, size_t count) OVERRIDE;
			virtual bool erase(uint32_t start, uint32_t end, bool forceUnlock = false) OVERRIDE;
			virtual bool erase() OVERRIDE;

		private:
			uint32_t start;
			uint32_t end;
		};

		class ArmInfoFlashMemoryAccess : public ArmFlashMemoryAccess
		{
		public:
			ArmInfoFlashMemoryAccess(
				MemoryArea::Name name,
				IDeviceHandle* devHandle,
				uint32_t start,
				uint32_t size,
				uint32_t seg,
				uint32_t banks,
				bool mapped,
				const bool isProtected,
				IMemoryManager* mm,
				uint8_t psa
				);
			virtual ~ArmInfoFlashMemoryAccess();

			virtual bool erase() OVERRIDE;

		private:
			uint32_t start;
			uint32_t end;
		};
		
		// MSP32 P4111 special hanlding
		class ArmFlashMemoryAccess2M : public ArmFlashMemoryAccess
		{
			public:
				ArmFlashMemoryAccess2M(
					MemoryArea::Name name,
					IDeviceHandle* devHandle,
					uint32_t start,
					uint32_t size,
					uint32_t seg,
					uint32_t banks,
					bool mapped,
					const bool isProtected,
					IMemoryManager* mm,
					uint8_t psa,
					IWriteProtection *writeProt
					);
				virtual ~ArmFlashMemoryAccess2M();

				virtual bool wakeup() OVERRIDE;

			private:
		};


		class ArmBslFlashMemoryAccess2M : public ArmFlashMemoryAccess2M
		{
			public:
				ArmBslFlashMemoryAccess2M(
					MemoryArea::Name name,
					IDeviceHandle* devHandle,
					uint32_t start,
					uint32_t size,
					uint32_t seg,
					uint32_t banks,
					bool mapped,
					const bool isProtected,
					IMemoryManager* mm,
					uint8_t psa);
				virtual ~ArmBslFlashMemoryAccess2M();

				virtual bool doWrite(uint32_t address, const uint8_t* buffer, size_t count) OVERRIDE;
				virtual bool erase(uint32_t start, uint32_t end, bool forceUnlock = false) OVERRIDE;
				virtual bool erase() OVERRIDE;

			private:
				uint32_t start;
				uint32_t end;
		};

		class ArmInfoFlashMemoryAccess2M : public ArmFlashMemoryAccess2M
		{
			public:
				ArmInfoFlashMemoryAccess2M(
					MemoryArea::Name name,
					IDeviceHandle* devHandle,
					uint32_t start,
					uint32_t size,
					uint32_t seg,
					uint32_t banks,
					bool mapped,
					const bool isProtected,
					IMemoryManager* mm,
					uint8_t psa
					);
				virtual ~ArmInfoFlashMemoryAccess2M();
				virtual bool erase() OVERRIDE;

			private:
				uint32_t start;
				uint32_t end;				
		};
	}
}
