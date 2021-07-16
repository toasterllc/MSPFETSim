/*
 * WriteProtection.h
 *
 * Copyright (C) 2007 - 2014 Texas Instruments Incorporated - http://www.ti.com/
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

		class IMemoryManager;

		class IWriteProtection
		{
		public:
			virtual ~IWriteProtection() {}
			virtual bool disableIfEnabled() = 0;
			virtual void restore() = 0;
			virtual bool isEnabled() = 0;
		};


		class NoWriteProtection : public IWriteProtection
		{
		public:
			explicit NoWriteProtection(IMemoryManager* mm) {}
			bool disableIfEnabled() { return true; }
			void restore() { return; }
			bool isEnabled() { return false; }
		};

		class WriteProtection : public IWriteProtection
		{
		public:
			WriteProtection(
				IMemoryManager* mm,
				uint32_t Register,
				uint16_t Bits,
				uint16_t Mask = 0xFFFF,
				uint16_t Pwd = 0x0
			);

			bool disableIfEnabled();
			void restore();
			bool isEnabled();

		private:
			bool readSettings();

			bool disable();

			IMemoryManager* mm;

			const uint32_t Register;
			const uint16_t Bits;
			const uint16_t Mask;
			const uint16_t Pwd;

			uint16_t registerValue;
			uint16_t registerBackup;
		};

		class WriteProtection432: public IWriteProtection
		{
		public:
			WriteProtection432(
				IMemoryManager* mm,
				uint32_t Register,
				uint16_t Bits,
				uint16_t Mask = 0xFFFF,
				uint16_t Pwd = 0x0,
				uint32_t unlockAddr = 0x0
				);

			virtual ~WriteProtection432() {}
			bool disableIfEnabled() { return true; }
			void restore() { return; }
			bool isEnabled();

		private:
			bool readSettings();

			IMemoryManager* mm;
			const uint32_t Register;
			const uint32_t unlockAddress;
			const uint16_t Bits;
			const uint16_t Mask;
			const uint16_t Pwd;
			uint16_t Value;
		};
	};
};
