/*
 * DeviceInfo.h 
 *
 * Data of device currently under control.
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

#include <MSP430_Debug.h>

#include "MemoryAreaBase.h"
#include "MpuFRx.h"
#include "WriteProtection.h"
#include "FuncletCode.h"
#include "../MSP432P4xx_FlashLibFc.h"
#include "../MSP432P4xx_PG11_FlashLibFc.h"
#include "../DAP_LOCK.h"

#include <boost/fusion/include/adapt_struct.hpp>
#include <boost/fusion/include/define_struct.hpp>

#include <DeviceDb/adapt_enum.h>


namespace TI
{
	namespace DLL430
	{
		DEFINE_ENUM(Architecture, (Cpu)(CpuX)(CpuXv2)(ArmCpu))

		DEFINE_ENUM(ClockSystemType, (BC_1xx)(BC_2xx)(FLLPLUS)(MOD_OSC))

		DEFINE_ENUM(PsaType, (Regular)(Enhanced))

		DEFINE_ENUM(MemoryType, (Flash)(Rom)(Ram)(Register))

		ADAPT_ENUM(DEVICE_CLOCK_CONTROL,
			(GCC_NONE)(GCC_STANDARD)(GCC_EXTENDED)(GCC_STANDARD_I))

		ADAPT_ENUM(EMEX_MODE,
		(EMEX_NONE)(EMEX_LOW)(EMEX_MEDIUM)(EMEX_HIGH)(EMEX_EXTRA_SMALL_5XX)(EMEX_SMALL_5XX)(EMEX_MEDIUM_5XX)(EMEX_LARGE_5XX)(EMEX_CORTEX_M4))

		inline void fromString(const char* txt, TI::DLL430::MemoryArea::Name& v)
		{
			std::string str(txt);

			//Memory name can continue after initial part (ie. Ram2, CpuXYZ, etc.)
			if      (str.find("None") == 0) v = TI::DLL430::MemoryArea::None;
			else if (str.find("Main") == 0) v = TI::DLL430::MemoryArea::Main;
			else if (str.find("Info") == 0) v = TI::DLL430::MemoryArea::Info;
			else if (str.find("Bsl") == 0) v = TI::DLL430::MemoryArea::Bsl;
			else if (str.find("BootCode") == 0) v = TI::DLL430::MemoryArea::BootCode;
			else if (str.find("Ram") == 0) v = TI::DLL430::MemoryArea::Ram;
			else if (str.find("UsbRam") == 0) v = TI::DLL430::MemoryArea::UsbRam;
			else if (str.find("Lcd") == 0) v = TI::DLL430::MemoryArea::Lcd;
			else if (str.find("Cpu") == 0) v = TI::DLL430::MemoryArea::Cpu;
			else if (str.find("Eem") == 0) v = TI::DLL430::MemoryArea::Eem;
			else if (str.find("Peripheral8bit") == 0) v = TI::DLL430::MemoryArea::Peripheral8bit;
			else if (str.find("Peripheral16bit") == 0) v = TI::DLL430::MemoryArea::Peripheral16bit;
			else if (str.find("IrVec") == 0) v = TI::DLL430::MemoryArea::IrVec;
			else if (str.find("Lib") == 0) v = TI::DLL430::MemoryArea::Lib;
			else if (str.find("LeaPeripheral") == 0) v = TI::DLL430::MemoryArea::LeaPeripheral;
			else if (str.find("LeaRam") == 0) v = TI::DLL430::MemoryArea::LeaRam; 
			else if (str.find("TinyRam") == 0) v = TI::DLL430::MemoryArea::TinyRam;
			else if (str.find("MidRom") == 0) v = TI::DLL430::MemoryArea::MidRom;
			else if (str.find("UssPeripheral") == 0) v = TI::DLL430::MemoryArea::UssPeripheral;
			else throw std::runtime_error("invalid memory name: " + str);
		}
	}
}


BOOST_FUSION_DEFINE_STRUCT((TI)(DLL430), IdCode,
	(uint16_t, version)
	(uint16_t, subversion)
	(uint8_t, revision)
	(uint8_t, maxRevision)
	(uint8_t, fab)
	(uint16_t, self)
	(uint8_t, config)
	(uint8_t, fuses)
	(uint32_t, activationKey)
)

BOOST_FUSION_DEFINE_STRUCT((TI)(DLL430), IdMask,
	(uint16_t, version)
	(uint16_t, subversion)
	(uint8_t, revision)
	(uint8_t, maxRevision)
	(uint8_t, fab)
	(uint16_t, self)
	(uint8_t, config)
	(uint8_t, fuses)
	(uint32_t, activationKey)
)

BOOST_FUSION_DEFINE_STRUCT((TI)(DLL430), ClockPair,
	(std::string, name)
	(uint8_t, value)
	(bool, defaultStop)
)

BOOST_FUSION_DEFINE_STRUCT((TI)(DLL430), PowerSettings,
	(uint32_t, testRegMask)
	(uint32_t, testRegDefault)
	(uint32_t, testRegEnableLpm5)
	(uint32_t, testRegDisableLpm5)
	(uint16_t, testReg3VMask)
	(uint16_t, testReg3VDefault)
	(uint16_t, testReg3VEnableLpm5)
	(uint16_t, testReg3VDisableLpm5)
)

BOOST_FUSION_DEFINE_STRUCT((TI)(DLL430), Features,
	(TI::DLL430::ClockSystemType, clockSystem)
	(bool, i2c)
	(bool, lcfe)
	(bool, quickMemRead)
	(bool, stopFllDbg)
	(bool, hasFram)
)

BOOST_FUSION_DEFINE_STRUCT((TI)(DLL430), ExtendedFeatures,
	(bool, tmr)
	(bool, jtag)
	(bool, dtc)
	(bool, sync)
	(bool, instr)
	(bool, _1377)
	(bool, psach)
	(bool, eemInaccessibleInLPM)
)

BOOST_FUSION_DEFINE_STRUCT((TI)(DLL430), VoltageInfo,
	(uint16_t, vccMin)
	(uint16_t, vccMax)
	(uint16_t, vccFlashMin)
	(uint16_t, vccSecureMin)
	(uint16_t, vppSecureMin)
	(uint16_t, vppSecureMax)
	(bool, testVpp)
)


namespace TI
{
	namespace DLL430
	{
		typedef std::map<hal_id, hal_id> FunctionMapping;

		typedef std::map<FuncletCode::Type, FuncletCode> FuncletMapping;

		static const size_t nrUsedClockModules = 32;
		typedef std::array<ClockPair, 32> ClockMapping;

		typedef std::array<std::string, 32> ClockNames;
	}
}

BOOST_FUSION_DEFINE_STRUCT((TI)(DLL430), ClockInfo,
	(DEVICE_CLOCK_CONTROL, clockControl)
	(uint32_t, mclkCntrl0)
	(TI::DLL430::ClockMapping, eemTimers)
	(TI::DLL430::ClockNames, eemClocks)
)

namespace TI
{
	namespace DLL430
	{
		struct WriteProtectionInfo
		{
			uint32_t address;
			uint32_t unlockAddress;
			uint16_t bits;
			uint16_t mask;
			uint16_t pwd;

			WriteProtectionInfo() : address(0), unlockAddress(0), bits(0), mask(0xFFFF), pwd(0) {}
		};
	}
}

BOOST_FUSION_ADAPT_STRUCT(TI::DLL430::WriteProtectionInfo,
	(uint32_t, address)
	(uint32_t, unlockAddress)
	(uint16_t, bits)
	(uint16_t, mask)
	(uint16_t, pwd)
	)

BOOST_FUSION_DEFINE_STRUCT((TI)(DLL430), MemoryAccess,
	(std::string, type)
	(bool, mpu)
	(TI::DLL430::WriteProtectionInfo, writeProtection)
)


namespace TI
{
	namespace DLL430
	{
		struct MemoryCreatorBase
		{
			virtual TI::DLL430::MemoryAreaBase* operator()(MemoryArea::Name, TI::DLL430::IDeviceHandle*,
				uint32_t, uint32_t, uint32_t, uint32_t,
				bool, const bool, IMemoryManager*, uint8_t) const = 0;
		};
		typedef std::shared_ptr<TI::DLL430::MemoryCreatorBase> MemoryCreatorPtr;

		template<class MemoryType>
		struct MemoryCreatorFR : public TI::DLL430::MemoryCreatorBase
		{
			MemoryCreatorFR(const MemoryAccess& access)
				: hasMpu(access.mpu)
				, address(access.writeProtection.address)
				, bits(access.writeProtection.bits)
				, mask(access.writeProtection.mask)
				, pwd(access.writeProtection.pwd) {}

			virtual TI::DLL430::MemoryAreaBase* operator()(MemoryArea::Name name,
				TI::DLL430::IDeviceHandle* devHandle,
				uint32_t start, uint32_t end,
				uint32_t seg, uint32_t banks,
				bool mapped, const bool protectable,
				TI::DLL430::IMemoryManager* mm, uint8_t psa) const
			{
				IMpu* mpu = hasMpu ? static_cast<IMpu*>(new MpuFRx(devHandle, mm)) : static_cast<IMpu*>(new NoMpu());
				IWriteProtection* writeProt = address ?
					static_cast<IWriteProtection*>(new WriteProtection(mm, address, bits, mask, pwd)) :
					static_cast<IWriteProtection*>(new NoWriteProtection(mm));
				return new MemoryType(name, devHandle, start, end, seg, banks, mapped, protectable, mm, psa, writeProt, mpu);
			}

			const bool hasMpu;
			const uint32_t address;
			const uint16_t bits;
			const uint16_t mask;
			const uint16_t pwd;
		};

		template<class MemoryType>
		struct MemoryCreator432 : public TI::DLL430::MemoryCreatorBase
		{
			MemoryCreator432(const MemoryAccess& access)
				: address(access.writeProtection.address)
				, bits(access.writeProtection.bits)
				, mask(access.writeProtection.mask)
				, pwd(access.writeProtection.pwd)
				, unlockAddress(access.writeProtection.unlockAddress) {}

			virtual TI::DLL430::MemoryAreaBase* operator()(MemoryArea::Name name,
				TI::DLL430::IDeviceHandle* devHandle,
				uint32_t start, uint32_t end,
				uint32_t seg, uint32_t banks,
				bool mapped, const bool protectable,
				TI::DLL430::IMemoryManager* mm, uint8_t psa) const
			{
				IWriteProtection* writeProt = address ?
					static_cast<IWriteProtection*>(new WriteProtection432(mm, address, bits, mask, pwd, unlockAddress)) :
					static_cast<IWriteProtection*>(new NoWriteProtection(mm));
				return new MemoryType(name, devHandle, start, end, seg, banks, mapped, protectable, mm, psa, writeProt);
			}

			const uint32_t address;
			const uint32_t unlockAddress;
			const uint16_t bits;
			const uint16_t mask;
			const uint16_t pwd;
		};

		template<class MemoryType>
		struct MemoryCreator : public TI::DLL430::MemoryCreatorBase
		{
			virtual TI::DLL430::MemoryAreaBase* operator()(MemoryArea::Name name,
				TI::DLL430::IDeviceHandle* devHandle,
				uint32_t start, uint32_t end,
				uint32_t seg, uint32_t banks,
				bool mapped, const bool protectable,
				TI::DLL430::IMemoryManager* mm, uint8_t psa) const
			{
				return new MemoryType(name, devHandle, start, end, seg, banks, mapped, protectable, mm, psa);
			}
		};

		typedef std::vector<uint8_t> MemoryMask;

		struct MemoryInfo
		{
			MemoryArea::Name name;
			MemoryType type;
			uint8_t bits;
			uint32_t size;
			uint32_t start;
			uint32_t segmentSize;
			uint32_t banks;
			bool mapped;
			bool protectable;
			MemoryMask mask;
			MemoryCreatorPtr memoryCreatorPtr;

			MemoryInfo() :
				name(MemoryArea::None),
				type(MemoryType::Flash),
				bits(0),
				size(0),
				start(0),
				segmentSize(0),
				banks(0),
				mapped(false),
				protectable(false)
			{}
		};
	}
}

BOOST_FUSION_ADAPT_STRUCT(TI::DLL430::MemoryInfo,
	(TI::DLL430::MemoryArea::Name, name)
	(TI::DLL430::MemoryType, type)
	(uint8_t, bits)
	(uint32_t, size)
	(uint32_t, start)
	(uint32_t, segmentSize)
	(uint32_t, banks)
	(bool, mapped)
	(bool, protectable)
)

namespace TI
{
	namespace DLL430
	{
		typedef std::map<std::string, MemoryInfo> MemoryLayout;
	}
}


BOOST_FUSION_DEFINE_STRUCT((TI)(DLL430), DeviceInfo,
	(std::string, description)
	(std::string, warning)
	(uint8_t, bits)
	(TI::DLL430::PsaType, psa)
	(TI::DLL430::Architecture, architecture)
	(EMEX_MODE, eem)
	(TI::DLL430::IdMask, idMask)
	(TI::DLL430::IdCode, idCode)
	(TI::DLL430::VoltageInfo, voltageInfo)
	(TI::DLL430::ClockInfo, clockInfo)
	(TI::DLL430::FunctionMapping, functionMap)
	(TI::DLL430::FuncletMapping, funcletMap)
	(TI::DLL430::Features, features)
	(TI::DLL430::ExtendedFeatures, extFeatures)
	(TI::DLL430::PowerSettings, powerSettings)
	(TI::DLL430::MemoryLayout, memoryLayout)
)


namespace TI
{
	namespace DLL430
	{
		struct Match
		{
			IdMask idMask;
			IdCode idCode;
		};

		inline bool operator<(const Match& lhs, const Match& rhs)
		{
			if (lhs.idCode.version != rhs.idCode.version)
				return lhs.idCode.version < rhs.idCode.version;

			else if (lhs.idCode.subversion != rhs.idCode.subversion)
				return  lhs.idCode.subversion < rhs.idCode.subversion;

			else if (lhs.idCode.revision != rhs.idCode.revision)
				return  lhs.idCode.revision < rhs.idCode.revision;

			else if (lhs.idCode.fab != rhs.idCode.fab)
				return  lhs.idCode.fab < rhs.idCode.fab;

			else if (lhs.idCode.self != rhs.idCode.self)
				return  lhs.idCode.self < rhs.idCode.self;

			else if (lhs.idCode.config != rhs.idCode.config)
				return  lhs.idCode.config < rhs.idCode.config;

			else if (lhs.idCode.fuses != rhs.idCode.fuses)
				return  lhs.idCode.fuses < rhs.idCode.fuses;

			else
				return  lhs.idCode.activationKey < rhs.idCode.activationKey;
		}

		inline bool operator==(const Match& match, const IdCode& idCode)
		{
			bool revisionMatch = ((idCode.revision & match.idMask.revision) == match.idCode.revision);
			if (match.idMask.maxRevision != 0)
			{
				revisionMatch = ((idCode.revision & match.idMask.revision) >= match.idCode.revision) &&
					((idCode.revision & match.idMask.maxRevision) <= match.idCode.maxRevision);
			}

			return ((idCode.version & match.idMask.version) == match.idCode.version
				&& (idCode.subversion & match.idMask.subversion) == match.idCode.subversion
				&& revisionMatch
				&& (idCode.fab & match.idMask.fab) == match.idCode.fab
				&& (idCode.self & match.idMask.self) == match.idCode.self
				&& (idCode.config & match.idMask.config) == match.idCode.config
				&& (idCode.fuses & match.idMask.fuses) == match.idCode.fuses
				&& (idCode.activationKey & match.idMask.activationKey) == match.idCode.activationKey);
		}

		typedef std::map<const Match, const DeviceInfo> DeviceMap;
	}
}
