/*
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

#include <pch.h>

#include "exportxml.h"

#include <hal.h>
#include <LockableRamMemoryAccess.h>
#include <SpecialMemoryTypes.h>
#include <FramMemoryAccessFRx9.h>
#include <TinyRandomMemoryAccess.h>
#include <MpuFRx.h>
#include <WriteProtection.h>
#include <RegisterAccessBase.h>

#include <ArmFlashMemoryAccess.h>
#include <ArmRandomMemoryAccess.h>
#include <ArmReadonlyMemroyAccess.h>
#include <ArmSpecialFlashMemoryAccess.h>
#include <MSP432P4xx_2M_FlashLibFc.h>

#include "DeviceInfo.h"
#include "pugixml.hpp"


using namespace TI::DLL430;
using namespace DeviceDb;


static inline pugi::xml_node addXmlElement(pugi::xml_node parent, const char* name)
{
	return parent.append_child(name);
}

static inline pugi::xml_node addXmlElement(pugi::xml_node parent, const char* name, const char* value)
{
	pugi::xml_node e = parent.append_child(name);
	e.text().set(value);
	return e;
}

static inline pugi::xml_node addXmlElement(pugi::xml_node parent, const char* name, const std::string& value)
{
	return addXmlElement(parent, name, value.c_str());
}

template<typename T>
static pugi::xml_node addXmlElement(pugi::xml_node parent, const char* name, const T& value, decltype(std::hex) base = std::hex)
{
	std::stringstream s;
	s << std::boolalpha << std::showbase << base << value;
	return addXmlElement(parent, name, s.str());
}

static inline pugi::xml_node addXmlElement(pugi::xml_node parent, const char* name, const uint8_t& value, decltype(std::hex) base = std::hex)
{
	return addXmlElement(parent, name, (unsigned)value, base);
}

static inline pugi::xml_node addXmlElement(pugi::xml_node parent, const char* name, const int8_t& value, decltype(std::hex) base = std::hex)
{
	return addXmlElement(parent, name, (int)value, base);
}


void DeviceDb::exportXml(const DeviceMap& deviceMap, const char *filename)
{
#define MACRO(x)  #x,
	const char* macroStrings[] = {
		MACRO(Zero)
		MACRO_LIST
	};
#undef MACRO
	static const bool FULL_EXPORT = false;

	pugi::xml_document doc;
	auto xmlRoot = addXmlElement(doc, "device-information");
	xmlRoot.append_attribute("version").set_value("1.0");

	for (const auto& entry : deviceMap)
	{
		const DeviceInfo& device = entry.second;

		auto xmlDevice = addXmlElement(xmlRoot, "device");

		addXmlElement(xmlDevice, "description", device.description);

		const IdMask& mask = device.idMask;
		const IdCode& value = device.idCode;

		auto xmlIdMask = addXmlElement(xmlDevice, "idMask");
		addXmlElement(xmlIdMask, "version", mask.version);
		if (FULL_EXPORT || mask.subversion != 0) addXmlElement(xmlIdMask, "subversion", mask.subversion);
		if (FULL_EXPORT || mask.revision != 0) addXmlElement(xmlIdMask, "revision", mask.revision);
		if (FULL_EXPORT || mask.maxRevision != 0) addXmlElement(xmlIdMask, "maxRevision", mask.maxRevision);
		if (FULL_EXPORT || mask.fab != 0) addXmlElement(xmlIdMask, "fab", mask.fab);
		if (FULL_EXPORT || mask.self != 0) addXmlElement(xmlIdMask, "self", mask.self);
		if (FULL_EXPORT || mask.config != 0) addXmlElement(xmlIdMask, "config", (int)mask.config);
		if (FULL_EXPORT || mask.fuses != 0) addXmlElement(xmlIdMask, "fuses", mask.fuses);
		if (FULL_EXPORT || mask.activationKey != 0) addXmlElement(xmlIdMask, "activationKey", mask.activationKey);

		auto xmlIdCode = addXmlElement(xmlDevice, "idCode");
		addXmlElement(xmlIdCode, "version", value.version);
		if (FULL_EXPORT || mask.subversion != 0) addXmlElement(xmlIdCode, "subversion", value.subversion);
		if (FULL_EXPORT || mask.revision != 0) addXmlElement(xmlIdCode, "revision", value.revision);
		if (FULL_EXPORT || mask.maxRevision != 0) addXmlElement(xmlIdCode, "maxRevision", value.maxRevision);
		if (FULL_EXPORT || mask.fab != 0) addXmlElement(xmlIdCode, "fab", value.fab);
		if (FULL_EXPORT || mask.self != 0) addXmlElement(xmlIdCode, "self", value.self);
		if (FULL_EXPORT || mask.config != 0) addXmlElement(xmlIdCode, "config", (int)value.config);
		if (FULL_EXPORT || mask.fuses != 0) addXmlElement(xmlIdCode, "fuses", value.fuses);
		if (FULL_EXPORT || mask.activationKey != 0) addXmlElement(xmlIdCode, "activationKey", value.activationKey);

		addXmlElement(xmlDevice, "psa", toString(device.psa));
		addXmlElement(xmlDevice, "bits", device.bits, std::dec);
		addXmlElement(xmlDevice, "architecture", toString(device.architecture));
		addXmlElement(xmlDevice, "eem", toString(device.eem));

		auto xmlClockInfo = addXmlElement(xmlDevice, "clockInfo");
		addXmlElement(xmlClockInfo, "clockControl", toString(device.clockInfo.clockControl));

		int idx = 16;
	
		auto xmlEemClocks = addXmlElement(xmlClockInfo, "eemClocks");
		for (const auto& clock : device.clockInfo.eemClocks)
		{
			if (!clock.empty())
			{
				addXmlElement(xmlEemClocks, "eemClock", clock).append_attribute("index").set_value(idx -16);
			}
			++idx;
		}

		idx = 16;
		auto xmlEemTimers = addXmlElement(xmlClockInfo, "eemTimers");
		for (const auto& timer : device.clockInfo.eemTimers)
		{
			if (FULL_EXPORT || !timer.name.empty())
			{
				auto eemTimer = addXmlElement(xmlEemTimers, "eemTimer");
				eemTimer.append_attribute("index").set_value(idx);
				addXmlElement(eemTimer, "name", timer.name);
				addXmlElement(eemTimer, "value", timer.value);
				const bool defaultStop = (device.clockInfo.mclkCntrl0 & (1 << (32 - idx))) != 0;
				if (FULL_EXPORT || defaultStop) addXmlElement(eemTimer, "defaultStop", defaultStop);
			}
			++idx;
		}

		auto xmlVoltage = addXmlElement(xmlDevice, "voltageInfo");
		addXmlElement(xmlVoltage, "vccMin", device.voltageInfo.vccMin, std::dec);
		addXmlElement(xmlVoltage, "vccMax", device.voltageInfo.vccMax, std::dec);
		addXmlElement(xmlVoltage, "vccFlashMin", device.voltageInfo.vccFlashMin, std::dec);
		addXmlElement(xmlVoltage, "vccSecureMin", device.voltageInfo.vccSecureMin, std::dec);
		addXmlElement(xmlVoltage, "vppSecureMin", device.voltageInfo.vppSecureMin, std::dec);
		addXmlElement(xmlVoltage, "vppSecureMax", device.voltageInfo.vppSecureMax, std::dec);
		addXmlElement(xmlVoltage, "testVpp", device.voltageInfo.testVpp, std::dec);

		auto xmlFeatures = addXmlElement(xmlDevice, "features");
		addXmlElement(xmlFeatures, "clockSystem", toString(device.features.clockSystem));
		if (FULL_EXPORT || device.features.lcfe) addXmlElement(xmlFeatures, "lcfe", device.features.lcfe, std::dec);
		if (FULL_EXPORT || device.features.quickMemRead) addXmlElement(xmlFeatures, "quickMemRead", device.features.quickMemRead, std::dec);
		if (FULL_EXPORT || device.features.i2c) addXmlElement(xmlFeatures, "i2c", device.features.i2c, std::dec);
		if (FULL_EXPORT || device.features.stopFllDbg) addXmlElement(xmlFeatures, "stopFllDbg", device.features.stopFllDbg, std::dec);
		if (FULL_EXPORT || device.features.hasFram) addXmlElement(xmlFeatures, "hasFram", device.features.hasFram, std::dec);

		auto xmlExtFeatures = addXmlElement(xmlDevice, "extFeatures");
		if (FULL_EXPORT || device.extFeatures.tmr) addXmlElement(xmlExtFeatures, "tmr", device.extFeatures.tmr, std::dec);
		if (FULL_EXPORT || device.extFeatures.jtag) addXmlElement(xmlExtFeatures, "jtag", device.extFeatures.jtag, std::dec);
		if (FULL_EXPORT || device.extFeatures.dtc) addXmlElement(xmlExtFeatures, "dtc", device.extFeatures.dtc, std::dec);
		if (FULL_EXPORT || device.extFeatures.sync) addXmlElement(xmlExtFeatures, "sync", device.extFeatures.sync, std::dec);
		if (FULL_EXPORT || device.extFeatures.instr) addXmlElement(xmlExtFeatures, "instr", device.extFeatures.instr, std::dec);
		if (FULL_EXPORT || device.extFeatures._1377) addXmlElement(xmlExtFeatures, "_1377", device.extFeatures._1377, std::dec);
		if (FULL_EXPORT || device.extFeatures.psach) addXmlElement(xmlExtFeatures, "psach", device.extFeatures.psach, std::dec);
		if (FULL_EXPORT || device.extFeatures.eemInaccessibleInLPM) addXmlElement(xmlExtFeatures, "eemInaccessibleInLPM", device.extFeatures.eemInaccessibleInLPM, std::dec);

		auto xmlPower = addXmlElement(xmlDevice, "powerSettings");
		if (FULL_EXPORT || device.powerSettings.testRegMask != 0)
		{
			addXmlElement(xmlPower, "testRegMask", device.powerSettings.testRegMask);
			addXmlElement(xmlPower, "testRegDefault", device.powerSettings.testRegDefault);
			addXmlElement(xmlPower, "testRegEnableLpm5", device.powerSettings.testRegEnableLpm5);
			addXmlElement(xmlPower, "testRegDisableLpm5", device.powerSettings.testRegDisableLpm5);
		}
		if (FULL_EXPORT || device.powerSettings.testReg3VMask)
		{
			addXmlElement(xmlPower, "testReg3VMask", device.powerSettings.testReg3VMask);
			addXmlElement(xmlPower, "testReg3VDefault", device.powerSettings.testReg3VDefault);
			addXmlElement(xmlPower, "testReg3VEnableLpm5", device.powerSettings.testReg3VEnableLpm5);
			addXmlElement(xmlPower, "testReg3VDisableLpm5", device.powerSettings.testReg3VDisableLpm5);
		}

		auto xmlMemoryMap = addXmlElement(xmlDevice, "memoryLayout");
		for (const auto& memoryArea : device.memoryLayout)
		{
			const auto& mem = memoryArea.second;

			auto xmlMemory = addXmlElement(xmlMemoryMap, "memory");
			xmlMemory.append_attribute("name").set_value(memoryArea.first.c_str());

			addXmlElement(xmlMemory, "type", toString(mem.type));

			if (FULL_EXPORT || mem.bits != 0) addXmlElement(xmlMemory, "bits", mem.bits, std::dec);
			addXmlElement(xmlMemory, "start", mem.start);
			addXmlElement(xmlMemory, "size", mem.size);
			addXmlElement(xmlMemory, "segmentSize", mem.segmentSize);
			addXmlElement(xmlMemory, "banks", mem.banks, std::dec);
			addXmlElement(xmlMemory, "mapped", mem.mapped, std::dec);
			if (FULL_EXPORT || mem.protectable) addXmlElement(xmlMemory, "protectable", mem.protectable);

			uint64_t mask = 0;
			for (const uint8_t m : mem.mask) mask <<= 8, mask |= static_cast<uint64_t>(m);
			if (FULL_EXPORT || mask != 0) addXmlElement(xmlMemory, "mask", mask);

			if (mem.memoryCreatorPtr)
			{
				auto xmlAccess = addXmlElement(xmlMemory, "memoryAccess");

				const auto exportAccess = [&xmlAccess](const std::string& type, bool hasMpu,
					uint32_t address, uint16_t bits, uint16_t mask, uint16_t pwd)
				{
					addXmlElement(xmlAccess, "type", type);
					if (FULL_EXPORT || hasMpu) addXmlElement(xmlAccess, "mpu", hasMpu, std::dec);
					if (address)
					{
						auto xmlWriteProt = addXmlElement(xmlAccess, "writeProtection");
						addXmlElement(xmlWriteProt, "address", address);
						addXmlElement(xmlWriteProt, "bits", bits);
						if (FULL_EXPORT || mask != 0xFFFF) addXmlElement(xmlWriteProt, "mask", mask);
						if (FULL_EXPORT || pwd) addXmlElement(xmlWriteProt, "pwd", pwd);
					}
				};

				if (typeid(*mem.memoryCreatorPtr) == typeid(MemoryCreator<LockableRamMemoryAccess>)) addXmlElement(xmlAccess, "type", "LockableRamMemoryAccess");
				else if (typeid(*mem.memoryCreatorPtr) == typeid(MemoryCreator<InformationFlashAccess>)) addXmlElement(xmlAccess, "type", "InformationFlashAccess");
				else if (typeid(*mem.memoryCreatorPtr) == typeid(MemoryCreator<BootcodeRomAccess>)) addXmlElement(xmlAccess, "type", "BootcodeRomAccess");
				else if (typeid(*mem.memoryCreatorPtr) == typeid(MemoryCreator<FlashMemoryAccess2ByteAligned>)) addXmlElement(xmlAccess, "type", "FlashMemoryAccess2ByteAligned");
				else if (typeid(*mem.memoryCreatorPtr) == typeid(MemoryCreator<BslFlashAccess>)) addXmlElement(xmlAccess, "type", "BslFlashAccess");
				else if (typeid(*mem.memoryCreatorPtr) == typeid(MemoryCreator<BslRomAccess>)) addXmlElement(xmlAccess, "type", "BslRomAccess");
				else if (typeid(*mem.memoryCreatorPtr) == typeid(MemoryCreator<BslRomAccessGR>)) addXmlElement(xmlAccess, "type", "BslRomAccessGR");
				else if (typeid(*mem.memoryCreatorPtr) == typeid(MemoryCreator<UsbRamAccess>)) addXmlElement(xmlAccess, "type", "UsbRamAccess");
				else if (typeid(*mem.memoryCreatorPtr) == typeid(MemoryCreator<RegisterAccess5xx>)) addXmlElement(xmlAccess, "type", "RegisterAccess5xx");
				else if (typeid(*mem.memoryCreatorPtr) == typeid(MemoryCreator<TinyRandomMemoryAccess>)) addXmlElement(xmlAccess, "type", "TinyRandomMemoryAccess");
				else if (MemoryCreatorFR<FramMemoryAccessBase>* mc = dynamic_cast<MemoryCreatorFR<FramMemoryAccessBase>*>(mem.memoryCreatorPtr.get()))
					exportAccess("FramMemoryAccessBase", mc->hasMpu, mc->address, mc->bits, mc->mask, mc->pwd);
				else if (MemoryCreatorFR<FramMemoryAccessFRx9>* mc = dynamic_cast<MemoryCreatorFR<FramMemoryAccessFRx9>*>(mem.memoryCreatorPtr.get()))
					exportAccess("FramMemoryAccessFRx9", mc->hasMpu, mc->address, mc->bits, mc->mask, mc->pwd);
				else if (typeid(*mem.memoryCreatorPtr) == typeid(MemoryCreator<ArmRandomMemoryAccess>)) addXmlElement(xmlAccess, "type", "ArmRandomMemoryAccess");
				else if (typeid(*mem.memoryCreatorPtr) == typeid(MemoryCreator<ArmReadonlyMemroyAccess>)) addXmlElement(xmlAccess, "type", "ArmReadonlyMemroyAccess");
				else if (typeid(*mem.memoryCreatorPtr) == typeid(MemoryCreator<ArmBslFlashMemoryAccess>)) addXmlElement(xmlAccess, "type", "ArmBslFlashMemoryAccess");
				else if (typeid(*mem.memoryCreatorPtr) == typeid(MemoryCreator<ArmInfoFlashMemoryAccess>)) addXmlElement(xmlAccess, "type", "ArmInfoFlashMemoryAccess");
				else if (typeid(*mem.memoryCreatorPtr) == typeid(MemoryCreator432<ArmFlashMemoryAccess>)) addXmlElement(xmlAccess, "type", "ArmFlashMemoryAccess");
				else if (typeid(*mem.memoryCreatorPtr) == typeid(MemoryCreator432<ArmFlashMemoryAccess2M>)) addXmlElement(xmlAccess, "type", "ArmFlashMemoryAccess2M");
				else if (typeid(*mem.memoryCreatorPtr) == typeid(MemoryCreator<ArmBslFlashMemoryAccess2M>)) addXmlElement(xmlAccess, "type", "ArmBslFlashMemoryAccess2M");
				else if (typeid(*mem.memoryCreatorPtr) == typeid(MemoryCreator<ArmInfoFlashMemoryAccess2M>)) addXmlElement(xmlAccess, "type", "ArmInfoFlashMemoryAccess2M");
				else throw std::runtime_error("Unknown memory creator: " + std::string(typeid(*mem.memoryCreatorPtr).name()));
			}
		}
		auto xmlFunctions = addXmlElement(xmlDevice, "functionMap");
		for (const auto& it : device.functionMap)
		{
			addXmlElement(xmlFunctions, macroStrings[it.first], macroStrings[it.second]);
		}

		static const FuncletCode funcletEraseDco = FuncletCode(eraseFuncletCodeDCO, sizeof(eraseFuncletCodeDCO), 4);
		static const FuncletCode funcletEraseXDco = FuncletCode(eraseFuncletCodeXDCO, sizeof(eraseFuncletCodeXDCO), 4);
		static const FuncletCode funcletEraseFll = FuncletCode(eraseFuncletCodeFLL, sizeof(eraseFuncletCodeFLL), 4);
		static const FuncletCode funcletEraseXFll = FuncletCode(eraseFuncletCodeXFLL, sizeof(eraseFuncletCodeXFLL), 4);
		static const FuncletCode funcletEraseXv2 = FuncletCode(eraseFuncletCodeXv2, sizeof(eraseFuncletCodeXv2));
		static const FuncletCode funcletEraseXv2Fram = FuncletCode(eraseFuncletCodeXv2FRAM, sizeof(eraseFuncletCodeXv2FRAM));
		static const FuncletCode funcletEraseXv2FR41xx = FuncletCode(eraseFuncletCodeXv2FR41xx, sizeof(eraseFuncletCodeXv2FR41xx));

		static const FuncletCode funcletWriteDco = FuncletCode(writeFuncletCodeDCO, sizeof(writeFuncletCodeDCO), 128);
		static const FuncletCode funcletWriteXDco = FuncletCode(writeFuncletCodeXDCO, sizeof(writeFuncletCodeXDCO), 256);
		static const FuncletCode funcletWrite430I = FuncletCode(writeFuncletCode430I, sizeof(writeFuncletCode430I), 128);
		static const FuncletCode funcletWriteFll = FuncletCode(writeFuncletCodeFLL, sizeof(writeFuncletCodeFLL), 128);
		static const FuncletCode funcletWriteXFll = FuncletCode(writeFuncletCodeXFLL, sizeof(writeFuncletCodeXFLL), 256);
		static const FuncletCode funcletWriteXv2 = FuncletCode(writeFuncletCodeXv2, sizeof(writeFuncletCodeXv2));
		static const FuncletCode funcletWriteXv2Fram = FuncletCode(writeFuncletCodeXv2FRAM, sizeof(writeFuncletCodeXv2FRAM));
		static const FuncletCode funcletWriteXv2Word = FuncletCode(writeFuncletCodeXv2WordMode, sizeof(writeFuncletCodeXv2WordMode));

		static const FuncletCode funcletMSP432P4xx_2M_Funclet = FuncletCode(MSP432P4xx_2M_Funclet, sizeof(MSP432P4xx_2M_Funclet) + 2, 0x4000);
		static const FuncletCode funcletMSP432P4xx_PG11_Funclet = FuncletCode(MSP432P4xx_PG11_Funclet, sizeof(MSP432P4xx_PG11_Funclet) + 2, 0x5000);
		static const FuncletCode funcletMSP432P4xx_Funclet = FuncletCode(MSP432P4xx_Funclet, sizeof(MSP432P4xx_Funclet) + 2, 0x4000);

		auto xmlFunclets = addXmlElement(xmlDevice, "funcletMap");
		for (const auto& it : device.funcletMap)
		{
			if (it.first == FuncletCode::ERASE)
			{
				if      (it.second == funcletEraseXv2) addXmlElement(xmlFunclets, "eraseFunclet", "EraseXv2");
				else if (it.second == funcletEraseXv2Fram) addXmlElement(xmlFunclets, "eraseFunclet", "EraseXv2FRAM");
				else if (it.second == funcletEraseDco) addXmlElement(xmlFunclets, "eraseFunclet", "EraseDCO");
				else if (it.second == funcletEraseXDco) addXmlElement(xmlFunclets, "eraseFunclet", "EraseXDCO");
				else if (it.second == funcletEraseFll) addXmlElement(xmlFunclets, "eraseFunclet", "EraseFLL");
				else if (it.second == funcletEraseXFll) addXmlElement(xmlFunclets, "eraseFunclet", "EraseXFLL");
				else if (it.second == funcletEraseXv2FR41xx) addXmlElement(xmlFunclets, "eraseFunclet", "EraseFR41xx");
				else if (it.second == funcletMSP432P4xx_Funclet) addXmlElement(xmlFunclets, "eraseFunclet", "MSP432P4xx_Funclet");
				else if (it.second == funcletMSP432P4xx_2M_Funclet) addXmlElement(xmlFunclets, "eraseFunclet", "MSP432P4xx_2M_Funclet");
				else if (it.second == funcletMSP432P4xx_PG11_Funclet) addXmlElement(xmlFunclets, "eraseFunclet", "MSP432P4xx_PG11_Funclet");
				else throw std::runtime_error("Unknown erase funclet");
			}
			if (it.first == FuncletCode::WRITE)
			{
				if      (it.second == funcletWriteXv2) addXmlElement(xmlFunclets, "writeFunclet", "WriteXv2");
				else if (it.second == funcletWriteXv2Fram) addXmlElement(xmlFunclets, "writeFunclet", "WriteXv2FRAM");
				else if (it.second == funcletWriteXv2Word) addXmlElement(xmlFunclets, "writeFunclet", "WriteXv2Word");
				else if (it.second == funcletWriteDco) addXmlElement(xmlFunclets, "writeFunclet", "WriteDCO");
				else if (it.second == funcletWriteXDco) addXmlElement(xmlFunclets, "writeFunclet", "WriteXDCO");
				else if (it.second == funcletWriteFll) addXmlElement(xmlFunclets, "writeFunclet", "WriteFLL");
				else if (it.second == funcletWriteXFll) addXmlElement(xmlFunclets, "writeFunclet", "WriteXFLL");
				else if (it.second == funcletWrite430I) addXmlElement(xmlFunclets, "writeFunclet", "Write430I");
				else if (it.second == funcletMSP432P4xx_Funclet) addXmlElement(xmlFunclets, "eraseFunclet", "MSP432P4xx_Funclet");
				else if (it.second == funcletMSP432P4xx_2M_Funclet) addXmlElement(xmlFunclets, "eraseFunclet", "MSP432P4xx_2M_Funclet");
				else if (it.second == funcletMSP432P4xx_PG11_Funclet) addXmlElement(xmlFunclets, "eraseFunclet", "MSP432P4xx_PG11_Funclet");
				else throw std::runtime_error("Unknown write funclet");
			}
			if (it.first == FuncletCode::SECURE)
			{
				addXmlElement(xmlFunclets, "DapSecure", "MSP432P4xx_DapSecure");
			}
		}
	}
	doc.save_file(filename);
}
