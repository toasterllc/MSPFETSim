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

#include <unordered_map>
#include <DeviceInfo.h>

#include "fromxml.h"
#include "setmember.h"
#include "readelement.h"

#include <LockableRamMemoryAccess.h>
#include "SpecialMemoryTypes.h"
#include <FramMemoryAccessFRx9.h>
#include "MpuFRx.h"
#include "WriteProtection.h"
#include "RegisterAccessBase.h"
#include "ArmFlashMemoryAccess.h"
#include "ArmRandomMemoryAccess.h"
#include "ArmSpecialFlashMemoryAccess.h"
#include "ArmReadonlyMemroyAccess.h"
#include "TinyRandomMemoryAccess.h"
#include "../MSP432P4xx_FlashLibFc.h"
#include "../MSP432P4xx_PG11_FlashLibFc.h"
#include "../MSP432P4xx_2M_FlashLibFc.h"
#include "../DAP_LOCK.h"


using namespace std;
using namespace TI::DLL430;


typedef unordered_map<const string, hal_id, hash<string>> MacroStrings;
#define MACRO(x)  {#x, ID_##x},
static const MacroStrings macroStrings = { MACRO_LIST };
#undef MACRO


int getIndex(int maxIdx, const pugi::xml_node e)
{
	const pugi::xml_attribute index = e.attribute("index");
	if (index.empty())
		throw runtime_error("missing clock index");

	const int idx = index.as_int(0xFFFFFFFF);
	if (idx >= maxIdx)
		throw runtime_error("clock index out of range: " + to_string(idx));

	return idx;
}

void fromXml(ClockNames* clocks, const pugi::xml_node e)
{
	if (!strcmp(e.name(), "eemClock"))
	{
		const int idx = getIndex(static_cast<int>(clocks->size()), e);
		(*clocks)[idx] = e.text().get();
	}
}

void fromXml(ClockMapping* timers, const pugi::xml_node e)
{
	if (!strcmp(e.name(), "eemTimer"))
	{
		const int idx = getIndex(static_cast<int>(timers->size()), e);
		readElement(e, (*timers)[idx]);
	}
}

void fromXml(FunctionMapping* functionMap, const pugi::xml_node e)
{
	const char* name = e.name();
	const char* value = e.text().get();

	const MacroStrings::const_iterator from = macroStrings.find(name);
	const MacroStrings::const_iterator to = macroStrings.find(value);

	if (from == macroStrings.end())
		throw runtime_error(string("invalid hal macro: ") + name);

	if (to == macroStrings.end())
		throw runtime_error(string("invalid hal macro: ") + value);

	(*functionMap)[from->second] = to->second;
}

void fromXml(FuncletMapping* funcletMap, const pugi::xml_node e)
{
	FuncletCode::Type type = FuncletCode::NONE;
	FuncletCode funcletCode;

	const char* name = e.name();

	if      (!strcmp(name, "eraseFunclet")) type = FuncletCode::ERASE;
	else if (!strcmp(name, "writeFunclet")) type = FuncletCode::WRITE;
	else if (!strcmp(name, "DapSecure")) type = FuncletCode::SECURE;
	else throw runtime_error(string("invalid funclet type: ") + name);

	const string txt = e.text().get();
	if      (txt == "EraseDCO") funcletCode = FuncletCode(eraseFuncletCodeDCO, sizeof(eraseFuncletCodeDCO), 4);
	else if (txt == "EraseXDCO") funcletCode = FuncletCode(eraseFuncletCodeXDCO, sizeof(eraseFuncletCodeXDCO), 4);
	else if (txt == "EraseFLL") funcletCode = FuncletCode(eraseFuncletCodeFLL, sizeof(eraseFuncletCodeFLL), 4);
	else if (txt == "EraseXFLL") funcletCode = FuncletCode(eraseFuncletCodeXFLL, sizeof(eraseFuncletCodeXFLL), 4);
	else if (txt == "EraseXv2") funcletCode = FuncletCode(eraseFuncletCodeXv2, sizeof(eraseFuncletCodeXv2));
	else if (txt == "EraseXv2FRAM") funcletCode = FuncletCode(eraseFuncletCodeXv2FRAM, sizeof(eraseFuncletCodeXv2FRAM));
	else if (txt == "EraseFR41xx") funcletCode = FuncletCode(eraseFuncletCodeXv2FR41xx, sizeof(eraseFuncletCodeXv2FR41xx));
	else if (txt == "WriteDCO") funcletCode = FuncletCode(writeFuncletCodeDCO, sizeof(writeFuncletCodeDCO), 128);
	else if (txt == "WriteXDCO") funcletCode = FuncletCode(writeFuncletCodeXDCO, sizeof(writeFuncletCodeXDCO), 256);
	else if (txt == "Write430I") funcletCode = FuncletCode(writeFuncletCode430I, sizeof(writeFuncletCode430I), 128);
	else if (txt == "WriteFLL") funcletCode = FuncletCode(writeFuncletCodeFLL, sizeof(writeFuncletCodeFLL), 128);
	else if (txt == "WriteXFLL") funcletCode = FuncletCode(writeFuncletCodeXFLL, sizeof(writeFuncletCodeXFLL), 256);
	else if (txt == "WriteXv2") funcletCode = FuncletCode(writeFuncletCodeXv2, sizeof(writeFuncletCodeXv2));
	else if (txt == "WriteXv2FRAM") funcletCode = FuncletCode(writeFuncletCodeXv2FRAM, sizeof(writeFuncletCodeXv2FRAM));
	else if (txt == "WriteXv2Word") funcletCode = FuncletCode(writeFuncletCodeXv2WordMode, sizeof(writeFuncletCodeXv2WordMode));
	else if (txt == "MSP432P4xx_Funclet") funcletCode = FuncletCode(MSP432P4xx_Funclet, sizeof(MSP432P4xx_Funclet) + 2, 0x4000);
	else if (txt == "MSP432P4xx_2M_Funclet") funcletCode = FuncletCode(MSP432P4xx_2M_Funclet, sizeof(MSP432P4xx_2M_Funclet) + 2, 0x4000);
	else if (txt == "MSP432P4xx_PG11_Funclet") funcletCode = FuncletCode(MSP432P4xx_PG11_Funclet, sizeof(MSP432P4xx_PG11_Funclet) + 2, 0x5000);
	else if (txt == "MSP432P4xx_DapSecure") funcletCode = FuncletCode(DAP_LOCK, sizeof(DAP_LOCK) + 2);	
	else throw runtime_error("invalid funclet: " + txt);

	(*funcletMap)[type] = funcletCode;
}

void fromXml(MemoryInfo* memory, const pugi::xml_node e)
{
	const char* name = e.name();

	if (!strcmp(name, "mask"))
	{
		uint64_t mask = stoull(e.text().get(), 0, 16);
		memory->mask.resize(6);
		for (int i = 5; i >= 0; --i)
		{
			memory->mask[i] = (mask & 0xFF);
			mask >>= 8;
		}
	}
	else if (!strcmp(name, "memoryAccess"))
	{
		MemoryAccess access;
		readElement(e, access);

		if      (access.type == "LockableRamMemoryAccess") memory->memoryCreatorPtr = make_shared<MemoryCreator<LockableRamMemoryAccess>>();
		else if (access.type == "BootcodeRomAccess") memory->memoryCreatorPtr = make_shared<MemoryCreator<BootcodeRomAccess>>();
		else if (access.type == "InformationFlashAccess") memory->memoryCreatorPtr = make_shared<MemoryCreator<InformationFlashAccess>>();
		else if (access.type == "FlashMemoryAccess2ByteAligned") memory->memoryCreatorPtr = make_shared<MemoryCreator<FlashMemoryAccess2ByteAligned>>();
		else if (access.type == "BslFlashAccess") memory->memoryCreatorPtr = make_shared<MemoryCreator<BslFlashAccess>>();
		else if (access.type == "BslRomAccess") memory->memoryCreatorPtr = make_shared<MemoryCreator<BslRomAccess>>();
		else if (access.type == "BslRomAccessGR") memory->memoryCreatorPtr = make_shared<MemoryCreator<BslRomAccessGR>>();
		else if (access.type == "RegisterAccess5xx") memory->memoryCreatorPtr = make_shared<MemoryCreator<RegisterAccess5xx>>();
		else if (access.type == "TinyRandomMemoryAccess") memory->memoryCreatorPtr = make_shared<MemoryCreator<TinyRandomMemoryAccess>>();
		else if (access.type == "UsbRamAccess") memory->memoryCreatorPtr = make_shared<MemoryCreator<UsbRamAccess>>();
		else if (access.type == "FramMemoryAccessFRx9") memory->memoryCreatorPtr = make_shared<MemoryCreatorFR<FramMemoryAccessFRx9>>(access);
		else if (access.type == "FramMemoryAccessBase") memory->memoryCreatorPtr = make_shared<MemoryCreatorFR<FramMemoryAccessBase>>(access);
		else if (access.type == "ArmFlashMemoryAccess") memory->memoryCreatorPtr = make_shared<MemoryCreator432<ArmFlashMemoryAccess>>(access);
		else if (access.type == "ArmRandomMemoryAccess") memory->memoryCreatorPtr = make_shared<MemoryCreator<ArmRandomMemoryAccess>>();
		else if (access.type == "ArmReadonlyMemroyAccess") memory->memoryCreatorPtr = make_shared<MemoryCreator<ArmReadonlyMemroyAccess>>();
		else if (access.type == "ArmBslFlashMemoryAccess") memory->memoryCreatorPtr = make_shared<MemoryCreator<ArmBslFlashMemoryAccess>>();
		else if (access.type == "ArmInfoFlashMemoryAccess") memory->memoryCreatorPtr = make_shared<MemoryCreator<ArmInfoFlashMemoryAccess>>();
		else if (access.type == "ArmFlashMemoryAccess2M") memory->memoryCreatorPtr = make_shared<MemoryCreator432<ArmFlashMemoryAccess2M>>(access);		
		else if (access.type == "ArmBslFlashMemoryAccess2M") memory->memoryCreatorPtr = make_shared<MemoryCreator<ArmBslFlashMemoryAccess2M>>();
		else if (access.type == "ArmInfoFlashMemoryAccess2M") memory->memoryCreatorPtr = make_shared<MemoryCreator<ArmInfoFlashMemoryAccess2M>>();
		else throw runtime_error("invalid memory access class: " + access.type);
	}
	else setMember(*memory, e);
}

void fromXml(MemoryLayout* memoryMap, const pugi::xml_node e)
{
	if (!strcmp(e.name(), "memory"))
	{
		const char* const memName = e.attribute("name").value();
		if (!memName[0])
			throw runtime_error("missing memory name");

		MemoryInfo memory;
		readElement(e, memory);
		fromString(memName, memory.name);

		(*memoryMap)[memName] = memory;
	}
}
