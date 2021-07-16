/*
 * SoftwareBreakpointManager.cpp
 *
 * Manages software breakpoints and replaced instruction opcodes.
 *
 * Copyright (C) 2007 - 2013 Texas Instruments Incorporated - http://www.ti.com/
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
#include "SoftwareBreakpointManager.h"
#include "../Exceptions/Exceptions.h"


using namespace TI::DLL430;

SoftwareBreakpointManager::ReadFunction SoftwareBreakpointManager::sRead;
SoftwareBreakpointManager::WriteFunction SoftwareBreakpointManager::sWrite;
SoftwareBreakpointManager::SyncFunction SoftwareBreakpointManager::sSync;

void SoftwareBreakpointManager::setMemoryAccessFunctions(SoftwareBreakpointManager::ReadFunction read,
                                                         SoftwareBreakpointManager::WriteFunction write,
                                                         SoftwareBreakpointManager::SyncFunction sync)
{
	sRead = read;
	sWrite = write;
	sSync = sync;
}


SoftwareBreakpointManager::SoftwareBreakpointManager(uint16_t triggerOpCode)
	: mTriggerOpCode(triggerOpCode)
{
}

void SoftwareBreakpointManager::setSoftwareTriggerAt(uint32_t address)
{
	if (!(sRead && sWrite && sSync))
		throw EM_MemoryAccessFunctionException();

	uint8_t instruction[2] = {0};
	if (!sRead(address, instruction, 2) || !sSync())
		throw EM_MemoryReadErrorException();

	const uint8_t opCode[2] = { static_cast<uint8_t>(mTriggerOpCode), static_cast<uint8_t>(mTriggerOpCode >> 8)};
	if (!sWrite(address, opCode, 2) || !sSync())
		throw EM_MemoryWriteErrorException();

	if (!verifyValueAt(address, mTriggerOpCode))
		throw EM_MemoryWriteErrorException();

	mInstructionTable[address] = (uint16_t)((instruction[0] & 0xFF) | (instruction[1] << 8));
}

void SoftwareBreakpointManager::removeSoftwareTriggerAt(uint32_t address)
{
	if (!(sRead && sWrite && sSync))
		throw EM_MemoryAccessFunctionException();

	uint8_t opCodeTmp[2] = {0};
	if (!sRead(address, opCodeTmp, 2) || !sSync())
		throw EM_MemoryReadErrorException();

	const uint16_t opCode = (uint16_t)((opCodeTmp[0] & 0xFF) + (opCodeTmp[1] << 8));

	if (opCode == mTriggerOpCode)
	{
		const uint16_t instruction = getInstructionAt(address);
		if (instruction == 0)
			throw EM_InstructionEntryMissingException();

		const uint8_t instrTmp[2] = {static_cast<uint8_t>(instruction), static_cast<uint8_t>(instruction >> 8)};
		if (!sWrite(address, instrTmp, 2) || !sSync())
			throw EM_MemoryWriteErrorException();

		if (!verifyValueAt(address, instruction))
			throw EM_MemoryWriteErrorException();
	}
	mInstructionTable.erase(address);
}


void SoftwareBreakpointManager::writeGroup(const std::vector<InstructionTable::iterator>& group)
{
	const uint32_t start = group.front()->first;
	const uint32_t end = group.back()->first + 2;

	std::vector<uint8_t> data(end - start);
	sRead(start, &data[0], data.size()) && sSync();

	for (const auto& entry : group)
	{
		data[entry->first - start] = entry->second & 0xFF;
		data[entry->first - start + 1] = entry->second >> 8;
	}

	sWrite(start, &data[0], data.size()) && sSync;
}

void SoftwareBreakpointManager::clearSoftwareTriggers()
{
	if (!(sRead && sWrite && sSync))
		throw EM_MemoryAccessFunctionException();

	std::vector<InstructionTable::iterator> group;
	for (InstructionTable::iterator it = mInstructionTable.begin(); it != mInstructionTable.end(); ++it)
	{
		//Read and verify it still exists
		uint8_t opCodeTmp[2] = {0};
		if (!sRead(it->first, opCodeTmp, 2) || !sSync())
			throw EM_MemoryReadErrorException();

		const uint16_t opCode = (uint16_t)((opCodeTmp[0] & 0xFF) + (opCodeTmp[1] << 8));

		if (opCode != mTriggerOpCode)
			continue;

		//Check if location is less than one segment away from previous location
		if (group.empty() || (it->first - group.back()->first <= 0x200))
		{
			group.push_back(it);
		}
		else
		{
			writeGroup(group);
			group.clear();
			group.push_back(it);
		}
	}
	if (!group.empty())
		writeGroup(group);

	mInstructionTable.clear();
}

uint32_t SoftwareBreakpointManager::numberOfActiveSoftwareTriggers() const
{
	return static_cast<uint32_t>(mInstructionTable.size());
}

const uint32_t opcodeSize = 2;

void SoftwareBreakpointManager::patchMemoryWrite(uint32_t address, uint8_t* data, size_t size)
{
	const uint32_t endAddress = address + static_cast<uint32_t>(size);

	InstructionTable::iterator it = mInstructionTable.begin();
	const InstructionTable::const_iterator end = mInstructionTable.end();

	while ((it != end) && (it->first + (opcodeSize-1) < address)) ++it;

	for (; (it != end) && (it->first < endAddress); ++it)
	{
		const uint32_t swbpAddress = it->first;
		const uint32_t overlapBegin = std::max(swbpAddress, address);
		const uint32_t overlapEnd = std::min(swbpAddress + opcodeSize, endAddress);
		const uint32_t dataOffset = (swbpAddress < address) ? 0 : swbpAddress - address;
		const uint32_t opcodeOffset = (swbpAddress < address) ? address - swbpAddress : 0;

		for (uint32_t i = 0; i < overlapEnd - overlapBegin; ++i)
		{
			const int shift = 8*(i + opcodeOffset);
			it->second &= ~(0xFF << shift);
			it->second |= data[i + dataOffset] << shift;

			data[i + dataOffset] = (mTriggerOpCode >> shift) & 0xFF;
		}
	}
}

void SoftwareBreakpointManager::patchMemoryRead(uint32_t address, uint8_t* data, size_t size)
{
	const uint32_t endAddress = address + static_cast<uint32_t>(size);

	InstructionTable::iterator it = mInstructionTable.begin();
	const InstructionTable::const_iterator end = mInstructionTable.end();

	while ((it != end) && (it->first + (opcodeSize-1) < address)) ++it;

	while ((it != end) && (it->first < endAddress))
	{
		InstructionTable::const_iterator tmp = it++;

		const uint32_t swbpAddress = tmp->first;
		const uint32_t overlapBegin = std::max(swbpAddress, address);
		const uint32_t overlapEnd = std::min(swbpAddress + opcodeSize, endAddress);
		const uint32_t dataOffset = (swbpAddress < address) ? 0 : swbpAddress - address;
		const uint32_t opcodeOffset = (swbpAddress < address) ? address - swbpAddress : 0;

		for (uint32_t i = 0; i < overlapEnd - overlapBegin; ++i)
		{
			const int shift = 8*(i + opcodeOffset);

			if (data[i + dataOffset] != ((mTriggerOpCode >> shift) & 0xFF))
			{
				mInstructionTable.erase(tmp->first);
				return; //swbp has been removed or modified
			}
		}

		for (uint32_t i = 0; i < overlapEnd - overlapBegin; ++i)
		{
			const int shift = 8*(i + opcodeOffset);
			data[i + dataOffset] = (tmp->second >> shift) & 0xFF;
		}
	}
}

uint16_t SoftwareBreakpointManager::getInstructionAt(uint32_t address) const
{
	InstructionTable::const_iterator it = mInstructionTable.find(address);
	return (it != mInstructionTable.end()) ? it->second : 0;
}

bool SoftwareBreakpointManager::verifyValueAt(uint32_t address, uint16_t expectedValue) const
{
	uint8_t actualValue[2] = { 0 };
	if (!sRead(address, actualValue, 2) || !sSync())
		throw EM_MemoryReadErrorException();

	return (actualValue[0] | (actualValue[1] << 8)) == expectedValue;
}

void SoftwareBreakpointManager::importInstructionTable(const SoftwareBreakpointManager& swbp)
{
	mInstructionTable.insert(swbp.mInstructionTable.begin(), swbp.mInstructionTable.end());
}
