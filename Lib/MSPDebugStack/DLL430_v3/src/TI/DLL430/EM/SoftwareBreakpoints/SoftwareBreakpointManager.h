/*
 * SoftwareBreakpointManager.h
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

#pragma once

namespace TI { namespace DLL430 {

class SoftwareBreakpointManager
{
public:
	typedef std::function<bool (uint32_t, uint8_t*, size_t)> ReadFunction;
	typedef std::function<bool (uint32_t, const uint8_t*, size_t)> WriteFunction;
	typedef std::function<bool ()> SyncFunction;

	static void setMemoryAccessFunctions(ReadFunction read, WriteFunction write, SyncFunction sync);

	explicit SoftwareBreakpointManager(uint16_t triggerOpCode);

	~SoftwareBreakpointManager() {}

	void setSoftwareTriggerAt(uint32_t address);

	void removeSoftwareTriggerAt(uint32_t address);

	void clearSoftwareTriggers();

	uint32_t numberOfActiveSoftwareTriggers() const;

	void patchMemoryWrite(uint32_t address, uint8_t* data, size_t size);

	void patchMemoryRead(uint32_t address, uint8_t* data, size_t size);

	uint16_t getInstructionAt(uint32_t address) const;

	void importInstructionTable(const SoftwareBreakpointManager& swbp);

private:
	typedef std::map<uint32_t, uint16_t> InstructionTable;

	void writeGroup(const std::vector<InstructionTable::iterator>& group);
	bool verifyValueAt(uint32_t address, uint16_t expectedValue) const;

	InstructionTable mInstructionTable;
	uint16_t mTriggerOpCode;

	static ReadFunction sRead;
	static WriteFunction sWrite;
	static SyncFunction sSync;
};

typedef std::shared_ptr<SoftwareBreakpointManager> SoftwareBreakpointManagerPtr;

}}
