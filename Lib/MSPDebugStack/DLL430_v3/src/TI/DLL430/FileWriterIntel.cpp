/*
 * FileWriterIntel.cpp
 *
 * File writer for Intel hex files
 *
 * Copyright (C) 2008 - 2014 Texas Instruments Incorporated - http://www.ti.com/
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

#include "FileWriterIntel.h"
#include "MemoryContent.h"
#include "EM/Exceptions/Exceptions.h"


using namespace TI::DLL430;
using namespace std;


FileWriterIntel::FileWriterIntel(const char* filename)
	: file(filename)
{
	if (!file)
		throw DLL430_FileOpenException();
}


void FileWriterIntel::write(const MemoryContent& src)
{
	for (const DataSegment& seg : src.segments)
	{
		writeSegment(seg);
	}
	writeRecord(0, 0, 1, 0);
}


void FileWriterIntel::writeSegment(const DataSegment& segment)
{
	static const size_t INTEL_HEX_LINE_DATA_LEN = 0x10;

	uint32_t addressOffset = 0;

	uint32_t address32 = segment.startAddress;
	size_t size = segment.data.size();
	size_t offset = 0;

	while (size > 0)
	{
		if (address32 - addressOffset > 0xFFFFF) //32bit offset
		{
			addressOffset = (address32 & 0xFFFF0000);
			uint8_t offset[2] = { static_cast<uint8_t>(addressOffset >> 24), static_cast<uint8_t>(addressOffset >> 16) };
			writeRecord(2, 0, 4, offset);
		}
		else if (address32 - addressOffset > 0xFFFF) //20bit offset
		{
			addressOffset = (address32 & 0xFFFF0);
			uint8_t offset[2] = { static_cast<uint8_t>(addressOffset >> 12), static_cast<uint8_t>(addressOffset >> 4) };
			writeRecord(2, 0, 2, offset);
		}

		const uint16_t address16 = static_cast<uint16_t>(address32 - addressOffset);

		uint8_t toWrite = static_cast<uint8_t>( min(size, INTEL_HEX_LINE_DATA_LEN) );

		const size_t align = address16 % INTEL_HEX_LINE_DATA_LEN;
		if (align > 0)
			toWrite = static_cast<uint8_t>( min(size, INTEL_HEX_LINE_DATA_LEN - align) );

		writeRecord(toWrite, address16, 0, &segment.data[offset]);

		address32 += toWrite;
		offset += toWrite;
		size -= toWrite;
	}
}


void FileWriterIntel::writeRecord(uint8_t size, uint16_t address, uint8_t type, const uint8_t* data)
{
	file << hex << setfill('0') << uppercase;
	file << ':' << setw(2) << (uint32_t)size << setw(4) << address << setw(2) << (int)type;

	uint8_t crc = size + (address & 0xFF) + (address >> 8) + type;

	if (data)
	{
		for (uint32_t i = 0; i < size; ++i)
		{
			file << setw(2) << (uint32_t)data[i];
			crc += data[i];
		}
	}
	crc = ~crc + 1;

	file << setw(2) << (uint32_t)crc << '\n';
}
