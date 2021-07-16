/*
 * FileReaderIntel.cpp
 *
 * File reader for Intel hex files
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

#include "FileReaderIntel.h"
#include "MemoryContent.h"
#include "EM/Exceptions/Exceptions.h"


using namespace TI::DLL430;
using namespace std;


bool FileReaderIntel::fileIsIntelHex(const char* filename)
{
	string firstWord;
	ifstream(filename) >> firstWord;

	return !firstWord.empty() && (firstWord[0] == ':');
}


FileReaderIntel::FileReaderIntel(const char* filename) : filename(filename) {}

template<typename T>
T readHexFromStream(istream& stream, T* value)
{
	const int numCharacters = 2 * sizeof(T);
	char byteString[numCharacters];
	stream.read(byteString, numCharacters);

	if (stream.fail())
		throw DLL430_Exception(INTEL_HEX_CODE_ERR, "Incomplete value in data record");

	stringstream conv( string(byteString, numCharacters) );
	uint64_t tmp = 0;
	conv >> hex >> tmp;

	if (conv.fail() || !conv.eof())
		throw DLL430_Exception(INTEL_HEX_CODE_ERR, "Error in data record");

	return *value = static_cast<T>(tmp);
}

FileReaderIntel::IntelHeader FileReaderIntel::readIntelHeader(istream& stream)
{
	IntelHeader header;
	readHexFromStream(stream, &header.size);
	readHexFromStream(stream, &header.address);
	readHexFromStream(stream, &header.type);
	return header;
}

bool FileReaderIntel::checkIntelCRC(const string& line, uint8_t crc)
{
	uint8_t actualCRC = 0;
	const int bytesInLine = static_cast<int>(line.size() / 2 - 1);

	stringstream crcStream(line);

	for (int i = 0; i < bytesInLine; ++i)
	{
		uint8_t byte = readHexFromStream(crcStream, &byte);
		actualCRC += byte;
	}

	actualCRC = ~actualCRC + 1;
	return (actualCRC == crc);
}

void FileReaderIntel::readIntelData(istream& stream, uint8_t size, DataSegment* segment)
{
	for (int i = 0; i < size; ++i)
	{
		uint8_t byte = readHexFromStream(stream, &byte);
		segment->data.push_back(byte);
	}
}

void FileReaderIntel::read(MemoryContent* data)
{
	//Size Address Type Data CRC
	//:10 0100 00 214601360121470136007EFE09D21901 40
	ifstream file(filename);
	if (!file.is_open())
		throw DLL430_FileOpenException();

	DataSegment segment;
	uint32_t addressOffset = 0;
	bool done = false;
	string line;

	while (getline(file, line))
	{
		if (line.empty() || line[0] != ':')
			throw DLL430_Exception(INTEL_HEX_CODE_ERR, "Line not starting with ':'");

		if (done)
			throw DLL430_Exception(FILE_END_ERR, "Content after EOF marker");

		line.erase(0, 1);

		uint16_t offset = 0;

		stringstream stream(line);

		IntelHeader header = readIntelHeader(stream);

		switch (header.type)
		{
		case 0: //Regular data
			if ((addressOffset + header.address) != (segment.startAddress + segment.data.size()))
			{
				if (!segment.data.empty())
				{
					data->segments.push_back(segment);
				}
				segment = DataSegment(addressOffset + header.address);
			}
			readIntelData(stream, header.size, &segment);
			break;

		case 1: //EOF record
			done = true;
			if (!segment.data.empty())
			{
				data->segments.push_back(segment);
			}
			break;

		case 2: //Extended address (20bit)
			readHexFromStream(stream, &offset);
			addressOffset = (uint32_t)offset << 4;
			break;

		case 3: //Only for x86 architecture, skip line
			continue;

		case 4: //Extended address (32bit)
			readHexFromStream(stream, &offset);
			addressOffset = (uint32_t)offset << 16;
			break;

		case 5: //Only for 386 architecture or higher, skip line
			continue;

		default: //Unknown record type, be generous and ignore
			continue;
		}

		uint8_t crc = 0;
		readHexFromStream(stream, &crc);

		if (!checkIntelCRC(line, crc))
			throw DLL430_Exception(INTEL_HEX_CODE_ERR, "CRC error");
	}
	if (!done)
		throw DLL430_Exception(FILE_END_ERR, "No EOF record");
}
