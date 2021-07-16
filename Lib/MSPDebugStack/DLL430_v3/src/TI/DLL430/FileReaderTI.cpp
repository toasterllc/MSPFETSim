/*
 * FileReaderTI.cpp
 *
 * File reader for TI txt files
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

#include "FileReaderTI.h"
#include "MemoryContent.h"
#include "EM/Exceptions/Exceptions.h"


using namespace TI::DLL430;
using namespace std;


bool FileReaderTI::fileIsTiTxt(const char* filename)
{
	string firstWord;
	ifstream(filename) >> firstWord;

	return !firstWord.empty() && (firstWord[0] == '@');
}


FileReaderTI::FileReaderTI(const char* filename) : filename(filename) {}


void FileReaderTI::read(MemoryContent* data)
{
	ifstream file(filename);
	if (!file.is_open())
		throw DLL430_FileOpenException();

	DataSegment segment;

	bool done = false;
	string line;
	while (getline(file, line))
	{
		// trim leading whitespace
		const string whitespaces(" \t\n\r");
		line.erase(0, line.find_first_not_of(whitespaces));

		if (!line.empty())
		{
			if (done)
				throw DLL430_Exception(FILE_END_ERR, "Content after EOF marker");

			done = (line[0] == 'q' || line[0] == 'Q');

			if (done || line[0] == '@')
			{
				if (!segment.data.empty())
				{
					data->segments.push_back(segment);
				}

				if (line[0] == '@')
				{
					segment = DataSegment(getTiFileAddress(line));
				}
			}
			else
			{
				getTiFileBytes(line, &segment.data);
			}
		}
	}

	if (!done)
		throw DLL430_Exception(FILE_END_ERR, "No EOF marker");
}


uint32_t FileReaderTI::getTiFileAddress(const string& line)
{
	stringstream stream(line.substr(1));

	uint32_t address = 0;
	stream >> hex >> address;
	if (stream.fail() || address > 0xFFFFFFFF)
		throw DLL430_Exception(FILE_DATA_ERR, "Invalid address");

	return address;
}


void FileReaderTI::getTiFileBytes(const string& line, vector<uint8_t>* buffer)
{
	stringstream stream(line);
	int numBytes = 0;

	uint32_t tmp = 0;
	while (stream >> hex >> tmp)
	{
		const uint8_t byte = (tmp & 0xFF);

		if (++numBytes > 16)
			throw DLL430_Exception(FILE_DATA_ERR, "Over 16 bytes in line");

		buffer->push_back(byte);
	}

	if (stream.fail() && !stream.eof())
		throw DLL430_Exception(FILE_DATA_ERR, "Invalid data");
}
