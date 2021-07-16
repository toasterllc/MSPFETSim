/*
 * FileWriterTI.cpp
 *
 * File writer for TI txt files
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

#include "FileWriterTI.h"
#include "MemoryContent.h"
#include "EM/Exceptions/Exceptions.h"


using namespace TI::DLL430;
using namespace std;


FileWriterTI::FileWriterTI(const char* filename)
	: file(filename)
{
	if (!file)
		throw DLL430_FileOpenException();
}


void FileWriterTI::write(const MemoryContent& src)
{
	for (const DataSegment& seg : src.segments)
	{
		writeTiSegment(seg);
	}
	file << "q\n";
}


void FileWriterTI::writeTiSegment(const DataSegment& segment)
{
	file << '@' << setw(4) << hex << setfill ('0') << uppercase << segment.startAddress << '\n';

	if (!segment.data.empty())
	{
		size_t numBytes = 0;
		for (uint8_t byte : segment.data)
		{
			file << setw(2) << hex << setfill ('0') << uppercase << (uint32_t)byte;
			size_t numInLine = (++numBytes % 16);
			file << (numInLine == 0 || numBytes >= segment.data.size() ? '\n' : ' ');
		}
	}
}
