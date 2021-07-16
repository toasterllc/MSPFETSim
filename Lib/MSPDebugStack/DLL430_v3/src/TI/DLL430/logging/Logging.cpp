/*
 * Logging.cpp
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

#include <pch.h>

#include <chrono>
#include <iomanip>

#include "Logging.h"

#ifdef DB_PRINT

//#define USE_STDOUT

using namespace TI::DLL430::Logging;

Debug& TI::DLL430::Logging::DefaultLogger()
{
	static Debug defaultLogger;
	return defaultLogger;
}

Debug::Debug()
	: filePtr(stdout)
{
#ifndef USE_STDOUT
	filePtr = fopen("comm.log", "w");
#endif

	if (!filePtr)
		fprintf(stderr, "File could not be opened\n!");
}

Debug::~Debug()
{
	if (filePtr)
		fclose(filePtr);
}

void Debug::PrintSendBuffer(const uint8_t *buf, long size)
{
	PrintBuffer(buf, size, "send:");
}

void Debug::PrintReceiveBuffer(const uint8_t *buf, long size)
{
	PrintBuffer(buf, size, "receive:");
}

void Debug::PrintBuffer(const uint8_t *buf, long size, const char *direction)
{
	if (filePtr)
	{
		const auto now = std::chrono::system_clock::now();
		const auto timeNow = std::chrono::system_clock::to_time_t(now);
		const auto lt = std::localtime(&timeNow);
		const auto usSinceEpoch = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();

		static const char* monthNames[12] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

		std::stringstream sstr;
		sstr << std::setfill('0') << "\n"
			<< (1900 + lt->tm_year) << '-' << monthNames[lt->tm_mon % 12] << '-' << std::setw(2) << lt->tm_mday << ' '
			<< std::setw(2) << lt->tm_hour << ':' << std::setw(2) << lt->tm_min << ':' << std::setw(2) << lt->tm_sec << '.'
			<< std::setw(6) << (usSinceEpoch % 1000000) << " "
			<< direction << " " << std::this_thread::get_id()
			<< std::endl << "[" << std::hex;

		for (long i = 0 ; i < size; ++i)
		{
			sstr << std::setw(2) << std::setfill('0') << static_cast<unsigned int>(buf[i]);
			if (i != size -1)
				sstr << "][";
		}
		sstr << "]" << std::endl << std::dec;
		fprintf(filePtr, "%s", sstr.str().c_str());
		fflush(filePtr);
	}
}

#endif
