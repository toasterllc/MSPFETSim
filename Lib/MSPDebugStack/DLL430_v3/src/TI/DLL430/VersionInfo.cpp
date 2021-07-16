/*
 * VersionInfo.cpp
 *
 * Formats version string.
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

#include "VersionInfo.h"
#include "../../../version.h"

using namespace TI::DLL430;
using namespace std;

#define TO_STRING(x) #x
#define MAKE_VERSION_STR(major, minor, patch, build) TO_STRING(major) "." TO_STRING(minor) "." TO_STRING(patch) "." TO_STRING(build)
#define MAKE_FN_NAME(major, minor, patch, build) _MSPDS_FUNCTION_VERSION_##major##_##minor##_##patch##_##build(void)
#define MAKE_FUNCTION_NAME(major, minor, patch, build) MAKE_FN_NAME(major, minor, patch, build)

static volatile char gVersion[]
#ifdef __APPLE__
	__attribute__((used))
#endif
	= "_MSPDS_VERSION_ " MAKE_VERSION_STR(VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, VERSION_BUILD);

#if defined(__cplusplus)
extern "C" {
#endif

DLL430_SYMBOL volatile char*
#if !defined(_WIN32) && !defined(_WIN64)
__attribute__((used))
#else
#endif
MAKE_FUNCTION_NAME(VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, VERSION_BUILD)
{
	return gVersion;
}

#if defined(__cplusplus)
}
#endif

VersionInfo::VersionInfo (uint8_t major, uint8_t minor, uint8_t patch, uint16_t flavor)
 : imajor(min(major, (uint8_t)9))
 , iminor(min(minor, (uint8_t)99))
 , patch(min(patch, (uint8_t)99))
 , flavor(min(flavor, (uint16_t)999))
{
}

VersionInfo::~VersionInfo ()
{
}

uint32_t VersionInfo::get () const
{
	/* a.bb.cc.ddd */
	return (this->imajor * 10000000) + (this->iminor * 100000) + (this->patch * 1000) + this->flavor;
}
