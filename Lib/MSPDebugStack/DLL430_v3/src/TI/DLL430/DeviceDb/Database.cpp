/*
 * Registration.cpp
 *
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

#include "Database.h"
#include "ziparchive.h"
#include "loadxml.h"
#include "exportxml.h"

using namespace TI::DLL430;
using namespace DeviceDb;


DeviceMap& Database::instance()
{
	static DeviceMap instance;
	return instance;
}


int32_t Database::getMaxId() const
{
	return (int32_t)instance().size() - 1;
}


void Database::dump(const char *filename) const
{
	exportXml(instance(), filename);
}

int32_t Database::findDevice(const IdCode& idCode)
{
	const DeviceMap& map = instance();

	const auto& it = find_if(map.cbegin(), map.cend(), [&idCode](const DeviceMap::value_type& m)
	{
		return m.first == idCode;
	});

	if(it != map.cend() )
	{
		return static_cast<int32_t>(std::distance<DeviceMap::const_iterator>(map.begin(), it));
	}
	return -1;
}

const DeviceInfo* Database::getDeviceInfo(size_t id) const
{
	if (id < instance().size())
	{
		auto it = instance().cbegin();
		std::advance(it, id);
		return &it->second;
	}
	return nullptr;
}

void Database::loadDevices(const std::string& file, bool usefile)
{
	Archive archive(file, usefile);
	XmlLoader(archive).load();
}

void Database::clearDevices()
{
	instance().clear();
}
