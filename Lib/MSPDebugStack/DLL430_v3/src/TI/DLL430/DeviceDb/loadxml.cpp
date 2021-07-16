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

#include <DeviceInfo.h>

#include "loadxml.h"
#include "Database.h"
#include "ziparchive.h"
#include "setmember.h"


using namespace std;
using namespace TI::DLL430;
using namespace DeviceDb;


void checkVersion(const pugi::xml_node root)
{
	static const int VERSION_MIN = 10, VERSION_MAX = 19;

	const string versionString = root.attribute("version").value();

	if (versionString.empty())
	{
		throw runtime_error("'device-information' missing version number");
	}

	const size_t len = versionString.length();
	if (len < 3 || versionString[len - 2] != '.')
	{
		throw runtime_error("malformed version number (expected 'x.y')");
	}

	const int version = 10 * stoi(versionString.substr(0, len - 2)) + stoi(versionString.substr(len - 1, 1));

	if (version < VERSION_MIN || version > VERSION_MAX)
	{
		throw runtime_error("unsupported device-information version '" + versionString +
			"' (must be " + to_string(VERSION_MIN / 10) + '.' + to_string(VERSION_MIN % 10) +
			" - " + to_string(VERSION_MAX / 10) + '.' + to_string(VERSION_MAX % 10) + ")");
	}
}


void readXmlDocument(const pugi::xml_document& doc)
{
	pugi::xml_node root = doc.child("device-information");

	if (root.empty())
	{
		throw runtime_error("wrong root element (expected 'device-information')");
	}

	checkVersion(root);

	DeviceInfo dummyDevice;
	MemoryInfo dummyMemory;

	for (pugi::xml_node element : doc.first_child().children())
	{
		const string name = element.name();

		if (name == "eemTimer")
		{
			readElement(element, dummyDevice.clockInfo.eemTimers[0]);
		}
		else if (name == "eemTimers")
		{
			readElement(element, dummyDevice.clockInfo.eemTimers);
		}
		else if (name == "eemClocks")
		{
			readElement(element, dummyDevice.clockInfo.eemClocks);
		}
		else if (name == "memory")
		{
			readElement(element, dummyMemory);
		}
		else if (name == "device")
		{
			DeviceInfo device;
			readElement(element, device);
			if (device.idCode.version && !device.description.empty())
			{
				device.clockInfo.mclkCntrl0 = 0;
				for (int i = 0; i < 32; ++i)
				{
					device.clockInfo.mclkCntrl0 |= device.clockInfo.eemTimers[i].defaultStop << (31 - i);
				}

				device.clockInfo.mclkCntrl0 = (device.clockInfo.mclkCntrl0 & 0xFFFF0000) >> 16 | ((device.clockInfo.mclkCntrl0 & 0xFFFF) << 16);

				if (!Database::instance().insert(DeviceMap::value_type(Match({ device.idMask, device.idCode }), device)).second)
				{
					throw runtime_error("device match already exists");
				}
			}
		}
		else
		{
			setMember(dummyDevice, element);
		}
	}
}


void XmlLoader::loadImports(const pugi::xml_document& doc)
{
	for (pugi::xml_node imp : doc.first_child().children("import"))
	{
		if (const pugi::xml_attribute file = imp.attribute("file"))
		{
			loadFile(file.value());
		}
	}
}


void XmlLoader::load()
{
	vector<string> files;
	archive.getFileList(&files);

	for (const auto& filename : files)
	{
		loadFile(filename);
	}
}


void XmlLoader::loadFile(const string& file)
{
	FileState& state = fileStates[file];

	try
	{
		if (state == LOADING)
			throw runtime_error("cyclic imports detected");

		if (state == NONE)
		{
			vector<char> data;
			archive.readFile(file, &data);

			pugi::xml_document doc;
			const pugi::xml_parse_result result = doc.load_buffer_inplace(data.data(), data.size());

			if (result.status != pugi::status_ok)
			{
				throw runtime_error(result.description());
			}

			state = LOADING;

			loadImports(doc);

			readXmlDocument(doc);

			state = LOADED;
		}
	}
	catch (const runtime_error& e)
	{
		throw runtime_error("In file '" + file + "':\n" + e.what());
	}
}


XmlLoader::~XmlLoader()
{
	ElementTableBase::clearTables();
}
