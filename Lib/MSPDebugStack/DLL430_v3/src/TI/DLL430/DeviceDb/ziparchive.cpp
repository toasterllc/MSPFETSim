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
#include <tinfl.c>
#include "ziparchive.h"
#include "devicedb.h"

using namespace std;
using namespace TI::DLL430;
using namespace DeviceDb;


#pragma pack(push, 1)
	struct LocalFileHeader
	{
		uint32_t signature;
		uint16_t version;
		uint16_t bitFlag;
		uint16_t compression;
		uint16_t modificationTime;
		uint16_t modificationDate;
		uint32_t crc;
		uint32_t compressedSize;
		uint32_t uncompressedSize;
		uint16_t fileNameLength;
		uint16_t extraFieldLength;
	};

	struct DirectoryFileHeader
	{
		uint32_t signature;
		uint16_t versionMadeBy;
		uint16_t versionNeeded;
		uint16_t bitFlag;
		uint16_t compression;
		uint16_t modificationTime;
		uint16_t modificationDate;
		uint32_t crc;
		uint32_t compressedSize;
		uint32_t uncompressedSize;
		uint16_t fileNameLength;
		uint16_t extraFieldLength;
		uint16_t fileCommentLength;
		uint16_t diskNumberStart;
		uint16_t internalFileAttributes;
		uint32_t externalFileAttributes;
		uint32_t offset;
	};

	struct EndOfDirectory
	{
		uint32_t signature;
		uint16_t diskNumber;
		uint16_t directoryStartDisk;
		uint16_t entriesOnDisk;
		uint16_t totalEntries;
		uint32_t directorySize;
		uint32_t directoryOffset;
		uint16_t commentLength;
	};
#pragma pack(pop)


void Archive::open(const string& filename, bool usefile)
{
	useFile = usefile;

	EndOfDirectory endOfDir;	
	uint8_t * pzip = &g_database[0];

	if (useFile)
	{
		file.open(filename, ios::binary | ios::in);
		if (!file)
		{
			throw runtime_error("failed to open archive");
		}
		file.seekg(-(int)sizeof(endOfDir), ios::end);
		file.read((char*)&endOfDir, sizeof(endOfDir));
	}
	else
	{
		memcpy(&endOfDir, &TI::DLL430::DeviceDb::g_database[sizeof(TI::DLL430::DeviceDb::g_database) - sizeof(endOfDir)], sizeof(endOfDir));
	}

	if (endOfDir.signature != 0x06054b50)
	{
		throw runtime_error("no file comment allowed");
	}

	if (useFile)
	{
		file.seekg(endOfDir.directoryOffset, ios::beg);
	}
	else
	{
		pzip = pzip + endOfDir.directoryOffset;
	}

	DirectoryFileHeader header;

	const uint16_t supportedCompressions[] = {0, 8, 9};

	for (int i = 0; i < endOfDir.totalEntries; ++i)
	{
		if (useFile)
		{
			file.read((char*)&header, sizeof(header));
		}
		else
		{
			memcpy(&header, pzip, sizeof(header));
			pzip = pzip + sizeof(header);
		}

		const uint16_t* endComp = end(supportedCompressions);
		if (find(begin(supportedCompressions), endComp, header.compression) == endComp)
			throw runtime_error("compression type not supported (only deflate/deflate64)");

		if (header.bitFlag & 0xfff9)
			throw runtime_error("unsupported option (encrypted?)");

		vector<char> name(header.fileNameLength);
		if (useFile)
		{
			file.read(name.data(), name.size());
		}
		else
		{
			memcpy(name.data(), pzip, name.size());
			pzip = pzip + name.size();
		}
		if (header.compressedSize > 0)
		{
			string n(name.data(), name.size());
			size_t pos = n.find_last_of("/") + 1;
			directory[n.substr(pos)] = header.offset;
		}
		if (useFile)
		{
			file.seekg(header.extraFieldLength + header.fileCommentLength, ios::cur);
		}
		else
		{
			pzip = pzip + header.extraFieldLength + header.fileCommentLength;
		}	
	}
}

void Archive::readFile(const string& filename, vector<char>* dst) const
{
	const auto it = directory.find(filename);
	if (it == directory.end())
		throw runtime_error("file does not exist");

	const uint32_t offset = it->second;
	uint8_t * pzip = &g_database[0];
	if (useFile)
	{
		file.seekg(offset, ios::beg);
	}
	else
	{
		pzip = pzip + offset;
	}

	LocalFileHeader header;
	if (useFile)
	{
		file.read((char*)&header, sizeof(header));
	}
	else
	{
		memcpy(&header, pzip, sizeof(header));
		pzip = pzip + sizeof(header);
	}

	if (useFile)
	{
		file.seekg(header.fileNameLength + header.extraFieldLength, ios::cur);
	}
	else
	{
		dst->resize(header.uncompressedSize);
		pzip = pzip + header.fileNameLength + header.extraFieldLength;
	}

	if (header.compression != 0)
	{
		vector<unsigned char> compressedData(header.compressedSize);
		if (useFile)
		{
			file.read((char*)compressedData.data(), compressedData.size());
		}
		else
		{
			memcpy(compressedData.data(), pzip, compressedData.size());
		}
		size_t uncompressed = tinfl_decompress_mem_to_mem(dst->data(), dst->size(), compressedData.data(), compressedData.size(), 0);
		if (uncompressed != header.uncompressedSize)
		{
			throw runtime_error("error decompressing data");
		}
	}
	else
	{
		if (useFile)
		{
			file.read(dst->data(), dst->size());
		}
		else
		{
			memcpy(dst->data(), pzip, dst->size());
			pzip = pzip + dst->size();
		}
	}
}


void Archive::getFileList(vector<string>* list) const
{
	for (const auto& it : directory)
	{
		if (it.first.rfind(".xml") != string::npos)
			list->push_back(it.first);
	}
}
