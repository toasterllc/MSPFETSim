/*
 * UpdateManagerMSP_FET430.cpp
 *
 * Functionality for updating MSP-FET430UIF debugger
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
#include <VersionInfo.h>

#include "FetHandle.h"
#include "FetHandleManager.h"
#include "ConfigManager.h"
#include "UpdateManagerMSP_FET430.h"
#include "HalExecCommand.h"
#include "FetControl.h"
#include "WatchdogControl.h"
#include "Record.h"
#include "DeviceHandleMSP430.h"
#include "MemoryContent.h"
#include "FileReader.h"

#include "../../../../Bios/include/UifHal.h"
#include "../../../../Bios/include/UifHalMsp430.h"
#include "../../../../Bios/include/UifBiosCore.h"
#include "../../../../Bios/include/ConfigureParameters.h"

using namespace TI::DLL430;
using namespace std;

const static uint32_t coreSignature = 0xFEDF2112;


UpdateManagerMSP_FET430::UpdateManagerMSP_FET430(FetHandle* fetHandle, ConfigManager* configManagerV3, TARGET_ARCHITECTURE_t arch)
	: fetHandle(fetHandle), configManagerV3(configManagerV3), currentArch(arch), percent(0), requiredUpdates(0)
{
	updateCmd.setTimeout(20000);
}

UpdateManagerMSP_FET430::~UpdateManagerMSP_FET430(){}


bool UpdateManagerMSP_FET430::isUpdateRequired(TARGET_ARCHITECTURE_t arch) const
{
	uint16_t expectedVersionmMajor = 0, expectedVersionmMinor =0;
	Record fetHalImage(UifHalImage, UifHalImage_address, UifHalImage_length_of_sections, UifHalImage_sections);

	fetHalImage.getWordAtAdr(0x253C, &expectedVersionmMajor);
	fetHalImage.getWordAtAdr(0x253E, &expectedVersionmMinor);

	std::vector<uint8_t> sw_info;

	sw_info.push_back(expectedVersionmMajor&0xFF);
	sw_info.push_back(expectedVersionmMajor>>8);
	sw_info.push_back(expectedVersionmMinor&0xFF);
	sw_info.push_back(expectedVersionmMinor>>8);

	unsigned char major=sw_info.at(1);
	 VersionInfo exptectedHallVersion((((major&0xC0)>>6)+1), (major&0x3f), sw_info.at(0),
		(sw_info.at(3)<<8)+sw_info.at(2));

 	bool isUpdateRequired = false;
	if (exptectedHallVersion.get() != (getHalVersion().get()))
	{
		isUpdateRequired = true;
	}
	if (checkCoreVersion() != 0)
	{
		isUpdateRequired = true;
	}
	//This is only a temporarry workaround to pervent the UIF to be updated for MSP432 support 
	if (TARGET_ARCHITECTURE_t::MSP430 != fetHandle->getControl()->getFetFpgaVersion())
	{
		isUpdateRequired = true;
	}
	return isUpdateRequired;
}


uint16_t UpdateManagerMSP_FET430::checkCoreVersion() const
{
	FetControl * control=this->fetHandle->getControl();

	//get current core version from FET
	const uint16_t actualFetCoreVersion = control->getFetCoreVersion();
	uint16_t expectedFetCoreVersion = 0;

	Record fetCoreImage(UifBiosCoreImage, UifBiosCoreImage_address, UifBiosCoreImage_length_of_sections, UifBiosCoreImage_sections);
	//get core version from image (core version is stored in address 0x4400)
	if (fetCoreImage.getWordAtAdr(0xFDD8, &expectedFetCoreVersion))
	{
		//if core versions do not match, update core
		if (expectedFetCoreVersion != actualFetCoreVersion)
		{
			return 1;
		}
	}
	return 0;
}

VersionInfo UpdateManagerMSP_FET430::getHalVersion() const
{
	const std::vector<uint8_t>* sw_info = this->fetHandle->getSwVersion();
	if (sw_info == nullptr)
	{
		return VersionInfo(0, 0, 0, 0);
	}

	if (sw_info->size() < 4)
	{
		return VersionInfo(0, 0, 0, 0);
	}

	unsigned char major = sw_info->at(1);
	return VersionInfo((((major & 0xC0) >> 6) + 1), (major & 0x3f), sw_info->at(0),
		(sw_info->at(3) << 8) + sw_info->at(2));
}


bool UpdateManagerMSP_FET430::firmWareUpdate(const char* fname, UpdateNotifyCallback callback, bool*)
{
	FetControl* control=this->fetHandle->getControl();

	if (control == nullptr)
		return false;

	const uint32_t biosVersion = getHalVersion().get();

	// core/HAL communication has changed with version 3.2.0.8
	// and after HAL update the core/HAL communication would not work
	// do not update as long as there is no direct core update
	if ((biosVersion > 30200000) && (biosVersion < 30200008) && (fname == nullptr))
	{
		return false;
	}

	// core will be updated through internal image
	if (checkCoreVersion() != 0)
	{
		requiredUpdates = 12;
		percent = 100 / requiredUpdates;
		if (callback)
		{
			callback(BL_INIT, 100 - (requiredUpdates * percent), 0);
		}
		if (!this->upInit(1))
		{
			return false;
		}
		if (callback)
		{
			--requiredUpdates;
			callback(BL_ERASE_FIRMWARE, 100 - (requiredUpdates * percent), 0);
		}
		if (!this->upCoreErase())
		{
			return false;
		}
		if (callback)
		{
			--requiredUpdates;
			callback(BL_PROGRAM_FIRMWARE, 100 - (requiredUpdates * percent), 0);
		}

		if (!this->upCoreWrite())
		{
			return false;
		}
		if (callback)
		{
			--requiredUpdates;
			callback(BL_PROGRAM_FIRMWARE, 100 - (requiredUpdates * percent), 0);
		}

		if (!this->upCoreRead())
		{
			return false;
		}
		if (callback)
		{
			--requiredUpdates;
			callback(BL_PROGRAM_FIRMWARE, 100 - (requiredUpdates * percent), 0);
		}

		// create 0x55 command erase signature
		// command forces safecore to search for new core on reset
		std::vector<uint8_t> data_55;
		data_55.push_back(0x03);
		data_55.push_back(0x55);
		uint8_t id=control->createResponseId();
		data_55.push_back(id);
		data_55.push_back(0x00);

		control->sendData(data_55);
		control->clearResponse();

		for (int i = 0; i < 8; ++i)
		{
			this_thread::sleep_for(chrono::seconds(1));
			if (callback)
			{
				--requiredUpdates;
				callback(BL_PROGRAM_FIRMWARE, 100 - (requiredUpdates * percent), 0);
			}
		}
		if (callback)
		{
			callback(BL_EXIT, 100, 0);
		}
	}

	MemoryContent firmware;

	if (fname)
	{
		//Do not allow using a file if there is no valid HAL on the UIF
		if (biosVersion < 20000000)
		{
			return false;
		}
		FileReader::create(fname)->read(&firmware);
	}
	else
	{
		/*if (currentArch == MSP432_M4)
		{
			firmware.fromSRec(UifHalImage, UifHalImage_address, UifHalImage_length_of_sections, UifHalImage_sections);
		}
		else
		{*/
			firmware.fromSRec(UifHalMsp430Image, UifHalMsp430Image_address, UifHalMsp430Image_length_of_sections, UifHalMsp430Image_sections);
		//}		
	}
	requiredUpdates = UifHalMsp430Image_sections * 2 + 5;
	percent = 100 / requiredUpdates;
	// start HAL update routine
	if (callback)
	{
		callback(BL_INIT, 100 - (requiredUpdates * percent), 0);
	}
	if (!this->upInit(1))
	{
		return false;
	}
	if (callback)
	{
		--requiredUpdates;
		callback(BL_ERASE_FIRMWARE, 100 - (requiredUpdates * percent), 0);
	}
	if (!this->upErase(firmware, callback))
	{
		return false;
	}
	if (callback)
	{
		callback(BL_PROGRAM_FIRMWARE, 100 - (requiredUpdates * percent), 0);
	}
	if (!this->upWrite(firmware, callback))
	{
		return false;
	}
	if (callback)
	{
		callback(BL_EXIT, 100 - (requiredUpdates * percent), 0);
	}
	if (!this->upInit(0))
	{
		return false;
	}
	if (callback)
	{
		--requiredUpdates;
		callback(BL_EXIT, 100 - (requiredUpdates * percent), 0);
	}

	fetHandle->getControl()->resetCommunication();

	// activate the new HAL (in case of downgrade: change to VCP)
	HalExecCommand initCmd;
	HalExecElement* el = new HalExecElement(ID_Init);
	initCmd.elements.emplace_back(el);

	const bool initFailed = !this->fetHandle->send(initCmd);

	// give the firmware time to execute initialisation
	this_thread::sleep_for(chrono::seconds(1));
	if (callback)
	{
		--requiredUpdates;
		callback(BL_EXIT, 100 - (requiredUpdates * percent), 0);
	}

	//Only an error if no user specified file is used (ie. init will fail for a downgrade)
	if (initFailed && !fname)
	{
		return false;
	}

	if (!initFailed)
	{
		//Perform a reset to make sure everything is correctly initialized
		HalExecElement* el = new HalExecElement(ID_Zero);
		el->appendInputData8(STREAM_CORE_ZERO_PUC_RESET);

		HalExecCommand cmd;
		cmd.elements.emplace_back(el);

		fetHandle->send(cmd);
		for (int i = 0; i < 2; ++i)
		{
			this_thread::sleep_for(chrono::seconds(1));
			if (callback)
			{
				--requiredUpdates;
				callback(BL_EXIT, 100 - (requiredUpdates * percent), 0);
			}
		}

		if (callback)
		{
			callback(BL_UPDATE_DONE, 100, 0);
		}
	}

	return true;
}

bool UpdateManagerMSP_FET430::upInit(unsigned char level)
{
	HalExecElement* el = new HalExecElement(ID_Zero, UpInit);
	el->setAddrFlag(false);
	el->appendInputData8(level);

	HalExecCommand cmd;
	cmd.elements.emplace_back(el);

	return this->fetHandle->send(cmd);
}

bool UpdateManagerMSP_FET430::upErase(const MemoryContent& firmware, UpdateNotifyCallback callback)
{
	for (size_t i = 0; i < firmware.segments.size(); ++i)
	{
		const DataSegment& seg = firmware.segments[i];
		HalExecElement* el = new HalExecElement(ID_Zero, UpErase);
		el->setAddrFlag(false);
		el->appendInputData32(seg.startAddress & 0xfffffffe);
		el->appendInputData32(static_cast<uint32_t>(seg.data.size()));

		updateCmd.elements.clear();
		updateCmd.elements.emplace_back(el);
		if (!this->fetHandle->send(updateCmd))
		{
			return false;
		}
		if (callback)
		{
			--requiredUpdates;
			callback(BL_ERASE_FIRMWARE, 100 - (requiredUpdates * percent), 0);
		}
	}
	return true;
}

bool UpdateManagerMSP_FET430::upWrite(const MemoryContent& firmware, UpdateNotifyCallback callback)
{
	for (size_t i = 0; i < firmware.segments.size(); ++i)
	{
		const DataSegment& seg = firmware.segments[i];

		// create Core telegram -> update write
		HalExecElement* el = new HalExecElement(ID_Zero, UpWrite);
		// no HAL id needed for update
		el->setAddrFlag(false);

		const uint32_t padding = seg.data.size() % 2;
		const uint32_t data2send = static_cast<uint32_t>(seg.data.size()) + padding;

		// add address data
		el->appendInputData32(seg.startAddress&0xfffffffe);
		el->appendInputData32(data2send);

		// add update data
		for (size_t n=0; n < seg.data.size(); n++)
			el->appendInputData8(seg.data[n] & 0xff);

		for (uint32_t p=0 ; p < padding; p++)
			el->appendInputData8(0xff);

		updateCmd.elements.clear();
		updateCmd.elements.emplace_back(el);
		if (!this->fetHandle->send(updateCmd))
		{
			return false;
		}

		if (callback)
		{
			--requiredUpdates;
			callback(BL_DATA_BLOCK_PROGRAMMED, 100 - (requiredUpdates * percent), 0);
		}
	}

	return true;
}


bool UpdateManagerMSP_FET430::upCoreErase()
{
	//create erase command
	updateCmd.elements.clear();
	HalExecElement* el = new HalExecElement(ID_Zero, UpErase);
	el->setAddrFlag(false);
	//append start address and length in bytes
	el->appendInputData32(0x2500);
	el->appendInputData32(0xbb00);

	updateCmd.elements.emplace_back(el);
	return this->fetHandle->send(updateCmd);
}

bool UpdateManagerMSP_FET430::upCoreWrite()
{
	Record fetCoreImage(UifBiosCoreImage, UifBiosCoreImage_address, UifBiosCoreImage_length_of_sections, UifBiosCoreImage_sections);

	updateCmd.elements.clear();
	HalExecElement* el = new HalExecElement(ID_Zero, UpWrite);
	el->setAddrFlag(false);
	//append flash start-address where following data will be flashed to
	el->appendInputData32(0x2500);
	//append number of data bytes, including number of sections, section start addresses and core signature
	el->appendInputData32((fetCoreImage.getNumOfAllDataWords() + fetCoreImage.getNumOfManageWords() + 2)*2);
	//append core signature
	el->appendInputData32(coreSignature);
	//append number of sections
	el->appendInputData16((uint16_t)fetCoreImage.getSectionCount());

	while (fetCoreImage.hasNextSection())
	{
		//append start address and length information of each section
		el->appendInputData16((uint16_t)fetCoreImage.getSectionStartAdr());
		el->appendInputData16((uint16_t)fetCoreImage.getSectionLength());

		while (fetCoreImage.sectionHasNextWord())
		{
			el->appendInputData16(fetCoreImage.getNextWord());
		}
		fetCoreImage.nextSection();
	}
	updateCmd.elements.emplace_back(el);
	return this->fetHandle->send(updateCmd);
}

//reads back flashed core data and compares it to data in core image
//returns: true, if data on FET and core image are equal
bool UpdateManagerMSP_FET430::upCoreRead()
{
	Record fetCoreImage(UifBiosCoreImage, UifBiosCoreImage_address, UifBiosCoreImage_length_of_sections, UifBiosCoreImage_sections);

	updateCmd.elements.clear();
	HalExecElement* el = new HalExecElement(ID_Zero, UpRead);
	el->setAddrFlag(false);
	//append start address for read access
	el->appendInputData32(0x2500);
	//append number of data bytes, including number of sections, section start addresses and core signature
	el->appendInputData32((fetCoreImage.getNumOfAllDataWords() + fetCoreImage.getNumOfManageWords() + 2)*2);
	updateCmd.elements.emplace_back(el);

	if (!this->fetHandle->send(updateCmd))
	{
		return false;
	}
	//compare core signature
	if (el->getOutputAt32(0) != coreSignature)
	{
		return false;
	}
	//compare number of sections
	if (el->getOutputAt16(4) != fetCoreImage.getSectionCount())
	{
		return false;
	}
	uint32_t overhead = 6;

	//compare data of current section, including start adress and length
	while (fetCoreImage.hasNextSection())
	{
		if (el->getOutputAt16(fetCoreImage.getCurrentPosByte() + overhead) != fetCoreImage.getSectionStartAdr())
		{
			return false;
		}
		overhead += 2;
		if (el->getOutputAt16(fetCoreImage.getCurrentPosByte() + overhead) != fetCoreImage.getSectionLength())
		{
			return false;
		}
		overhead += 2;

		while (fetCoreImage.sectionHasNextWord())
		{
			if (el->getOutputAt16(fetCoreImage.getCurrentPosByte() + overhead) != fetCoreImage.getNextWord())
			{
				return false;
			}
		}
		fetCoreImage.nextSection();
	}
	return true;
}
/*EOF*/
