/*
 * UpdateManagerFet.cpp
 *
 * Functionality for updating eZ-FET & MSP-FET debugger
 *
 * Copyright (C) 2007 - 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *	Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 *
 *	Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in the
 *	documentation and/or other materials provided with the
 *	distribution.
 *
 *	Neither the name of Texas Instruments Incorporated nor the names of
 *	its contributors may be used to endorse or promote products derived
 *	from this software without specific prior written permission.
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

#include <BSL430_DLL/Connections/MSPBSL_Connection5xxUSB.h>
#include <BSL430_DLL/MSPBSL_Factory.h>
#include <BSL430_DLL/Utility_Classes/MSPBSL_CRCEngine.h>

#include "UpdateManagerFet.h"
#include "VersionInfo.h"
#include "FetHandle.h"
#include "FetHandleManager.h"
#include "ConfigManager.h"
#include "DeviceInfo.h"
#include "HalExecCommand.h"
#include "FetControl.h"
#include "WatchdogControl.h"
#include "DeviceHandleMSP430.h"
#include "HidUpdateManager.h"

#include "Record.h"
#include "MemoryContent.h"
#include "FileReader.h"

#include "../../../../Bios/include/eZ_FetDcdc.h"
#include "../../../../Bios/include/eZ_FetHal.h"
#include "../../../../Bios/include/eZ_FetHil.h"
#include "../../../../Bios/include/eZ_FetCore.h"
#include "../../../../Bios/include/eZ_FetComChannel.h"
#include "../../../../Bios/include/eZ_FetDcdcController.h"
#include "../../../../Bios/include/eZ_FetDcdcControllerV2x.h"


#include "../../../../Bios/include/MSP_FetDcdc.h"
#include "../../../../Bios/include/MSP_FetHal.h"
#include "../../../../Bios/include/MSP_FetHil.h"
#include "../../../../Bios/include/MSP_FetCore.h"
#include "../../../../Bios/include/MSP_FetComChannel.h"
#include "../../../../Bios/include/MSP_FetDcdcController.h"
#include "../../../../Bios/include/MSP_FetDcdcControllerV2x.h"

//Removed due to license limitations
//#define FPGA_UPDATE

#ifdef FPGA_UPDATE
#include "../../../../Bios/include/MSP_FetFpgaHal.h"
#endif

#include "../../../../Bios/include/ConfigureParameters.h"

using namespace TI::DLL430;
using namespace std;


static string UpdateLog;

UpdateManagerFet::UpdateManagerFet(FetHandle* fetHandle, ConfigManager* configManagerV3, FetHandleManager* fhManager)
	: fetHandle(fetHandle)
	, configManagerV3(configManagerV3)
	, fetHandleManager(fhManager)
	, requiredUpdates(0)
	, percent(0)
	, intCallback(nullptr)
{
}


bool UpdateManagerFet::updateCore(MemoryContent &firmware)
{
	double requiredCoreUpdates = 4.0 + firmware.segments.size();
	double percentCore = 100.0 / requiredCoreUpdates;

	try
	{
		int currentPid = 0;
		string currentDevice;
		if (fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC || fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC_V2x)
		{
			currentPid = MSPBSL_MSP_FET_USB_PID;
			currentDevice = "MSP430F6638";
		}
		else
		{
			currentPid = MSPBSL_EZ_FET_USB_PID;
			currentDevice = "MSP430F5528";
		}

		UpdateLog.append("----TRACE---------------eZ_FET start BSL update------------------------------;\n");

		// erase reset vector of core
		upCoreErase();

		if (intCallback)
		{
			--requiredCoreUpdates;
			intCallback(BL_DATA_BLOCK_PROGRAMMED, (uint32_t)(100 - (requiredCoreUpdates)*percentCore), 0);
		}

		this_thread::sleep_for(chrono::seconds(4));

		fetHandle->shutdown();

		if (intCallback)
		{
			--requiredCoreUpdates;
			intCallback(BL_DATA_BLOCK_PROGRAMMED, (uint32_t)(100 - (requiredCoreUpdates)*percentCore), 0);
		}

		uint32_t countedHidDevices = 0, timeout = 50;
		do
		{
			this_thread::sleep_for(chrono::seconds(1));
			countedHidDevices = HidUpdateManager::countHidDevices(currentPid);

			if (intCallback)
			{
				intCallback(BL_DATA_BLOCK_PROGRAMMED, (uint32_t)(100 - (requiredCoreUpdates)*percentCore), 0);
			}
		}
		while (!countedHidDevices && timeout--);

		if (!countedHidDevices)
		{
			UpdateLog.append("----TRACE--- Did not find any HIDs\n");
		}

		if (intCallback)
		{
			--requiredCoreUpdates;
			intCallback(BL_DATA_BLOCK_PROGRAMMED, (uint32_t)(100 - (requiredCoreUpdates)*percentCore), 0);
		}

		UpdateLog.append("----TRACE----fetHandle->shutdown()\n");
		unique_ptr<MSPBSL_Connection5xxUSB> eZ_FET(dynamic_cast<MSPBSL_Connection5xxUSB*>(MSPBSL_Factory::getMSPBSL_Connection("DEVICE:" + currentDevice + " VID:0x2047 PID:0x"+ convertPid(currentPid) +"")));

		if (eZ_FET.get() == nullptr)
		{
			UpdateLog.append("----TRACE----MSPBSL_Factory::getMSPBSL_Connection()\n");
			return false;
		}

		UpdateLog.append("----TRACE----unique_ptr<MSPBSL_Connection5xxUSB> eZ_FET\n");

		if (eZ_FET->loadRAM_BSL(currentPid) != 0)
		{
			//Reset FET
			eZ_FET->closeBslconnection();
			UpdateLog.append("----TRACE----eZ_FET->loadRAM_BSL() != 0 \n");
			return false;
		}

		string verString = "";
		eZ_FET->TX_BSL_Version(verString);
		UpdateLog.append("----TRACE----eZ_FET->TX_BSL_Version(verString);\n");

		eZ_FET->massErase();
		UpdateLog.append("----TRACE----eZ_FET->massErase();\n");

		if (firmware.segments.empty())
		{
			UpdateLog.append("----TRACE----firmware.segments.empty()\n");
			return false;
		}

		for (size_t i = 0; i < firmware.segments.size(); i++)
		{
			const DataSegment& seg = firmware.segments[i];

			vector<uint8_t> Buffer(seg.data.size());

			MSPBSL_CRCEngine crcEngine("5xx_CRC");
			crcEngine.initEngine(0xFFFF);

			for (uint32_t n=0; n < seg.data.size(); n++)
			{
				Buffer[n]= (seg.data[n] & 0xff);
				crcEngine.addByte(seg.data[n] & 0xff);
			}

			eZ_FET->RX_DataBlockFast(&Buffer[0], (uint32_t)seg.startAddress&0xfffffffe, (uint16_t)seg.data.size());

			uint16_t currentCoreCRC[1] = {0};
			eZ_FET->CRC_Check(currentCoreCRC, (uint32_t)seg.startAddress & 0xfffffffe, static_cast<uint16_t>(seg.data.size()));

			uint32_t expectedCoreCRC = static_cast<uint32_t>(crcEngine.getHighByte()) << 8;
			expectedCoreCRC |= (static_cast<uint32_t> (crcEngine.getLowByte()) & 0x0000FFFF);

			if (expectedCoreCRC != currentCoreCRC[0])
			{
				if (i != 0)// just debug exeption
				{
					eZ_FET->closeBslconnection();
					UpdateLog.append("----TRACE----eZ_FET end BSL update faild\n");
					return false;
				}
			}
			if (intCallback)
			{
				--requiredCoreUpdates;
				intCallback(BL_DATA_BLOCK_PROGRAMMED, (uint32_t)(100 - (requiredCoreUpdates)*percentCore), 0);
			}
		}
		UpdateLog.append("----TRACE---------------eZ_FET end BSL update------------------------------;\n");
		eZ_FET->closeBslconnection();
	}
	catch (...)
	{
		UpdateLog.append("----TRACE----eZ_FET end BSL update failed\n");
		return false;
	}

	if (intCallback)
	{
		--requiredCoreUpdates;
		intCallback(BL_DATA_BLOCK_PROGRAMMED, (uint32_t)(100 - (requiredCoreUpdates)*percentCore), 0);
	}
	return true;
}


bool UpdateManagerFet::isUpdateRequired(TARGET_ARCHITECTURE_t arch) const
{
	bool isUpdateRequired = false;
	if (checkHalVersion() != 0)
	{
		isUpdateRequired = true;
	}
	if (checkCoreVersion() != 0)
	{
		isUpdateRequired = true;
	}
	if (checkDcdcLayerVersion() != 0)
	{
		isUpdateRequired = true;
	}
	if (checkDcdcSubMcuVersion() != 0)
	{
		isUpdateRequired = true;
	}
	if (checkHilVersion() != 0)
	{
		isUpdateRequired = true;
	}
	if (checkFpgaVersion() != 0)
	{
		isUpdateRequired = true;
	}
	if (checkUartVersion() != 0)
	{
		isUpdateRequired = true;
	}
	return isUpdateRequired;
}


uint16_t UpdateManagerFet::checkHilVersion() const
{
	//get current hil version from FET
	const uint16_t currentHilVersion = fetHandle->getControl()->getHilVersion();
	//get hil CRC from FET
	const uint16_t currentHilCrc = fetHandle->getControl()->getFetHilCrc();

	uint16_t expectedHilVersion = 0;
	uint16_t expectedHilCrc = 0;

	Record *image = 0;
	uint16_t retVal = 0;

	if (fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC || fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC_V2x)
	{
		image = new Record(MSP_FetHilImage, MSP_FetHilImage_address, MSP_FetHilImage_length_of_sections, MSP_FetHilImage_sections);
	}
	else
	{
		image = new Record(eZ_FetHilImage, eZ_FetHilImage_address, eZ_FetHilImage_length_of_sections, eZ_FetHilImage_sections);
	}

	//if hil versions or CRC's do not match, update hil
	if (image && image->getWordAtAdr(0x18F6, &expectedHilVersion) && image->getWordAtAdr(0x18FA, &expectedHilCrc))
	{
		if ((expectedHilVersion != currentHilVersion) || (expectedHilCrc != currentHilCrc))
		{
			retVal = 1;
		}
	}

	delete image;

	return retVal;
}

uint16_t UpdateManagerFet::checkUartVersion() const
{
	uint16_t retVal = 0;

	const uint16_t currentUartVersion = fetHandle->getControl()->getFetComChannelVersion();
	const uint16_t currentFetComChannelCrc= fetHandle->getControl()->getFetComChannelCrc();

	uint16_t expectedUartVersion = 0;
	uint16_t expectedFetComChannelCRC = 0;

	Record *image = 0;

	if (fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC || fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC_V2x)
	{
		image = new Record(MSP_FetComChannelImage, MSP_FetComChannelImage_address, MSP_FetComChannelImage_length_of_sections, MSP_FetComChannelImage_sections);
	}
	else
	{
		image = new Record(eZ_FetComChannelImage, eZ_FetComChannelImage_address, eZ_FetComChannelImage_length_of_sections, eZ_FetComChannelImage_sections);
	}

	if (image && image->getWordAtAdr(0x1984, &expectedUartVersion) && image->getWordAtAdr(0x19FA, &expectedFetComChannelCRC))
	{
		if ((expectedUartVersion != currentUartVersion) || (expectedFetComChannelCRC != currentFetComChannelCrc))
		{
			retVal = 1;
		}
	}

	delete image;

	return retVal;
}

uint16_t UpdateManagerFet::checkFpgaVersion() const
{
	uint16_t retVal = 0;

#ifdef FPGA_UPDATE
	if (fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC || fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC_V2x)
	{
		const uint16_t currentFpgaVersion = fetHandle->getControl()->getFetFpgaVersion();
		uint16_t expectedFpgaVersion;

		Record fpgaVersion(MSP_FetFpgaHalImage, MSP_FetFpgaHalImage_address, MSP_FetFpgaHalImage_length_of_sections, MSP_FetFpgaHalImage_sections);

		if (fpgaVersion.getWordAtAdr(0x1978, &expectedFpgaVersion))
		{
			if (expectedFpgaVersion > currentFpgaVersion)
			{
				retVal = 1;
			}
		}
	}
#endif

	return retVal;
}

uint16_t UpdateManagerFet::checkDcdcLayerVersion() const
{
	//get current dcdc layer version from FET
	const uint32_t currentDcdcLayerVersion = fetHandle->getControl()->getDcdcLayerVersion();
	//get dcdc layer CRC from FET
	const uint16_t currentDcdcCrc= fetHandle->getControl()->getFetDcdcCrc();

	uint16_t expectedDcdcCrc = 0;
	uint16_t expectedDcdcLayerVersion = 0;

	Record *image = 0;
	uint16_t retVal = 0;

	if (fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC || fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC_V2x)
	{
		image = new Record(MSP_FetDcdcImage, MSP_FetDcdcImage_address, MSP_FetDcdcImage_length_of_sections, MSP_FetDcdcImage_sections);
	}
	else
	{
		image = new Record(eZ_FetDcdcImage, eZ_FetDcdcImage_address, eZ_FetDcdcImage_length_of_sections, eZ_FetDcdcImage_sections);
	}

	//if hil versions or CRC's do not match, update hil
	if (image && image->getWordAtAdr(0x1804, &expectedDcdcLayerVersion) && image->getWordAtAdr(0x187A, &expectedDcdcCrc))
	{
		if ((expectedDcdcLayerVersion != currentDcdcLayerVersion) || (expectedDcdcCrc != currentDcdcCrc))
		{
			retVal = 1;
		}
	}

	delete image;

	return retVal;
}

uint16_t UpdateManagerFet::checkDcdcSubMcuVersion() const
{
	const uint32_t currentDcdcSubMcuVersion = fetHandle->getControl()->getDcdcSubMcuVersion();
	uint16_t expectedDcdcSubMcuVersion = 0;

	Record *image = 0;
	uint16_t retVal = 0;

	if (fetHandle->getControl()->getFetToolId() == eZ_FET_WITH_DCDC || fetHandle->getControl()->getFetToolId() == eZ_FET_WITH_DCDC_NO_FLOWCT)
	{
		image = new Record(eZ_FetDcdcControllerImage, eZ_FetDcdcControllerImage_address, eZ_FetDcdcControllerImage_length_of_sections, eZ_FetDcdcControllerImage_sections);
	}
	else if (fetHandle->getControl()->getFetToolId() == eZ_FET_WITH_DCDC_V2x)
	{
		image = new Record(eZ_FetDcdcControllerV2xImage, eZ_FetDcdcControllerV2xImage_address, eZ_FetDcdcControllerV2xImage_length_of_sections, eZ_FetDcdcControllerV2xImage_sections);
	}
	else if (fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC)
	{
		image = new Record(MSP_FetDcdcControllerImage, MSP_FetDcdcControllerImage_address, MSP_FetDcdcControllerImage_length_of_sections, MSP_FetDcdcControllerImage_sections);
	}
	else if (fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC_V2x)
	{
		image = new Record(MSP_FetDcdcControllerV2xImage, MSP_FetDcdcControllerV2xImage_address, MSP_FetDcdcControllerV2xImage_length_of_sections, MSP_FetDcdcControllerV2xImage_sections);
	}

	if (image && image->getWordAtAdr(0x1000, &expectedDcdcSubMcuVersion))
	{
		//if dcdc sub-mcu versions do not match, update sub-mcu
		if (currentDcdcSubMcuVersion != expectedDcdcSubMcuVersion)
		{
			retVal = 1;
		}
	}

	delete image;

	return retVal;
}

uint16_t UpdateManagerFet::checkHalVersion() const
{
	//get hal CRC from FET
	const uint16_t currentHalCrc = fetHandle->getControl()->getFetHalCrc();
	uint16_t expectedHalCrc  = 0;

	Record *image;
	uint16_t retVal = 0;

	if (fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC || fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC_V2x)
	{
		image = new Record(MSP_FetHalImage, MSP_FetHalImage_address, MSP_FetHalImage_length_of_sections, MSP_FetHalImage_sections);
	}
	else
	{
		image = new Record(eZ_FetHalImage, eZ_FetHalImage_address, eZ_FetHalImage_length_of_sections, eZ_FetHalImage_sections);
	}

	if (image->getWordAtAdr(0x197A, &expectedHalCrc))
	{
		if (expectedHalCrc != currentHalCrc)
		{
			retVal = 1;
		}
	}

	delete image;

	return retVal;
}


uint16_t UpdateManagerFet::checkCoreVersion() const
{
	//get current core version from FET
	const uint16_t actualFetCoreVersion = fetHandle->getControl()->getFetCoreVersion();
	//get core CRC from FET
	const uint16_t currentCoreCrc= fetHandle->getControl()->getFetCoreCrc();

	uint16_t expectedFetCoreVersion = 0;
	uint16_t expectedCoreCrc = 0;

	uint16_t versionLocation = 0;
	uint16_t crcLocation = 0;

	Record *image;
	uint16_t retVal = 0;

	if (fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC || fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC_V2x)
	{
		image = new Record(MSP_FetCoreImage, MSP_FetCoreImage_address, MSP_FetCoreImage_length_of_sections, MSP_FetCoreImage_sections);
		versionLocation = 0x8004;
		crcLocation = 0x8002;
	}
	else
	{
		image = new Record(eZ_FetCoreImage, eZ_FetCoreImage_address, eZ_FetCoreImage_length_of_sections, eZ_FetCoreImage_sections);
		versionLocation = 0x4404;
		crcLocation = 0x4402;
	}

	if (image->getWordAtAdr(versionLocation, &expectedFetCoreVersion) && image->getWordAtAdr(crcLocation, &expectedCoreCrc))
	{
		//if core versions or CRC's do not match, update core
		if ((expectedFetCoreVersion != actualFetCoreVersion) || (currentCoreCrc != expectedCoreCrc))
		{
			retVal = 1;
		}
	}

	delete image;

	return retVal;
}

VersionInfo UpdateManagerFet::getHalVersion() const
{
	const std::vector<uint8_t>* sw_info = this->fetHandle->getSwVersion();
	const uint16_t currentHalCrc = fetHandle->getControl()->getFetHalCrc();
	uint16_t expectedHalCrc = 0;

	Record *image;

	if (fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC || fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC_V2x)
	{
		image = new Record(MSP_FetHalImage, MSP_FetHalImage_address, MSP_FetHalImage_length_of_sections, MSP_FetHalImage_sections);
	}
	else
	{
		image = new Record(eZ_FetHalImage, eZ_FetHalImage_address, eZ_FetHalImage_length_of_sections, eZ_FetHalImage_sections);
	}

	if (image->getWordAtAdr(0x197A, &expectedHalCrc))
	{
		if (expectedHalCrc != currentHalCrc)
		{
			delete image;
			return VersionInfo(1, 0, 0, 0);
		}
	}

	delete image;

	if (sw_info==nullptr)
	{
		return VersionInfo(0, 0, 0, 0);
	}
	if (sw_info->size()<4)
	{
		return VersionInfo(0, 0, 0, 0);
	}

	unsigned char major = sw_info->at(1);

	return VersionInfo((((major&0xC0)>>6)+1), (major&0x3f), sw_info->at(0),
					   (sw_info->at(3)<<8)+sw_info->at(2));
}

bool UpdateManagerFet::updateHal()
{
	bool returnValue = false;
	MemoryContent firmware;

	if (fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC || fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC_V2x)
	{
		firmware.fromSRec(MSP_FetHalImage, MSP_FetHalImage_address, MSP_FetHalImage_length_of_sections, MSP_FetHalImage_sections);
	}
	else
	{
		firmware.fromSRec(eZ_FetHalImage, eZ_FetHalImage_address, eZ_FetHalImage_length_of_sections, eZ_FetHalImage_sections);
	}
	returnValue = updateFirmware(firmware);
	if (!returnValue)
	{
		UpdateLog.append("----TRACE----HalLayer update failed\n");
	}

	return returnValue;
}

bool UpdateManagerFet::updateHil()
{
	bool returnValue = false;
	MemoryContent firmware;

	if (fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC || fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC_V2x)
	{
		firmware.fromSRec(MSP_FetHilImage, MSP_FetHilImage_address, MSP_FetHilImage_length_of_sections, MSP_FetHilImage_sections);
	}
	else
	{
		firmware.fromSRec(eZ_FetHilImage, eZ_FetHilImage_address, eZ_FetHilImage_length_of_sections, eZ_FetHilImage_sections);
	}

	returnValue = updateFirmware(firmware);
	if (!returnValue)
	{
		UpdateLog.append("----TRACE----HilLayer update failed\n");
	}

	return returnValue;
}

bool UpdateManagerFet::updateFpga()
{
	bool returnValue = true;
	MemoryContent firmware;
#ifdef FPGA_UPDATE

	firmware.fromSRec(MSP_FetFpgaHalImage, MSP_FetFpgaHalImage_address, MSP_FetFpgaHalImage_length_of_sections, MSP_FetFpgaHalImage_sections);

	if (firmware.segments.empty())
	{
		return false;
	}

	this->upInit(1);

	if (returnValue && !this->upErase(firmware))
	{
		returnValue = false;
	}
	if (returnValue && !this->upWrite(firmware))
	{
		returnValue = false;
	}

	// Start the FPGA update
	this->upInit(3);

	this_thread::sleep_for(chrono::seconds(1));

	// Restore the normal HAL module
	if (returnValue && !updateHal())
	{
		returnValue = false;
	}

	if (!returnValue)
	{
		UpdateLog.append("----TRACE----FPGA update failed\n");
	}
#endif
	return returnValue;
}

bool UpdateManagerFet::updateDcdcLayer()
{
	bool returnValue = false;
	MemoryContent firmware;

	if (fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC || fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC_V2x)
	{
		firmware.fromSRec(MSP_FetDcdcImage, MSP_FetDcdcImage_address, MSP_FetDcdcImage_length_of_sections, MSP_FetDcdcImage_sections);
	}
	else
	{
		firmware.fromSRec(eZ_FetDcdcImage, eZ_FetDcdcImage_address, eZ_FetDcdcImage_length_of_sections, eZ_FetDcdcImage_sections);
	}
	returnValue = updateFirmware(firmware);
	if (!returnValue)
	{
		UpdateLog.append("----TRACE----DcdcLayer update failed \n");
	}

	return returnValue;
}

bool UpdateManagerFet::updateSubMcu()
{
	bool returnValue = true;
	MemoryContent firmware;

	IDeviceHandleManager* dhm = this->fetHandle->getDeviceHandleManager();

	 // Now programm the Sub MCU
	this->upInit(2);

	//-----------------------------------------
	configManagerV3->setJtagMode(SPYBIWIRE_DCDC);
	if (!configManagerV3->start())
	{
		UpdateLog.append("----TRACE---- configManagerV3->start() \n");
	}

	// save the device handle to work with
	IDeviceHandle* singleDevice = dhm->createDeviceHandle(0, SPYBIWIRE_IF);

	if (singleDevice == nullptr)
	{
		configManagerV3->stop();
		UpdateLog.append("----TRACE---- singleDevice==NULL \n");
		return false;
	}

	if (singleDevice->getJtagId() != 0x89)
	{
		returnValue = false;
		UpdateLog.append("----TRACE---- singleDevice->getJtagId() != 0x89 \n");
	}

	if (returnValue)
	{
		const long setId = singleDevice->identifyDevice(0, false);
		if (setId == -5555)
		{
			returnValue = false;
			UpdateLog.append("----TRACE---- Fuse Blown\n");
		}
		else if (setId < 0)
		{
			returnValue = false;
			UpdateLog.append("----TRACE----No device detected\n");
		}
	}

	if (returnValue)
	{
		returnValue = this->programmSubMcu(singleDevice);
		if (!returnValue)
		{
			UpdateLog.append("----TRACE----programm the Sub MCU update failed \n");
		}

		this->upInit(0);

		if (!configManagerV3->stop())
		{
			UpdateLog.append("----TRACE----Stop JTAG done failed \n");
		}
	}


	// destroy device handle
	dhm->destroyDeviceHandle(singleDevice);

	return returnValue;
}

bool UpdateManagerFet::updateComChannel()
{
	MemoryContent firmware;

	if (fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC || fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC_V2x)
	{
		firmware.fromSRec(MSP_FetComChannelImage, MSP_FetComChannelImage_address, MSP_FetComChannelImage_length_of_sections, MSP_FetComChannelImage_sections);
	}
	else
	{
		firmware.fromSRec(eZ_FetComChannelImage, eZ_FetComChannelImage_address, eZ_FetComChannelImage_length_of_sections, eZ_FetComChannelImage_sections);
	}

	bool returnValue = updateFirmware(firmware);
	if (!returnValue)
	{
		UpdateLog.append("----TRACE----Uart Layer update failed \n");
	}

	return returnValue;
}

bool UpdateManagerFet::firmWareUpdate(const char* fname, UpdateNotifyCallback callback, bool* coreUpdate)
{
	FetControl* control = this->fetHandle->getControl();
	intCallback = nullptr;
	if (control == nullptr)
	{
		return false;
	}
	if (callback)
	{
		intCallback = callback;
	}
	bool returnValue = true;

	UpdateLog.clear();
	UpdateLog.append("\n\n\n ------------------------Start Firmware update--------------------------- \n");

	if (intCallback)
	{
		intCallback(BL_INIT, 0, 0);
		intCallback(BL_PROGRAM_FIRMWARE, 0, 0);
	}

	if (checkCoreVersion() != 0)
	{
		*coreUpdate = true;
		MemoryContent firmware;
		if (control->getFetToolId() == MSP_FET_WITH_DCDC || control->getFetToolId() == MSP_FET_WITH_DCDC_V2x)
		{
			firmware.fromSRec(MSP_FetCoreImage, MSP_FetCoreImage_address, MSP_FetCoreImage_length_of_sections, MSP_FetCoreImage_sections);
		}
		else
		{
			firmware.fromSRec(eZ_FetCoreImage, eZ_FetCoreImage_address, eZ_FetCoreImage_length_of_sections, eZ_FetCoreImage_sections);
		}
		UpdateLog.append("----TRACE----call updateCore(firmware)\n");
		bool retval = updateCore(firmware);

		if (intCallback)
		{
			intCallback(BL_DATA_BLOCK_PROGRAMMED, 100, 0);
		}

		if (!retval)
		{
#if defined(_WIN32) || defined(_WIN64)
			char binaryPath[256] = { 0 };
			uint32_t pathLength = 0;

			pathLength = GetModuleFileName(0, binaryPath, sizeof(binaryPath));
			while (pathLength > 0 && binaryPath[pathLength - 1] != '\\')
			{
				--pathLength;
			}
			string logfile = string(binaryPath, pathLength) + "Update.log";
#else
			string logfile = "Update.log";
#endif
			UpdateLog.append("\n---------------------Firmware upate end--------------------------\n");

			ofstream(logfile.c_str(), ios::app | ios::out) << UpdateLog;
		}

		return retval;
	}

	// just for core update test do not call during normal debug
	if (fname && string(fname).find("CORE_RST_VECTOR_ERASE") != string::npos)
	{
		upCoreErase();
		if (intCallback)
		{
			intCallback(BL_DATA_BLOCK_PROGRAMMED, 100, 0);
			intCallback(BL_UPDATE_DONE, 0, 0);
			intCallback(BL_EXIT, 0, 0);
		}
		UpdateLog.append("----TRACE----CORE_RST_VECTOR_ERASE done\n");
		return true;
	}
	else if (fname)
	{
		MemoryContent firmware;
		try
		{
			FileReader::create(fname)->read(&firmware);
		}
		catch (const std::runtime_error&)
		{
			UpdateLog.append("----TRACE--- firmware.readFirmware(fname)failed\n");
			return false;
		}
		if (intCallback)
		{
			intCallback(BL_DATA_BLOCK_PROGRAMMED, 0, 0);
		}
		returnValue = updateFirmware(firmware);
		if (!returnValue)
		{
			UpdateLog.append("----TRACE--- returnValue = updateFirmware(firmware) failed\n");
		}

		if (intCallback)
		{
			intCallback(BL_DATA_BLOCK_PROGRAMMED, 100, 0);
			intCallback(BL_UPDATE_DONE, 0, 0);
			intCallback(BL_EXIT, 0, 0);
		}
	}
	else
	{
		requiredUpdates = 1;
		requiredUpdates += numStepsHalFirmware();
		requiredUpdates += numStepsHilFirmware();
		requiredUpdates += numStepsFpgaFirmware();
		requiredUpdates += numStepsDcdcLayerFirmware();
		requiredUpdates += numStepsSubMcuLayerFirmware();
		requiredUpdates += numStepsComChannelFirmware();

		percent = 100 / requiredUpdates;
		this->upInit(4);// stop vcc supervision
		if (intCallback)
		{
			intCallback(BL_DATA_BLOCK_PROGRAMMED, 100-(requiredUpdates)*percent, 0);
		}
		//--------------------------------------HalUpdate-----------------------------------------------------
		if (isHalUpdateRequired())
		{
			returnValue = updateHal();			control->resetCommunication();
		}
		else
		{
			requiredUpdates -= numStepsHalFirmware();
		}

		if (intCallback)
		{
			intCallback(BL_DATA_BLOCK_PROGRAMMED, 100-(requiredUpdates)*percent, 0);
		}
		//--------------------------------------HilUpdate-----------------------------------------------------
		if (returnValue && isHilUpdateRequired())
		{
			returnValue = updateHil();

			control->resetCommunication();
		}
		else
		{
			requiredUpdates -= numStepsHilFirmware();
		}

		if (intCallback)
		{
			intCallback(BL_DATA_BLOCK_PROGRAMMED, 100-(requiredUpdates)*percent, 0);
		}

		//--------------------------------------FpgaUpdate-----------------------------------------------------
		if (returnValue && isFpgaUpdateRequired())
		{
			returnValue = updateFpga();

			control->resetCommunication();
		}
		else
		{
			requiredUpdates -= numStepsFpgaFirmware();
		}

		if (intCallback)
		{
			intCallback(BL_DATA_BLOCK_PROGRAMMED, (100-(requiredUpdates)*percent), 0);
		}

		//--------------------------------------DcdcLayerUpdate-----------------------------------------------------
		if (returnValue && isDcdcLayerUpdateRequired())
		{
			returnValue = updateDcdcLayer();

			control->resetCommunication();
		}
		else
		{
			requiredUpdates -= numStepsDcdcLayerFirmware();
		}

		if (intCallback)
		{
			intCallback(BL_DATA_BLOCK_PROGRAMMED, 100-(requiredUpdates)*percent, 0);
		}

		//--------------------------------------SubMcuUpdate-----------------------------------------------------
		if (returnValue && isSubMcuLayerUpdateRequired())
		{
			returnValue = updateSubMcu();

			control->resetCommunication();
		}
		else
		{
			requiredUpdates -= numStepsSubMcuLayerFirmware();
		}

		if (intCallback)
		{
			intCallback(BL_DATA_BLOCK_PROGRAMMED, 100-(requiredUpdates)*percent, 0);
		}

		//--------------------------------------ComChannelUpdate-----------------------------------------------------
		if (returnValue && isComChannelUpdateRequired())
		{
			returnValue = updateComChannel();

			control->resetCommunication();
		}
		this->upInit(5);// start vcc supervision

		if (intCallback)
		{
			intCallback(BL_DATA_BLOCK_PROGRAMMED, 100, 0);
			intCallback(BL_UPDATE_DONE, 0, 0);
			intCallback(BL_EXIT, 0, 0);
		}
	}


	if (!returnValue)
	{
#if defined(_WIN32) || defined(_WIN64)
		char binaryPath[256] = { 0 };
		uint32_t pathLength = 0;

		pathLength = GetModuleFileName(0, binaryPath, sizeof(binaryPath));
		while (pathLength > 0 && binaryPath[pathLength - 1] != '\\')
		{
			--pathLength;
		}
		string logfile = string(binaryPath, pathLength) + "Update.log";
#else
		string logfile = "Update.log";
#endif
		UpdateLog.append("\n---------------------Firmware upate end--------------------------\n");

		ofstream(logfile.c_str(), ios::app | ios::out) << UpdateLog;
	}
	return returnValue;
}

bool UpdateManagerFet::programmSubMcu(IDeviceHandle * singleDevice)
{
	MemoryContent firmware;

	if (fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC)
	{
		firmware.fromSRec(MSP_FetDcdcControllerImage, MSP_FetDcdcControllerImage_address, MSP_FetDcdcControllerImage_length_of_sections, MSP_FetDcdcControllerImage_sections);
	}
	else if ( fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC_V2x)
	{
		firmware.fromSRec(MSP_FetDcdcControllerV2xImage, MSP_FetDcdcControllerV2xImage_address, MSP_FetDcdcControllerV2xImage_length_of_sections, MSP_FetDcdcControllerV2xImage_sections);
	}
	else if (fetHandle->getControl()->getFetToolId() == eZ_FET_WITH_DCDC_V2x)
	{
		firmware.fromSRec(eZ_FetDcdcControllerV2xImage, eZ_FetDcdcControllerV2xImage_address, eZ_FetDcdcControllerV2xImage_length_of_sections, eZ_FetDcdcControllerV2xImage_sections);
	}
	else
	{
		firmware.fromSRec(eZ_FetDcdcControllerImage, eZ_FetDcdcControllerImage_address, eZ_FetDcdcControllerImage_length_of_sections, eZ_FetDcdcControllerImage_sections);
	}

	if (firmware.segments.empty())
	{
		return false;
	}
	if (!singleDevice)
	{
		UpdateLog.append("----TRACE---- SUB mcu !singleDevice\n");
		return false;
	}
	IMemoryManager* mm = singleDevice->getMemoryManager();

	if (!mm)
	{
		UpdateLog.append("----TRACE---- SUB mcu !mm\n");
		return false;
	}
	MemoryArea* main = mm->getMemoryArea(MemoryArea::Main);

	singleDevice->reset();

	if (intCallback)
	{
		intCallback(BL_DATA_BLOCK_PROGRAMMED, 100-(requiredUpdates)*percent, 0);
	}

	//erase sub MCU.
	bool eraseSubMcuMain = main->erase();
	if (!eraseSubMcuMain)
	{
		UpdateLog.append("----TRACE---- SUB mcu !eraseSubMcuMain\n");
		return false;
	}

	MemoryArea* info = mm->getMemoryArea(MemoryArea::Info);
	singleDevice->reset();

	//erase sub MCU.
	bool eraseSubMcuinfo = info->erase();
	if (!eraseSubMcuinfo)
	{
		UpdateLog.append("----TRACE---- SUB mcu !eraseSubMcuinfo\n");
		return false;
	}

	if (intCallback)
	{
		intCallback(BL_DATA_BLOCK_PROGRAMMED, 100-(--requiredUpdates)*percent, 0);
	}

	singleDevice->reset();

	bool writeSubMcu = true;
	for (DataSegment& segment : firmware.segments)
	{
		if (!mm->write(segment.startAddress, segment.data.data(), segment.data.size()) || !mm->sync())
		{
			UpdateLog.append("----TRACE---- SUB mcu !writeSubMcu\n");
			writeSubMcu = false;
			break;
		}
		if (intCallback)
		{
			intCallback(BL_DATA_BLOCK_PROGRAMMED, 100 - (--requiredUpdates)*percent, 0);
		}
	}
	return writeSubMcu;
}

bool UpdateManagerFet::updateFirmware(const MemoryContent &firmware)
{
	if (firmware.segments.empty())
	{
		return false;
	}
	// start HAL update routine
	this->upInit(1);

	if (!this->upErase(firmware))
	{
		return false;
	}
	if (!this->upWrite(firmware))
	{
		return false;
	}
	this->upInit(0);

	// give the firmware time to execute initialisation
	this_thread::sleep_for(chrono::seconds(1));
	return true;
}

void UpdateManagerFet::upInit(unsigned char level)
{
	HalExecCommand updateCmd;
	updateCmd.setTimeout(10000);
	HalExecElement* el = new HalExecElement(ID_Zero, UpInit);
	el->setAddrFlag(false);
	el->appendInputData8(level);

	updateCmd.elements.emplace_back(el);
	this->fetHandle->send(updateCmd);
}

bool UpdateManagerFet::upErase(const MemoryContent& firmware)
{
	for (size_t i = 0; i < firmware.segments.size(); ++i)
	{
		const DataSegment& seg = firmware.segments[i];

		HalExecElement* el = new HalExecElement(ID_Zero, UpErase);
		el->setAddrFlag(false);

		el->appendInputData32(seg.startAddress & 0xfffffffe);
		el->appendInputData32(static_cast<uint32_t>(seg.data.size()));

		HalExecCommand updateCmd;
		updateCmd.setTimeout(40000);
		updateCmd.elements.emplace_back(el);
		if (this->fetHandle->send(updateCmd)==false)
		{
			return false;
		}
		if (intCallback)
		{
			intCallback(BL_DATA_BLOCK_PROGRAMMED, 100-(--requiredUpdates)*percent, 0);
		}
	}
	return true;
}

bool UpdateManagerFet::upWrite(const MemoryContent& firmware)
{
	for (size_t i = firmware.segments.size(); i > 0; i--)
	{
		const DataSegment& seg = firmware.segments[i-1];

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
		for (size_t n = 0; n < seg.data.size(); n++)
		{
			el->appendInputData8(seg.data[n] & 0xff);
		}
		for (uint32_t p = 0; p < padding; p++)
		{
			el->appendInputData8(0xff);
		}

		HalExecCommand updateCmd;
		updateCmd.setTimeout(45000);
		updateCmd.elements.emplace_back(el);
		if (this->fetHandle->send(updateCmd)==false)
		{
			return false;
		}
		if (intCallback)
		{
			intCallback(BL_DATA_BLOCK_PROGRAMMED, 100-(--requiredUpdates)*percent, 0);
		}
	}
	return true;
}

bool UpdateManagerFet::upCoreErase()
{
	FetControl * control=this->fetHandle->getControl();
	// create 0x55 command erase signature
	// command forces safecore to search for new core on reset
	std::vector<uint8_t> data_55;
	data_55.push_back(0x03);
	data_55.push_back(0x55);
	uint8_t id = control->createResponseId();
	data_55.push_back(id);
	data_55.push_back(0x00);

	control->sendData(data_55);
	control->clearResponse();
	return true;
}

bool UpdateManagerFet::upCoreWrite()
{
	return true;
}

//reads back flashed core data and compares it to data in core image
//returns: true, if data on FET and core image are equal
bool UpdateManagerFet::upCoreRead()
{
	return true;
}

bool UpdateManagerFet::isHalUpdateRequired()
{
	return (getHalVersion().get() == 10000000);
}

bool UpdateManagerFet::isHilUpdateRequired()
{
	return (checkHilVersion() != 0);
}

bool UpdateManagerFet::isFpgaUpdateRequired()
{
	return (checkFpgaVersion() != 0);
}

bool UpdateManagerFet::isDcdcLayerUpdateRequired()
{
	return (checkDcdcLayerVersion() != 0);
}

bool UpdateManagerFet::isSubMcuLayerUpdateRequired()
{
	return (checkDcdcSubMcuVersion() != 0);
}

bool UpdateManagerFet::isComChannelUpdateRequired()
{
	return (checkUartVersion() != 0);
}

uint32_t UpdateManagerFet::numStepsHalFirmware()
{
	if (fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC || fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC_V2x)
	{
		return MSP_FetHalImage_sections * 2;
	}
	else
	{
		return eZ_FetHalImage_sections * 2;
	}
}

uint32_t UpdateManagerFet::numStepsHilFirmware()
{
	if (fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC || fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC_V2x)
	{
		return MSP_FetHilImage_sections * 2;
	}
	else
	{
		return eZ_FetHilImage_sections * 2;
	}
}

uint32_t UpdateManagerFet::numStepsFpgaFirmware()
{
#ifdef FPGA_UPDATE
	return MSP_FetFpgaHalImage_sections * 2;
#else
	return 1;
#endif
}

uint32_t UpdateManagerFet::numStepsDcdcLayerFirmware()
{
	if (fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC || fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC_V2x)
	{
		return MSP_FetDcdcImage_sections * 2;
	}
	else
	{
		return eZ_FetDcdcImage_sections * 2;
	}
}

uint32_t UpdateManagerFet::numStepsSubMcuLayerFirmware()
{
	if (fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC)
	{
		return MSP_FetDcdcControllerImage_sections + 1;
	}
	else if (fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC_V2x)
	{
		return MSP_FetDcdcControllerV2xImage_sections + 1;
	}
	else
	{
		return eZ_FetDcdcControllerImage_sections + 1;
	}
}

uint32_t UpdateManagerFet::numStepsComChannelFirmware()
{
	if (fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC || fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC_V2x)
	{
		return MSP_FetComChannelImage_sections * 2;
	}
	else
	{
		return eZ_FetComChannelImage_sections * 2;
	}
}

/*EOF*/
