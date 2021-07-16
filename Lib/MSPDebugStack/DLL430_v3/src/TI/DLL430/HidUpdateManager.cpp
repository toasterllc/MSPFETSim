/*
 * HidUpdateManager.h
 *
 * Recovery for broken eZ-FETs and MSP-FET Debuggers
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
#include "HidUpdateManager.h"

#include <hidapi.h>

#include <BSL430_DLL/Connections/MSPBSL_Connection5xxUSB.h>
#include <BSL430_DLL/MSPBSL_Factory.h>
#include <BSL430_DLL/Utility_Classes/MSPBSL_CRCEngine.h>

#include "MemoryContent.h"

#include "../../../../Bios/include/eZ_FetCore.h"
#include "../../../../Bios/include/MSP_FetCore.h"

using namespace TI::DLL430;
using namespace std;

struct BslFetWrapper
{
	explicit BslFetWrapper(MSPBSL_Connection5xxUSB *ptr) : ptr_(ptr)
	{
	}

	BslFetWrapper& operator=(BslFetWrapper& bsl)
	{
		return *this;
	}

	~BslFetWrapper()
	{
		if (ptr_)
		{
			ptr_->closeBslconnection();
			delete ptr_;
		}

		ptr_ = nullptr;

		hid_exit();
	}

	MSPBSL_Connection5xxUSB* get()
	{
		return ptr_;
	}

	bool isNull() const
	{
		return ptr_ == nullptr;
	}

	MSPBSL_Connection5xxUSB *ptr_;
};

uint32_t HidUpdateManager::countHidDevices(uint16_t productId)
{
	hid_device_info *hidDevices = hid_enumerate(MSPBSL_STANDARD_USB_VID, productId);
	hid_device_info *hidDevicesIt = hidDevices;

	uint32_t count = 0;
	while (hidDevicesIt != nullptr)
	{
		count++;
		hidDevicesIt =  hidDevicesIt->next;
	}
	hid_free_enumeration(hidDevices);
	hid_exit();

	return count;
}


HidUpdateManager::HidUpdateManager ()
{
}

HidUpdateManager::~HidUpdateManager (){}

bool HidUpdateManager::hid_firmWareUpdate(const char * fname, UpdateNotifyCallback callback)
{
	bool returnValue = false;
	try
	{
		const bool eZRecoveryNeeded = (HidUpdateManager::countHidDevices(MSPBSL_EZ_FET_USB_PID) > 0);
		const bool mspFetRecoveryNeeded = (HidUpdateManager::countHidDevices(MSPBSL_MSP_FET_USB_PID) > 0);

		uint16_t currentPid = 0;
		if (eZRecoveryNeeded)
		{
			currentPid = MSPBSL_EZ_FET_USB_PID;
		}
		else if (mspFetRecoveryNeeded)
		{
			currentPid = MSPBSL_MSP_FET_USB_PID;
		}

		//------------------------------------------------------------------------------------------
		BslFetWrapper bslWrapper(hid_enumerateBSL(currentPid));

		if (bslWrapper.isNull())
		{
			return false;
		}

		uint32_t requiredUpdates = 3;
		const uint32_t percent = 100/requiredUpdates;

		if (callback)
		{
			callback(BL_INIT, 0, 0);
			callback(BL_PROGRAM_FIRMWARE, 0, 0);
			callback(BL_DATA_BLOCK_PROGRAMMED, 0, 0);
		}

		MemoryContent firmware;

		if (eZRecoveryNeeded)
		{
			uint16_t toolIdBSL = hid_getBSLToolId(bslWrapper.get());
			if (toolIdBSL == eZ_FET_WITH_DCDC || toolIdBSL == eZ_FET_WITH_DCDC_0X3FF || toolIdBSL == eZ_FET_NO_DCDC || toolIdBSL == eZ_FET_WITH_DCDC_NO_FLOWCT || toolIdBSL == eZ_FET_WITH_DCDC_V2x)
			{
				//load eZ-FET CORE imag ewith TOOL ID 0xAAAA -> DCDC Sub mcu is present
				firmware.fromSRec(eZ_FetCoreImage, eZ_FetCoreImage_address, eZ_FetCoreImage_length_of_sections, eZ_FetCoreImage_sections);
			}
		}
		else if (mspFetRecoveryNeeded)
		{
			uint16_t toolIdBSL = hid_getBSLToolId(bslWrapper.get());
			if (toolIdBSL == MSP_FET_WITH_DCDC || toolIdBSL == MSP_FET_WITH_DCDC_V2x)
			{
				firmware.fromSRec(MSP_FetCoreImage, MSP_FetCoreImage_address, MSP_FetCoreImage_length_of_sections, MSP_FetCoreImage_sections);
			}
		}

		if (callback)
		{
			callback(BL_DATA_BLOCK_PROGRAMMED, (100-(--requiredUpdates)*percent), 0);
		}

		returnValue = hid_updateCore(firmware, bslWrapper.get());

		if (callback)
		{
			callback(BL_DATA_BLOCK_PROGRAMMED, (100-(--requiredUpdates)*percent), 0);
		}
	}
	catch (...)
	{
		returnValue = false;
	}

	if (callback)
	{
		callback(BL_DATA_BLOCK_PROGRAMMED, 100, 0);
		callback(BL_UPDATE_DONE, 0, 0);
		callback(BL_EXIT, 0, 0);
	}
	return returnValue;
}

uint16_t HidUpdateManager::hid_getBSLToolId(MSPBSL_Connection5xxUSB* bsl)
{
	uint8_t data[2];
	// read tool-ID stored in BSL memory
	bsl->TX_DataBlock(data, 0x100e, 2);
	uint16_t toolId = data[1];
	toolId = toolId<<8;
	toolId |= static_cast<uint16_t>(data[0]);
	return toolId;
}

MSPBSL_Connection5xxUSB* HidUpdateManager::hid_enumerateBSL(uint16_t currentPid)
{
	string currentDevice = "";

	if (currentPid == MSPBSL_MSP_FET_USB_PID)
	{
		currentDevice = "MSP430F6638";
	}
	if (currentPid == MSPBSL_EZ_FET_USB_PID)
	{
		currentDevice = "MSP430F5528";
	}

	MSPBSL_Connection5xxUSB* BslFet = dynamic_cast<MSPBSL_Connection5xxUSB*>(MSPBSL_Factory::getMSPBSL_Connection("DEVICE:" + currentDevice + " VID:0x2047 PID:0x"+ convertPid(currentPid) +"")); // works for 6638 and 5528 because of generic usb bsl

	if (BslFet)
	{
		if (BslFet->loadRAM_BSL(currentPid) != 0)
		{
			//Reset FET
			BslFet->closeBslconnection();
			delete BslFet;
			return 0;
		}
		string verString = "BUG";
		BslFet->TX_BSL_Version(verString);
	}

	return BslFet;
}

bool HidUpdateManager::hid_updateCore(const MemoryContent &firmware, MSPBSL_Connection5xxUSB* BslFet) const
{
	// erase reset vector of core
	BslFet->massErase();
	BslFet->eraseSegment(0x197F);
	BslFet->eraseSegment(0x18FF);
	BslFet->eraseSegment(0x187F);

	// Info A hanlding -> unlock it
	BslFet->toggleInfo();
	// erase Info A
	BslFet->eraseSegment(0x19FF);

	if (firmware.segments.empty())
	{
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

		BslFet->RX_DataBlockFast(&Buffer[0], (uint32_t)seg.startAddress&0xfffffffe, (uint16_t)seg.data.size());

		uint16_t currentCoreCRC[1];
		BslFet->CRC_Check(currentCoreCRC, (uint32_t)seg.startAddress & 0xfffffffe, static_cast<uint16_t>(seg.data.size()));

		uint32_t expectedCoreCRC = crcEngine.getHighByte()<<8;
		expectedCoreCRC |= static_cast<uint32_t>(crcEngine.getLowByte());

		if (expectedCoreCRC != currentCoreCRC[0])
		{
			if (i != 0)// just debug exeption
			{
				BslFet->closeBslconnection();
				return false;
			}
		}

	}
	return true;
}
