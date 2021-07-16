/*
 * FetHandleManagerImpl.cpp
 *
 * Implementation of IFetHandle interface.
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

#include "FetHandleManagerImpl.h"
#include "FetHandle.h"
#include "Log.h"
#include "IoChannelFactory.h"
#include "UsbCdcIoChannel.h"
#include "HidUpdateManager.h"

using namespace TI::DLL430;



bool FetHandleManagerImpl::createPortList(const char* type, bool update, bool open)
{
	if ( !ids.empty() && update )
		clearPortList();

	const std::string hid = "HID_FET";

	if (HidUpdateManager::countHidDevices(MSPBSL_EZ_FET_USB_PID) > 0)
	{
		ids[hid] = PortInfo(hid, "", PortInfo::BSL);
	}
	else if (HidUpdateManager::countHidDevices(MSPBSL_MSP_FET_USB_PID) > 0)
	{
		ids[hid] = PortInfo(hid, "", PortInfo::BSL);
	}
	else
	{
		IoChannelFactory::enumeratePorts(ids, type, open);
	}

	return true;
}

PortInfo* FetHandleManagerImpl::getPortElement(std::string name)
{
	PortMap::iterator it = ids.begin();

	//If any USB port was specified, use the first free UIF
	if (name == "USB" || name == "TIUSB" || name=="CDC")
	{
		createPortList("CDC", true, false);

		for (it = ids.begin(); it != ids.end(); ++it)
		{
			if (it->second.status == PortInfo::freeForUse)
				break;
		}
	}
	else
	{
		it = ids.find(name);
	}
	return (it != ids.end()) ? &it->second : nullptr;
}

PortInfo* FetHandleManagerImpl::getPortElementBySN(std::string serialnumber)
{
	createPortList("CDC", true, false);

	PortMap::iterator it = ids.begin();
	for (it = ids.begin(); it != ids.end(); ++it)
	{
		if (it->second.serial == serialnumber)
			break;
	}
	return (it != ids.end()) ? &it->second : nullptr;
}

void FetHandleManagerImpl::clearPortList()
{
	ids.clear();
}

PortInfo* FetHandleManagerImpl::getPortElement(long idx)
{
	PortMap::iterator it = ids.begin();
	for (int i = 0; (i < idx) && (it != ids.end()); ++i)
	{
		++it;
	}
	return (ids.end() != it) ? &it->second : nullptr;
}


IFetHandle* FetHandleManagerImpl::createFetHandle(const PortInfo& portInfo, TARGET_ARCHITECTURE_t arch)
{
	std::unique_ptr<IFetHandle> handle(new FetHandle(portInfo, this,arch));

	return handle->hasCommunication() ? handle.release() : nullptr;
}

void FetHandleManagerImpl::destroyFetHandle (IFetHandle* handle) const
{
	delete handle;
}
