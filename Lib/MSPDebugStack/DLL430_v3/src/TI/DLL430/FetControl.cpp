/*
 * FetControl.cpp
 *
 * Flash Emulation Tool Control.
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
#include "FetControl.h"

#include "IoChannel.h"
#include "HalResponseHandler.h"
#include "FetControlThread.h"
#include "HalExecCommand.h"
#include "VersionInfo.h"

#include <boost/thread.hpp>

using namespace TI::DLL430;

FetControl::FetControl(IoChannel* channel)
	: communication(true)
	, channel(channel)
	, fetCoreVersion(0)
	, fetHilVersion(0)
	, fetFpgaVersion(0)
	, fetToolId(0x1111)
	, fetDcdcLayerVersion(0)
	, fetDcdcSubMcuVersion(0)
	, fetComChannelVersion(0)
	, fetHilCrc(0)
	, fetHalCrc(0)
	, fetDcdcCrc(0)
	, fetCoreCrc(0)
	, fetComChannelCrc(0)
	, currentId(0)
	, reader(0)
{

	const uint8_t INFO_U1_HW[] = { 0x55, 0xFF, 40, 1 };
	fetHardwareVersions[0xCCCC] = std::vector<uint8_t>(INFO_U1_HW, INFO_U1_HW + 4);

	const uint8_t INFO_EZ_FET_HW[] = { 0x45, 0xFF, 0, 4 };
	fetHardwareVersions[0xAAAA] = std::vector<uint8_t>(INFO_EZ_FET_HW, INFO_EZ_FET_HW + 4);

	const uint8_t INFO_EZ_FET_HW_NO_FLOW_CTS[] = { 0x45, 0xFF, 0, 5 };
	fetHardwareVersions[0xAAAC] = std::vector<uint8_t>(INFO_EZ_FET_HW_NO_FLOW_CTS, INFO_EZ_FET_HW_NO_FLOW_CTS + 4);

	const uint8_t INFO_EZ_FET_HW_V2[] = { 0x45, 0xFF, 0, 6 };
	fetHardwareVersions[0xAAAD] = std::vector<uint8_t>(INFO_EZ_FET_HW_V2, INFO_EZ_FET_HW_V2 + 4);

	const uint8_t INFO_EZ_FET_LITE_HW[] = { 0x45, 0xFF, 0, 3 };
	fetHardwareVersions[0xAAAB] = std::vector<uint8_t>(INFO_EZ_FET_LITE_HW, INFO_EZ_FET_LITE_HW + 4);

	const uint8_t INFO_MSP_FET_HW[] = { 0x55, 0xFF, 0, 3 };
	fetHardwareVersions[0xBBBB] = std::vector<uint8_t>(INFO_MSP_FET_HW, INFO_MSP_FET_HW + 4);

	const uint8_t INFO_MSP_FET_HW2x[] = { 0x55, 0xFF, 0, 3 };
	fetHardwareVersions[0xBBBC] = std::vector<uint8_t>(INFO_MSP_FET_HW2x, INFO_MSP_FET_HW2x + 4);

	this->reader = new FetControlThread(*this);
	this->channel->setParent(this);

	this->channel->open();
	this->reader->start();

	//needs to be true for resetCommunication
	communication = resetCommunication();
}

FetControl::~FetControl ()
{
	boost::unique_lock<boost::mutex> lock(this->rhMutex);

	this->responseHandlers.clear();

	lock.unlock();

	shutdown();

	delete this->reader;
}

void FetControl::shutdown()
{
	if (hasCommunication())
	{
		// com Reset
		std::vector<uint8_t> data;
		data.push_back(0x03);
		data.push_back(0x92);
		data.push_back(0x00);
		data.push_back(0x00);
		sendData(data);
	}
	this->reader->stop();
	this->channel->close();
	communication = false;
}

bool FetControl::hasCommunication() const
{
	return communication;
}

bool FetControl::send(HalExecCommand& command)
{
	boost::unique_lock<boost::recursive_mutex> lock(sendMutex);
	return command.send(*this, *this->channel);
}

bool FetControl::sendData(const std::vector<uint8_t>& data)
{
	boost::unique_lock<boost::recursive_mutex> lock(sendMutex);
	if (data.size()>250)
	{
		return false;
	}
	return this->channel->write(&data[0], data.size()) > 0;
}

bool FetControl::pauseLoopCmd (uint8_t id)
{
	bool success = true;
	if (id > 0)
	{
		HalExecElement* el = new HalExecElement(ID_Zero, CmdPauseLoop);
		el->appendInputData8(id);

		HalExecCommand command;
		command.elements.emplace_back(el);

		success = this->send(command);
	}
	return success;
}

bool FetControl::resumeLoopCmd (uint8_t id)
{
	bool success = true;
	if (id > 0)
	{
		HalExecElement* el = new HalExecElement(ID_Zero, CmdResumeLoop);
		el->appendInputData8(id);

		HalExecCommand command;
		command.elements.emplace_back(el);

		success = send(command);
	}
	return success;
}

bool FetControl::kill (uint8_t id)
{
	bool success = false;

	if (id > 0)
	{
		boost::unique_lock<boost::mutex> lock(this->rhMutex);

		ResponseHandlerTable::iterator it = responseHandlers.find(id);
		if (it != responseHandlers.end())
			responseHandlers.erase(it);
	}

	HalExecCommand kill;
	HalExecElement* el = new HalExecElement(ID_Zero, CmdKill);
	el->appendInputData8(id);
	kill.elements.emplace_back(el);

	success = this->send(kill);
	// Free the ID from the reserved list
	{
		boost::unique_lock<boost::mutex> lock(this->CriMutex);
		std::map<uint8_t, bool>::iterator it = reservedIds.find(id & 0x3f);

		if (it != reservedIds.end())
		{
			reservedIds.erase(it);
		}
	}

	return success;
}

bool FetControl::resetCommunication()
{
	std::this_thread::sleep_for(std::chrono::milliseconds(10));

	std::vector<uint8_t> data;
	data.push_back(0x03);
	data.push_back(0x92);
	data.push_back(0x00);
	data.push_back(0x00);
	this->sendData(data);		// reset connection

	std::this_thread::sleep_for(std::chrono::milliseconds(100));

	HalExecElement* el = new HalExecElement(ID_Zero);
	el->appendInputData8(STREAM_CORE_ZERO_VERSION);

	HalExecCommand cmd;
	cmd.elements.emplace_back(el);

	const bool success = send(cmd);
	if (success)
	{
		fetSwVersion.push_back(el->getOutputAt8(0));
		fetSwVersion.push_back(el->getOutputAt8(1));
		fetSwVersion.push_back(el->getOutputAt8(2));
		fetSwVersion.push_back(el->getOutputAt8(3));
		fetHwVersion.push_back(el->getOutputAt8(4));
		fetHwVersion.push_back(el->getOutputAt8(5));
		fetHwVersion.push_back(el->getOutputAt8(6));
		fetHwVersion.push_back(el->getOutputAt8(7));


		unsigned char major=fetSwVersion.at(1);
		uint8_t minor = (major & 0x3f);
		uint8_t patch = fetSwVersion.at(0);
		uint16_t flavor = (fetSwVersion.at(3) << 8) + fetSwVersion.at(2);
		VersionInfo HalVersion((((major & 0xC0) >> 6) + 1), minor, patch, flavor);

		// this is the minimum hal version number for new eZ-FET or MSP-FET
		if (HalVersion.get() >= 30300000 ||
			(fetSwVersion[0] == 0xAA && fetSwVersion[1] == 0xAA && fetSwVersion[2] == 0xAA && fetSwVersion[3] == 0xAA) || /*eZ-FET*/
			(fetSwVersion[0] == 0xCC && fetSwVersion[1] == 0xCC && fetSwVersion[2] == 0xCC && fetSwVersion[3] == 0xCC) || /*MSP-FET UIF*/
			(fetSwVersion[0] == 0xAA && fetSwVersion[1] == 0xAB && fetSwVersion[2] == 0xAA && fetSwVersion[3] == 0xAB) ||/*eZ-FET no DCDC*/
			(fetSwVersion[0] == 0xBB && fetSwVersion[1] == 0xBB && fetSwVersion[2] == 0xBB && fetSwVersion[3] == 0xBB) ||  /*MSP-FET*/
			(fetSwVersion[0] == 0xBB && fetSwVersion[1] == 0xBC && fetSwVersion[2] == 0xBB && fetSwVersion[3] == 0xBC) ||/*MSP-FET v2*/
			(fetSwVersion[0] == 0xAA && fetSwVersion[1] == 0xAD && fetSwVersion[2] == 0xAA && fetSwVersion[3] == 0xAD)) /*eZ-FET v2*/
		{
			fetToolId	   = (el->getOutputAt16(8));
			fetCoreVersion = (el->getOutputAt16(10));
			fetHilVersion  = (el->getOutputAt16(12));
			fetDcdcLayerVersion  = (el->getOutputAt16(14));
			fetDcdcSubMcuVersion  = (el->getOutputAt16(16));
			fetComChannelVersion = (el->getOutputAt16(18));

			fetHilCrc = (el->getOutputAt16(20));
			fetHalCrc = (el->getOutputAt16(22));
			fetDcdcCrc = (el->getOutputAt16(24));
			fetCoreCrc = (el->getOutputAt16(26));
			fetComChannelCrc = (el->getOutputAt16(28));

			fetFpgaVersion = (el->getOutputAt16(30));
		}
		else if (HalVersion.get() <= 30300000 && HalVersion.get() >= 30000000) // this handles an older verion of MSP-FET430UIF
		{
			fetToolId = 0xCCCC;
			fetCoreVersion = (el->getOutputAt16(8));
		}
		else
		{
			fetToolId = 0x1111;
		}
	}
	return success;
}

bool FetControl::resetFetState()
{
	// reset global vars
	HalExecCommand ResetCmd;
	HalExecElement* el = new HalExecElement(ID_ResetStaticGlobalVars);
	ResetCmd.elements.emplace_back(el);
	return this->send(ResetCmd);
}

uint16_t FetControl::getFetToolId() const
{
 	return fetToolId;
}

uint16_t FetControl::getHilVersion() const
{
 	return fetHilVersion;
}

uint16_t FetControl::getDcdcLayerVersion() const
{
	return fetDcdcLayerVersion;
}

uint16_t FetControl::getDcdcSubMcuVersion() const
{
	return fetDcdcSubMcuVersion;
}

uint16_t FetControl::getFetCoreVersion() const
{
	return fetCoreVersion;
}

uint16_t FetControl::getFetCoreCrc() const
{
	return fetCoreCrc;
}

uint16_t FetControl::getFetHalCrc() const
{
	return fetHalCrc;
}

uint16_t FetControl::getFetHilCrc() const
{
	return fetHilCrc;
}

uint16_t FetControl::getFetFpgaVersion() const
{
	return fetFpgaVersion;
}

uint16_t FetControl::getFetDcdcCrc() const
{
	return fetDcdcCrc;
}

uint16_t FetControl::getFetComChannelVersion() const
{
	return fetComChannelVersion;
}

uint16_t FetControl::getFetComChannelCrc() const
{
	return fetComChannelCrc;
}

bool FetControl::registerResponseHandler (uint8_t id, HalResponseHandlerPtr h)
{
	boost::lock_guard<boost::mutex> lock(this->rhMutex);

	HalResponseHandlerPtr& handler = responseHandlers[id];
	if (!handler)
	{
		handler = h;
		return true;
	}
	return false;
}

void FetControl::unregisterResponseHandler (uint8_t id, HalResponseHandlerPtr h)
{
	boost::lock_guard<boost::mutex> lock(rhMutex);

	ResponseHandlerTable::iterator it = responseHandlers.find(id);
	if (it != responseHandlers.end() && it->second == h)
		responseHandlers.erase(it);
}

void FetControl::unregisterResponseHandler (HalResponseHandlerPtr h)
{
	boost::lock_guard<boost::mutex> lock(rhMutex);

	ResponseHandlerTable::iterator it = responseHandlers.begin();
	while (it != responseHandlers.end())
	{
		ResponseHandlerTable::iterator tmp = it++;
		if (tmp->second == h)
			responseHandlers.erase(tmp);
	}
}

HalResponseHandlerPtr FetControl::findResponseHandler (uint8_t id)
{
	if (id==0)
	{
		// sytem error message from firmware
		return HalResponseHandlerPtr();
	}

	boost::lock_guard<boost::mutex> lock(rhMutex);

	ResponseHandlerTable::iterator it = responseHandlers.find(id);

	if (it != responseHandlers.end())
	{
		if (it->second->isAsync() && !it->second->isContinuous() && (id & 0x40))
		{
			boost::lock_guard<boost::mutex> lock(this->CriMutex);
			// If this was an event that we were waiting for, remove the ID from the reserved list
			std::map<uint8_t, bool>::iterator idIt= reservedIds.find(id & 0x3f);
			if (idIt != reservedIds.end())
			{
				reservedIds.erase(idIt);
			}
		}
	}

	return (it != responseHandlers.end()) ? it->second : HalResponseHandlerPtr();
}

HalResponseHandlerPtr FetControl::findResponseHandler (HalResponseHandlerPtr h)
{
	boost::lock_guard<boost::mutex> lock(this->rhMutex);

	ResponseHandlerTable::iterator iter;

	for (iter = responseHandlers.begin(); iter != responseHandlers.end(); ++iter )
	{
		if (iter->second == h)
			return h;
	}
	return HalResponseHandlerPtr();
}

uint8_t FetControl::createResponseId (bool reserveId)
{
	boost::lock_guard<boost::mutex> lock(this->CriMutex);
	do
	{
		if (++currentId > 0x3f)
			currentId = 0x1;
	} while (reservedIds.count(currentId)>0); // Do not use reserved IDs!

	if (reserveId)
	{
		// If required by the caller, reserve this ID so that it cannot be re-used
		reservedIds[currentId] = true;
	}

	return currentId;
}

void FetControl::clearResponse()
{
	boost::lock_guard<boost::mutex> lock(this->rhMutex);

	currentId = 0x3f;
	this->responseHandlers.clear();
}

const std::vector<uint8_t>* FetControl::getHwVersion()
{
	return &fetHardwareVersions[this->getFetToolId()];
}

const std::vector<uint8_t>* FetControl::getSwVersion() const
{
	return &this->fetSwVersion;
}

std::string FetControl::getSerial() const
{
	return channel ? channel->getSerial() : "";
}

void FetControl::provideSystemErrorMsg(HalResponse& resp)
{
	const HalResponse::errorType error = resp.getError();
	const std::vector<uint8_t>& respData = resp.get();

	if ( error != HalResponse::Error_None )
	{
		if (lNotifyCallback)
		{
			lNotifyCallback(error);
		}
	}
	else if (respData.size() > 4 && respData[0] == 0x92)
	{
		#ifndef NDEBUG
		printf("firmware system error %02x%02x\n", respData[4], respData[3]);
		#endif
	}
	else if (respData.size() > 3 && respData[0] == 0x95)
	{
		if (lNotifyCallback)
		{
			this->lNotifyCallback(respData[3]);
		}
	}
	else
	{
		#ifndef NDEBUG
		printf("firmware system error \n");
		#endif
	}

}

void FetControl::provideSystemConnectMsg(bool connect)
{
	if (!connect && this->lNotifyCallback)
	{
		if (communication)
		{
			communication=false;
			this->lNotifyCallback(0);
		}
	}
}

void FetControl::addSystemNotifyCallback(const NotifyCallback& callback)
{
	boost::lock_guard<boost::mutex> lock(rhMutex);
	lNotifyCallback = callback;
}
