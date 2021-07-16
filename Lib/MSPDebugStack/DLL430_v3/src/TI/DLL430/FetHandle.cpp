/*
 * FetHandle.h
 *
 * Communication with FET.
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
#include "FetHandle.h"
#include "FetControl.h"
#include "../../../version.h"
#include "HalExecCommand.h"
#include "ConfigManager.h"
#include "DeviceHandleManager.h"
#include "IoChannelFactory.h"


using namespace TI::DLL430;

const char* FetHandle::id = "FetHandle";

FetHandle::FetHandle(const PortInfo& portInfo, FetHandleManager* fhm, TARGET_ARCHITECTURE_t arch)
 : version(VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, VERSION_BUILD)
 , channel(0)
 , control(0)
 , configManager(0)
 , deviceHandleManager(nullptr)
 , communication(false)
{
	channel = IoChannelFactory::createIoChannel(portInfo);

	if (!this->channel)
		return;

	this->control = new FetControl(this->channel);

	if (this->control->hasCommunication())
	{
		communication=true;
		this->deviceHandleManager = new DeviceHandleManager(this);
		this->configManager = new ConfigManager(this, fhm, arch);
		this->configManager->init();
	}
}

FetHandle::~FetHandle()
{
	delete configManager;
	delete control;
	delete deviceHandleManager;
	delete channel;
}

void FetHandle::shutdown()
{
	if (control)
		control->shutdown();
}

bool FetHandle::hasCommunication() const
{
	return communication;
}

std::string FetHandle::getCurrentPortName()
{
	return this->channel->getName();
}

IConfigManager* FetHandle::getConfigManager ()
{
	return this->configManager;
}

FetControl* FetHandle::getControl ()
{
	return this->control;
}

IDeviceHandleManager* FetHandle::getDeviceHandleManager ()
{
	return this->deviceHandleManager;
}

bool FetHandle::send (HalExecCommand &command)
{
	return this->control->send(command);
}

bool FetHandle::kill(HalExecCommand &command)
{
	bool retVal = false;
	if (control->kill(command.getResponseId()))
	{
		command.clearResponseId();
		retVal = true;
	}
	return retVal;
}

bool FetHandle::kill(uint8_t respId)
{
	return control->kill(respId);
}

bool FetHandle::pauseLoopCmd(unsigned long respId)
{
	return (respId == 0) || control->pauseLoopCmd((uint8_t)respId);
}

bool FetHandle::resumeLoopCmd(unsigned long respId)
{
	return (respId == 0) || control->resumeLoopCmd((uint8_t)respId);
}

const std::vector<uint8_t>* FetHandle::getHwVersion() const
{
	return control ? control->getHwVersion() : nullptr;
}

const std::vector<uint8_t>* FetHandle::getSwVersion() const
{
	return control ? control->getSwVersion() : nullptr;
}

void FetHandle::addSystemNotifyCallback(const NotifyCallback& notifyCallback)
{
	if (this->control!=nullptr)
		this->control->addSystemNotifyCallback(notifyCallback);
}

bool FetHandle::sendHilCommand(HIL_COMMAND command, uint32_t data)
{
	HalExecElement *el = new HalExecElement(ID_HilCommand);
	el->appendInputData32((uint32_t)command);
	el->appendInputData32(data);
	el->appendInputData32(0);
	el->appendInputData32(0);

	HalExecCommand cmd;
	cmd.elements.emplace_back(el);

	return send(cmd);
}

uint64_t FetHandle::sendJtagShift(HIL_COMMAND shiftType, uint64_t data, int32_t bitSize)
{
	HalExecElement *el = new HalExecElement(ID_HilCommand);
	el->appendInputData32((uint32_t)shiftType);
	el->appendInputData64(data);
	el->appendInputData32((uint32_t)bitSize);

	HalExecCommand cmd;
	cmd.elements.emplace_back(el);

	uint64_t result = (uint64_t)-1;
	if (send(cmd))
	{
		result  = cmd.elements[0]->getOutputAt64(0);
	}
	return result;
}

bool FetHandle::setJtagPin(JTAG_PIN pin, bool state)
{
	HalExecElement* el = new HalExecElement(ID_BitSequence);
	el->appendInputData8(1);
	el->appendInputData16(state << pin);
	el->appendInputData16(1 << pin);
	el->appendInputData16(0);

	HalExecCommand cmd;
	cmd.elements.emplace_back(el);
	return send(cmd);
}

bool FetHandle::resetState()
{
	return control ? control->resetFetState() : false;
}
