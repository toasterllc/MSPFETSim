/*
 * EnergyTraceManager.cpp
 *
 * Functionality for EnergyTrace.
 *
 * Copyright (c) 2007 - 2016 Texas Instruments Incorporated - http://www.ti.com/
 *
 * All rights reserved not granted herein.
 * Limited License.
 *
 * Texas Instruments Incorporated grants a world-wide, royalty-free,
 * non-exclusive license under copyrights and patents it now or hereafter
 * owns or controls to make, have made, use, import, offer to sell and sell ("Utilize")
 * this software subject to the terms herein.  With respect to the foregoing patent
 * license, such license is granted  solely to the extent that any such patent is necessary
 * to Utilize the software alone.  The patent license shall not apply to any combinations which
 * include this software, other than combinations with devices manufactured by or for TI (“TI Devices”).
 * No hardware patent is licensed hereunder.
 *
 * Redistributions must preserve existing copyright notices and reproduce this license (including the
 * above copyright notice and the disclaimer and (if applicable) source code license limitations below)
 * in the documentation and/or other materials provided with the distribution
 *
 * Redistribution and use in binary form, without modification, are permitted provided that the following
 * conditions are met:
 *
 *	* No reverse engineering, decompilation, or disassembly of this software is permitted with respect to any
 *     software provided in binary form.
 *	* any redistribution and use are licensed by TI for use only with TI Devices.
 *	* Nothing shall obligate TI to provide you with source code for the software licensed and provided to you in object code.
 *
 * If software source code is provided to you, modification and redistribution of the source code are permitted
 * provided that the following conditions are met:
 *
 *   * any redistribution and use of the source code, including any resulting derivative works, are licensed by
 *     TI for use only with TI Devices.
 *   * any redistribution and use of any object code compiled from the source code and any resulting derivative
 *     works, are licensed by TI for use only with TI Devices.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of its suppliers may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * DISCLAIMER.
 *
 * THIS SOFTWARE IS PROVIDED BY TI AND TI’S LICENSORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL TI AND TI’S LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <pch.h>
#include <float.h>

#include "EnergyTraceManager.h"
#include "EnergyTraceProcessorId7.h"
#include "EnergyTraceProcessorId8.h"
#include "FetHandle.h"
#include "HalExecCommand.h"
#include "HalResponse.h"
#include "HalResponseHandler.h"
#include "FetControl.h"
#include "PollingManager.h"
#include "../warnings/Warnings.h"
#include <boost/thread.hpp>

using namespace TI::DLL430;

static const uint32_t ENERGYTRACE_BUFFER_SIZE = 12;

//-------------------------------------------------------------
EnergyTraceManager::EnergyTraceManager(FetHandle* parent, PollingManager* pollingManager)
 : mParent(parent)
 , mCbx(0)
 , mPollingManager(pollingManager)
 , vcc(0)
{
	mPollingManager->setEnergyTraceCallback(std::bind(&EnergyTraceManager::runEvent, this, std::placeholders::_1));

	if (parent->getControl()->getFetToolId() == eZ_FET_WITH_DCDC 
		|| parent->getControl()->getFetToolId() == eZ_FET_WITH_DCDC_NO_FLOWCT
		|| parent->getControl()->getFetToolId() == eZ_FET_WITH_DCDC_V2x)
	{
		calibrationValues.resize(2);
		const double resistors[2] = {DBL_MAX, 2200.0};
		resistorValues = std::vector<double>(resistors, resistors+2);

		// no load, Calibrate Resistor 4
		uint16_t calResistors[] = {0, 1};
		calibrationResistors = std::vector<uint16_t>(calResistors, calResistors + (sizeof(calResistors) / sizeof(uint16_t)));
		timerStep = 640;
	}

	if (parent->getControl()->getFetToolId() == MSP_FET_WITH_DCDC || parent->getControl()->getFetToolId() ==  MSP_FET_WITH_DCDC_V2x)
	{
		calibrationValues.resize(5);
		const double resistors[5] = {DBL_MAX, 1000.0 + 0.75, 162 + 0.75, 82.0 + 0.75, 51.98};//+0.75 --> R of Switch
		resistorValues = std::vector<double>(resistors, resistors+5);

		uint16_t calResistors[] = {0, 4, 2, 1, 7};
		calibrationResistors = std::vector<uint16_t>(calResistors, calResistors + (sizeof(calResistors) / sizeof(uint16_t)));
		timerStep = 800;
	}
}

//-------------------------------------------------------------
EnergyTraceManager::~EnergyTraceManager ()
{
	mPollingManager->setEnergyTraceCallback(nullptr);
}

bool EnergyTraceManager::ResetEnergyTrace()
{
	boost::lock_guard<boost::mutex> lock(callbackMutex);

	if (mDataProcessor)
	{
		mDataProcessor->Reset();
		mDataProcessor->setCalibrationValues(&calibrationValues[0], vcc);
		return true;
	}
	return false;
}

//-------------------------------------------------------------
bool EnergyTraceManager::startEnergyTrace(DebugEventTarget* cb, ETMode_t mode, ETCallback_mode callbackMode, IDeviceHandle* devHandle)
{
	// Reset the processor at the start of a new EnergyTrace session
	if (cb != nullptr)
	{
		mCbx = cb;
	}

	boost::lock_guard<boost::mutex> lock(callbackMutex);

	if (mParent->getControl()->getFetToolId() == eZ_FET_WITH_DCDC_V2x)
	{		
		//switch on dcdc supply
		HalExecElement* el = new HalExecElement(ID_Zero, coreSwitchFet);
		el->appendInputData16(mParent->getConfigManager()->ET_ON);
		HalExecCommand dcdcCmd;
		dcdcCmd.elements.emplace_back(el);
		dcdcCmd.setTimeout(15000);

		if (!mParent->getControl()->send(dcdcCmd))
		{
			return false;
		}

		if (WarningFactory::instance())
		{
			WarningFactory::instance()->message(MESSAGE_LEVEL_T::MSPDS_MESSAGE_LEVEL_INFORMATION, WarningCode::WARNING_DCDC_SUPPLY);
		}
	}

	// Event type 7 ------------------------------------------------------------------------------------
	if (mode == ET_PROFILING_ANALOG_DSTATE)
	{
		mDataProcessor.reset(new EnergyTraceProcessorId7(static_cast<uint32_t>(calibrationValues.size()), ENERGYTRACE_BUFFER_SIZE, devHandle));
		// Pass on the calibration values to the processor
		mDataProcessor->setTimerStep(timerStep);
		mDataProcessor->setResistorValues(&resistorValues[0]);
		mDataProcessor->setCalibrationValues(&calibrationValues[0], vcc);

		if ((callbackMode == ET_CALLBACKS_ONLY_DURING_RUN) && devHandle)
		{
			if (devHandle->getTargetArchitecture() == MSP430)
				return mPollingManager->startEnergyTracePolling(ET_POLLING_ANALOG_DSTATE, ET_POLLING_GATED_ON);
			else
				return mPollingManager->startEnergyTracePolling(ET_POLLING_ANALOG_DSTATE_PC, ET_POLLING_GATED_ON);
		}
	}

	// Event type 8 ---------------------------------------------------------------------------------------
	if (mode == ET_PROFILING_ANALOG)
	{
		mDataProcessor.reset(new EnergyTraceProcessorId8(static_cast<uint32_t>(calibrationValues.size()), ENERGYTRACE_BUFFER_SIZE, devHandle));
		// Pass on the calibration values to the processor
		mDataProcessor->setTimerStep(timerStep);
		mDataProcessor->setResistorValues(&resistorValues[0]);
		mDataProcessor->setCalibrationValues(&calibrationValues[0], vcc);

		const EtGatedMode gatedMode = (callbackMode == ET_CALLBACKS_ONLY_DURING_RUN) ? ET_POLLING_GATED_ON : ET_POLLING_GATED_OFF;

		return mPollingManager->startEnergyTracePolling(ET_POLLING_ANALOG, gatedMode);
	}

	return false;
}

//-------------------------------------------------------------
void EnergyTraceManager::runEvent(MessageDataPtr messageData)
{
	uint16_t eventMask = 0;
	(*messageData) >> eventMask;

	if (mCbx)
	{
		uint8_t numRecords = 0;
		uint8_t sizeOfRecords = 0;

		(*messageData) >> numRecords;
		(*messageData) >> sizeOfRecords;

		boost::lock_guard<boost::mutex> lock(callbackMutex);

		if (this->mDataProcessor->AddData((void*)messageData->data(), numRecords * sizeOfRecords))
		{
			mCbx->event(DebugEventTarget::EnergyTraceData);
		}
	}
}

//-------------------------------------------------------------
void* EnergyTraceManager::getEnergyTraceBuffer()
{
	if (mDataProcessor)
	{
		return mDataProcessor->GetReadBufferPtr();
	}
	return nullptr;
}

//-------------------------------------------------------------
size_t EnergyTraceManager::getEnergyTraceBufferSize()
{
	if (mDataProcessor)
	{
		return mDataProcessor->GetReadBufferSize();
	}
	return 0;
}

//-------------------------------------------------------------
void EnergyTraceManager::doCalibration(uint16_t vcc)
{
	if (mParent->getControl()->getFetToolId() == eZ_FET_WITH_DCDC ||
		mParent->getControl()->getFetToolId() == eZ_FET_WITH_DCDC_NO_FLOWCT ||
		mParent->getControl()->getFetToolId() == eZ_FET_WITH_DCDC_V2x)
	{
		calibrateResistor(vcc);
	}

	if (mParent->getControl()->getFetToolId() == MSP_FET_WITH_DCDC || mParent->getControl()->getFetToolId() == MSP_FET_WITH_DCDC_V2x)
	{
		calibrateResistor(vcc);
	}

	this->vcc = vcc;
}

//-------------------------------------------------------------
void EnergyTraceManager::calibrateResistor(uint16_t vcc)
{
	// send Calibration comand to dcdc Firmware
	HalExecElement* el = new HalExecElement(ID_Zero, dcdcCalibrate);
	el->setAddrFlag(false);

	el->appendInputData16(static_cast<uint16_t>(calibrationResistors.size())); //count resistors

	// now add Resistors
	for (size_t i = 0; i < calibrationResistors.size(); i++)
	{
		el->appendInputData16(calibrationResistors[i]);
	}
	//cal. VCC
	el->appendInputData16(vcc);

	HalExecCommand dcdcCmd;
	dcdcCmd.setTimeout(20000);
	dcdcCmd.elements.emplace_back(el);
	mParent->getControl()->send(dcdcCmd);

	int pos = 0;
	for (size_t i = 0; i < calibrationResistors.size(); i++, pos+=8)
	{
		double ticks = el->getOutputAt32(pos);
		double time = el->getOutputAt32(pos + 4) * timerStep; // time in ns
		calibrationValues[i] = ticks * 1000.0 * 1000.0 / time;
	}
}

//-------------------------------------------------------------
void EnergyTraceManager::stopPolling()
{
	mPollingManager->stopEnergyTracePolling();

	if (mParent->getControl()->getFetToolId() == eZ_FET_WITH_DCDC_V2x)
	{
		//switch on dcdc supply
		HalExecElement* el = new HalExecElement(ID_Zero, coreSwitchFet);
		el->appendInputData16(mParent->getConfigManager()->LDO_ON);
		HalExecCommand dcdcCmd;
		dcdcCmd.elements.emplace_back(el);
		dcdcCmd.setTimeout(15000);
		mParent->getControl()->send(dcdcCmd);

		if (WarningFactory::instance())
		{
			WarningFactory::instance()->message(MESSAGE_LEVEL_T::MSPDS_MESSAGE_LEVEL_INFORMATION, WarningCode::WARNING_LDO_SUPPLY);
		}
	}
}
