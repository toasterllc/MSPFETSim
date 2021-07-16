/*
 * EnergyTraceProcessor.cpp
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
#include "EnergyTraceProcessor.h"

using namespace TI::DLL430;

//---------------------------------
EnergyTraceProcessor::EnergyTraceProcessor(uint32_t numCalibrationPoints)
	: numCalibrationPoints(numCalibrationPoints)
	, tickThreshold(0)
	, oneTickinMicroWsec(numCalibrationPoints)
	, mVoutFilter(50)
	, mFilterEnable(true)
	, mTimeTag_us(0.0)
	, mPrevTimeTag(0)
	, mPrevCurrentTick(0)
	, mCalibrationValues(numCalibrationPoints)
	, mResistorValues(numCalibrationPoints, 0)
	, timeBase_ns(640)
	, mSkip(0)
{
}

//---------------------------------
EnergyTraceProcessor::~EnergyTraceProcessor()
{
}

//---------------------------------
void EnergyTraceProcessor::calculateCalibration(uint16_t vcc)
{
	for (uint32_t i = 1; i < numCalibrationPoints; ++i)
	{
		const double x0 = mCalibrationValues[i-1].refCurrent;
		const double y0 = mCalibrationValues[i-1].threshold;

		const double x1 = mCalibrationValues[i].refCurrent;
		const double y1 = mCalibrationValues[i].threshold;

		mCalibrationValues[i-1].gradient =  (x1 - x0) / (y1 - y0);
		mCalibrationValues[i-1].offset = y0; // Offset is tick count

		// Calculate energy content in uWsec (=uJ) per 1 tick
		// This calculation assumes that the value stored in
		// calibrationTickArray[x] is per 1msec
		// First we calculate the equivalent of 1 tick per 1 msec in mA current
		// Then we simply multiply it with the actual voltage

		const double oneTickPerMsecinNA = mCalibrationValues[i].refCurrent /
										 (mCalibrationValues[i].threshold - mCalibrationValues[0].threshold);
		oneTickinMicroWsec[i-1] = (oneTickPerMsecinNA/1000.0)*(((double)vcc)/1000.0)/1000.0;
	}

	const double y0 = mCalibrationValues[0].threshold;
	//Added - calculate threshold for current update when current is 0 (standby)
	tickThreshold = (unsigned int)(y0*(double)minUpdateRateInMsec); // Adaptive filter threshold
}

//---------------------------------
void EnergyTraceProcessor::setCalibrationValues(double *calibrationValues, uint16_t vcc)
{
	mCalibrationValues[0].refCurrent = 0;
	mCalibrationValues[0].threshold = calibrationValues[0];

	for (size_t i = 1; i < numCalibrationPoints; ++i)
	{
		mCalibrationValues[i].refCurrent = ((double)vcc) / mResistorValues[i] * 1000.0 * 1000.0;
		mCalibrationValues[i].threshold = calibrationValues[i];
	}

	calculateCalibration(vcc);
}

//---------------------------------
void EnergyTraceProcessor::setResistorValues(double *resistorValues)
{
	for (uint32_t i = 1; i < numCalibrationPoints; ++i)
	{
		mResistorValues[i] = resistorValues[i];
	}
}

//---------------------------------
void EnergyTraceProcessor::setTimerStep(uint32_t step)
{
	timeBase_ns = step;
}
