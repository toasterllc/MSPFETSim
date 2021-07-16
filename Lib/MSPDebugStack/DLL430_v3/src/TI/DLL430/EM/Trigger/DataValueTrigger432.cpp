/*
 * DataValueTrigger432.cpp
 *
 * Common implementation for triggers on 432
 *
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
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

#include "DataValueTrigger432.h"
#include "../EemRegisters/EemRegisterAccess432.h"
#include "../Exceptions/Exceptions.h"

using namespace std;
using namespace TI::DLL430;

DataValueTrigger432::DataValueTrigger432(TYPE type, uint32_t id)
	: DataTrigger432(type, id),
	dependencyTrigger_(nullptr),
	dataSize_(0)
{
}

void DataValueTrigger432::read()
{
}


void DataValueTrigger432::write() const
{
	if (dependencyTrigger_ == nullptr)
		return;
	static uint32_t tmp = readEemRegister432(0x0DFC);
	/* write linked trigger first, since the function register will be overwritten by the hardware
	*  when setting data value trigger
	*/
	dependencyTrigger_->write();
	// Disable DWT before config - function regsier to 0	
	writeEemRegister432(0x1028 + id_ * 0x10, 0);
	writeEemRegister432(0x1020 + id_ * 0x10, comparatorRegister_);
	// set mask
	writeEemRegister432(0x1024 + id_ * 0x10, mask_);
	// Enable DWT after config	
	writeEemRegister432(0x1028 + id_ * 0x10, functionRegister_);
}

void DataValueTrigger432::reset()
{
	if (dependencyTrigger_ != nullptr)
		dependencyTrigger_->reset();
	dependencyTrigger_ = nullptr;

	comparatorRegister_ = 0;
	functionRegister_ = 0;
	mask_ = 0;

	isInUse_ = false;
	isEnabled_ = false;
}

void DataValueTrigger432::setDependencyTrigger(DataTrigger432 *dependencyTrigger)
{
	dependencyTrigger_ = dependencyTrigger;
	/* set DATAVADDR0 and DATAVADDR1 to dependency comperator register (12:15 and 16:19)
	*/
	functionRegister_ = (functionRegister_ & (~(0xff << 12))) | (dependencyTrigger_->getId() << 12) | (dependencyTrigger_->getId() << 16);
}

void DataValueTrigger432::setAddress(uint32_t address, uint32_t mask)
{
	if (dependencyTrigger_ == nullptr)
		return;
	dependencyTrigger_->setValue(address);
	dependencyTrigger_->setMask(mask);
	/* Set DATAVMATCH bit */
	functionRegister_ |= (1 << 8);
}

void DataValueTrigger432::setDataSize(uint8_t dataSize)
{
	dataSize_ = dataSize;
	/* bit 10..11 */
	functionRegister_ = (functionRegister_ & (~(0x3<<10))) | (dataSize_ << 10);
}