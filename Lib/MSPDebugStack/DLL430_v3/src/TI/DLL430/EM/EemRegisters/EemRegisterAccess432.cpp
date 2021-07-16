/*
 * EemRegisterAccess.cpp
 *
 * Handles access to eem registers through EemMemoryAccess
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

#include "EemRegisterAccess432.h"
#include "ArmRandomMemoryAccess.h"
#include "../Exceptions/Exceptions.h"

namespace
{
	static TI::DLL430::ArmRandomMemoryAccess* ema_ = 0;
}

void TI::DLL430::setEemRegisterAccess432(ArmRandomMemoryAccess* ema)
{
	ema_ = ema;
}

void TI::DLL430::writeEemRegister432(uint32_t reg, uint32_t value)
{
	if (!(ema_ && ema_->write(reg, value) && ema_->sync()))
		throw EM_RegisterWriteException();
}

uint32_t TI::DLL430::readEemRegister432(uint32_t reg)
{
	uint8_t value[4] = {0};
	if (!(ema_ && ema_->read(reg, value, sizeof(value)) && ema_->sync()))
		throw EM_RegisterReadException();

	return value[0] | (value[1] << 8) | (value[2] << 16) | (value[3] << 24);
}
