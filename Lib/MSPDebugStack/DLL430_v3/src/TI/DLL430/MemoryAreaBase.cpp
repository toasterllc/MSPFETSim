/*
 * MemoryAreaBase.cpp
 *
 * Base class for all memory classes.
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
#include "MemoryAreaBase.h"
#include "CpuRegisters.h"
#include "IMemoryManager.h"
#include "HalExecCommand.h"
#include "IDeviceHandle.h"

using namespace TI::DLL430;

MemoryAreaBase::MemoryAreaBase (MemoryArea::Name name,
								IDeviceHandle* devHandle,
								uint32_t start,
								uint32_t size,
								uint32_t seg,
								uint32_t banks,
								bool mapped,
								const bool protectable,
								uint8_t psa)

 : name(name)
 , devHandle(devHandle)
 , psaType(psa)
 , err(MEMORY_NO_ERROR)
 , start(start)
 , end(start + size - 1)
 , segmentSize(seg)
 , banks(banks)
 , mapped(mapped)
 , isProtectable(protectable)
 , locked(isProtectable ? true : false)
 , isAccessible_(true)
{
}

MemoryArea::Name MemoryAreaBase::getName () const
{
	return this->name;
}

MemoryError MemoryAreaBase::getError ()
{
	MemoryError error = err;
	err = MEMORY_NO_ERROR;
	return error;
}

bool MemoryAreaBase::isReadOnly () const
{
	return false;
}

uint32_t MemoryAreaBase::getStart () const
{
	return start;
}

uint32_t MemoryAreaBase::getEnd () const
{
	return end;
}

uint32_t MemoryAreaBase::getSize () const
{
	return (this->end - this->start) + 1;
}

uint32_t MemoryAreaBase::getSegmentSize () const
{
	return segmentSize;
}

uint32_t MemoryAreaBase::getBanks () const
{
	return banks;
}

bool MemoryAreaBase::isMapped () const
{
	return mapped;
}

bool MemoryAreaBase::isLocked () const
{
	return isProtectable && locked;
}

//If memory isn't lockable, return true and pretend success
//(otherwise certain IDEs will abort initialization)
bool MemoryAreaBase::lock ()
{
	if (isProtectable)
		locked = true;

	return true;
}

bool MemoryAreaBase::unlock ()
{
	if (isProtectable)
		locked = false;

	return true;
}

bool MemoryAreaBase::read (uint32_t address, uint8_t* buffer, size_t count)
{
	err = MEMORY_READ_ERROR;
	bool success = doRead(address, buffer, count);
	if (success)
		err = MEMORY_NO_ERROR;

	return success;
}


MemoryAreaBase::Alignment MemoryAreaBase::alignData(uint32_t address, uint32_t count) const
{
	return Alignment(address, 0, 0);
}


bool MemoryAreaBase::overwrite (uint32_t address, const uint8_t* buffer, size_t count)
{
	err = MEMORY_WRITE_ERROR;
	bool success = doOverwrite(address, buffer, count);
	if (success)
		err = MEMORY_NO_ERROR;

	return success;
}


bool MemoryAreaBase::write (uint32_t address, const uint8_t* buffer, size_t count)
{
	err = MEMORY_WRITE_ERROR;
	bool success = doWrite(address, buffer, count);
	if (success)
		err = MEMORY_NO_ERROR;

	return success;
}

bool MemoryAreaBase::write (uint32_t address, uint32_t value)
{
	err = MEMORY_WRITE_ERROR;
	bool success = doWrite(address, value);
	if (success)
		err = MEMORY_NO_ERROR;

	return success;

}

bool MemoryAreaBase::defaultRead(hal_id readMacro, IMemoryManager* mm, uint32_t address, uint8_t* buffer, size_t count)
{
	uint32_t pc = 0;

	if (mm)
	{
		CpuRegisters* cpu = mm->getCpuRegisters();
		if (!cpu)
		{
			return false;
		}

		cpu->read(0, &pc, 1);
	}

	const bool omitFirst = (address & 0x1);
	if (omitFirst)
	{
		--address;
		++count;
	}
	const bool omitLast = (count & 1);
	if (omitLast)
	{
		++count;
	}

	HalExecElement* el = new HalExecElement(this->devHandle->checkHalId(readMacro));
	el->appendInputData32(this->getStart() + address);
	el->appendInputData32(static_cast<uint32_t>(count / 2));
	el->appendInputData32(pc);
	el->setOutputSize(count);

	ReadElement r(buffer, count, omitFirst, omitLast, address);
	this->readMap[this->elements.size()] = r;
	this->elements.emplace_back(el);
	return true;
}


bool MemoryAreaBase::doRead(uint32_t address, uint8_t* buffer, size_t count)
{
	return defaultRead(ID_ReadMemWords, nullptr, address, buffer, count);
}

bool MemoryAreaBase::doOverwrite(uint32_t address, const uint8_t* buffer, size_t count)
{
	return doWrite(address, buffer, count);
}

bool MemoryAreaBase::doWrite (uint32_t address, const uint8_t* buffer, size_t count)
{
	return false;
}

bool MemoryAreaBase::doWrite (uint32_t address, uint32_t value)
{
	return false;
}

bool MemoryAreaBase::erase ()
{
	//return true if nothing implemented, means nothing to do
	return true;
}

bool MemoryAreaBase::erase(uint32_t start, uint32_t end, bool forceUnlock)
{
	//return true if nothing implemented, means nothing to do
	return true;
}

bool MemoryAreaBase::verify(uint32_t address, const uint8_t* buffer, size_t count)
{
	if (address & 0x1)
	{
		uint8_t tmp = 0;
		if (!this->read(address++, &tmp, 1) || !this->sync())
			return false;

		if (buffer? tmp != *(buffer++): tmp != 0xFF)
			return false;

		--count;
	}

	if (count > 1)
	{
		HalExecCommand cmd;
		cmd.setTimeout( std::max(20000u, static_cast<uint32_t>(count) / 15) ); //Set timeout according to data size

		HalExecElement*el = new HalExecElement(this->devHandle->checkHalId(ID_Psa));
		el->appendInputData32(static_cast<uint32_t>((address+this->getStart()) & 0xFFFFFFFF));
		el->appendInputData32(static_cast<uint32_t>((count / 2) & 0xFFFFFFFF));
		el->appendInputData8(this->psaType);
		cmd.elements.emplace_back(el);
		if (!this->devHandle->send(cmd))
			return false;

		if (MemoryAreaBase::psa(address+this->getStart(), buffer, (size_t)((uint32_t)count&0xfffffffe)) != el->getOutputAt16(0))
			return false;
	}

	if (count & 1)
	{
		uint8_t tmp = 0;
		if (!this->read(address + static_cast<uint32_t>(count) - 1, &tmp, 1) || !this->sync())
			return false;

		if (buffer? tmp != buffer[count - 1]: tmp != 0xFF)
			return false;
	}
	return true;
}

bool MemoryAreaBase::send(std::vector< std::unique_ptr<HalExecElement> >* elem, HalExecCommand* cmd)
{
	move(elem->begin(), elem->end(), back_inserter(cmd->elements));
	elem->clear();
	if (!this->devHandle->send(*cmd))
	{
		move(cmd->elements.begin(), cmd->elements.end(), back_inserter(*elem));
		cmd->elements.clear();
		return false;
	}
	return true;
}

bool MemoryAreaBase::sync()
{
	if (!preSync())
		return false;

	if (this->elements.empty())
		return true;

	HalExecCommand cmd;
	cmd.setTimeout(60000);

	if (!send(&this->elements, &cmd))
		return false;

	return postSync(cmd);
}

uint16_t MemoryAreaBase::psa(uint32_t address, const uint8_t* buffer, size_t count)
{
	if (address & 0x1)
		return false;

	if (count & 1)
		return false;

	/* Start value for PSA calculation */
	const uint16_t initial = static_cast<uint16_t>((address-2) & 0xFFFF);
	/* Polynom value for PSA calculation */
	const uint16_t polynom = 0x0805;

	uint16_t remainder = initial;
	for (size_t i = 0; i < count; i += 2) {
		// Calculate the PSA (Pseudo Signature Analysis) value
		if ((remainder & 0x8000) != 0)
			remainder = ((remainder ^ polynom) << 1) | 0x0001;
		else
			remainder <<= 1;

		// if pointer is 0 then use erase check mask, otherwise data
		if (buffer == 0)
			remainder ^= static_cast<uint16_t>(0xFFFF);
		else
			remainder ^= static_cast<uint16_t>((buffer[i] & 0xFF) | ((buffer[i+1] & 0xFF) << 8));
	}

	return remainder;
}

void MemoryAreaBase::setAccessible(bool accessible)
{
	isAccessible_ = accessible;
}

bool MemoryAreaBase::isAccessible() const
{
	return isAccessible_;
}

bool MemoryAreaBase::postSync(const HalExecCommand& cmd)
{
	for (size_t n = 0; n < cmd.elements.size(); ++n)
	{
		ReadElement_map::iterator it = this->readMap.find(n);
		if (it != this->readMap.end())
		{
			ReadElement r = it->second;
			size_t size = r.size;
			if (r.omitLast)
			{
				--size;
			}

			const HalExecElement& el = *cmd.elements[n];
			for (size_t i = 0, k = (r.omitFirst ? 1 : 0); k < size; ++k, ++i)
			{
				r.v_buffer[i] = el.getOutputAt8(k);
			}
			this->readMap.erase(it);
		}
	}
	return true;
}
