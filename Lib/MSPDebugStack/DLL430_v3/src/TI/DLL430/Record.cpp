/*
 * Record.cpp
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
#include "Record.h"

#include <numeric>


using namespace TI::DLL430;

//provides access to Record Code Array
//
//params: data,	pointer to array containing flash data
//		  address, pointer to array containing the start addresses of all sections
//		  length, pointer to array containing the length of all sections
//		  sections, number of available sections
Record::Record(const uint16_t* data, const uint32_t* address, const uint32_t* length, const uint32_t sections)
: data(data)
, address(address)
, length(length)
, sectionCount(sections)
, currentSection(0)
, currentWord(0)
{
}

//params: none
//returns: true, if there's a next word in current section
bool Record::sectionHasNextWord() const
{
	return currentWord < sectionEnd(currentSection);
}

//params: none
//returns: true, if there's a next section
bool Record::hasNextSection() const
{
	return currentSection < sectionCount;
}

//params: none
//returns: sets current section to next section
//		  (nextWord returns the first word in the new section)
bool Record::nextSection()
{
	if (this->hasNextSection())
	{
		currentWord = sectionEnd(currentSection++);
		return true;
	}
	return false;
}

//params: none
//returns: current word in section
uint16_t Record::getNextWord()
{
	return data[currentWord++];
}

//params: address
//params: if data contains given address,
//returns: true, if data contains given address
bool Record::getWordAtAdr(uint32_t searchAdr, uint16_t* retWord) const
{
	const uint16_t *sectorPtr = data;

	for (uint32_t section = 0; section < sectionCount; ++section)
	{
		if (address[section] <= searchAdr)
		{
			const uint32_t dataOffset = (searchAdr - address[section]) / 2;
			if (dataOffset < length[section])
			{
				*retWord = sectorPtr[dataOffset];
				return true;
			}
		}
		sectorPtr += length[section];
	}
	return false;
}

//params: none
//returns: physical start address of current section
uint32_t Record::getSectionStartAdr() const
{
	return (currentSection < sectionCount) ? address[currentSection] : 0;
}

//params: none
//returns: current position in bytes
uint32_t Record::getCurrentPosByte() const
{
	return currentWord * 2;
}

//params: none
//returns: length of current section in words
uint32_t Record::getSectionLength() const
{
	return (currentSection < sectionCount) ? length[currentSection] : 0;
}

//params: none
//returns: number of all sections in array
uint32_t Record::getSectionCount() const
{
	return sectionCount;
}

//params: none
//returns: number of all words in data array
uint32_t Record::getNumOfAllDataWords() const
{
	return std::accumulate(length, length + sectionCount, 0);
}

//params: true, if core signature will be added to output
//returns: number of additional words for section-address and -length
uint32_t Record::getNumOfManageWords() const
{
	return sectionCount * 2 + 1;
}

//params: section
//returns: absolute position of last word in section
uint32_t Record::sectionEnd(uint32_t section) const
{
	return std::accumulate(length, length + section + 1, 0);
}
