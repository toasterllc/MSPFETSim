/*
 * Record.h
 *
 * Handles access to different memory modules by means of address.
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

#pragma once


namespace TI
{
	namespace DLL430
	{

		class Record
		{
		public:
			Record(const uint16_t* data, const uint32_t* address, const uint32_t* length, const uint32_t sections);

			bool sectionHasNextWord() const;
			bool hasNextSection() const;
			bool nextSection();
			bool getWordAtAdr(uint32_t address, uint16_t* retWord) const;
			uint16_t getNextWord();
			uint32_t getSectionStartAdr() const;
			uint32_t getCurrentPosByte() const;
			uint32_t getSectionLength() const;
			uint32_t getSectionCount() const;
			uint32_t getNumOfAllDataWords() const;
			uint32_t getNumOfManageWords() const;

		private:
			const uint16_t* data;
			const uint32_t* address;
			const uint32_t* length;
			const uint32_t sectionCount;
			uint32_t currentSection;
			uint32_t currentWord;

			uint32_t sectionEnd(uint32_t section) const;
		};
	}
}
