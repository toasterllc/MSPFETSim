/*
* fromelement.h
*
* Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
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

#include "adapt_enum.h"
#include "readelement.h"
#include "pugixml.hpp"


template <typename ElementType>
typename std::enable_if<std::is_class<ElementType>::value, void>::type
fromElement(const pugi::xml_node e, ElementType& element)
{
	readElement(e, element);
}

template<typename ElementType>
typename std::enable_if<std::is_enum<ElementType>::value, void>::type
fromElement(const pugi::xml_node e, ElementType& element)
{
	TI::DLL430::fromString(e.text().get(), element);
}

inline void fromElement(const pugi::xml_node e, std::string& v) { v = e.text().get(); }
inline void fromElement(const pugi::xml_node e, int8_t& v) { v = (int8_t)strtol(e.text().get(), nullptr, 0); }
inline void fromElement(const pugi::xml_node e, int16_t& v) { v = (int16_t)strtol(e.text().get(), nullptr, 0); }
inline void fromElement(const pugi::xml_node e, int32_t& v) { v = strtol(e.text().get(), nullptr, 0); }
inline void fromElement(const pugi::xml_node e, uint8_t& v) { v = (uint8_t)strtoul(e.text().get(), nullptr, 0); }
inline void fromElement(const pugi::xml_node e, uint16_t& v) { v = (uint16_t)strtoul(e.text().get(), nullptr, 0); }
inline void fromElement(const pugi::xml_node e, uint32_t& v) { v = strtoul(e.text().get(), nullptr, 0); }
inline void fromElement(const pugi::xml_node e, float& v) { v = (float)strtod(e.text().get(), nullptr); }
inline void fromElement(const pugi::xml_node e, double& v) { v = strtod(e.text().get(), nullptr); }
inline void fromElement(const pugi::xml_node e, bool& v) { v = !strcmp(e.text().get(), "true"); }
