/*
* setmember.h
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


#include <type_traits>
#include <boost/mpl/size.hpp>
#include <boost/fusion/sequence/intrinsic/at_c.hpp>

#include "fromelement.h"


template<class StructType, const size_t Idx>
static void setFieldByName(pugi::xml_node, StructType&, std::true_type) {}


template<class StructType, const size_t Idx>
static void setFieldByName(pugi::xml_node xmlElement, StructType& obj, std::false_type)
{
	const char* fieldName = boost::fusion::extension::struct_member_name<StructType, Idx>::call();

	if (!strcmp(xmlElement.name(), fieldName))
	{
		fromElement(xmlElement, boost::fusion::at_c<Idx>(obj));
		return;
	}

	setFieldByName<StructType, Idx + 1>(xmlElement, obj, std::integral_constant<bool, Idx + 1 == boost::mpl::size<StructType>::value>());
}


template<class StructType>
void setMember(StructType& c, pugi::xml_node e)
{
	setFieldByName<StructType, 0>(e, c, std::integral_constant<bool, 0 == boost::mpl::size<StructType>::value>());
}
