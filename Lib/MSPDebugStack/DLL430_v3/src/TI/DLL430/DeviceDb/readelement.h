/*
* readelement.h
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

#include <unordered_map>
#include "pugixml.hpp"
#include "fromxml.h"


class ElementTableBase
{
public:
	static void clearTables()
	{
		for (auto& table : tables())
			table->clear();
	}

protected:
	ElementTableBase() { tables().push_back(this); }
	virtual ~ElementTableBase() {}

	virtual void clear() = 0;

private:
	static std::vector<ElementTableBase*>& tables() {
		static std::vector<ElementTableBase*> tables_;
		return tables_;
	}
};


template<typename ElementType>
class ElementTable : ElementTableBase
{
public:
	const ElementType& getElement(const std::string& id)
	{
		const auto it = table.find(id);
		if (it == table.end())
			throw std::runtime_error("element id not found: '" + id + "' for type " + typeid(ElementType).name());

		return it->second;
	}

	void addElement(const std::string& id, const ElementType& element)
	{
		if (!table.insert(typename std::unordered_map<const std::string, ElementType, std::hash<std::string> >::value_type(id, element)).second)
			throw std::runtime_error("element id already in use: '" + id + "' for type " + typeid(ElementType).name());
	}

	void clear() OVERRIDE { table.clear(); }

private:
	std::unordered_map<const std::string, ElementType, std::hash<std::string>> table;
};


template <typename ElementType>
void readElement(const pugi::xml_node e, ElementType& element)
{
	const char* id = e.attribute("id").as_string(nullptr);
	const char* ref = e.attribute("ref").as_string(nullptr);

	static ElementTable<ElementType> table;
	element = ref ? table.getElement(ref) : ElementType();

	for (pugi::xml_node f : e.children())
	{
		fromXml(&element, f);
	}

	if (id)
		table.addElement(id, element);
}
