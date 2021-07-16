/*
 * PortInfo.h
 *
 * Information about a physical communication port.
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

		/** \brief information about a port
		 *
		 * This class allows to identify and select a port by its name.
		 * Ports have a prefix like HID (for USB device using the
		 * Human Interface Devices protocol) or LPT (for devices
		 * using the parallel port) and a number as postfix.
		 */
		struct PortInfo
		{
			enum Status {freeForUse, inUseByAnotherInstance};
			enum Type { UNKNOWN, CDC, HID, BSL };

			PortInfo() : name(""), path(""), type(UNKNOWN), serial(""), status(freeForUse), useCrc(false), useFlowControl(false) {}

			PortInfo(const std::string& name, const std::string& path, Type type, const std::string& serial = "")
				: name(name)
				, path(path)
				, type(type)
				, serial(serial)
				, status(freeForUse)
				, useCrc(false)
				, useFlowControl(false)
			{
			}

			std::string name;
			std::string path;
			Type type;
			std::string serial;
			Status status;
			bool useCrc;
			bool useFlowControl;
		};

		typedef std::map<std::string, PortInfo> PortMap;
	}
}
