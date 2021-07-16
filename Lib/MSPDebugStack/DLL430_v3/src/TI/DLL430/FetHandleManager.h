/*
 * FetHandleManager.h
 *
 * Base class singleton for managing fet handles.
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

#include "IFetHandle.h"
#include "PortInfo.h"
#include "Log.h"

namespace TI
{
	namespace DLL430
	{
		/** \brief manager to create and destroy API handles */
		class FetHandleManager
		{
		public:
			virtual ~FetHandleManager() {};

			/** \brief create a list of available ports
			 *
			 * The scans the system for connected and supported devices. The result
			 * is cached and reused on the next call if update is not true.
			 *
			 * \param type filter device types
			 * \param update force rescanning
			 * \param open if false, the device is scanned but not opened
			 * \return pointer to the list
			 */
			virtual bool createPortList(const char* type, bool update = false, bool open = true) = 0;

			/** \brief delete list of available ports
			 *
			 * Delete the result of a previous scan
			 *
			 */
			virtual void clearPortList() = 0;

			/** \brief return number of avaible ports
			 *
			 * Return number of ports found by a previous scan
			 *
			 * \return number of ports
			 */
			virtual size_t getPortNumber() = 0;

			/** \brief return the instance PortInfo
			 *
			 * searches in the port list for the connection with given name
			 *
			 * \return pointer to PortInfo instance or nullptr if no instance found
			 */
			virtual PortInfo* getPortElement(std::string name) = 0;

			/** \brief return the instance PortInfo
			 *
			 * return the instance at the given position
			 *
			 * \return pointer to PortInfo instance or nullptr if no instance found
			 */
			virtual PortInfo* getPortElement(long idx) = 0;

			/** \brief return the instance PortInfo
			*
			* return the instance by using the serial number
			*
			* \return pointer to PortInfo instance or nullptr if no instance found
			*/
			virtual PortInfo* getPortElementBySN(std::string serialnumber) = 0;

			/** \brief create a new API handle instance
			 *
			 * This creates a new API handle. You can have more than one but do
			 * not create multiple handles for the same port! The handle MUST be
			 * deleted using destroyFetHandle()!
			 *
			 * \param port the PortInfoList iterator of the selected port
			 * \return pointer to the API handle
			 */
			virtual IFetHandle* createFetHandle(const PortInfo&, TARGET_ARCHITECTURE_t) = 0;

			/** \brief destroy a previously created API handle
			 *
			 * \param handle the API handle to destroy
			 */
			virtual void destroyFetHandle(IFetHandle* handle) const = 0;
		};

	};
};
