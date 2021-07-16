/*
 * IUpdateManager.h
 *
 * Provides routines for update handling for various debuggers
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

#include <MSP430_FET.h>
#include "VersionInfo.h"


static const uint16_t MSPBSL_STANDARD_USB_VID = 0x2047;
static const uint16_t MSPBSL_EZ_FET_USB_PID = 0x0203;
static const uint16_t MSPBSL_MSP_FET_USB_PID = 0x0204;

static const uint16_t eZ_FET_WITH_DCDC = 0xAAAA;
static const uint16_t eZ_FET_NO_DCDC = 0xAAAB;
static const uint16_t eZ_FET_WITH_DCDC_NO_FLOWCT = 0xAAAC;
static const uint16_t eZ_FET_WITH_DCDC_V2x = 0xAAAD;
static const uint16_t eZ_FET_WITH_DCDC_0X3FF = 0x3FFF;
static const uint16_t MSP_FET_WITH_DCDC = 0xBBBB;
static const uint16_t MSP_FET_WITH_DCDC_V2x = 0xBBBC;
static const uint16_t MSP_FET430 = 0xCCCC;
static const uint16_t DUMMY = 0x1111;

namespace TI
{
	namespace DLL430
	{
		typedef std::function<void(uint32_t, uint32_t, uint32_t)> UpdateNotifyCallback;

		/** \brief manage the target device and the connection between FET and target device */
		class IUpdateManager
		{
		public:
			virtual ~IUpdateManager() {}

			/** \brief get the version of the FET Hal
			 *
			 * \return the version information
			 */
			virtual VersionInfo getHalVersion () const = 0;

			/** \brief chick if firmware update is required
			 *
			 * \return true if firmware update is required
			 */
			virtual bool isUpdateRequired(TARGET_ARCHITECTURE_t arch) const = 0;

			/** \brief perform firmwareupdate
			 *
			 * \param fname defines the TI-txt file to be used for update or nullptr for internal image
			 * \param callback defines the callback for update messages or nullptr for no messages
			 * \param clientHandle reference given by the caller instance, returned in callback
			 * \return true on success
			 */
			virtual bool firmWareUpdate(const char* fname, UpdateNotifyCallback callback = 0, bool* coreUpdate = 0) = 0;
		};
	}
}
