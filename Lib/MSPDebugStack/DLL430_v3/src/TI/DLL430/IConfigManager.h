/*
 * IConfigManager.h
 *
 * Handles JTAG communication.
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

#include "VersionInfo.h"
#include "IUpdateManager.h"

namespace TI
{
	namespace DLL430
	{
		static const uint16_t STOP_DEVICE = 0xA55A;
		static const uint16_t MAIN_ERASE_MODE = 0x1A1A;
		static const uint16_t MASS_ERASE_MODE = 0x1B1B;
		static const uint16_t LONG_MAILBOX_MODE = 0x11;

		class IDeviceHandle;
		class FetHandleManager;
		class EnergyTraceManager;
		/** \brief manage the target device and the connection between FET and target device */
		class IConfigManager
		{
		public:
			virtual ~IConfigManager() {}

			/** \brief get the version of the FET BIOS
			 *
			 * \return the version information
			 */
			virtual VersionInfo getHalVersion () const = 0;

			/** \brief set the VCC for the target device
			 *
			 * \param vcc the VCC value in mV
			 * \return true if the VCC was set successfully, else false
			 */
			virtual bool setDeviceVcc (uint16_t vcc) = 0;

			/** \brief get the VCC of the target device
			 *
			 * \return the VCC value in mV
			 */
			 virtual uint16_t getDeviceVcc () const = 0;

			/** \brief get the external VCC of the target device
			 *
			 * \return the external VCC in mV
			 */
			virtual uint16_t getExternalVcc () const = 0;

			/** \brief JTAG speed */
			enum JTAG_4WIRE_SPEED
			{
				JTAG_4WIRE_SPEED_15_MHZ = 1,
				JTAG_4WIRE_SPEED_8_MHZ = 2,
				JTAG_4WIRE_SPEED_4_MHZ = 4,
				JTAG_4WIRE_SPEED_2_MHZ = 8,
				JTAG_4WIRE_SPEED_1_MHZ = 16,
				JTAG_4WIRE_SPEED_500_KHZ = 32,
				JTAG_4WIRE_SPEED_250_KHZ = 64,
				JTAG_4WIRE_SPEED_750_KHZ = 128

			};

			/** \brief VCC_SUPPLY opptions for eZ-FET */
			typedef enum VCC_SUPPLY
			{
				NO_SUPPLY = 0x04,
				ET_OFF = 0x05,
				ET_ON = 0x06,
				LDO_OFF = 0x07,
				ALL_OFF = 0x8,
				LDO_ON = 0x09
			} VCC_SUPPLY_T;

			enum JTAG_2WIRE_SPEED
			{
				JTAG_2WIRE_SPEED_1200_KHZ = 0x800A,
				JTAG_2WIRE_SPEED_600_KHZ = 0x600A,
				JTAG_2WIRE_SPEED_400_KHZ = 0x400A,
				JTAG_2WIRE_SPEED_200_KHZ = 0x200A,
				JTAG_2WIRE_SPEED_100_KHZ = 0x100A
			};

			/** \brief select a JTAG mode
			 *
			 * \param mode select the mode to use (default: JTAG_MODE_4WIRE)
			 */
			virtual void setJtagMode(INTERFACE_TYPE mode) = 0;
		     /** \brief select JTAG speed
			 *
			 * \param speed is the devicer for SPI moudule
			 */
			virtual bool setJtagSpeed(JTAG_4WIRE_SPEED speedJtag, JTAG_2WIRE_SPEED speedSbw) = 0;
		    /** \get Interface mode SBW2, SBW4 or JTAG
			 *
			 * \return selected JTAG protocol
			 */
			virtual INTERFACE_TYPE getInterfaceMode(TARGET_ARCHITECTURE_t arch) const = 0;
			/** \brief start JTAG control (without sync)
			 *
			 * \return true if JTAG was started successfully, else false
			 */
			virtual int16_t start() = 0;
			 /** \brief start JTAG control (without sync)
			 *
			 * \return true if JTAG was started successfully, else false
			 */
			virtual int16_t start(const std::string& pwd, uint32_t deviceCode) = 0;

			/** \brief stop JTAG control
			 *
			 * \param pwd start with password
			 * \param deviceCode start with deviceCode
			 * \return true if JTAG was stopped successfully, else false
			 */
			virtual bool stop () = 0;

			/** \brief reset the target device
			 *
			 * \param vcc turn VCC off and on
			 * \param pin pull the reset pin (in microSeconds)
			 * \return true if every step was successful, else false
			 */

			virtual bool reset(bool vcc, bool nmi, uint16_t JtagId, uint32_t rstHalId) = 0;

			/** \brief perform firmwareupdate
			 *
			 * \param fname defines the TI-txt file to be used for update or nullptr for internal image
			 * \param callback defines the callback for update messages or nullptr for no messages
			 * \param clientHandle reference given by the caller instance, returned in callback
			 * \return true on success
			 */
			virtual bool firmWareUpdate(const char * fname, UpdateNotifyCallback callback = 0, bool* coreUpdate = 0) = 0;

			/** \brief set device password
			 *
			 * \param pwd is the password for the device in hex notation, beginning with "0x"
			 */
			virtual void setPassword(const std::string& pwd) = 0;

			/** \brief set device code
			 *
			 * \param deviceCode for devices that need more specific identification (L092)
			 */
			virtual bool setDeviceCode(uint32_t deviceCode = 0 )= 0;

			/** \brief check if calibration before flash access is enabled
			 */
			virtual bool freqCalibrationEnabled() const = 0;

			virtual void init() = 0;

			virtual long MSP430I_MagicPattern(uint16_t ifMode) = 0;

			virtual bool isUpdateRequired(TARGET_ARCHITECTURE_t arch) const = 0;

			virtual bool jtagErase(uint16_t eraseKey) = 0;

			virtual void setEnergyTraceManager(EnergyTraceManager*) = 0;

			virtual bool isEnergyTraceSupported() = 0;

			virtual void setCurrentDrive(uint32_t value) = 0;

			virtual bool configureJtagSpeed(uint32_t speed = 0) = 0;

			virtual bool ulpDebugEnabled() const = 0;

			virtual bool atProbeStateEnabled() const = 0;

			virtual void setUlpDebug(bool ulp = false)= 0;

			virtual bool configureOverCurrent(bool state = false) = 0;

			virtual void setDisableInterruptsMode(uint32_t mode) = 0;

			virtual uint32_t getDisableInterruptsMode() const = 0;

			virtual bool updateDisableInterruptsMode() = 0;

			virtual bool setJTAGLock5xx(uint32_t value) = 0;
		};
	}
}
