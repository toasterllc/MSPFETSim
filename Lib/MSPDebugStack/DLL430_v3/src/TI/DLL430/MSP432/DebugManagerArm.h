/*
 * DebugManagerArm.h
 *
 * Provides routines for handling a debug session.
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

#include "IDebugManager.h"
#include <stdint.h>

namespace TI
{
	namespace DLL430
	{
		class IDeviceHandle;
		struct DeviceInfo;

		/** \brief manage debug actions on the target device */
		class DebugManagerArm : public IDebugManager
		{
		public:
			DebugManagerArm(IDeviceHandle*, const DeviceInfo&);
			~DebugManagerArm();

			DebugManagerArm(const DebugManagerArm&) = delete;
			DebugManagerArm& operator=(const DebugManagerArm&) = delete;

			/** \brief reestablish JTAG connection after releasing it
			 * Function will reconnect JTAG and resume any polling
			 * \return true if run started successfully, else false
			 */
			bool reconnectJTAG();

			/** \brief release JTAG control to let the device execute
			 *
			 * The developper must not assume any amount of time until the JTAG pins are
			 * released.
			 *
			 * \param controlType indicates wether waitForEem is activated
			 * \param target call the event method of target (only valid if toBreakpoint is true)
			 * \param eventType bitmask for type of event to react on
			 * \return true if run started successfully, else false
			 */
			bool run (uint16_t controlType, DebugEventTarget* target = 0, bool releaseJTAG=false);

			/** \brief stop the device
			 *
			 * If a command is running in the FET, this kills it if possible. It also
			 * starts and syncs JTAG.
			 *
			 * \return true if stopped successfully, else false
			 */
			bool stop (bool jtagWasReleased = false);

			/** \brief do a single step
			 *
			 * A SingleStep is performed on the target device. When this function returns,
			 * the device is already stopped. Prior to using this function, the PC must be
			 * set the CPU memory area. The updated value can also be read from there.
			 *
			 * \param cycles the number of device target CPU cycles of the instruction
			 * \return true on successful action, else false
			 */
			bool singleStep (uint32_t* cycles = 0);

			/** \brief return the clock control level of device
			 *
			 * the returned value depends on the identification of the device
			 * and the database
			 *
			 * \return level of clock control (1-3)
			 */
			uint8_t getClockControl() const;

			/** \brief return the general clock control setting
			 *
			 * return the 16Bit control value
			 *
			 * \return control value
			 */
			uint16_t getClockControlSetting() const;

			/** \brief set the general clock control setting
			 *
			 * set the 16Bit control value
			 */
			void setClockControlSetting(uint16_t clkcntrl);

			/** \brief return the default generals clock control setting
			 *
			 * return the 16Bit control value
			 *
			 * \return control value
			 */
			uint16_t getGeneralClockDefaultSetting() const;

			/** \brief return the default clock control module setting
			 *
			 * return the 16Bit control value
			 *
			 * \return control value
			 */
			uint32_t getClockModuleDefaultSetting() const;

			/** \brief return the clock control module setting
			 *
			 * return the 16Bit control value
			 *
			 * \return control value
			 */
			uint32_t getClockModuleSetting() const;

			/** \brief set the clock control module setting
			 *
			 * set the 32Bit control value
			 */
			void setClockModuleSetting(uint32_t modules);

			char** getModuleStrings(uint32_t * n) const;

			char** getClockStrings(uint32_t * n) const;

			/** \brief init the EEM-Register
			 *
			 * depending on the target hardware, the existing EEM-register are
			 * set to default (0)
			 *
			 * \return true if data is successfully written to device, else false
			 */
			bool initEemRegister();

			/** \brief Enable eem polling loop
			 *
			 * \param mask of events to poll for
			 *
			 * \return true if successfull, otherwise false
			 */
			bool activatePolling(uint16_t);

			/** \brief Enable polling of JState register
			 *
			 * \param cb Callback target to handle jstate changes
			 *
			 * \return true if successfull, otherwise false
			 */
			bool activateJStatePolling(DebugEventTarget* cb);

			/** \brief Return current low power mode x.5 state
			 *
			 * Returns the last reported state without actively querying
			 *
			 * \return true if device is in LPMx.5
			 */
			bool queryIsInLpm5State();

			/** \brief set the opcode parameter
			 *
			 * needed for ID_RestoreContext_ReleaseJtag and set by SET_MDB_BEFORE_RUN
			 *
			 * \param value opcode to be used by ID_RestoreContext_ReleaseJtag
			 *
			 * \return true if successfull, otherwise false
			 */
			void setOpcode(uint16_t value);

			/** \brief call macro ID_SyncJtag_Conditional_SaveContext
			 *
			 * calles the macro ID_SyncJtag_Conditional_SaveContext and saves
			 * the returned register values
			 *
			 * \return true on success
			 */
			bool saveContext();

			/** \brief enable/disable low power mode debugging
			 *
			 * synchronize the call and response of a loop command
			 */
			void setLpmDebugging(bool enable);

			/** \brief Check for low power mode debugging
			 *
			 * return true if power mode debugging is enabled
			 */
			bool getLpmDebugging();

			/** \brief Pause all polling loops
			 *
			 *
			 */
			void pausePolling();

			/** \brief Resume all polling loops
			 *
			 *
			 */
			void resumePolling();

			/** \brief sync JTAG if an externl wakeup event happends
			 *
			 *
			 */
			bool syncDeviceAfterLPMx5();

			/** \brief Retrieve current cycle counter value
			 *
			 *
			 */
			uint64_t getCycleCounterValue();

			/** \brief Reset cycle counter value
			 *
			 *
			 */
			void resetCycleCounterValue();

			/** \brief Start polling loop for state storage events on UIF
			 *
			 *
			 */
			bool startStoragePolling();

			/** \brief Stop polling loop for state storage events on UIF
			 *
			 *
			 */
			bool stopStoragePolling();

			void setPollingManager(PollingManager* pollingManager);

			void enableLegacyCycleCounter(bool enable);
			bool legacyCycleCounterEnabled() const;

			bool setPCtoSafeLocation(){ return false; }

		private:
			IDeviceHandle *mDeviceHandle;
			PollingManager* mPollingManager;
			DebugEventTarget* mCallback;
			bool ulpDebug;

			void runEvent(MessageDataPtr messageData);
		};

	}
}
