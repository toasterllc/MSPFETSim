/*
 * DeviceHandleArm.h
 *
 * Communication with target device.
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

#include "IDeviceHandle.h"
#include "MemoryManager.h"
#include "DebugManagerArm.h"
#include "DeviceInfo.h"
#include "WatchdogControl.h"
#include "EM/EmulationManager/EmulationManager430.h"
#include "EM/Exceptions/Exceptions.h"
#include "EemMemoryAccess.h"

namespace TI
{
	namespace DLL430
	{
		class FetHandle;
		struct DeviceInfo;
		class FetControl;
		class HalCommand;
		class ClockCalibration;

		static const uint32_t MEMTYPE_OFFSET = 0x00000FCC;
		static const uint32_t PID_OFFSET     = 0x00000FD0;
		static const uint32_t CID_OFFSET     = 0x00000FF0;
		static const uint32_t CPUID_OFFSET   = 0x00000D00;

		/// \brief Defines the peripheral class
		typedef enum
		{
			COMPONENT_ROM_TABLE,
			COMPONENT_SCS,
			COMPONENT_SCS_WITH_FPU,
			COMPONENT_DWT,
			COMPONENT_FPB,
			COMPONENT_ITM,
			COMPONENT_TPIU,
			COMPONENT_ETM,
			COMPONENT_UNKNOWN
		} ComponentType;

		/// \brief Component/Peripheral contained insied the MEM-AP
		class ComponentPeripheral
		{
		public:
			ComponentPeripheral(uint32_t base, uint32_t PID[8], uint32_t CID[4])
			{
				m_base = base;

				if ((CID[0] == 0x0000000D) && // Coresight ROM Table
				   (CID[1] == 0x00000010) &&
				   (CID[2] == 0x00000005) &&
				   (CID[3] == 0x000000B1))
				{
					m_componentType = COMPONENT_ROM_TABLE;
				}
				else if ((PID[4] == 0x00000004) &&  // Coresight SCS
				        (PID[0] == 0x00000000 || PID[0] == 0x0000000C) &&
				        (PID[1] == 0x000000B0) &&
				        (PID[2] == 0x0000000B) &&
				        (PID[3] == 0x00000000) &&
				        (CID[0] == 0x0000000D) &&
				        (CID[1] == 0x000000E0) &&
				        (CID[2] == 0x00000005) &&
				        (CID[3] == 0x000000B1))
				{
					m_componentType = PID[0] ? COMPONENT_SCS_WITH_FPU : COMPONENT_SCS;
				}
				else if ((PID[4] == 0x00000004) && // Coresight DWT
						(PID[5] == 0x00000000) &&
						(PID[6] == 0x00000000) &&
						(PID[7] == 0x00000000) &&
						(PID[0] == 0x00000002) &&
						(PID[1] == 0x000000B0) &&
						(PID[2] == 0x0000003B) &&
						(PID[3] == 0x00000000) &&
						(CID[0] == 0x0000000D) &&
						(CID[1] == 0x000000E0) &&
						(CID[2] == 0x00000005) &&
						(CID[3] == 0x000000B1))
				{
					m_componentType = COMPONENT_DWT;
				}
				else if ((PID[4] == 0x00000004) && // Coresight FPB
                        (PID[5] == 0x00000000) &&
                        (PID[6] == 0x00000000) &&
                        (PID[7] == 0x00000000) &&
                        (PID[0] == 0x00000003) &&
                        (PID[1] == 0x000000B0) &&
                        (PID[2] == 0x0000002B) &&
                        (PID[3] == 0x00000000) &&
                        (CID[0] == 0x0000000D) &&
                        (CID[1] == 0x000000E0) &&
                        (CID[2] == 0x00000005) &&
                        (CID[3] == 0x000000B1))
				{
					m_componentType = COMPONENT_FPB;
				}
				else if ((PID[4] == 0x00000004) && // Coresight ITM
                        (PID[5] == 0x00000000) &&
                        (PID[6] == 0x00000000) &&
                        (PID[7] == 0x00000000) &&
                        (PID[0] == 0x00000001) &&
                        (PID[1] == 0x000000B0) &&
                        (PID[2] == 0x0000003B) &&
                        (PID[3] == 0x00000000) &&
                        (CID[0] == 0x0000000D) &&
                        (CID[1] == 0x000000E0) &&
                        (CID[2] == 0x00000005) &&
                        (CID[3] == 0x000000B1))
				{
					m_componentType = COMPONENT_ITM;
				}
				else
				{
					m_componentType = COMPONENT_UNKNOWN;
				}
			};
			~ComponentPeripheral() {};

			uint32_t getBase() const {return m_base;};
			ComponentType getComponentType() const {return m_componentType;};

		private:
			ComponentType m_componentType;
			uint32_t m_base;
		};

		/// \brief Access Port Identification structure
		typedef struct
		{
			uint8_t  portNum; // Access port number
			uint32_t idr;     // Identification register
			uint32_t base;    // Base address register
			uint32_t cfg;     // Configuration register
			uint64_t pid;     // PID of the ROM Table is used for device identification
			std::vector<ComponentPeripheral> components;

			/// \brief Validate the values
			// Only little-endian MEM-APs are supported
			bool validate() const {return !cfg && (base & 0x3) && ((idr & 0x0001FF00) == 0x00010000);};
		} AccessPort;

		class DeviceHandleArm : public IDeviceHandle
		{
		public:
			DeviceHandleArm(FetHandle*, uint32_t deviceCode, INTERFACE_TYPE iMode);
			~DeviceHandleArm();

			DeviceHandleArm(const DeviceHandleArm&) = delete;
			DeviceHandleArm& operator=(const DeviceHandleArm&) = delete;

			EmulationManagerPtr getEmulationManager();
			IMemoryManager* getMemoryManager ();
			IDebugManager* getDebugManager ();
			FetHandle* getFetHandle () { return parent; }
			ClockCalibration* getClockCalibration() { return clockCalibration; }

			/// \brief Run the boot code writing the the specified command in the mailbox first
			/// \param command[in] Command to be put in the mailbox
			/// \return True if bootcode execution succeeded
			long magicPatternSend(uint16_t ifMode);
			int32_t identifyDevice (uint32_t activationKey, bool afterMagicPattern);
			bool secure ();
			bool reset (bool hardReset = false);

			FetControl* getControl ();

			bool send (HalExecCommand &command);

			uint32_t readJtagId();
			uint32_t getJtagId();
			uint32_t getDeviceIdPtr();
			uint32_t getEemVersion();
			bool isJtagFuseBlown();
			uint32_t getDeviceCode() const;
			INTERFACE_TYPE getInterfaceMode();

			const std::string & getDescription() const { return deviceInfo.description; }

			int32_t setDeviceId (long id);

			hal_id checkHalId(hal_id base_id) const;

			const FuncletCode& getFunclet(FuncletCode::Type funclet);

			bool supportsQuickMemRead() const;
			uint16_t getMinFlashVcc() const;
			bool hasFram() const;
			bool hasLPMx5() const;

			void disableHaltOnWakeup();

			bool eemAccessibleInLpm() const;

			bool deviceSupportsEnergyTrace() const;

			TARGET_ARCHITECTURE_t getTargetArchitecture() const;

			void setWatchdogControl(std::shared_ptr<WatchdogControl>);
			std::shared_ptr<WatchdogControl> getWatchdogControl() const;

			bool restoreTinyRam();

		private:
			FetHandle* parent;
			EmulationManagerPtr emulationManager;
			IMemoryManager* memoryManager;
			DebugManagerArm* debugManager;
			ClockCalibration* clockCalibration;

			uint16_t minFlashVcc;

			uint32_t jtagId;
			DeviceInfo deviceInfo;
			uint32_t eemVersion;
			INTERFACE_TYPE mode;
			
			std::vector<AccessPort> accessPortIds;

			std::shared_ptr<WatchdogControl> wdt;

			void configure (const DeviceInfo& info);
			bool sendDeviceConfiguration(uint32_t parameter, uint32_t value);
			bool parseAndAddComponent(uint8_t portNum, std::vector<ComponentPeripheral> &vec, uint32_t baseAddress, uint64_t *ROMpid);
			bool mDeviceHasBeenIdentified;
			int32_t readCpuId();
			int32_t isDaplocked();

		};
	}
}
