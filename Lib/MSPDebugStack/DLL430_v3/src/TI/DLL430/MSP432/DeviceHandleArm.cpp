/*
 * DeviceHandleArm.cpp
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

#include <pch.h>
#include <MSP430.h>

#include "VersionInfo.h"
#include "DeviceHandleArm.h"
#include "DeviceDb/Database.h"
#include "DebugManagerArm.h"
#include "MemoryManager.h"
#include "HalExecCommand.h"
#include "FetControl.h"
#include "WatchdogControl.h"
#include "FetHandle.h"
#include "ClockCalibration.h"
#include "EM/EmulationManager/EmulationManager432.h"
#include "EM/SoftwareBreakpoints/ISoftwareBreakpoints.h"
#include "EM/SoftwareBreakpoints/SoftwareBreakpointManager.h"
#include "EM/EemRegisters/EemRegisterAccess432.h"
#include "EM/Exceptions/Exceptions.h"
#include "ArmRandomMemoryAccess.h"
#include "CpuRegisters.h"
#include "DeviceInfo.h"

#include "../../Bios/include/error_def.h"
#include "../../Bios/include/ConfigureParameters.h"

#include <iostream>

using namespace TI::DLL430;

static const uint32_t DAP_CHECK_LOCKED = 0x10; 
static const int32_t DAP_IS_LOCKED= -2;

DeviceHandleArm::DeviceHandleArm(FetHandle* parent, uint32_t deviceCode, INTERFACE_TYPE iMode)
 : parent(parent)
 , memoryManager(nullptr)
 , debugManager(nullptr)
 , clockCalibration(nullptr)
 , minFlashVcc(1800)
 , jtagId(0)
 , eemVersion(0)
 , mode(iMode)
 , mDeviceHasBeenIdentified(false)
{
	accessPortIds.clear();
}

DeviceHandleArm::~DeviceHandleArm()
{
	setEemRegisterAccess432(nullptr);
	SoftwareBreakpointManager::setMemoryAccessFunctions(nullptr, nullptr, nullptr);

	delete memoryManager;
	delete debugManager;
	delete clockCalibration;
}

EmulationManagerPtr DeviceHandleArm::getEmulationManager()
{
	if (!emulationManager)
		throw EM_NoEmulationManagerException();

	return this->emulationManager;
}

IMemoryManager* DeviceHandleArm::getMemoryManager()
{
	return this->memoryManager;
}

IDebugManager* DeviceHandleArm::getDebugManager()
{
	return this->debugManager;
}

long DeviceHandleArm::magicPatternSend(uint16_t ifMode)
{
	HalExecCommand writeMagicPattern;
	HalExecElement *eMagic = new HalExecElement(ID_UnlockDap);
	writeMagicPattern.elements.emplace_back(eMagic);

	if (!this->send(writeMagicPattern))
	{
		return -1;
	}				
	return 1;
}

INTERFACE_TYPE DeviceHandleArm::getInterfaceMode()
{
	return mode;
}

int32_t DeviceHandleArm::readCpuId()
{
	// Read the CPUID
	HalExecElement* el;

	el = new HalExecElement(ID_GetCpuIdArm);

	HalExecCommand readCpuIDCmd;
	readCpuIDCmd.elements.emplace_back(el);
	if (!this->send(readCpuIDCmd))
	{
		return -1;
	}

	IdCode idCode;
	idCode.version = readCpuIDCmd.elements[0]->getOutputAt32(0) & 0xFFFF;
	idCode.subversion = 0x0000;
	idCode.revision = readCpuIDCmd.elements[0]->getOutputAt32(4) & 0xFFFF;
	idCode.maxRevision = 0xFF;
	idCode.activationKey = 0;

	if (idCode.version == 0)
	{
		return -1;
	}

	long devId = DeviceDb::Database().findDevice(idCode);
	if (devId > 0)
	{
		mDeviceHasBeenIdentified = true;
		setDeviceId(devId);
		this->debugManager->stop();
	}
	return devId;
}

int32_t DeviceHandleArm::identifyDevice(uint32_t activationKey, bool afterMagicPattern)
{
	sendDeviceConfiguration(CONFIG_PARAM_CLK_CONTROL_TYPE, 0);
	sendDeviceConfiguration(CONFIG_PARAM_SFLLDEH, 0);
	sendDeviceConfiguration(CONFIG_PARAM_DEFAULT_CLK_CONTROL, 0);
	sendDeviceConfiguration(CONFIG_PARAM_ENHANCED_PSA, 0);
	sendDeviceConfiguration(CONFIG_PARAM_PSA_TCKL_HIGH, 0);
	sendDeviceConfiguration(CONFIG_PARAM_POWER_TESTREG_MASK, 0x0000);
	sendDeviceConfiguration(CONFIG_PARAM_POWER_TESTREG3V_MASK, 0x0000);
	sendDeviceConfiguration(CONFIG_PARAM_POWER_TESTREG_DEFAULT, 0);
	sendDeviceConfiguration(CONFIG_PARAM_POWER_TESTREG3V_DEFAULT, 0);
	sendDeviceConfiguration(CONFIG_ALT_ROM_ADDR_FOR_CPU_READ, 0);
	sendDeviceConfiguration(CONFIG_ASSERT_BSL_VALID_BIT,0);
	sendDeviceConfiguration(CONFIG_WDT_ADDRESS_5XX, 0);
	sendDeviceConfiguration(CONFIG_PARAM_INTERRUPT_OPTIONS, DISABLE_INTERRUPTS_NONE);
	sendDeviceConfiguration(CONFIG_PARAM_ULP_MSP432, 0);

	HalExecCommand cmd;
	HalExecElement* el = new HalExecElement(ID_ScanApArm);

	cmd.elements.emplace_back(el);
	cmd.setTimeout(5000);

	if (!this->send(cmd))
	{
		return -1;
	}

	if (isJtagFuseBlown())
	{
		return DAP_IS_LOCKED;
	}
	
	AccessPort temp;
	accessPortIds.clear();

	temp.idr = cmd.elements[0]->getOutputAt32(accessPortIds.size() * 13);
	if (temp.idr)
	{
		temp.base = cmd.elements[0]->getOutputAt32(accessPortIds.size() * 13 + 4);
		temp.cfg = cmd.elements[0]->getOutputAt32(accessPortIds.size() * 13 + 8);
		temp.portNum = cmd.elements[0]->getOutputAt8(accessPortIds.size() * 13 + 12);
		if (temp.validate())
		{
			// Discover all the components in the AP
			parseAndAddComponent(static_cast<uint8_t>(accessPortIds.size()), temp.components, temp.base & 0xFFFFFFFC, &temp.pid);
			accessPortIds.push_back(temp);
		}
	}

	// No APs/Components found
	if (!accessPortIds.size() || accessPortIds[0].components.size() < 2)
	{
		return -1;
	}
	
	int32_t devId = readCpuId();
	if(!devId)
	{
		return -1;		
	}
	return devId;
}

bool DeviceHandleArm::reset(bool hardReset)
{
	HalExecElement *el = new HalExecElement(ID_ResetArm);
	el->appendInputData16(hardReset ? 1 : 0); // RsetType: 0 = Core, 1 = System

	HalExecCommand resetCmd;
	resetCmd.elements.emplace_back(el);

	if (!this->send(resetCmd))
	{
		return false;
	}

	if (!this->memoryManager)
	{
		return false;
	}
	CpuRegisters *cpu = this->memoryManager->getCpuRegisters();
	if(cpu)
	{
		cpu->fillCache(0,18);
	}
	return true;
}

int32_t DeviceHandleArm::setDeviceId(long id)
{
	if (mDeviceHasBeenIdentified)
	{
		// Enable debug
		HalExecCommand enableDebugCmd;
		enableDebugCmd.elements.emplace_back(new HalExecElement(ID_EnableDebugArm));
		this->send(enableDebugCmd);

		const DeviceInfo* devInfo = DeviceDb::Database().getDeviceInfo(id);
		if (devInfo)
		{
			this->configure(*devInfo);
			return 0;
		}
		return -1;
	}
	else
	{
		return this->identifyDevice(0, true);
	}
}

void DeviceHandleArm::setWatchdogControl(std::shared_ptr<WatchdogControl> ctrl)
{
	this->wdt = ctrl;
}

std::shared_ptr<WatchdogControl> DeviceHandleArm::getWatchdogControl() const
{
	return this->wdt;
}


void DeviceHandleArm::configure(const DeviceInfo& info)
{
	setEemRegisterAccess432(nullptr);

	deviceInfo = info;

	delete this->memoryManager;
	this->memoryManager = new MemoryManager(this, info);

	setEemRegisterAccess432( dynamic_cast<ArmRandomMemoryAccess*>(memoryManager->getMemoryArea(MemoryArea::Eem)) );

	delete this->debugManager;
	this->debugManager = new DebugManagerArm(this, info);

	this->emulationManager = EmulationManager432::create(info.eem);

	//this->funcletTable = info.funcletMap;
	this->minFlashVcc = info.voltageInfo.vccFlashMin; 
}

bool DeviceHandleArm::restoreTinyRam()
{
	return true;
}

bool DeviceHandleArm::sendDeviceConfiguration(uint32_t parameter, uint32_t value)
{
	HalExecElement* el = new HalExecElement(ID_Configure);
	el->appendInputData32(parameter);
	el->appendInputData32(value);

	HalExecCommand configCmd;
	configCmd.elements.emplace_back(el);

	return this->send(configCmd);
}

uint32_t DeviceHandleArm::getDeviceCode() const
{
	return 0;
}

uint32_t DeviceHandleArm::getJtagId()
{
	return this->jtagId;
}

uint32_t DeviceHandleArm::getDeviceIdPtr()
{
	return 0;
}

uint32_t DeviceHandleArm::getEemVersion()
{
	return 0;
}

uint32_t DeviceHandleArm::readJtagId()
{
	HalExecCommand cmd;
	HalExecElement* el = new HalExecElement(ID_GetJtagIdCodeArm);

	cmd.elements.emplace_back(el);
	if (!this->send(cmd))
	{
		return 0;
	}

	this->jtagId = cmd.elements[0]->getOutputAt32(0);

	return this->jtagId;
};

bool DeviceHandleArm::isJtagFuseBlown()
{
	HalExecCommand readlock;
	HalExecElement* lock = new HalExecElement(ID_CheckDapLockArm);
	readlock.elements.emplace_back(lock);

	if (!this->send(readlock))
	{
		return false;
	}

	uint32_t lockVal = readlock.elements[0]->getOutputAt32(0);

	if (lockVal & DAP_CHECK_LOCKED)
	{
		return true;
	}
	return false;
}

bool DeviceHandleArm::secure()
{
	bool success = false, successErase = false;
	
	const FuncletCode& funclet = this->getFunclet(FuncletCode::Type::SECURE);

	const uint8_t* code = (uint8_t*)funclet.codeNoOffset();
	const size_t count = funclet.codeSize();

	if (count > 0x10000)
	{
		return false;
	}

	MemoryArea* info = memoryManager->getMemoryArea(MemoryArea::Info, 0);

	successErase = info->erase(info->getStart(), info->getStart() + static_cast<uint32_t>(count));
	success = info->write(0, code, count);

	return success && successErase;
}

FetControl* DeviceHandleArm::getControl()
{
	return parent->getControl();
}

bool DeviceHandleArm::send(HalExecCommand &command)
{
	return this->parent->getControl()->send(command);
}

hal_id DeviceHandleArm::checkHalId(hal_id base_id) const
{
	const auto it = deviceInfo.functionMap.find(base_id);
	return (it != deviceInfo.functionMap.end()) ? it->second : base_id;
}

const FuncletCode& DeviceHandleArm::getFunclet(FuncletCode::Type funclet)
{
	static FuncletCode dummy;
	const auto it = deviceInfo.funcletMap.find(funclet);
	return (it != deviceInfo.funcletMap.end()) ? it->second : dummy;
}

bool DeviceHandleArm::supportsQuickMemRead() const
{
	return false;
}

uint16_t DeviceHandleArm::getMinFlashVcc() const
{
	return minFlashVcc;
}

bool DeviceHandleArm::hasFram() const
{
	return false;
}

bool DeviceHandleArm::hasLPMx5() const
{
	return false;
}

void DeviceHandleArm::disableHaltOnWakeup()
{
	// Disable Debug
	HalExecCommand cmd;
	cmd.elements.emplace_back(new HalExecElement(ID_DisableDebugArm));
	this->send(cmd);
}

bool DeviceHandleArm::eemAccessibleInLpm() const
{
	return false;
}

bool DeviceHandleArm::deviceSupportsEnergyTrace() const
{
	return true;
}

TARGET_ARCHITECTURE_t DeviceHandleArm::getTargetArchitecture()const
{
	return TARGET_ARCHITECTURE_t::MSP432_M4;
}

bool DeviceHandleArm::parseAndAddComponent(uint8_t portNum, std::vector<ComponentPeripheral> &vec, uint32_t baseAddress, uint64_t *ROMpid)
{
	HalExecElement *el;
	if (mode == SWD_MSP432)
	{
		el = new HalExecElement(ID_MemApTransactionArmSwd);
	}
	else
	{
		el = new HalExecElement(ID_MemApTransactionArm);
	}


	el->appendInputData16(portNum);                  // APSEL
	el->appendInputData16(1);                        // rnw = READ
	el->appendInputData16(2);                       // dataWidth = 32
	el->appendInputData32(baseAddress + PID_OFFSET);// address
	el->appendInputData32(12 * 4);                  // size in bytes

	HalExecCommand readPidCidCmd;
	readPidCidCmd.elements.emplace_back(el);
	readPidCidCmd.setTimeout(5000);

	if (!this->send(readPidCidCmd))
	{
		return false;
	}

	uint32_t PID[] = {readPidCidCmd.elements[0]->getOutputAt32(0x10),
					  readPidCidCmd.elements[0]->getOutputAt32(0x14),
					  readPidCidCmd.elements[0]->getOutputAt32(0x18),
					  readPidCidCmd.elements[0]->getOutputAt32(0x1C),
					  readPidCidCmd.elements[0]->getOutputAt32(0x00),
					  readPidCidCmd.elements[0]->getOutputAt32(0x04),
					  readPidCidCmd.elements[0]->getOutputAt32(0x08),
					  readPidCidCmd.elements[0]->getOutputAt32(0x0C)};

	uint32_t CID[] = {readPidCidCmd.elements[0]->getOutputAt32(0x20),
					  readPidCidCmd.elements[0]->getOutputAt32(0x24),
					  readPidCidCmd.elements[0]->getOutputAt32(0x28),
					  readPidCidCmd.elements[0]->getOutputAt32(0x2C)};

	ComponentPeripheral component(baseAddress, PID, CID);

	if (component.getComponentType() != COMPONENT_UNKNOWN)
	{
		vec.push_back(component);
	}
	if (component.getComponentType() == COMPONENT_ROM_TABLE)
	{
		*ROMpid = (((uint64_t)PID[7] & 0xff) << 56) |
			(((uint64_t)PID[6] & 0xff) << 48) |
			(((uint64_t)PID[5] & 0xff) << 40) |
			(((uint64_t)PID[4] & 0xff) << 32) |
			(((uint64_t)PID[3] & 0xff) << 24) |
			(((uint64_t)PID[2] & 0xff) << 16) |
			(((uint64_t)PID[1] & 0xff) << 8) |
			(((uint64_t)PID[0] & 0xff));

		uint32_t tableOffset = 0;
		uint32_t ROMEntry;
		do
		{
			HalExecElement *el;
			if (mode == SWD_MSP432)
			{
				el = new HalExecElement(ID_MemApTransactionArmSwd);
			}
			else
			{
				el = new HalExecElement(ID_MemApTransactionArm);
			}


			el->appendInputData16(portNum);                   // APSEL
			el->appendInputData16(1);                         // rnw = READ
			el->appendInputData16(2);                         // dataWidth = 32
			el->appendInputData32(baseAddress + tableOffset); // address
			el->appendInputData32(4);                         // size in bytes

			HalExecCommand readRomEntry;
			readRomEntry.elements.emplace_back(el);
			readRomEntry.setTimeout(5000);
			if (!this->send(readRomEntry))
			{
				return false;
			}
			ROMEntry = readRomEntry.elements[0]->getOutputAt32(0);
			if (ROMEntry & 0x1)
			{
				parseAndAddComponent(portNum, vec, baseAddress + (ROMEntry & 0xFFFFF000), ROMpid);
			}
			tableOffset += 4;
		} while (ROMEntry);
	}

	switch (component.getComponentType())
	{
	case COMPONENT_SCS:
		// Configure the base address for the SCS
		sendDeviceConfiguration(CONFIG_PARAM_SCS_BASE_ADDRESS, component.getBase());
		break;

	case COMPONENT_FPB:
		// Configure the base address for the FPB
		sendDeviceConfiguration(CONFIG_PARAM_FPB_BASE_ADDRESS, component.getBase());
		break;

	default:
		// Do nothing
		break;
	}

	return true;
}
