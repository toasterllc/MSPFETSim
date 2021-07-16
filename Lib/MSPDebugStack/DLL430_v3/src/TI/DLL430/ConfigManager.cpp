/*
 * ConfigManager.cpp
 *
 * Functionality for configuring target device.
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
#include "VersionInfo.h"
#include "FetHandle.h"
#include "ConfigManager.h"
#include "HalExecCommand.h"
#include "FetControl.h"
#include "WatchdogControl.h"
#include "Record.h"
#include "PinSequence.h"
#include "IDeviceHandle.h"
#include "UpdateManagerFet.h"
#include "UpdateManagerMSP_FET430.h"
#include "UpdateManagerDummy.h"
#include "FetHandleManager.h"
#include "ConfigureParameters.h"
#include "EnergyTrace_TSPA/EnergyTraceManager.h"
#include "JtagId.h"

using namespace TI::DLL430;
using namespace std;

ConfigManager::ConfigManager(FetHandle* parent, FetHandleManager* fhManager, TARGET_ARCHITECTURE_t arch)
	: parent(parent)
	, updateManagerFet(nullptr)
	, vcc(0)
	, mode(JTAG_IF)
	, mEnergyTraceManager(nullptr)
	, deviceCode(0)
	, mhighres(0)
	, freqCalibration(true)
	, ulpDebug(false)
	, atProbeState(false)
	, arch(arch)
	, disableInterruptsMode(DISABLE_INTERRUPTS_NONE)
	, disableInterruptsModeBackup(DISABLE_INTERRUPTS_NONE)
{
	updateCmd.setTimeout(20000); 

	FetControl* control = this->parent->getControl();
	if (control->getFetToolId()== eZ_FET_WITH_DCDC || control->getFetToolId() == eZ_FET_NO_DCDC
		|| control->getFetToolId() == MSP_FET_WITH_DCDC || control->getFetToolId() == eZ_FET_WITH_DCDC_NO_FLOWCT
		|| control->getFetToolId() == MSP_FET_WITH_DCDC_V2x || control->getFetToolId() == eZ_FET_WITH_DCDC_V2x)
	{
		this->updateManagerFet = new UpdateManagerFet(parent, this, fhManager);
	}
	else if (control->getFetToolId()== MSP_FET430)
	{
		this->updateManagerFet = new UpdateManagerMSP_FET430(parent, this,arch);
	}
	else
	{
		this->updateManagerFet = new UpdateManagerDummy();
	}
}

ConfigManager::~ConfigManager()
{
	FetControl* control=this->parent->getControl();

	// send reset command to FET
	if ((control != nullptr)&&(control->hasCommunication()))
	{
		this->stop();
	}
	delete updateManagerFet;
}

bool ConfigManager::isUpdateRequired(TARGET_ARCHITECTURE_t arch) const
{
	return updateManagerFet->isUpdateRequired(arch);
}

bool ConfigManager::isEnergyTraceSupported()
{
	FetControl* fet = this->parent->getControl();
	const uint16_t toolId = fet ? fet->getFetToolId() : 0;
	return toolId == eZ_FET_WITH_DCDC || toolId == MSP_FET_WITH_DCDC || toolId == eZ_FET_WITH_DCDC_NO_FLOWCT || toolId == MSP_FET_WITH_DCDC_V2x || toolId == eZ_FET_WITH_DCDC_V2x;
}

void ConfigManager::init ()
{
	string tag;
	string value;

	// If any kind of update is required do not configure any JTAG or SBW Speed-> perhaps you will use a null pointer
	if (updateManagerFet->isUpdateRequired(TARGET_ARCHITECTURE::MSP430))
	{
		return;
	}
	// read configuration for JTAG speed EDT trace....
	JTAG_4WIRE_SPEED jtagSpeed = JTAG_4WIRE_SPEED_4_MHZ;
	JTAG_2WIRE_SPEED sbwSpeed  = JTAG_2WIRE_SPEED_400_KHZ;

	string iniFile = "MSP430DLL.INI";

	if (const char* iniPathEnv = getenv("MSP430_DLL_INI_PATH"))
	{
		iniFile = string(iniPathEnv) + "/" + iniFile;
	}

	ifstream DllV3Ini(iniFile.c_str());

	while (DllV3Ini && !DllV3Ini.eof())
	{
		DllV3Ini >> tag >> value;
		if (tag == "SBW_SPEED")
		{
			if (value == "JTAG_2WIRE_SPEED_600_KHZ")
			{
				sbwSpeed = JTAG_2WIRE_SPEED_600_KHZ;
			}
			if (value == "JTAG_2WIRE_SPEED_400_KHZ")
			{
				sbwSpeed = JTAG_2WIRE_SPEED_400_KHZ;
			}
			if (value == "JTAG_2WIRE_SPEED_200_KHZ")
			{
				sbwSpeed = JTAG_2WIRE_SPEED_200_KHZ;
			}
			if (value == "JTAG_2WIRE_SPEED_100_KHZ")
			{
				sbwSpeed = JTAG_2WIRE_SPEED_100_KHZ;
			}
		}
		if (tag == "JTAG_SPEED")
		{
			if (value == "JTAG_4WIRE_SPEED_15_MHZ")
			{
				jtagSpeed = JTAG_4WIRE_SPEED_15_MHZ;
			}
			if (value == "JTAG_4WIRE_SPEED_8_MHZ")
			{
				jtagSpeed = JTAG_4WIRE_SPEED_8_MHZ;
			}
			if (value == "JTAG_4WIRE_SPEED_4_MHZ")
			{
				jtagSpeed = JTAG_4WIRE_SPEED_4_MHZ;
			}
			if (value == "JTAG_4WIRE_SPEED_2_MHZ")
			{
				jtagSpeed = JTAG_4WIRE_SPEED_2_MHZ;
			}
			if (value == "JTAG_4WIRE_SPEED_1_MHZ")
			{
				jtagSpeed = JTAG_4WIRE_SPEED_1_MHZ;
			}
			if (value == "JTAG_4WIRE_SPEED_750_KHZ")
			{
				jtagSpeed = JTAG_4WIRE_SPEED_750_KHZ;
			}
			if (value == "JTAG_4WIRE_SPEED_500_KHZ")
			{
				jtagSpeed = JTAG_4WIRE_SPEED_500_KHZ;
			}
			if (value == "JTAG_4WIRE_SPEED_250_KHZ")
			{
				jtagSpeed = JTAG_4WIRE_SPEED_250_KHZ;
			}
		}
		if (tag == "ULP_DEBUG")
		{
			ulpDebug = (value == "ON");
		}

		if (tag == "DCO_CALIBRATION")
		{
			freqCalibration = (value != "OFF");
		}

		if (tag == "AT_PROBE_STATE")
		{
			atProbeState = (value == "ON");
		}
	}
	this->setJtagSpeed(jtagSpeed, sbwSpeed);
}

VersionInfo ConfigManager::getHalVersion() const
{
	return this->updateManagerFet->getHalVersion();
}


void ConfigManager::setJtagMode(INTERFACE_TYPE mode)
{
	this->mode = mode;
}

int16_t ConfigManager::start()
{
	return this->start(password, deviceCode);
}

bool ConfigManager::jtagErase(uint16_t eraseKey)
{
	if (this->start() != 0x1)
	{
		return false;
	}
	HalExecCommand cmd;
	cmd.setTimeout(10000);
	HalExecElement* el = new HalExecElement(ID_SendJtagMailboxXv2);
	el->appendInputData16(LONG_MAILBOX_MODE);
	el->appendInputData16(STOP_DEVICE);
	el->appendInputData16(eraseKey);
	cmd.elements.emplace_back(el);

	if (!this->parent->send(cmd))
	{
		return false;
	}

	// assert hard RST/NMI and feed in magic pattern to stop device execution
	// thr RST/NMI will remove the register protection
	if (!this->reset(false, true, 0x99, ID_ResetXv2))
	{
		return false;
	}
	// restart jtag connection and if needed feed in JTAG passowrd
	if (this->start() != 0x1)
	{
		return false;
	}
#if defined(_WIN32) || defined(_WIN64) ||  defined(__APPLE__)
	this_thread::sleep_for(chrono::milliseconds(100));
#else
	this_thread::sleep_for(chrono::milliseconds(500));
#endif
	return true;
}

uint16_t AsciiToHex(const char* password)
{
	return strtoul(std::string(password, 4).c_str(), 0, 16) & 0xFFFF;
}

INTERFACE_TYPE ConfigManager::getInterfaceMode(TARGET_ARCHITECTURE_t arch) const
{
	HalExecCommand cmd;
	HalExecElement* el;// = new HalExecElement(ID_GetInterfaceMode);
	if (arch == TARGET_ARCHITECTURE_t::MSP432_M4)
	{
		el = new HalExecElement(ID_GetInterfaceModeArm);
	}
	else
	{
		el = new HalExecElement(ID_GetInterfaceMode);
	}
	
	cmd.elements.emplace_back(el);

	if (this->parent->send(cmd))
	{
		const uint16_t jtagID = el->getOutputAt16(0);
		const uint16_t ifMode = el->getOutputAt16(2);

		if (jtagID != 0xFFFF)
		{
			switch (ifMode)
			{
			case 0: // JTAG Mode
				return JTAG_IF;
			case 1: // SBW2 Mode
				return SPYBIWIRE_IF;
			case 2: // SBW4 Mode
				return SPYBIWIREJTAG_IF;
			case 7: // JTAG_432 Mode
				return JTAG_MSP432;
			case 8: // SWD_432 Mode
				return SWD_MSP432;
			}
		}
	}
	return UNDEF_IF; //JTAG_MODE_UNDEF; //ToDo
}

int16_t ConfigManager::start(const string& pwd, uint32_t deviceCode)
{
	//#words in hex, ie. #characters / 4
	const uint16_t pwLength = (uint16_t)pwd.length() / 4;

	// if we have an L092 Rom device
	if (deviceCode == 0xDEADBABE)
	{
		if (pwLength > 4)
		{
			return -2;
		}

		HalExecElement* elUnlock = new HalExecElement(ID_UnlockC092);
		elUnlock->appendInputData16(pwLength);

		const char* pwdinternal = pwd.c_str();
		for (uint16_t i = 0; i < pwLength; i++)
		{
			uint16_t hexWord = AsciiToHex(pwdinternal);
			elUnlock->appendInputData16(hexWord);
			pwdinternal += 4;
		}

		HalExecCommand cmd;
		cmd.elements.emplace_back(elUnlock);

		if (!this->parent->send(cmd))
		{
			return -2;
		}
		return 1;
	}
	// if we have an L092 device
	if (deviceCode == 0xA55AA55A|| deviceCode == 0x5AA55AA5)
	{
		HalExecElement* elActivation = new HalExecElement(ID_StartJtagActivationCode);

		elActivation->appendInputData8(0);
		elActivation->appendInputData8(0);
		elActivation->appendInputData32(deviceCode);

		HalExecCommand cmd;
		cmd.elements.emplace_back(elActivation);
		cmd.setTimeout(10000);

		if (!this->parent->send(cmd))
		{
			return -2;
		}
		return 1;
	}
	// if we have a device locked with a custom password
	if ( !pwd.empty() )
	{
		if (pwLength > 60)
		{
			return 0;
		}

		HalExecElement* elUnlock = new HalExecElement(ID_UnlockDeviceXv2);
		switch (this->mode)
		{
			case JTAG_IF:
				elUnlock->appendInputData16(0);
				break;
			case SPYBIWIRE_IF:
				elUnlock->appendInputData16(1);
				break;
				case SPYBIWIREJTAG_IF:
				elUnlock->appendInputData16(2);
				break;
			default:
				delete elUnlock;
				return 0;
		}

		elUnlock->appendInputData16(pwLength);

		const char* pwdinternal = pwd.c_str();
		for (uint16_t i = 0; i < pwLength; i++)
		{
			const uint16_t hexWord = AsciiToHex(pwdinternal);
			elUnlock->appendInputData16(hexWord);
			pwdinternal += 4;
		}

		HalExecCommand cmd;
		cmd.elements.emplace_back(elUnlock);

		if (!this->parent->send(cmd))
		{
			return -2;
		}
		#ifndef NDEBUG
			printf("Unlock device\n");
		#endif

		return 1;
	}
	// if we have a "normal" msp430 device without special handling
	if ((deviceCode != 0xA55AA55A && deviceCode != 0x5AA55AA5) || deviceCode == 0x80058005)
	{
		HalExecCommand startJtag;
		HalExecElement* elStart = new HalExecElement(ID_StartJtag);
		switch (this->mode) {
		case JTAG_IF:
			elStart->appendInputData8(0);
			break;
		case SPYBIWIRE_IF:
			elStart->appendInputData8(1);
			break;
		case SPYBIWIREJTAG_IF:
			elStart->appendInputData8(2);
			break;
		case SPYBIWIRE_DCDC:
			elStart->appendInputData8(5);
			break;
		case SPYBIWIRE_MSP_FET_IF:
			elStart->appendInputData8(6);
			break;
		case JTAG_MSP432:
			elStart->appendInputData8(7);
			break;
		case SWD_MSP432:
			elStart->appendInputData8(8);
			break;
		default:
			delete elStart;
			return 0;
		}

		startJtag.elements.emplace_back(elStart);
		if (!this->parent->send(startJtag))
		{
			return -1;
		}
		const uint8_t numOfDevices = elStart->getOutputAt8(0);
		return numOfDevices;
	}
	return 0;
}

bool ConfigManager::configureJtagSpeed(uint32_t speed)
{
	enum INTERFACE_SPEED iSpeed = (enum INTERFACE_SPEED)speed;
	bool retValue = false;
	switch (iSpeed)
	{
		case FAST:
			if (arch == TARGET_ARCHITECTURE_t::MSP430)
			{
				retValue = this->setJtagSpeed(JTAG_4WIRE_SPEED_8_MHZ, JTAG_2WIRE_SPEED_600_KHZ);
			}
			else
			{
				retValue = this->setJtagSpeed(JTAG_4WIRE_SPEED_15_MHZ, JTAG_2WIRE_SPEED_600_KHZ);
			}
			break;
		case MEDIUM:
			retValue = this->setJtagSpeed(JTAG_4WIRE_SPEED_4_MHZ, JTAG_2WIRE_SPEED_400_KHZ);
			break;
		case SLOW:
			retValue = this->setJtagSpeed(JTAG_4WIRE_SPEED_1_MHZ, JTAG_2WIRE_SPEED_200_KHZ);
			break;
		default:
			retValue = this->setJtagSpeed(JTAG_4WIRE_SPEED_4_MHZ, JTAG_2WIRE_SPEED_600_KHZ);
			break;
	}
	return retValue;
}

bool ConfigManager::setJtagSpeed(JTAG_4WIRE_SPEED speedJtag, JTAG_2WIRE_SPEED speedSbw)
{
	FetControl * control=this->parent->getControl();
	if (control->getFetToolId() != MSP_FET430)
	{
		HalExecElement* el = new HalExecElement(ID_Configure);
		el->appendInputData32(CONFIG_PARAM_JTAG_SPEED);
		el->appendInputData32(speedJtag);
		el->appendInputData32(speedSbw);

		HalExecCommand configCmd;
		configCmd.elements.emplace_back(el);
		return this->parent->send(configCmd);
	}
	return true;
}

bool ConfigManager::stop ()
{
	HalExecCommand stopJtag;
	stopJtag.elements.emplace_back(new HalExecElement(ID_StopJtag));
	return this->parent->send(stopJtag);
}

long ConfigManager::MSP430I_MagicPattern(uint16_t ifMode)
{
	uint16_t mode[2] = {0};

	if (ifMode == AUTOMATIC_IF)
	{
		mode[0] = SPYBIWIRE_IF;
		mode[1] = SPYBIWIREJTAG_IF;
	}
	else
	{
		mode[0]	= ifMode;
		mode[1]	= ifMode;
	}

	for (uint16_t i = 0; i < 2; i++)
	{
		setJtagMode((INTERFACE_TYPE)mode[i]);
		this->start();

		HalExecElement* el = new HalExecElement(ID_Reset430I);
		HalExecCommand cmd;
		cmd.elements.emplace_back(el);

		bool success = this->parent->send(cmd);

		if (success)
		{
			const uint8_t chainLen = el->getOutputAt8(0);
			const uint8_t iJtagID = el->getOutputAt8(1);

			if ((chainLen > 0) && (iJtagID == 0x89))
			{
				// Return the protocol
				return 0;
			}
		}
	}
	return -1;
}


bool ConfigManager::reset(bool vcc, bool nmi, uint16_t JtagId, uint32_t rstHalId)
{
	if (jtagIdIsXv2(JtagId) || deviceCode == 0x20404020)
	{
		if (vcc)
		{
			uint16_t voltage = this->getDeviceVcc();
			if (!this->setDeviceVcc(0))
			{
				return false;
			}
			// keep voltage 0 for minmum 5 seconds
			this_thread::sleep_for(chrono::seconds(5));

			if (!this->setDeviceVcc(voltage))
			{
				return false;
			}
			this->start();
		}

		if (vcc || nmi)
		{
			hal_id resetMacro;
			if (deviceCode == 0x20404020)
			{
				resetMacro = ID_Reset430I;
			}
			else if (deviceCode == 0xA55AA55A || deviceCode == 0x5AA55AA5)
			{
				resetMacro = ID_ResetL092;
			}
			else
			{
				resetMacro = (hal_id)rstHalId;
			}

			HalExecElement* el = new HalExecElement(resetMacro);
			if (deviceCode == 0xA55AA55A || deviceCode == 0x5AA55AA5)
			{
				el->appendInputData32(deviceCode);
			}

			HalExecCommand cmd;
			cmd.setTimeout(10000);
			cmd.elements.emplace_back(el);
			if (!this->parent->send(cmd))
			{
				return false;
			}
		}
	}
	else
	{
		/* for all other CPU architectures we just toggle the RST pint to create a BOR */
		if (nmi)
		{
			list<PinState> pinStates;
			pinStates.push_back( PinState(JTAG_PIN_SELTST, false)(JTAG_PIN_RST, false).setDelay(10) );
			pinStates.push_back( PinState(JTAG_PIN_SELTST, false)(JTAG_PIN_RST, true) );
			if (!sendPinSequence(pinStates, parent))
			{
				return false;
			}
		}
		if (vcc)
		{
			uint16_t voltage = this->getDeviceVcc();
			if (!this->setDeviceVcc(0))
			{
				return false;
			}
			// keep voltate 0 for minmum 5 seconds
			this_thread::sleep_for(chrono::seconds(5));
			if (!this->setDeviceVcc(voltage))
			{
				return false;
			}
		}
	}
	return true;
}

void ConfigManager::setCurrentDrive(uint32_t value)
{
	if (value)
	{
		const uint16_t ENERGYTRACE_FINE_MODE = 0x8000;
		mhighres = ENERGYTRACE_FINE_MODE;
	}
	else
	{
		mhighres = 0;
	}
}

void ConfigManager::setEnergyTraceManager(EnergyTraceManager * etm)
{
	mEnergyTraceManager	= etm;
}


bool ConfigManager::setVccEzFet(uint16_t vcc)
{
	#ifndef NDEBUG
		printf("VCC  in[mV]: %i\n", vcc);
	#endif

	if (vcc)
	{
		uint16_t actualVcc = 0;
		//calculate VCC to configure sub mcu.
		for (uint16_t i = 0 ; i < 4; i++)
		{
			actualVcc += this->getDeviceVcc();
		}
		actualVcc = actualVcc / 4;

		actualVcc = ((actualVcc + 50) / 100) * 100;

		if (actualVcc > 3600)
		{
			actualVcc = 3600;
		}

		if (actualVcc < 1800)
		{
			actualVcc = 1800;
		}

		//  send VCC set comand to SubMco
		HalExecElement* el = new HalExecElement(ID_Zero, dcdcSetVcc);
		el->appendInputData16(actualVcc); // mhighres Added for high resolution mode --> default 0;

		HalExecCommand dcdcCmd;
		dcdcCmd.elements.emplace_back(el);
		if (!this->parent->send(dcdcCmd))
		{
			return false;
		}

		// now run calibration to support Energy Trace
		if (mEnergyTraceManager)
		{
			// Cut the power switch to the target first before doing calibration
			HalExecElement* el = new HalExecElement(ID_Zero, coreSwitchFet);
			el->appendInputData16(ALL_OFF);
			HalExecCommand dcdcCmd;
			dcdcCmd.elements.emplace_back(el);
			dcdcCmd.setTimeout(15000);
			if (!this->parent->getControl()->send(dcdcCmd))
			{
				return false;
			}

			this->mEnergyTraceManager->doCalibration(actualVcc);
		}
		// then send power on command to hil module to switch MOSFET
		el = new HalExecElement(ID_Zero, coreSwitchFet);
		el->appendInputData16(LDO_ON);
		HalExecCommand hilCmd;
		hilCmd.elements.emplace_back(el);
		hilCmd.setTimeout(15000);
		if (!this->parent->send(hilCmd))
		{
			return false;
		}
		this->vcc = actualVcc;

	#if defined(_WIN32) || defined(_WIN64) ||  defined(__APPLE__)
		if (vcc)
		{
			this_thread::sleep_for(chrono::milliseconds(500));
		}
	#else
		this_thread::sleep_for(chrono::milliseconds(1000));
	#endif

	}
	else // just shutdown voltage
	{
		// then send power on command to hil module to switch MOSFET
		HalExecElement* el = new HalExecElement(ID_Zero, coreSetVcc);
		el->appendInputData16(0);

		HalExecCommand hilCmd;
		hilCmd.elements.emplace_back(el);
		if (!this->parent->send(hilCmd))
		{
			return false;
		}

		// Send power down comand to Sub mcu Firmware
		el = new HalExecElement(ID_Zero, dcdcPowerDown);

		HalExecCommand dcdcCmd;
		dcdcCmd.elements.emplace_back(el);
		if (!this->parent->send(dcdcCmd))
		{
			return false;
		}
	}
	return true;
}

bool ConfigManager::setVccMspFET(uint16_t vcc)
{
	HalExecCommand cmd;
	HalExecElement* el;

	el = new HalExecElement(ID_Zero, coreSwitchFet);
	el->appendInputData16(0);
	cmd.elements.emplace_back(el);
	cmd.setTimeout(15000);
	if (!this->parent->send(cmd))
	{
		return false;
	}

	if (vcc)
	{
		// send VCC set command to SubMcu - configure PWM pulses
		el = new HalExecElement(ID_Zero, dcdcSetVcc);
		el->appendInputData16(vcc);
		cmd.setTimeout(10000);
		cmd.elements.clear();
		cmd.elements.emplace_back(el);
		if (!this->parent->send(cmd))
		{
			return false;
		}

		el = new HalExecElement(ID_Zero, coreSetVcc);
		el->appendInputData16(vcc);
		cmd.elements.clear();
		cmd.setTimeout(10000);
		cmd.elements.emplace_back(el);
		if (!this->parent->send(cmd))
		{
			return false;
		}

	#if defined(_WIN32) || defined(_WIN64) ||  defined(__APPLE__)
		this_thread::sleep_for(chrono::milliseconds(100));
	#else
		this_thread::sleep_for(chrono::milliseconds(700));
	#endif

		// now run calibration to support Energy Trace
		if (mEnergyTraceManager)
		{
			this->mEnergyTraceManager->doCalibration(vcc);
		}
		// then send power on command to hil module to switch MOSFET
		el = new HalExecElement(ID_Zero, coreSwitchFet);
		el->appendInputData16(1);
		cmd.elements.clear();
		cmd.setTimeout(10000);
		cmd.elements.emplace_back(el);
		if (!this->parent->send(cmd))
		{
			return false;
		}
	}
	else // just shotdown voltate
	{
		el = new HalExecElement(ID_Zero, coreSetVcc);
		el->appendInputData16(vcc);
		cmd.elements.clear();
		cmd.setTimeout(10000);
		cmd.elements.emplace_back(el);
		if (!this->parent->send(cmd))
		{
			return false;
		}
	#if defined(_WIN32) || defined(_WIN64) ||  defined(__APPLE__)
		this_thread::sleep_for(chrono::milliseconds(100));
	#else
		this_thread::sleep_for(chrono::milliseconds(700));
	#endif

		// Send power down command to Sub mcu Firmware
		el = new HalExecElement(ID_Zero, dcdcPowerDown);
		cmd.elements.clear();
		cmd.elements.emplace_back(el);
		if (!this->parent->send(cmd))
		{
			return false;
		}
	}
	return true;
}

bool ConfigManager::setVccMspFetUif(uint16_t vcc)
{
	HalExecCommand cmd;
	#ifndef NDEBUG
		printf("VCC  in[mV]: %i\n", vcc);
	#endif
		HalExecElement* el = new HalExecElement(ID_SetVcc);
		el->appendInputData16(vcc);
		cmd.elements.emplace_back(el);
		cmd.setTimeout(5000);
		if (!this->parent->send(cmd))
		{
			return false;
		}
		this->vcc = vcc;
		
	#if defined(_WIN32) || defined(_WIN64) ||  defined(__APPLE__)
		if (vcc)
		{
			this_thread::sleep_for(chrono::milliseconds(1000));
		}
	#else
			this_thread::sleep_for(chrono::milliseconds(2500));
	#endif
		return true;
}

bool ConfigManager::setDeviceVcc (uint16_t vcc)
{
	FetControl * control=this->parent->getControl();
	if (control->getFetToolId() == eZ_FET_WITH_DCDC 
		|| control->getFetToolId() == eZ_FET_WITH_DCDC_NO_FLOWCT
		|| control->getFetToolId() == eZ_FET_WITH_DCDC_V2x)
	{
		return setVccEzFet(vcc);
	}
	else if (control->getFetToolId() == eZ_FET_NO_DCDC)
	{
		// fixed LDO voltage, No power switch
		return true;
	}
	else if (control->getFetToolId() == MSP_FET_WITH_DCDC || control->getFetToolId() == MSP_FET_WITH_DCDC_V2x)
	{
		return setVccMspFET(vcc);
	}
	else if (control->getFetToolId() == MSP_FET430)
	{
		return setVccMspFetUif(vcc);
	}
	return false;
}

uint16_t ConfigManager::getDeviceVcc () const
{
	HalExecCommand cmd;
	FetControl * control = this->parent->getControl();
	HalExecElement* el;
	if (control->getFetToolId() != MSP_FET430)
	{
		el = new HalExecElement(ID_Zero, coreGetVcc);
	}	
	else
	{
		el = new HalExecElement(ID_GetVcc);
	}

	cmd.elements.emplace_back(el);
	if (!this->parent->send(cmd))
	{
		return 0;
	}
	return el->getOutputAt16(0);
}

uint16_t ConfigManager::getExternalVcc () const
{
	HalExecCommand cmd;
	FetControl * control = this->parent->getControl();
	HalExecElement* el;
	if (control->getFetToolId() != MSP_FET430)
	{
		el = new HalExecElement(ID_Zero, coreGetVcc);
	}
	else
	{
		el = new HalExecElement(ID_GetVcc);
	}

	cmd.elements.emplace_back(el);
	if (!this->parent->send(cmd))
		return 0;

	return el->getOutputAt16(2);
}

bool ConfigManager::configureOverCurrent(bool state)
{
	FetControl * control=this->parent->getControl();
	if (control->getFetToolId() != MSP_FET430)
	{
		HalExecCommand cmd;
		HalExecElement* el = new HalExecElement(ID_Zero, CmdOverCurrent);
		el->appendInputData8(state);
		cmd.elements.emplace_back(el);
		if (!this->parent->send(cmd))
		{
			return false;
		}
	}
	return true;
}

void ConfigManager::setDisableInterruptsMode(uint32_t mode)
{
	disableInterruptsMode = mode;
}

uint32_t ConfigManager::getDisableInterruptsMode() const
{
	return disableInterruptsMode;
}

bool ConfigManager::updateDisableInterruptsMode()
{
	if (disableInterruptsMode == disableInterruptsModeBackup)
	{
		return true;
	}

	HalExecElement* el = new HalExecElement(ID_Configure);
	el->appendInputData32(CONFIG_PARAM_INTERRUPT_OPTIONS);
	el->appendInputData32(disableInterruptsMode);

	HalExecCommand configCmd;
	configCmd.elements.emplace_back(el);

	if (this->parent->send(configCmd))
	{
		disableInterruptsModeBackup = disableInterruptsMode;
		return true;
	}

	return false;
}

bool ConfigManager::setJTAGLock5xx(uint32_t value)
{
	HalExecElement* el = new HalExecElement(ID_Configure);
	el->appendInputData32(CONFIG_PARAM_JTAG_LOCK_5XX);
	el->appendInputData32(value);

	HalExecCommand configCmd;
	configCmd.elements.emplace_back(el);

	if (this->parent->send(configCmd))
	{
		return true;
	}

	return false;
}

bool ConfigManager::firmWareUpdate(const char* fname, UpdateNotifyCallback callback, bool* coreUpdate)
{
	return this->updateManagerFet->firmWareUpdate(fname, callback, coreUpdate);
}

void ConfigManager::setPassword(const string& pwd)
{
	this->password = pwd;
}

bool ConfigManager::setDeviceCode(uint32_t deviceCode)
{
	 this->deviceCode = deviceCode;
	 return true;
}

bool ConfigManager::freqCalibrationEnabled() const
{
	return freqCalibration;
}

bool ConfigManager::ulpDebugEnabled() const
{
	return ulpDebug;
}

bool ConfigManager::atProbeStateEnabled() const
{
	return atProbeState;
}

void ConfigManager::setUlpDebug(bool ulp)
{
	ulpDebug = ulp;

	if (arch == MSP432_M4)
	{
		HalExecElement* el = new HalExecElement(ID_Configure);
		el->appendInputData32(CONFIG_PARAM_ULP_MSP432);
		el->appendInputData32(ulp);

		HalExecCommand configCmd;
		configCmd.elements.emplace_back(el);
		parent->send(configCmd);
	}
}
