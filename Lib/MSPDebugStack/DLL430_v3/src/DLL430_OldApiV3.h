/*
 * DLL430_OldApi.h
 *
 * Old API interface for IAR - V3 implementation.
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

#include "DLL430_OldApi.h"

#include "FetHandleManager.h"
#include "Log.h"

#include "EM/BreakpointManager/IBreakpointManager.h"
#include "EM/TriggerCondition/ITriggerCondition.h"
#include "EM/VariableWatch/IWatchedVariable.h"

#include "EnergyTraceManager.h"
#include "PollingManager.h"
#include "IMemoryManager.h"


class DLL430_OldApiV3 : public DLL430_OldApi, public TI::DLL430::DebugEventTarget
{
public:
	DLL430_OldApiV3 ();
	~DLL430_OldApiV3 ();

	DLL430_OldApiV3(const DLL430_OldApiV3&) = delete;
	DLL430_OldApiV3& operator=(const DLL430_OldApiV3&) = delete;

	/* this implements the DebugEventTarget interface */
	void event(TI::DLL430::DebugEventTarget::EventType e,  uint32_t lParam=0, uint16_t wParam=0);

	void log(TI::DLL430::LogTarget::Severity, uint32_t, const char*);

	bool GetNumberOfUsbIfs(int32_t* Number);
	bool GetNameOfUsbIf(int32_t Idx, char** Name, int32_t* Status);
	bool SetTargetArchitecture(TARGET_ARCHITECTURE_t architecture);
	bool loadDeviceDb(const char* file);
	bool DumpDeviceDb(const char* file);
	bool clearDeviceDb();
	bool Initialize(const char* port, int32_t* version);
	bool SetSystemNotfyCallback(SYSTEM_NOTIFY_CALLBACK parSystemNotifyCallback);
	bool OpenDevice(const char* Device, const char* Password, int32_t PwLength, int32_t DeviceCode, int32_t setId);
	bool GetFoundDevice(uint8_t* FoundDevice, int32_t count);
	bool Close(int32_t vccOff);
	bool Configure(enum CONFIG_MODE mode, int32_t value);
	int32_t Error_Number();
	const char* Error_String(int32_t errorNumber);
	bool GetJtagID(int32_t* JtagId);
	bool Identify(uint8_t* buffer, int32_t count, int32_t setId, const char* Password, int32_t PwLength, int32_t code);
	bool Device(int32_t localDeviceId, uint8_t* buffer, int32_t count);
	bool VCC(int32_t voltage);
	bool GetCurVCCT(int32_t* voltage);
	bool GetExtVoltage(int32_t* voltage, int32_t* state);
	bool Erase(int32_t type, int32_t address, int32_t length);
	bool Memory(int32_t address, uint8_t* buf, int32_t count, int32_t rw);
	bool Secure();
	bool ReadOutFile(int32_t wStart, int32_t wLength, const char* lpszFileName, int32_t iFileType);
	bool ProgramFile(const char* File, int32_t eraseType, int32_t verifyMem);
	bool VerifyFile(const char* File);
	bool VerifyMem(int32_t StartAddr, int32_t Length, const uint8_t* DataArray);
	bool EraseCheck(int32_t StartAddr, int32_t Length);
	bool Reset(int32_t method, int32_t execute, int32_t releaseJTAG);
	bool InterfaceType(enum INTERFACE_TYPE* type) const;
	bool RegisterMessageCallback(MessageCallbackFn callback);

	bool ExtRegisters(int32_t address, uint8_t * buffer, int32_t count, int32_t rw);

	bool Registers(int32_t* registers, int32_t mask, int32_t rw);
	bool Register(int32_t* reg, int32_t regNb, int32_t rw);
	bool Run(int32_t mode, int32_t releaseJTAG);
	bool State(int32_t* state, int32_t stop, int32_t* pCPUCycles);
	bool CcGetClockNames(int32_t localDeviceId, EemGclkCtrl_t** CcClockNames);
	bool CcGetModuleNames(int32_t localDeviceId, EemMclkCtrl_t** CcModuleNames);
	bool EEM_Init(MSP430_EVENTNOTIFY_FUNC callback, int32_t clientHandle, const MessageID_t* pMsgIdBuffer);
	bool EEM_SetBreakpoint(uint16_t* pwBpHandle, const BpParameter_t* pBpBuffer);
	bool EEM_GetBreakpoint(uint16_t wBpHandle, BpParameter_t* pBpDestBuffer);
	bool EEM_SetCombineBreakpoint(CbControl_t CbControl, uint16_t wCount, uint16_t* pwCbHandle, const uint16_t* pawBpHandle);
	bool EEM_GetCombineBreakpoint(uint16_t wCbHandle, uint16_t* pwCount, uint16_t* pawBpHandle);

	bool EEM_SetTrace(const TrParameter_t* pTrBuffer);
	bool EEM_GetTrace(TrParameter_t* pTrDestBuffer);
	bool EEM_ReadTraceBuffer(TraceBuffer_t* pTraceBuffer);
	bool EEM_ReadTraceData(TraceBuffer_t* pTraceBuffer, uint32_t *pulCount);
	bool EEM_RefreshTraceBuffer();

	bool EEM_SetVariableWatch(VwEnable_t VwEnable);
	bool EEM_SetVariable(uint16_t* pVwHandle, const VwParameter_t* pVwBuffer);
	bool EEM_GetVariableWatch(VwEnable_t* pVwEnable, VwResources_t* paVwDestBuffer);

	bool EEM_SetClockControl(const CcParameter2_t* pCcBuffer);
	bool EEM_GetClockControl(CcParameter2_t* pCcDestBuffer);

	bool EEM_SetSequencer(const SeqParameter_t* pSeqBuffer);
	bool EEM_GetSequencer(SeqParameter_t* pSeqDestBuffer);
	bool EEM_ReadSequencerState(SeqState_t* pSeqState);

	bool EEM_SetCycleCounterMode(CycleCounterMode_t mode);
	bool EEM_ConfigureCycleCounter(uint32_t wCounter, CycleCounterConfig_t pCycConfig);
	bool EEM_ReadCycleCounterValue(uint32_t wCounter, uint64_t* value);
	bool EEM_WriteCycleCounterValue(uint32_t wCounter, uint64_t value);
	bool EEM_ResetCycleCounter(uint32_t wCounter);

	bool FET_SelfTest(int32_t count, uint8_t* buffer);
	bool FET_SetSignals(int32_t SigMask, int32_t SigState);
	bool FET_Reset();
	bool FET_I2C(int32_t address, uint8_t* buffer, int32_t count, int32_t rw);
	bool FET_EnterBootloader();
	bool FET_ExitBootloader();
	bool FET_GetFwVersion(int32_t* version);
	bool FET_GetHwVersion(uint8_t** version, int32_t* count);
	bool FET_FwUpdate(const char* lpszFileName, DLL430_FET_NOTIFY_FUNC callback, int32_t clientHandle);

	void HIL_ResetJtagTap();
	void HIL_FuseCheck();
	bool HIL_Open();
	bool HIL_Configure(enum CONFIG_MODE mode, int32_t value);
	bool HIL_Connect();
	bool HIL_Connect_Entry_State(int32_t value = 0);
	bool HIL_Bsl();
	bool HIL_Close(int32_t vccOff);
	bool HIL_TCK(int32_t state);
	bool HIL_TMS(int32_t state);
	bool HIL_TDI(int32_t state);
	bool HIL_RST(int32_t state);
	bool HIL_TST(int32_t state);
	uint64_t HIL_JTAG_IR(int32_t instruction);
	uint64_t HIL_JTAG_IR4(int32_t instruction);
	uint64_t HIL_JTAG_DR(int64_t data, int32_t bitSize);
	uint64_t HIL_JTAG_IR_DR(uint32_t instruction, uint64_t data, uint32_t bits);
	bool HIL_DPACC(uint8_t address, uint32_t *data, uint16_t RdnWr);
	bool HIL_APACC(uint8_t portNum, uint8_t address, uint32_t *data, uint16_t RdnWr);
	bool HIL_MEMAP(uint8_t portNum, uint32_t address, uint32_t *data, uint16_t RdnWr);
	bool HIL_TCLK(uint8_t value);

	void execThread();
	void execNotifyCallback(SYSTEM_EVENT_MSP);
	void iNotifyCallback(uint32_t systemEventID);

	/**
		\brief EnergyTrace features
	*/
	bool EnableEnergyTrace(const EnergyTraceSetup* setup, const EnergyTraceCallbacks* callbacks, EnergyTraceHandle* handle);
	bool DisableEnergyTrace(const EnergyTraceHandle handle);
	bool ResetEnergyTrace(const EnergyTraceHandle handle);

	void setErrorCode(ERROR_CODE error) { this->errNum = error; }

	SyncedCallWrapper<DLL430_OldApi> SyncedCall() { return SyncedCallWrapper<DLL430_OldApi>(this, &apiMutex); }

private:
	template<typename T>
	class TableEntry
	{
	public:
		TableEntry() : inUse_(false) {}

		void set(T ptr) { entry_ = ptr; inUse_ = true; }
		void clear() { entry_.reset(); inUse_ = false; }
		void reserveSlot() { entry_.reset(); inUse_ = true; }
		bool inUse() const { return inUse_; }
		const T& value() const { return entry_; }

	private:
		T entry_;
		bool inUse_;
	};

	bool hardwareTriggerAtAddressExists(uint32_t address) const;
	bool softwareTriggerAtAddressExists(uint32_t address) const;
	bool rangeTriggerIncludingAddressExists(uint32_t address) const;
	bool softwareTriggerInRangeExists(uint32_t start, uint32_t end, BpRangeAction_t inOut) const;
	bool triggerConflictsWithExistingTrigger(const BpParameter_t* bpBuffer) const;
	bool criticalRrcmInstructionAt(uint32_t address);
	TI::DLL430::TriggerConditionPtr triggerConditionFromBpParameter(TI::DLL430::EmulationManagerPtr emuManager, const BpParameter_t* bpBuffer);
	void addBreakpointsAndStorage(TI::DLL430::EmulationManagerPtr emuManager, TI::DLL430::TriggerConditionPtr triggerCondition, BpAction_t reactions, uint16_t handle);
	void updateStorageReactions(TI::DLL430::EmulationManagerPtr emuManager);
	void updateCounterReactions(TI::DLL430::EmulationManagerPtr emuManager);
	void clearSoftwareTriggers();
	void restoreSoftwareTriggers(std::map<uint16_t, BpParameter_t>& bpStorage);
	bool disableSoftwareBreakpointsOnClose();

	void resetEM();

	bool writeToExternalMemory();
	bool lockMemory(TI::DLL430::MemoryArea::Name memoryName, bool lock);

	void prepareEemAccess() const;
	void checkCycleCounterConflict(uint32_t wCounter) const;

	bool deviceIsRunning() const;

	typedef std::map<long, TableEntry<TI::DLL430::TriggerConditionPtr> > TriggerTable;
	TriggerTable triggers;
	TriggerTable traceTriggers;
	TriggerTable counterTriggers;

	std::map<int32_t, TableEntry<TI::DLL430::BreakpointPtr> > breakpoints;
	std::map<uint16_t, std::vector<uint16_t> > triggerCombinations;
	std::map<uint16_t, TI::DLL430::WatchedVariablePtr> watchedVariables;
	boost::mutex watchedVariablesMutex;
	VwEnable_t varWatch_state;

	// Contains all the config data
	std::map<enum CONFIG_MODE, int32_t> config_settings;

	// Container to keep persistent list of port names to be referenced from IDE
	// Pointers to strings in table get invalidated when initializing
	struct port_name {
		char name[64];
		port_name() { memset(name, 0, sizeof(name)); }
	};
	typedef std::deque<port_name> port_names_list_type;
	port_names_list_type port_names;

	CcParameter2_t clock_control;
	EemMclkCtrl_t moduleNameBuffer;

	std::map<uint16_t, BpParameter_t> bp_storage;
	std::map<uint16_t, VwResources_t> vw_storage;
	TrParameter_t trace_storage;	// trace API parameter buffer
	SeqParameter_t sequencer_control;

	std::unique_ptr<TI::DLL430::FetHandleManager> manager;
	TI::DLL430::IFetHandle* handle;

	uint32_t errNum;

	// only 1 FET with one device
	TI::DLL430::IDeviceHandle * singleDevice;
	int32_t selectedJtagMode;

	bool devInfoFilled;
	DEVICE_T devInfo;
	uint32_t devCode;
	TARGET_ARCHITECTURE_t devArchitecture;

	struct {
		enum STATE_MODES state;
		bool jtagReleased;
		struct {
			MSP430_EVENTNOTIFY_FUNC func;
			int32_t clientHandle;
			MessageID_t ids;
		} cb;
	} debug;

	SYSTEM_NOTIFY_CALLBACK notifyCallback;
	uint32_t clientHandle;

	TI::DLL430::PollingManager *mPollingManager;

	/**
	EnergyTrace variables
	*/
	TI::DLL430::EnergyTraceManager *mEnergyTraceManager;
	EnergyTraceSetup mPdSetup;		   /**< EnergyTrace parameters */
	EnergyTraceCallbacks mPdCallbacks; /**< EnergyTrace callback functions */

	boost::recursive_mutex apiMutex;
	boost::mutex callbackMutex;

	uint32_t lastCycleCount;
};
