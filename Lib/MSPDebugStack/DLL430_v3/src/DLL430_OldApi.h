/*
 * DLL430_OldApi.h
 *
 * Old API interface for IAR & CSS.
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

#include "MSP430.h"
#include "MSP430_EEM.h"
#include "MSP430_FET.h"
#include "MSP430_HIL.h"
#include "MSP430_EnergyTrace.h"

#include "SyncedCallWrapper.h"


class DLL430_OldApi
{
public:
	virtual ~DLL430_OldApi() {};

	virtual SyncedCallWrapper<DLL430_OldApi> SyncedCall() = 0;

	virtual bool GetNumberOfUsbIfs(int32_t* Number) = 0;
	virtual bool GetNameOfUsbIf(int32_t Idx, char** Name, int32_t* Status) = 0;
	virtual bool loadDeviceDb(const char* file) = 0;
	virtual bool DumpDeviceDb(const char* file) = 0;
	virtual bool clearDeviceDb() = 0;
	virtual bool Initialize(const char* port, int32_t* version) = 0;
	virtual bool SetTargetArchitecture(TARGET_ARCHITECTURE_t architecture) = 0;
	virtual bool SetSystemNotfyCallback(SYSTEM_NOTIFY_CALLBACK parSystemNotifyCallback) = 0;
	virtual bool OpenDevice(const char* Device, const char* Password, int32_t PwLength, int32_t DeviceCode, int32_t setId) = 0;
	virtual bool GetFoundDevice(uint8_t* FoundDevice, int32_t count) = 0;
	virtual bool Close(int32_t vccOff) = 0;
	virtual bool Configure(enum CONFIG_MODE mode, int32_t value) = 0;
	virtual int32_t Error_Number() = 0;
	virtual const char* Error_String(int32_t errorNumber) = 0;
	virtual bool GetJtagID(int32_t* JtagId) = 0;
	virtual bool Device(int32_t localDeviceId, uint8_t* buffer, int32_t count) = 0;
	virtual bool VCC(int32_t voltage) = 0;
	virtual bool GetCurVCCT(int32_t* voltage) = 0;
	virtual bool GetExtVoltage(int32_t* voltage, int32_t* state) = 0;
	virtual bool Erase(int32_t type, int32_t address, int32_t length) = 0;
	virtual bool Memory(int32_t address, uint8_t* buf, int32_t count, int32_t rw) = 0;
	virtual bool Secure() = 0;
	virtual bool ReadOutFile(int32_t wStart, int32_t wLength, const char* lpszFileName, int32_t iFileType) = 0;
	virtual bool ProgramFile(const char* File, int32_t eraseType, int32_t verifyMem) = 0;
	virtual bool VerifyFile(const char* File) = 0;
	virtual bool VerifyMem(int32_t StartAddr, int32_t Length, const uint8_t* DataArray) = 0;
	virtual bool EraseCheck(int32_t StartAddr, int32_t Length) = 0;
	virtual bool Reset(int32_t method, int32_t execute, int32_t releaseJTAG) = 0;
	virtual bool Registers(int32_t* registers, int32_t mask, int32_t rw) = 0;
	virtual bool InterfaceType(INTERFACE_TYPE* type) const = 0;
	virtual bool RegisterMessageCallback(MessageCallbackFn callback) = 0;

	virtual bool ExtRegisters(int32_t address, uint8_t * buffer, int32_t count, int32_t rw) = 0;

	virtual bool Register(int32_t* reg, int32_t regNb, int32_t rw) = 0;
	virtual bool Run(int32_t mode, int32_t releaseJTAG) = 0;
	virtual bool State(int32_t* state, int32_t stop, int32_t* pCPUCycles) = 0;

	virtual bool CcGetClockNames(int32_t localDeviceId, EemGclkCtrl_t** CcClockNames) = 0;
	virtual bool CcGetModuleNames(int32_t localDeviceId, EemMclkCtrl_t** CcModuleNames) = 0;
	virtual bool EEM_Init(MSP430_EVENTNOTIFY_FUNC callback, int32_t clientHandle, const MessageID_t* pMsgIdBuffer) = 0;
	virtual bool EEM_SetBreakpoint(uint16_t* pwBpHandle, const BpParameter_t* pBpBuffer) = 0;
	virtual bool EEM_GetBreakpoint(uint16_t wBpHandle, BpParameter_t* pBpDestBuffer) = 0;
	virtual bool EEM_SetCombineBreakpoint(CbControl_t CbControl, uint16_t wCount, uint16_t* pwCbHandle, const uint16_t* pawBpHandle) = 0;
	virtual bool EEM_GetCombineBreakpoint(uint16_t wCbHandle, uint16_t* pwCount, uint16_t* pawBpHandle) = 0;
	virtual bool EEM_SetTrace(const TrParameter_t* pTrBuffer) = 0;
	virtual bool EEM_GetTrace(TrParameter_t* pTrDestBuffer) = 0;
	virtual bool EEM_ReadTraceBuffer(TraceBuffer_t* pTraceBuffer) = 0;
	virtual bool EEM_ReadTraceData(TraceBuffer_t* pTraceBuffer, uint32_t *pulCount) = 0;
	virtual bool EEM_RefreshTraceBuffer() = 0;
	virtual bool EEM_SetVariableWatch(VwEnable_t VwEnable) = 0;
	virtual bool EEM_SetVariable(uint16_t* pVwHandle, const VwParameter_t* pVwBuffer) = 0;
	virtual bool EEM_GetVariableWatch(VwEnable_t* pVwEnable, VwResources_t* paVwDestBuffer) = 0;
	virtual bool EEM_SetClockControl(const CcParameter2_t* pCcBuffer) = 0;
	virtual bool EEM_GetClockControl(CcParameter2_t* pCcDestBuffer) = 0;
	virtual bool EEM_SetSequencer(const SeqParameter_t* pSeqBuffer) = 0;
	virtual bool EEM_GetSequencer(SeqParameter_t* pSeqDestBuffer) = 0;
	virtual bool EEM_ReadSequencerState(SeqState_t* pSeqState) = 0;
	virtual bool EEM_SetCycleCounterMode(CycleCounterMode_t mode) = 0;
	virtual bool EEM_ConfigureCycleCounter(uint32_t wCounter, CycleCounterConfig_t pCycConfig) = 0;
	virtual bool EEM_ReadCycleCounterValue(uint32_t wCounter, uint64_t* value) = 0;
	virtual bool EEM_WriteCycleCounterValue(uint32_t wCounter, uint64_t value) = 0;
	virtual bool EEM_ResetCycleCounter(uint32_t wCounter) = 0;

	virtual bool FET_SelfTest(int32_t count, uint8_t* buffer) = 0;
	virtual bool FET_SetSignals(int32_t SigMask, int32_t SigState) = 0;
	virtual bool FET_Reset() = 0;
	virtual bool FET_I2C(int32_t address, uint8_t* buffer, int32_t count, int32_t rw) = 0;
	virtual bool FET_EnterBootloader() = 0;
	virtual bool FET_ExitBootloader() = 0;
	virtual bool FET_GetFwVersion(int32_t* version) = 0;
	virtual bool FET_GetHwVersion(uint8_t** version, int32_t* count) = 0;
	virtual bool FET_FwUpdate(const char* lpszFileName, DLL430_FET_NOTIFY_FUNC callback, int32_t clientHandle) = 0;

	virtual void HIL_ResetJtagTap() = 0;
	virtual bool HIL_Open() = 0;
	virtual bool HIL_Configure(enum CONFIG_MODE mode, int32_t value) = 0;
	virtual bool HIL_Connect() = 0;
	virtual bool HIL_Connect_Entry_State(int32_t value) = 0;
	virtual bool HIL_Bsl() = 0;
	virtual bool HIL_Close(int32_t vccOff) = 0;
	virtual bool HIL_TCK(int32_t state) = 0;
	virtual bool HIL_TMS(int32_t state) = 0;
	virtual bool HIL_TDI(int32_t state) = 0;
	virtual bool HIL_RST(int32_t state) = 0;
	virtual bool HIL_TST(int32_t state) = 0;
	virtual uint64_t HIL_JTAG_IR(int32_t instruction) = 0;
	virtual uint64_t HIL_JTAG_IR4(int32_t instruction) = 0;
	virtual uint64_t HIL_JTAG_DR(int64_t data, int32_t bits) = 0;
	virtual uint64_t HIL_JTAG_IR_DR(uint32_t instruction, uint64_t data, uint32_t bits) = 0;
	virtual bool HIL_DPACC(uint8_t address, uint32_t *data, uint16_t RdnWr) = 0;
	virtual bool HIL_APACC(uint8_t portNum, uint8_t address, uint32_t *data, uint16_t RdnWr) = 0;
	virtual bool HIL_MEMAP(uint8_t portNum, uint32_t address, uint32_t *data, uint16_t RdnWr) = 0;
	virtual bool HIL_TCLK(uint8_t value) = 0;
	virtual	void HIL_FuseCheck() = 0;

	/**
		\brief Power debugging features
	*/
	virtual bool EnableEnergyTrace(const EnergyTraceSetup* setup, const EnergyTraceCallbacks* callbacks, EnergyTraceHandle* handle) = 0;
	virtual bool DisableEnergyTrace(const EnergyTraceHandle handle) = 0;
	virtual bool ResetEnergyTrace(const EnergyTraceHandle handle) = 0;

	virtual void setErrorCode(ERROR_CODE error) = 0;
};
