/*
 * DLL430_capi.cpp
 *
 * C API implementation.
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
#include "DLL430_OldApiV3.h"
#include <cstring>

DLL430_OldApi* DLL430_CurrentInstance = 0;
#if defined(_WIN32) || defined (_WIN64)
#define ENTRY_FUNCTION bool
#define EXIT_FUNCTION void
#elif defined (UNIX)
#define ENTRY_FUNCTION bool __attribute__ ((constructor))
#define EXIT_FUNCTION void __attribute__ ((destructor))
#endif

ENTRY_FUNCTION oldapi_INIT()
{
	DLL430_CurrentInstance = 0;
	return true;
}

EXIT_FUNCTION oldapi_EXIT()
{
	if (DLL430_CurrentInstance != 0)
		delete DLL430_CurrentInstance;
}

static STATUS_T toStatus (bool b)
{
	return b ? STATUS_OK : STATUS_ERROR;
}

static void createInstance ()
{
	if (DLL430_CurrentInstance == 0)
	{
		DLL430_CurrentInstance = new DLL430_OldApiV3;
		DLL430_CurrentInstance->loadDeviceDb(NULL);
	}
}

STATUS_T WINAPI MSP430_GetNumberOfUsbIfs(int32_t* Number) 
{
	createInstance();
	int32_t count;

	if (!DLL430_CurrentInstance->GetNumberOfUsbIfs(&count))
		return toStatus(false);

	if (Number)
	{
		*Number = count;
	}
	return toStatus(true);
}

STATUS_T WINAPI MSP430_GetNameOfUsbIf(int32_t Idx, char** Name, int32_t* Status)
{
	createInstance();

	return toStatus(DLL430_CurrentInstance->GetNameOfUsbIf(Idx, Name, Status));
}

STATUS_T WINAPI MSP430_LoadDeviceDb(const char* file)
{
	if (!DLL430_CurrentInstance)
		return STATUS_OK;
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->loadDeviceDb(file));
}

STATUS_T WINAPI MSP430_DumpDeviceDb(const char* file)
{
	if (!DLL430_CurrentInstance)
		return STATUS_OK;
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->DumpDeviceDb(file));
}

STATUS_T WINAPI MSP430_ClearDeviceDb()
{
	if (!DLL430_CurrentInstance)
		return STATUS_OK;
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->clearDeviceDb());
}

STATUS_T WINAPI MSP430_SetTargetArchitecture(TARGET_ARCHITECTURE_t architecture)
{
	createInstance();
	return toStatus(DLL430_CurrentInstance->SetTargetArchitecture(architecture));
}

STATUS_T WINAPI MSP430_Initialize(const char* port, int32_t* version)
{
	createInstance();

	if ((version==nullptr)||(port==nullptr))
		return toStatus(false);

	// test port for 'LPT'
	if ((*port == '1' || *port == '2' || *port == '3') || (!strncmp(port, "LPT", 3)))
	{
		DLL430_CurrentInstance->setErrorCode(NO_LPT_SUPPORT);
		return toStatus(false);
	}

	return toStatus(DLL430_CurrentInstance->Initialize(port, version));
	bool ret = false;

}

STATUS_T WINAPI MSP430_SET_SYSTEM_NOTIFY_CALLBACK(SYSTEM_NOTIFY_CALLBACK parSystemNotifyCallback)
{
	if (!DLL430_CurrentInstance)
		return STATUS_OK;
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->SetSystemNotfyCallback(parSystemNotifyCallback));
}

STATUS_T WINAPI MSP430_OpenDevice(const char* Device, const char* Password, int32_t PwLength, int32_t DeviceCode, int32_t setId)
{
	if (!DLL430_CurrentInstance)
		return STATUS_OK;
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->SyncedCall()->OpenDevice(Device, Password, PwLength, DeviceCode, setId));
}

STATUS_T WINAPI MSP430_GetFoundDevice(uint8_t* FoundDevice, int32_t count)
{
	if (!DLL430_CurrentInstance)
		return STATUS_OK;
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->GetFoundDevice(FoundDevice, count));
}

STATUS_T WINAPI MSP430_Close(int32_t vccOff)
{
	if (!DLL430_CurrentInstance)
		return STATUS_OK;
	//Call must not be syncronized (potential dead lock)
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->Close(vccOff));
}

STATUS_T WINAPI MSP430_Configure(int32_t mode, int32_t value)
{
	enum CONFIG_MODE m = (enum CONFIG_MODE)mode;
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->SyncedCall()->Configure(m, value));
}

int32_t WINAPI MSP430_Error_Number(void)
{
	if (!DLL430_CurrentInstance)
		return 0;
	return DLL430_CurrentInstance->Error_Number();
}

const char* WINAPI MSP430_Error_String(int32_t errorNumber)
{
	if (!DLL430_CurrentInstance)
	{
		return "MSP DebugStack not initialized";
	}
	return DLL430_CurrentInstance->Error_String(errorNumber);
}

STATUS_T WINAPI MSP430_GetJtagID(int32_t* JtagId)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->GetJtagID(JtagId));
}

STATUS_T WINAPI MSP430_Device(int32_t localDeviceId, uint8_t* buffer, int32_t count)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->Device(localDeviceId, buffer, count));
}

STATUS_T WINAPI MSP430_VCC(int32_t voltage)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->SyncedCall()->VCC(voltage));
}

STATUS_T WINAPI MSP430_GetCurVCCT(int32_t* voltage)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->SyncedCall()->GetCurVCCT(voltage));
}

STATUS_T WINAPI MSP430_GetExtVoltage(int32_t* voltage, int32_t* state)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->SyncedCall()->GetExtVoltage(voltage, state));
}

STATUS_T WINAPI MSP430_Erase(int32_t type, int32_t address, int32_t length)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->SyncedCall()->Erase(type, address, length));
}

STATUS_T WINAPI MSP430_Memory(int32_t address, uint8_t* buffer, int32_t count, int32_t rw)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->SyncedCall()->Memory(address, buffer, count, rw));
}

STATUS_T WINAPI MSP430_Secure(void)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->SyncedCall()->Secure());
}

STATUS_T WINAPI MSP430_ReadOutFile(int32_t wStart, int32_t wLength, const char* lpszFileName, int32_t iFileType)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->SyncedCall()->ReadOutFile(wStart, wLength, lpszFileName, iFileType));
}

STATUS_T WINAPI MSP430_ProgramFile(const char* File, int32_t eraseType, int32_t verifyMem)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->SyncedCall()->ProgramFile(File, eraseType, verifyMem));
}

STATUS_T WINAPI MSP430_VerifyFile(const char* File)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->VerifyFile(File));
}

STATUS_T WINAPI MSP430_VerifyMem(int32_t StartAddr, int32_t Length, const uint8_t* DataArray)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->SyncedCall()->VerifyMem(StartAddr, Length, DataArray));
}

STATUS_T WINAPI MSP430_EraseCheck(int32_t StartAddr, int32_t Length)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->SyncedCall()->EraseCheck(StartAddr, Length));
}

STATUS_T WINAPI MSP430_Reset(int32_t method, int32_t execute, int32_t releaseJTAG)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->SyncedCall()->Reset(method, execute, releaseJTAG));
}

STATUS_T WINAPI MSP430_ExtRegisters(int32_t address, uint8_t* buffer, int32_t count, int32_t rw)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->ExtRegisters(address, (uint8_t*)buffer, count, rw));
}

STATUS_T WINAPI MSP430_Registers(int32_t* registers, int32_t mask, int32_t rw)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->Registers(registers, mask, rw));
}

STATUS_T WINAPI MSP430_Register(int32_t* reg, int32_t regNb, int32_t rw)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->Register(reg, regNb, rw));
}

STATUS_T WINAPI MSP430_Run(int32_t mode, int32_t releaseJTAG)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->SyncedCall()->Run(mode, releaseJTAG));
}

STATUS_T WINAPI MSP430_State(int32_t* state, int32_t stop, int32_t* pCPUCycles)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->SyncedCall()->State(state, stop, pCPUCycles));
}

STATUS_T WINAPI MSP430_CcGetClockNames(int32_t localDeviceId, EemGclkCtrl_t** CcClockNames)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->CcGetClockNames(localDeviceId, CcClockNames));
}

STATUS_T WINAPI MSP430_CcGetModuleNames(int32_t localDeviceId, EemMclkCtrl_t** CcModuleNames)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->CcGetModuleNames(localDeviceId, CcModuleNames));
}

STATUS_T WINAPI MSP430_EEM_Init(
	MSP430_EVENTNOTIFY_FUNC callback,
	int32_t clientHandle,
	const MessageID_t* pMsgIdBuffer
)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->SyncedCall()->EEM_Init(callback, clientHandle, pMsgIdBuffer));
}

STATUS_T WINAPI MSP430_EEM_SetBreakpoint(uint16_t* pwBpHandle, const BpParameter_t* pBpBuffer)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->SyncedCall()->EEM_SetBreakpoint(pwBpHandle, pBpBuffer));
}

STATUS_T WINAPI MSP430_EEM_GetBreakpoint(uint16_t wBpHandle, BpParameter_t* pBpDestBuffer)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->EEM_GetBreakpoint(wBpHandle, pBpDestBuffer));
}

STATUS_T WINAPI MSP430_EEM_SetCombineBreakpoint(
	CbControl_t CbControl,
	uint16_t wCount,
	uint16_t* pwCbHandle,
	const uint16_t* pawBpHandle
)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->SyncedCall()->EEM_SetCombineBreakpoint(CbControl, wCount, pwCbHandle, pawBpHandle));
}

STATUS_T WINAPI MSP430_EEM_GetCombineBreakpoint(
	uint16_t wCbHandle,
	uint16_t* pwCount,
	uint16_t* pawBpHandle
)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->EEM_GetCombineBreakpoint(wCbHandle, pwCount, pawBpHandle));
}

STATUS_T WINAPI MSP430_EEM_SetTrace(const TrParameter_t* pTrBuffer)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->SyncedCall()->EEM_SetTrace(pTrBuffer));
}

STATUS_T WINAPI MSP430_EEM_GetTrace(TrParameter_t* pTrDestBuffer)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->EEM_GetTrace(pTrDestBuffer));
}

STATUS_T WINAPI MSP430_EEM_ReadTraceBuffer(TraceBuffer_t* pTraceBuffer)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->SyncedCall()->EEM_ReadTraceBuffer(pTraceBuffer));
}

DLL430_SYMBOL STATUS_T WINAPI MSP430_EEM_ReadTraceData(TraceBuffer_t* pTraceBuffer, uint32_t* pulCount)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->SyncedCall()->EEM_ReadTraceData(pTraceBuffer, pulCount));
}

STATUS_T WINAPI MSP430_EEM_RefreshTraceBuffer(void)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->SyncedCall()->EEM_RefreshTraceBuffer());
}

STATUS_T WINAPI MSP430_EEM_SetVariableWatch(VwEnable_t VwEnable)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->SyncedCall()->EEM_SetVariableWatch(VwEnable));
}

STATUS_T WINAPI MSP430_EEM_SetVariable(uint16_t* pVwHandle, const VwParameter_t* pVwBuffer)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->SyncedCall()->EEM_SetVariable(pVwHandle, pVwBuffer));
}

STATUS_T WINAPI MSP430_EEM_GetVariableWatch(VwEnable_t* pVwEnable, VwResources_t* paVwDestBuffer)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->EEM_GetVariableWatch(pVwEnable, paVwDestBuffer));
}

STATUS_T WINAPI MSP430_EEM_SetClockControl(const CcParameter_t* pCcBuffer)
{
	CcParameter2_t tmp;
	tmp.ccControl = pCcBuffer->ccControl;
	tmp.ccGeneralCLK = pCcBuffer->ccGeneralCLK;
	tmp.ccModule = pCcBuffer->ccModule;
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->SyncedCall()->EEM_SetClockControl(&tmp));
}

STATUS_T WINAPI MSP430_EEM_GetClockControl(CcParameter_t* pCcDestBuffer)
{
	CcParameter2_t tmp;
	STATUS_T ret =  toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->EEM_GetClockControl(&tmp));
	if (ret == STATUS_OK)
	{
		pCcDestBuffer->ccControl = tmp.ccControl;
		pCcDestBuffer->ccGeneralCLK = tmp.ccGeneralCLK;
		pCcDestBuffer->ccModule = tmp.ccModule;
	}
	return ret;
}

STATUS_T WINAPI MSP430_EEM_SetClockControl2(const CcParameter2_t* pCcBuffer)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->SyncedCall()->EEM_SetClockControl(pCcBuffer));
}

STATUS_T WINAPI MSP430_EEM_GetClockControl2(CcParameter2_t* pCcDestBuffer)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->EEM_GetClockControl(pCcDestBuffer));
}

STATUS_T WINAPI MSP430_EEM_SetSequencer(const SeqParameter_t* pSeqBuffer)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->SyncedCall()->EEM_SetSequencer(pSeqBuffer));
}

STATUS_T WINAPI MSP430_EEM_GetSequencer(SeqParameter_t* pSeqDestBuffer)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->EEM_GetSequencer(pSeqDestBuffer));
}

STATUS_T WINAPI MSP430_EEM_ReadSequencerState(SeqState_t* pSeqState)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->SyncedCall()->EEM_ReadSequencerState(pSeqState));
}

STATUS_T WINAPI MSP430_EEM_SetCycleCounterMode(CycleCounterMode_t mode)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->EEM_SetCycleCounterMode(mode));
}

STATUS_T WINAPI MSP430_EEM_ConfigureCycleCounter(uint32_t wCounter, CycleCounterConfig_t pCycConfig)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->EEM_ConfigureCycleCounter(wCounter, pCycConfig));
}

STATUS_T WINAPI MSP430_EEM_ReadCycleCounterValue(uint32_t wCounter, uint64_t* value)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->EEM_ReadCycleCounterValue(wCounter, value));
}

STATUS_T WINAPI MSP430_EEM_WriteCycleCounterValue(uint32_t wCounter, uint64_t value)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->EEM_WriteCycleCounterValue(wCounter, value));
}

STATUS_T WINAPI MSP430_EEM_ResetCycleCounter(uint32_t wCounter)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->EEM_ResetCycleCounter(wCounter));
}

STATUS_T WINAPI MSP430_FET_SelfTest(int32_t count, uint8_t* buffer)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->FET_SelfTest(count, buffer));
}

STATUS_T WINAPI MSP430_FET_SetSignals(int32_t SigMask, int32_t SigState)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->FET_SetSignals(SigMask, SigState));
}

STATUS_T WINAPI MSP430_FET_Reset(void)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->FET_Reset());
}

STATUS_T WINAPI MSP430_FET_I2C(int32_t address, uint8_t* buffer, int32_t count, int32_t rw)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->FET_I2C(address, buffer, count, rw));
}

STATUS_T WINAPI MSP430_FET_EnterBootloader(void)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->FET_EnterBootloader());
}

STATUS_T WINAPI MSP430_FET_ExitBootloader(void)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->FET_ExitBootloader());
}

STATUS_T WINAPI MSP430_FET_GetFwVersion(int32_t* version)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->FET_GetFwVersion(version));
}

STATUS_T WINAPI MSP430_FET_GetHwVersion(uint8_t** version, int32_t* count)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->FET_GetHwVersion(version, count));
}

STATUS_T WINAPI MSP430_FET_FwUpdate(
	const char* lpszFileName,
	DLL430_FET_NOTIFY_FUNC callback,
	int32_t clientHandle
)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->SyncedCall()->FET_FwUpdate(lpszFileName, callback, clientHandle));
}

void WINAPI MSP430_HIL_ResetJtagTap(void)
{
	 if (DLL430_CurrentInstance)
		 DLL430_CurrentInstance->HIL_ResetJtagTap();
}

void WINAPI MSP430_HIL_FuseCheck(void)
{
	 if (DLL430_CurrentInstance)
		 DLL430_CurrentInstance->HIL_FuseCheck();
}


STATUS_T WINAPI MSP430_HIL_Open(void)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->HIL_Open());
}

STATUS_T WINAPI MSP430_HIL_Configure(int32_t mode, int32_t value)
{
	enum CONFIG_MODE m = (enum CONFIG_MODE)mode;
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->HIL_Configure(m, value));
}

STATUS_T WINAPI MSP430_HIL_Connect()
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->HIL_Connect());
}

STATUS_T WINAPI MSP430_HIL_Connect_Entry_State(int32_t value)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->HIL_Connect_Entry_State(value));
}

STATUS_T WINAPI MSP430_HIL_Close(int32_t vccOff)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->HIL_Close(vccOff));
}

STATUS_T WINAPI MSP430_HIL_Bsl(void)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->HIL_Bsl());
}

int32_t WINAPI MSP430_HIL_JTAG_IR(int32_t instruction)
{
	uint64_t retValue = (uint64_t)-1;
	if (DLL430_CurrentInstance)
		retValue = DLL430_CurrentInstance->HIL_JTAG_IR(instruction);

	return (int32_t)retValue;
}

int32_t WINAPI MSP430_HIL_JTAG_IR4(int32_t instruction)
{
	uint64_t retValue = (uint64_t)-1;
	if (DLL430_CurrentInstance)
		retValue = DLL430_CurrentInstance->HIL_JTAG_IR4(instruction);

	return (int32_t)retValue;
}

int32_t WINAPI MSP430_HIL_JTAG_DR(int32_t data, int32_t bits)
{
	uint64_t retValue = (uint64_t)-1;
	if (DLL430_CurrentInstance)
		retValue = DLL430_CurrentInstance->HIL_JTAG_DR(data, bits);

	return (uint32_t)retValue;
}

int64_t WINAPI MSP430_HIL_JTAG_DRX(int64_t data, int32_t bits)
{
	uint64_t retValue = (uint64_t)-1;
	if (DLL430_CurrentInstance)
		retValue = DLL430_CurrentInstance->HIL_JTAG_DR(data, bits);

	return (int64_t)retValue;
}

int64_t WINAPI MSP430_HIL_JTAG_IR_DRX(int32_t instruction, int64_t data, int32_t bits)
{
	uint64_t retValue = (uint64_t)-1;
	if (DLL430_CurrentInstance)
		retValue = DLL430_CurrentInstance->HIL_JTAG_IR_DR(instruction, data, bits);

	return (int64_t)retValue;
}

STATUS_T WINAPI MSP430_HIL_TCK(int32_t state)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->HIL_TCK(state));
}

STATUS_T WINAPI MSP430_HIL_TMS(int32_t state)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->HIL_TMS(state));
}

STATUS_T WINAPI MSP430_HIL_TDI(int32_t state)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->HIL_TDI(state));
}

STATUS_T WINAPI MSP430_HIL_RST(int32_t state)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->HIL_RST(state));
}

STATUS_T WINAPI MSP430_HIL_TST(int32_t state)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->HIL_TST(state));
}

STATUS_T WINAPI MSP430_HIL_DPACC(uint8_t address, uint32_t *data, uint16_t RdnWr)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->HIL_DPACC(address, data, RdnWr));
}

STATUS_T WINAPI MSP430_HIL_APACC(uint8_t portNum, uint8_t address, uint32_t *data, uint16_t RdnWr)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->HIL_APACC(portNum, address, data, RdnWr));
}

STATUS_T WINAPI MSP430_HIL_MEMAP(uint8_t portNum, uint32_t address, uint32_t *data, uint16_t RdnWr)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->HIL_MEMAP(portNum, address, data, RdnWr));
}

STATUS_T WINAPI MSP430_HIL_TCLK_V3(uint8_t value)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->HIL_TCLK(value));
}
STATUS_T WINAPI MSP430_EnableEnergyTrace(const EnergyTraceSetup* setup, const EnergyTraceCallbacks* callbacks, EnergyTraceHandle* handle)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->EnableEnergyTrace(setup, callbacks, handle));
}

STATUS_T WINAPI MSP430_DisableEnergyTrace(const EnergyTraceHandle handle)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->DisableEnergyTrace(handle));
}

STATUS_T WINAPI MSP430_ResetEnergyTrace( const EnergyTraceHandle handle )
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->ResetEnergyTrace(handle));
}

STATUS_T WINAPI MSP430_GetInterface_Type(enum INTERFACE_TYPE* type)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->InterfaceType(type));
}

STATUS_T WINAPI MSP430_RegisterMessageCallback(MessageCallbackFn callback)
{
	return toStatus(DLL430_CurrentInstance && DLL430_CurrentInstance->RegisterMessageCallback(callback));
}

static std::string getFETName(uint8_t *hwVersion)
{
	const uint8_t INFO_U1_HW[] = { 0x55, 0xFF, 40, 1 };
	const uint8_t INFO_EZ_FET_HW[] = { 0x45, 0xFF, 0, 4 };
	const uint8_t INFO_EZ_FET_HW_NO_FLOW_CTS[] = { 0x45, 0xFF, 0, 5 };
	const uint8_t INFO_EZ_FET_LITE_HW[] = { 0x45, 0xFF, 0, 3 };
	const uint8_t INFO_MSP_FET_HW[] = { 0x55, 0xFF, 0, 3 };
	const uint8_t INFO_EZ430[] = { 0x45, 0x46, 0x00, 0x02 };
	const uint8_t INFO_EZ430_1_5[] = { 0x45, 0xFF, 0x00, 0x02 };
	const uint8_t INFO_MSP_FET_HW2x[] = { 0x55, 0xFF, 0, 3 };
	const uint8_t INFO_EZ_FET_HW_V2[] = { 0x45, 0xFF, 0, 6 };

	std::string nameFET;

	if (std::equal(hwVersion, hwVersion + 4, INFO_U1_HW))
	{
		nameFET = "MSP-FET430UIF";
	}
	else if (std::equal(hwVersion, hwVersion + 4, INFO_EZ_FET_HW))
	{
		nameFET = "eZ-FET";
	}
	else if (std::equal(hwVersion, hwVersion + 4, INFO_EZ_FET_HW_V2))
	{
		nameFET = "eZ-FET V2";
	}
	else if (std::equal(hwVersion, hwVersion + 4, INFO_EZ_FET_HW_NO_FLOW_CTS))
	{
		nameFET = "eZ-FET";
	}
	else if (std::equal(hwVersion, hwVersion + 4, INFO_EZ_FET_LITE_HW))
	{
		nameFET = "eZ-FET Lite";
	}
	else if (std::equal(hwVersion, hwVersion + 4, INFO_MSP_FET_HW))
	{
		nameFET = "MSP-FET";
	}
	else if (std::equal(hwVersion, hwVersion + 4, INFO_MSP_FET_HW2x))
	{
		nameFET = "MSP-FET V2";
	}
	else if (std::equal(hwVersion, hwVersion + 4, INFO_EZ430) || std::equal(hwVersion, hwVersion + 4, INFO_EZ430_1_5))
	{
		nameFET = "eZ430";
	}
	else
	{
		nameFET = "Unknown";
	}

	return nameFET;
}

static std::string getLaunchpadName(const std::string& msp, const std::string& fet)
{
	static std::string launchpad;
	static std::map<std::pair<std::string, std::string>, std::string> lutLaunchpads = 
	{
		{std::make_pair("eZ-FET", "MSP430FR4133"), "MSP-EXP430FR4133"},
		{std::make_pair("eZ-FET", "MSP430FR5969"), "MSP-EXP430FR5969"},
		{std::make_pair("eZ-FET", "MSP430FR6989"), "MSP-EXP430FR6989"},
		{std::make_pair("eZ-FET Lite", "MSP430F5529"), "MSP-EXP430F5529LP"},
		{std::make_pair("eZ430", "MSP430G2xx3"), "MSP-EXP430G2"},
		{std::make_pair("eZ-FET V2", "MSP430G2xx3"), "MSP-EXP430G2ET"},
		{std::make_pair("eZ-FET", "MSP430FR2355"), "MSP-EXP430FR2355"},
		{std::make_pair("eZ-FET", "MSP430FR2433"), "MSP-EXP430FR2433"},
		{std::make_pair("eZ-FET", "MSP430FR2311"), "MSP-EXP430FR2311"},
		{std::make_pair("eZ-FET", "MSP430FR5994"), "MSP-EXP430FR5994"},
		{std::make_pair("eZ-FET V2", "MSP430FR2476"), "LP-MSP430FR2476"},
		{std::make_pair("eZ-FET Lite", "MSP430FR6047"), "EVM430-FR6047"}
	};

	if (lutLaunchpads.find(std::make_pair(fet, msp)) != lutLaunchpads.end())
	{
		launchpad = lutLaunchpads[std::make_pair(fet, msp)];
	}

	return launchpad;
}

STATUS_T WINAPI MSP430_Autodetect(struct DETECTIONLIST *list, uint32_t *numElements, uint32_t sizeOfStruct)
{	int32_t numFETs = 0;
	uint32_t nElements = *numElements;

	memset(list, 0x00, nElements * sizeOfStruct);

	if (MSP430_GetNumberOfUsbIfs(&numFETs) == STATUS_ERROR)
	{
		return toStatus(false);
	}

	uint32_t i = 0;
	for (uint32_t idx = 0; idx < static_cast<uint32_t>(numFETs) && i < nElements; ++idx)
	{
		char *name = nullptr;
		int32_t status = 0;
		int32_t version = 0;
		uint8_t *hwVersion = 0;
		int32_t cnt = 0;

		if (MSP430_GetNameOfUsbIf(idx, &name, &status) == STATUS_ERROR)
		{
			list[i].errorCode = MSP430_Error_Number();
			++i;
			continue;
		}

		if (status == ENABLE)
		{
			/* FET already in use */
			continue;
		}

		if (MSP430_Initialize(name, &version) == STATUS_ERROR)
		{
			list[i].errorCode = MSP430_Error_Number();
			++i;
			continue;
		}

		if (MSP430_FET_GetHwVersion(&hwVersion, &cnt) == STATUS_ERROR)
		{
			list[i].errorCode = MSP430_Error_Number();
			++i;
			continue;
		}

		if (version <= 0)
		{
			// update FET FW
			if (MSP430_FET_FwUpdate(NULL, NULL, 0) == STATUS_ERROR)
			{
				list[i].errorCode = MSP430_Error_Number();
				++i;
				continue;
			}
		}

		// check for external power source
		int32_t volt, state;
		if (MSP430_GetExtVoltage(&volt, &state) == STATUS_ERROR)
		{
			list[i].errorCode = MSP430_Error_Number();
			++i;
			continue;
		}

		if (state != EX_POWER_OK)
		{
			if (state == NO_EX_POWER)
			{
				if (MSP430_VCC(3300) == STATUS_ERROR)
				{
					list[i].errorCode = MSP430_Error_Number();
					++i;
					continue;
				}
			}
			else
			{
				// Error: no power
				list[i].errorCode = MSP430_Error_Number();
				++i;
				continue;
			}
		}

		MSP430_SetTargetArchitecture(MSP430);

		if (MSP430_OpenDevice("UNKNOWN", "", 0, 0, DEVICE_UNKNOWN) == STATUS_ERROR)
		{
			/* try MSP432 */
			if (MSP430_SetTargetArchitecture(MSP432_M4) == STATUS_ERROR)
			{
				list[i].errorCode = MSP430_Error_Number();
				++i;
				continue;
			}
			if (MSP430_OpenDevice("UNKNOWN", "", 0, 0, DEVICE_UNKNOWN) == STATUS_ERROR)
			{
				list[i].errorCode = MSP430_Error_Number();
				++i;
				continue;
			}
		}

		DEVICE_T device;
		if (MSP430_GetFoundDevice((uint8_t*)&device, sizeof(device)) == STATUS_ERROR)
		{
			list[i].errorCode = MSP430_Error_Number();
			++i;
			continue;
		}

		uint32_t size;

		size = (sizeof(device.string) > sizeof(list[i].nameDevice)) ? sizeof(list[i].nameDevice) : sizeof(device.string);
		std::copy(device.string, device.string + size, list[i].nameDevice);

		std::string FETName = getFETName(hwVersion);
		size = (FETName.size() > sizeof(list[i].nameFET)) ? sizeof(list[i].nameFET) : FETName.size();
		std::copy(FETName.c_str(), FETName.c_str() + size, list[i].nameFET);

		std::string launchpad = getLaunchpadName(std::string(list[i].nameDevice), FETName);
		if (!launchpad.empty())
		{
			size = (launchpad.size() > sizeof(list[i].nameLaunchpad)) ? sizeof(list[i].nameLaunchpad) : launchpad.size();
			std::copy(launchpad.c_str(), launchpad.c_str() + size, list[i].nameLaunchpad);

		}

		MSP430_Close(1);
		++i;
	}

	*numElements = i;

	return toStatus(true);
}

STATUS_T WINAPI MSP430_GetFETName(char *str, uint32_t size)
{
	uint8_t *hwVersion = 0;
	int32_t cnt = 0;

	if (MSP430_FET_GetHwVersion(&hwVersion, &cnt) == STATUS_ERROR)
	{
		return toStatus(false);
	}

	std::string fet = getFETName(hwVersion);

	uint32_t len = (size > fet.length()) ? fet.length() : size;

	std::copy(fet.c_str(), fet.c_str() + len, str);
	return toStatus(true);
}

STATUS_T WINAPI MSP430_GetLaunchpadName(const char *device, const char *fetname, char *launchpadname, uint32_t size)
{

	std::string launchpad = getLaunchpadName(std::string(device), std::string(fetname));

	if (launchpad.empty())
	{
		return toStatus(true);
	}

	uint32_t len = (size > launchpad.length()) ? launchpad.length() : size;
	std::copy(launchpad.c_str(), launchpad.c_str() + len, launchpadname);
	return toStatus(true);
}