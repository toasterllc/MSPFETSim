/* Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
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
#include "Types.h"
#include "Stream.h"
#include "HIL.h"
#include "EDT.h"

int16_t jtagIdIsValid(uint32_t id)
{
	return id == 0x89 || id == 0x8D || id == 0x91 || id == 0x95 || id == 0x98 || id == 0x99 || id == 0x4ba00477;
}

int16_t jtagIdIsMSP432(uint32_t id)
{
	return id == 0x4ba00477;
}

int16_t jtagIdIsMSP430(uint32_t id)
{
	return id == 0x89 || id == 0x8D || id == 0x91 || id == 0x95 || id == 0x98 || id == 0x99;
}

int16_t jtagIdIsXv2(uint32_t id)
{
	return id == 0x91 || id == 0x95 || id == 0x98 || id == 0x99;
}

uint16_t* getTargetRunningVar(void)
{
    uint16_t* syncWithRunVarAddress;
    if(STREAM_getSharedVariable(ID_SHARED_MEMORY_TYPE_SYNC_RUN, &syncWithRunVarAddress))
    {
        return syncWithRunVarAddress;
    }
    return 0;
}

#define T_ARCH_MSP430 0
#define T_ARCH_MSP432 1

// Event types  send async to DLL First word in message
enum EVENT_TYPE_FLAGS
{
    // set if breakpoint is hit
    BP_HIT_FLAG = 0x1,
    // State Sotrage Event
    STATE_STORAGE_FLAG = 0x2,
    //JSTATE Capute
    JSTATE_CAPTURE_FLAG = 0x4,
	//Power profiling data capture
	ENERGYTRACE_INFO = 0x8,
	// Variable watch event
    VARIABLE_WATCH_FLAG = 0x10
};

#define NO_FLAG			0x00
#define LOCK_INFO_A_FLAG	0x01

struct _DeviceSettings_
{
    uint32_t  clockControlType;
    uint16_t stopFLL;
    uint16_t assertBslValidBit;
};
typedef struct _DeviceSettings_ DeviceSettings;

struct _DevicePowerSettings_
{
    uint32_t powerTestRegMask;
    uint32_t powerTestRegDefault;
    uint32_t enableLpmx5TestReg;
    uint32_t disableLpmx5TestReg;

    uint16_t powerTestReg3VMask;
    uint16_t powerTestReg3VDefault;
    uint16_t enableLpmx5TestReg3V;
    uint16_t disableLpmx5TestReg3V;
};
typedef struct _DevicePowerSettings_ DevicePowerSettings;

typedef struct _ARMConfigSettings
{
    uint32_t scsBase; // System Control Space base address
    uint32_t fpbBase; // FLASH Patch Block base address
    uint32_t interruptOptions; // Options to enable/disable interrupts when single stepping or letting the device run
    uint32_t ulpDebug; // Options to enable/disable entry to LPM. poll for PB hit in low poer mode
} ARMConfigSettings;

//extern void _init_Hal(void);

#define MACRO_LIST                           \
    MACRO(Init)                              \
    MACRO(SetVcc)                            \
    MACRO(GetVcc)                            \
    MACRO(StartJtag)                         \
    MACRO(StartJtagActivationCode)           \
    MACRO(StopJtag)                          \
    MACRO(Configure)                         \
    MACRO(GetFuses)                          \
    MACRO(BlowFuse)                          \
    MACRO(WaitForEem)                        \
    MACRO(BitSequence)                       \
    MACRO(GetJtagId)                         \
    MACRO(SetDeviceChainInfo)                \
    MACRO(SetChainConfiguration)             \
    MACRO(GetNumOfDevices)                   \
    MACRO(GetInterfaceMode)                  \
    MACRO(GetDeviceIdPtr)                    \
    MACRO(SyncJtag_AssertPor_SaveContext)    \
    MACRO(SyncJtag_Conditional_SaveContext)  \
    MACRO(RestoreContext_ReleaseJtag)        \
    MACRO(ReadMemBytes)                      \
    MACRO(ReadMemWords)                      \
    MACRO(ReadMemQuick)                      \
    MACRO(WriteMemBytes)                     \
    MACRO(WriteMemWords)                     \
    MACRO(EemDataExchange)                   \
    MACRO(EemDataExchangeAFE2xx)             \
    MACRO(SingleStep)                        \
    MACRO(ReadAllCpuRegs)                    \
    MACRO(WriteAllCpuRegs)                   \
    MACRO(Psa)                               \
    MACRO(ExecuteFunclet)                    \
    MACRO(ExecuteFuncletJtag)                \
    MACRO(GetDcoFrequency)                   \
    MACRO(GetDcoFrequencyJtag)               \
    MACRO(GetFllFrequency)                   \
    MACRO(GetFllFrequencyJtag)               \
    MACRO(WaitForStorage)                    \
    MACRO(SyncJtag_AssertPor_SaveContextX)   \
    MACRO(SyncJtag_Conditional_SaveContextX) \
    MACRO(RestoreContext_ReleaseJtagX)       \
    MACRO(ReadMemBytesX)                     \
    MACRO(ReadMemWordsX)                     \
    MACRO(ReadMemQuickX)                     \
    MACRO(WriteMemBytesX)                    \
    MACRO(WriteMemWordsX)                    \
    MACRO(EemDataExchangeX)                  \
    MACRO(SingleStepX)                       \
    MACRO(ReadAllCpuRegsX)                   \
    MACRO(WriteAllCpuRegsX)                  \
    MACRO(PsaX)                              \
    MACRO(ExecuteFuncletX)                   \
    MACRO(GetDcoFrequencyX)                  \
    MACRO(GetFllFrequencyX)                  \
    MACRO(WaitForStorageX)                   \
    MACRO(BlowFuseXv2)                       \
    MACRO(BlowFuseFram)                      \
    MACRO(SyncJtag_AssertPor_SaveContextXv2) \
    MACRO(SyncJtag_Conditional_SaveContextXv2)\
    MACRO(RestoreContext_ReleaseJtagXv2)      \
    MACRO(ReadMemWordsXv2)                    \
    MACRO(ReadMemQuickXv2)                    \
    MACRO(WriteMemWordsXv2)                   \
    MACRO(EemDataExchangeXv2)                 \
    MACRO(SingleStepXv2)                      \
    MACRO(ReadAllCpuRegsXv2)                  \
    MACRO(WriteAllCpuRegsXv2)                 \
    MACRO(PsaXv2)                             \
    MACRO(ExecuteFuncletXv2)                  \
    MACRO(UnlockDeviceXv2)                    \
    MACRO(MagicPattern)                       \
    MACRO(UnlockC092)                         \
    MACRO(HilCommand)                         \
    MACRO(PollJStateReg)                      \
    MACRO(PollJStateRegFR57xx)                \
    MACRO(IsJtagFuseBlown)                    \
    MACRO(ResetXv2)                           \
    MACRO(WriteFramQuickXv2)                  \
    MACRO(SendJtagMailboxXv2)                 \
    MACRO(SingleStepJStateXv2)                \
    MACRO(PollJStateRegEt8)                   \
    MACRO(ResetStaticGlobalVars)              \
    MACRO(Reset430I)                          \
    MACRO(PollJStateReg430I)                  \
    MACRO(PollJStateReg20)                    \
    MACRO(SwitchMosfet)                       \
    MACRO(ResetL092)                          \
    MACRO(DummyMacro)                         \
    MACRO(Reset5438Xv2)                       \
    MACRO(LeaSyncConditional)                 \
    MACRO(GetJtagIdCodeArm)                   \
    MACRO(ScanApArm)                          \
    MACRO(MemApTransactionArm)                \
    MACRO(ReadAllCpuRegsArm)                  \
    MACRO(WriteAllCpuRegsArm)                 \
    MACRO(EnableDebugArm)                     \
    MACRO(DisableDebugArm)                    \
    MACRO(RunArm)                             \
    MACRO(HaltArm)                            \
    MACRO(ResetArm)                           \
    MACRO(SingleStepArm)                      \
    MACRO(WaitForDebugHaltArm)                \
    MACRO(MemApTransactionArmSwd)             \
    MACRO(GetInterfaceModeArm)                \
    MACRO(PollDStatePCRegEt)                  \
    MACRO(GetCpuIdArm)                        \
    MACRO(CheckDapLockArm)                    \
    MACRO(UnlockDap)                          \
    MACRO(UssSyncConditional)

#define MACRO(x)  ID_##x,
enum hal_id
{
    MACRO(Zero)
    MACRO_LIST
    NUM_MACROS
};
#undef MACRO


#define HAL_FUNCTIONS_DEFAULT_SIZE    NUM_MACROS
#define HAL_FUNCTIONS_SIZE   (NUM_MACROS+2)

///**
// * HAL Function Prototypes
// */
//#define MACRO(x)  int16_t _hal_##x (uint16_t flags);
//MACRO(Zero)
//MACRO_LIST
//#undef MACRO
//
//void _init_Hal(void);

/**
 * HAL Function Pointer - these are the HAL Exports
 */
// common macros
#define HAL_Zero                              (hal_functions_[ID_Zero].function)
#define HAL_Init                              (hal_functions_[ID_Init].function)
//#define HAL_SetVcc                            (hal_functions_[ID_SetVcc].function)
//#define HAL_GetVcc                            (hal_functions_[ID_GetVcc].function)
#define HAL_StartJtag                         (hal_functions_[ID_StartJtag].function)
#define HAL_StartJtagActivationCode           (hal_functions_[ID_StartJtagActivationCode].function)
#define HAL_StopJtag                          (hal_functions_[ID_StopJtag].function)
#define HAL_Configure                         (hal_functions_[ID_Configure].function)
#define HAL_GetFuses                          (hal_functions_[ID_GetFuses].function)
#define HAL_BlowFuse                          (hal_functions_[ID_BlowFuse].function)
#define HAL_WaitForEem                        (hal_functions_[ID_WaitForEem].function)
#define HAL_BitSequence                       (hal_functions_[ID_BitSequence].function)
#define HAL_GetJtagId                         (hal_functions_[ID_GetJtagId].function)
#define HAL_SetDeviceChainInfo                (hal_functions_[ID_SetDeviceChainInfo].function)
#define HAL_SetChainConfiguration             (hal_functions_[ID_SetChainConfiguration].function)
#define HAL_GetNumOfDevices                   (hal_functions_[ID_GetNumOfDevices].function)
#define HAL_GetInterfaceMode                  (hal_functions_[ID_GetInterfaceMode].function)
#define HAL_MagicPattern                      (hal_functions_[MagicPattern].function)
// MSP430 architecture
#define HAL_SyncJtag_AssertPor_SaveContext    (hal_functions_[ID_SyncJtag_AssertPor_SaveContext].function)
#define HAL_SyncJtag_Conditional_SaveContext  (hal_functions_[ID_SyncJtag_Conditional_SaveContext].function)
#define HAL_RestoreContext_ReleaseJtag        (hal_functions_[ID_RestoreContext_ReleaseJtag].function)
#define HAL_ReadMemBytes                      (hal_functions_[ID_ReadMemBytes].function)
#define HAL_ReadMemWords                      (hal_functions_[ID_ReadMemWords].function)
#define HAL_ReadMemQuick                      (hal_functions_[ID_ReadMemQuick].function)
#define HAL_WriteMemBytes                     (hal_functions_[ID_WriteMemBytes].function)
#define HAL_WriteMemWords                     (hal_functions_[ID_WriteMemWords].function)
#define HAL_EemDataExchange                   (hal_functions_[ID_EemDataExchange].function)
#define HAL_SingleStep                        (hal_functions_[ID_SingleStep].function)
#define HAL_ReadAllCpuRegs                    (hal_functions_[ID_ReadAllCpuRegs].function)
#define HAL_WriteAllCpuRegs                   (hal_functions_[ID_WriteAllCpuRegs].function)
#define HAL_Psa                               (hal_functions_[ID_Psa].function)
#define HAL_ExecuteFunclet                    (hal_functions_[ExecuteFunclet].function)
// MSP430X architecture
#define HAL_SyncJtag_AssertPor_SaveContextX   (hal_functions_[ID_SyncJtag_AssertPor_SaveContextX].function)
#define HAL_SyncJtag_Conditional_SaveContextX (hal_functions_[ID_SyncJtag_Conditional_SaveContextX].function)
#define HAL_RestoreContext_ReleaseJtagX       (hal_functions_[ID_RestoreContext_ReleaseJtagX].function)
#define HAL_ReadMemBytesX                     (hal_functions_[ID_ReadMemBytesX].function)
#define HAL_ReadMemWordsX                     (hal_functions_[ID_ReadMemWordsX].function)
#define HAL_ReadMemQuickX                     (hal_functions_[ID_ReadMemQuickX].function)
#define HAL_WriteMemBytesX                    (hal_functions_[ID_WriteMemBytesX].function)
#define HAL_WriteMemWordsX                    (hal_functions_[ID_WriteMemWordsX].function)
#define HAL_EemDataExchangeX                  (hal_functions_[ID_EemDataExchangeX].function)
#define HAL_SingleStepX                       (hal_functions_[ID_SingleStepX].function)
#define HAL_ReadAllCpuRegsX                   (hal_functions_[ID_ReadAllCpuRegsX].function)
#define HAL_WriteAllCpuRegsX                  (hal_functions_[ID_WriteAllCpuRegsX].function)
#define HAL_PsaX                              (hal_functions_[ID_PsaX].function)
#define HAL_ExecuteFuncletX                   (hal_functions_[ExecuteFuncletX].function)
// CoreIp430Xv2 architecture
#define HAL_SyncJtag_AssertPor_SaveContextXv2   (hal_functions_[ID_SyncJtag_AssertPor_SaveContextXv2].function)
#define HAL_SyncJtag_Conditional_SaveContextXv2 (hal_functions_[ID_SyncJtag_Conditional_SaveContextXv2].function)
#define HAL_RestoreContext_ReleaseJtagXv2       (hal_functions_[ID_RestoreContext_ReleaseJtagXv2].function)
#define HAL_ReadMemBytesXv2                     (hal_functions_[ID_ReadMemBytesXv2].function)
#define HAL_ReadMemWordsXv2                     (hal_functions_[ID_ReadMemWordsXv2].function)
#define HAL_ReadMemQuickXv2                     (hal_functions_[ID_ReadMemQuickXv2].function)
#define HAL_WriteMemBytesXv2                    (hal_functions_[ID_WriteMemBytesXv2].function)
#define HAL_WriteMemWordsXv2                    (hal_functions_[ID_WriteMemWordsXv2].function)
#define HAL_EemDataExchangeXv2                  (hal_functions_[ID_EemDataExchangeXv2].function)
#define HAL_SingleStepXv2                       (hal_functions_[ID_SingleStepXv2].function)
#define HAL_ReadAllCpuRegsXv2                   (hal_functions_[ID_ReadAllCpuRegsXv2].function)
#define HAL_WriteAllCpuRegsXv2                  (hal_functions_[ID_WriteAllCpuRegsXv2].function)
#define HAL_PsaXv2                              (hal_functions_[ID_PsaXv2].function)
#define HAL_ExecuteFuncletXv2                   (hal_functions_[ID_ExecuteFuncletXv2].function)
#define HAL_UnlockDeviceXv2                     (hal_functions_[ID_UnlockDeviceXv2].function)
#define HAL_UnlockC092                          (hal_functions_[ID_UnlockC092].function)
#define HAL_HilCommand                          (hal_functions_[ID_HilCommand].function)
#define HAL_PollJStateRegFR57xx                 (hal_functions_[ID_PollJStateRegFR57xx].function)
#define Hal_PollDStatePCRegEt                   (hal_functions_[ID_PollDStatePCRegEt].function)
#define Hal_PollJStateRegEt8                    (hal_functions_[ID_PollJStateRegEt8].function)
#define Hal_PollJStateReg                       (hal_functions_[ID_PollJStateReg].function)

#define HAL_IsJtagFuseBlown                     (hal_functions_[ID_IsJtagFuseBlown].function)
#define HAL_ResetXv2                            (hal_functions_[ID_ResetXv2].function)


#define HAL_GetDcoFrequency                     (hal_functions_[ID_GetDcoFrequency].function)
#define HAL_GetFllFrequency                     (hal_functions_[ID_GetFllFrequency].function)
#define HAL_WriteFramQuickXv2                   (hal_functions_[ID_WriteFramQuickXv2].function)
#define HAL_SendJtagMailboxXv2                  (hal_functions_[ID_SendJtagMailboxXv2].function)

#define HAL_ReadAllCpuRegsNon1377Xv2            (hal_functions_[ID_ReadAllCpuRegsNon1377Xv].function)
#define HAL_SingleStepJStateXv2                 (hal_functions_[ID_SingleStepJStateXv2].function)
#define HAL_ResetMsp430I                        (hal_functions_[ID_ResetMsp430I].function)
#define HAL_PollMSP430I40xx                     (hal_functions_[ID_PollMSP430I40xx].function)

#define HAL_LeaSyncConditional                  (hal_functions_[ID_LeaSyncConditional].function)

#define HAL_SIGNATURE 0xBEEFBEEFul

//extern void globalVarsInit(void);

REQUIRED(_edt_Common_Methods_HAL)
edt_common_methods_t  _edt_Common_Methods_HAL = {};

REQUIRED(_edt_Distinct_Methods_HAL)
edt_distinct_methods_t _edt_Distinct_Methods_HAL = {};

//extern int16_t _hil_Init( void );
//extern uint32_t _hal_mclkCntrl0;

// prototypes for core/HAL interface
//void *ResetFirmware(void *stream_adr, uint32_t device_flags, uint8_t v3opHilCrcOk, uint8_t v3opDcdcCcOk);

CONST_AT(HAL_INFOS hal_infos_, 0x1904) =
{
  MEMBER_FN_PTR(ResetFirmware),                // _initTask
  (int16_t)((VERSION_MAJOR - 1) << 14 | (VERSION_MINOR << 8) | VERSION_PATCH),
  VERSION_BUILD
};
REQUIRED(hal_infos_)

const uint32_t hal_Signature_ = HAL_SIGNATURE;
//#pragma required = hal_Signature_

////! \brief Pointer to HAL IRQ vector table
//RO_PLACEMENT_NO_INIT volatile const uint16_t hil_Start_UP_;
//RO_PLACEMENT_NO_INIT volatile const uint32_t  hil_signature_;
//RO_PLACEMENT_NO_INIT volatile const uint16_t  hil_version_;
//RO_PLACEMENT_NO_INIT volatile const uint16_t  hil_versionCmp_;


int16_t _dummy_Init( void ){return 0;};
int16_t _dummy_SetVcc(uint16_t Vcc){return 0;};
void _dummy_SwitchVccFET(uint16_t state) {return;};
int16_t _dummy_GetVcc(double* Vcc, double* ExtVcc){return 0;};

int16_t _dummy_SetProtocol(uint16_t protocol_id){return 0;};
void  _dummy_SetPsaSetup(uint16_t enhanced){return;};
void  _dummy_SetPsaTCLK(uint16_t tclkValue){return;};

int16_t _dummy_Open( uint8_t state ){return 0;};
int16_t _dummy_Close( void ){return 0;};
int16_t _dummy_IccMonitor_Process(uint16_t flags){return 0;}; // flags: to be compatible with HIL calls
void  _dummy_EntrySequences(uint8_t states){return;};

void _dummy_SetReset(uint8_t value){return;};
void _dummy_SetTest(uint8_t value){return;};
void _dummy_SetTMS(uint8_t value){return;};
void _dummy_SetTDI(uint8_t value){return;};
void _dummy_SetTCK(uint8_t value){return;};

void _dummy_SetJtagSpeed(uint16_t jtagSpeed, uint16_t sbwSpeed){return;};
void _dummy_ConfigureSetPc (uint16_t PCclockBeforeCapture){return;};

void _dummy_initDelayTimer(void) {return;};
int16_t _dummy_regulateVcc(void) {return 0;};
uint16_t _dummy_getFpgaVersion(void) {return 0;};

void _dummy(uint16_t dummy){return;};

int16_t _dummy_TapReset_Dma(void){return 0;}
int16_t _dummy_CheckJtagFuse_Dma(void){return 0;}
SBWShiftProxy<uint8_t> _dummy_Instr_Dma(uint8_t Instruction){return 0;}
SBWShiftProxy<uint8_t> _dummy_SetReg_XBits08_Dma(uint8_t Data){return 0;}
SBWShiftProxy<uint16_t> _dummy_SetReg_XBits16_Dma(uint16_t Data){return 0;}
SBWShiftProxy<uint32_t,20> _dummy_SetReg_XBits20_Dma(uint32_t Data){return 0;}
SBWShiftProxy<uint32_t> _dummy_SetReg_XBits32_Dma(uint32_t Data){return 0;}
SBWShiftProxy<uint64_t> _dummy_SetReg_XBits64_Dma(uint64_t Data){return 0;}
uint64_t _dummy_SetReg_XBits8_64_Dma(uint64_t Data, uint16_t loopCount, uint16_t Pg){return 0;}
void  _dummy_Tclk_Dma(uint8_t state){return;}
void  _dummy_StepPsa_Dma(uint32_t length){return;}
void  _dummy_StepPsaTclkHigh_Dma(uint32_t length){return;}
int16_t _dummy_BlowFuse_Dma(uint8_t targetHasTestVpp){return 0;}
void  _dummy_ConfigureSpeed_Dma(uint16_t speed){return;}
void  _dummy_initTimerB0(void){return;}
void  _dummy_BSL_EntrySequence(uint16_t switchBypassOff){return;}
uint8_t _dummy_GetPrevInstruction (void){return 0;}
void  _dummy_ReadADC12(void){return;}
void _dummy_ConfigFpgaIoMode(uint16_t mode){return;}

uint64_t _dummy_SetReg_XBits(uint64_t *data, uint16_t count){return 0;}
SBWShiftProxy<uint8_t> _dummy_Instr_4(uint8_t Data){return 0;}

int16_t _dummy_write_read_Dp(uint8_t address, uint32_t *data, uint16_t rnw) {return -1;}
int16_t _dummy_write_read_Ap(uint32_t address, uint32_t *data, uint16_t rnw) {return -1;}
int16_t _dummy_write_read_mem_Ap(uint16_t ap_sel, uint32_t address, uint32_t *data, uint16_t rnw) {return -1;}

void _dummy_setFpgaTimeOut(uint16_t state) {return;};

struct stream_funcs *_stream_Funcs = nullptr;

HAL_INFOS hal_infos_in_ram_ = {};

void globalVarsInit(void)
{
    memset(&hal_infos_in_ram_, 0, sizeof(HAL_INFOS));

    //SyncJtagAssertPor_SaveContextXv2.c
    memset(mclk_modules, 0, sizeof(mclk_modules));
    memset(cswValues, 0, sizeof(cswValues));
    armConfigSettings.scsBase = 0xE000E000; // Default location of the SCS
    activeDevice = 0;
    numOfDevices = 0;
    TCE = 0;
    _hal_mclkCntrl0 = 0x0417;
    // Disable device power settings by default
    memset(&devicePowerSettings, 0, sizeof(devicePowerSettings));
}

// global variables to handle JTAG chain
uint16_t activeDevice = 0;
REQUIRED(activeDevice)

uint8_t numOfDevices = 0;
REQUIRED(numOfDevices)

uint16_t TCE = 0;
REQUIRED(TCE)

DevicePowerSettings devicePowerSettings = {};
REQUIRED(devicePowerSettings)

DeviceSettings deviceSettings = {};
REQUIRED(deviceSettings)

// global variables to handle Emulation clock control
uint32_t _hal_mclkCntrl0 = 0;
REQUIRED(_hal_mclkCntrl0)

HalRec hal_functions_[HAL_FUNCTIONS_SIZE] = {};

// Unused, but also defined by HIL.h, where it is used
//uint16_t setPCclockBeforeCapture = 0;
//REQUIRED(setPCclockBeforeCapture)

uint16_t altRomAddressForCpuRead = 0;
REQUIRED(altRomAddressForCpuRead)

uint16_t wdtctlAddress5xx = 0x15C;
REQUIRED(wdtctlAddress5xx)

uint16_t enhancedPsa = 0;
REQUIRED(enhancedPsa)

int8_t traceBuffer[256] = {};              ///< Trace buffer used to print out JTAG IR and DR shofts to debug port

// global variables use for JTAG/SWD DAP
uint32_t cswValues[4] = {};
REQUIRED(cswValues)

ARMConfigSettings armConfigSettings = {};
REQUIRED(armConfigSettings)

#ifndef __data20
#define __data20
#endif

#define MACRO(x)  {ID_##x, &MSPFETSim::_hal_##x },
HalRec hal_functions_default_[HAL_FUNCTIONS_DEFAULT_SIZE] =
{
    MACRO(Zero)
    MACRO_LIST
};
#undef MACRO

REQUIRED(_init_Hal)


void _init_Hal(void)
{
    uint8_t i;

    for(i=0; i < (sizeof(hal_functions_)/sizeof(HalRec)); i++)
    {
        if(i < (sizeof(hal_functions_default_)/sizeof(HalRec)))
        {
            hal_functions_[i].id = hal_functions_default_[i].id;
            hal_functions_[i].function = hal_functions_default_[i].function;
        }
        else
        {
            hal_functions_[i].id = 0xFFFF;
            hal_functions_[i].function = NULL;
        }
    }
}

HAL_FUNCTION(_hal_Zero)
{
    return 0;
}

HAL_FUNCTION(_hal_Init)
{
    //EDT_Init();
    return 0;
}

//extern uint16_t LPMx5_DEVICE_STATE;
//extern int16_t intstate;
//extern uint64_t prevJState;
//extern uint16_t lastTraceWritePos;


HAL_FUNCTION(_hal_ResetStaticGlobalVars)
{
    LPMx5_DEVICE_STATE = 1;
    intstate = 0;
    prevJState = 0x0000000000000000;
    lastTraceWritePos = 0;
    return 0;
}

HAL_FUNCTION(_hal_SetVcc)
{
    uint16_t vcc;
    STREAM_get_word(&vcc);
    IHIL_SetVcc(vcc);
    return 0;
}


HAL_FUNCTION(_hal_SwitchMosfet)
{
#ifdef  MSP_FET
    uint16_t on = 0;
    STREAM_get_word(&on);
    if (on)
    {
        IHIL_SwitchVccFET(1);
    }
    else
    {
        IHIL_SwitchVccFET(0);
    }
    return 0;
#else
    return -1;
#endif
}


HAL_FUNCTION(_hal_GetVcc)
{
    double vcc = 0;
    double ext_vcc = 0;

    IHIL_GetVcc(&vcc, &ext_vcc);
    STREAM_put_word((uint16_t)vcc);
    STREAM_put_word((uint16_t)ext_vcc);
    return 0;
}

HAL_FUNCTION(_hal_GetFuses)
{
    config_fuses();
    STREAM_put_byte((uint8_t)SetReg_8Bits(0));
    return 0;
}

HAL_FUNCTION(_hal_DummyMacro)
{
    return -1;
}

// called via cstartup form BIOS_HalInterfaceInit
REQUIRED(ResetFirmware)
void *ResetFirmware(struct stream_funcs* stream_adr, uint32_t device_flags, uint8_t v3opHilCrcOk, uint8_t v3opDcdcCcOk)
{
    globalVarsInit();
    memcpy((HAL_INFOS*)&hal_infos_in_ram_,&hal_infos_, sizeof(hal_infos_));
    // save address of stream funcs, located in core
     _stream_Funcs = stream_adr;
    _init_Hal();
    hal_infos_in_ram_.swCmp_0 = (VERSION_MAJOR_CMP - 1) << 14 | (VERSION_MINOR_CMP << 8) | VERSION_PATCH_CMP;
    hal_infos_in_ram_.swCmp_1 = VERSION_BUILD_CMP;

    {
       // check if we have a valid hil layer programmed into our tool
        if(hil_signature_ == 0xF00DF00D && hil_Start_UP_ && v3opHilCrcOk)
        {
            // MSPFETSim: not sure what this craziness is, but its result is calling `_hil_getEdtCommen`
            // INIT HIL layer
//            HilInitGetEdtCommenFunc hilEdtCom = NULL;
//            // set pointer to edt commen functions
//            hilEdtCom = (HilInitGetEdtCommenFunc)0x18A0;
            _hil_getEdtCommen(&_edt_Common_Methods_HAL);
            hal_infos_in_ram_.hil_version = hil_version_;
            hal_infos_in_ram_.hil_versionCmp = hil_versionCmp_;
#ifdef eZ_FET
            if(v3opDcdcCcOk)
            {
                _edt_Common_Methods_HAL.SwitchVccFET(LDO_ON);
            }
#endif


#ifdef MSP_FET
            // Force to not update for Beta1
            hal_infos_in_ram_.fpga_version          = CALL_MEMBER_FN_PTR(_edt_Common_Methods_HAL.getFpgaVersion)();
            // power up VCC as FET power supply
            if(v3opDcdcCcOk)
            {
                CALL_MEMBER_FN_PTR(_edt_Common_Methods_HAL.SetVcc)(3300);
                CALL_MEMBER_FN_PTR(_edt_Common_Methods_HAL.SwitchVccFET)(1);
            }
#else
            hal_infos_in_ram_.fpga_version = 0;
#endif
        }
        else
        {
            // if hil layer is not valid or no startup address was found
            hal_infos_in_ram_.hil_version = 0;
            hal_infos_in_ram_.fpga_version = 0;

            _edt_Common_Methods_HAL.Init                 = MEMBER_FN_PTR(_dummy_Init);
            _edt_Common_Methods_HAL.SetVcc               = MEMBER_FN_PTR(_dummy_SetVcc);
            _edt_Common_Methods_HAL.GetVcc               = MEMBER_FN_PTR(_dummy_GetVcc);
            _edt_Common_Methods_HAL.SetProtocol          = MEMBER_FN_PTR(_dummy_SetProtocol);
            _edt_Common_Methods_HAL.SetPsaTCLK           = MEMBER_FN_PTR(_dummy_SetPsaTCLK);
            _edt_Common_Methods_HAL.Open                 = MEMBER_FN_PTR(_dummy_Open);
            _edt_Common_Methods_HAL.Close                = MEMBER_FN_PTR(_dummy_Close);
            _edt_Common_Methods_HAL.Delay_1us            = MEMBER_FN_PTR(_dummy);
            _edt_Common_Methods_HAL.Delay_1ms            = MEMBER_FN_PTR(_dummy);
            _edt_Common_Methods_HAL.Loop                 = MEMBER_FN_PTR(_dummy_IccMonitor_Process);
            _edt_Common_Methods_HAL.EntrySequences       = MEMBER_FN_PTR(_dummy_EntrySequences);
            _edt_Common_Methods_HAL.SetReset             = MEMBER_FN_PTR(_dummy_SetReset);
            _edt_Common_Methods_HAL.SetTest              = MEMBER_FN_PTR(_dummy_SetTest);
            _edt_Common_Methods_HAL.SetTMS               = MEMBER_FN_PTR(_dummy_SetTMS);
            _edt_Common_Methods_HAL.SetTDI               = MEMBER_FN_PTR(_dummy_SetTDI);
            _edt_Common_Methods_HAL.SetTCK               = MEMBER_FN_PTR(_dummy_SetTCK);
            _edt_Common_Methods_HAL.SetJtagSpeed         = MEMBER_FN_PTR(_dummy_SetJtagSpeed);
            _edt_Common_Methods_HAL.ConfigureSetPc       = MEMBER_FN_PTR(_dummy_ConfigureSetPc);
            _edt_Common_Methods_HAL.initDelayTimer       = MEMBER_FN_PTR(_dummy_initDelayTimer);
            _edt_Common_Methods_HAL.regulateVcc          = MEMBER_FN_PTR(_dummy_regulateVcc);
            _edt_Common_Methods_HAL.getFpgaVersion       = MEMBER_FN_PTR(_dummy_getFpgaVersion);
            _edt_Common_Methods_HAL.BSL_EntrySequence    = MEMBER_FN_PTR(_dummy_BSL_EntrySequence);
            _edt_Common_Methods_HAL.SwitchVccFET         = MEMBER_FN_PTR(_dummy_SwitchVccFET);
            _edt_Common_Methods_HAL.setFpgaTimeOut       = MEMBER_FN_PTR(_dummy_setFpgaTimeOut);
            _edt_Common_Methods_HAL.regulateVcc          = MEMBER_FN_PTR(_dummy_regulateVcc);
            _edt_Common_Methods_HAL.ReadADC12            = MEMBER_FN_PTR(_dummy_ReadADC12);
            _edt_Common_Methods_HAL.ConfigFpgaIoMode     = MEMBER_FN_PTR(_dummy_ConfigFpgaIoMode);

            _edt_Distinct_Methods_HAL.TapReset           = MEMBER_FN_PTR(_dummy_TapReset_Dma);
            _edt_Distinct_Methods_HAL.CheckJtagFuse      = MEMBER_FN_PTR(_dummy_CheckJtagFuse_Dma);
            _edt_Distinct_Methods_HAL.Instr              = MEMBER_FN_PTR(_dummy_Instr_Dma);
            _edt_Distinct_Methods_HAL.SetReg_XBits08     = MEMBER_FN_PTR(_dummy_SetReg_XBits08_Dma);
            _edt_Distinct_Methods_HAL.SetReg_XBits16     = MEMBER_FN_PTR(_dummy_SetReg_XBits16_Dma);
            _edt_Distinct_Methods_HAL.SetReg_XBits20     = MEMBER_FN_PTR(_dummy_SetReg_XBits20_Dma);
            _edt_Distinct_Methods_HAL.SetReg_XBits32     = MEMBER_FN_PTR(_dummy_SetReg_XBits32_Dma);
            _edt_Distinct_Methods_HAL.SetReg_XBits64     = MEMBER_FN_PTR(_dummy_SetReg_XBits64_Dma);
            _edt_Distinct_Methods_HAL.SetReg_XBits8_64   = MEMBER_FN_PTR(_dummy_SetReg_XBits8_64_Dma);
            _edt_Distinct_Methods_HAL.Tclk               = MEMBER_FN_PTR(_dummy_Tclk_Dma);
            _edt_Distinct_Methods_HAL.StepPsa            = MEMBER_FN_PTR(_dummy_StepPsaTclkHigh_Dma);
            _edt_Distinct_Methods_HAL.BlowFuse           = MEMBER_FN_PTR(_dummy_BlowFuse_Dma);
            _edt_Distinct_Methods_HAL.GetPrevInstruction = MEMBER_FN_PTR(_dummy_GetPrevInstruction);
            _edt_Distinct_Methods_HAL.SetReg_XBits       = MEMBER_FN_PTR(_dummy_SetReg_XBits);
            _edt_Distinct_Methods_HAL.Instr04            = MEMBER_FN_PTR(_dummy_Instr_4);
            _edt_Distinct_Methods_HAL.write_read_Dp      = MEMBER_FN_PTR(_dummy_write_read_Dp);
            _edt_Distinct_Methods_HAL.write_read_Ap      = MEMBER_FN_PTR(_dummy_write_read_Ap);
            _edt_Distinct_Methods_HAL.write_read_mem_Ap  = MEMBER_FN_PTR(_dummy_write_read_mem_Ap);
        }
    }

    _hal_mclkCntrl0=0x040f;
    hal_infos_in_ram_.hal_size = sizeof(hal_functions_)/sizeof(HalRec);
    hal_infos_in_ram_.hal_list_ptr = hal_functions_;
    return((void*)&hal_infos_in_ram_); // return software infos
}

uint16_t hal_GetHilVersion()
{
    return 0;
}






/**
* \ingroup MODULMACROS
*
* \file BitSequence.c
*
* \brief Streaming of single JTAG signals
*/

/**
  BitSequence
  Streaming of single JTAG signals. Used to manually set the level of one or
  more JTAG signal to a dedicated level. E.g. can be used to manually pull
  the RST pin low.
  inData:  <count(8)> (<writeBits(16)> <maskBits(16)> <delay(16)>){*}
  outData: -

          count: number of following bit manipulations
          maskBits: the mask to apply when setting bits
          writeBits: the values to set the bits to
  Bit definition:
          Bit 0 - TMS
          Bit 1 - TDI
          Bit 2 - TDO (writing this bit has no effect since its an input)
          Bit 3 - TCK
          Bit 4 - ------
          Bit 5 - SELT#
          Bit 6 - TGTRST
          Bit 7 - ------
          Bit 8 - TEST
          Bit 9 - ------
          Bit 10- ENTDI2TDO
          Bit 11- VCCTON
          Bit 12- TDIOFF
          Bit 13- VF2TDI
          Bit 14- VF2TEST
          Bit 15- ------
*/

#define PinTMS  0x01             //Bit0
#define PinTDI  0x02             //Bit1
#define PinTDO  0x04             //Bit2
#define PinTCK  0x08             //Bit3
#define PinTGTRST 0x040          //Bit6
#define PinTest 0x0100           //Bit8

HAL_FUNCTION(_hal_BitSequence)
{
    uint8_t numBitSequences;
    uint16_t writeBits;
    uint16_t maskBits;
    uint16_t delay;

    if(STREAM_get_byte(&numBitSequences) < 0)
    {
        return HALERR_NO_NUM_BITS;
    }

    while(numBitSequences)
    {
        if(STREAM_get_word(&writeBits) < 0)
        {
            return HALERR_ARRAY_SIZE_MISMATCH;
        }

        if(STREAM_get_word(&maskBits) < 0)
        {
            return HALERR_ARRAY_SIZE_MISMATCH;
        }

        if(STREAM_get_word(&delay) < 0)
        {
            return HALERR_ARRAY_SIZE_MISMATCH;
        }

        // Set reset bit using macros - automatically handles SBW/JTAG mode
        if(maskBits & PinTGTRST)
        {
            if(writeBits & PinTGTRST)
            {
                IHIL_SetReset(1);
            }
            else
            {
                IHIL_SetReset(0);
            }
        }

        // Set test bit using macros - automatically handles SBW/JTAG mode
        if(maskBits & PinTest)
        {
            if(writeBits & PinTest)
            {
                IHIL_SetTest(1);
            }
            else
            {
                IHIL_SetTest(0);
            }
        }

        // Set TMS bit using macros - automatically handles SBW/JTAG mode
        if(maskBits & PinTMS)
        {
            if(writeBits & PinTMS)
            {
                IHIL_SetTMS(1);
            }
            else
            {
                IHIL_SetTMS(0);
            }
        }

        // Set TCK bit using macros - automatically handles SBW/JTAG mode
        if(maskBits & PinTCK)
        {
            if(writeBits & PinTCK)
            {
                IHIL_SetTCK(1);
            }
            else
            {
                IHIL_SetTCK(0);
            }
        }

        // Set TDI bit using macros - automatically handles SBW/JTAG mode
        if(maskBits & PinTDI)
        {
            if(writeBits & PinTDI)
            {
                IHIL_SetTDI(1);
            }
            else
            {
                IHIL_SetTDI(0);
            }
        }


        --numBitSequences;

        IHIL_Delay_1ms(delay);
    }
    return 0;
}

/**
* \ingroup MODULMACROS
*
* \file BlowFuse.c
*
* \brief Blow JTAG security fuse
*/

/**
  BlowFuse
  Blow JTAG security fuse.
  After calling this function JTAG access is not possible anymore.
  inData:  <tgtHasTest(8)>
  outData: -
  tgtHasTest: specifies if target device has TEST pin or not (bool)
*/

#if defined(MSP430_UIF) || defined(MSP_FET)
HAL_FUNCTION(_hal_BlowFuse)
{
    int16_t ret_value = HALERR_UNDEFINED_ERROR;
    uint8_t hasTestVpp = 0;

    if ( STREAM_get_byte(&hasTestVpp) != -1 )
    {
        IHIL_BlowFuse(hasTestVpp);
        ret_value = 0;
    }
    return ret_value;
}
#endif

// Fuse blow disabled for of E-FET
#ifdef eZ_FET
HAL_FUNCTION(_hal_BlowFuse)
{
    // fuseblow is not supported on eZ-FET
    return API_CALL_NOT_SUPPORTED;
}
#endif
/**
* \ingroup <FILEGROUP>
*
* \file BlowFuseFram.c
*
* \brief Blow JTAG security fuse
*/

/**
  BlowFuse
  Blow JTAG security fuse.
  After calling this function JTAG access is not possible anymore.
  inData:  <tgtHasTest(8)>
  outData: -
  tgtHasTest: specifies if target device has TEST pin or not (bool)
*/

//! \todo implementation & rename to jtag access protection
#if defined(MSP430_UIF) || defined(MSP_FET)
HAL_FUNCTION(_hal_BlowFuseFram)
{
    int16_t ret_value = 0;
    uint16_t lockKey[2] = {0x5555,0x5555};

    // Write password into device
    WriteMemWordXv2(0xFF80,lockKey[0]);
    WriteMemWordXv2(0xFF82,lockKey[1]);

    // now perform a BOR via JTAG - we loose control of the device then...
    test_reg();
    SetReg_32Bits(0x00000200);

    return ret_value;
}
#endif

#ifdef eZ_FET
HAL_FUNCTION(_hal_BlowFuseFram)
{
    // fuseblow is not supported on eZ-FET
    return API_CALL_NOT_SUPPORTED;
}
#endif
/**
* \ingroup <FILEGROUP>
*
* \file BlowFuseXv2.c
*
* \brief Blow JTAG security fuse
*/

/**
  BlowFuse
  Blow JTAG security fuse.
  After calling this function JTAG access is not possible anymore.
  inData:  <tgtHasTest(8)>
  outData: -
  tgtHasTest: specifies if target device has TEST pin or not (bool)
*/

uint32_t JTAGLock5xx = 0xCACACACA;

//! \todo implementation & rename to jtag access protection
#if defined(MSP430_UIF) || defined(MSP_FET)
HAL_FUNCTION(_hal_BlowFuseXv2)
{
    int16_t ret_value = 0;
    int16_t bslValue = 0;
    StreamSafe stream_tmp;
    uint8_t stream_in_tmp[26];
    uint16_t i =0;

    WriteMemWordXv2(0x182, 0x0003);
    bslValue = ReadMemWordXv2(0x182);

    if( (bslValue & 0x8000) == 0 )
    {
        // Coppy incoming stream to forward it to HAL_ExecuteFuncletXv2
        for(i=0;i<(sizeof(stream_in_tmp) - 4);i++)
        {
            STREAM_get_byte(&stream_in_tmp[i]);
        }
        // add Password to stream
        stream_in_tmp[i]   = (uint8_t) (JTAGLock5xx & 0xFF);
        stream_in_tmp[i+1] = (uint8_t) ((JTAGLock5xx >> 8) & 0xFF);
        stream_in_tmp[i+2] = (uint8_t) ((JTAGLock5xx >> 16) & 0xFF);
        stream_in_tmp[i+3] = (uint8_t) ((JTAGLock5xx >> 24) & 0xFF);

        // Write password into device
        STREAM_internal_stream(stream_in_tmp, sizeof(stream_in_tmp), MESSAGE_OUT_TO_DLL, 0, &stream_tmp);
        (this->*HAL_ExecuteFuncletXv2)(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG); // Data from DLL Stream
        STREAM_external_stream(&stream_tmp);

        // now perform a BOR via JTAG - we loose control of the device then...
        test_reg();
        SetReg_32Bits(0x00000200);
    }
    else
    {
         ret_value = HALERR_UNDEFINED_ERROR;
    }
    return ret_value;
}
#endif

#ifdef eZ_FET
HAL_FUNCTION(_hal_BlowFuseXv2)
{
    // fuseblow is not supported on eZ-FET
    return API_CALL_NOT_SUPPORTED;
}
#endif
/**
* \ingroup MODULMACROS
*
* \file Configure.c
*
* \brief Set certain parameters to spcified values
*/

/**
  Configure
  Configures specified parameter to the specified value
  inData:  <configureParameter(32) value(32)>
  outData: <>
  configureParameter: contains the parameter to be modified
  value: specifies the vlaue to which to set the parameter
*/

//#if defined(MSP430_UIF)
//    #ifdef ARCH_MSP432
//        extern ARMConfigSettings armConfigSettings;
//    #endif
//#endif
//
//#if defined(MSP_FET)
//        extern ARMConfigSettings armConfigSettings;
//#endif
//
//extern ARMConfigSettings armConfigSettings;
//
//extern DeviceSettings deviceSettings;
//extern DevicePowerSettings devicePowerSettings;
//extern uint16_t altRomAddressForCpuRead;
//extern uint16_t wdtctlAddress5xx;
//extern uint16_t assertBslValidBit;
//extern uint16_t enhancedPsa;

HAL_FUNCTION(_hal_Configure)
{
    uint32_t configureParameter;
    uint32_t value;

    // Retrieve parameters from stream
    if(STREAM_get_long(&configureParameter) < 0)
    {
        return (HALERR_CONFIG_NO_PARAMETER);
    }

    if(STREAM_get_long(&value) < 0)
    {
        return (HALERR_CONFIG_NO_VALUE);
    }

    // Decode and parameter and set value
    switch(configureParameter)
    {
    case (CONFIG_PARAM_ENHANCED_PSA):
        enhancedPsa = value;
        break;

    case (CONFIG_PARAM_PSA_TCKL_HIGH):
        IHIL_SetPsaTCLK(value);
        break;

    case (CONFIG_PARAM_CLK_CONTROL_TYPE):
        deviceSettings.clockControlType = value;
        break;

    case (CONFIG_PARAM_DEFAULT_CLK_CONTROL):
        break;

    case (CONFIG_PARAM_POWER_TESTREG_MASK):
        devicePowerSettings.powerTestRegMask = value;
        break;

    case (CONFIG_PARAM_POWER_TESTREG_DEFAULT):
        devicePowerSettings.powerTestRegDefault = value;
        break;

    case (CONFIG_PARAM_TESTREG_ENABLE_LPMX5):
        devicePowerSettings.enableLpmx5TestReg = value;
        break;

    case (CONFIG_PARAM_TESTREG_DISABLE_LPMX5):
        devicePowerSettings.disableLpmx5TestReg = value;
        break;

    case (CONFIG_PARAM_POWER_TESTREG3V_MASK):
        devicePowerSettings.powerTestReg3VMask = value;
        break;

    case (CONFIG_PARAM_POWER_TESTREG3V_DEFAULT):
        devicePowerSettings.powerTestReg3VDefault = value;
        break;

    case (CONFIG_PARAM_TESTREG3V_ENABLE_LPMX5):
        devicePowerSettings.enableLpmx5TestReg3V = value;
        break;

    case (CONFIG_PARAM_TESTREG3V_DISABLE_LPMX5):
        devicePowerSettings.disableLpmx5TestReg3V = value;
        break;

    case (CONFIG_PARAM_JTAG_SPEED):
    {
        uint32_t sbwValue = 0;
        if(STREAM_get_long(&sbwValue) < 0)
        {
            return (HALERR_CONFIG_NO_VALUE);
        }
        IHIL_SetJtagSpeed(value, sbwValue);
    break;
    }

    case (CONFIG_PARAM_SFLLDEH):
        deviceSettings.stopFLL = value;
        break;

    case (CONFIG_ALT_ROM_ADDR_FOR_CPU_READ):
        altRomAddressForCpuRead = value;
        break;

    case (CONFIG_WDT_ADDRESS_5XX):
        wdtctlAddress5xx = value;
        break;

    case (CONFIG_ASSERT_BSL_VALID_BIT):
        deviceSettings.assertBslValidBit = value;
        break;

    case (CONFIG_PARAM_SCS_BASE_ADDRESS):
#if defined(MSP430_UIF) || defined(MSP_FET)
    #ifdef ARCH_MSP432
        armConfigSettings.scsBase = value;
    #endif
#endif
        break;

    case (CONFIG_PARAM_FPB_BASE_ADDRESS):
#if defined(MSP430_UIF) || defined(MSP_FET)
    #ifdef ARCH_MSP432
        armConfigSettings.fpbBase = value;
    #endif
#endif
        break;

    case (CONFIG_PARAM_INTERRUPT_OPTIONS):
#if defined(MSP430_UIF) || defined(MSP_FET)
  #ifdef ARCH_MSP432
        armConfigSettings.interruptOptions = value | 0x80;
  #endif
#endif
        break;

    case (CONFIG_PARAM_ULP_MSP432):
#if defined(MSP430_UIF) || defined(MSP_FET)
    #ifdef ARCH_MSP432
        armConfigSettings.ulpDebug = value;
    #endif
#endif
        break;

    case (CONFIG_PARAM_JTAG_LOCK_5XX):
        JTAGLock5xx = value;
        break;
        
    default:
        return (CONFIG_PARAM_UNKNOWN_PARAMETER);
    }

    return(0);
}

/*
 * \ingroup MODULMACROS
 *
 * \file DisableDebugArm.c
 *
 * \brief <FILEBRIEF>
*/


/**
  DisableDebugArm: disables Debug
*/
//#if defined(MSP430_UIF) || defined(MSP_FET)
//extern ARMConfigSettings armConfigSettings;
//#endif

HAL_FUNCTION(_hal_DisableDebugArm)
{

#if defined(MSP430_UIF) || defined(MSP_FET)
    // Disable breakpoints
    uint32_t data = KEY;
    if(IHIL_Write_Read_Mem_Ap(0, FP_CTRL, &data, WRITE) == -1)
    {
         return HALERR_UNDEFINED_ERROR;
    }
    // Disable Debug
    data = DBGKEY;
    if(IHIL_Write_Read_Mem_Ap(0, DHCSR, &data, WRITE) == -1)
    {
         return HALERR_UNDEFINED_ERROR;
    }
#endif

    return 0;
}
/**
 * \ingroup MODULMACROS
 *
 * \file DummyHal430.c
 *
 * \brief Set certain parameters to spcified values
 */


#ifdef MSP430_UIF
    HAL_FUNCTION(_hal_BlowFuse)                          {return  -1;}
    HAL_FUNCTION(_hal_StartJtagActivationCode)           {return  -1;}
    HAL_FUNCTION(_hal_WaitForEem)                        {return  -1;}
    HAL_FUNCTION(_hal_GetJtagId)                         {return  -1;}
    HAL_FUNCTION(_hal_SetDeviceChainInfo)                {return  -1;}
    HAL_FUNCTION(_hal_SetChainConfiguration)             {return  -1;}
    HAL_FUNCTION(_hal_GetNumOfDevices)                   {return  -1;}
    HAL_FUNCTION(_hal_GetDeviceIdPtr)                    {return  -1;}
    HAL_FUNCTION(_hal_SyncJtag_AssertPor_SaveContext)    {return  -1;}
    HAL_FUNCTION(_hal_SyncJtag_Conditional_SaveContext)  {return  -1;}
    HAL_FUNCTION(_hal_RestoreContext_ReleaseJtag)        {return  -1;}
    HAL_FUNCTION(_hal_ReadMemBytes)                      {return  -1;}
    HAL_FUNCTION(_hal_ReadMemWords)                      {return  -1;}
    HAL_FUNCTION(_hal_ReadMemQuick)                      {return  -1;}
    HAL_FUNCTION(_hal_WriteMemBytes)                     {return  -1;}
    HAL_FUNCTION(_hal_WriteMemWords)                     {return  -1;}
    HAL_FUNCTION(_hal_EemDataExchange)                   {return  -1;}
    HAL_FUNCTION(_hal_EemDataExchangeAFE2xx)             {return  -1;}
    HAL_FUNCTION(_hal_SingleStep)                        {return  -1;}
    HAL_FUNCTION(_hal_ReadAllCpuRegs)                    {return  -1;}
    HAL_FUNCTION(_hal_WriteAllCpuRegs)                   {return  -1;}
    HAL_FUNCTION(_hal_Psa)                               {return  -1;}
    HAL_FUNCTION(_hal_ExecuteFunclet)                    {return  -1;}
    HAL_FUNCTION(_hal_ExecuteFuncletJtag)                {return  -1;}
    HAL_FUNCTION(_hal_GetDcoFrequency)                   {return  -1;}
    HAL_FUNCTION(_hal_GetDcoFrequencyJtag)               {return  -1;}
    HAL_FUNCTION(_hal_GetFllFrequency)                   {return  -1;}
    HAL_FUNCTION(_hal_GetFllFrequencyJtag)               {return  -1;}
    HAL_FUNCTION(_hal_WaitForStorage)                    {return  -1;}
    HAL_FUNCTION(_hal_SyncJtag_AssertPor_SaveContextX)   {return  -1;}
    HAL_FUNCTION(_hal_SyncJtag_Conditional_SaveContextX) {return  -1;}
    HAL_FUNCTION(_hal_RestoreContext_ReleaseJtagX)       {return  -1;}
    HAL_FUNCTION(_hal_ReadMemBytesX)                     {return  -1;}
    HAL_FUNCTION(_hal_ReadMemWordsX)                     {return  -1;}
    HAL_FUNCTION(_hal_ReadMemQuickX)                     {return  -1;}
    HAL_FUNCTION(_hal_WriteMemBytesX)                    {return  -1;}
    HAL_FUNCTION(_hal_WriteMemWordsX)                    {return  -1;}
    HAL_FUNCTION(_hal_EemDataExchangeX)                  {return  -1;}
    HAL_FUNCTION(_hal_SingleStepX)                       {return  -1;}
    HAL_FUNCTION(_hal_ReadAllCpuRegsX)                   {return  -1;}
    HAL_FUNCTION(_hal_WriteAllCpuRegsX)                  {return  -1;}
    HAL_FUNCTION(_hal_PsaX)                              {return  -1;}
    HAL_FUNCTION(_hal_ExecuteFuncletX)                   {return  -1;}
    HAL_FUNCTION(_hal_GetDcoFrequencyX)                  {return  -1;}
    HAL_FUNCTION(_hal_GetFllFrequencyX)                  {return  -1;}
    HAL_FUNCTION(_hal_WaitForStorageX)                   {return  -1;}
    HAL_FUNCTION(_hal_BlowFuseXv2)                       {return  -1;}
    HAL_FUNCTION(_hal_BlowFuseFram)                      {return  -1;}
    HAL_FUNCTION(_hal_SyncJtag_AssertPor_SaveContextXv2) {return  -1;}
    HAL_FUNCTION(_hal_SyncJtag_Conditional_SaveContextXv2){return  -1;}
    HAL_FUNCTION(_hal_RestoreContext_ReleaseJtagXv2)      {return  -1;}
    HAL_FUNCTION(_hal_ReadMemWordsXv2)                    {return  -1;}
    HAL_FUNCTION(_hal_ReadMemQuickXv2)                    {return  -1;}
    HAL_FUNCTION(_hal_WriteMemWordsXv2)                   {return  -1;}
    HAL_FUNCTION(_hal_EemDataExchangeXv2)                 {return  -1;}
    HAL_FUNCTION(_hal_SingleStepXv2)                      {return  -1;}
    HAL_FUNCTION(_hal_ReadAllCpuRegsXv2)                  {return  -1;}
    HAL_FUNCTION(_hal_WriteAllCpuRegsXv2)                 {return  -1;}
    HAL_FUNCTION(_hal_PsaXv2)                             {return  -1;}
    HAL_FUNCTION(_hal_ExecuteFuncletXv2)                  {return  -1;}
    HAL_FUNCTION(_hal_UnlockDeviceXv2)                    {return  -1;}
    HAL_FUNCTION(_hal_MagicPattern)                       {return  -1;}
    HAL_FUNCTION(_hal_UnlockC092)                         {return  -1;}
    HAL_FUNCTION(_hal_PollJStateReg)                      {return  -1;}
    HAL_FUNCTION(_hal_PollJStateRegFR57xx)                {return  -1;}
    HAL_FUNCTION(_hal_IsJtagFuseBlown)                    {return  -1;}
    HAL_FUNCTION(_hal_ResetXv2)                           {return  -1;}
    HAL_FUNCTION(_hal_WriteFramQuickXv2)                  {return  -1;}
    HAL_FUNCTION(_hal_SendJtagMailboxXv2)                 {return  -1;}
    HAL_FUNCTION(_hal_SingleStepJStateXv2)                {return  -1;}
    HAL_FUNCTION(_hal_PollJStateRegEt8)                   {return  -1;}
    HAL_FUNCTION(_hal_Reset430I)                          {return  -1;}
    HAL_FUNCTION(_hal_PollJStateReg430I)                  {return  -1;}
    HAL_FUNCTION(_hal_PollJStateReg20)                    {return  -1;}
    HAL_FUNCTION(_hal_ResetL092)                          {return  -1;}
    HAL_FUNCTION(_hal_Reset5438Xv2)                       {return  -1;}
    HAL_FUNCTION(_hal_LeaSyncConditional)                 {return  -1;}
    HAL_FUNCTION(_hal_PollDStatePCRegEt)                  {return  -1;}
    HAL_FUNCTION(_hal_UssSyncConditional)                 {return  -1;}
#endif

/**
 * \ingroup MODULMACROS
 *
 * \file DummyHal432.c
 *
 * \brief Set certain parameters to spcified values
 */


#ifdef MSP430_UIF
    HAL_FUNCTION(_hal_WriteAllCpuRegsArm)          {return  -1;}
    HAL_FUNCTION(_hal_WaitForDebugHaltArm)         {return  -1;}
    HAL_FUNCTION(_hal_SingleStepArm)               {return  -1;}
    HAL_FUNCTION(_hal_ScanApArm)                   {return  -1;}
    HAL_FUNCTION(_hal_RunArm)                      {return  -1;}
    HAL_FUNCTION(_hal_ResetArm)                    {return  -1;}
    HAL_FUNCTION(_hal_ReadAllCpuRegsArm)           {return  -1;}
    HAL_FUNCTION(_hal_MemApTransactionArm)         {return  -1;}
    HAL_FUNCTION(_hal_HaltArm)                     {return  -1;}
    HAL_FUNCTION(_hal_GetJtagIdCodeArm)            {return  -1;}
    HAL_FUNCTION(_hal_EnableDebugArm)              {return  -1;}
    HAL_FUNCTION(_hal_DisableDebugArm)             {return  -1;}
    HAL_FUNCTION(_hal_MemApTransactionArmSwd)      {return  -1;}
    HAL_FUNCTION(_hal_GetInterfaceModeArm)         {return  -1;}
    HAL_FUNCTION(_hal_GetCpuIdArm)                 {return  -1;}
    HAL_FUNCTION(_hal_CheckDapLockArm)             {return  -1;}
    HAL_FUNCTION(_hal_UnlockDap)                   {return  -1;}
#endif
/**
* \ingroup MODULEMACROS
*
* \file EemDataExchange.c
*
* \brief Streaming read and write of EEM register values
*/
/*
 * EemDataExchange.c
 *
 * <FILEBRIEF>
 */


/**
  EemDataExchange
  Streaming read and write of EEM register values. This function is for
  16bit CPUs.
  inData:  <count(8)> (<addr(8)> [<ivalue(16)>]){*}
  outData: <ovalue(16)>{*}
  count: number of the following exchanges
  addr: address to read from or write to, the LSB marks a read operation
  ivalue: only present for write operations
  ovalue: one for each read operation
*/

//extern uint16_t lastTraceWritePos;

HAL_FUNCTION(_hal_EemDataExchange)
{
  uint8_t NumberOfExchanges;
  uint8_t tmp_char;
  uint16_t tmp_uint;

  STREAM_get_byte(&NumberOfExchanges);
  eem_data_exchange();

  while(NumberOfExchanges)
  {
    STREAM_get_byte(&tmp_char);
    if(tmp_char & 0x01)
    { // read access
      SetReg_16Bits(tmp_char);   // load address
      // shift in dummy 0
      STREAM_put_word(SetReg_16Bits(0)); // put output into stream
    }
    else
    { // write access
      STREAM_get_word(&tmp_uint);
      SetReg_16Bits(tmp_char);           // load address
      SetReg_16Bits(tmp_uint);  // shift in value

      //Reset when state storage is reset
      if ((tmp_char == 0x9E) && (tmp_uint & 0x40))
      {
          lastTraceWritePos = 0;
      }
    }
    NumberOfExchanges--;
  }

  return 0;
}

/**
* \ingroup MODULMACROS
*
* \file EemDataExchangeAFE2xx.c
*
* \brief Streaming read and write of EEM register values
*/


//Dummy implementation to retain macro numbers
HAL_FUNCTION(_hal_EemDataExchangeAFE2xx)
{
    return 0;
}
/**
* \ingroup MODULMACROSX
*
* \file EemDataExchangeX.c
*
* \brief Streaming read and write of EEM register values
*/


/**
  EemDataExchangeX
  Streaming read and write of EEM register values. This function is for
  20bit CPUs.
  inData:  <count(8)> (<addr(8)> [<ivalue(32)>]){*}
  outData: <ovalue(32)>{*}
  count: number of the following exchanges
  addr: address to read from or write to, the LSB marks a read operation
  ivalue: only present for write operations
  ovalue: one for each read operation
*/
//extern uint16_t lastTraceWritePos;

HAL_FUNCTION(_hal_EemDataExchangeX)
{
  uint8_t NumberOfExchanges = 0;
  uint8_t tmp_char= 0;
  uint32_t tmp_ulong= 0;

  STREAM_get_byte(&NumberOfExchanges);

  eem_data_exchange32();

  while(NumberOfExchanges)
  {
    STREAM_get_byte(&tmp_char);
    if(tmp_char & 0x01)
    { // read access
      SetReg_32Bits(tmp_char);   // load address
      STREAM_put_long(SetReg_32Bits(0)); // put output into stream
    }
    else
    { // write access
      STREAM_get_long(&tmp_ulong);
      SetReg_32Bits(tmp_char);           // load address
      SetReg_32Bits(tmp_ulong);  // shift in value

       if ((tmp_char == 0x9E) && (tmp_ulong & 0x40))
       {
          lastTraceWritePos = 0;
       }
    }
    NumberOfExchanges--;
  }

  return 0;
}

/**
* \ingroup MODULMACROSXV2
*
* \file EemDataExchangeXv2.c
*
* \brief Streaming read and write of EEM register values
*/


/**
  EemDataExchangeXv2
  Streaming read and write of EEM register values. This function is for
  20bit CPUs.
  inData:  <count(8)> (<addr(8)> [<ivalue(32)>]){*}
  outData: <ovalue(32)>{*}
  count: number of the following exchanges
  addr: address to read from or write to, the LSB marks a read operation
  ivalue: only present for write operations
  ovalue: one for each read operation
*/
//extern uint16_t lastTraceWritePos;
//extern uint32_t _hal_mclkCntrl0;

HAL_FUNCTION(_hal_EemDataExchangeXv2)
{
    uint8_t tmp_char= 0 , NumberOfExchanges = 0;
    uint32_t lOut_long = 0, tmp_ulong = 0;

    STREAM_get_byte(&NumberOfExchanges);

    //eem_data_exchange
    eem_data_exchange32();

    while(NumberOfExchanges)
    {
        STREAM_get_byte(&tmp_char);
        if(tmp_char & 0x01)
        {   // read access
            if((tmp_char & 0xfe) == MODCLKCTRL0)
            { // read access
                lOut_long = _hal_mclkCntrl0;
            }
            else
            {
                SetReg_32Bits(tmp_char);   // load address
                lOut_long = SetReg_32Bits(0);
            }
            STREAM_put_long(lOut_long); // put output into stream
        }
        else
        { // write access
            STREAM_get_long(&tmp_ulong);

            if((tmp_char & 0xfe) == MODCLKCTRL0)
            {

                _hal_mclkCntrl0 = tmp_ulong;
            }
            else
            {
                if ((tmp_char == 0x9E) && (tmp_ulong & 0x40))
                {
                    lastTraceWritePos = 0;
                }

                SetReg_32Bits(tmp_char);           // load address
                SetReg_32Bits(tmp_ulong);  // shift in value
            }
        }
        NumberOfExchanges--;
    }
    return 0;
}




/*
 * \ingroup MODULMACROS
 *
 * \file EnableDebugArm.c
 *
 * \brief <FILEBRIEF>
 */


/**
  EnableDebug: halts the CPU and enables Debug
*/
//#if defined(MSP430_UIF) || defined(MSP_FET)
//extern ARMConfigSettings armConfigSettings;
//#endif

HAL_FUNCTION(_hal_EnableDebugArm)
{
#if defined(MSP430_UIF) || defined(MSP_FET)
    uint32_t data = DBGKEY | C_DEBUGEN;
    if(IHIL_Write_Read_Mem_Ap(0, DHCSR, &data, WRITE) == -1)
    {
        return HALERR_UNDEFINED_ERROR;
    }

    // Configure which vectors to catch
    data = TRCENA | VC_HARDERR | VC_CORERESET;
    if(IHIL_Write_Read_Mem_Ap(0, DEMCR, &data, WRITE) == -1)
    {
        return HALERR_UNDEFINED_ERROR;
    }

    // Clear all fault flags
    data = EXTERNAL | VCATCH | DWTTRAP | BKPT | HALTED;
    if(IHIL_Write_Read_Mem_Ap(0, DFSR, &data, WRITE) == -1)
    {
        return HALERR_UNDEFINED_ERROR;
    }
#endif
    return 0;
}
/**
* \ingroup MODULMACROS
*
* \file ExecuteFunclet.c
*
* \brief Execute a funclet on the target
*/

//!
//! Funclet parameters:
//!
//!   Parameter       Register
//!   =========       ========
//!   Address         R5
//!   Size            R6
//!   LockA           R8
//!   Type            R9
//!
//!   General Purpose Registers used by funclet
//!   =========================================
//!   The following registers are used by the funclet for general purpose. The
//!   debugger is responsible for restoring the value of the registers after
//!   completion.
//!
//!   R10             Memory pointer
//!   R11             Compare register/Outer loop counter
//!   R12             Inner delay loop counter
//!
//!   Funclet memory map
//!   ==================
//!   The Funclet uses a very specifc memory map, so that the debugger can check
//!   for specific values on the MAB to track the progress of the funclet. The
//!   following table documents this memory map:
//!
//!   Section:        Location:       Size:     Remarks:
//!   --------        ---------       --------  --------
//!   Initialization  0000h - 003Eh   32 words  Initializaiton of variables
//!   WaitforDeadLoop 0040h - 0046h    3 words  Waiting for the debugger acknowledge
//!   CodeBody        0048h - 00ECh   84 words  Code body which performs actual task
//!   Stop            00EEh            1 word   End of program
//!   ControlWord     00F0h            1 word   Control word to comms between debugger and funclet
//!   FCTL1Value      00F2h            1 word   Saved value for restoring later
//!   FCTL2Value      00F4h            1 word   Saved value for restoring later
//!   FCTL3Value      00F6h            1 word   Saved value for restoring later
//!   UserData        0100h - xxxxh    -        Specific data to be written to flash
//!
/*---------------------------------------------------------------------------*/


#undef REG_ADDRESS
#define REG_ADDRESS 5
#undef REG_SIZE
#define REG_SIZE    6
#undef REG_LOCKA
#define REG_LOCKA   8
#undef REG_TYPE
#define REG_TYPE    9
#undef REG_GP1
#define REG_GP1     10  // General purpose registers used by the funclet
#undef REG_GP2
#define REG_GP2     11
#undef REG_GP3
#define REG_GP3     12

#undef WAIT_FOR_DEAD_START
#define WAIT_FOR_DEAD_START (0x20)  // Code position where the WaitForDead loop starts
#undef WAIT_FOR_DEAD_END
#define WAIT_FOR_DEAD_END   (0x26)  // Code position where the WaitForDead loop ends
#undef EXECUTE_FUNCLET
#define EXECUTE_FUNCLET     (0x28)  // Location of actual funclet body
#undef FINISHED_OFFSET
#define FINISHED_OFFSET     (0x5C)  // Location of the final jmp $ instruction of the funclet
#undef CONTROL_WORD_OFFSET
#define CONTROL_WORD_OFFSET (0x5E)  // Location of the control word
#undef DATA_OFFSET
#define DATA_OFFSET         (0x60)  // Location where data starts

//extern DeviceSettings deviceSettings;

void setFuncletRegisters_hal_ExecuteFunclet(const uint16_t* registerData)
{
    WriteCpuReg(REG_ADDRESS, registerData[0]);
    WriteCpuReg(REG_SIZE, registerData[1]);
    WriteCpuReg(REG_TYPE, registerData[2]);
    WriteCpuReg(REG_LOCKA, registerData[3]);
	WriteCpuReg(REG_GP1, registerData[4]);
	WriteCpuReg(REG_GP2, registerData[5]);
	WriteCpuReg(REG_GP3, registerData[6]);
}

struct {
    uint32_t lLen;
    uint16_t ret_len;
    uint32_t Addr;
    uint16_t memSize;
    uint16_t LockA;
    uint16_t usType;
    uint16_t startAddr;
    uint16_t R12_BCSLTC1;
    uint16_t R11_DCO;
    uint16_t registerBackups[7];
    uint16_t FCTL1Value;
    uint16_t FCTL2Value;
    uint16_t FCTL3Value;
} _hal_ExecuteFunclet_staticVars = {};

HAL_FUNCTION(_hal_ExecuteFunclet)
{
#define setFuncletRegisters setFuncletRegisters_hal_ExecuteFunclet
    STATIC_VARS_START(_hal_ExecuteFunclet);
    DECL_STATIC_VAR(lLen);
    DECL_STATIC_VAR(ret_len);
    DECL_STATIC_VAR(Addr);
    DECL_STATIC_VAR(memSize);
    DECL_STATIC_VAR(LockA);
    DECL_STATIC_VAR(usType);
    DECL_STATIC_VAR(startAddr);
    DECL_STATIC_VAR(R12_BCSLTC1);
    DECL_STATIC_VAR(R11_DCO);
    DECL_STATIC_VAR(registerBackups);
    DECL_STATIC_VAR(FCTL1Value);
    DECL_STATIC_VAR(FCTL2Value);
    DECL_STATIC_VAR(FCTL3Value);
    
    int16_t ret_value = 0;
    uint16_t lOut;

    uint16_t tmpFCTL3Value;

    uint16_t tgtStart     =0x0;
    uint16_t data         =0x0;
    uint16_t timeOut      =3000;


    StreamSafe stream_tmp;

    if(flags & MESSAGE_NEW_MSG)
    {
        // get target RAM start
        if(STREAM_get_word(&tgtStart) != 0)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_RAM_START);
        }
       /// get available RAM size (ram - funclet size)
        if(STREAM_get_word(&memSize)!= 0)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_RAM_SIZE);
        }
        /// get RAM Start + Code offset
        if(STREAM_get_word(&startAddr)!= 0)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_OFFSET);
        }
        // get start addres as int32_t value
        if(STREAM_get_long(&Addr) != 0)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_ADDRESS);
        }
        // get length ot be flashed
        if(STREAM_get_long(&lLen) != 0)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_LENGTH);
        }
        if(STREAM_get_word(&usType) != 0)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_TYPE);
        }
        // lock A handling
        if(STREAM_get_word(&LockA) == -1)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_LOCKA);
        }

        if(STREAM_get_word(&R11_DCO) == -1)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_LOCKA);
        }

        if(STREAM_get_word(&R12_BCSLTC1) == -1)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_LOCKA);
        }

        registerBackups[0] = ReadCpuReg(REG_ADDRESS);
        registerBackups[1] = ReadCpuReg(REG_SIZE);
        registerBackups[2] = ReadCpuReg(REG_TYPE);
        registerBackups[3] = ReadCpuReg(REG_LOCKA);
        registerBackups[4] = ReadCpuReg(REG_GP1);
        registerBackups[5] = ReadCpuReg(REG_GP2);
        registerBackups[6] = ReadCpuReg(REG_GP3);

        // Setup the Flash Controller
        // Read FCTL registers for later restore
        FCTL1Value = ReadMemWord(0x128);
        FCTL2Value = ReadMemWord(0x12A);
        FCTL3Value = ReadMemWord(0x12C);
        // Restore password byte
        FCTL1Value ^= 0x3300;
        FCTL2Value ^= 0x3300;
        FCTL3Value ^= 0x3300;

        WriteMemWord(0x12A,0xA544);     // Source = MCLK, DIV = 5.
        WriteMemWord(0x12C,LockA);      // Set LockA
        tmpFCTL3Value = ReadMemWord(0x12C);    // Read out register again

        if((LockA & 0xff) != (tmpFCTL3Value & 0xff))
        {   // Value of lockA is not as expected, so toggle it
            WriteMemWord(0x12C,LockA | 0x40);
        }
        {
            uint16_t registerValues[7] = {(uint16_t)Addr, (uint16_t)lLen, usType, LockA, 0,R11_DCO,R12_BCSLTC1};
            setFuncletRegisters(registerValues);
            WriteCpuReg(2, 0);
        }
        if (deviceSettings.clockControlType != GCC_NONE)
        {
            if(deviceSettings.stopFLL)
            {
                uint16_t clkCntrl = 0x11;
                clkCntrl &= ~0x10;
                eem_data_exchange();
                SetReg_16Bits(MX_GCLKCTRL + MX_WRITE);
                SetReg_16Bits(clkCntrl);
            }
        }
        // i_SetPcRel
        SetPc(startAddr);
        IHIL_Tclk(1);

        // prepare release & release
        cntrl_sig_16bit();
        SetReg_16Bits(0x0401);
        addr_capture();
        cntrl_sig_release();

        // Poll until the funclet reaches the WaitForDead loop,
        // ie. it is ready to process data
        do
        {
            IHIL_Delay_1ms(1);
            addr_capture();
            lOut = SetReg_16Bits(0);
            timeOut--;
        }
        while(!((lOut >= (startAddr + WAIT_FOR_DEAD_START)) && (lOut <= (startAddr + WAIT_FOR_DEAD_END))) && timeOut);
    }

    while(lLen && (ret_value == 0) && timeOut)
    {
      uint16_t writePos = startAddr + DATA_OFFSET;
      const uint16_t writeEndPos = writePos + (2*lLen < memSize ? 2*lLen : memSize);
      uint16_t numWordsToWrite = 0;

      SetPc(startAddr + FINISHED_OFFSET);
      // The device has limited RAM available to write updates to,
      // make sure we stay within this limit: Total memory size - 96 bytes for the funclet
      halt_cpu();
      IHIL_Tclk(0);
      cntrl_sig_16bit();
      SetReg_16Bits(0x2408);

      while( (writePos < writeEndPos) && (ret_value == 0) )
      {
          ret_value = STREAM_get_word(&data);

          addr_16bit();
          SetReg_16Bits(writePos);
          data_to_addr();
          SetReg_16Bits(data);
          IHIL_Tclk(1);
          IHIL_Tclk(0);

          writePos += 2;
      }
      release_cpu();

      numWordsToWrite = (writePos - (startAddr + DATA_OFFSET)) / 2;

      WriteCpuReg(REG_SIZE, numWordsToWrite);
      WriteCpuReg(REG_ADDRESS, Addr);

      Addr += 2 * numWordsToWrite;
      lLen -= numWordsToWrite;

      SetPc(startAddr + EXECUTE_FUNCLET);
      cntrl_sig_release();

      // Poll until the funclet reaches the Stop loop,
      // ie. it is finished
      timeOut = 3000;
      do
      {
          IHIL_Delay_1ms(1);
          addr_capture();
          lOut = SetReg_16Bits(0);
          timeOut--;
      }
      while(!(lOut == (startAddr + FINISHED_OFFSET)) && timeOut);
    }

    if(flags & MESSAGE_LAST_MSG )
    { // ExitFlashWrite
        {   //Setup values for watchdog control regsiters
            uint8_t DummyIn[8] = {WDTCTL_ADDRESS & 0xFF,(WDTCTL_ADDRESS >> 8) & 0xFF,
                                        WDTHOLD_DEF,WDTPW_DEF,0,0,0,0};
            STREAM_internal_stream(DummyIn, sizeof(DummyIn), NULL, 0, &stream_tmp);
            (this->*HAL_SyncJtag_Conditional_SaveContext)(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG);
            STREAM_external_stream(&stream_tmp);
        }
        setFuncletRegisters(registerBackups);
        // Restore the Flash controller registers
        WriteMemWord(0x128,FCTL1Value);
        WriteMemWord(0x12A,FCTL2Value);
        WriteMemWord(0x12C,FCTL3Value);
        tmpFCTL3Value = ReadMemWord(0x12C);    // Read out register again

        if((FCTL3Value & 0xff) != (tmpFCTL3Value & 0xff))
        {   // Value of lockA is not as expected, so toggle it
            WriteMemWord(0x12C,FCTL3Value | 0x40);
        }

        STREAM_put_word(ret_len);
    }
    else if(ret_value == 1)
    {
        STREAM_out_change_type(RESPTYP_ACKNOWLEDGE);
        ret_value = 0;
    }
    else
    {
         //Setup values for watchdog control regsiters
         uint8_t DummyIn[8] = {WDTCTL_ADDRESS & 0xFF,(WDTCTL_ADDRESS >> 8) & 0xFF,
                                     WDTHOLD_DEF,WDTPW_DEF,0,0,0,0};
         STREAM_internal_stream(DummyIn, sizeof(DummyIn), NULL, 0, &stream_tmp);
         (this->*HAL_SyncJtag_Conditional_SaveContext)(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG);
         STREAM_external_stream(&stream_tmp);

         setFuncletRegisters(registerBackups);
         // Restore the Flash controller registers
         WriteMemWord(0x128,FCTL1Value);
         WriteMemWord(0x12A,FCTL2Value);
         WriteMemWord(0x12C,FCTL3Value);
         tmpFCTL3Value = ReadMemWord(0x12C);    // Read out register again

         if((FCTL3Value & 0xff) != (tmpFCTL3Value & 0xff))
         {   // Value of lockA is not as expected, so toggle it
             WriteMemWord(0x12C,FCTL3Value | 0x40);
         }

         ret_value = HALERR_EXECUTE_FUNCLET_EXECUTION_ERROR;
    }
    return(ret_value);
#undef setFuncletRegisters
}
/**
* \ingroup MODULMACROS
*
* \file ExecuteFuncletJtag.c
*
* \brief  Execute a funclet on the target
*/

//! Funclet parameters:
//!
//!   Parameter       Register
//!   =========       ========
//!   Address         R5
//!   Size            R6
//!   LockA           R8
//!   Type            R9
//!
//!   General Purpose Registers used by funclet
//!   =========================================
//!   The following registers are used by the funclet for general purpose. The
//!   debugger is responsible for restoring the value of the registers after
//!   completion.
//!
//!   R10             Memory pointer
//!   R11             Compare register/Outer loop counter
//!   R12             Inner delay loop counter
//!
//!   Funclet memory map
//!   ==================
//!   The Funclet uses a very specifc memory map, so that the debugger can check
//!   for specific values on the MAB to track the progress of the funclet. The
//!   following table documents this memory map:
//!
//!   Section:        Location:       Size:     Remarks:
//!   --------        ---------       --------  --------
//!   Initialization  0000h - 003Eh   32 words  Initializaiton of variables
//!   WaitforDeadLoop 0040h - 0046h    3 words  Waiting for the debugger acknowledge
//!   CodeBody        0048h - 00ECh   84 words  Code body which performs actual task
//!   Stop            00EEh            1 word   End of program
//!   ControlWord     00F0h            1 word   Control word to comms between debugger and funclet
//!   FCTL1Value      00F2h            1 word   Saved value for restoring later
//!   FCTL2Value      00F4h            1 word   Saved value for restoring later
//!   FCTL3Value      00F6h            1 word   Saved value for restoring later
//!   UserData        0100h - xxxxh    -        Specific data to be written to flash
//!
/*---------------------------------------------------------------------------*/



#define REG_ADDRESS 5
#define REG_SIZE    6
#define REG_LOCKA   8
#define REG_TYPE    9

#define REG_GP1     10  // General purpose registers used by the funclet
#define REG_GP2     11
#define REG_GP3     12

#define WAIT_FOR_DEAD_START (0x20)  // Code position where the WaitForDead loop starts
#define WAIT_FOR_DEAD_END   (0x26)  // Code position where the WaitForDead loop ends
#define EXECUTE_FUNCLET     (0x28)  // Location of actual funclet body
#define FINISHED_OFFSET     (0x5C)  // Location of the final jmp $ instruction of the funclet
#define CONTROL_WORD_OFFSET (0x5E)  // Location of the control word
#define DATA_OFFSET         (0x60)  // Location where data starts

//extern DeviceSettings deviceSettings;

void setFuncletRegisters_hal_ExecuteFuncletJtag(const uint16_t* registerData)
{
    WriteCpuReg(REG_ADDRESS, registerData[0]);
    WriteCpuReg(REG_SIZE, registerData[1]);
    WriteCpuReg(REG_TYPE, registerData[2]);
    WriteCpuReg(REG_LOCKA, registerData[3]);
	WriteCpuReg(REG_GP1, registerData[4]);
	WriteCpuReg(REG_GP2, registerData[5]);
	WriteCpuReg(REG_GP3, registerData[6]);
}

uint16_t funcletBackup[0x30] = {0};

void stopAndRestoreRam(uint16_t startAddr)
{
    int16_t i = 0;
    cntrl_sig_high_byte();
    SetReg_8Bits((CNTRL_SIG_TAGFUNCSAT | CNTRL_SIG_TCE1 | CNTRL_SIG_CPU) >> 8);
    SyncJtag();
    SetPcJtagBug(ROM_ADDR);
    IHIL_Tclk(1);

    halt_cpu();
    IHIL_Tclk(0);
    cntrl_sig_16bit();
    SetReg_16Bits(0x2408);

    for (i = 0; i < 0x30; ++i)
    {
        const uint16_t addr = startAddr + i * 2;
        addr_16bit();
        SetReg_16Bits(addr);
        data_to_addr();
        SetReg_16Bits(funcletBackup[i]);
        IHIL_Tclk(1);
        IHIL_Tclk(0);
    }
    release_cpu();
}

struct {
    uint32_t lLen;
    uint32_t Addr;
    uint16_t memSize;
    uint16_t LockA;
    uint16_t usType;
    uint16_t startAddr;
    uint16_t R12_BCSLTC1;
    uint16_t R11_DCO;
    uint16_t registerBackups[7];
    uint16_t FCTL1Value;
    uint16_t FCTL2Value;
    uint16_t FCTL3Value;
} _hal_ExecuteFuncletJtag_staticVars = {};

HAL_FUNCTION(_hal_ExecuteFuncletJtag)
{
#define setFuncletRegisters setFuncletRegisters_hal_ExecuteFuncletJtag
    STATIC_VARS_START(_hal_ExecuteFuncletJtag);
    DECL_STATIC_VAR(lLen);
    DECL_STATIC_VAR(Addr);
    DECL_STATIC_VAR(memSize);
    DECL_STATIC_VAR(LockA);
    DECL_STATIC_VAR(usType);
    DECL_STATIC_VAR(startAddr);
    DECL_STATIC_VAR(R12_BCSLTC1);
    DECL_STATIC_VAR(R11_DCO);
    DECL_STATIC_VAR(registerBackups);
    DECL_STATIC_VAR(FCTL1Value);
    DECL_STATIC_VAR(FCTL2Value);
    DECL_STATIC_VAR(FCTL3Value);
    
    int16_t ret_value = 0;

    uint16_t tmpFCTL3Value, lOut = 0;

    uint16_t tgtStart     = 0x0;
    uint16_t data         = 0x0;
    uint16_t timeOut      = 3000;
    int16_t i = 0;
    StreamSafe stream_tmp;

    if(flags & MESSAGE_NEW_MSG)
    {
        // get target RAM start
        if(STREAM_get_word(&tgtStart) != 0)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_RAM_START);
        }
       /// get available RAM size (ram - funclet size)
        if(STREAM_get_word(&memSize)!= 0)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_RAM_SIZE);
        }
        /// get RAM Start + Code offset
        if(STREAM_get_word(&startAddr)!= 0)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_OFFSET);
        }
        // get start addres as int32_t value
        if(STREAM_get_long(&Addr) != 0)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_ADDRESS);
        }
        // get length ot be flashed
        if(STREAM_get_long(&lLen) != 0)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_LENGTH);
        }
        if(STREAM_get_word(&usType) != 0)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_TYPE);
        }
        // lock A handling
        if(STREAM_get_word(&LockA) == -1)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_LOCKA);
        }

         if(STREAM_get_word(&R11_DCO) == -1)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_LOCKA);
        }

        if(STREAM_get_word(&R12_BCSLTC1) == -1)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_LOCKA);
        }

        SetPcJtagBug(ROM_ADDR);
        IHIL_Tclk(1);

        for (i = 0; i < 0x30; ++i)
        {
            const uint16_t addr = startAddr + i*2;
            funcletBackup[i] = ReadMemWord(addr);
        }

        registerBackups[0] = ReadCpuReg(REG_ADDRESS);
        registerBackups[1] = ReadCpuReg(REG_SIZE);
        registerBackups[2] = ReadCpuReg(REG_TYPE);
        registerBackups[3] = ReadCpuReg(REG_LOCKA);
		registerBackups[4] = ReadCpuReg(REG_GP1);
		registerBackups[5] = ReadCpuReg(REG_GP2);
		registerBackups[6] = ReadCpuReg(REG_GP3);

        // Setup the Flash Controller
        // Read FCTL registers for later restore
        FCTL1Value = ReadMemWord(0x128);
        FCTL2Value = ReadMemWord(0x12A);
        FCTL3Value = ReadMemWord(0x12C);

        // Restore password byte
        FCTL1Value ^= 0x3300;
        FCTL2Value ^= 0x3300;
        FCTL3Value ^= 0x3300;

        WriteMemWord(0x12A,0xA544);     // Source = MCLK, DIV = 5.
        WriteMemWord(0x12C,LockA);      // Set LockA
        tmpFCTL3Value = ReadMemWord(0x12C) ;   // Read out register again

        if((LockA & 0xff) != (tmpFCTL3Value & 0xff))
        {   // Value of lockA is not as expected, so toggle it
            WriteMemWord(0x12C,LockA | 0x40);
        }

        {
            uint16_t registerValues[7] = {(uint16_t)Addr, (uint16_t)lLen, usType, LockA, 0, R11_DCO, R12_BCSLTC1};
            setFuncletRegisters(registerValues);
            WriteCpuReg(2, 0);
        }

        if (deviceSettings.clockControlType != GCC_NONE)
        {
            if(deviceSettings.stopFLL)
            {
                uint16_t clkCntrl = 0x11;
                clkCntrl &= ~0x10;
                eem_data_exchange();
                SetReg_16Bits(MX_GCLKCTRL + MX_WRITE);
                SetReg_16Bits(clkCntrl);
            }
        }

        SetPcJtagBug(startAddr);
        cntrl_sig_release();
        addr_capture();

        // Poll until the funclet reaches the WaitForDead loop,
        // ie. it is ready to process data
        do
        {
            IHIL_Delay_1ms(1);
            lOut = SetReg_16Bits(0);
            timeOut--;
        }
        while(!((lOut >= (startAddr + WAIT_FOR_DEAD_START)) && (lOut <= (startAddr + WAIT_FOR_DEAD_END))) && timeOut);

        stopAndRestoreRam(startAddr);
    }

    while(lLen && (ret_value == 0) && timeOut)
    {
      uint16_t writePos = startAddr + DATA_OFFSET;
      const uint16_t writeEndPos = writePos + (2*lLen < memSize ? 2*lLen : memSize);
      uint16_t numWordsToWrite = 0;

      // The device has limited RAM available to write updates to,
      // make sure we stay within this limit: Total memory size - 96 bytes for the funclet
      halt_cpu();
      IHIL_Tclk(0);
      cntrl_sig_16bit();
      SetReg_16Bits(0x2408);

      while( (writePos < writeEndPos) && (ret_value == 0) )
      {
          ret_value = STREAM_get_word(&data);
          addr_16bit();
          SetReg_16Bits(writePos);
          data_to_addr();
          SetReg_16Bits(data);
          IHIL_Tclk(1);
          IHIL_Tclk(0);
          writePos += 2;
      }

      release_cpu();

      numWordsToWrite = (writePos - (startAddr + DATA_OFFSET)) / 2;

      WriteCpuReg(REG_SIZE, numWordsToWrite);
      WriteCpuReg(REG_ADDRESS, Addr);

      Addr += 2 * numWordsToWrite;
      lLen -= numWordsToWrite;

      SetPcJtagBug(startAddr + EXECUTE_FUNCLET);
      cntrl_sig_release();
      addr_capture();

      // Poll until the funclet reaches the Stop loop,
      // ie. it is finished
      timeOut = 3000;
      do
      {
          IHIL_Delay_1ms(1);
          lOut = SetReg_16Bits(0);
          timeOut--;
      }
      while(!(lOut == (startAddr + FINISHED_OFFSET)) && timeOut);

      stopAndRestoreRam(startAddr);

      if ( timeOut == 0)
        ret_value = HALERR_EXECUTE_FUNCLET_EXECUTION_TIMEOUT;
    }

    if((flags & MESSAGE_LAST_MSG) || (ret_value != 1) )
    {
        {   //Setup values for watchdog control regsiters
            uint8_t DummyIn[8] = {WDTCTL_ADDRESS & 0xFF,(WDTCTL_ADDRESS >> 8) & 0xFF,
                                        WDTHOLD_DEF,WDTPW_DEF,0,0,0,0};
            STREAM_internal_stream(DummyIn, sizeof(DummyIn), NULL, 0, &stream_tmp);
            (this->*HAL_SyncJtag_Conditional_SaveContext)(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG);
            STREAM_external_stream(&stream_tmp);
        }
        setFuncletRegisters(registerBackups);
        // Restore the Flash controller registers
        WriteMemWord(0x128,FCTL1Value);
        WriteMemWord(0x12A,FCTL2Value);
        WriteMemWord(0x12C,FCTL3Value);
        tmpFCTL3Value = ReadMemWord(0x12C);    // Read out register again

        if((FCTL3Value & 0xff) != (tmpFCTL3Value & 0xff))
        {   // Value of lockA is not as expected, so toggle it
            WriteMemWord(0x12C,FCTL3Value | 0x40);
        }
    }

    if(flags & MESSAGE_LAST_MSG)
    {
        STREAM_put_word(0);
    }
    else if(ret_value == 1)
    {
        STREAM_out_change_type(RESPTYP_ACKNOWLEDGE);
        ret_value = 0;
    }
    else
    {
         ret_value = HALERR_EXECUTE_FUNCLET_EXECUTION_ERROR;
    }
    return(ret_value);
#undef setFuncletRegisters
}
/**
* \ingroup MODULMACROS
*
* \file ExecuteFuncletX.c
*
* \brief  Execute a funclet on the target
*/

//! Funclet parameters:
//!
//!   Parameter       Register
//!   =========       ========
//!   Address         R5
//!   Size            R6
//!   LockA           R8
//!   Type            R9
//!
//!   General Purpose Registers used by funclet
//!   =========================================
//!   The following registers are used by the funclet for general purpose. The
//!   debugger is responsible for restoring the value of the registers after
//!   completion.
//!
//!   R10             Memory pointer
//!   R11             Compare register/Outer loop counter
//!   R12             Inner delay loop counter
//!
//!   Funclet memory map
//!   ==================
//!   The Funclet uses a very specifc memory map, so that the debugger can check
//!   for specific values on the MAB to track the progress of the funclet. The
//!   following table documents this memory map:
//!
//!   Section:        Location:       Size:     Remarks:
//!   --------        ---------       --------  --------
//!   Initialization  0000h - 003Eh   32 words  Initializaiton of variables
//!   WaitforDeadLoop 0040h - 0046h    3 words  Waiting for the debugger acknowledge
//!   CodeBody        0048h - 00ECh   84 words  Code body which performs actual task
//!   Stop            00EEh            1 word   End of program
//!   ControlWord     00F0h            1 word   Control word to comms between debugger and funclet
//!   FCTL1Value      00F2h            1 word   Saved value for restoring later
//!   FCTL2Value      00F4h            1 word   Saved value for restoring later
//!   FCTL3Value      00F6h            1 word   Saved value for restoring later
//!   UserData        0100h - xxxxh    -        Specific data to be written to flash
//!
/*---------------------------------------------------------------------------*/



#undef REG_ADDRESS
#define REG_ADDRESS 5
#undef REG_SIZE
#define REG_SIZE    6
#undef REG_LOCKA
#define REG_LOCKA   8
#undef REG_TYPE
#define REG_TYPE    9

#undef REG_GP1
#define REG_GP1     10  // General purpose registers used by the funclet
#undef REG_GP2
#define REG_GP2     11
#undef REG_GP3
#define REG_GP3     12

#undef WAIT_FOR_DEAD_START
#define WAIT_FOR_DEAD_START (0x22)  // Code position where the WaitForDead loop starts
#undef WAIT_FOR_DEAD_END
#define WAIT_FOR_DEAD_END   (0x2C)  // Code position where the WaitForDead loop ends
#undef EXECUTE_FUNCLET
#define EXECUTE_FUNCLET     (0x2E)  // Location of actual funclet body
#undef FINISHED_OFFSET
#define FINISHED_OFFSET     (0x68)  // Location of the final jmp $ instruction of the funclet
#undef CONTROL_WORD_OFFSET
#define CONTROL_WORD_OFFSET (0x6A)  // Location of the control word
#undef DATA_OFFSET
#define DATA_OFFSET         (0x6C)  // Location where data starts

void setFuncletRegisters_hal_ExecuteFuncletX(const uint32_t* registerData)
{
    WriteCpuRegX(REG_ADDRESS, registerData[0]);
    WriteCpuRegX(REG_SIZE, registerData[1]);
    WriteCpuRegX(REG_TYPE, registerData[2]);
    WriteCpuRegX(REG_LOCKA, registerData[3]);
	WriteCpuRegX(REG_GP1, registerData[4]);
	WriteCpuRegX(REG_GP2, registerData[5]);
	WriteCpuRegX(REG_GP3, registerData[6]);
}

struct {
    uint32_t lLen;
    uint16_t ret_len;
    uint32_t Addr;
    uint16_t memSize;
    uint16_t LockA;
    uint16_t usType;
    uint16_t startAddr;
    uint16_t R12_BCSLTC1;
    uint16_t R11_DCO;
    uint32_t registerBackups[7];
    uint16_t FCTL1Value;
    uint16_t FCTL2Value;
    uint16_t FCTL3Value;
} _hal_ExecuteFuncletX_staticVars = {};

HAL_FUNCTION(_hal_ExecuteFuncletX)
{
#define setFuncletRegisters setFuncletRegisters_hal_ExecuteFuncletX
    STATIC_VARS_START(_hal_ExecuteFuncletX);
    DECL_STATIC_VAR(lLen);
    DECL_STATIC_VAR(ret_len);
    DECL_STATIC_VAR(Addr);
    DECL_STATIC_VAR(memSize);
    DECL_STATIC_VAR(LockA);
    DECL_STATIC_VAR(usType);
    DECL_STATIC_VAR(startAddr);
    DECL_STATIC_VAR(R12_BCSLTC1);
    DECL_STATIC_VAR(R11_DCO);
    DECL_STATIC_VAR(registerBackups);
    DECL_STATIC_VAR(FCTL1Value);
    DECL_STATIC_VAR(FCTL2Value);
    DECL_STATIC_VAR(FCTL3Value);
    
    int16_t ret_value = 0;
    uint32_t lOut_long = 0;
    uint16_t tmpFCTL3Value;

    uint16_t tgtStart     =0x0;
    uint16_t data         =0x0;
    uint16_t timeOut      =3000;

    StreamSafe stream_tmp;

    if(flags & MESSAGE_NEW_MSG)
    {
        // get target RAM start
        if(STREAM_get_word(&tgtStart) != 0)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_RAM_START);
        }
       /// get available RAM size (ram - funclet size)
        if(STREAM_get_word(&memSize)!= 0)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_RAM_SIZE);
        }
        /// get RAM Start + Code offset
        if(STREAM_get_word(&startAddr)!= 0)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_OFFSET);
        }
        // get start addres as int32_t value
        if(STREAM_get_long(&Addr) != 0)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_ADDRESS);
        }
        // get length ot be flashed
        if(STREAM_get_long(&lLen) != 0)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_LENGTH);
        }
        if(STREAM_get_word(&usType) != 0)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_TYPE);
        }
        // lock A handling
        if(STREAM_get_word(&LockA) == -1)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_LOCKA);
        }

        if(STREAM_get_word(&R11_DCO) == -1)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_LOCKA);
        }

        if(STREAM_get_word(&R12_BCSLTC1) == -1)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_LOCKA);
        }

        registerBackups[0] = ReadCpuRegX(REG_ADDRESS);
        registerBackups[1] = ReadCpuRegX(REG_SIZE);
        registerBackups[2] = ReadCpuRegX(REG_TYPE);
        registerBackups[3] = ReadCpuRegX(REG_LOCKA);
		registerBackups[4] = ReadCpuRegX(REG_GP1);
		registerBackups[5] = ReadCpuRegX(REG_GP2);
		registerBackups[6] = ReadCpuRegX(REG_GP3);

        // Setup the Flash Controller
        // Read FCTL registers for later restore
        FCTL1Value = ReadMemWordX(0x128);
        FCTL2Value = ReadMemWordX(0x12A);
        FCTL3Value = ReadMemWordX(0x12C);
        // Restore password byte
        FCTL1Value ^= 0x3300;
        FCTL2Value ^= 0x3300;
        FCTL3Value ^= 0x3300;

        WriteMemWordX(0x12A,0xA544);     // Source = MCLK, DIV = 5.
        WriteMemWordX(0x12C,LockA);      // Set LockA
        tmpFCTL3Value = ReadMemWordX(0x12C);    // Read out register again

        if((LockA & 0xff) != (tmpFCTL3Value & 0xff))
        {   // Value of lockA is not as expected, so toggle it
            WriteMemWordX(0x12C,LockA | 0x40);
        }

        {
            uint32_t registerValues[7] = {Addr, lLen, usType, LockA, 0, R11_DCO, R12_BCSLTC1};
            setFuncletRegisters(registerValues);
            WriteCpuRegX(2, 0);
        }

        // i_SetPcRel
        SetPcX(startAddr);
        // prepare release & release
        cntrl_sig_16bit();
        SetReg_16Bits(0x0401);
        addr_capture();
        cntrl_sig_release();

        // Poll until the funclet reaches the WaitForDead loop,
        // ie. it is ready to process data
        do
        {
            IHIL_Delay_1ms(1);
            addr_capture();
            lOut_long = SetReg_20Bits(0);
            timeOut--;
        }
        while(!((lOut_long >= ((uint32_t)startAddr + WAIT_FOR_DEAD_START)) && (lOut_long <= ((uint32_t)startAddr + WAIT_FOR_DEAD_END))) && timeOut);
    }

    while (lLen && (ret_value == 0) && timeOut)
    {
      uint16_t writePos = startAddr + DATA_OFFSET;
      const uint16_t writeEndPos = writePos + (2*lLen < memSize ? 2*lLen : memSize);
      uint16_t numWordsToWrite = 0;

      SetPcX(startAddr + FINISHED_OFFSET);

      // The device has limited RAM available to write updates to,
      // make sure we stay within this limit: Total memory size - 96 bytes for the funclet
      halt_cpu();
      IHIL_Tclk(0);
      cntrl_sig_low_byte();
      SetReg_16Bits(0x08);

      while( (writePos < writeEndPos) && (ret_value == 0) )
      {
          ret_value = STREAM_get_word(&data);
          addr_16bit();
          SetReg_20Bits(writePos);
          data_to_addr();
          SetReg_16Bits(data);
          IHIL_Tclk(1);
          IHIL_Tclk(0);
          writePos += 2;
      }

      release_cpu();

      numWordsToWrite = (writePos - (startAddr + DATA_OFFSET)) / 2;

	  WriteCpuRegX(REG_SIZE, numWordsToWrite);
      WriteCpuRegX(REG_ADDRESS, Addr);

      Addr += 2 * numWordsToWrite;
      lLen -= numWordsToWrite;

      SetPcX(startAddr + EXECUTE_FUNCLET);
      cntrl_sig_release();

      // Poll until the funclet reaches the Stop loop,
      // ie. it is finished
      timeOut = 3000;
      do
      {
          IHIL_Delay_1ms(1);
          addr_capture();
          lOut_long = SetReg_20Bits(0);
          timeOut--;
      }
      while(!(lOut_long == ((uint32_t)startAddr + FINISHED_OFFSET)) && timeOut);
    }

    if(flags & MESSAGE_LAST_MSG )
    { // ExitFlashWrite
        {   //Setup values for watchdog control regsiters
            uint8_t DummyIn[8] = {WDTCTL_ADDRESS & 0xFF,(WDTCTL_ADDRESS >> 8) & 0xFF,
                                        WDTHOLD_DEF,WDTPW_DEF,0,0,0,0};
            STREAM_internal_stream(DummyIn, sizeof(DummyIn), NULL, 0, &stream_tmp);
            (this->*HAL_SyncJtag_Conditional_SaveContextX)(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG);
            STREAM_external_stream(&stream_tmp);
        }
        setFuncletRegisters(registerBackups);
        // Restore the Flash controller registers
        WriteMemWordX(0x128,FCTL1Value);
        WriteMemWordX(0x12A,FCTL2Value);
        WriteMemWordX(0x12C,FCTL3Value);
        tmpFCTL3Value = ReadMemWordX(0x12C);    // Read out register again

        if((FCTL3Value & 0xff) != (tmpFCTL3Value & 0xff))
        {   // Value of lockA is not as expected, so toggle it
            WriteMemWordX(0x12C,FCTL3Value | 0x40);
        }

        STREAM_put_word(ret_len);
    }
    else if(ret_value == 1)
    {
        STREAM_out_change_type(RESPTYP_ACKNOWLEDGE);
        ret_value = 0;
    }
    else
    {
         //Setup values for watchdog control regsiters
         uint8_t DummyIn[8] = {WDTCTL_ADDRESS & 0xFF,(WDTCTL_ADDRESS >> 8) & 0xFF,
                                        WDTHOLD_DEF,WDTPW_DEF,0,0,0,0};
         STREAM_internal_stream(DummyIn, sizeof(DummyIn), NULL, 0, &stream_tmp);
         (this->*HAL_SyncJtag_Conditional_SaveContextX)(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG);
         STREAM_external_stream(&stream_tmp);

         setFuncletRegisters(registerBackups);
         // Restore the Flash controller registers
         WriteMemWordX(0x128,FCTL1Value);
         WriteMemWordX(0x12A,FCTL2Value);
         WriteMemWordX(0x12C,FCTL3Value);
         tmpFCTL3Value = ReadMemWordX(0x12C);    // Read out register again

         if((FCTL3Value & 0xff) != (tmpFCTL3Value & 0xff))
         {   // Value of lockA is not as expected, so toggle it
             WriteMemWordX(0x12C,FCTL3Value | 0x40);
         }

         ret_value = HALERR_EXECUTE_FUNCLET_EXECUTION_ERROR;
    }
    return(ret_value);
#undef setFuncletRegisters
}
/**
* \ingroup MODULMACROSXV2
*
* \file ExecuteFuncletXv2.c
*
* \brief  Execute a funclet on the target
*/


#define startAddrOfs FlashWrite_o_[4]

#define REG_ADDRESS 5
#define REG_SIZE    6
#define REG_LOCKA   8
#define REG_TYPE    9

#define TIMEOUT_COUNT   300u
/*
#define AddrL        FlashWrite_o_[6]
#define AddrH        FlashWrite_o_[7]
#define LengthL      FlashWrite_o_[8]
#define LengthH      FlashWrite_o_[9]
#define LockA        FlashWrite_o_[10]
*/

//extern uint16_t altRomAddressForCpuRead;


void setFuncletRegisters_hal_ExecuteFuncletXv2(const uint32_t* registerData)
{
    uint32_t  Rx;
    uint16_t Mova;
    uint16_t Rx_l;
    uint16_t Register;

    Mova  = 0x0080;
    Rx = registerData[0];
    Register = REG_ADDRESS;
    Mova += (uint16_t)((Rx>>8) & 0x00000F00);
    Mova += (Register & 0x000F);
    Rx_l  = (uint16_t)Rx;
    WriteCpuRegXv2(Mova, Rx_l);

    Mova  = 0x0080;
    Rx = registerData[1];
    Register = REG_SIZE;
    Mova += (uint16_t)((Rx>>8) & 0x00000F00);
    Mova += (Register & 0x000F);
    Rx_l  = (uint16_t)Rx;
    WriteCpuRegXv2(Mova, Rx_l);

    Mova  = 0x0080;
    Rx = registerData[2]; // erase type
    Register = REG_TYPE;
    Mova += (uint16_t)((Rx>>8) & 0x00000F00);
    Mova += (Register & 0x000F);
    Rx_l  = (uint16_t)Rx;
    WriteCpuRegXv2(Mova, Rx_l);

    Mova  = 0x0080;
    Rx = registerData[3];
    Register = REG_LOCKA;
    Mova += (uint16_t)((Rx>>8) & 0x00000F00);
    Mova += (Register & 0x000F);
    Rx_l  = (uint16_t)Rx;
    WriteCpuRegXv2(Mova, Rx_l);
}

/**
  WriteFlashBlockXv2
  Write words (16bit values) to a memory mapped Flash location (in Block mode).
  inData:  <tgtStart(16)> <tgtLen(16)> <addr(32)> <length(32)> <data(16)>{*}
  outData: <tgtLen_used(16)>
  tgtStart: target memory start address used to load erase code to.
  tgtLen: available target memory
  addr: the address to start writing to
  length: number of words to write
  data: data to write to the given location
  tgtLen_used: actual amount of target memory used to execute erase code
  (must be <= length)
*/

struct {
    uint32_t lLen;
    uint16_t ret_len;
    uint32_t registerBackups[4];
    uint16_t allignNeed;
    uint16_t dataL;
    uint16_t dataH;
} _hal_ExecuteFuncletXv2_staticVars = {};

HAL_FUNCTION(_hal_ExecuteFuncletXv2)
{
#define setFuncletRegisters setFuncletRegisters_hal_ExecuteFuncletXv2
    STATIC_VARS_START(_hal_ExecuteFuncletXv2);
    DECL_STATIC_VAR(lLen);
    DECL_STATIC_VAR(ret_len);
    DECL_STATIC_VAR(registerBackups);
    DECL_STATIC_VAR(allignNeed);
    DECL_STATIC_VAR(dataL);
    DECL_STATIC_VAR(dataH);
    
    uint8_t jtagMailboxIn = 0;

    uint16_t tgtStart     =0x0 ;
    uint32_t Addr          =0x0 ;
    uint16_t LockA        =0x0 ;
    uint16_t usType       =0x0 ;

    uint16_t startAddr;
    uint16_t Mova;
    int16_t ret_value = 0;
    StreamSafe stream_tmp;

    if(flags & MESSAGE_NEW_MSG)
    {
        // get target RAM start
        if(STREAM_get_word(&tgtStart) != 0)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_RAM_START);
        }
        STREAM_discard_bytes(2); // Ram size
        /// get RAM Start + Code offset
        if(STREAM_get_word(&startAddr)!= 0)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_OFFSET);
        }
        // get start addres as int32_t value
        if(STREAM_get_long(&Addr) != 0)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_ADDRESS);
        }
        // get length ot be flashed
        if(STREAM_get_long(&lLen) != 0)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_LENGTH);
        }
        if(STREAM_get_word(&usType) != 0)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_TYPE);
        }
        // lock A handling
        if(STREAM_get_word(&LockA) == -1)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_LOCKA);
        }

        if(STREAM_discard_bytes(4) == -1)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_LOCKA);
        }

        {
            uint32_t registerValues[4] = {Addr, lLen, usType, LockA};
            setFuncletRegisters(registerValues);
        }

        // i_SetPcRel
        SetPcXv2(0x0080, startAddr);
        IHIL_Tclk(1);
        // prepare release & release
        cntrl_sig_16bit();
        SetReg_16Bits(0x0401);
        addr_capture();
        cntrl_sig_release();
        {
            uint32_t Timeout = 0;
            do
            {
                IHIL_Delay_1ms(5);    // Delay determined experimentally on FR5729 device
                Timeout++;
            }
            while(i_ReadJmbOut() != 0xABADBABE && Timeout < TIMEOUT_COUNT);
            if(Timeout >= TIMEOUT_COUNT)
            {
                ret_value = HALERR_EXECUTE_FUNCLET_EXECUTION_TIMEOUT;
            }
        }
        if(lLen%2)
        {
             lLen--;  // allign lLen
             allignNeed = 1;
        }
    }

    // Excecute funclet
    for(; lLen && (ret_value == 0); lLen--)
    {
      if (lLen%2)
      {
          ret_value = STREAM_get_word(&dataH);
          i_WriteJmbIn32(dataL,dataH);
      }
      else
      {
          ret_value = STREAM_get_word(&dataL);
      }
    }

    if(allignNeed && ret_value == 0 && lLen == 0 && flags & MESSAGE_LAST_MSG )
    {
         ret_value = STREAM_get_word(&dataL);
         jtagMailboxIn = i_WriteJmbIn32(dataL,0xAAAA);

         if(jtagMailboxIn == 1)
         {
            return (HALERR_EXECUTE_FUNCLET_FINISH_TIMEOUT);
         }
         allignNeed = 0;
    }

    if(flags & MESSAGE_LAST_MSG )//|| lLen == 0)
    { // ExitFlashWrite
        uint32_t Timeout = 0;
        do
        {
            IHIL_Delay_1ms(10);     // Delay determined experimentally on FR5729 device
            Timeout++;
        }
        while(i_ReadJmbOut() != 0xCAFEBABE && Timeout < TIMEOUT_COUNT);
        if(Timeout >= TIMEOUT_COUNT)
        {
            ret_value = HALERR_EXECUTE_FUNCLET_FINISH_TIMEOUT;
            //Setup values for watchdog control regsiters

            uint8_t DummyIn[8] = {(uint8_t)(wdtctlAddress5xx & 0xFF), (uint8_t)((wdtctlAddress5xx >> 8) & 0xFF),
                                    WDTHOLD_DEF,WDTPW_DEF,0,0,0,0};

            STREAM_internal_stream(DummyIn, sizeof(DummyIn), NULL, 0, &stream_tmp);
            (this->*HAL_SyncJtag_Conditional_SaveContextXv2)(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG);
            STREAM_external_stream(&stream_tmp);

            Mova = 0x0060 | ((10 << 8) & 0x0F00);
            registerBackups[0] = ReadCpuRegXv2(Mova);
            Mova = 0x0060 | ((11 << 8) & 0x0F00);
            registerBackups[1] = ReadCpuRegXv2(Mova);

            if( registerBackups[1] != 0 && registerBackups[0] != 0xFFFE)
            {
                ret_value = -10;
            }
        }
        {
            //Setup values for watchdog control regsiters
            uint8_t DummyIn[8] = {(uint8_t)(wdtctlAddress5xx & 0xFF), (uint8_t)((wdtctlAddress5xx >> 8) & 0xFF),
                                    WDTHOLD_DEF,WDTPW_DEF,0,0,0,0};

            STREAM_internal_stream(DummyIn, sizeof(DummyIn), NULL, 0, &stream_tmp);
            (this->*HAL_SyncJtag_Conditional_SaveContextXv2)(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG);
            STREAM_external_stream(&stream_tmp);
        }

        STREAM_put_word(ret_len);
    }
    else if(ret_value == 1)
    {
        STREAM_out_change_type(RESPTYP_ACKNOWLEDGE);
        ret_value = 0;
    }
    else
    {
         //Setup values for watchdog control regsiters
         uint8_t DummyIn[8] = {(uint8_t)(wdtctlAddress5xx & 0xFF), (uint8_t)((wdtctlAddress5xx >> 8) & 0xFF),
                                     WDTHOLD_DEF,WDTPW_DEF,0,0,0,0};

         STREAM_internal_stream(DummyIn, sizeof(DummyIn), NULL, 0, &stream_tmp);
         (this->*HAL_SyncJtag_Conditional_SaveContextXv2)(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG);
         STREAM_external_stream(&stream_tmp);

         ret_value = HALERR_EXECUTE_FUNCLET_EXECUTION_ERROR;
    }
    return(ret_value);
#undef setFuncletRegisters
}
/*
 * \ingroup MODULMACROS
 *
 * \file GetCpuIdArm.c
 *
 * \brief <FILEBRIEF>
 */


/**
  GetJtagIdCode: get the JEP idcode from the debug port
*/
HAL_FUNCTION(_hal_GetCpuIdArm)
{
#ifdef MSP_FET
    uint32_t cpuId = 0, revision = 0;
    uint16_t timeout = 20;

    do
    {
        int16_t  result1 = IHIL_Write_Read_Mem_Ap(0, 0x0020100C, &cpuId, READ);
        int16_t  result2 =IHIL_Write_Read_Mem_Ap(0, 0x00201010, &revision, READ);
        if(!result2 || !result1)
        {
            return HALERR_UNDEFINED_ERROR;
        }
    }
    while((!cpuId || !revision ) && --timeout );

    if (cpuId && revision)
    {
        STREAM_put_long(cpuId);
        STREAM_put_long(revision);
        return 0;
    }
#endif
    return HALERR_UNDEFINED_ERROR;
}

HAL_FUNCTION(_hal_CheckDapLockArm)
{
#ifdef MSP_FET
    uint32_t lock = 0;
    uint32_t pw = 0x0000695A;

    IHIL_Write_Read_Mem_Ap(0, 0xE0044000, &pw, WRITE);
    IHIL_Write_Read_Mem_Ap(0, 0xE0044020, &lock, READ);
    STREAM_put_long(lock);
#endif
    return 0;
}

HAL_FUNCTION(_hal_UnlockDap)
{
#ifdef MSP_FET
    uint32_t pw = 0x0000695A;

    uint32_t fa1 = 0x1;
    uint32_t fa2 = 0x0;
    uint32_t fa4 = 0x6902;

    IHIL_Write_Read_Mem_Ap(0, 0xE0044000, &pw, WRITE);

    IHIL_Write_Read_Mem_Ap(0, 0xE0044004, &fa1, WRITE);
    IHIL_Write_Read_Mem_Ap(0, 0xE0044008, &fa2, WRITE);
    IHIL_Write_Read_Mem_Ap(0, 0xE0044010, &fa4, WRITE);

#endif
    return 0;
}
/**
* \ingroup MODULMACROS
*
* \file GetDcoFrequency.c
*
* \brief <FILEBRIEF>
*/


#define FlashUpperBoarder 2140000ul // 2,14 MHz
#define FlashLowerBoarder 1410000ul // 1,41 MHz

uint16_t loopDco[24] =
{
    0x40b2, 0x5a80, 0x0120, 0xc232, 0xc0f2, 0x003f, 0x0057, 0xd0f2,
    0x0007, 0x0057, 0x45c2, 0x0056, 0x46c2, 0x0057, 0x43c2, 0x0058,
    0xea0a, 0xe909, 0x5319, 0x23fe, 0x531a, 0x23fc, 0x4303, 0x3fff
};
const uint16_t sizeLoopDco = (uint16_t)(sizeof(loopDco)/sizeof(*loopDco));

uint16_t loopFll[29] =
{
    0x40b2, 0x5a80, 0x0120, 0xc232, 0xd072, 0x0040, 0x4032, 0x0040,
    0x40f2, 0x0080, 0x0052, 0x43c2, 0x0050, 0x45c2, 0x0051, 0xd0f2,
    0x0080, 0x0053, 0xc0f2, 0x005f, 0x0054, 0xea0a, 0xe909, 0x5319,
    0x23fe, 0x531a, 0x23fc, 0x4303, 0x3fff
};
const uint16_t sizeLoopFll = (uint16_t)(sizeof(loopFll)/sizeof(*loopFll));

/*--------------------------------------------------------------------------------*/
#ifdef MSP430_UIF
    uint16_t Time = 0;

    #pragma vector=TIMERA1_VECTOR
    __interrupt void TIMER_A_ISR_(void)
    {
        Time++;
        TACTL &= ~TAIFG;
    }

    void StartTimer()
    {
        TACTL =  0;                                             // STOP Timer
        TACTL &= ~CCIFG;                                            // Clear the interrupt flag
        TACTL =  TASSEL_2;                                      // Timer is runnig at 1 Mhz
        TACCR0 = 0x2E9;                                      // Load CCR0 with delay... (1ms delay)
        TAR = 0;
        TACTL |= TACLR + MC_1 + TAIE;                               // Start Timer
    }

    void StopTimer()
    {
        TACTL &= ~MC_1;
        TACTL &= ~TAIE;                               // Start Timer
    }
#endif

/*--------------------------------------------------------------------------------*/

#ifdef eZ_FET
    void StartTimer()
    {
        TB0CTL = 0;                                             // STOP Timer
        TB0CTL = ID__8 + TBSSEL__SMCLK;                         // Timer_B source:SMCLK ,SMCLK/0 = 25
        TB0CCR0 = 0x9cf;                                        // Load CCR0 with delay... (1ms delay)
        TB0CCTL0 &= ~CCIFG;                                     // Clear the interrupt flag
        TB0CCTL0 |= CCIE ;
        TB0CTL |= TBCLR + MC__UP;
    }

    void StopTimer()
    {
        TB0CTL &= ~MC__UP;
        TB0CTL = 0;                                             // STOP Timer
        // restore timer B0 1ms delay value
        IHIL_InitDelayTimer();
    }
#endif

///*--------------------------------------------------------------------------------*/
//
//#ifdef MSP_FET
//    void StartTimer()
//    {
//        TA2CTL = 0;                                             // STOP Timer
//        TA2CTL = ID__8 + TASSEL__SMCLK;                         // Timer_A source:SMCLK ,SMCLK/0 = 25
//        TA2CCR0 = 0x7d8;                                        // Load CCR0 with delay... (1ms delay)
//        TA2CCTL0 &= ~CCIFG;                                     // Clear the interrupt flag
//        TA2CCTL0 |= CCIE ;
//        TA2CTL |= TACLR + MC__UP;
//    }
//
//    void StopTimer()
//    {
//        TA2CTL &= ~MC__UP;
//        TA2CTL = 0;                                             // STOP Timer
//        // restore timer A2 1ms delay value
//        IHIL_InitDelayTimer();
//    }
//#endif

uint32_t (MSPFETSim::*ReadCounterRegsFunc)() = nullptr;
void (MSPFETSim::*WriteRegFunc)(int16_t, uint32_t) = nullptr;
void (MSPFETSim::*SetPCFunc)(uint32_t) = nullptr;
void (MSPFETSim::*WriteRamFunc)(uint16_t, const uint16_t*, uint16_t) = nullptr;
void (MSPFETSim::*ReadRamFunc)(uint16_t, uint16_t*, uint16_t) = nullptr;
HalFuncInOut SyncFunc = nullptr;

void readFromRam(uint16_t address, uint16_t* buffer, uint16_t numWords)
{
    while (numWords-- > 0)
    {
        *buffer = ReadMemWord(address);
        address += 2;
        ++buffer;
    }
}

void writeToRam(uint16_t address, const uint16_t* data, uint16_t numWords)
{
    while (numWords-- > 0)
    {
        WriteMemWord(address, *data);
        address += 2;
        ++data;
    }
}

void readFromRamX(uint16_t address, uint16_t* buffer, uint16_t numWords)
{
    while (numWords-- > 0)
    {
        *buffer = ReadMemWordX(address);
        address += 2;
        ++buffer;
    }
}

void writeToRamX(uint16_t address, const uint16_t* data, uint16_t numWords)
{
    while (numWords-- > 0)
    {
        WriteMemWordX(address, *data);
        address += 2;
        ++data;
    }
}

uint32_t readCounterRegisters()
{
    uint16_t r9 = 0 , r10 = 0;
    uint32_t counter = 0;

    r9 = ReadCpuReg_uint16_t(9);
    r10 = ReadCpuReg_uint16_t(10);
    counter = r10;
    return (counter << 16) | r9;
}

uint32_t readCounterRegistersX()
{
    uint32_t r9 = 0, r10 = 0;

    r9 = ReadCpuRegX(9);
    r10 = ReadCpuRegX(10);
    return (r10 << 16) | r9;
}

void writeRegister(int16_t r, uint32_t value)
{
    WriteCpuReg(r, value);
}

void writeRegisterX(int16_t r, uint32_t value)
{
    WriteCpuRegX(r, value);
}

void setPC(uint32_t address)
{
    SetPc(address);
}

void setPCX(uint32_t address)
{
    SetPcX(address);
}

void setPCJtag(uint32_t address)
{
    SetPcJtagBug(address);
    IHIL_Tclk(1);
}


uint32_t measureFrequency(uint16_t RamStart, uint16_t DCO, uint16_t BCS1) {
    return 0;
}

//uint32_t measureFrequency(uint16_t RamStart, uint16_t DCO, uint16_t BCS1)
//{
//    uint32_t startTime;
//    uint32_t stopTime;
//    uint32_t elapseTime = 0;
//    uint32_t counter = 0;
//    uint32_t freq = 0;
//
//    #if defined(eZ_FET) || defined(MSP_FET)
//    uint16_t *TimerTick = 0;
//    #endif
//    WriteRegFunc(5, DCO);
//    WriteRegFunc(6, BCS1);
//
//    SetPCFunc(RamStart);
//    cntrl_sig_release();
//
//    #if defined(eZ_FET) || defined(MSP_FET)
//        STREAM_getSharedVariable(ID_SHARED_MEMORY_TYPE_TIMER_TICK, &TimerTick);
//    #endif
//
//    StartTimer(); // Time the delay (as we could be interrupted, etc.).
//
//    #if defined(eZ_FET) || defined(MSP_FET)
//        startTime =  *(uint16_t*)TimerTick; // System timestamp (in milliseconds).
//        __delay_cycles(800000ul);
//        stopTime =  *(uint16_t*)TimerTick;
//    #endif
//
//    #ifdef MSP430_UIF
//        startTime = Time; // System timestamp (in milliseconds).
//        __delay_cycles(240000ul); //~30ms
//        stopTime = Time;
//    #endif
//
//    StopTimer();
//    {
//        StreamSafe stream_tmp;
//        uint8_t DummyIn[8] = {0x20,0x01,0x80,0x5A,0,0,0,0}; // wdtAddr, wdtCtrl
//        STREAM_internal_stream(DummyIn, sizeof(DummyIn), 0, 0, &stream_tmp);
//        SyncFunc(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG);
//        STREAM_external_stream(&stream_tmp);
//    }
//
//    elapseTime = startTime < stopTime ? stopTime - startTime : startTime - stopTime; // System time can wrap.
//
//    counter = ReadCounterRegsFunc();
//
//    // Calculate a device speed (cycles/second). Multiply counter value by 3 (3 clock cycles per loop),
//    // and add 31 cycles for funclet setup, then normalize to one second.
//    freq = ((counter * 3 + 31) * 1000) / elapseTime;
//
//    #if defined(eZ_FET) || defined(MSP_FET)
//        STREAM_deleteSharedVariable(ID_SHARED_MEMORY_TYPE_TIMER_TICK);
//    #endif
//    return freq;
//}



int16_t findDcoSettings(uint16_t jtagBug)
{
    uint16_t MAXRSEL   = 0x7;
    uint16_t DCO  = 0x0;
    uint16_t BCS1 = 0x6;
    uint16_t BackupRam[30];
    uint16_t RamStart = 0;
    uint32_t  DcoFreq = 0;

    // get target RAM start
    if(STREAM_get_word(&RamStart) < 0)
    {
        return(HALERR_EXECUTE_FUNCLET_NO_RAM_START);
    }

    if(STREAM_get_word(&MAXRSEL) < 0)
    {
        return(HALERR_EXECUTE_FUNCLET_NO_MAXRSEL);
    }

    if (MAXRSEL == 0xF)
    {
        BCS1 = 0x9;
    }

    (this->*SetPCFunc)(ROM_ADDR); //Prevent Ram corruption on F123/F413

    //----------------Backup original Ram content--------------
    (this->*ReadRamFunc)(RamStart, BackupRam, sizeLoopDco);

    // ----------------Download DCO measure funclet------------
    (this->*WriteRamFunc)(RamStart, loopDco, sizeLoopDco);

    // do measurement
    int16_t allowedSteps = 40;

    do
    {
        DcoFreq = measureFrequency(RamStart, (DCO<<5), (0x80|BCS1));
        //Ram content will probably be corrupted after each measurement on devices with jtag bug
        //Reupload on every iteration
        if (jtagBug)
        {
            (this->*WriteRamFunc)(RamStart, loopDco, sizeLoopDco);
        }

        if (DcoFreq == 0)
        {
            return (-1);
        }

        if (DcoFreq > FlashUpperBoarder) // Check for upper limit - 10%.
        {
            if(DCO-- == 0)
            {
                DCO = 7;
                if (BCS1-- == 0)
                {
                    return (-1); // Couldn't get DCO working with correct frequency.
                }
            }
        }
        else if (DcoFreq < FlashLowerBoarder) // Check for lower limit + 10%.
        {
            if(++DCO > 7)
            {
                DCO = 0;
                if (++BCS1 > MAXRSEL)
                {
                    return (-1); // Couldn't get DCO working with correct frequency.
                }
            }
        }
        else
        {
            break;
        }
    }
    while (--allowedSteps > 0);

    // restore Ram content
    (this->*WriteRamFunc)(RamStart, BackupRam, sizeLoopDco);

    if (allowedSteps <= 0)
    {
        return (-1); // Couldn't get DCO working with correct frequency.
    }
    // measurement end

    // return measured values
    STREAM_put_word( (DCO<<5) );
    STREAM_put_word( (0x80|BCS1) );
    STREAM_put_word( 0 );
    return 0;
}


int16_t findFllSettings(uint16_t jtagBug)
{
    uint16_t first = 0, last = 27, mid, reload;
    uint16_t BackupRam[30];
    uint16_t RamStart = 0;
    uint32_t  DcoFreq = 0;

    // get target RAM start
    if(STREAM_get_word(&RamStart) < 0)
    {
        return(HALERR_EXECUTE_FUNCLET_NO_RAM_START);
    }

    (this->*SetPCFunc)(ROM_ADDR); //Prevent Ram corruption on F123/F413

    //----------------Backup original Ram content--------------
    (this->*ReadRamFunc)(RamStart, BackupRam, sizeLoopFll);

    // ----------------Download FLL measure funclet------------
    (this->*WriteRamFunc)(RamStart, loopFll, sizeLoopFll);

    // Binary search through the available frequencies selecting the highest frequency < (476KHz - 10%).
    while (first + 1 < last)
    {
         mid = (last + first) / 2;
         reload = 0;

        // Select DCO range from 0.23MHz to 11.2MHz. Specify frequency via Ndco. Disable Modulation. Enable DCO+.
        DcoFreq = measureFrequency(RamStart, (mid << 3), 0);

        //Ram content will probably be corrupted after each measurement on devices with jtag bug
        //Reupload on every iteration
        if (jtagBug)
        {
            (this->*WriteRamFunc)(RamStart, loopFll, sizeLoopFll);
        }


        if (DcoFreq == 0)
        {
            break;
        }
        else
        {
            if (DcoFreq > FlashUpperBoarder) // Max. Flash Controller frequency - 10%.
            {
                last = mid;
                reload = 1;
            }
            else
            {
                first = mid;
            }
        }
    }
    if (reload)
    {
          DcoFreq = measureFrequency(RamStart, (first << 3), 0);
    }

    // restore Ram content
    (this->*WriteRamFunc)(RamStart, BackupRam, sizeLoopDco);

    if (((DcoFreq < 257000 * 5) || (DcoFreq > 476000 * 5)))
    {
        return -1;
    }
    // measurement end

    // return measured values
    STREAM_put_word(0x00);
    STREAM_put_word( (first << 3) );
    STREAM_put_word(0x80);
    STREAM_put_word(0x80);
    STREAM_put_word(0);
    return 0;
}


HAL_FUNCTION(_hal_GetDcoFrequency)
{
    ReadCounterRegsFunc = &MSPFETSim::readCounterRegisters;
    WriteRegFunc = &MSPFETSim::writeRegister;
    SetPCFunc = &MSPFETSim::setPC;
    WriteRamFunc = &MSPFETSim::writeToRam;
    ReadRamFunc = &MSPFETSim::readFromRam;
    SyncFunc = HAL_SyncJtag_Conditional_SaveContext;
    return findDcoSettings(0);
}

HAL_FUNCTION(_hal_GetDcoFrequencyJtag)
{
    ReadCounterRegsFunc = &MSPFETSim::readCounterRegisters;
    WriteRegFunc = &MSPFETSim::writeRegister;
    SetPCFunc = &MSPFETSim::setPCJtag;
    WriteRamFunc = &MSPFETSim::writeToRam;
    ReadRamFunc = &MSPFETSim::readFromRam;
    SyncFunc = HAL_SyncJtag_Conditional_SaveContext;
    return findDcoSettings(1);
}

HAL_FUNCTION(_hal_GetDcoFrequencyX)
{
    ReadCounterRegsFunc = &MSPFETSim::readCounterRegistersX;
    WriteRegFunc = &MSPFETSim::writeRegisterX;
    SetPCFunc = &MSPFETSim::setPCX;
    WriteRamFunc = &MSPFETSim::writeToRamX;
    ReadRamFunc = &MSPFETSim::readFromRamX;
    SyncFunc = HAL_SyncJtag_Conditional_SaveContextX;
    return findDcoSettings(0);
}

HAL_FUNCTION(_hal_GetFllFrequency)
{
    ReadCounterRegsFunc = &MSPFETSim::readCounterRegisters;
    WriteRegFunc = &MSPFETSim::writeRegister;
    SetPCFunc = &MSPFETSim::setPC;
    WriteRamFunc = &MSPFETSim::writeToRam;
    ReadRamFunc = &MSPFETSim::readFromRam;
    SyncFunc = HAL_SyncJtag_Conditional_SaveContext;
    return findFllSettings(0);
}

HAL_FUNCTION(_hal_GetFllFrequencyJtag)
{
    ReadCounterRegsFunc = &MSPFETSim::readCounterRegisters;
    WriteRegFunc = &MSPFETSim::writeRegister;
    SetPCFunc = &MSPFETSim::setPCJtag;
    WriteRamFunc = &MSPFETSim::writeToRam;
    ReadRamFunc = &MSPFETSim::readFromRam;
    SyncFunc = HAL_SyncJtag_Conditional_SaveContext;
    return findFllSettings(1);
}

HAL_FUNCTION(_hal_GetFllFrequencyX)
{
    ReadCounterRegsFunc = &MSPFETSim::readCounterRegistersX;
    WriteRegFunc = &MSPFETSim::writeRegisterX;
    SetPCFunc = &MSPFETSim::setPCX;
    WriteRamFunc = &MSPFETSim::writeToRamX;
    ReadRamFunc = &MSPFETSim::readFromRamX;
    SyncFunc = HAL_SyncJtag_Conditional_SaveContextX;
    return findFllSettings(0);
}
/**
* \ingroup MODULMACROS
*
* \file GetDeviceIdPtr.c
*
* \brief <FILEBRIEF>
*/


/**
  GetJtagId: required as separate macro beside StartJtag to get the JtagId
             of an individual device in the JTAG chain
*/
HAL_FUNCTION(_hal_GetDeviceIdPtr)
{
    uint16_t JtagId = 0;
    uint16_t CoreIpId = 0;
    int32_t DeviceIpPointer = 0;
    int32_t IdDataAddr = 0;
    uint32_t lOut_long = 0;

    JtagId = cntrl_sig_capture();
    if (!jtagIdIsValid(JtagId))
    {
        return HALERR_UNDEFINED_ERROR;
    }
    else if (jtagIdIsXv2(JtagId))
    {
        // Get Core identification info
        core_ip_pointer();
        CoreIpId = SetReg_16Bits(0);
        // Get device identification pointer
        if(JtagId == JTAGVERSION95)
        {
            IHIL_Delay_1ms(1500);
        }
        device_ip_pointer();
        lOut_long = SetReg_20Bits(0);
        // The ID pointer is an un-scrambled 20bit value
        DeviceIpPointer = ((lOut_long & 0xFFFF) << 4 )  + (lOut_long >> 16 );
        if(DeviceIpPointer)
        {
            IdDataAddr = DeviceIpPointer + 4;
        }
    }
    else
    {
        IdDataAddr = 0xFF0;
    }

    STREAM_put_long(DeviceIpPointer);
    STREAM_put_long(IdDataAddr);
    STREAM_put_word(CoreIpId);
    return 0;
}
/**
* \ingroup MODULMACROSXV2
*
* \file GetInterfaceMode.c
*
* \brief <FILEBRIEF>
*/


#if defined(MSP430_UIF) || defined(MSP_FET)
HAL_FUNCTION(_hal_GetInterfaceMode)
{
    uint16_t id=0, loopCount=7,i=0, protocol =0;
//    #ifdef MSP_FET
//        HilInitGetEdtDistinctFunc hilEdtDis = ((void*)0);
//    #endif

    // create known state
    #ifndef MSP_FET
        IHIL_Close();
    #endif
    for (i = 0; i < loopCount; i++)
    {
        // set JTAG mode 1xx- 4xx
        if(i == 0 || i == 3)// set JTAG
        {
            protocol = JTAG;
        }
        else
	    {
            // set SBW2 mode all devices
			if(i == 2 || i == 5)
			{
                protocol = SPYBIWIRE;
			}
            // set SBW4 mode all devices
			else if(i == 1|| i ==4 || i ==6)
			{
                protocol = SPYBIWIREJTAG;
			}
        }
        IHIL_SetProtocol(protocol);
        #ifdef MSP_FET
            // MSPFETSim: not sure what this craziness is, but its result is calling `_hil_getEdtDistinct`
//            HilInitGetEdtDistinctFunc hilEdtDis = (HilInitGetEdtDistinctFunc)0x1880;
//            hilEdtDis(&_edt_Distinct_Methods_HAL);
            _hil_getEdtDistinct(&_edt_Distinct_Methods_HAL);
        #endif

        // Run  4wire/SBW entry Sequence & Reset high
        IHIL_Open(RSTHIGH);
        // Reset TAP state machine -> Run-Test/Idle
        IHIL_TapReset();
        // Run Fuse Check
        IHIL_CheckJtagFuse();
        // shift out JTAG ID
        id = cntrl_sig_capture();
        // Release JTAG, to set the interface into an known state /
        #ifndef MSP_FET
            IHIL_Close();
        #endif
        // now check for vaild JTAG ID
        if (jtagIdIsValid(id))
        {
            STREAM_put_word(id);
            STREAM_put_word(protocol);
            return 0;
        }
    }
    // Error no mode found
    STREAM_put_word(0xFFFF);
    STREAM_put_word(0xAAAA);
    return HALERR_UNDEFINED_ERROR;
}
#endif

#if defined(eZ_FET)
HAL_FUNCTION(_hal_GetInterfaceMode)
{
    STREAM_put_word(0x99);
    STREAM_put_word(SPYBIWIRE);
    return 0;

}
#endif
/**
* \ingroup MODULMACROSXV2
*
* \file GetInterfaceModeArm.c
*
* \brief <FILEBRIEF>
*/


HAL_FUNCTION(_hal_GetInterfaceModeArm)
{
    #if defined(MSP430_UIF) || defined(MSP_FET)
    uint32_t idCode = 0;
    uint16_t loopCount = 4, i = 0, protocol = 0;

//    #ifdef MSP_FET
//        HilInitGetEdtDistinctFunc hilEdtDis = ((void*)0);
//    #endif

    // create known state
    #ifndef MSP_FET
        IHIL_Close();
    #endif
    for (i = 0; i < loopCount; i++)
    {
        // set JTAG 4 mode all ARM devices
        if(i == 1 || i == 3)
        {
            #ifdef MSP_FET
                protocol = SWD_432;
            #endif
        }
        // set SWD  mode for all devices
        else if(i == 0|| i == 2 )
        {
            protocol =  JTAG_432;
        }

        IHIL_SetProtocol(protocol);
        #ifdef MSP_FET
            // MSPFETSim: not sure what this craziness is, but its result is calling `_hil_getEdtDistinct`
//            HilInitGetEdtDistinctFunc hilEdtDis = (HilInitGetEdtDistinctFunc)0x1880;
//            hilEdtDis(&_edt_Distinct_Methods_HAL);
            _hil_getEdtDistinct(&_edt_Distinct_Methods_HAL);
        #endif


        IHIL_Open(RSTHIGH);
        // Reset TAP state machine -> Run-Test/Idle
        IHIL_TapReset();
        // Run Fuse Check
        idCode = CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.GetJtagIdCode)();

        if (idCode != 0xFFFFFFFE)
        {
            STREAM_put_word(idCode);
            STREAM_put_word(protocol);
            return 0;
        }
    }
    #endif
    // Error no mode found
    STREAM_put_word(0xFFFF);
    STREAM_put_word(0xAAAA);
    return HALERR_UNDEFINED_ERROR;
}


/**
* \ingroup MODULMACROS
*
* \file GetJtagId.c
*
* \brief <FILEBRIEF>
*/


/**
  GetJtagId: required as seperate macro beside StartJtag to get the JtagId
             of an individual device in the JTAG chain
*/
HAL_FUNCTION(_hal_GetJtagId)
{
    int16_t i;
    uint16_t JtagId = 0;

    for (i = 0; i < 4; ++i)
    {
        JtagId = cntrl_sig_capture();
        // now get device id pointer and core ip id
        if (jtagIdIsValid(JtagId))
        {
            STREAM_put_word(JtagId);
            return 0;
        }
    }// end of JTAG id scan
     return HALERR_UNDEFINED_ERROR;
}
/*
 * \ingroup MODULMACROS
 *
 * \file GetJtagIdCodeArm.c
 *
 * \brief <FILEBRIEF>
 */


/**
  GetJtagIdCode: get the JEP idcode from the debug port
*/
HAL_FUNCTION(_hal_GetJtagIdCodeArm)
{
#if defined(MSP430_UIF) || defined(MSP_FET)
    uint32_t JtagId = CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.GetJtagIdCode)();
    if (JtagId)
    {
        STREAM_put_long(JtagId);
        return 0;
    }
#endif
    return HALERR_UNDEFINED_ERROR;
}
/**
* \ingroup MODULMACROS
*
* \file GetNumOfDevices.c
*
* \brief <FILEBRIEF>
*/

/**
  GetNumOfDevices
  Scans a JTAG chain for available devices and
  reports back the number of devices.
  inData:  -
  outData: <numOfDevices(8)>
  numOfDevices: Number of devices connected through a JTAG chain.
*/

HAL_FUNCTION(_hal_GetNumOfDevices)
{
  uint8_t numOfDevices;
#ifdef DB_PRINT
  prtstr("   @ID_GetNumOfDevices<");
#endif

  numOfDevices = 1;

  STREAM_put_byte((uint8_t)numOfDevices);
  return 0;
}
/*
 * \ingroup MODULMACROS
 *
 * \file HaltArm.c
 *
 * \brief <FILEBRIEF>
 */


/**
  Halt: Halts the Core
*/
//#if defined(MSP430_UIF) || defined(MSP_FET)
//extern ARMConfigSettings armConfigSettings;
//#endif

HAL_FUNCTION(_hal_HaltArm)
{
    int16_t retVal = 0;
#if defined(MSP430_UIF) || defined(MSP_FET)
    uint16_t* syncWithRunVarAddress = getTargetRunningVar();
    if(syncWithRunVarAddress)
    {
        *syncWithRunVarAddress = 0x0000;
    }
    // Set system and debug power up if cleared
    if( powerUpArm() != 0)
    {
        return HALERR_UNDEFINED_ERROR;
    }

    if(checkWakeup() != 0)
    {
        retVal = HALERR_UNDEFINED_ERROR;
    }

    uint8_t retry = MAX_RETRY;

    uint32_t data = DBGKEY | C_HALT | C_DEBUGEN;
    IHIL_Write_Read_Mem_Ap(0, DHCSR, &data, WRITE);

    do
    {
        IHIL_Write_Read_Mem_Ap(0, DHCSR, &data, READ);
    } while(!(data & S_HALT) && --retry);

    if(!retry)
    {
        retVal = HALERR_UNDEFINED_ERROR;
    }

    // Read out why did we halt?
    IHIL_Write_Read_Mem_Ap(0, DFSR, &data, READ);
    STREAM_put_long(data);
#endif
    return retVal;
}
/**
* \ingroup MODULMACROS
*
* \file HilCommand.c
*
* \brief <FILEBRIEF>
*/

/**
  HilCommand
  Low level Hil command to execute or instruction/data to shift.
  inData:  <command(32)> (<data(32)> <bits(32)>)
  outData: -
*/

typedef enum {
    HIL_CMD_RESET_JTAG_TAP,
    HIL_CMD_OPEN,
    HIL_CMD_CONNECT,
    HIL_CMD_CLOSE,
    HIL_CMD_JTAG_IR,
    HIL_CMD_JTAG_DR,
    HIL_TEST_REG,
    HIL_TEST_REG_3V,
    HIL_CMD_QUERY_JSTATE,
    HIL_CMD_WRITE_JMB,
    HIL_CMD_READ_JMB,
    HIL_CMD_CONFIGURE,
    HIL_CMD_BSL,
    HIL_CMD_FUSE_CHECK,
    HIL_CMD_JTAG_IR4,
    HIL_CMD_DPACC,
    HIL_CMD_APACC,
    HIL_CMD_MEM_AP,
    HIL_CMD_BSL_1XX_4XX,
    HIL_CMD_TCLK,
} HIL_COMMAND;

HAL_FUNCTION(_hal_HilCommand)
{
    uint64_t shiftedOut = 0;
    uint32_t command = 0;
    uint32_t dataLow = 0;
    uint32_t dataHigh = 0;
    uint64_t data = 0;
    uint32_t bits = 16;
    int16_t streamResult = 0;
    int16_t retVal = 0;

    while (streamResult == 0)
    {
        uint64_t value = 0;

        if( STREAM_get_long(&command) < 0 )
        {
            return HALERR_NO_COMMAND;
        }
        if ( STREAM_get_long(&dataLow) < 0 )
        {
            return HALERR_NO_DATA;
        }
        if ( STREAM_get_long(&dataHigh) < 0 )
        {
            return HALERR_NO_DATA;
        }
        streamResult = STREAM_get_long(&bits);
        if (streamResult < 0)
        {
            return HALERR_NO_BIT_SIZE;
        }

        data = dataLow | ((uint64_t)dataHigh << 32);

        if (command == HIL_CMD_JTAG_IR)
        {
            uint8_t bitCounter;

            // Force 8-bits for IR shifts
            // This is necessary because the instruction is shifted into the target MSB-first
            // by the SPI interface
            bits = 8;

            for(bitCounter = 0; bitCounter < bits; ++bitCounter)
            {
                value <<= 1;
                value |= (data >> bitCounter) & 1;
            }
        }
        else
        {
            value = data;
        }

        switch (command)
        {
        case HIL_CMD_RESET_JTAG_TAP:
            IHIL_TapReset();
            break;

        case HIL_CMD_FUSE_CHECK:
            IHIL_CheckJtagFuse();
            break;

        case HIL_CMD_OPEN:
        case HIL_CMD_CONNECT:
            IHIL_Open((uint8_t)dataLow);
            break;

        case HIL_CMD_BSL:
            IHIL_BSL_EntrySequence(1);
            break;

       case HIL_CMD_BSL_1XX_4XX:
            IHIL_BSL_EntrySequence1xx_4xx();
            break;

        case HIL_CMD_CLOSE:
            IHIL_Close();
            if (dataLow != 0)
            {
                IHIL_SetVcc(0);
            }
            break;

        case HIL_CMD_JTAG_IR:
            shiftedOut = CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.Instr)((uint8_t)value);
            STREAM_put_long( (uint32_t)(shiftedOut &0xFFFFFFFF) );
            STREAM_put_long( (uint32_t)(shiftedOut >> 32) );
            break;

        case HIL_CMD_JTAG_IR4:
            shiftedOut = IHIL_Instr4((uint8_t)value);
            STREAM_put_long( (uint32_t)(shiftedOut &0xFFFFFFFF) );
            STREAM_put_long( (uint32_t)(shiftedOut >> 32) );
            break;

        case HIL_CMD_JTAG_DR:
            switch (bits)
            {
            case  8:
                shiftedOut = SetReg_8Bits((uint8_t)value & 0xFF);
                break;
            case 16:
                shiftedOut = SetReg_16Bits((uint16_t)value & 0xFFFF);
                break;
            case 20:
                shiftedOut = SetReg_20Bits((uint32_t)value & 0xFFFFF);
                break;
            case 32:
                shiftedOut = SetReg_32Bits((uint32_t)value & 0xFFFFFFFF);
                break;
            case 64:
                shiftedOut = SetReg_64Bits(value);
                break;
            case 35:
                shiftedOut = SetReg_35Bits(&value);
                break;
            default:
                return HALERR_INVALID_BIT_SIZE;
            }
            STREAM_put_long( (uint32_t)(shiftedOut &0xFFFFFFFF) );
            STREAM_put_long( (uint32_t)(shiftedOut >> 32) );
            break;

        case HIL_TEST_REG:
           test_reg();
           SetReg_32Bits((uint32_t)value);
           break;

        case HIL_TEST_REG_3V:
           test_reg_3V();
           SetReg_16Bits((uint16_t)value);
           break;

        case HIL_CMD_QUERY_JSTATE:
           jstate_read();
           shiftedOut = SetReg_64Bits(0x0000000000000000);
           STREAM_put_long( (uint32_t)(shiftedOut &0xFFFFFFFF) );
           STREAM_put_long( (uint32_t)(shiftedOut >> 32) );
           break;

        case HIL_CMD_TCLK:
           IHIL_Tclk((uint8_t)dataLow);
           break;
           
        case HIL_CMD_WRITE_JMB:
          {
            uint8_t jtagMailboxIn = 0;
            if(bits == 16)
            {
                jtagMailboxIn = i_WriteJmbIn((uint16_t)dataLow);
            }
            else if(bits == 32)
            {
                jtagMailboxIn = i_WriteJmbIn32((uint16_t)dataLow, (uint16_t)(dataLow >> 16));
            }
            else
            {
                return (HALERR_JTAG_MAILBOX_IN_TIMOUT);
            }

            if(jtagMailboxIn == 1)
            {
                return (HALERR_JTAG_MAILBOX_IN_TIMOUT);
            }
            break;
          }
        case HIL_CMD_READ_JMB:
            shiftedOut = i_ReadJmbOut();
            STREAM_put_long( (uint32_t)(shiftedOut &0xFFFFFFFF) );
            STREAM_put_long( (uint32_t)(shiftedOut >> 32) );
            break;

        case HIL_CMD_CONFIGURE:
            IHIL_SetProtocol(dataLow);

            {
                // MSPFETSim: not sure what this craziness is, but its result is calling `_hil_getEdtDistinct`
//                HilInitGetEdtDistinctFunc hilEdtDis = (HilInitGetEdtDistinctFunc)0x1880;
//                hilEdtDis(&_edt_Distinct_Methods_HAL);
                _hil_getEdtDistinct(&_edt_Distinct_Methods_HAL);
            }
            break;

        case HIL_CMD_DPACC:
            retVal = (IHIL_Write_Read_Dp((uint8_t)dataLow, // address
                                        &dataHigh,              // data
                                        (uint16_t)bits)   // rnw
                                         > 0) ? 0 : HALERR_DAP_NACK;
            STREAM_put_long(dataHigh);
            break;

        case HIL_CMD_APACC:
            retVal = (IHIL_Write_Read_Ap(dataLow,              // address | APSEL
                                        &dataHigh,            // data
                                        (uint16_t)bits) // rnw
                                         > 0) ? 0 : HALERR_DAP_NACK;
            STREAM_put_long(dataHigh);
            break;

        case HIL_CMD_MEM_AP:
            retVal = (IHIL_Write_Read_Mem_Ap((uint16_t)(bits >> 16),// ap_sel
                                            dataLow,                     // address
                                            &dataHigh,                   // data
                                            (uint16_t)bits)        // rnw
                                            > 0) ? 0 : HALERR_DAP_NACK;
              STREAM_put_long(dataHigh);

            break;

        default:
          retVal = HALERR_UNKNOWN_COMMAND;
        }
    }
    return retVal;
}

/**
* \ingroup MODULMACROSXV2
*
* \file IsJtagFuseBlown.c
*
* \brief Check if the JTAG fuse is blown
*/

/**
  IsJtagFuseBlown

*/

HAL_FUNCTION(_hal_IsJtagFuseBlown)
{
    uint16_t  i = 0, lOut = 0;
    uint16_t jtagId = 0;

    for (i = 0; i < 3; i++) // First test could be negative.
    {
        jtagId = cntrl_sig_capture();
        if(jtagId != JTAGVERSION99 && jtagId != JTAGVERSION98 && jtagId != JTAGVERSION95 && jtagId != JTAGVERSION91 && jtagId != JTAGVERSION)
        {
            return -1;
        }

        lOut = SetReg_16Bits(0xaaaa);
        // If the fuse is blown, the device will be in JTAG bypass mode, and data
        // input to the device will be echoed with a one-bit delay.
        if (lOut == 0x5555)
        {
            STREAM_put_word(lOut); // Fuse is blown.
        }
    }
    STREAM_put_word(lOut); // Fuse is not blown.

    return 0;
}
/**
* \ingroup MODULMACROSXV2
*
* \file LeaSyncConditional.c
*
* \brief <FILEBRIEF>
*/
/*
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
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


#define LEASCCNF1 0x08
#define LEASCBSY  0x01

HAL_FUNCTION(_hal_LeaSyncConditional)
{
    uint32_t baseAddres = 0;
    uint16_t tclkCycles = 20000;
    volatile uint32_t currentPcAddress = 0;
    uint16_t leaReg = 0;

    STREAM_get_long(&baseAddres);

    leaReg = ReadMemWordXv2(baseAddres | LEASCCNF1);
    if(!(leaReg & LEASCBSY))
    {
        STREAM_put_word((leaReg & LEASCBSY));
        return 0;
    }

    // Set PC to "safe" address
    SetPcXv2(0x0080, SAFE_PC_ADDRESS);
    cntrl_sig_16bit();
    SetReg_16Bits(0x0501);
    IHIL_Tclk(1);
    addr_capture();

    cntrl_sig_16bit();
    SetReg_16Bits(0x1501);

    while(tclkCycles-- > 0)
    {
        IHIL_TCLK();
    }
    addr_capture();
    currentPcAddress = SetReg_20Bits(0x0);

    if((currentPcAddress != SAFE_PC_ADDRESS) && (currentPcAddress != SAFE_PC_ADDRESS+2))
    {
        return -1;
    }

    leaReg = ReadMemWordXv2(baseAddres | LEASCCNF1);
    STREAM_put_word((leaReg & LEASCBSY));
    return 0;
}
/**
* \ingroup MODULMACROS
*
* \file MagicPattern.c
*
* \brief Get control over the device
*/

/**
  MagicPattern
  This function will try to get control over the device. It will try to reset and
  stop the device.
  It will automatically switch between the different JTAG protocols also it handles
  the LPMx.5

  inData:  void
  outData: <chainLen(8)>
  outData: <jtagId(8)>

  protocol: 0 = 4-wire Jtag
  jtagId: the Jtag identifier value
*/

#define BOOT_DATA_CRC_WRONG 0xC3C3

//#ifdef uController_uif
//extern void JSBW_EntrySequences(uint8_t states);
//extern void JSBW_TapReset(void);
//extern void JSBW_MagicPattern(void);
//extern void JSBW_JtagUnlock(void);
//extern void jRelease(void);
//#endif

#define BOOT_DATA_CRC_WRONG 0xC3C3

uint16_t magicPattern = 0;

#ifdef MSP430_UIF
    int16_t magicPatternJsbw2();
#endif

//#pragma optimize = medium
HAL_FUNCTION(_hal_MagicPattern)
{
    uint16_t id = 0;
    uint8_t chainLen = 1;
    uint16_t protocol = SPYBIWIRE;

    STREAM_get_word(&protocol);

#ifdef eZ_FET
    if(protocol !=  SPYBIWIRE)
    {
       return HALERR_MAGIC_PATTERN;
    }
#endif

#ifdef MSP430_UIF
    if(protocol == SPYBIWIRE_MSP_FET)
    {
        return magicPatternJsbw2();
    }
#endif

    IHIL_SetProtocol(protocol);
#if defined(eZ_FET) || defined(MSP_FET)
    {
        // MSPFETSim: not sure what this craziness is, but its result is calling `_hil_getEdtDistinct`
//        HilInitGetEdtDistinctFunc hilEdtDis = (HilInitGetEdtDistinctFunc)0x1880;
//        hilEdtDis(&_edt_Distinct_Methods_HAL);
        _hil_getEdtDistinct(&_edt_Distinct_Methods_HAL);
    }
#endif
    // run entry sequnce but pull rst low during the sequence to wake-up the device

    IHIL_Close();

    CALL_MEMBER_FN_PTR(_edt_Common_Methods_HAL.regulateVcc)();
    
    IHIL_Open(RSTLOW);
    IHIL_TapReset();
    IHIL_CheckJtagFuse();
    // put in magic pattern to stop user code execution

    if(i_WriteJmbIn(MAGIC_PATTERN) == 1)
    {
        return (HALERR_JTAG_MAILBOX_IN_TIMOUT);
    }

    // run entry sequnce but pull rst high during the sequence
    IHIL_Open(RSTHIGH);
    IHIL_TapReset();
    IHIL_CheckJtagFuse();

    // Corrupted CRC handling - start --------------------------------------
    // read JTAG mailbox to see if CRC of Bootdata is damaged.
    if(i_ReadJmbOut16() == BOOT_DATA_CRC_WRONG)
    {
        return (HALERR_MAGIC_PATTERN_BOOT_DATA_CRC_WRONG);
    }
    // Corrupted CRC handling - end ----------------------------------------

    id = cntrl_sig_capture() ;
    if (jtagIdIsXv2(id))
    {
        //Disable Lpmx5 and power gaiting settings
        if(id == JTAGVERSION98)
        {
            // just disable JTAG io lock
            test_reg_3V();
            SetReg_16Bits(0x4020);
        }
        if(id == JTAGVERSION99)
        {
            test_reg_3V();
            SetReg_16Bits(0x40A0);
            test_reg();
            SetReg_32Bits(0x00010000);
        }
        STREAM_put_byte((uint8_t)chainLen);
        STREAM_put_byte((uint8_t)id);

#ifdef MSP_FET
        if (protocol == SPYBIWIRE_MSP_FET)
        {
            // MSPFETSim: not sure what this craziness is, but its result is calling `_hil_getEdtDistinct`
//            HilInitGetEdtDistinctFunc hilEdtDis = (HilInitGetEdtDistinctFunc)0x1880;
//            hilEdtDis(&_edt_Distinct_Methods_HAL);
            _hil_getEdtDistinct(&_edt_Distinct_Methods_HAL);
            protocol = SPYBIWIREJTAG;
            IHIL_SetProtocol(protocol);
            IHIL_Open(RSTHIGH);
            IHIL_TapReset();
            IHIL_CheckJtagFuse();
        }
#endif
        STREAM_put_byte((uint8_t)protocol);

        return 0;
    }
    return (HALERR_MAGIC_PATTERN);
}

#ifdef MSP430_UIF

// Force wakeup in SBW4 mode for LPMx.5 because JTAG pins are locked by LPMx.5
// This case happens if the device is woken up, but goes into LPMx.5 very quickly
// and the device wasn't under JTAG control from a previous debug session
int16_t magicPatternJsbw2()
{
    uint16_t i = 0;
    uint16_t id = 0;
    uint8_t chainLen = 1;
    uint16_t protocol = SPYBIWIREJTAG;
    IHIL_SetProtocol(protocol);

    while( i < 3)
    {
        //  feet in JSBW magic pattern Use real RST and TST pin of TOOL
        JSBW_EntrySequences(0);
        JSBW_TapReset();
        JSBW_MagicPattern();
        IHIL_Delay_1ms(10);
        jRelease();

        // disable JTAG lock
        JSBW_EntrySequences(2);
        JSBW_TapReset();
        JSBW_JtagUnlock();
        IHIL_Delay_1ms(10);

        // reconnect in normal JTAG 4 wire mode
        IHIL_Open(RSTHIGH);
        IHIL_TapReset();
        IHIL_CheckJtagFuse();

        id = cntrl_sig_capture();
        if (jtagIdIsXv2(id))
        {
            //Disable Lpmx5 and power gaiting settings
            if(id == JTAGVERSION98)
            {
                // just disable JTAG i lock
                test_reg_3V();
                SetReg_16Bits(0x4020);
            }
            if(id == JTAGVERSION99)
            {
                test_reg_3V();
                SetReg_16Bits(0x40A0);
                test_reg();
                SetReg_32Bits(0x00010000);
            }
            STREAM_put_byte((uint8_t)chainLen);
            STREAM_put_byte((uint8_t)id);
            STREAM_put_byte((uint8_t)protocol);

            return 0;
        }
        i++;
    }
    return (HALERR_MAGIC_PATTERN);
}
#endif
/*
 * \ingroup MODULMACROS
 *
 * \file MEMAPTransactionArm.c
 *
 * \brief <FILEBRIEF>
 */


/**
  MEMAPTransaction
  Performs a memory transaction (Read/Write) on the specified port
  PortNum<8>: selects the port number
  RnW<8>: selects whether to perform a Read or a Write trasaction
  Datawidth<16>: Indicates the width of the data transaction (8-/16-/32-bits)
  Address<32>: Destination address
  Count<32>: The number of bytes to write
*/
//#if defined(MSP430_UIF) || defined(MSP_FET)
//    extern uint32_t cswValues[4];
//#endif

#define ADDRESS_WRAPPING 0x00000FFFul

/// \brief Low-level write function with timeout
/// \return 0 if timeout occured and non-zero if success
//#pragma inline=forced
uint8_t writeLowLevel(uint8_t address, uint32_t dataIn)
{
    uint8_t retry = MAX_RETRY;
    uint8_t ack = 0;
    uint64_t dataIn64 = ((uint64_t)dataIn) << 3;

    do
    {
        uint64_t tmp = dataIn64 | (address >> 1) | WRITE;
        ack = SetReg_35Bits(&tmp) & 0x7;
    } while((ack != ACK) && (--retry));

    return retry;
}

/// \brief Low-level read function with timeout
/// \return 0 if timeout occured and non-zero if success
//#pragma inline=forced
uint8_t readLowLevel(uint8_t address, uint32_t *dataInOut)
{
    uint8_t retry = MAX_RETRY;
    uint64_t dataIn64 = ((uint64_t)*dataInOut) << 3;
    uint64_t dataOut64;

    do
    {
        uint64_t tmp = dataIn64 | (address >> 1) | READ;
        dataOut64 = SetReg_35Bits(&tmp);
    }
    while(((dataOut64 & 0x7) != ACK) && (--retry));

    *dataInOut = dataOut64 >> 3;

    return retry;
}

void SwdHandleError()
{
    uint32_t errorCheck = 0, resetError = 0;
    // hanlde locked out state
    IHIL_Write_Read_Dp(DP_DPIDR, &errorCheck, READ);
    // read
    IHIL_Write_Read_Dp(DP_CTRL_STAT, &errorCheck, READ);

    if(errorCheck & DP_CTRL_STAT_STICKYCMP)
    {
        resetError |= STKCMPCLR;
    }
    if(errorCheck & DP_CTRL_STAT_STICKYERR)
    {
        resetError |= STKERRCLR;
    }
    if(errorCheck & DP_CTRL_STAT_WDATAERR)
    {
        resetError |= WDERRCLR;
    }
    if(errorCheck & DP_CTRL_STAT_STICKYORUN)
    {
        resetError |= ORUNERRCLR;
    }
    if(resetError != 0)
    {
        IHIL_Write_Read_Dp(DP_DPIDR, &resetError, WRITE);
    }
}

int16_t JtagHandleError()
{
    int16_t retVal = 0;
    uint32_t errorCheck = 0, resetError = 0;
    // Check for errors
    //prepare read CTRL/STAT reg
    IHIL_Write_Read_Dp(DP_CTRL_STAT, &errorCheck, READ);
    //read CTRL/STAT reg
    IHIL_Write_Read_Dp(DP_CTRL_STAT, &errorCheck, READ);

    resetError = errorCheck;
    if(errorCheck & DP_CTRL_STAT_STICKYCMP)
    {
        resetError |= DP_CTRL_STAT_STICKYCMP;
    }
    if(errorCheck & DP_CTRL_STAT_STICKYERR)
    {
        resetError |= DP_CTRL_STAT_STICKYERR;
    }
    if(errorCheck & DP_CTRL_STAT_WDATAERR)
    {
        resetError |= DP_CTRL_STAT_WDATAERR;
    }
    if(errorCheck & DP_CTRL_STAT_STICKYORUN)
    {
        resetError |= DP_CTRL_STAT_STICKYORUN;
    }
    if(resetError != 0)
    {
        IHIL_Write_Read_Dp(DP_CTRL_STAT, &resetError, WRITE);

         //prepare read CTRL/STAT reg
        IHIL_Write_Read_Dp(DP_CTRL_STAT, &errorCheck, READ);
        //read CTRL/STAT reg
        IHIL_Write_Read_Dp(DP_CTRL_STAT, &errorCheck, READ);
        if(errorCheck & DP_CTRL_STAT_STICKYERR ||
           errorCheck & DP_CTRL_STAT_STICKYORUN ||
           errorCheck & DP_CTRL_STAT_STICKYCMP ||
           errorCheck & DP_CTRL_STAT_WDATAERR)
        {
            retVal = -1;
        }
        else
        {
            retVal = 0;
        }
    }
    else
    {
        retVal = 0;
    }
    return retVal;
}

/// \brief Stream the correct data size back to the DLL
#define STREAM_DATA(data)                                     \
    do                                                        \
    {                                                         \
        data >>= (address & 0x3) * 8;                         \
        switch(dataWidth)                                     \
        {                                                     \
        case AP_CSW_SIZE_8BIT:                                \
            STREAM_put_byte((uint8_t)data);             \
            break;                                            \
        case AP_CSW_SIZE_16BIT:                               \
            STREAM_put_word((uint16_t)data);            \
            break;                                            \
        case AP_CSW_SIZE_32BIT:                               \
            STREAM_put_long((uint32_t)data);             \
            break;                                            \
        }                                                     \
    } while(0)


struct {
    uint64_t apsel;
    uint16_t rnw;
    uint16_t dataWidth;
    uint32_t address;
    uint32_t count;
} _hal_MemApTransactionArm_staticVars = {};

HAL_FUNCTION(_hal_MemApTransactionArm)
{
    STATIC_VARS_START(_hal_MemApTransactionArm);
    DECL_STATIC_VAR(apsel);
    DECL_STATIC_VAR(rnw);
    DECL_STATIC_VAR(dataWidth);
    DECL_STATIC_VAR(address);
    DECL_STATIC_VAR(count);
    
    int16_t retVal = 0;
#if defined(MSP430_UIF) || defined(MSP_FET)

    uint8_t *pBuf;
    uint16_t sizeOfBuf;

    uint32_t data;

    if(flags & MESSAGE_NEW_MSG)
    {   // Do initial setup
        apsel = 0ull;
        if(STREAM_get_word((uint16_t *)&apsel) == -1)
        {
            return -1;
        }
        if(STREAM_get_word(&rnw) == -1)
        {
            return -1;
        }
        if(STREAM_get_word(&dataWidth) == -1)
        {
            return -1;
        }
        if(STREAM_get_long(&address) == -1)
        {
            return -1;
        }
        if(STREAM_get_long(&count) == -1)
        {
            return -1;
        }
        if(!count)
        { // Nothing to do, just return without failure
            return 0;
        }

        apsel <<= 24;
        count >>= dataWidth; // Calculate the number of transactions needed

        JtagHandleError();

        // Write CSW register
        data = cswValues[apsel & 0x3] | ((count == 1) ? AP_CSW_ADDRINC_OFF : AP_CSW_ADDRINC_SINGLE) | dataWidth;
        IHIL_Write_Read_Ap(AP_CSW, &data, WRITE);

        // Write TAR register
        IHIL_Write_Read_Ap(AP_TAR, &address, WRITE);

    }

    // Now that the setup is done, do the actual data transfer
    if(rnw == WRITE)
    {
        STREAM_out_change_type(RESPTYP_ACKNOWLEDGE);
        STREAM_flush();
        STREAM_get_buffer((void **)&pBuf,&sizeOfBuf);
        while(count && sizeOfBuf)
        {
            switch(dataWidth)
            {
            case 0:
              data = (*((uint8_t*) pBuf));
              break;
            case 1:
              data = (*((uint16_t*) pBuf));
              break;
            default:
              data = (*((uint32_t*) pBuf));
              break;
            }
            data <<= (address & 0x3) * 8;  // Move to correct byte lane

            writeLowLevel(AP_DRW, data);

            pBuf += (1 << dataWidth);
            address += (1 << dataWidth);
            if(sizeOfBuf >= (1 << dataWidth))
            {
                sizeOfBuf -= (1 << dataWidth);
            }
            else
            {
                sizeOfBuf = 0;
            }
            --count;

            // Check for address wrapping
            if(count && !(address & ADDRESS_WRAPPING))
            { // Write TAR register
                writeLowLevel(AP_TAR, address);
            }
        }
    }
    else
    { // READ
        data = 0;
        // Initiate read
        readLowLevel(AP_DRW,&data);

        while(--count)
        {
            readLowLevel(AP_DRW,&data);
            STREAM_DATA(data);
            // Check for address wrapping
            if(count)
            {
                address += (1 << dataWidth);
                if(!(address & ADDRESS_WRAPPING))
                { // Write TAR register
                    JtagHandleError();
                    IHIL_Instr4(IR4_APACC);
                    writeLowLevel(AP_TAR, address);
                    readLowLevel(AP_DRW,&data);
                }
            }
        }
        IHIL_Instr4(IR4_DPACC);
        readLowLevel(DP_RDBUFF,&data);
        STREAM_DATA(data);
    }

    if(count)
    { // More data expected
        retVal = 1;
    }
    else
    {
        data = 0;
        IHIL_Write_Read_Dp(DP_RDBUFF, &data, READ);
        retVal = JtagHandleError();
    }

#endif
    return retVal;
}


struct {
    uint64_t apsel;
    uint16_t rnw;
    uint16_t dataWidth;
    uint32_t address;
    uint32_t count;
} _hal_MemApTransactionArmSwd_staticVars = {};

HAL_FUNCTION(_hal_MemApTransactionArmSwd)
{
    STATIC_VARS_START(_hal_MemApTransactionArmSwd);
    DECL_STATIC_VAR(apsel);
    DECL_STATIC_VAR(rnw);
    DECL_STATIC_VAR(dataWidth);
    DECL_STATIC_VAR(address);
    DECL_STATIC_VAR(count);
    
    int16_t retVal = 0;
#ifdef MSP_FET

    uint8_t *pBuf;
    uint16_t sizeOfBuf;

    uint32_t data;

    if(flags & MESSAGE_NEW_MSG)
    {   // Do initial setup
        apsel = 0ull;
        if(STREAM_get_word((uint16_t *)&apsel) == -1)
        {
            return -1;
        }
        if(STREAM_get_word(&rnw) == -1)
        {
            return -1;
        }
        if(STREAM_get_word(&dataWidth) == -1)
        {
            return -1;
        }
        if(STREAM_get_long(&address) == -1)
        {
            return -1;
        }
        if(STREAM_get_long(&count) == -1)
        {
            return -1;
        }
        if(!count)
        { // Nothing to do, just return without failure
            return 0;
        }
        count >>= dataWidth; // Calculate the number of transactions needed

        // enalbe ORUNDETECT in CTRL/STAT
        IHIL_Write_Read_Dp(DP_CTRL_STAT, &data, READ);
        data |= 0x00000001;
        IHIL_Write_Read_Dp(DP_CTRL_STAT, &data, WRITE);


        // Write CSW register
        data = cswValues[apsel & 0x3] | ((count == 1) ? AP_CSW_ADDRINC_OFF : AP_CSW_ADDRINC_SINGLE) | dataWidth;
        IHIL_Write_Read_Ap(AP_CSW, &data, WRITE);

        // Write TAR register
        IHIL_Write_Read_Ap(AP_TAR, &address, WRITE);

        if(rnw == READ)
        {
            uint64_t retVal = 0, i = 0;
            // dummy read
            uint8_t regiser  = SWD_AP | READ << 1 | (AP_DRW & 0x0c);
            do
            {
                retVal = IHIL_SwdTransferData(regiser, &data, READ);
                SwdHandleError();
            }
            while(++i < MAX_RETRY && retVal != SWD_ACK);

            if(i == MAX_RETRY)
            {
                return -1;
            }
            // dummy read end
        }
    }
    if(rnw == WRITE)
    {
        STREAM_get_buffer((void **)&pBuf,&sizeOfBuf);
        while(count && sizeOfBuf)
        {
            uint64_t retVal = 0, i = 0;
            // Move to correct byte lane
            switch(dataWidth)
            {
            case 0:
              data = (*((uint8_t*) pBuf));
              break;
            case 1:
              data = (*((uint16_t*) pBuf));
              break;
            default:
              data = (*((uint32_t*) pBuf));
              break;
            }
            data <<= (address & 0x3) * 8;

            uint8_t regiser  = SWD_AP | WRITE << 1 | (AP_DRW & 0x0c);
            do
            {
                retVal = IHIL_SwdTransferData(regiser, &data, WRITE);
            }
            while(++i < MAX_RETRY && retVal != SWD_ACK);

            if(i == MAX_RETRY)
            {
                    return -1;
            }

            pBuf += (1 << dataWidth);
            address += (1 << dataWidth);
            if(sizeOfBuf >= (1 << dataWidth))
            {
                sizeOfBuf -= (1 << dataWidth);
            }
            else
            {
                sizeOfBuf = 0;
            }
            --count;

            // Check for address wrapping
            if(count && !(address & ADDRESS_WRAPPING))
            { // Write TAR register
                 IHIL_Write_Read_Ap(AP_TAR, &address, WRITE);
                 SwdHandleError();
            }
        }
    }
    else
    {
        //Read
        uint64_t retVal = 0, i = 0;
        uint8_t regiser  = SWD_AP | READ << 1 | (AP_DRW & 0x0c);

        while(--count)
        {
            i = 0;
            data = 0;
            do
            {
                retVal = IHIL_SwdTransferData(regiser, &data, READ);
            }
            while(++i < MAX_RETRY && retVal != SWD_ACK);
            if(i == MAX_RETRY)
            {
                data = 0;
            }
            STREAM_DATA(data);

            if(count)
            {
                address += (1 << dataWidth);
                if(!(address & ADDRESS_WRAPPING))
                {
                    SwdHandleError();

                    // Write TAR register
                    IHIL_Write_Read_Ap(AP_TAR, &address, WRITE);
                    //Trigger new read
                    i = 0;
                    do
                    {
                        retVal = IHIL_SwdTransferData(regiser, &data, READ);
                        SwdHandleError();
                    }
                    while(++i < MAX_RETRY && retVal != SWD_ACK);
                }
            }
        }
    }

    if(count)
    { // More data expected
        retVal = 1;
    }
    else
    {
        IHIL_Write_Read_Dp(DP_RDBUFF, &data, READ);
        STREAM_DATA(data);
        SwdHandleError();

        // disable ORUNDETECT in CTRL/STAT
        IHIL_Write_Read_Dp(DP_CTRL_STAT, &data, READ);
        data &= ~0x00000001;
        IHIL_Write_Read_Dp(DP_CTRL_STAT, &data, WRITE);
    }
#endif
    return retVal;
}
/**
* \ingroup MODULMACROS
*
* \file PollJStateReg.c
*
* \brief <FILEBRIEF>
*/



#define JSTATE20  0x20
#define JSTATE21  0x21
#define DSTATE_PC 0x00

#define ARM_PCMCTL0 (0x40010000UL)
#define ARM_PCSR    (0xE000101CUL)

//-----------------------------------------------------------------------------
#if defined(eZ_FET) || defined(MSP_FET)
//-----------------------------------------------------------------------------

#pragma pack(1)
typedef struct DState432
{
    uint32_t pc;
    uint32_t pstate;
} DState432_t;
#pragma pack()

#pragma pack(1)
typedef struct DState430
{
    uint64_t JState;
} DState430_t;
#pragma pack()

#pragma pack(1)
typedef struct EnergyTraceRecordEt7
{
    uint8_t eventID;
    uint32_t TimeStamp;
    union
    {
        DState430_t dstate430;
        DState432_t dstate432;
    };
    uint32_t currentTicks;
    uint16_t voltage;
} EnergyTraceRecordEt7_t;
#pragma pack()

#pragma pack(1)
typedef struct EnergyTraceRecordEt8
{
    uint8_t eventID;
    uint32_t TimeStamp;
    uint32_t currentTicks;
    uint16_t voltage;
} EnergyTraceRecordEt8_t;
#pragma pack()

#define NUM_RECORDS_TO_CAPTURE 5

typedef enum ETMode
{
    ET_OFF_MODE = 0,
} ETMode_t;


//uint32_t getTimeStamp();
//uint32_t getIMeasure();

//#pragma inline=forced
uint32_t getTimeStamp()
{
    // MSPFETSim: disable EnergyTrace stuff
    UNIMP_FN();
    return 0;
//    uint32_t TimeStamp = 0;
//    uint16_t currentTA0R = TA0R;
//    uint16_t currentTA0R2 = TA0R;
//    uint16_t * TimeTick = 0;
//    uint16_t testa = 0;
//
//    if (currentTA0R2 < currentTA0R)
//    {
//        currentTA0R = currentTA0R2;
//    }
//
//    STREAM_getSharedVariable(ID_SHARED_MEMORY_TYPE_TIME_TICK, &TimeTick);
//    testa = *(uint16_t*)TimeTick;
//
//    TimeStamp = (((uint32_t)testa) << 16) + (uint32_t)(currentTA0R & 0xFFFF);
//    return TimeStamp;
}

//#pragma inline=forced
uint32_t getIMeasure()
{
    // MSPFETSim: disable EnergyTrace stuff
    UNIMP_FN();
    return 0;
//    uint32_t IMeasure = 0;
//#ifdef MSP_FET
//    uint16_t currentIClocks = TB0R;
//    uint16_t currentIClocks2 = TB0R;
//#else
//    uint16_t currentIClocks = TA2R;
//    uint16_t currentIClocks2 = TA2R;
//#endif
//    uint16_t* ITick = 0;
//    uint16_t testt = 0;
//
//    if (currentIClocks2 < currentIClocks)
//    {
//        currentIClocks = currentIClocks2;
//    }
//
//    STREAM_getSharedVariable(ID_SHARED_MEMORY_TYPE_I_TICK, &ITick);
//    testt = *(uint16_t*)ITick;
//
//    IMeasure = (((uint32_t)testt) << 16) + (uint32_t)(currentIClocks & 0xFFFF);
//    return IMeasure;
}
//-----------------------------------------------------------------Event type 8 only analog ------------------------------------------------------------------

struct {
    EnergyTraceRecordEt8_t buffer[NUM_RECORDS_TO_CAPTURE];
    uint16_t currentIndex;
} _hal_PollJStateRegEt8_staticVars = {};

HAL_FUNCTION(_hal_PollJStateRegEt8)
{
    STATIC_VARS_START(_hal_PollJStateRegEt8);
    DECL_STATIC_VAR(buffer);
    DECL_STATIC_VAR(currentIndex);
    
    double vcc = 0;
    double extVcc= 0 ;
    uint16_t mEtGatedMode = 0;
    uint16_t* syncWithRunVarAddress = 0;

    if(STREAM_discard_bytes(8) == -1)
    {
            return -1;
    }
    // request shared var from bios to sync with RestoreContextRun function

    STREAM_get_word(&mEtGatedMode);

    if(mEtGatedMode)
    {
        syncWithRunVarAddress = getTargetRunningVar();
        if(syncWithRunVarAddress)
        {
             if(!(*syncWithRunVarAddress))
             {
                currentIndex = 0;
                return 2;
             }
        }
        else
        {
            return -1;
        }
    }

    buffer[currentIndex].eventID = 8;

    // MSPFETSim: disable timer-related stuff
//    while(TA0R > 0xFFA0 || TA0R  < 2)
//    {
//        IHIL_Delay_1us(3);
//    }

    _DINT_FET();

    buffer[currentIndex].TimeStamp = getTimeStamp();
    buffer[currentIndex].currentTicks = getIMeasure();

    IHIL_GetVcc(&vcc, &extVcc);

    _EINT_FET();
    buffer[currentIndex].voltage = (uint16_t)(vcc ? vcc : extVcc);

    // Energy trace data is available send it first -> don't check LPMx.5 or Breakpoint hit
    if(++currentIndex == NUM_RECORDS_TO_CAPTURE)
    {
        currentIndex = 0;
        STREAM_put_word(ENERGYTRACE_INFO);                  // Event ID
        STREAM_put_byte(NUM_RECORDS_TO_CAPTURE);            // Number of records that is sent
        STREAM_put_byte(sizeof(EnergyTraceRecordEt8_t));    // Size of Record
        STREAM_put_bytes((void *)buffer, sizeof(EnergyTraceRecordEt8_t) * NUM_RECORDS_TO_CAPTURE);
        return 1;
    }

    return 2;
}
//-----------------------------------------------------------------------------
#endif
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
#ifdef MSP430_UIF
//-----------------------------------------------------------------------------
uint16_t targetIsRunning = 0;

HAL_FUNCTION(_hal_PollJStateRegEt8)
{
    return -1;
}
//-----------------------------------------------------------------------------
#endif
//-----------------------------------------------------------------------------

//#pragma inline=forced
int16_t PollforBpHit(uint64_t JStateValue)
{
    uint16_t* syncWithRunVarAddress = 0;
    if((!(JStateValue & LPMX5_MASK_J)) && (JStateValue & BP_HIT_MASK_J))
    {
        STREAM_put_word(BP_HIT_FLAG);
        STREAM_put_long(JStateValue & 0xFFFFFFFF);
        STREAM_put_long(JStateValue >> 32);

        // Stop polling in next loop run --> Device CPU is stoped due to BP Hit
        syncWithRunVarAddress = getTargetRunningVar();
        if(syncWithRunVarAddress)
        {
             *syncWithRunVarAddress = 0x0000;
        }
        return  1;
    }
    return 2;
}

uint64_t prevJState = 0x0000000000000000;

//#pragma inline=forced
int16_t PollforLPMx5(uint64_t JStateValue, uint16_t forceSendState)
{
    StreamSafe stream_tmp;
    int16_t RetState = -1, LPMx5Wakeup = 0;

    uint16_t* syncWithRunVarAddress = 0;

    // if we enter an LPM4 and the PC indicates taht the device wakes up form
    // LPMx.5 scan mailbox for Bootcode pattern
    if (!(JStateValue & LPMX5_MASK_J) &&
      (((JStateValue & LPM4_1MASK_J) == LPM4_1MASK_J) || ((JStateValue & LPM4_2MASK_J) == LPM4_2MASK_J)))
    {
        if (jmb_exchange() == JTAGVERSION99)
        {
            // check if JTAG mailbox is ready & perform input request
            if (SetReg_16Bits(0x0004) == 0x1207)
            {
                if (SetReg_16Bits(0x0000) == 0xA55A)
                {
                    LPMx5Wakeup = 1;
                }
            }
        }
    }

    if(forceSendState || ((prevJState & LPMX5_MASK_J) != (JStateValue & LPMX5_MASK_J)) || LPMx5Wakeup)
    {
        STREAM_put_word(JSTATE_CAPTURE_FLAG);
        STREAM_put_long(JStateValue & 0xFFFFFFFF);
        STREAM_put_long(JStateValue >> 32);

        if(((!(JStateValue & LPMX5_MASK_J)) && !forceSendState) || (LPMx5Wakeup && !forceSendState))
        {   //Device woke up on its own, so sync again
            //Setup values for watchdog control regsiters
            uint8_t DummyIn[8] = {(uint8_t)(wdtctlAddress5xx & 0xFF), (uint8_t)((wdtctlAddress5xx >> 8) & 0xFF),
                                     WDTHOLD_DEF,WDTPW_DEF,0,0,0,0};
            STREAM_internal_stream(DummyIn, sizeof(DummyIn), NULL, 0, &stream_tmp);
            (this->*HAL_SyncJtag_Conditional_SaveContextXv2)(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG);
            STREAM_external_stream(&stream_tmp);

             // Stop polling in next loop run --> Device CPU is stoped due to BP Hit
            syncWithRunVarAddress = getTargetRunningVar();
            if(syncWithRunVarAddress)
            {
                *syncWithRunVarAddress = 0x0000;
            }
        }
        RetState = 1;
    }
    else
    {
        RetState = 2;
    }
    if(forceSendState)
    {
        RetState = 0;
    }
    else
    {
        prevJState = JStateValue;
    }
    return RetState;
}

//-----------------------------------------------------------------Event type 7----------------------------------------------------------------------

/**
  PollJStateReg
  Queries the JSTATE register and reports any changes in the relevant bits
  inData:  <forceSendState(16)> <pollBreakpoints(16)> <pollLPM(16)> <doEnergyTrace(16)> <gateMode(16)>
  outData: <eventFlag(16)> [<JStateLow(32)> <JStateHigh(32)> / <numEnergyTraceRecords> <energyTrace records>]
*/
//int16_t PollJStateReg(uint16_t JStateVersion);

HAL_FUNCTION(_hal_PollJStateReg20)
{
    return PollJStateReg(JSTATE20);
}

HAL_FUNCTION(_hal_PollJStateReg)
{
    return PollJStateReg(JSTATE21);
}

HAL_FUNCTION(_hal_PollDStatePCRegEt)
{
    return PollJStateReg(DSTATE_PC);
}

struct {
    EnergyTraceRecordEt7_t buffer[NUM_RECORDS_TO_CAPTURE];
    uint16_t currentIndex;
} PollJStateReg_staticVars = {};

int16_t PollJStateReg(uint16_t JStateVersion)
{
    STATIC_VARS_START(PollJStateReg);
    DECL_STATIC_VAR(buffer);
    DECL_STATIC_VAR(currentIndex);
    
    uint64_t lOut_long_long = 0;
#if defined(MSP_FET)
    uint32_t lOut_long = 0;
#endif
    int16_t RetState = -1;
    uint16_t forceSendState = 0;

    uint16_t mBreakpointPollingActive = 0;
    uint16_t mLpmPollingActive = 0;
    uint16_t mEtActive = 0;
    uint16_t* syncWithRunVarAddress = 0;

    STREAM_get_word(&forceSendState);
    STREAM_get_word(&mBreakpointPollingActive);
    STREAM_get_word(&mLpmPollingActive);
    STREAM_get_word(&mEtActive);

    if(STREAM_discard_bytes(2) == -1)
    {
        return -1;
    }

    syncWithRunVarAddress = getTargetRunningVar();
    if(!syncWithRunVarAddress)
    {
        return -1;
    }

    if(IHIL_GetPrevInstruction() != IR_JSTATE_ID)
    {
        // Query new JSTATE
        jstate_read();
    }

#if defined(eZ_FET) || defined(MSP_FET)
    // check if Energy Trace should be enabled
    if(*syncWithRunVarAddress && mEtActive)
    {
        double vcc = 0;
        double extVcc = 0;

        buffer[currentIndex].eventID = 7;

        // MSPFETSim: disable timer-related stuff
//        while(TA0R > 0xFFA0 || TA0R  < 2)
//        {
//            IHIL_Delay_1us(3);
//        }

        _DINT_FET();

        buffer[currentIndex].TimeStamp = getTimeStamp();
        buffer[currentIndex].currentTicks = getIMeasure();

        IHIL_GetVcc(&vcc, &extVcc);

        _EINT_FET();

#if defined(MSP_FET)
        if(JStateVersion == DSTATE_PC)
        {
            // read out PC sampling register
            IHIL_Write_Read_Mem_Ap(0, ARM_PCSR, &lOut_long, READ);
            buffer[currentIndex].dstate432.pc = lOut_long;
            // get PCMCTL0 register
            IHIL_Write_Read_Mem_Ap(0, ARM_PCMCTL0, &lOut_long, READ);
            buffer[currentIndex].dstate432.pstate = lOut_long;
        }
        else
        {
#endif
            lOut_long_long = SetReg8_64Bits(0xFFFFFFFFFFFFFFFF,30,JStateVersion);
            buffer[currentIndex].dstate430.JState = lOut_long_long;
#if defined(MSP_FET)
        }
#endif
        buffer[currentIndex].voltage = (uint16_t)(vcc ? vcc : extVcc);

        // Energy trace data is available send it first -> don't check LPMx.5 or Breakpoint hit
        if(++currentIndex == NUM_RECORDS_TO_CAPTURE)
        {
            currentIndex = 0;
            STREAM_put_word(ENERGYTRACE_INFO);                  // Event ID
            STREAM_put_byte(NUM_RECORDS_TO_CAPTURE);            // Number of records that is sent
            STREAM_put_byte(sizeof(EnergyTraceRecordEt7_t));    // Size of Record
            STREAM_put_bytes((void *)buffer, sizeof(EnergyTraceRecordEt7_t) * NUM_RECORDS_TO_CAPTURE);
            return 1;
        }
    }
    else
#endif
    {
        if(!(*syncWithRunVarAddress))
        {
            if(forceSendState)
            {
                STREAM_put_word(JSTATE_CAPTURE_FLAG);
                STREAM_put_long(0);
                STREAM_put_long(0);
                return 0;
            }
            else
            {
                return 2;
            }
        }
        lOut_long_long = SetReg_64Bits(0xFFFFFFFFFFFFFFFF);
    }

    if(*syncWithRunVarAddress && mBreakpointPollingActive)
    {
        RetState = PollforBpHit(lOut_long_long);
        if(RetState == 1)
        {
            return RetState;
        }
    }

    if((*syncWithRunVarAddress || forceSendState) && mLpmPollingActive && (lOut_long_long>>56 & 0xC0) != 0x40)
    {
        RetState = PollforLPMx5(lOut_long_long, forceSendState);
        if(RetState == 1 || RetState == 0)
        {
            return RetState;
        }
    }
    return 2;
}
/**
* \file PollStateReg430I.c
*
* \brief Write words (16bit values) to a memory mapped location
*/

//! \ingroup MODULMACROS
//! \file PollJStateReg430I.c
//! \brief


/**
  PollJStateReg430I
  Queries the JSTATE register and reports any changes in the relevant bits
  inData:  <forceSendState(16)>
  outData: <captureFlag(16)> <JStateLow(32)> <JStateHigh(32)>
*/

HAL_FUNCTION(_hal_PollJStateReg430I)
{
    int16_t RetState = -1;
    uint16_t forceSendState = 0;
    uint16_t lOut = 0;

    STREAM_get_word(&forceSendState);

    if(forceSendState)
    {
        lOut = cntrl_sig_capture();

        //To check if the device woke up in the meantime
        //and prevent an unwanted reset/BSL entry sequence
        if (lOut != JTAGVERSION)
        {
            IHIL_Open(RSTHIGH);
            IHIL_TapReset();
            IHIL_CheckJtagFuse();
            lOut = cntrl_sig_capture();
        }

        if(lOut == JTAGVERSION)
        {   // Active
            STREAM_put_word(JSTATE_CAPTURE_FLAG);
            STREAM_put_long(0x00000000);
            STREAM_put_long(0x00000000);
        }
        else
        {   // LPMx.5
            STREAM_put_word(JSTATE_CAPTURE_FLAG);
            STREAM_put_long(0x00000000);
            STREAM_put_long(0xC0000000);
        }
        RetState = 0;
    }
    return RetState;
}
/**
* \ingroup MODULMACROS
*
* \file PollJStateRegFR57xx.c
*
* \brief <FILEBRIEF>
*/


#define ACTIVE              1
#define LPM5_MODE           2

int16_t intstate = 0;

uint8_t getSystemState()
{
    volatile uint8_t TDO_Value = 0;
    uint8_t state = ACTIVE;

    IHIL_Delay_1ms(5);
    if (jtagIdIsXv2(cntrl_sig_capture()))
    {
        intstate = 0; // Active mode
    }
    else
    {
        intstate++;
        state = LPM5_MODE;
    }

    if (intstate > 1) // device is in LPMx. now
    {
        intstate = 0;
        IHIL_EntrySequences(RSTHIGH);
        IHIL_TapReset();

        TDO_Value = cntrl_sig_capture();
        if ( jtagIdIsXv2(TDO_Value) && (jmb_exchange() == TDO_Value)) // if device wakeup in between check content of jtag mailbox
        {
            state = ACTIVE;
            // check if JTAG mailbox is ready & perform input request
            if(SetReg_16Bits(0x0004) == 0x1207) // OUTRDY  | INRDY JMB_OUTREQ
            {
                if (SetReg_16Bits(0x0000) == 0xA55A)
                {
                    StreamSafe stream_tmp;

                    //Setup values for watchdog control regsiters
                    uint8_t Dummy[8] = {(uint8_t)(wdtctlAddress5xx & 0xFF), (uint8_t)((wdtctlAddress5xx >> 8) & 0xFF),
                                            WDTHOLD_DEF|WDTSSEL_ACLK,WDTPW_DEF,0,0,0,0};

                    STREAM_internal_stream(Dummy, sizeof(Dummy), 0, 0, &stream_tmp);
                    (this->*HAL_SyncJtag_Conditional_SaveContextXv2)(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG);
                    STREAM_external_stream(&stream_tmp);
                }
                else
                {
                    TDO_Value = 0xFF;
                    state = LPM5_MODE;
                }
            }
        }
    }
    return state;
 }

uint16_t LPMx5_DEVICE_STATE  = ACTIVE;       // Assume device starts in Wake-up mode
/**
  PollJStateReg
  Queries the JSTATE register and reports any changes in the relevant bits
  inData:  <maskLow(32)> <maskHigh(32)> <forceSendState(16)>
  outData: <captureFlag(16)> <JStateLow(32)> <JStateHigh(32)>
*/
HAL_FUNCTION(_hal_PollJStateRegFR57xx)
{
    int16_t RetState = HALERR_UNDEFINED_ERROR;

    volatile uint8_t state = LPM5_MODE;

    uint16_t oldLPMx5_DEVICE_STATE = LPMx5_DEVICE_STATE;   // Initialize to not transition

    uint16_t forceSendState = 0;
    STREAM_get_word(&forceSendState);
    state = getSystemState();

    // this is for querry LPMx.5 just one time
    if(forceSendState)
    {
        if (state == ACTIVE)
        {   // Active
            STREAM_put_word(JSTATE_CAPTURE_FLAG);
            STREAM_put_long(0x00000000);
            STREAM_put_long(0x00000000);
        }
        else
        {   // LPMx.5
            STREAM_put_word(JSTATE_CAPTURE_FLAG);
            STREAM_put_long(0x00000000);
            STREAM_put_long(0x40000000);
        }
        RetState = 0;
    }
    else
    {
        LPMx5_DEVICE_STATE = state;

        if(oldLPMx5_DEVICE_STATE != LPMx5_DEVICE_STATE)
        {
            if( LPMx5_DEVICE_STATE == ACTIVE )
            {

                STREAM_put_word(JSTATE_CAPTURE_FLAG);
                STREAM_put_long(0x00000000);
                STREAM_put_long(0x00000000);
                RetState = 1;
            }
            else
            {
                STREAM_put_word(JSTATE_CAPTURE_FLAG);
                STREAM_put_long(0x00000000);
                STREAM_put_long(0x40000000);
                RetState = 1;
            }
        }
    }
    return RetState;
}
/**
* \ingroup MODULMACROS
*
* \file Psa.c
*
* \brief Calculate CRC16 checksum over the specified memory area using
* the integrated PSA hardware
*/


/**
  Psa
  Calculate CRC16 checksum over the specified memory area using
  the integrated PSA hardware.
  inData:  <addr(32)> <length(32)> <type(8)>
  outData: <psaCrc(16)>
  addr:    start address of CRC calculation (must be even)
  length:  number of words (16bit) to be verified
  type:    specifies the type of verification, !!not required for Xv2!!
           (0 = regular, 1 = enhanced, retrieved from device DB)
  psaCrc:  CRC-16 value
*/


//extern uint16_t enhancedPsa;

HAL_FUNCTION(_hal_Psa)
{
    //Setup values for watchdog control regsiters
    uint8_t DummyIn[4] = {WDTCTL_ADDRESS & 0xFF, (WDTCTL_ADDRESS >> 8) & 0xFF,
                                        WDTHOLD_DEF, WDTPW_DEF};
    uint32_t addr;
    uint32_t length;
    StreamSafe stream_tmp;
    int16_t ret_value = 0;

#if defined(eZ_FET) || defined(MSP_FET)
    // MSPFETSim: not sure what this craziness is, but its result is calling `_hil_getEdtDistinct`
//    HilInitGetEdtDistinctFunc hilEdtDis = (HilInitGetEdtDistinctFunc)0x1880;
//    hilEdtDis(&_edt_Distinct_Methods_HAL);
    _hil_getEdtDistinct(&_edt_Distinct_Methods_HAL);
#endif

    if(STREAM_get_long(&addr) != 0)
    {
        return HALERR_PSA_NO_ADDRESS;
    }
    if(STREAM_get_long(&length) < 0)
    {
        return HALERR_PSA_NO_SIZE;
    }

    STREAM_internal_stream(DummyIn, sizeof(DummyIn), NULL, 0, &stream_tmp);
    ret_value = (this->*HAL_SyncJtag_AssertPor_SaveContext)(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG);
    STREAM_external_stream(&stream_tmp);
    if(ret_value < 0)
    {
        return ret_value;
    }

    instrLoad();

    if(enhancedPsa)
    {
        SetPc(addr - 4);
        JTAG_EnhancedPsaSetup(addr);
    }
    else
    {
        JTAG_PsaSetup(addr);
    }

    data_psa();

    IHIL_StepPsa(length);

    shift_out_psa();
    STREAM_put_word(SetReg_16Bits(0));

    IHIL_Tclk(1);

    if(enhancedPsa)
    {
        JTAG_EnhancedPsaEnd();
    }
    else
    {
        JTAG_PsaEnd();
    }

    STREAM_internal_stream(DummyIn, sizeof(DummyIn), NULL, 0, &stream_tmp);
    ret_value = (this->*HAL_SyncJtag_AssertPor_SaveContext)(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG);
    STREAM_external_stream(&stream_tmp);

    return(ret_value);
}
/**
* \ingroup MODULMACROSX
*
* \file PsaX.c
*
* \brief Calculate CRC16 checksum over the specified memory area using
* the integrated PSA hardware
*/


HAL_FUNCTION(_hal_PsaX)
{
    //Setup values for watchdog control regsiters
    uint8_t DummyIn[4] = {WDTCTL_ADDRESS & 0xFF, (WDTCTL_ADDRESS >> 8) & 0xFF,
                                        WDTHOLD_DEF, WDTPW_DEF};
    uint32_t addr;
    uint32_t length;
    StreamSafe stream_tmp;
    int16_t ret_value = 0;

#if defined(eZ_FET) || defined(MSP_FET)
    // MSPFETSim: not sure what this craziness is, but its result is calling `_hil_getEdtDistinct`
//    HilInitGetEdtDistinctFunc hilEdtDis = (HilInitGetEdtDistinctFunc)0x1880;
//    hilEdtDis(&_edt_Distinct_Methods_HAL);
    _hil_getEdtDistinct(&_edt_Distinct_Methods_HAL);
#endif

    if(STREAM_get_long(&addr) != 0)
    {
        return HALERR_PSA_NO_ADDRESS;
    }
    if(STREAM_get_long(&length) < 0)
    {
        return HALERR_PSA_NO_SIZE;
    }

    STREAM_internal_stream(DummyIn, sizeof(DummyIn), NULL, 0, &stream_tmp);
    ret_value = (this->*HAL_SyncJtag_AssertPor_SaveContextX)(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG);
    STREAM_external_stream(&stream_tmp);
    if(ret_value < 0)
    {
        return ret_value;
    }

    SetPcX(addr - 4);
    JTAG_EnhancedPsaSetup(addr);

    data_psa();

    IHIL_StepPsa(length);

    shift_out_psa();
    STREAM_put_word(SetReg_16Bits(0));

    IHIL_Tclk(1);

    JTAG_EnhancedPsaEnd();

    STREAM_internal_stream(DummyIn, sizeof(DummyIn), NULL, 0, &stream_tmp);
    ret_value = (this->*HAL_SyncJtag_AssertPor_SaveContextX)(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG);
    STREAM_external_stream(&stream_tmp);

    return(ret_value);
}
/**
* \ingroup MODULMACROSXV2
*
* \file PsaXv2.c
*
* \brief Calculate CRC16 checksum over the specified memory area using
* the integrated PSA hardware
*/


//extern uint8_t mclk_modules[32];

/**
  PsaXv2
  Calculate CRC16 checksum over the specified memory area using
  the integrated PSA hardware.
  inData:  <addr(32)> <length(32)> <type(8)>
  outData: <psaCrc(16)>
  addr:    start address of CRC calculation (must be even)
  length:  number of words (16bit) to be verified
  type:    specifies the type of verification, !!not required for Xv2!!
           (0 = regular, 1 = enhanced, retrieved from device DB)
  psaCrc:  CRC-16 value
*/
HAL_FUNCTION(_hal_PsaXv2)
{
    //Setup values for watchdog control regsiters
    uint8_t DummyIn[37] = {(uint8_t)(wdtctlAddress5xx & 0xFF), (uint8_t)((wdtctlAddress5xx >> 8) & 0xFF),
                                        WDTHOLD_DEF, WDTPW_DEF};
    uint32_t addr;
    uint32_t length;
    int16_t i;
    StreamSafe stream_tmp;

#if defined(eZ_FET) || defined(MSP_FET)
    // MSPFETSim: not sure what this craziness is, but its result is calling `_hil_getEdtDistinct`
//    HilInitGetEdtDistinctFunc hilEdtDis = (HilInitGetEdtDistinctFunc)0x1880;
//    hilEdtDis(&_edt_Distinct_Methods_HAL);
    _hil_getEdtDistinct(&_edt_Distinct_Methods_HAL);
#endif

    STREAM_get_long(&addr);
    STREAM_get_long(&length);

    for(i=0;i<32;i++)
    {
        DummyIn[5+i]=mclk_modules[i];
    }
    STREAM_internal_stream(DummyIn, sizeof(DummyIn), NULL, 0, &stream_tmp);
    (this->*HAL_SyncJtag_AssertPor_SaveContextXv2)(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG);
    STREAM_external_stream(&stream_tmp);

    {
        uint16_t Mova;
        uint16_t Pc_l;

        Mova  = 0x0080;
        Mova += (uint16_t)(((addr)>>8) & 0x00000F00);
        Pc_l  = (uint16_t)(((addr) & 0xFFFF));
        /**
          * Start of macro: i_SetPc
          */
        SetPcXv2(Mova, Pc_l);
        IHIL_Tclk(1);
        cntrl_sig_16bit();
        SetReg_16Bits(0x0501);
        /**
        * End of macro: i_SetPc
        */
    }

    data_16bit();
    SetReg_16Bits((uint16_t)(addr - 2));
    data_psa();

    IHIL_StepPsa(length);

    shift_out_psa();
    STREAM_put_word(SetReg_16Bits(0));

    for(i=0;i<32;i++)
    {
        DummyIn[5+i] = mclk_modules[i];
    }
    STREAM_internal_stream(DummyIn, sizeof(DummyIn), NULL, 0, &stream_tmp);
    (this->*HAL_SyncJtag_AssertPor_SaveContextXv2)(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG);
    STREAM_external_stream(&stream_tmp);

    return 0;
}
/**
* \ingroup MODULMACROS
*
* \file ReadAllCpuRegs.c
*
* \brief Read CPU register values, except R0 and R2
*/

#ifdef DB_PRINT
#    include "debug.h"
#endif

/**
 * ReadAllCpuRegs
 * Read CPU register values, except R0 and R2. This function is for 16bit CPUs.
 * inData:  -
 * outData: <SP(32)> <Rn(32)>{12}
 * SP: stack pointer register (R1)
 * Rn: registers R4-R15
 */
HAL_FUNCTION(_hal_ReadAllCpuRegs)
{
    uint8_t Registers;
    for (Registers = 1; Registers < 16; Registers++)
    {
        if(Registers == 2)
        {
          Registers += 2;
        }
        STREAM_put_word(ReadCpuReg(Registers));
    }
    return 0;

}
/*
 * \ingroup MODULMACROSMSP432
 *
 * \file ReadAllCpuRegsArm.c
 *
 * \brief Read CPU register values, except R0 and R2
 */


/**
  ReadAllCpuRegsArm
  Read CPU register values.
  inData:  -
  outData: R0-R12, SP, LR, PC, SR, XPRS,SpecialRegs, MSP_SP, PSP_SP
*/
//#if defined(MSP430_UIF) || defined(MSP_FET)
//extern ARMConfigSettings armConfigSettings;
//#endif

int16_t ReadCpurRegArm(uint32_t *data, uint32_t Rx)
{
    uint8_t retry = 0;
    IHIL_Write_Read_Mem_Ap(0, DCRSR, &Rx, WRITE); // Request Read

    retry = MAX_RETRY;

    // check if wirte was sucessfully
    uint32_t returnVal = 0;
    do
    {
        IHIL_Write_Read_Mem_Ap(0, DHCSR, &returnVal, READ);
    } while(--retry && !(returnVal & S_REGRDY));

    IHIL_Write_Read_Mem_Ap(0, DCRDR, data, READ);  // Read value

    if(!retry)
    {
        return 0;
    }
    return 1;
}


HAL_FUNCTION(_hal_ReadAllCpuRegsArm)
{
#if defined(MSP430_UIF) || defined(MSP_FET)
    uint32_t data = 0;
    // Read the General Purpose registers.
    for (uint16_t Rx = 0; Rx < 16; ++Rx)
    {
        if(!ReadCpurRegArm(&data, Rx))
        {
            return HALERR_UNDEFINED_ERROR;
        }
        STREAM_put_long(data);
    }

    if(!ReadCpurRegArm(&data, xPSR_REG))
    {
        return HALERR_UNDEFINED_ERROR;
    }
    STREAM_put_long(data);

    if(!ReadCpurRegArm(&data, spec_REG))
    {
        return HALERR_UNDEFINED_ERROR;
    }
    STREAM_put_long(data);

    if(!ReadCpurRegArm(&data, MSP_SP))
    {
        return HALERR_UNDEFINED_ERROR;
    }
    STREAM_put_long(data);

    if(!ReadCpurRegArm(&data, PSP_SP))
    {
        return HALERR_UNDEFINED_ERROR;
    }
    STREAM_put_long(data);
#endif
    return 0;
}
/**
* \ingroup MODULMACROSXV2
*
* \file ReadAllCpuRegsNon1377Xv2.c
*
* \brief Read CPU register values, except R0 and R2
*/


///**
//  ReadAllCpuRegsXv2
//  Read CPU register values, except R0 and R2. This function is for 16bit CPUs.
//  inData:  -
//  outData: <SP(24)> <Rn(24)>{12}
//  SP: stack pointer register (R1)
//  Rn: registers R4-R15
//*/
//
////extern uint16_t romAddressForCpuRead;
//
//HAL_FUNCTION(_hal_ReadAllCpuRegsNon1377Xv2)
//{
//  decl_out
//  uint16_t Registers;
//  uint16_t Mova;
//  uint32_t Rx;
//
//  // Read the General Purpose registers.
//  for (Registers = 1; Registers < 16; Registers++) // Read registers SP, and R4 through R15.
//  {
//    if(Registers == 2)
//    {
//      Registers += 2;
//    }
//    Mova  = 0x0060;
//    Mova += (Registers << 8) & 0x0F00;
//    ReadCpuRegXv2(Mova, Rx);
//    STREAM_put_bytes((uint8_t*)&Rx, 3);
//  }
//
//  // all CPU register values have been moved to the JMBOUT register address
//  // -> JMB needs to be cleared again!!
//  i_ReadJmbOut(Rx)
//  return 0;
//}

/**
* \ingroup MODULMACROSX
*
* \file ReadAllCpuRegsX.c
*
* \brief Read CPU register values, except R0 and R2
*/


HAL_FUNCTION(_hal_ReadAllCpuRegsX)
{
    uint8_t Registers;
    uint32_t tmp = 0;

    for (Registers = 1; Registers < 16; Registers++)
    {
        if(Registers == 2)
        {
          Registers += 2;
        }
        tmp = ReadCpuRegX(Registers);
        STREAM_put_bytes((uint8_t*)&tmp, 3);
    }
    return 0;
}

/**
* \ingroup MODULMACROSXV2
*
* \file ReadAllCpuRegsXv2.c
*
* \brief Read CPU register values, except R0 and R2
*/


//extern uint16_t altRomAddressForCpuRead;

/**
ReadAllCpuRegsXv2
Read CPU register values, except R0 and R2. This function is for 16bit CPUs with note 1377.
inData:  -
outData: <SP(24)> <Rn(24)>{12}
SP: stack pointer register (R1)
Rn: registers R4-R15
*/

HAL_FUNCTION(_hal_ReadAllCpuRegsXv2)
{
    uint16_t Registers;
    uint16_t Mova;
    uint32_t Rx;
    uint16_t id = cntrl_sig_capture();

    // Read the General Purpose registers.
    for (Registers = 1; Registers < 16; Registers++) // Read registers SP, and R4 through R15.
    {
        if(Registers == 2)
        {
            Registers += 2;
        }
        Mova  = 0x0060;
        Mova += (Registers << 8) & 0x0F00;
        Rx = ReadCpuRegXv2(Mova);
        STREAM_put_bytes((uint8_t*)&Rx, 3);

        if(id == JTAGVERSION91 || id == JTAGVERSION99 || id == JTAGVERSION98)
        {
            // Set PC to "safe" address
            SetPcXv2(0x0080, SAFE_PC_ADDRESS);
            cntrl_sig_16bit();
            SetReg_16Bits(0x0501);
            IHIL_Tclk(1);
            addr_capture();
        }
    }
    // all CPU register values have been moved to the JMBOUT register address
    // -> JMB needs to be cleared again!!
    Rx = i_ReadJmbOut();
    return 0;
}

/**
* \ingroup MODULMACROS
*
* \file ReadMemBytes.c
*
* \brief Read bytes from a memory mapped location
*/


/**
  ReadMemBytes
  Read bytes from a memory mapped location.
  inData:  <addr(32)> <length(32)>
  outData: <data(8)>{*}
  addr: the address to start reading from
  length: number of bytes to read
  data: requested data
*/

HAL_FUNCTION(_hal_ReadMemBytes)
{
    int16_t ret_value = 0;
    uint32_t i;
    uint32_t lAddr;
    uint32_t lLen;

    if(STREAM_get_long(&lAddr) != 0)
    {
        ret_value = HALERR_READ_MEM_BYTES_NO_ADDRESS;
        goto exit;
    }
    if(STREAM_get_long(&lLen) < 0)
    {
        ret_value = HALERR_READ_MEM_BYTES_NO_SIZE;
        goto exit;
    }
    halt_cpu();
    IHIL_Tclk(0);
    cntrl_sig_16bit();
    SetReg_16Bits(0x2419);

    lLen *= 2; //DLL always sends size in word
    for(i = 0; i < lLen; i++)
    {
        addr_16bit();
        SetReg_16Bits((uint16_t)lAddr);
        data_to_addr();
        IHIL_Tclk(1);
        IHIL_Tclk(0);
        STREAM_put_byte((uint8_t)(SetReg_16Bits(0) & 0xFF));
        lAddr += 1;
    }
    release_cpu();
exit:
    return(ret_value);
}

/**
* \ingroup MODULMACROSX
*
* \file ReadMemBytesX.c
*
* \brief Read bytes from a memory mapped location
*/

HAL_FUNCTION(_hal_ReadMemBytesX)
{
    uint32_t i;
    uint32_t lAddr;
    uint32_t lLen;

    if(STREAM_get_long(&lAddr) != 0)
    {
        return HALERR_READ_MEM_WORD_NO_ADDRESS;
    }
    if(STREAM_get_long(&lLen) < 0)
    {
        return HALERR_READ_MEM_WORD_NO_SIZE;
    }

    halt_cpu();
    IHIL_Tclk(0);
    cntrl_sig_low_byte();
    SetReg_16Bits(0x19);

    lLen *= 2; //DLL always sends size in word
    for(i = 0; i < lLen; i++)
    {
        addr_16bit();
        SetReg_20Bits(lAddr);
        data_to_addr();
        IHIL_Tclk(1);
        IHIL_Tclk(0);
        STREAM_put_byte((uint8_t)(SetReg_16Bits(0) & 0xFF));
        lAddr += 1;
    }
    IHIL_Tclk(1);

    release_cpu();
    return 0;
}

/**
* \ingroup MODULMACROSXV2
*
* \file ReadMemBytesXv2.c
*
* \brief Read bytes from a memory mapped location
*/

/**
  ReadMemBytesXv2
  Read bytes from a memory mapped location.
  inData:  <addr(32)> <length(32)>
  outData: <data(8)>{*}
  addr: the address to start reading from
  length: number of bytes to read
  data: requested data
*/

HAL_FUNCTION(_hal_ReadMemBytesXv2)
{
  //! \todo not supported for Xv2
  return HALERR_UNDEFINED_ERROR;
}

/**
* \ingroup <FILEGROUP>
*
* \file ReadMemQuick.c
*
* \brief Read words (16bit values) from a memory mapped location in data quick mode
*/

/**
  ReadMemQuick
  Read words (16bit values) from a memory mapped location in data quick mode.
  inData:  <addr(32)> <length(32)>
  outData: <data(16)>{*}
  addr: the address to start reading from
  length: number of words to read
  data: requested data
*/


HAL_FUNCTION(_hal_ReadMemQuick)
{
    uint32_t i;
    uint32_t lAddr;
    uint32_t lLen;
    uint16_t Pc;

    STREAM_get_long(&lAddr);
    STREAM_get_long(&lLen);

    Pc  = (uint16_t)(((lAddr-2) & 0xFFFF));

    instrLoad();

    SetPc(Pc);

    halt_cpu();

    data_quick();

    for (i = 0; i < lLen; i++)
    {
        IHIL_Tclk(1);
        IHIL_Tclk(0);
        STREAM_put_word(SetReg_16Bits(0));
    }
    IHIL_Tclk(1);

    release_cpu();

    return 0;
}
/**
* \ingroup MODULMACROSX
*
* \file ReadMemQuickX.c
*
* \brief Read words (16bit values) from a memory mapped location in data quick mode
*/


HAL_FUNCTION(_hal_ReadMemQuickX)
{
    uint32_t i;
    uint32_t lAddr;
    uint32_t lLen;

    STREAM_get_long(&lAddr);
    STREAM_get_long(&lLen);

    instrLoad();

    SetPcX(lAddr - 2);

    halt_cpu();

    data_quick();

    for (i = 0; i < lLen; i++)
    {
        IHIL_Tclk(1);
        IHIL_Tclk(0);
        STREAM_put_word(SetReg_16Bits(0));
    }
    IHIL_Tclk(1);

    release_cpu();

    return 0;
}
/**
* \ingroup MODULMACROSXV2
*
* \file ReadMemQuickXv2.c
*
* \brief Read words (16bit values) from a memory mapped location in data quick mode
*/


/**
  ReadMemQuickXv2
  Read words (16bit values) from a memory mapped location in data quick mode.
  inData:  <addr(32)> <length(32)>
  outData: <data(16)>{*}
  addr: the address to start reading from
  length: number of words to read
  data: requested data
*/

HAL_FUNCTION(_hal_ReadMemQuickXv2)
{
    uint32_t i;
    uint32_t lAddr;
    uint32_t lLen;
    uint32_t lPc;
    uint16_t Mova;
    uint16_t Pc_l;
    uint16_t id = cntrl_sig_capture();

    STREAM_get_long(&lAddr);
    STREAM_get_long(&lLen);
    STREAM_get_long(&lPc);
    if(id == JTAGVERSION91 || id == JTAGVERSION99 || id == JTAGVERSION98)
    { // Set PC to "safe" address
        lPc = SAFE_PC_ADDRESS;
    }

    Mova  = 0x0080;
    Mova += (uint16_t)((lAddr>>8) & 0x00000F00);
    Pc_l  = (uint16_t)((lAddr & 0xFFFF));

    // SET PROGRAM COUNTER for QUICK ACCESS
    SetPcXv2(Mova, Pc_l);
    cntrl_sig_16bit();
    SetReg_16Bits(0x0501);
    IHIL_Tclk(1);
    addr_capture();
    // END OF SETTING THE PROGRAM COUNTER
    data_quick();

    // DATA QUICK LOOP
    for (i = 0; i < lLen; i++)
    {
        IHIL_Tclk(1);
        IHIL_Tclk(0);
        STREAM_put_word( SetReg_16Bits(0));
    }
    // Check save State
    cntrl_sig_capture();
    SetReg_16Bits(0x0000);

    // Restore PC
    Mova  = 0x0080;
    Mova += (uint16_t)((lPc>>8) & 0x00000F00);
    Pc_l  = (uint16_t)((lPc & 0xFFFF));

    // SET PROGRAM COUNTER for Backup
    SetPcXv2(Mova, Pc_l);
    cntrl_sig_16bit();
    SetReg_16Bits(0x0501);
    IHIL_Tclk(1);
    addr_capture();

    return 0;
}
/**
* \ingroup MODULMACROS
*
* \file ReadMemWords.c
*
* \brief Read words (16bit values) from a memory mapped location
*/


//    extern uint32_t lAddr;
//    extern uint32_t lLen;

/**
  ReadMemWords
  Read words (16bit values) from a memory mapped location.
  inData:  <addr(32)> <length(32)>
  outData: <data(16)>{*}
  addr: the address to start reading from
  length: number of words to read
  data: requested data
*/

HAL_FUNCTION(_hal_ReadMemWords)
{
    uint32_t i;
    uint32_t lAddr;
    uint32_t lLen;

    if(STREAM_get_long(&lAddr) != 0)
    {
        return HALERR_READ_MEM_WORD_NO_ADDRESS;
    }
    if(STREAM_get_long(&lLen) < 0)
    {
        return HALERR_READ_MEM_WORD_NO_SIZE;
    }

    halt_cpu();
    IHIL_Tclk(0);
    for(i = 0; i < lLen; i++)
    {
        addr_16bit();
        SetReg_16Bits((uint16_t)lAddr);
        data_to_addr();
        IHIL_Tclk(1);
        IHIL_Tclk(0);
        STREAM_put_word(SetReg_16Bits(0));
        lAddr += 2;
    }
    IHIL_Tclk(1);
    release_cpu();

    instrLoad();

    return 0;
}
/**
* \ingroup MODULMACROSX
*
* \file ReadMemWordsX.c
*
* \brief Read words (16bit values) from a memory mapped location
*/


HAL_FUNCTION(_hal_ReadMemWordsX)
{
    uint32_t i;
    uint32_t lAddr;
    uint32_t lLen;

    if(STREAM_get_long(&lAddr) != 0)
    {
        return HALERR_READ_MEM_WORD_NO_ADDRESS;
    }
    if(STREAM_get_long(&lLen) < 0)
    {
        return HALERR_READ_MEM_WORD_NO_SIZE;
    }

    halt_cpu();
    IHIL_Tclk(0);

    for(i = 0; i < lLen; i++)
    {
        addr_16bit();
        SetReg_20Bits(lAddr);
        data_to_addr();
        IHIL_Tclk(1);
        IHIL_Tclk(0);
        STREAM_put_word(SetReg_16Bits(0));
        lAddr += 2;
    }
    IHIL_Tclk(1);

    release_cpu();
    return 0;
}

/**
* \ingroup MODULMACROSXV2
*
* \file ReadMemWordsXv2.c
*
* \brief Read words (16bit values) from a memory mapped location
*/


/**
  ReadMemWordsXv2
  Read words (16bit values) from a memory mapped location.
  inData:  <addr(32)> <length(32)>
  outData: <data(16)>{*}
  addr: the address to start reading from
  length: number of words to read
  data: requested data
*/

HAL_FUNCTION(_hal_ReadMemWordsXv2)
{
  uint32_t i;
  uint32_t lAddr;
  uint32_t lLen;

  STREAM_get_long(&lAddr);
  STREAM_get_long(&lLen);

  IHIL_Tclk(0);

  for(i = 0; i < lLen; i++)
  {
    addr_16bit();
    SetReg_20Bits(lAddr);
    IHIL_Tclk(1);
    IHIL_Tclk(0);
    data_capture();
    STREAM_put_word(SetReg_16Bits(0));
    lAddr += 2;
  }
  IHIL_Tclk(1);
  // one or more cycle, so CPU is driving correct MAB
  IHIL_TCLK();

  return 0;
}

/**
* \file Reset430I.c
*
* \brief Write words (16bit values) to a memory mapped location
*/

//! \ingroup MODULMACROS
//! \file Reset430I.c
//! \brief


/**
   _hal_Reset430I
   this macro will apply a BSL sequence to reset the
   device and keep it in LPM4 afterwards

*/

HAL_FUNCTION(_hal_Reset430I)
{
        volatile uint16_t id=0;

        /* Apply BSL Entry Sequence to Stop device execution */
        IHIL_BSL_EntrySequence(1);
        /* Now the device should be in LPM4 */

        IHIL_Delay_1ms(500);

        IHIL_Open(RSTHIGH);
        IHIL_TapReset();
        IHIL_CheckJtagFuse();

        id = cntrl_sig_capture();
        if (id == JTAGVERSION)
        {
            STREAM_put_byte((uint8_t)0x1);
            STREAM_put_byte((uint8_t)0x89);
            STREAM_put_byte((uint8_t)SPYBIWIREJTAG);
            return 0;
        }
        return -1;
}
/*
 * \ingroup MODULMACROS
 *
 * \file ResetArm.c
 *
 * \brief <FILEBRIEF>
 */


/**
  Halt: Resets and Halts the Core
*/
//#if defined(MSP430_UIF) || defined(MSP_FET)
//extern ARMConfigSettings armConfigSettings;
//#endif

HAL_FUNCTION(_hal_ResetArm)
{
#if defined(MSP430_UIF) || defined(MSP_FET)
    uint16_t resetType = 0;

    if(STREAM_get_word(&resetType) == -1)
    {
        return HALERR_UNDEFINED_ERROR;
    }
    uint32_t data = VECTKEY | (resetType ? SYSRESETREQ : VECTRESET);
    if(IHIL_Write_Read_Mem_Ap(0, AIRCR, &data, WRITE) == -1)
    {
        return HALERR_UNDEFINED_ERROR;
    }
    if(IHIL_Write_Read_Mem_Ap(0, DHCSR, &data, READ) == -1)
    {
        return HALERR_UNDEFINED_ERROR;
    }
    if(resetType == 0)
    {
        if(((data & 0xFF000000) >> 24) != 0x2)
        {
            return HALERR_UNDEFINED_ERROR;
        }
    }

#endif
    return 0;
}
/**
* \ingroup MODULMACROSXV2
*
* \file ResetXv2.c
*
* \brief Apply an RST NMI to a CPUxv2 device
*/

#define L092 1
#define GENERIC_XV2 0


/**
   _hal_ResetXv2
   this macro will applay an RST NMI to a CPUxv2 device.
   after the reset the device execution will be stopped
   by using the JTAG mailbox

*/
HAL_FUNCTION(_hal_ResetXv2)
{
    // put in magic pattern to stop user code execution
    IHIL_EntrySequences(RSTLOW);
    IHIL_TapReset();

    if(i_WriteJmbIn(MAGIC_PATTERN) == 1)
    {
        return HALERR_UNDEFINED_ERROR;
    }
    IHIL_Delay_1ms(40);

    IHIL_SetReset(1);
    if (jtagIdIsXv2(cntrl_sig_capture()))
    {
        return 0;
    }
    return HALERR_UNDEFINED_ERROR;
}

HAL_FUNCTION(_hal_Reset5438Xv2)
{
    IHIL_SetReset(0);
    IHIL_Delay_1ms(40);

    IHIL_SetTest(0);
    IHIL_Delay_1ms(40);

    IHIL_EntrySequences(RSTHIGH);
    IHIL_TapReset();

    if (jtagIdIsXv2(cntrl_sig_capture()))
    {
        return 0;
    }
    return HALERR_UNDEFINED_ERROR;
}

HAL_FUNCTION(_hal_ResetL092)
{
    uint8_t jtagMailboxIn = 0;
    uint32_t activationKey = 0;
    if(STREAM_get_long(&activationKey) < 0)
    {
        return(HALERR_START_JTAG_NO_ACTIVATION_CODE);
    }
    if(activationKey == L092_MODE || activationKey == C092_MODE)
    {
        // L092 has no JTAG chain support
        IHIL_SetReset(0);
        IHIL_Open(RSTLOW);
        IHIL_TapReset();
        IHIL_SetReset(0);

        jtagMailboxIn = i_WriteJmbIn32(activationKey>>16 , activationKey);
        // check for Timeout issue during mailbox request
        IHIL_SetReset(1);
        IHIL_Delay_1ms(3000);
        if(jtagMailboxIn == 1)
        {
            return (HALERR_JTAG_PASSWORD_WRONG);
        }
        if (jtagIdIsXv2(cntrl_sig_capture()))
        {
            return 0;
        }
    }
    return HALERR_UNDEFINED_ERROR;
}
/**
* \ingroup MODULMACROS
*
* \file RestoreContext_ReleaseJtag.c
*
* \brief Restore the CPU context and releast Jtag control
*/


//extern DeviceSettings deviceSettings;

/**
  RestoreContext_ReleaseJtag
  Restore the CPU context and releast Jtag control.
  inData:  <wdtAddr(16)> <wdtCtrl(16)> <PC(32)> <SR(16)><eemCtrl(16)> <mdb(16)>
  outData: -
  wdtAddr: watchdog timer address
  wdtCtrl: watchdog timer control value
  PC: program counter register (R0)
  SR: status register (R2)
  eemCtrl: EEM control bits (EEM_EN and optional EEM_CLK_EN)
  mdb: value to be put on the Memory Data Bus before release (in case !0)
*/
HAL_FUNCTION(_hal_RestoreContext_ReleaseJtag)
{
    uint16_t wdt_addr;
    uint16_t wdt_value;
    uint16_t pc[2];
    uint16_t sr;
    uint16_t control_mask;
    uint16_t mdb;
    uint16_t releaseJtag;
    uint16_t* syncWithRunVarAddress = 0;

    // Check all the input parameters
    if(STREAM_get_word(&wdt_addr) != 0)
    {
      return (HALERR_RESTORECONTEXT_RELEASE_JTAG_NO_WDT_ADDRESS);
    }
    if(STREAM_get_word(&wdt_value) != 0)
    {
      return (HALERR_RESTORECONTEXT_RELEASE_JTAG_NO_WDT_VALUE);
    }
    if(STREAM_get_long((uint32_t*)&pc) != 0)
    {
      return (HALERR_RESTORECONTEXT_RELEASE_JTAG_NO_PC);
    }
    if(STREAM_get_word(&sr) != 0)
    {
      return (HALERR_RESTORECONTEXT_RELEASE_JTAG_NO_SR);
    }
    if(STREAM_get_word(&control_mask) != 0)
    {
      return (HALERR_RESTORECONTEXT_RELEASE_JTAG_NO_CONTROL_MASK);
    }
    if(STREAM_get_word(&mdb) != 0)
    {
        return (HALERR_RESTORECONTEXT_RELEASE_JTAG_NO_MDB);
    }
    if (STREAM_get_word(&releaseJtag) == -1)
    {
        releaseJtag = 0;
    }
    STREAM_discard_bytes(2);


    // Write back Status Register
    WriteCpuReg(2, sr);

    // Restore Watchdog Control Register
    WriteMemWord(wdt_addr, wdt_value);

    // restore Program Counter
    SetPc(pc[0]); // High part is not relevant for MSP430 original architecture

/* Workaround for MSP430F149 derivatives */
    {
      uint16_t backup[3] = {0};
      eem_data_exchange();
      SetReg_16Bits(0x03);  backup[0] = SetReg_16Bits(0);
      SetReg_16Bits(0x0B);  backup[1] = SetReg_16Bits(0);
      SetReg_16Bits(0x13);  backup[2] = SetReg_16Bits(0);

      SetReg_16Bits(0x02);  SetReg_16Bits(0);
      SetReg_16Bits(0x0A);  SetReg_16Bits(0);
      SetReg_16Bits(0x12);  SetReg_16Bits(0);

      SetReg_16Bits(0x02);  SetReg_16Bits(backup[0]);
      SetReg_16Bits(0x0A);  SetReg_16Bits(backup[1]);
      SetReg_16Bits(0x12);  SetReg_16Bits(backup[2]);
    }
/**/

    if (deviceSettings.clockControlType != GCC_NONE)
    {
        if(deviceSettings.stopFLL)
        {
            uint16_t clkCntrl = 0;
            // read access to EEM General Clock Control Register (GCLKCTRL)
            eem_data_exchange();
            SetReg_16Bits(MX_GCLKCTRL + MX_READ);
            clkCntrl = SetReg_16Bits(0);

            // added UPSF: FE427 does regulate the FLL to the upper boarder
            // added the switch off and release of FLL (JTFLLO)
            clkCntrl &= ~0x10;
            eem_data_exchange();
            SetReg_16Bits(MX_GCLKCTRL + MX_WRITE);
            SetReg_16Bits(clkCntrl);
        }
    }

    if (deviceSettings.clockControlType == GCC_EXTENDED)
    {
        eem_data_exchange();
        SetReg_16Bits(MX_GENCNTRL + MX_WRITE);                              // write access to EEM General Control Register (MX_GENCNTRL)
        SetReg_16Bits(EMU_FEAT_EN | EMU_CLK_EN | CLEAR_STOP | EEM_EN);      // write into MX_GENCNTRL
    }
    if (deviceSettings.clockControlType == GCC_STANDARD_I)
    {
        eem_data_exchange();
        SetReg_16Bits(MX_GENCNTRL + MX_WRITE);  // write access to EEM General Control Register (MX_GENCNTRL)
        SetReg_16Bits(EMU_FEAT_EN);             // write into MX_GENCNTRL
    }

    // activate EEM
    eem_write_control();
    SetReg_16Bits(control_mask);

    // Pre-initialize MDB before release if
    if(mdb)
    {
        data_16bit();
        SetReg_16Bits(mdb);
        IHIL_Tclk(0);
        addr_capture();
        IHIL_Tclk(1);
    }
    else
    {
        addr_capture();
    }

    syncWithRunVarAddress = getTargetRunningVar();
    if(syncWithRunVarAddress)
    {
          *syncWithRunVarAddress = 0x0001;
    }

    // release target device from JTAG control
    cntrl_sig_release();

    if(releaseJtag)
    {
        IHIL_Close(); // release JTAG on go
    }
    return (0);
}
/**
* \ingroup MODULMACROSX
*
* \file RestoreContext_ReleaseJtagX.c
*
* \brief Restore the CPU context and releast Jtag control
*/


//extern DeviceSettings deviceSettings;


HAL_FUNCTION(_hal_RestoreContext_ReleaseJtagX)
{
    uint16_t wdt_addr;
    uint16_t wdt_value;
    uint32_t  pc;
    uint16_t sr;
    uint16_t control_mask;
    uint16_t mdb;
    uint16_t releaseJtag;
    uint16_t* syncWithRunVarAddress = 0;

    if(STREAM_get_word(&wdt_addr) != 0)
    {
      return HALERR_RESTORECONTEXT_RELEASE_JTAG_NO_WDT_ADDRESS;
    }
    if(STREAM_get_word(&wdt_value) != 0)
    {
      return HALERR_RESTORECONTEXT_RELEASE_JTAG_NO_WDT_VALUE;
    }
    if(STREAM_get_long(&pc) != 0)
    {
      return HALERR_RESTORECONTEXT_RELEASE_JTAG_NO_PC;
    }
    if(STREAM_get_word(&sr) != 0)
    {
      return HALERR_RESTORECONTEXT_RELEASE_JTAG_NO_SR;
    }
    if(STREAM_get_word(&control_mask) != 0)
    {
      return HALERR_RESTORECONTEXT_RELEASE_JTAG_NO_CONTROL_MASK;
    }
    if(STREAM_get_word(&mdb) != 0)
    {
      return HALERR_RESTORECONTEXT_RELEASE_JTAG_NO_MDB;
    }
    if (STREAM_get_word(&releaseJtag) == -1)
    {
      releaseJtag = 0;
    }
    STREAM_discard_bytes(2);

    // Write back Status Register
    WriteCpuRegX(2, sr);

    // Restore Watchdog Control Register
    WriteMemWordX(wdt_addr, wdt_value);

    // restore Program Counter
    SetPcX(pc);

    if (deviceSettings.clockControlType == GCC_EXTENDED)
    {
        eem_data_exchange32();
        SetReg_32Bits(MX_GENCNTRL + MX_WRITE);              // write access to EEM General Control Register (MX_GENCNTRL)
        SetReg_32Bits(EMU_FEAT_EN | EMU_CLK_EN | CLEAR_STOP | EEM_EN);
    }

    // activate EEM
    eem_write_control();
    SetReg_16Bits(control_mask);

    // Pre-initialize MDB before release if
    if(mdb)
    {
        data_16bit();
        SetReg_16Bits(mdb);
        IHIL_Tclk(0);
        addr_capture();
        IHIL_Tclk(1);
    }
    else
    {
      addr_capture();
    }

    syncWithRunVarAddress = getTargetRunningVar();
    if(syncWithRunVarAddress)
    {
          *syncWithRunVarAddress = 0x0001;
    }

    // release target device from JTAG control
    cntrl_sig_release();

    if(releaseJtag)
    {
        IHIL_Close(); // release JTAG on go
    }

    return(0);
}
/**
* \ingroup MODULMACROSXV2
*
* \file RestoreContext_ReleaseJtagXv2.c
*
* \brief Restore the CPU context and releast Jtag control
*/


#define CPUOFF              (0x0010u)
#define DBGJTAGON           (0x0080u)

/**
  RestoreContext_ReleaseJtagXv2
  Restore the CPU context and releast Jtag control.
  inData:  <wdtAddr(16)> <wdtCtrl(16)> <PC(32)> <SR(16)><eemCtrl(16)> <mdb(16)>
  outData: -
  wdtAddr: watchdog timer address
  wdtCtrl: watchdog timer control value
  PC: program counter register (R0)
  SR: status register (R2)
  eemCtrl: EEM control bits (EEM_EN and optional EEM_CLK_EN)
  mdb: value to be put on the Memory Data Bus before release (in case !0)
*/

//extern DevicePowerSettings devicePowerSettings;

HAL_FUNCTION(_hal_RestoreContext_ReleaseJtagXv2)
{
    uint16_t wdt_addr;
    uint16_t wdt_value;
    uint16_t Sr;
    uint16_t control_mask = 0;
    uint32_t  lPc;
    uint16_t Mova;
    uint16_t Pc_l;
    uint16_t Mdb;
    uint16_t releaseJtag = 0;
    uint16_t ulpDebug = 0;
    uint16_t* syncWithRunVarAddress = 0;


    STREAM_get_word(&wdt_addr);  // 0
    STREAM_get_word(&wdt_value);  // 2
    STREAM_get_long(&lPc);       // 4-7
    STREAM_get_word(&Sr);       // 8
    STREAM_get_word(&control_mask); // 10
    STREAM_get_word(&Mdb);
    STREAM_get_word(&releaseJtag);
    STREAM_get_word(&ulpDebug);
    // Write back Status Register
    WriteCpuRegXv2(0x0082, Sr);

    // Restore Watchdog Control Register
    WriteMemWordXv2(wdt_addr,wdt_value);

    /* Note 1411 */
    /* check if CPU is OFF, and decrement PC = PC-2 */
    if(Sr & CPUOFF)
    {
        lPc -= 2;
    }
    /* End of note 1411 */

    Mova  = 0x0080;
    Mova += (uint16_t)((lPc>>8) & 0x00000F00);
    Pc_l  = (uint16_t)((lPc & 0xFFFF));

    // restore Program Counter
    SetPcXv2(Mova, Pc_l);

    // prepare release
    if(Mdb)
    {
        cntrl_sig_16bit();
        SetReg_16Bits(0x0401);
        // preload the MDB before releasing the CPU
        data_16bit();
        IHIL_Tclk(1);
        SetReg_16Bits(Mdb);
        IHIL_Tclk(0);
        // release the busses - otherwise modules/flash can't control the MDB
        // = get out of DATA_* instruction
        addr_capture();
        // here one more clock cycle is required to advance the CPU
        // otherwise enabling the EEM can stop the device again
        {
            cntrl_sig_capture();
            if(SetReg_16Bits(0) & CNTRL_SIG_HALT)
            {
                cntrl_sig_16bit();
                SetReg_16Bits(0x0403);
            }
        }
        IHIL_Tclk(1);
    }
    else
    {
        IHIL_Tclk(1);
        cntrl_sig_16bit();
        SetReg_16Bits(0x0401);
        addr_capture();
        // here one more clock cycle is required to advance the CPU
        // otherwise enabling the EEM would stop the device again, because
        {
            cntrl_sig_capture();
            IHIL_Tclk(0);

            // shift out current control signal register value
            if(SetReg_16Bits(0) & CNTRL_SIG_HALT)
            {
                cntrl_sig_16bit();
                SetReg_16Bits(0x0403);
            }
            IHIL_Tclk(1);
        }
    }
    // now we can enable the EEM
    // --> this should work asynchronously
    eem_write_control();
    SetReg_16Bits(control_mask);

    // -------------------------Power mode handling start ----------------------
    if(ulpDebug)
    {
        // Set LPMx.5 settings
        EnableLpmx5();
    }
    else
    {
        uint16_t id = cntrl_sig_capture();
        DisableLpmx5();
        //Manually disable DBGJTAGON bit
        if(id == JTAGVERSION99)
        {
            test_reg_3V();
            SetReg_16Bits(SetReg_16Bits(0x0000) & ~DBGJTAGON);
        }
    }
    // -------------------------Power mode handling end ------------------------

    syncWithRunVarAddress = getTargetRunningVar();
    if(syncWithRunVarAddress)
    {
          *syncWithRunVarAddress = 0x0001;
    }
    // release target device from JTAG control
    cntrl_sig_release();

    if(releaseJtag)
    {
        IHIL_Close(); // release JTAG on go
    }
    return 0;
}
/*
 * \ingroup MODULMACROS
 *
 * \file RunArm.c
 *
 * \brief <FILEBRIEF>
 */


/**
  Run: Runs the Core Debug
*/
#define DISABLE_INTERRUPTS_RUN 0x02
#define DISABLE_INTERRUPTS_UPDATE_PRIMSK 0x80

//#if defined(MSP430_UIF) || defined(MSP_FET)
//extern ARMConfigSettings armConfigSettings;
//extern int16_t writeCpuRegsArm(uint32_t data, uint32_t Rx);
//extern int16_t ReadCpurRegArm(uint32_t *data, uint32_t Rx);
//#endif

int16_t confSystemHighPower()
{
    // Set system and debug power up if cleared
    uint32_t data = 0;
    uint16_t* syncWithRunVarAddress = getTargetRunningVar();

    //force high system power
    if( powerUpArm() != 0)
    {
        return -1;
    }
    // Clear sleep on ISR exit if it was set by user code
    if(IHIL_Write_Read_Mem_Ap(0, SCB_SCR, &data, READ) == -1)
    {
        return -1;
    }
    if(data & SCB_SCR_SLEEPONEXIT_Msk)
    {
        data &= ~SCB_SCR_SLEEPONEXIT_Msk;
        if(IHIL_Write_Read_Mem_Ap(0, SCB_SCR, &data, WRITE) == -1)
        {
            return - 1;
        }
    }
    if(syncWithRunVarAddress)
    {
          *syncWithRunVarAddress = 0x0001;
    }
    // Do the actual run
    data = DBGKEY | C_DEBUGEN;
    if(IHIL_Write_Read_Mem_Ap(0, DHCSR, &data, WRITE) == -1)
    {
        return -1;
    }
    return 0;
}

int16_t confSystemLowPower()
{
    // Set system and debug power up if cleared
    uint32_t val = 0;
    uint32_t data = 0;
    uint16_t* syncWithRunVarAddress = getTargetRunningVar();

    IHIL_Write_Read_Dp(DP_CTRL_STAT, &val, READ);

    // Request system power state tansitiopn

    // deassert CSYSPWRUPREQ
    val &=  ~DP_CTRL_STAT_CSYSPWRUPREQ;
    // keep CDBGPWRUPREQ asserted
    val |= DP_CTRL_STAT_CDBGPWRUPREQ;
    IHIL_Write_Read_Dp(DP_CTRL_STAT, &val, WRITE);

    if(syncWithRunVarAddress)
    {
        *syncWithRunVarAddress = 0x0001;
    }
    // Do the actual run
    data = DBGKEY | C_DEBUGEN;
    if(IHIL_Write_Read_Mem_Ap(0, DHCSR, &data, WRITE) == -1)
    {
        return -1;
    }
    // deassert CDBGPWRUPREQ
    val  &=  ~DP_CTRL_STAT_CDBGPWRUPREQ;
    IHIL_Write_Read_Dp(DP_CTRL_STAT, &val, WRITE);

    // wait some time for ACK to settle
    IHIL_Delay_1ms(200);

    return 0;
}

HAL_FUNCTION(_hal_RunArm)
{
#if defined(MSP430_UIF) || defined(MSP_FET)
    uint32_t data = 0;
    uint16_t releaseJtag = 0;
    uint16_t retry = MAX_RETRY;

    STREAM_get_word(&releaseJtag);

    // First Disable all breakpoints
    data = KEY;
    if(IHIL_Write_Read_Mem_Ap(0, FP_CTRL, &data, WRITE) == -1)
    {
        return HALERR_UNDEFINED_ERROR;
    }

    if (armConfigSettings.interruptOptions & DISABLE_INTERRUPTS_UPDATE_PRIMSK)
    {
      if (armConfigSettings.interruptOptions & DISABLE_INTERRUPTS_RUN)
      {
        // Making PRIMASK be effective requires one single step before
        ReadCpurRegArm(&data, spec_REG);
        data |= 0x01;
        writeCpuRegsArm(data, spec_REG);
      }
      else
      {
        // Making PRIMASK be effective requires one single step before
        ReadCpurRegArm(&data, spec_REG);
        data &= ~0x01;
        writeCpuRegsArm(data, spec_REG);
      }

      armConfigSettings.interruptOptions &= ~DISABLE_INTERRUPTS_UPDATE_PRIMSK;
    }

    // Perform single step with interrupts disabled to step over current hw breakpoint
    data = DBGKEY | C_DEBUGEN | C_MASKINTS | C_HALT;
    if(IHIL_Write_Read_Mem_Ap(0, DHCSR, &data, WRITE ) == -1)
    {
        return HALERR_UNDEFINED_ERROR;
    }

    data &= ~(C_HALT);
    data |= C_STEP;
    if(IHIL_Write_Read_Mem_Ap(0, DHCSR, &data, WRITE ) == -1)
    {
        return HALERR_UNDEFINED_ERROR;
    }

    // Wait for step to finish
    do
    {
        if(IHIL_Write_Read_Mem_Ap(0, DHCSR, &data, READ) == -1)
        {
            return HALERR_UNDEFINED_ERROR;
        }
    } while(!(data & S_HALT) && --retry);

    // disbale masking of interrupts
    data = DBGKEY | C_DEBUGEN | C_HALT;
    if(IHIL_Write_Read_Mem_Ap(0, DHCSR, &data, WRITE ) == -1)
    {
        return HALERR_UNDEFINED_ERROR;
    }

    // Re-enable all breakpoints
    data = KEY | ENABLE;
    if(IHIL_Write_Read_Mem_Ap(0, FP_CTRL, &data, WRITE) == -1)
    {
        return HALERR_UNDEFINED_ERROR;
    }

    if(armConfigSettings.ulpDebug)
    {
        if(confSystemLowPower() != 0)
        {
            return HALERR_UNDEFINED_ERROR;
        }
    }
    else
    {
        if(confSystemHighPower() != 0)
        {
            return HALERR_UNDEFINED_ERROR;
        }
    }

    if(releaseJtag)
    {
        IHIL_Close(); // release JTAG on go
    }
#endif
    return 0;
}
/*
 * \ingroup MODULMACROS
 *
 * \file ScanAPArm.c
 *
 * \brief <FILEBRIEF>
 */


/**
  ScanAP: Scan access port and provide back nil terminated list
*/
//#if defined(MSP430_UIF) || defined(MSP_FET)
//    extern uint32_t cswValues[4];
//#endif

HAL_FUNCTION(_hal_ScanApArm)
{
#if defined(MSP430_UIF) || defined(MSP_FET)
    uint32_t val = 0;
    uint32_t apsel = 0;

    // Set system and debug power up if cleared
    if( powerUpArm() != 0)
    {
        return HALERR_UNDEFINED_ERROR;
    }

    IHIL_Write_Read_Ap(AP_IDR | apsel, &val, READ);
    if(val)
    {
        STREAM_put_long(val); // IDR value

        IHIL_Write_Read_Ap(AP_BASE | apsel, &val, READ);
        STREAM_put_long(val); // BASE value

        IHIL_Write_Read_Ap(AP_CFG | apsel, &val, READ);
        STREAM_put_long(val); // CFG value

        STREAM_put_byte(apsel >> 24); // Port number

        // Read and save CSW value
        IHIL_Write_Read_Ap(AP_CSW | apsel, &val, READ);
        cswValues[(apsel >> 24) & 0x3] = val & ~(AP_CSW_SIZE_MASK | AP_CSW_ADDRINC_MASK);
    }
    STREAM_put_long(val);
#endif
    return 0;
}
/**
* \ingroup MODULMACROSXV2
*
* \file SendJtagMailboxXv2.c
*
* \brief <FILEBRIEF>
*/

/**
  WriteMemBytesXv2
  Write bytes to a memory mapped location.
  inData:  <addr(32)> <length(32)> <data(8)>{*}
  outData: -
  addr: the address to start writing to
  length: number of bytes to write
  data: data to write to the given location
*/

struct {
    uint16_t  MailBoxMode;
    uint16_t  DATA1;
    uint16_t  DATA2;
} _hal_SendJtagMailboxXv2_staticVars = {};

HAL_FUNCTION(_hal_SendJtagMailboxXv2)
{
    STATIC_VARS_START(_hal_SendJtagMailboxXv2);
    DECL_STATIC_VAR(MailBoxMode);
    DECL_STATIC_VAR(DATA1);
    DECL_STATIC_VAR(DATA2);
    
    uint8_t jtagMailboxIn = 0;

    if(STREAM_get_word(&MailBoxMode) != 0)
    {
        return(HALERR_UNDEFINED_ERROR);
    }
    // get length ot be flashed
    if(STREAM_get_word(&DATA1) != 0)
    {
        return(HALERR_UNDEFINED_ERROR);
    }
    if(STREAM_get_word(&DATA2) == -1)
    {
        return(HALERR_UNDEFINED_ERROR);
    }
    IHIL_EntrySequences(RSTLOW);
    IHIL_TapReset();

    if(MailBoxMode == 0x11)// 32Bit Mode
    {
        jtagMailboxIn = i_WriteJmbIn32(DATA1,DATA2);
    }
    else // 16 Bit Mode
    {
        jtagMailboxIn = i_WriteJmbIn(DATA1);
    }
    // check if mailbox input was ok
    if(jtagMailboxIn)
    {
        return(HALERR_UNDEFINED_ERROR);
    }
    IHIL_SetReset(1);
    IHIL_Delay_1ms(40);

    IHIL_EntrySequences(RSTHIGH);
    IHIL_Delay_1ms(200);
    IHIL_TapReset();


    if (jtagIdIsXv2(cntrl_sig_capture()))
    {
        return 0;
    }
    return HALERR_UNDEFINED_ERROR;
}
/*
 * \ingroup MODULMACROS
 *
 * \file SetChainConfiguration.c
 *
 * \brief <FILEBRIEF>
 */


HAL_FUNCTION(_hal_SetChainConfiguration)
{
    return (0);
}
/**
* \ingroup MODULMACROS
*
* \file SetDeviceChainInfo.c
*
* \brief <FILEBRIEF>
*/


HAL_FUNCTION(_hal_SetDeviceChainInfo)
{
    return 0;
}
/**
* \ingroup MODULMACROS
*
* \file SingleStep.c
*
* \brief Do everything that is needed to perform a SingleStep
*/


/**
  SingleStep
  Do everything that is needed to perform a SingleStep.
  inData:  <wdtAddr(16)> <wdtCtrl(16)> <PC(32)> <SR(16)><eemCtrl(16)> <mdb(16)>
  outData: <wdtCtrl(16)> <PC(32)> <SR(16)>
  wdtAddr: watchdog timer address
  wdtCtrl: watchdog timer control value
  PC: program counter register (R0)
  SR: status register (R2)
  eemCtrl: EEM control bits (EEM_EN and optional EEM_CLK_EN)
  mdb: value to be put on the Memory Data Bus before release (in case !0)
*/
HAL_FUNCTION(_hal_SingleStep)
{
    int16_t RetState = HALERR_UNDEFINED_ERROR;
    uint16_t i = 0;
    uint16_t bpCntrlType = BPCNTRL_IF;
    uint16_t extraStep = 0;

    // local variables to enable preserve of EEM trigger block 0
    uint16_t trig0_Bckup_cntrl;
    uint16_t trig0_Bckup_mask;
    uint16_t trig0_Bckup_comb;
    uint16_t trig0_Bckup_cpuStop;
    uint16_t trig0_Bckup_value;

    StreamSafe stream_tmp;
    uint8_t stream_in_release[18];
    uint8_t stream_in_sync[4];
    for(i=0;i<sizeof(stream_in_release);i++)
    {
        STREAM_get_byte(&stream_in_release[i]);
    }
    STREAM_discard_bytes(2);

    for(i=0;i<sizeof(stream_in_sync);i++)
    {
        STREAM_get_byte(&stream_in_sync[i]);
    }


    if (stream_in_release[8] & STATUS_REG_CPUOFF)
    {
      // If the CPU is OFF, only step after the CPU has been awakened by an interrupt.
      // This permits single step to work when the CPU is in LPM0 and 1 (as well as 2-4).
      bpCntrlType = BPCNTRL_NIF;
      // Emulation logic requires an additional step when the CPU is OFF (but only if there is not a pending interrupt).
      cntrl_sig_capture();
      if (!(SetReg_16Bits(0x0000)  & CNTRL_SIG_INTR_REQ))
      {
        extraStep = 1;
      }
    }

    { // Preserve breakpoint block 0
      eem_data_exchange();
      // read control register
      SetReg_16Bits(MX_CNTRL + MX_READ);     // shift in control register address with read request
      trig0_Bckup_cntrl = SetReg_16Bits(0x0000);                 // dummy shift to read out content remember content locally
      // read mask register
      SetReg_16Bits(MX_MASK + MX_READ);
      trig0_Bckup_mask = SetReg_16Bits(0x0000);
      // read combination register
      SetReg_16Bits(MX_COMB + MX_READ);
      trig0_Bckup_comb = SetReg_16Bits(0x0000);
      // read CPU stop reaction register
      SetReg_16Bits(MX_CPUSTOP + MX_READ);
      trig0_Bckup_cpuStop = SetReg_16Bits(0x0000);
      // read out trigger block value register
      SetReg_16Bits(MX_BP + MX_READ);
      trig0_Bckup_value = SetReg_16Bits(0x0000);
    }

    { // Configure "Single Step Trigger" using Trigger Block 0
      eem_data_exchange();
      // write control register
      SetReg_16Bits(MX_CNTRL + MX_WRITE);
      SetReg_16Bits(BPCNTRL_EQ | BPCNTRL_RW_DISABLE | bpCntrlType | BPCNTRL_MAB);
      // write mask register
      SetReg_16Bits(MX_MASK  + MX_WRITE);
      SetReg_16Bits((uint16_t)BPMASK_DONTCARE);
      // write combination register
      SetReg_16Bits(MX_COMB + MX_WRITE);
      SetReg_16Bits(0x0001);
      // write CPU stop reaction register
      SetReg_16Bits(MX_CPUSTOP + MX_WRITE);
      SetReg_16Bits(0x0001);
    }

    // TODO: Take care about the CPU cycles counts of the instructions to be single stepped
    //       Not implemented yet in DLLv3, refer to DLLv2 source code for implementation

    // now restore context and release CPU from JTAG control
    STREAM_internal_stream(stream_in_release, sizeof(stream_in_release), MESSAGE_NO_OUT, 0, &stream_tmp);
    RetState = (this->*HAL_RestoreContext_ReleaseJtag)(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG); // Data from DLL Stream
    STREAM_external_stream(&stream_tmp);

    if(RetState != 0)
    { // return error if not successful
      return RetState;
    }

    while(1) // this is not an endless loop, it will break if no "extra step" is required
    {
      // Wait for EEM stop reaction
      eem_read_control();
      do
      {
        i++;
      }
      while(!(SetReg_16Bits(0x0000) & 0x0080) && i < 50); // Wait for breakpoint hit

      // Timout?
      if(i >= 50)
      {
        return (HALERR_SINGLESTEP_WAITFOREEM_TIMEOUT);
      }

      // check if an extra step was required
      if(extraStep)
      {
        extraStep = 0;  // reset the "extra step" flag

        // release the CPU once again for an "extra step" from JTAG control
        addr_capture();
        cntrl_sig_release();
      }
      else
      {
        break;
      }
    }

    { // Restore Trigger Block 0
      eem_data_exchange();
      // write control register
      SetReg_16Bits(MX_CNTRL + MX_WRITE);
      SetReg_16Bits(trig0_Bckup_cntrl);
      // write mask register
      SetReg_16Bits(MX_MASK  + MX_WRITE);
      SetReg_16Bits(trig0_Bckup_mask);
      // write combination register
      SetReg_16Bits(MX_COMB + MX_WRITE);
      SetReg_16Bits(trig0_Bckup_comb);
      // write CPU stop reaction register
      SetReg_16Bits(MX_CPUSTOP + MX_WRITE);
      SetReg_16Bits(trig0_Bckup_cpuStop);
      // write trigger block value register
      SetReg_16Bits(MX_BP + MX_WRITE);
      SetReg_16Bits(trig0_Bckup_value);
    }

    // now sync the CPU again to JTAG control and save the current context
    STREAM_internal_stream(stream_in_sync, sizeof(stream_in_sync), MESSAGE_OUT_TO_DLL, 0, &stream_tmp);
    RetState = (this->*HAL_SyncJtag_Conditional_SaveContext)(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG); // In
    STREAM_external_stream(&stream_tmp);

    if(RetState != 0)
    { // return error if not successful
      return RetState;
    }

    return RetState;
}
/*
 * \ingroup MODULMACROS
 *
 * \file SingleStepArm.c
 *
 * \brief <FILEBRIEF>
 */


/**
  SingleStepMSP432: Single Steps the Core Debug
*/

#define DISABLE_INTERRUPTS_SINGLE_STEP 0x01

//#if defined(MSP430_UIF) || defined(MSP_FET)
//extern ARMConfigSettings armConfigSettings;
//#endif

HAL_FUNCTION(_hal_SingleStepArm)
{
#if defined(MSP430_UIF) || defined(MSP_FET)
    uint32_t data;
    uint16_t retry = MAX_RETRY;

    // First Disable all breakpoints
    data = KEY;
    if(IHIL_Write_Read_Mem_Ap(0, FP_CTRL, &data, WRITE) == -1)
    {
        return HALERR_UNDEFINED_ERROR;
    }

    // Perform actual single step with interrupts disabled
    data = DBGKEY | C_DEBUGEN | C_HALT;
    
    if (armConfigSettings.interruptOptions & DISABLE_INTERRUPTS_SINGLE_STEP)
    {
      data |= C_MASKINTS;
    }
    
    if(IHIL_Write_Read_Mem_Ap(0, DHCSR, &data, WRITE) == -1)
    {
        return HALERR_UNDEFINED_ERROR;
    }
    
    data &= ~(C_HALT);
    data |= C_STEP;
    if(IHIL_Write_Read_Mem_Ap(0, DHCSR, &data, WRITE) == -1)
    {
        return HALERR_UNDEFINED_ERROR;
    }
    
    // Wait for step to finish
    do
    {
        if(IHIL_Write_Read_Mem_Ap(0, DHCSR, &data, READ) == -1)
        {
            return HALERR_UNDEFINED_ERROR;
        }
    } while(!(data & S_HALT) && --retry);

    // disbale masking of interrupts
    data = DBGKEY | C_DEBUGEN | C_HALT;
    if(IHIL_Write_Read_Mem_Ap(0, DHCSR, &data, WRITE ) == -1)
    {
        return HALERR_UNDEFINED_ERROR;
    }
    
    // Re-enable all breakpoints
    data = KEY | ENABLE;
    if(IHIL_Write_Read_Mem_Ap(0, FP_CTRL, &data, WRITE) == -1)
    {
        return HALERR_UNDEFINED_ERROR;
    }
    if(!retry)
    {
        return HALERR_UNDEFINED_ERROR;
    }
#endif

    return 0;
}
/**
* \ingroup MODULMACROSXV2
*
* \file SingleStepJState.c
*
* \brief Do everything that is needed to perform a SingleStep
*/


/**
  SingleStepJStateXv2
  Same description and parameter list as SingleStep.
  Refer to SingleStep for details.
*/
HAL_FUNCTION(_hal_SingleStepJStateXv2)
{
  int16_t RetState = HALERR_UNDEFINED_ERROR;
  uint16_t i = 0;
  StreamSafe stream_tmp;
  uint8_t MyIn[31] = {
                  /*Number of EEM Data Exchanges*/
                  0x09,
                  /*Read Trigger Block 0*/
                  0x01,0x03,0x05,0x07,0x81,
                  /*Configure Trigger Block 0 for Single Step*/
                  0x02,0x00,0x00, 0x00,0x00,
                  0x04,0xFF,0xFF, 0x0F,0x00,
                  0x06,0x01,0x00, 0x00,0x00,
                  0x80,0x01,0x00, 0x00,0x00,
  };
  uint8_t MyOut[22]; // large enough to hold all EEM registers being read
  uint8_t stream_in_release[18];
  uint8_t stream_in_sync[4];
  uint16_t irRequest = 0;

  for(i=0;i<sizeof(stream_in_release);i++)
  {
    STREAM_get_byte(&stream_in_release[i]);
  }
  STREAM_get_word(&irRequest);

  for(i=0;i<sizeof(stream_in_sync);i++)
  {
    STREAM_get_byte(&stream_in_sync[i]);
  }

  if((!(stream_in_release[8] & CPUOFF)) || (irRequest & 0x4))
  {
    STREAM_internal_stream(MyIn, sizeof(MyIn), MyOut, sizeof(MyOut), &stream_tmp);
    (this->*HAL_EemDataExchangeXv2)(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG);
    STREAM_external_stream(&stream_tmp);
  }
  STREAM_internal_stream(stream_in_release, sizeof(stream_in_release), MESSAGE_NO_OUT, 0, &stream_tmp);
  (this->*HAL_RestoreContext_ReleaseJtagXv2)(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG); // Data from DLL Stream
  STREAM_external_stream(&stream_tmp);

  jstate_read();

  // check if device steped into LPMx5
  if((SetReg_64Bits(0x0000000000000000) & LPMX5_MASK_J))         // check if device steped into LPMx5 bit 64 will be 1
  {                                         // do not care about trigger block 0 values ->Lpmx.5 will reset the device eem module
      RetState = 0;
      return RetState;
  } //check end

  if((!(stream_in_release[8] & CPUOFF)) || (irRequest & 0x4))
  {
    // poll for CPU stop reaction
    jstate_read();
    do
    {
        i++;
    }
    while((BP_HIT_MASK_J != (SetReg_64Bits(0x0000000000000000) & BP_HIT_MASK_J)) && (i < 500));

    if(i < 500)
    {
      // take target under JTAG control
      STREAM_internal_stream(stream_in_sync, sizeof(stream_in_sync), MESSAGE_OUT_TO_DLL, 0, &stream_tmp);
      RetState = (this->*HAL_SyncJtag_Conditional_SaveContextXv2)(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG); // In
      STREAM_external_stream(&stream_tmp);
      // restore EEM Trigger Block 0
      MyIn[0]  = 0x05;
      MyIn[1]  = 0x00;
      MyIn[2]  = MyOut[0];
      MyIn[3]  = MyOut[1];
      MyIn[4]  = MyOut[2];
      MyIn[5]  = MyOut[3];
      MyIn[6]  = 0x02;
      MyIn[7]  = MyOut[4];
      MyIn[8]  = MyOut[5];
      MyIn[9]  = MyOut[6];
      MyIn[10] = MyOut[7];
      MyIn[11] = 0x04;
      MyIn[12] = MyOut[8];
      MyIn[13] = MyOut[9];
      MyIn[14] = MyOut[10];
      MyIn[15] = MyOut[11];
      MyIn[16] = 0x06;
      MyIn[17] = MyOut[12];
      MyIn[18] = MyOut[13];
      MyIn[19] = MyOut[14];
      MyIn[20] = MyOut[15];
      MyIn[21] = 0x80;
      MyIn[22] = MyOut[16];
      MyIn[23] = MyOut[17];
      MyIn[24] = MyOut[18];
      MyIn[25] = MyOut[19];

      STREAM_internal_stream(MyIn, sizeof(MyIn), MESSAGE_NO_OUT, 0, &stream_tmp);
      (this->*HAL_EemDataExchangeXv2)(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG);
      STREAM_external_stream(&stream_tmp);
    }
  }
  else
  {
      // take target under JTAG control
      STREAM_internal_stream(stream_in_sync, sizeof(stream_in_sync), MESSAGE_OUT_TO_DLL, 0, &stream_tmp);
      RetState = (this->*HAL_SyncJtag_Conditional_SaveContextXv2)(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG); // In
      STREAM_external_stream(&stream_tmp);
  }
  return RetState;
}
/**
* \ingroup MODULMACROSX
*
* \file SingleStepX.c
*
* \brief Do everything that is needed to perform a SingleStep
*/


/**
  SingleStepX
  Do everything that is needed to perform a SingleStep.
  inData:  <wdtAddr(16)> <wdtCtrl(16)> <PC(32)> <SR(16)><eemCtrl(16)> <mdb(16)>
  outData: <wdtCtrl(16)> <PC(32)> <SR(16)>
  wdtAddr: watchdog timer address
  wdtCtrl: watchdog timer control value
  PC: program counter register (R0)
  SR: status register (R2)
  eemCtrl: EEM control bits (EEM_EN and optional EEM_CLK_EN)
  mdb: value to be put on the Memory Data Bus before release (in case !0)
*/
HAL_FUNCTION(_hal_SingleStepX)
{
    int16_t RetState = HALERR_UNDEFINED_ERROR;
    uint16_t i = 0;
    uint16_t bpCntrlType = BPCNTRL_IF;
    uint16_t extraStep = 0;

    // local variables to enable preserve of EEM trigger block 0
    uint32_t trig0_Bckup_cntrl;
    uint32_t trig0_Bckup_mask;
    uint32_t trig0_Bckup_comb;
    uint32_t trig0_Bckup_cpuStop;
    uint32_t trig0_Bckup_value;

    StreamSafe stream_tmp;
    uint8_t stream_in_release[18];
    uint8_t stream_in_sync[4];
    for(i=0;i<sizeof(stream_in_release);i++)
    {
        STREAM_get_byte(&stream_in_release[i]);
    }
    STREAM_discard_bytes(2);

    for(i=0;i<sizeof(stream_in_sync);i++)
    {
        STREAM_get_byte(&stream_in_sync[i]);
    }

    if (stream_in_release[8] & STATUS_REG_CPUOFF)
    {
      // If the CPU is OFF, only step after the CPU has been awakened by an interrupt.
      // This permits single step to work when the CPU is in LPM0 and 1 (as well as 2-4).
      bpCntrlType = BPCNTRL_NIF;
      // Emulation logic requires an additional step when the CPU is OFF (but only if there is not a pending interrupt).
      cntrl_sig_capture();
      if (!(SetReg_16Bits(0x0000) & CNTRL_SIG_INTR_REQ))
      {
        extraStep = 1;
      }
    }

    { // Preserve breakpoint block 0
      eem_data_exchange32();
      // read control register
      SetReg_32Bits(MX_CNTRL + MX_READ);     // shift in control register address with read request
      // dummy shift to read out content
      trig0_Bckup_cntrl = SetReg_32Bits(0x0000);             // remember content locally
      // read mask register
      SetReg_32Bits(MX_MASK + MX_READ);
      trig0_Bckup_mask = SetReg_32Bits(0x0000);
      // read combination register
      SetReg_32Bits(MX_COMB + MX_READ);
      trig0_Bckup_comb = SetReg_32Bits(0x0000);
      // read CPU stop reaction register
      SetReg_32Bits(MX_CPUSTOP + MX_READ);
      trig0_Bckup_cpuStop = SetReg_32Bits(0x0000);
      // read out trigger block value register
      SetReg_32Bits(MX_BP + MX_READ);
      trig0_Bckup_value = SetReg_32Bits(0x0000);
    }

    { // Configure "Single Step Trigger" using Trigger Block 0
      eem_data_exchange32();
      // write control register
      SetReg_32Bits(MX_CNTRL + MX_WRITE);
      SetReg_32Bits(BPCNTRL_EQ | BPCNTRL_RW_DISABLE | bpCntrlType | BPCNTRL_MAB);
      // write mask register
      SetReg_32Bits(MX_MASK  + MX_WRITE);
      SetReg_32Bits(BPMASK_DONTCARE);
      // write combination register
      SetReg_32Bits(MX_COMB + MX_WRITE);
      SetReg_32Bits(0x0001);
      // write CPU stop reaction register
      SetReg_32Bits(MX_CPUSTOP + MX_WRITE);
      SetReg_32Bits(0x0001);
    }

    // now restore context and release CPU from JTAG control
    STREAM_internal_stream(stream_in_release, sizeof(stream_in_release), MESSAGE_NO_OUT, 0, &stream_tmp);
    RetState = (this->*HAL_RestoreContext_ReleaseJtagX)(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG); // Data from DLL Stream
    STREAM_external_stream(&stream_tmp);

    if(RetState != 0)
    { // return error if not successful
      return RetState;
    }

    while(1) // this is not an endless loop, it will break if no "extra step" is required
    {
      // Wait for EEM stop reaction
      eem_read_control();
      do
      {
        i++;
      }
      while(!(SetReg_16Bits(0x0000) & 0x0080) && i < 100);

      // Timout?
      if(i >= 100)
      {
        return (HALERR_SINGLESTEP_WAITFOREEM_TIMEOUT);
      }

      // check if an extra step was required
      if(extraStep)
      {
        extraStep = 0;  // reset the "extra step" flag

        // release the CPU once again for an "extra step" from JTAG control
        addr_capture();
        cntrl_sig_release();
      }
      else
      {
        break;
      }
    }

    { // Restore Trigger Block 0
      eem_data_exchange32();
      // write control register
      SetReg_32Bits(MX_CNTRL + MX_WRITE);
      SetReg_32Bits(trig0_Bckup_cntrl);
      // write mask register
      SetReg_32Bits(MX_MASK  + MX_WRITE);
      SetReg_32Bits(trig0_Bckup_mask);
      // write combination register
      SetReg_32Bits(MX_COMB + MX_WRITE);
      SetReg_32Bits(trig0_Bckup_comb);
      // write CPU stop reaction register
      SetReg_32Bits(MX_CPUSTOP + MX_WRITE);
      SetReg_32Bits(trig0_Bckup_cpuStop);
      // write trigger block value register
      SetReg_32Bits(MX_BP + MX_WRITE);
      SetReg_32Bits(trig0_Bckup_value);
    }

    // now sync the CPU again to JTAG control and save the current context
    STREAM_internal_stream(stream_in_sync, sizeof(stream_in_sync), MESSAGE_OUT_TO_DLL, 0, &stream_tmp);
    RetState = (this->*HAL_SyncJtag_Conditional_SaveContextX)(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG); // In
    STREAM_external_stream(&stream_tmp);

    if(RetState != 0)
    { // return error if not successful
      return RetState;
    }

    return RetState;
}
/**
* \ingroup MODULMACROSXV2
*
* \file SingleStepXv2.c
*
* \brief Do everything that is needed to perform a SingleStep
*/


/**
  SingleStepXv2
  Same description and parameter list as SingleStep.
  Refer to SingleStep for details.
*/
HAL_FUNCTION(_hal_SingleStepXv2)
{
  int16_t RetState = HALERR_UNDEFINED_ERROR;
  uint16_t i = 0;
  StreamSafe stream_tmp;
  uint8_t MyIn[31] = {
                  /*Number of EEM Data Exchanges*/
                  0x09,
                  /*Read Trigger Block 0*/
                  0x01,0x03,0x05,0x07,0x81,
                  /*Configure Trigger Block 0 for Single Step*/
                  0x02,0x00,0x00, 0x00,0x00,
                  0x04,0xFF,0xFF, 0x0F,0x00,
                  0x06,0x01,0x00, 0x00,0x00,
                  0x80,0x01,0x00, 0x00,0x00,
  };
  uint8_t MyOut[22]; // large enough to hold all EEM registers being read
  uint8_t stream_in_release[18];
  uint8_t stream_in_sync[4];
  uint16_t irRequest = 0;

  for(i=0;i<sizeof(stream_in_release);i++)
  {
    STREAM_get_byte(&stream_in_release[i]);
  }
  STREAM_get_word(&irRequest);

  for(i=0;i<sizeof(stream_in_sync);i++)
  {
    STREAM_get_byte(&stream_in_sync[i]);
  }
  if((!(stream_in_release[8] & CPUOFF)) || (irRequest & 0x4))
  {
    STREAM_internal_stream(MyIn, sizeof(MyIn), MyOut, sizeof(MyOut), &stream_tmp);
    (this->*HAL_EemDataExchangeXv2)(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG);
    STREAM_external_stream(&stream_tmp);
  }
  STREAM_internal_stream(stream_in_release, sizeof(stream_in_release), MESSAGE_NO_OUT, 0, &stream_tmp);
  (this->*HAL_RestoreContext_ReleaseJtagXv2)(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG); // Data from DLL Stream
  STREAM_external_stream(&stream_tmp);

  if((!(stream_in_release[8] & CPUOFF)) || (irRequest & 0x4))
  {
    // poll for CPU stop reaction
    eem_read_control();
    do
    {
      i++;
    }
    while(!(SetReg_16Bits(0x0000) & 0x0080) && (i < 500));

    if(i < 500)
    {
      // take target under JTAG control
      STREAM_internal_stream(stream_in_sync, sizeof(stream_in_sync), MESSAGE_OUT_TO_DLL, 0, &stream_tmp);
      RetState = (this->*HAL_SyncJtag_Conditional_SaveContextXv2)(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG); // In
      STREAM_external_stream(&stream_tmp);
      // restore EEM Trigger Block 0
      MyIn[0]  = 0x05;
      MyIn[1]  = 0x00;
      MyIn[2]  = MyOut[0];
      MyIn[3]  = MyOut[1];
      MyIn[4]  = MyOut[2];
      MyIn[5]  = MyOut[3];
      MyIn[6]  = 0x02;
      MyIn[7]  = MyOut[4];
      MyIn[8]  = MyOut[5];
      MyIn[9]  = MyOut[6];
      MyIn[10] = MyOut[7];
      MyIn[11] = 0x04;
      MyIn[12] = MyOut[8];
      MyIn[13] = MyOut[9];
      MyIn[14] = MyOut[10];
      MyIn[15] = MyOut[11];
      MyIn[16] = 0x06;
      MyIn[17] = MyOut[12];
      MyIn[18] = MyOut[13];
      MyIn[19] = MyOut[14];
      MyIn[20] = MyOut[15];
      MyIn[21] = 0x80;
      MyIn[22] = MyOut[16];
      MyIn[23] = MyOut[17];
      MyIn[24] = MyOut[18];
      MyIn[25] = MyOut[19];

      STREAM_internal_stream(MyIn, sizeof(MyIn), MESSAGE_NO_OUT, 0, &stream_tmp);
      (this->*HAL_EemDataExchangeXv2)(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG);
      STREAM_external_stream(&stream_tmp);
    }
  }
  else
  {
      // First Check if we did not go into LPMx.5
      if (jtagIdIsValid(cntrl_sig_capture()))
      {
          // take target under JTAG control
          STREAM_internal_stream(stream_in_sync, sizeof(stream_in_sync), MESSAGE_OUT_TO_DLL, 0, &stream_tmp);
          RetState = (this->*HAL_SyncJtag_Conditional_SaveContextXv2)(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG); // In
          STREAM_external_stream(&stream_tmp);
      }
      else
      {
          // Device stepped into LPMx.5
          RetState = 0;
      }
  }
  return RetState;
}
/**
* \ingroup MODULMACROS
*
* \file StartJtag.c
*
* \brief Open and reset the physical connection to the target device
* JTAG interface
*/


/**
  StartJtag
  Open and reset the physical connection to the target device JTAG interface.
  Implies release of the target from JTAG control.
  inData:  <protocol(8)>
  outData: <jtagId(8)>
  protocol: 0 = 4-wire Jtag
  jtagId: the Jtag identifier value
*/

#if defined(eZ_FET) || defined(MSP_FET)
HAL_FUNCTION(_hal_StartJtag)
{
    uint8_t chainLen;
    uint8_t protocol;
    int16_t ret_value = -1;
//    HilInitGetEdtDistinctFunc hilEdtDis = ((void*)0);

    if(STREAM_get_byte(&protocol) < 0)
    {
        ret_value = HALERR_START_JTAG_NO_PROTOCOL;
        return(ret_value);
    }

    if(IHIL_SetProtocol(protocol))
    {
        ret_value = HALERR_START_JTAG_PROTOCOL_UNKNOWN;
        return(ret_value);
    }

    // MSPFETSim: not sure what this craziness is, but its result is calling `_hil_getEdtDistinct`
//    HilInitGetEdtDistinctFunc hilEdtDis = (HilInitGetEdtDistinctFunc)0x1880;
//    hilEdtDis(&_edt_Distinct_Methods_HAL);
    _hil_getEdtDistinct(&_edt_Distinct_Methods_HAL);

    IHIL_Open(RSTHIGH);
    IHIL_TapReset();
    chainLen = 1;
    IHIL_CheckJtagFuse();

    STREAM_put_byte((uint8_t)chainLen);
    if (chainLen > 0)
    {
        ret_value = 0;
    }
    return(ret_value);
}
#endif

#ifdef MSP430_UIF
HAL_FUNCTION(_hal_StartJtag)
{
    uint8_t chainLen;
    uint8_t protocol;
    int16_t ret_value = HALERR_UNDEFINED_ERROR;

    if(STREAM_get_byte(&protocol) < 0)
    {
        ret_value = HALERR_START_JTAG_NO_PROTOCOL;
        return(ret_value);
    }
    if(IHIL_SetProtocol(protocol))
    {
        ret_value = HALERR_START_JTAG_PROTOCOL_UNKNOWN;
        return(ret_value);
    }

    IHIL_Open(RSTHIGH);
    IHIL_TapReset();
    chainLen = 1;
    IHIL_CheckJtagFuse();

    STREAM_put_byte((uint8_t)chainLen);
    if (chainLen > 0)
    {
        ret_value = 0;
    }
    return(ret_value);
}
#endif
/**
* \ingroup MODULMACROS
*
* \file StartJtagActivationCode.c
*
* \brief <FILEBRIEF>
*/


/**
  StartJtag
  Open and reset the physical connection to the target device JTAG interface.
  Implies release of the target from JTAG control.
  inData:  <protocol(8)>
  outData: <jtagId(8)>
  protocol: 0 = 4-wire Jtag
  jtagId: the Jtag identifier value
*/

HAL_FUNCTION(_hal_StartJtagActivationCode)
{
    uint8_t chainLen = 0;
    uint8_t protocol;
    uint32_t ActivationCode;
    int16_t ret_value = HALERR_UNDEFINED_ERROR;
    uint8_t jtagMailboxIn = 0;

    if(STREAM_get_byte(&protocol) < 0)
    {
        ret_value = HALERR_START_JTAG_NO_PROTOCOL;
        return(ret_value);
    }
    STREAM_discard_bytes(1);

    if(STREAM_get_long(&ActivationCode) < 0)
    {
        ret_value = HALERR_START_JTAG_NO_ACTIVATION_CODE;
        return(ret_value);
    }
    // LO92 could just operrate in JTAG mode. Perhaps this must be changed for
    // feature devices
    protocol = 0;
    if(IHIL_SetProtocol(protocol))
    {
        ret_value = HALERR_START_JTAG_PROTOCOL_UNKNOWN;
        return(ret_value);
    }

#if defined(eZ_FET) || defined(MSP_FET)
    {
        // MSPFETSim: not sure what this craziness is, but its result is calling `_hil_getEdtDistinct`
//        HilInitGetEdtDistinctFunc hilEdtDis = (HilInitGetEdtDistinctFunc)0x1880;
//        hilEdtDis(&_edt_Distinct_Methods_HAL);
        _hil_getEdtDistinct(&_edt_Distinct_Methods_HAL);
    }
#endif

    // check if activation code is valid
    if(ActivationCode == L092_MODE || ActivationCode == C092_MODE)
    {
        // L092 has no JTAG chain support
        IHIL_SetReset(0);
        IHIL_Open(RSTLOW);
        IHIL_TapReset();
        IHIL_SetReset(0);
        chainLen = 1;

        jtagMailboxIn = i_WriteJmbIn32(ActivationCode>>16 , ActivationCode);
        // check for Timeout issue during mailbox request
        IHIL_SetReset(1);
        IHIL_Delay_1ms(3000);
        if(jtagMailboxIn == 1)
        {
            return (HALERR_JTAG_PASSWORD_WRONG);
        }
    }
    STREAM_put_byte((uint8_t)chainLen);
    if (chainLen > 0)
    {
        ret_value = 0;
    }
    return(ret_value);
}

/**
* \ingroup MODULMACROS
*
* \file StopJtag.c
*
* \brief <FILEBRIEF>
*/


//extern DevicePowerSettings devicePowerSettings;

/**
  StopJtag
  Close the physical connection to the target JTAG interface.
*/

HAL_FUNCTION(_hal_StopJtag)
{
    RestoreTestRegs();
    IHIL_Close();

  return 0;
}

/**
* \ingroup MODULMACROS
*
* \file SyncJtag_AssertPor_SaveContext.c
*
* \brief Sync with device, assert a Power On Reset and save processor context
*/


//extern DeviceSettings deviceSettings;

/**
  SyncJtag_AssertPor_SaveContext
  inData:  <wdtAddr(16)> <wdtCtrl(16)>
  outData: <wdtCtrl(16)> <PC(32)> <SR(16)>
  wdtAddr: watchdog timer address
  wdtCtrl: watchdog timer control value
  PC: program counter register (R0)
  SR: status register (R2)
*/
HAL_FUNCTION(_hal_SyncJtag_AssertPor_SaveContext)
{
    uint16_t i = 0, lOut = 0, ctl_sync = 0;
    uint16_t MyOut[4];    //! Output
    uint16_t address;
    uint16_t wdtVal;
    uint16_t* syncWithRunVarAddress = 0;

    // Check input parameters before we proceed
    if(STREAM_get_word(&address) != 0)
    {
        return (HALERR_SYNC_JTAG_ASSERT_POR_NO_WDT_ADDRESS);
    }
    if(STREAM_get_word(&wdtVal) != 0)
    {
        return (HALERR_SYNC_JTAG_ASSERT_POR_NO_WDT_VALUE);
    }

    syncWithRunVarAddress = getTargetRunningVar();
    if(syncWithRunVarAddress)
    {
        *syncWithRunVarAddress = 0x0000;
    }

    // Sync the JTAG
    cntrl_sig_16bit();
    SetReg_16Bits(0x2401);

    if(cntrl_sig_capture() != JTAGVERSION)
    {
        return (HALERR_JTAG_VERSION_MISMATCH);
    }

    cntrl_sig_capture();
    lOut = SetReg_16Bits(0x0000);    // read control register once

    IHIL_Tclk(1);

    if(!(lOut & CNTRL_SIG_TCE))
    {
        // If the JTAG and CPU are not already synchronized ...
        // Initiate Jtag and CPU synchronization. Read/Write is under CPU control. Source TCLK via TDI.
        // Do not effect bits used by DTC (CPU_HALT, MCLKON).
        cntrl_sig_high_byte();
        SetReg_8Bits((CNTRL_SIG_TAGFUNCSAT | CNTRL_SIG_TCE1 | CNTRL_SIG_CPU) >> 8); // initiate CPU synchronization but release low byte of CNTRL sig register to CPU control

        // Address Force Sync special handling
        eem_data_exchange();               // read access to EEM General Clock Control Register (GCLKCTRL)
        SetReg_16Bits(MX_GCLKCTRL + MX_READ);
        lOut = SetReg_16Bits(0);                 // read the content of GCLKCNTRL into lOUt
        // Set Force Jtag Synchronization bit in Emex General Clock Control register.
        lOut |=  0x0040;                 // 0x0040 = FORCE_SYN in DLLv2
        eem_data_exchange();                // Stability improvement: should be possible to remove this, required only once at the beginning
        SetReg_16Bits(MX_GCLKCTRL + MX_WRITE);   // write access to EEM General Clock Control Register (GCLKCTRL)
        lOut = SetReg_16Bits(lOut);              // write into GCLKCNTRL
        // Reset Force Jtag Synchronization bit in Emex General Clock Control register.
        lOut &= ~0x0040;
        eem_data_exchange();                // Stability improvement: should be possible to remove this, required only once at the beginning
        SetReg_16Bits(MX_GCLKCTRL + MX_WRITE);   // write access to EEM General Clock Control Register (GCLKCTRL)
        lOut = SetReg_16Bits(lOut);             // write into GCLKCNTRL

        ctl_sync =  SyncJtag();

        if(!(ctl_sync & CNTRL_SIG_CPU_HALT))
        { // Synchronization failed!
            return (HALERR_SYNC_JTAG_ASSERT_POR_JTAG_TIMEOUT);
        }
    }// end of if(!(lOut & CNTRL_SIG_TCE))

    if(ctl_sync & CNTRL_SIG_CPU_HALT)
    {
        IHIL_Tclk(0);
        cntrl_sig_16bit();
        SetReg_16Bits(0x2401);
        IHIL_Tclk(1);
    }
    else
    {
        cntrl_sig_16bit();
        SetReg_16Bits(0x2401);
    }

    if (deviceSettings.assertBslValidBit)
    {
        // here we add bit de assert bit 7 in JTAG test reg to enalbe clocks again
        test_reg();
        lOut = SetReg_8Bits(0x00);
        lOut |= 0x80; //DE_ASSERT_BSL_VALID;
        test_reg();
        SetReg_8Bits(lOut); // Bit 7 is de asserted now
    }

    // execute a dummy instruction here
    data_16bit();
    IHIL_Tclk(1);                   // Stability improvement: should be possible to remove this TCLK is already 1
    SetReg_16Bits(0x4303);         // 0x4303 = NOP
    IHIL_Tclk(0);
    data_capture();
    IHIL_Tclk(1);

    // step until next instruction load boundary if not being already there
    if(instrLoad() != 0)
    {
        return (HALERR_INSTRUCTION_BOUNDARY_ERROR);
    }

    if (deviceSettings.clockControlType == GCC_EXTENDED)
    {
        // Perform the POR
        eem_data_exchange();
        SetReg_16Bits(MX_GENCNTRL + MX_WRITE);               // write access to EEM General Control Register (MX_GENCNTRL)
        SetReg_16Bits(EMU_FEAT_EN | EMU_CLK_EN | CLEAR_STOP | EEM_EN);   // write into MX_GENCNTRL

        eem_data_exchange(); // Stability improvement: should be possible to remove this, required only once at the beginning
        SetReg_16Bits(MX_GENCNTRL + MX_WRITE);               // write access to EEM General Control Register (MX_GENCNTRL)
        SetReg_16Bits(EMU_FEAT_EN | EMU_CLK_EN);         // write into MX_GENCNTRL
    }
    if (deviceSettings.clockControlType == GCC_STANDARD_I)
    {
        eem_data_exchange();
        SetReg_16Bits(MX_GENCNTRL + MX_WRITE);  // write access to EEM General Control Register (MX_GENCNTRL)
        SetReg_16Bits(EMU_FEAT_EN);             // write into MX_GENCNTRL
    }

    IHIL_Tclk(0);
    cntrl_sig_16bit();
    SetReg_16Bits(CNTRL_SIG_READ | CNTRL_SIG_TCE1 | CNTRL_SIG_PUC | CNTRL_SIG_TAGFUNCSAT); // Assert PUC
    IHIL_Tclk(1);
    cntrl_sig_16bit();
    SetReg_16Bits(CNTRL_SIG_READ | CNTRL_SIG_TCE1 | CNTRL_SIG_TAGFUNCSAT); // Negate PUC

    IHIL_Tclk(0);
    cntrl_sig_16bit();
    SetReg_16Bits(CNTRL_SIG_READ | CNTRL_SIG_TCE1 | CNTRL_SIG_PUC | CNTRL_SIG_TAGFUNCSAT); // Assert PUC
    IHIL_Tclk(1);
    cntrl_sig_16bit();
    SetReg_16Bits(CNTRL_SIG_READ | CNTRL_SIG_TCE1 | CNTRL_SIG_TAGFUNCSAT); // Negate PUC

    // Explicitly set TMR
    SetReg_16Bits(CNTRL_SIG_READ | CNTRL_SIG_TCE1); // Enable access to Flash registers

    flash_16bit_update();               // Disable flash test mode
    SetReg_16Bits(FLASH_SESEL1);     // Pulse TMR
    SetReg_16Bits(FLASH_SESEL1 | FLASH_TMR);
    SetReg_16Bits(FLASH_SESEL1);
    SetReg_16Bits(FLASH_SESEL1 | FLASH_TMR); // Set TMR to user mode

    cntrl_sig_high_byte();
    SetReg_8Bits((CNTRL_SIG_TAGFUNCSAT | CNTRL_SIG_TCE1) >> 8); // Disable access to Flash register

    // step until an appropriate instruction load boundary
    for(i = 0; i < 10; i++)
    {
      addr_capture();
        lOut = SetReg_16Bits(0x0000);
        if(lOut == 0xFFFE || lOut == 0x0F00)
        {
            break;
        }
        IHIL_TCLK();
    }
    if(i == 10)
    {
        return (HALERR_INSTRUCTION_BOUNDARY_ERROR);
    }
    IHIL_TCLK();
    IHIL_TCLK();

    IHIL_Tclk(0);
    addr_capture();
    SetReg_16Bits(0x0000);
    IHIL_Tclk(1);

    // step until next instruction load boundary if not being already there

    if(instrLoad() != 0)
    {
        return (HALERR_INSTRUCTION_BOUNDARY_ERROR);
    }

    // Hold Watchdog
    MyOut[0] = ReadMemWord(address); // safe WDT value
    wdtVal |= (MyOut[0] & 0xFF); // set original bits in addition to stop bit
    WriteMemWord(address, wdtVal);

    // read MAB = PC here
    addr_capture();
    MyOut[1] = SetReg_16Bits(0);
    MyOut[2] = 0; // high PC always 0 for MSP430 architecture

    // set PC to a save address pointing to ROM to avoid RAM corruption on certain devices
    SetPc(ROM_ADDR);

    // read status register
    MyOut[3] = ReadCpuReg(2);

    // return output
    STREAM_put_bytes((uint8_t*)MyOut,8);

    return(0);
}

/**
* \ingroup MODULMACROSX
*
* \file SyncJtag_AssertPor_SaveContextX.c
*
* \brief Sync with device, assert a Power On Reset and save processor context
*/


//extern DeviceSettings deviceSettings;

/**
  SyncJtag_AssertPor_SaveContextX
  inData:  <wdtAddr(16)> <wdtCtrl(16)>
  outData: <wdtCtrl(16)> <PC(32)> <SR(16)>
  wdtAddr: watchdog timer address
  wdtCtrl: watchdog timer control value
  PC: program counter register (R0)
  SR: status register (R2)
*/
HAL_FUNCTION(_hal_SyncJtag_AssertPor_SaveContextX)
{
    uint16_t i = 0, ctl_sync = 0;
    uint16_t MyOut[4];    //! Output
    uint16_t address;
    uint16_t wdtVal, lOut = 0;
    uint16_t* syncWithRunVarAddress = 0;
    uint32_t lOut_long = 0;

    // Check input parameters before we proceed
    if(STREAM_get_word(&address) != 0)
    {
        return (HALERR_SYNC_JTAG_ASSERT_POR_NO_WDT_ADDRESS);
    }
    if(STREAM_get_word(&wdtVal) != 0)
    {
        return (HALERR_SYNC_JTAG_ASSERT_POR_NO_WDT_VALUE);
    }

    syncWithRunVarAddress = getTargetRunningVar();
    if(syncWithRunVarAddress)
    {
        *syncWithRunVarAddress = 0x0000;
    }

    // Sync the JTAG
    cntrl_sig_16bit();
    SetReg_16Bits(0x2401);
    if(cntrl_sig_capture() != JTAGVERSION)
    {
        return (HALERR_JTAG_VERSION_MISMATCH);
    }

    cntrl_sig_capture();
    lOut = SetReg_16Bits(0x0000);    // read control register once

    IHIL_Tclk(1);

    if(!(lOut & CNTRL_SIG_TCE))
    {
        // If the JTAG and CPU are not already synchronized ...
        // Initiate Jtag and CPU synchronization. Read/Write is under CPU control. Source TCLK via TDI.
        // Do not effect bits used by DTC (CPU_HALT, MCLKON).
        cntrl_sig_high_byte();
        SetReg_8Bits((CNTRL_SIG_TAGFUNCSAT | CNTRL_SIG_TCE1 | CNTRL_SIG_CPU) >> 8); // initiate CPU synchronization but release low byte of CNTRL sig register to CPU control

        // Address Force Sync special handling
        eem_data_exchange32();              // read access to EEM General Clock Control Register (GCLKCTRL)
        SetReg_32Bits(MX_GCLKCTRL + MX_READ);
        lOut = SetReg_32Bits(0);                 // read the content of GCLKCNTRL into lOUt
        // Set Force Jtag Synchronization bit in Emex General Clock Control register.
        lOut |=  0x0040;                 // 0x0040 = FORCE_SYN in DLLv2
        eem_data_exchange32();                // Stability improvement: should be possible to remove this, required only once at the beginning
        SetReg_32Bits(MX_GCLKCTRL + MX_WRITE);   // write access to EEM General Clock Control Register (GCLKCTRL)
        lOut = SetReg_32Bits(lOut);              // write into GCLKCNTRL
        // Reset Force Jtag Synchronization bit in Emex General Clock Control register.
        lOut &= ~0x0040;
        eem_data_exchange32();                // Stability improvement: should be possible to remove this, required only once at the beginning
        SetReg_32Bits(MX_GCLKCTRL + MX_WRITE);   // write access to EEM General Clock Control Register (GCLKCTRL)
        SetReg_32Bits(lOut);              // write into GCLKCNTRL

        ctl_sync =  SyncJtag();

        if(!ctl_sync)
        { // Synchronization failed!
            return (HALERR_SYNC_JTAG_ASSERT_POR_JTAG_TIMEOUT);
        }
    }// end of if(!(lOut & CNTRL_SIG_TCE))

    if(ctl_sync & CNTRL_SIG_CPU_HALT)
    {
        IHIL_Tclk(0);
        cntrl_sig_16bit();
        SetReg_16Bits(0x2401);
        IHIL_Tclk(1);
    }
    else
    {
        cntrl_sig_16bit();
        SetReg_16Bits(0x2401);
    }
    // execute a dummy instruction here
    data_16bit();
    IHIL_Tclk(1);                   // Stability improvement: should be possible to remove this TCLK is already 1
    SetReg_16Bits(0x4303);         // 0x4303 = NOP
    IHIL_Tclk(0);
    data_capture();
    IHIL_Tclk(1);

    // step until next instruction load boundary if not being already there
    if( instrLoad() != 0)
    {
        return (HALERR_INSTRUCTION_BOUNDARY_ERROR);
    }

    if (deviceSettings.clockControlType == GCC_EXTENDED)
    {
        eem_data_exchange32();
        SetReg_32Bits(MX_GENCNTRL + MX_WRITE);               // write access to EEM General Control Register (MX_GENCNTRL)
        SetReg_32Bits(EMU_FEAT_EN | EMU_CLK_EN | CLEAR_STOP | EEM_EN);   // write into MX_GENCNTRL

        eem_data_exchange32(); // Stability improvement: should be possible to remove this, required only once at the beginning
        SetReg_32Bits(MX_GENCNTRL + MX_WRITE);               // write access to EEM General Control Register (MX_GENCNTRL)
        SetReg_32Bits(EMU_FEAT_EN | EMU_CLK_EN);         // write into MX_GENCNTRL
    }

    IHIL_Tclk(0);
    cntrl_sig_16bit();
    SetReg_16Bits(CNTRL_SIG_READ | CNTRL_SIG_TCE1 | CNTRL_SIG_PUC | CNTRL_SIG_TAGFUNCSAT); // Assert PUC
    IHIL_Tclk(1);
    cntrl_sig_16bit();
    SetReg_16Bits(CNTRL_SIG_READ | CNTRL_SIG_TCE1 | CNTRL_SIG_TAGFUNCSAT); // Negate PUC

    IHIL_Tclk(0);
    cntrl_sig_16bit();
    SetReg_16Bits(CNTRL_SIG_READ | CNTRL_SIG_TCE1 | CNTRL_SIG_PUC | CNTRL_SIG_TAGFUNCSAT); // Assert PUC
    IHIL_Tclk(1);
    cntrl_sig_16bit();
    SetReg_16Bits(CNTRL_SIG_READ | CNTRL_SIG_TCE1 | CNTRL_SIG_TAGFUNCSAT); // Negate PUC

    // Explicitly set TMR
    SetReg_16Bits(CNTRL_SIG_READ | CNTRL_SIG_TCE1); // Enable access to Flash registers

    flash_16bit_update();               // Disable flash test mode
    SetReg_16Bits(FLASH_SESEL1);     // Pulse TMR
    SetReg_16Bits(FLASH_SESEL1 | FLASH_TMR);
    SetReg_16Bits(FLASH_SESEL1);
    SetReg_16Bits(FLASH_SESEL1 | FLASH_TMR); // Set TMR to user mode

    cntrl_sig_high_byte();
    SetReg_8Bits((CNTRL_SIG_TAGFUNCSAT | CNTRL_SIG_TCE1) >> 8); // Disable access to Flash register

    // step until an appropriate instruction load boundary
    for(i = 0; i < 10; i++)
    {
        addr_capture();
        lOut_long = SetReg_20Bits(0x0000);
        if((lOut_long & 0xFFFF) == 0xFFFE || (lOut_long & 0xFFFF) == 0x0F00)
        {
            break;
        }
        IHIL_TCLK();
    }
    if(i == 10)
    {
        return (HALERR_INSTRUCTION_BOUNDARY_ERROR);
    }
    IHIL_TCLK();
    IHIL_TCLK();

    IHIL_Tclk(0);
    addr_capture();
    SetReg_16Bits(0x0000);
    IHIL_Tclk(1);

    // step until next instruction load boundary if not being already there
    if(instrLoad() != 0)
    {
        return (HALERR_INSTRUCTION_BOUNDARY_ERROR);
    }

    // Hold Watchdog
    MyOut[0] = ReadMemWordX(address); // safe WDT value
    wdtVal |= (MyOut[0] & 0x00FF); // set original bits in addition to stop bit
    WriteMemWordX(address, wdtVal);

    // read MAB = PC here
    addr_capture();
    lOut_long = SetReg_20Bits(0);
    MyOut[1] = (uint16_t)(lOut_long & 0xFFFF);
    MyOut[2] = (uint16_t)(lOut_long >> 16);


    // set PC to a save address pointing to ROM to avoid RAM corruption on certain devices
    SetPcX(ROM_ADDR);

    {
        uint32_t Rx;
        // read status register
        Rx = ReadCpuRegX(2);
        MyOut[3] = (uint16_t)Rx;
    }

    // return output
    STREAM_put_bytes((uint8_t*)MyOut,8);

    return(0);
}

/**
* \ingroup MODULMACROSXV2
*
* \file SyncJtag_AssertPor_SaveContextXv2.c
*
* \brief Sync with device, assert a Power On Reset and save processor context
*/



//extern uint32_t _hal_mclkCntrl0;
uint8_t mclk_modules[32];

/**
  SyncJtag_AssertPor_SaveContextXv2
  inData:  <wdtAddr(16)> <wdtCtrl(16)>
  outData: <wdtCtrl(16)> <PC(32)> <SR(16)>
  wdtAddr: watchdog timer address
  wdtCtrl: watchdog timer control value
  PC: program counter register (R0)
  SR: status register (R2)
*/

//extern DevicePowerSettings devicePowerSettings;

HAL_FUNCTION(_hal_SyncJtag_AssertPor_SaveContextXv2)
{
    uint16_t MyOut[4];
    int16_t i;
    uint16_t address;
    uint16_t wdtVal;
    uint16_t id = cntrl_sig_capture();
    uint32_t lOut_long = 0;

    // -------------------------Power mode handling start ----------------------
    //Disable Lpmx5 and power gaiting settings
    if( id == JTAGVERSION99)
    {
        test_reg_3V();
        SetReg_16Bits(0x40A0);
        test_reg();
        SetReg_32Bits(0x00010000);
    }
    // -------------------------Power mode handling end ------------------------
    // enable clock control before sync
    // switch all functional clocks to JCK = 1 and stop them
    eem_data_exchange32();
    SetReg_32Bits(GENCLKCTRL + WRITE);
    SetReg_32Bits(MCLK_SEL3 + SMCLK_SEL3 + ACLK_SEL3 + STOP_MCLK + STOP_SMCLK + STOP_ACLK);
    // enable Emualtion Clocks
    eem_write_control();
    SetReg_16Bits(EMU_CLK_EN + EEM_EN);

    cntrl_sig_16bit();
    // release RW and BYTE control signals in low byte, set TCE1 & CPUSUSP(!!) & RW
    SetReg_16Bits(0x1501);

    if(wait_for_synch())
    {
        // provide one more clock to empty the pipe
        IHIL_TCLK();

        cntrl_sig_16bit();
        // release CPUFLUSH(=CPUSUSP) signal and apply POR signal
        SetReg_16Bits(0x0C01);
        IHIL_Delay_1ms(40);

        // release POR signal again
        SetReg_16Bits(0x0401); // disable fetch of CPU // changed from 401 to 501

        if(id == JTAGVERSION91 || id == JTAGVERSION99 || id == JTAGVERSION98)
        {   // Force PC so safe value memory location, JMP $
            data_16bit();
            IHIL_TCLK();
            IHIL_TCLK();
            SetReg_16Bits(SAFE_PC_ADDRESS);
            // drive safe address into pc end
            IHIL_TCLK();

            if(id == JTAGVERSION91)
            {
                IHIL_TCLK();
            }

            data_capture();
        }
        else
        {
            IHIL_TCLK();
            IHIL_TCLK();
            IHIL_TCLK();
        }

        // TWO more to release CPU internal POR delay signals
        IHIL_TCLK();
        IHIL_TCLK();

        // set CPUFLUSH signal
        cntrl_sig_16bit();
        SetReg_16Bits(0x0501);
        IHIL_TCLK();

        // set EEM FEATURE enable now!!!
        eem_write_control();
        SetReg_16Bits(EMU_FEAT_EN + EMU_CLK_EN + CLEAR_STOP);

        // Check that sequence exits on Init State
        cntrl_sig_capture();
        SetReg_16Bits(0x0000);
    //    lout == 0x0301,0x3fd3
        // hold Watchdog Timer
        STREAM_get_word(&address);
        STREAM_get_word(&wdtVal);

        MyOut[0] = ReadMemWordXv2(address);
        wdtVal |= (MyOut[0] & 0xFF); // set original bits in addition to stop bit
        WriteMemWordXv2(address, wdtVal);

        // Capture MAB - the actual PC value is (MAB - 4)
        addr_capture();
        lOut_long = SetReg_20Bits(0);
        /*****************************************/
        /* Note 1495, 1637 special handling      */
        /*****************************************/
        if(((lOut_long & 0xFFFF) == 0xFFFE) || (id == JTAGVERSION91) || (id == JTAGVERSION99) || (id == JTAGVERSION98))
        {
          MyOut[1] = ReadMemWordXv2(0xFFFE);
          MyOut[2] = 0;
        }
        /*********************************/
        else
        {
          lOut_long -= 4;
          MyOut[1] = (uint16_t)(lOut_long & 0xFFFF);
          MyOut[2] = (uint16_t)(lOut_long >>16);
        }
        // Status Register should be always 0 after a POR
        MyOut[3] = 0;
        STREAM_discard_bytes(1);
        for(i=0; i<32; i++)
        {
          uint16_t v;
          STREAM_get_byte(&mclk_modules[i]);

          if(mclk_modules[i] != 0)
          {
            // check if module clock control is enabled for corresponding module
            if(_hal_mclkCntrl0 & ((uint32_t) 0x00000001 << i))
              v = 1;
            else
              v = 0;
            WriteMemWordXv2(ETKEYSEL, ETKEY + mclk_modules[i]);
            WriteMemWordXv2(ETCLKSEL, v);
          }
        }
        // switch back system clocks to original clock source but keep them stopped
        eem_data_exchange32();
        SetReg_32Bits(GENCLKCTRL + WRITE);
        SetReg_32Bits(MCLK_SEL0 + SMCLK_SEL0 + ACLK_SEL0 + STOP_MCLK + STOP_SMCLK + STOP_ACLK);

        // reset Vacant Memory Interrupt Flag inside SFRIFG1
        if(id == JTAGVERSION91)
        {
            volatile uint16_t specialFunc = ReadMemWordXv2(0x0102);
            if(specialFunc & 0x8)
            {
                SetPcXv2(0x80, SAFE_PC_ADDRESS);
                cntrl_sig_16bit();
                SetReg_16Bits(0x0501);
                IHIL_Tclk(1);
                addr_capture();

                specialFunc &= ~0x8;
                WriteMemWordXv2(0x0102, specialFunc);
            }
        }

        STREAM_put_bytes((uint8_t*)MyOut,8);

        return 0; // retrun status OK
    }
    else
    {
        return HALERR_UNDEFINED_ERROR; // return errer in case if syc fails
    }
}
/**
* \ingroup MODULMACROS
*
* \file SyncJtag_Conditional_SaveContext.c
*
* \brief <FILEBRIEF>
*/


#define MAX_TCE1 10
/**
  SyncJtag_Conditional_SaveContext
  inData:  <wdtAddr(16)> <wdtCtrl(16)>
  outData: <wdtCtrl(16)> <PC(32)> <SR(16)>
  wdtAddr: watchdog timer address
  wdtCtrl: watchdog timer control value
  PC:      program counter register (R0)
  SR:      status register (R2)
*/

//extern DeviceSettings deviceSettings;

//--------------------------------------------------------------------------
// STATUS_T clkTclkAndCheckDTC(void)
// \todo Günther: Seems to only apply to devices with the DTC bug, determine the effect
//       this code has on devices that do not have the DTC bug
int32_t clkTclkAndCheckDTC_hal_SyncJtag_Conditional_SaveContext(void)
{
#define MAX_DTC_CYCLE 10
  uint16_t cntrlSig;
  int16_t dtc_cycle_cnt = 0;
  int32_t timeOut =0;
  do
    {
      IHIL_Tclk(0);

      cntrl_sig_capture();
      cntrlSig = SetReg_16Bits(0);

      if ((dtc_cycle_cnt > 0) &&
          ((cntrlSig & CNTRL_SIG_CPU_HALT) == 0))
      {
        // DTC cycle completed, take over control again...
        cntrl_sig_16bit();
        SetReg_16Bits(CNTRL_SIG_TCE1 | CNTRL_SIG_CPU | CNTRL_SIG_TAGFUNCSAT);
      }
      if ((dtc_cycle_cnt == 0) &&
          ((cntrlSig & CNTRL_SIG_CPU_HALT) == CNTRL_SIG_CPU_HALT))
      {
        // DTC cycle requested, grant it...
        cntrl_sig_16bit();
        SetReg_16Bits(CNTRL_SIG_CPU_HALT | CNTRL_SIG_TCE1 |
                      CNTRL_SIG_CPU | CNTRL_SIG_TAGFUNCSAT);
        dtc_cycle_cnt++;
      }
      IHIL_Tclk(1);
      timeOut++;
    }
    while ((dtc_cycle_cnt < MAX_DTC_CYCLE) &&
           ((cntrlSig & CNTRL_SIG_CPU_HALT) == CNTRL_SIG_CPU_HALT)&& timeOut < 5000);

  if (dtc_cycle_cnt < MAX_DTC_CYCLE)
  {
    return (0);
  }
  else
  {
    return (-1);
  }
}

HAL_FUNCTION(_hal_SyncJtag_Conditional_SaveContext)
{
#define clkTclkAndCheckDTC  clkTclkAndCheckDTC_hal_SyncJtag_Conditional_SaveContext
  uint16_t i = 0, lOut = 0, ctl_sync = 0;
  int16_t MyOut[5] = {0};
  uint16_t address;
  uint16_t wdtVal;
  uint16_t statusReg = 0;
  uint16_t* syncWithRunVarAddress = 0;

  if(STREAM_get_word(&address) != 0)
  {
      return (HALERR_SYNC_JTAG_CONDITIONAL_NO_WDT_ADDRESS);
  }

  if(STREAM_get_word(&wdtVal) == -1)
  {
      return (HALERR_SYNC_JTAG_CONDITIONAL_NO_WDT_VALUE);
  }

  syncWithRunVarAddress = getTargetRunningVar();
  if(syncWithRunVarAddress)
  {
    *syncWithRunVarAddress = 0x0000;
  }

  IHIL_Tclk(1);  // Stability improvent: should be possible to remove this here, default state of TCLK should be one

  cntrl_sig_capture();

  if(!(SetReg_16Bits(0x0000) & CNTRL_SIG_TCE))
  {
   // If the JTAG and CPU are not already synchronized ...
    // Initiate Jtag and CPU synchronization. Read/Write is under CPU control. Source TCLK via TDI.
    // Do not effect bits used by DTC (CPU_HALT, MCLKON).
    cntrl_sig_high_byte();
    SetReg_8Bits((CNTRL_SIG_TAGFUNCSAT | CNTRL_SIG_TCE1 | CNTRL_SIG_CPU) >> 8);

    // A bug in first F43x and F44x silicon requires that Jtag synchronization be forced /* when the CPU is Off */.
    // Since the JTAG and CPU are not yet synchronized, the CPUOFF bit in the lower byte of the cntrlSig
    // register is not valid.
    // TCE eventually set indicates synchronization (and clocking via TCLK).
    ctl_sync =  SyncJtag();

    if(!ctl_sync)
    { // Synchronization failed!
        return (HALERR_SYNC_JTAG_CONDITIONAL_JTAG_TIMEOUT);
    }
  }// end of if(!(lOut & CNTRL_SIG_TCE))

  if(ctl_sync & CNTRL_SIG_CPU_HALT)
  {
      IHIL_Tclk(0);
      cntrl_sig_16bit();
      // Clear HALT. Read/Write is under CPU control. As a precaution, disable interrupts.
      SetReg_16Bits(CNTRL_SIG_TCE1 | CNTRL_SIG_CPU | CNTRL_SIG_TAGFUNCSAT);
      IHIL_Tclk(1);
  }
  else
  {
      cntrl_sig_16bit();
      // Clear HALT. Read/Write is under CPU control. As a precaution, disable interrupts.
      SetReg_16Bits(CNTRL_SIG_TCE1 | CNTRL_SIG_CPU | CNTRL_SIG_TAGFUNCSAT);
  }

  if (deviceSettings.assertBslValidBit)
  {
      // here we add bit de assert bit 7 in JTAG test reg to enalbe clocks again
      test_reg();
      lOut = SetReg_8Bits(0x00);
      lOut |= 0x80; //DE_ASSERT_BSL_VALID;
      test_reg();
      SetReg_8Bits(lOut); // Bit 7 is de asserted now
  }

   // step until next instruction load boundary if not being already there
  if(instrLoad() != 0)
  {
      return (HALERR_INSTRUCTION_BOUNDARY_ERROR);
  }

  // read MAB = PC here
  addr_capture();
  MyOut[1] = SetReg_16Bits(0x0000);
  MyOut[2] = 0; // High part for MSP430 devices is always 0

 // disable EEM and clear stop reaction
  eem_write_control();
  SetReg_16Bits(0x0003);
  SetReg_16Bits(0x0000);

    if (deviceSettings.clockControlType == GCC_EXTENDED)
    {
        eem_data_exchange();
        SetReg_16Bits(MX_GENCNTRL + MX_WRITE);               // write access to EEM General Control Register (MX_GENCNTRL)
        SetReg_16Bits(EMU_FEAT_EN | EMU_CLK_EN | CLEAR_STOP | EEM_EN);   // write into MX_GENCNTRL

        eem_data_exchange(); // Stability improvent: should be possible to remove this, required only once at the beginning
        SetReg_16Bits(MX_GENCNTRL + MX_WRITE);               // write access to EEM General Control Register (MX_GENCNTRL)
        SetReg_16Bits(EMU_FEAT_EN | EMU_CLK_EN);             // write into MX_GENCNTRL
    }
    if (deviceSettings.clockControlType == GCC_STANDARD_I)
    {
        eem_data_exchange();
        SetReg_16Bits(MX_GENCNTRL + MX_WRITE);  // write access to EEM General Control Register (MX_GENCNTRL)
        SetReg_16Bits(EMU_FEAT_EN);             // write into MX_GENCNTRL
    }

    if (deviceSettings.clockControlType != GCC_NONE)
    {
        if(deviceSettings.stopFLL)
        {
            uint16_t clkCntrl = 0;
            // read access to EEM General Clock Control Register (GCLKCTRL)
            eem_data_exchange();
            SetReg_16Bits(MX_GCLKCTRL + MX_READ);
            clkCntrl = SetReg_16Bits(0);
            // added UPSF: FE427 does regulate the FLL to the upper boarder
            // added the switch off and release of FLL (JTFLLO)
            clkCntrl |= 0x10;
            eem_data_exchange();
            SetReg_16Bits(MX_GCLKCTRL + MX_WRITE);
            SetReg_16Bits(clkCntrl);
        }
    }
  //-----------------------------------------------------------------------
  // Execute a dummy instruction (BIS #0,R4) to work around a problem in the F12x that can sometimes cause
  // the first TCLK after synchronization to be lost. If the TCLK is lost, try the cycle again.
  // Note: It is critical that the dummy instruction require exactly two cycles to execute. The version of BIS #
  // used does not use the constant generator, and so it requires two cycles.
  // The dummy instruction also provides a needed extra clock when the device is stopped by the Emex module and it is OFF.
  // The dummy instruction also possibly initiates processing of a pending interrupt.
 #define BIS_IMM_0_R4 0xd034
  data_16bit();
  IHIL_Tclk(1);  // Added for 2xx support
  SetReg_16Bits(BIS_IMM_0_R4);
  if(clkTclkAndCheckDTC() != 0)
  {
      return (HALERR_SYNC_JTAG_CONDITIONAL_JTAG_TIMEOUT);
  }

  cntrl_sig_capture();
  if(SetReg_16Bits(0x0000) & CNTRL_SIG_INSTRLOAD)  // Still on an instruction load boundary?
  {
      data_16bit();
      IHIL_Tclk(1);  // Added for 2xx support
      SetReg_16Bits(BIS_IMM_0_R4);
      if(clkTclkAndCheckDTC() != 0)
      {
        return (HALERR_SYNC_JTAG_CONDITIONAL_JTAG_TIMEOUT);
      }
  }
  data_16bit();
  IHIL_Tclk(1);  // Added for 2xx support
  SetReg_16Bits(0x0000);
  if(clkTclkAndCheckDTC() != 0)
  {
    return (HALERR_SYNC_JTAG_CONDITIONAL_JTAG_TIMEOUT);
  }
  // Advance to an instruction load boundary if an interrupt is detected.
  // The CPUOff bit will be cleared.
  // Basically, if there is an interrupt pending, the above dummy instruction
  // will have initialted its processing, and the CPU will not be on an
  // instruction load boundary following the dummy instruction.

  i = 0;
  cntrl_sig_capture();
  while(!(SetReg_16Bits(0) & CNTRL_SIG_INSTRLOAD) && (i < MAX_TCE1))
  {
      if(clkTclkAndCheckDTC() != 0)
      {
          return (HALERR_SYNC_JTAG_CONDITIONAL_JTAG_TIMEOUT);
      }
      i++;
  }

  if(i == MAX_TCE1)
  {
      return (HALERR_SYNC_JTAG_CONDITIONAL_JTAG_TIMEOUT);
  }

  // Read PC now!!! Only the NOP or BIS #0,R4 instruction above was clocked into the device
  // The PC value should now be (OriginalValue + 2)
  // read MAB = PC here
  addr_capture();
  MyOut[1] = SetReg_16Bits(0x0000) - 4;
  MyOut[2] = 0; // High part for MSP430 devices is always 0

  if(i == 0)
  { // An Interrupt was not detected
    //       lOut does not contain the content of the CNTRL_SIG register anymore at this point
    //       need to capture it again...different to DLLv3 sequence but don't expect any negative effect due to recapturing
      cntrl_sig_capture();
      if(SetReg_16Bits(0x0000) & CNTRL_SIG_CPU_OFF)
      {
          MyOut[1] += 2;

          data_16bit();
          IHIL_Tclk(1);               // Added for 2xx support
          SetReg_16Bits(0xC032);
          if(clkTclkAndCheckDTC() != 0)
          {
            return (HALERR_SYNC_JTAG_CONDITIONAL_JTAG_TIMEOUT);
          }

          data_16bit();
          IHIL_Tclk(1);               // Added for 2xx support \Günther: Why set TCLK 1 again?
          SetReg_16Bits(0x0010);
          if(clkTclkAndCheckDTC() != 0)
          {
            return (HALERR_SYNC_JTAG_CONDITIONAL_JTAG_TIMEOUT);
          }
          // DLLv2 preserve the CPUOff bit
          statusReg |= STATUS_REG_CPUOFF;
      }
  }
  else
  {
    addr_capture();
    MyOut[1] = SetReg_16Bits(0);
    MyOut[2] = 0; // High part for MSP430 devices is always 0
    MyOut[4] = 1; // Inform DLL about interrupt
  }

  // DLL v2: deviceHasDTCBug
  //     Configure the DTC so that it transfers after the present CPU instruction is complete (ADC10FETCH).
  //     Save and clear ADC10CTL0 and ADC10CTL1 to switch off DTC (Note: Order matters!).

  // Regain control of the CPU. Read/Write will be set, and source TCLK via TDI.
  cntrl_sig_16bit();
  SetReg_16Bits(CNTRL_SIG_TCE1 | CNTRL_SIG_READ | CNTRL_SIG_TAGFUNCSAT);

  // Test if we are on an instruction load boundary
  if(isInstrLoad() != 0)
  {
      return (HALERR_INSTRUCTION_BOUNDARY_ERROR);
  }

  // Hold Watchdog
  MyOut[0] = ReadMemWord (address); // safe WDT value
  wdtVal |= (MyOut[0] & 0xFF); // set original bits in addition to stop bit
  WriteMemWord(address, wdtVal);

  // set PC to a save address pointing to ROM to avoid RAM corruption on certain devices
  SetPc(ROM_ADDR);

  // read status register
  MyOut[3] = ReadCpuReg(2);
  MyOut[3] |= statusReg;   // compine with preserved CPUOFF bit setting

  // return output
  STREAM_put_bytes((uint8_t*)MyOut, sizeof(MyOut));

  return (0);
#undef clkTclkAndCheckDTC
}
/**
* \ingroup MODULMACROSX
*
* \file SyncJtag_Conditional_SaveContextX.c
*
* \brief <FILEBRIEF>
*/


#define MAX_TCE1 10
/**
  SyncJtag_Conditional_SaveContext
  inData:  <wdtAddr(16)> <wdtCtrl(16)>
  outData: <wdtCtrl(16)> <PC(32)> <SR(16)>
  wdtAddr: watchdog timer address
  wdtCtrl: watchdog timer control value
  PC:      program counter register (R0)
  SR:      status register (R2)
*/

//extern DeviceSettings deviceSettings;

//--------------------------------------------------------------------------
// STATUS_T clkTclkAndCheckDTC(void)
// \todo G?nther: Seems to only apply to devices with the DTC bug, determine the effect
//       this code has on devices that do not have the DTC bug
int32_t clkTclkAndCheckDTC_hal_SyncJtag_Conditional_SaveContextX(void)
{
#define MAX_DTC_CYCLE 10
  uint16_t cntrlSig;
  int16_t dtc_cycle_cnt = 0;
  int32_t timeOut= 0;


  do
    {
      IHIL_Tclk(0);

      cntrl_sig_capture();
      cntrlSig = SetReg_16Bits(0);

      if ((dtc_cycle_cnt > 0) &&
          ((cntrlSig & CNTRL_SIG_CPU_HALT) == 0))
      {
        // DTC cycle completed, take over control again...
        cntrl_sig_16bit();
        SetReg_16Bits(CNTRL_SIG_TCE1 | CNTRL_SIG_CPU | CNTRL_SIG_TAGFUNCSAT);
      }
      if ((dtc_cycle_cnt == 0) &&
          ((cntrlSig & CNTRL_SIG_CPU_HALT) == CNTRL_SIG_CPU_HALT))
      {
        // DTC cycle requested, grant it...
        cntrl_sig_16bit();
        SetReg_16Bits(CNTRL_SIG_CPU_HALT | CNTRL_SIG_TCE1 |
                      CNTRL_SIG_CPU | CNTRL_SIG_TAGFUNCSAT);
        dtc_cycle_cnt++;
      }
      IHIL_Tclk(1);
      timeOut++;
    }
    while ((dtc_cycle_cnt < MAX_DTC_CYCLE) &&
           ((cntrlSig & CNTRL_SIG_CPU_HALT) == CNTRL_SIG_CPU_HALT) && timeOut < 5000);

  if (dtc_cycle_cnt < MAX_DTC_CYCLE)
  {
    return (0);
  }
  else
  {
    return (-1);
  }
}


HAL_FUNCTION(_hal_SyncJtag_Conditional_SaveContextX)
{
#define clkTclkAndCheckDTC  clkTclkAndCheckDTC_hal_SyncJtag_Conditional_SaveContextX
  uint16_t ctl_sync = 0;
  uint16_t i = 0;
  int16_t MyOut[5] = {0};
  uint16_t address;
  uint16_t wdtVal;
  uint16_t statusReg = 0;
  uint16_t* syncWithRunVarAddress = 0;
  uint32_t lOut_long = 0;

  if(STREAM_get_word(&address) != 0)
  {
      return (HALERR_SYNC_JTAG_CONDITIONAL_NO_WDT_ADDRESS);
  }

  if(STREAM_get_word(&wdtVal) == -1)
  {
      return (HALERR_SYNC_JTAG_CONDITIONAL_NO_WDT_VALUE);
  }

  syncWithRunVarAddress = getTargetRunningVar();
  if(syncWithRunVarAddress)
  {
    *syncWithRunVarAddress = 0x0000;
  }

  // DLLv2 syncCPUAndCyclesAfterDebug()
  IHIL_Tclk(1);  // Stability improvement: should be possible to remove this here, default state of TCLK should be one

  cntrl_sig_capture();
  if(!(SetReg_16Bits(0x0000) & CNTRL_SIG_TCE))
  {
   // If the JTAG and CPU are not already synchronized ...
    // Initiate Jtag and CPU synchronization. Read/Write is under CPU control. Source TCLK via TDI.
    // Do not effect bits used by DTC (CPU_HALT, MCLKON).
    cntrl_sig_high_byte();
    SetReg_8Bits((CNTRL_SIG_TAGFUNCSAT | CNTRL_SIG_TCE1 | CNTRL_SIG_CPU) >> 8);

    // TCE eventually set indicates synchronization (and clocking via TCLK).
    ctl_sync =  SyncJtag();

    if(!ctl_sync)
    { // Synchronization failed!
        return (HALERR_SYNC_JTAG_CONDITIONAL_JTAG_TIMEOUT);
    }
  }// end of if(!(lOut & CNTRL_SIG_TCE))

  if(ctl_sync & CNTRL_SIG_CPU_HALT)
  {
      IHIL_Tclk(0);
      cntrl_sig_16bit();
      // Clear HALT. Read/Write is under CPU control. As a precaution, disable interrupts.
      SetReg_16Bits(CNTRL_SIG_TCE1 | CNTRL_SIG_CPU | CNTRL_SIG_TAGFUNCSAT);
      IHIL_Tclk(1);
  }
  else
  {
      cntrl_sig_16bit();
      // Clear HALT. Read/Write is under CPU control. As a precaution, disable interrupts.
      SetReg_16Bits(CNTRL_SIG_TCE1 | CNTRL_SIG_CPU | CNTRL_SIG_TAGFUNCSAT);
  }

   // step until next instruction load boundary if not being already there
  if(instrLoad() != 0)
  {
      return (HALERR_INSTRUCTION_BOUNDARY_ERROR);
  }

  // read MAB = PC here
  addr_capture();
  lOut_long = SetReg_20Bits(0x000);
  MyOut[1] = (uint16_t)(lOut_long & 0xFFFF);
  MyOut[2] = (uint16_t)(lOut_long >> 16);

  // DLLv2: Check if a breakpoint was hit
  // \todo Determine if this is needed

 // disable EEM and clear stop reaction
  eem_write_control();
  SetReg_16Bits(0x0003);
  SetReg_16Bits(0x0000);

  if (deviceSettings.clockControlType == GCC_EXTENDED)
  {
        eem_data_exchange32();
        SetReg_32Bits(MX_GENCNTRL + MX_WRITE);               // write access to EEM General Control Register (MX_GENCNTRL)
        SetReg_32Bits(EMU_FEAT_EN | EMU_CLK_EN | CLEAR_STOP | EEM_EN);   // write into MX_GENCNTRL

        eem_data_exchange32(); // Stability improvement: should be possible to remove this, required only once at the beginning
        SetReg_32Bits(MX_GENCNTRL + MX_WRITE);               // write access to EEM General Control Register (MX_GENCNTRL)
        SetReg_32Bits(EMU_FEAT_EN | EMU_CLK_EN);         // write into MX_GENCNTRL
  }

  //-----------------------------------------------------------------------
  data_16bit();
  IHIL_Tclk(1);               // added for 2xx support
  SetReg_16Bits(0x4303);
  IHIL_Tclk(0);
  data_capture();
  IHIL_Tclk(1);


  // Advance to an instruction load boundary if an interrupt is detected.
  // The CPUOff bit will be cleared.
  // Basically, if there is an interrupt pending, the above dummy instruction
  // will have initialted its processing, and the CPU will not be on an
  // instruction load boundary following the dummy instruction.

  // \G?nther: Looks a lot like clckToInstrLoad(), with the exception that clkTclkAndCheckDTC()
  //           is called continuously
  // \WLUT   : Correct;-)
  i = 0;
  cntrl_sig_capture();
  while(!(SetReg_16Bits(0) & CNTRL_SIG_INSTRLOAD) && (i < MAX_TCE1))
  {
      if(clkTclkAndCheckDTC() != 0)
      {
          return (HALERR_SYNC_JTAG_CONDITIONAL_JTAG_TIMEOUT);
      }
      i++;

      SetReg_16Bits(0);
  }

  if(i == MAX_TCE1)
  {
      return (HALERR_SYNC_JTAG_CONDITIONAL_JTAG_TIMEOUT);
  }

  // Read PC now!!! Only the NOP or BIS #0,R4 instruction above was clocked into the device
  // The PC value should now be (OriginalValue + 2)
  // read MAB = PC here
  addr_capture();
  lOut_long = SetReg_20Bits(0x0000);
  lOut_long -= 2;
  MyOut[1] = (uint16_t)(lOut_long & 0xFFFF);
  MyOut[2] = (uint16_t)(lOut_long >> 16);

  if(i == 0)
  { // An Interrupt was not detected
    // WLUT: lOut does not contain the content of the CNTRL_SIG register anymore at this point
    //       need to capture it again...different to DLLv3 sequence but don't expect any negative effect due to recapturing
      cntrl_sig_capture();
      if(SetReg_16Bits(0x0000) & CNTRL_SIG_CPU_OFF)
      {
          MyOut[1] += 2;

          data_16bit();
          IHIL_Tclk(1);               // Added for 2xx support
          SetReg_16Bits(0xC032);
          if(clkTclkAndCheckDTC() != 0)
          {
            return (HALERR_SYNC_JTAG_CONDITIONAL_JTAG_TIMEOUT);
          }

          data_16bit();
          IHIL_Tclk(1);               // Added for 2xx support \G?nther: Why set TCLK 1 again?
          SetReg_16Bits(0x0010);
          if(clkTclkAndCheckDTC() != 0)
          {
            return (HALERR_SYNC_JTAG_CONDITIONAL_JTAG_TIMEOUT);
          }
          // DLLv2 preserve the CPUOff bit
          statusReg |= STATUS_REG_CPUOFF;
      }
  }
  else
  {
    addr_capture();
    lOut_long = SetReg_20Bits(0);
    MyOut[1] = (uint16_t)(lOut_long & 0xFFFF);
    MyOut[2] = (uint16_t)(lOut_long >> 16);
    MyOut[4] = 1; // Inform DLL about interrupt
  }

  // DLL v2: deviceHasDTCBug
  //     Configure the DTC so that it transfers after the present CPU instruction is complete (ADC10FETCH).
  //     Save and clear ADC10CTL0 and ADC10CTL1 to switch off DTC (Note: Order matters!).

  // Regain control of the CPU. Read/Write will be set, and source TCLK via TDI.
  cntrl_sig_16bit();
  SetReg_16Bits(CNTRL_SIG_TCE1 | CNTRL_SIG_READ | CNTRL_SIG_TAGFUNCSAT);

                // Test if we are on an instruction load boundary
  if(isInstrLoad() != 0)
  {
      return (HALERR_INSTRUCTION_BOUNDARY_ERROR);
  }

  // Hold Watchdog
  MyOut[0] = ReadMemWordX(address); // safe WDT value
  wdtVal |= (MyOut[0] & 0x00FF); // set original bits in addition to stop bit
  WriteMemWordX(address, wdtVal);

  // set PC to a save address pointing to ROM to avoid RAM corruption on certain devices
  SetPcX(ROM_ADDR);

  // read status register
  MyOut[3] = ReadCpuRegX(2);
  MyOut[3] |= statusReg;   // compine with preserved CPUOFF bit setting

  // return output
  STREAM_put_bytes((uint8_t*)MyOut, sizeof(MyOut));

  return (0);
#undef clkTclkAndCheckDTC
}
/**
* \ingroup MODULMACROSXV2
*
* \file SyncJtag_Conditional_SaveContextXv2.c
*
* \brief <FILEBRIEF>
*/



#define SR             2
#define	EEM_STOPPED    0x0080

//extern uint16_t altRomAddressForCpuRead;

/**
  SyncJtag_Conditional_SaveContextXv2
  inData:  <wdtAddr(16)> <wdtCtrl(16)>
  outData: <wdtCtrl(16)> <PC(32)> <SR(16)>
  wdtAddr: watchdog timer address
  wdtCtrl: watchdog timer control value
  PC:      program counter register (R0)
  SR:      status register (R2)
*/

//extern DevicePowerSettings devicePowerSettings;

//! \todo EEM cycle counter, decide if it should be handled by host DLL!
HAL_FUNCTION(_hal_SyncJtag_Conditional_SaveContextXv2)
{
  uint8_t pipe_empty = 0;
  uint8_t cpuoff = 0;
  uint16_t irRequest = 0;
  const uint32_t MaxCyclesForSync = 10000; // must be defined, dependant on DMA (burst transfer)!!!
  uint32_t i = 0;
  uint16_t MyOut[5];
  uint16_t wdt_addr = 0;
  uint16_t wdt_value = 0;
  uint32_t TimeOut = 0;
  uint16_t* syncWithRunVarAddress = getTargetRunningVar();
  uint16_t lOut = 0;
  uint32_t lOut_long = 0;

//------------------------------------------------------------------------------
// Start note 822: Special handling ynchronize to JTAG
    if(syncWithRunVarAddress)
    {
        *syncWithRunVarAddress = 0x0000;
    }
    // -------------------------Power mode handling start ----------------------
    DisableLpmx5();
    // -------------------------Power mode handling end ------------------------

  // read out EEM control register...
  eem_read_control();
  // ... and check if device got already stopped by the EEM
  if (!(SetReg_16Bits(0x0000) & EEM_STOPPED))
  { // do this only if the device is NOT already stopped.
    // read out control signal register first
    cntrl_sig_capture();
    // check if CPUOFF bit is set
    if(!(SetReg_16Bits(0x0000) & CNTRL_SIG_CPUOFF))
    { // do the following only if the device is NOT in Low Power Mode
      uint32_t tbValue;
      uint32_t tbCntrl;
      uint32_t tbMask;
      uint32_t tbComb;
      uint32_t tbBreak;

      // Trigger Block 0 value register
      eem_data_exchange32();
      SetReg_32Bits(MBTRIGxVAL + READ + TB0);   // load address
      // shift in dummy 0
      tbValue = SetReg_32Bits(0);               // Trigger Block 0 control register
      SetReg_32Bits(MBTRIGxCTL + READ + TB0);   // load address
      // shift in dummy 0
      tbCntrl = SetReg_32Bits(0);               // Trigger Block 0 mask register
      SetReg_32Bits(MBTRIGxMSK + READ + TB0);   // load address
      // shift in dummy 0
      tbMask = SetReg_32Bits(0);
      // Trigger Block 0 combination register
      SetReg_32Bits(MBTRIGxCMB + READ + TB0);   // load address
      tbComb = SetReg_32Bits(0);                         // shift in dummy 0
      // Trigger Block 0 combination register
      SetReg_32Bits(BREAKREACT + READ);         // load address
      tbBreak = SetReg_32Bits(0);                         // shift in dummy 0

      // now configure a trigger on the next instruction fetch
      SetReg_32Bits(MBTRIGxCTL + WRITE + TB0);   // load address
      SetReg_32Bits(CMP_EQUAL + TRIG_0 + MAB);
      SetReg_32Bits(MBTRIGxMSK + WRITE + TB0);   // load address
      SetReg_32Bits(MASK_ALL);
      SetReg_32Bits(MBTRIGxCMB + WRITE + TB0);   // load address
      SetReg_32Bits(EN0);
      SetReg_32Bits(BREAKREACT + WRITE);         // load address
      SetReg_32Bits(EN0);

      // enable EEM to stop the device
      SetReg_32Bits(GENCLKCTRL + WRITE);         // load address
      SetReg_32Bits(MCLK_SEL0 + SMCLK_SEL0 + ACLK_SEL0 + STOP_MCLK + STOP_SMCLK + STOP_ACLK);
      eem_write_control();
      SetReg_16Bits(EMU_CLK_EN + CLEAR_STOP + EEM_EN);

      {
        int16_t lTimeout = 0;
        do
        {
          eem_read_control();
          lTimeout++;
        }
        while(!(SetReg_16Bits(0x0000) & EEM_STOPPED) && lTimeout < 3000);
      }
      // restore the setting of Trigger Block 0 previously stored
      // Trigger Block 0 value register
      eem_data_exchange32();
      SetReg_32Bits(MBTRIGxVAL + WRITE + TB0);   // load address
      SetReg_32Bits(tbValue);
      SetReg_32Bits(MBTRIGxCTL + WRITE + TB0);   // load address
      SetReg_32Bits(tbCntrl);
      SetReg_32Bits(MBTRIGxMSK + WRITE + TB0);   // load address
      SetReg_32Bits(tbMask);
      SetReg_32Bits(MBTRIGxCMB + WRITE + TB0);   // load address
      SetReg_32Bits(tbComb);
      SetReg_32Bits(BREAKREACT + WRITE);         // load address
      SetReg_32Bits(tbBreak);
    }
  }
// End: special handling note 822
//------------------------------------------------------------------------------

  // enable clock control before sync
  eem_write_control();
  SetReg_16Bits(EMU_CLK_EN + EEM_EN);

  // sync device to JTAG
  SyncJtagXv2();

  // reset CPU stop reaction - CPU is now under JTAG control
  // Note: does not work on F5438 due to note 772, State storage triggers twice on single stepping
  eem_write_control();
  SetReg_16Bits(EMU_CLK_EN + CLEAR_STOP + EEM_EN);
  SetReg_16Bits(EMU_CLK_EN + CLEAR_STOP);

  cntrl_sig_16bit();
  SetReg_16Bits(0x1501);
  // clock system into Full Emulation State now...
  // while checking control signals CPUSUSP (pipe_empty), CPUOFF and HALT

  cntrl_sig_capture();
  lOut = SetReg_16Bits(0);

  ///////////////////////////////////////////
  // Note 805: check if wait is set
  if((lOut & 0x8) == 0x8)
  {
    // wait until wait is end
    while((lOut & 0x8) == 0x8 && TimeOut++ < 30000)
    {
        IHIL_Tclk(0); // provide falling clock edge
        IHIL_Tclk(1); // provide rising clock edge
        cntrl_sig_capture();
        lOut = SetReg_16Bits(0);
    }
  }
  //Note 805 end: Florian, 21 Dec 2010
  ///////////////////////////////////////////

  cntrl_sig_capture();
  do
  {
    IHIL_Tclk(0); // provide falling clock edge
    // check control signals during clock low phase
    // shift out current control signal register value
    (SetReg_16Bits(0x000) & CNTRL_SIG_CPUSUSP) ? (pipe_empty = TRUE) : (pipe_empty = FALSE);
    IHIL_Tclk(1); // provide rising clock edge
    i++; // increment loop counter for braek condition
  }
  // break condition:
  //    pipe_empty = 1
  //    or an error occured (MaxCyclesForSync exeeded!!)
  while(!pipe_empty && (i < MaxCyclesForSync));

  //! \todo check error condition
  if(i >= MaxCyclesForSync)
  {
    ;
  }

  // the interrupts will be diabled now - JTAG takes over control of the control signals
  cntrl_sig_16bit();
  SetReg_16Bits(0x0501);

  // recall an interrupt request
  /**
   * End of  : i_Conditional
   */

  /**
   * Begin of: i_SaveContext
   */
  // provide 1 clock in order to have the data ready in the first transaction slot
  IHIL_TCLK();
  addr_capture();
  lOut_long = SetReg_20Bits(0);

  cntrl_sig_capture();
  lOut = SetReg_16Bits(0);
  // shift out current control signal register value
  if(lOut & CNTRL_SIG_CPUOFF)
  {
    cpuoff = TRUE;
  }
  else
  {
    cpuoff = FALSE;
  }
  irRequest = (lOut & 0x4);

  // adjust program counter according to control signals
  if(cpuoff)
  {
    lOut_long -= 2;
  }
  else
  {
    lOut_long -= 4;
  }
  {
      uint32_t  tempPc = lOut_long;
      /********************************************************/
      /* Note 1495, 1637 special handling for program counter */
      /********************************************************/
      if((tempPc & 0xFFFF) == 0xFFFE)
      {
        uint32_t  tempPc = 0;
        uint16_t read = ReadMemWordXv2(0xFFFE);
        tempPc = (0x00000000 | read);

        MyOut[1] = (uint16_t)(tempPc & 0xFFFF);
        MyOut[2] = (uint16_t)(tempPc >>16);
      }
      /* End note 1495, 1637 */
      else
      {
        MyOut[1] = (uint16_t)(tempPc & 0xFFFF);
        MyOut[2] = (uint16_t)(tempPc >>16);
      }
  }
  /**
  * End of  : i_SaveContext
  */

  // set EEM FEATURE enable now!!!
  eem_write_control();
  SetReg_16Bits(EMU_FEAT_EN + EMU_CLK_EN + CLEAR_STOP);

  // check for Init State
  cntrl_sig_capture();
  SetReg_16Bits(0);

  // hold Watchdog Timer
  STREAM_get_word(&wdt_addr);
  STREAM_get_word(&wdt_value);

  MyOut[0] = ReadMemWordXv2(wdt_addr);
  wdt_value |= (MyOut[0] & 0xFF); // set original bits in addition to stop bit
  WriteMemWordXv2(wdt_addr, wdt_value);

  {
    uint16_t Mova;
    uint16_t Rx_l;
    uint32_t  Rx;
    // save Status Register content
    Mova  = 0x0060;
    Mova += (SR<<8) & 0x0F00;
    Rx = ReadCpuRegXv2(Mova);
    MyOut[3] = (uint16_t)Rx;
    // reset Status Register to 0 - needs to be restored again for release
    Mova  = 0x0080;
    Mova += (SR & 0x000F);
    Rx_l = (((uint16_t)Rx) & 0xFFE7); // clear CPUOFF/GIE bit
    WriteCpuRegXv2(Mova, Rx_l);
  }
  MyOut[4] = irRequest;

  STREAM_put_bytes((uint8_t*)MyOut,10);
  return 0;
}
/**
* \ingroup MODULMACROSXV2
*
* \file UnlockC092.c
*
* \brief <FILEBRIEF>
*/


HAL_FUNCTION(_hal_UnlockC092)
{
    uint16_t passwordLenght= 0x0 ;
    uint16_t i = 0x0;
    uint16_t id = 0;
    //StreamSafe stream_tmp;
    uint16_t Password[4] = {0};

     // steam get password length
     if(STREAM_get_word(&passwordLenght) != 0)
     {
            return(HALERR_UNLOCK_NO_PASSWORD_LENGTH);
     }
     // steam get password itself
     for(i =0; i < passwordLenght; i++)
     {
        if(STREAM_get_word(&Password[i]) < 0)
        {
            return(HALERR_UNLOCK_INVALID_PASSWORD_LENGTH);
        }
     }
    //--------------------------------------------------------------------------
    //phase 0 if device was in LPMx5
    //--------------------------------------------------------------------------

     IHIL_SetProtocol(0); // C092 can just operrate in SBW4 Mode
#if defined(eZ_FET) || defined(MSP_FET)
    {
        // MSPFETSim: not sure what this craziness is, but its result is calling `_hil_getEdtDistinct`
//        HilInitGetEdtDistinctFunc hilEdtDis = (HilInitGetEdtDistinctFunc)0x1880;
//        hilEdtDis(&_edt_Distinct_Methods_HAL);
        _hil_getEdtDistinct(&_edt_Distinct_Methods_HAL);
    }
#endif

    //--------------------------------------------------------------------------
    //phase 1 of device entry using a user password
    //--------------------------------------------------------------------------
     // Apply  4wire/SBW entry Sequence & holt Reset low
     IHIL_SetReset(0);
     IHIL_Open(RSTHIGH);
     IHIL_TapReset();
     IHIL_SetReset(0);
     IHIL_CheckJtagFuse();

     // check for Timeout issue during mailbox request
     if(i_WriteJmbIn32(Password[0], Password[1]) == 1)
     {
        return (HALERR_JTAG_PASSWORD_WRONG);
     }
     if(passwordLenght > 2)
     {
       // now feed in last 2 words of password
        IHIL_SetReset(1);
        IHIL_Delay_1ms(200);
        if(i_WriteJmbIn32(Password[2], Password[3]) == 1)
        {
            return (HALERR_JTAG_PASSWORD_WRONG);
        }
     }
     id = cntrl_sig_capture();
     if(id != 0x95)
     {
        return (HALERR_JTAG_PASSWORD_WRONG);
     }
     IHIL_SetReset(1);
     return(0);
}
/**
* \ingroup MODULMACROSXV2
*
* \file UnlockDeviceXv2.c
*
* \brief <FILEBRIEF>
*/


HAL_FUNCTION(_hal_UnlockDeviceXv2)
{
    uint16_t passwordLenght= 0x0 ;
    uint16_t i = 0x0;
    uint16_t Password[60] = {0};
    uint16_t protocol;


    // get the protocol
    if(STREAM_get_word(&protocol) < 0)
    {
        return (HALERR_START_JTAG_NO_PROTOCOL);
    }
     // steam get password length
     if(STREAM_get_word(&passwordLenght) != 0)
     {
        return (HALERR_UNLOCK_NO_PASSWORD_LENGTH);
     }
     // steam get password itself
     for(i =0; i < passwordLenght; i++)
     {
        if(STREAM_get_word(&Password[i]) < 0)
        {
            return (HALERR_UNLOCK_INVALID_PASSWORD_LENGTH);
        }
     }
    //--------------------------------------------------------------------------
    //phase 0 if device was in LPMx5 or camping in an endless loop
    //--------------------------------------------------------------------------

     IHIL_SetProtocol(protocol); // this is jsut for debug must be changed
#if defined(eZ_FET) || defined(MSP_FET)
    {
        // MSPFETSim: not sure what this craziness is, but its result is calling `_hil_getEdtDistinct`
//        HilInitGetEdtDistinctFunc hilEdtDis = (HilInitGetEdtDistinctFunc)0x1880;
//        hilEdtDis(&_edt_Distinct_Methods_HAL);
        _hil_getEdtDistinct(&_edt_Distinct_Methods_HAL);
    }
#endif
     IHIL_Open(RSTHIGH);
     IHIL_TapReset();

     IHIL_Close();
    //--------------------------------------------------------------------------
    //phase 1 of device entry using a user password
    //--------------------------------------------------------------------------
     // Apply  4wire/SBW entry Sequence & holt Reset low
     IHIL_Open(RSTLOW);
     // reset TAP state machine -> Run-Test/Idle
     IHIL_TapReset();
     // start JTAG mailbox & feed in password exchange request
     // check for Timeout issue during mailbox request
     if(i_WriteJmbIn32(MAGIC_PATTERN , DR_JMB_PASSWORD_EXCHANGE_REQUEST) == 1)
     {
        return (HALERR_JTAG_PASSWORD_WRONG);
     }

     //-------------------------------------------------------------------------
     //phase 2 of device entry using a user password
     //-------------------------------------------------------------------------
     // Apply again 4wire/SBW entry Sequence & Reset high
     IHIL_Open(RSTHIGH);
     // Reset TAP state machine -> Run-Test/Idle
     IHIL_TapReset();
     i = 0x0;
     // start JTAG mailbox & feed in password & make sure that you are faster
     // then 1.2 seconds if not the bootcode will kick you out & Use 16Bit JBM mode
     while(i < passwordLenght)
     {
        // check for Timeout becuase of wong passord which was shifted in
        if(i_WriteJmbIn(Password[i]) == 1)
        {
            return (HALERR_JTAG_PASSWORD_WRONG);
        }
        i++;
    }

    return (0);
}
/**
* \ingroup MODULMACROSXV2
*
* \file LeaSyncConditional.c
*
* \brief <FILEBRIEF>
*/


#define UUPSRIS         (0x0EC4)
#define STPBYDB         (0x0004)

HAL_FUNCTION(_hal_UssSyncConditional)
{
    uint16_t tclkCycles = 200;
    uint32_t baseAddress = 0;

    STREAM_get_long(&baseAddress);

    //Read UUPS_CTL regsier and check for USS_Busy bit
    uint16_t ussReg = ReadMemWordXv2(UUPSRIS);
    if(!(ussReg & STPBYDB))
    {
        STREAM_put_word(ussReg & STPBYDB);
        return 0;
    }
    // apply TCLK cycles
    while(tclkCycles-- > 0)
    {
        IHIL_TCLK();
    }

    IHIL_Tclk(1);

    STREAM_put_word(0);
    return 0;
}
/*
 * \ingroup MODULMACROS
 *
 * \file WaitForDebugHaltArm.c
 *
 * \brief <FILEBRIEF>
 */


/**
  WaitForDebugHalt
  Test the DHCSR register for specific bits and
  return if one of them is set.
  This is best used with the ExecLoop command type.
  inData:  <Mask(16)>
  outData: <Ctrl(16)>
          Mask: the bits to look for
          Ctrl: the read DHCSR register
*/
//#if defined(MSP430_UIF) || defined(MSP_FET)
//extern ARMConfigSettings armConfigSettings;
//#endif

int16_t checkDhcsrRegForBbHit()
{
    uint32_t dhcsrValue = 0;
    uint16_t* syncWithRunVarAddress = getTargetRunningVar();

    if(syncWithRunVarAddress)
    {
         if(!(*syncWithRunVarAddress))
         {
            return 2;
         }
    }

    if(IHIL_Write_Read_Mem_Ap(0, DHCSR, &dhcsrValue, READ) == -1)
    {
        return HALERR_UNDEFINED_ERROR;
    }
    if(dhcsrValue & S_HALT)
    {

        STREAM_put_long(dhcsrValue);
        if(syncWithRunVarAddress)
        {
            *syncWithRunVarAddress = 0x0000;
        }
        return 1;
    }
    return 2;
}


HAL_FUNCTION(_hal_WaitForDebugHaltArm)
{
    int16_t RetState = HALERR_UNDEFINED_ERROR;
#if defined(MSP430_UIF) || defined(MSP_FET)
    uint32_t val = 0;
    if(armConfigSettings.ulpDebug)
    {
        IHIL_Write_Read_Dp(DP_CTRL_STAT, &val, READ);

        if(val & DP_CTRL_STAT_CDBGPWRUPACK )
        {
            if( powerUpArm() != 0)
            {
                return RetState = 2;
            }
            if(checkWakeup() != 0)
            {
                return RetState = 2;
            }
            RetState =  checkDhcsrRegForBbHit();
        }
    }
    else // high power force
    {
        RetState = checkDhcsrRegForBbHit();
    }
#endif
    return RetState;
}
/**
* \ingroup MODULMACROS
*
* \file WaitForEem.c
*
* \brief <FILEBRIEF>
*/


/**
  WaitForEem
  Test the EEM General Debug Control (GENCTRL) register for specific bits and
  return if one of them is set.
  This is best used with the ExecLoop command type.
  inData:  <eemCtrlMask(16)>
  outData: <eemCtrl(16)>
          eemCtrlMask: the bits to look for
          eemCtrl: the read GENCTRL EEM register
*/

HAL_FUNCTION(_hal_WaitForEem)
{
    int16_t RetState = HALERR_UNDEFINED_ERROR;
    uint32_t lMask = 0;
    uint16_t sGenCtrl = 0, lOut = 0;
    uint16_t eventMask = 0;
    uint16_t* syncWithRunVarAddress = 0;

    STREAM_get_word((uint16_t*)&lMask);

    syncWithRunVarAddress = getTargetRunningVar();
    if(syncWithRunVarAddress)
    {
         if(!(*syncWithRunVarAddress))
         {
            return 2;
         }
    }
    else
    {
        return -1;
    }

    if(!jtagIdIsValid(cntrl_sig_capture()))
    {
        return 2;
    }

    // poll for bits in EEM GENCTRL register
    eem_read_control();
    lOut = SetReg_16Bits(0x0000);

    if(lOut & lMask)
    {
        if(syncWithRunVarAddress)
        {
            *syncWithRunVarAddress = 0x0000;
        }
        sGenCtrl |= lOut;
        eventMask |= BP_HIT_FLAG;
        STREAM_put_word(eventMask);
        STREAM_put_word(sGenCtrl);
        RetState = 1;
    }
    else
    {
        RetState = 2;
    }

    return RetState;
}
/**
* \ingroup MODULMACROS
*
* \file WaitForStorage.c
*
* \brief Check the state storage register for storage events and return storage * data on change
*/


uint16_t lastTraceWritePos = 0;

/**
  WaitForStorage
  Check the state storage register for storage events and return storage data on change.
  This is best used with the ExecLoop command type.
*/

struct {
    uint16_t noChangeSince;
} _hal_WaitForStorage_staticVars = {};

HAL_FUNCTION(_hal_WaitForStorage)
{
    STATIC_VARS_START(_hal_WaitForStorage);
    DECL_STATIC_VAR(noChangeSince);
    
    int16_t RetState = 2;
    uint16_t sStStorCtrl = 0;

    if(!jtagIdIsValid(cntrl_sig_capture()))
    {
        return 2;
    }
    eem_data_exchange();
    SetReg_16Bits(0x9F);
    sStStorCtrl = SetReg_16Bits(0);

    //Storage written bit
    if (sStStorCtrl & 0x100)
    {
        //Variable watch mode
        if ( (sStStorCtrl & 0x6) == 0x4 )
        {
            uint16_t readPos = 0;

            STREAM_put_word(VARIABLE_WATCH_FLAG);

            //Reset storage
            SetReg_16Bits(0x9E);
            SetReg_16Bits(sStStorCtrl | 0x40);

            for (readPos = 0; readPos < 8; ++readPos)
            {
                //Slice bits autoincrement on reading

                SetReg_16Bits(0x9A);
                SetReg_16Bits(readPos << 2);

                //Read MAB value
                SetReg_16Bits(0x9D);
                STREAM_put_long(SetReg_16Bits(0));
                //Read MDB value

                SetReg_16Bits(0x9D);
                STREAM_put_word(SetReg_16Bits(0));
            }

            RetState = 1;
        }

        //Trace mode
        else
        {
            int16_t writePos = 0;
            int16_t newEntries = 0;

            SetReg_16Bits(0x9B);

            writePos = SetReg_16Bits(0) >> 10;

            //store until full bit
            if (sStStorCtrl & 0x8)
            {
                //storage full bit (or writePos = 0, if written after polling storage control)
                if ((sStStorCtrl & 0x200) || (writePos == 0))
                {
                    //Disable and reset bits when full to avoid further events
                    //Enable bit will be set again with next reset from DLL
                    SetReg_16Bits(0x9E);
                    SetReg_16Bits((sStStorCtrl | 0x40) & ~0x1);

                    newEntries = 8 - lastTraceWritePos;
                }
                else if (writePos != lastTraceWritePos)
                {
                    newEntries = writePos - lastTraceWritePos;
                }
            }
            //Store continuously (only history mode: stop on trigger)
            else
            {
                if (writePos != lastTraceWritePos)
                {
                    noChangeSince = 0;
                }
                else if (++noChangeSince == 20)
                {
                    newEntries = (sStStorCtrl & 0x200) ? 8 : writePos;
                }
            }

            lastTraceWritePos = writePos;

            if (newEntries > 0)
            {
                uint16_t readPos = (writePos - newEntries) & 0x7;

                STREAM_put_word(STATE_STORAGE_FLAG);
                STREAM_put_word(newEntries);

                do
                {
                    //Slice bits autoincrement on reading

                    SetReg_16Bits(0x9A);
                    SetReg_16Bits(readPos << 2);

                    //Read MAB value
                    SetReg_16Bits(0x9D);
                    STREAM_put_long(SetReg_16Bits(0));

                    //Read MDB value
                    SetReg_16Bits(0x9D);
                    STREAM_put_word(SetReg_16Bits(0));

                    //Read Ctrl value
                    SetReg_16Bits(0x9D);
                    STREAM_put_word(SetReg_16Bits(0));

                    if (++readPos > 7)
                    {
                        readPos = 0;
                    }

                } while (readPos != writePos);

                RetState = 1;
            }
        }
    }

    return RetState;
}
/**
* \ingroup MODULMACROS
*
* \file WaitForStorageX.c
*
* \brief Check the state storage register for storage events and return storage * data on change
*/


//extern uint16_t lastTraceWritePos;

/**
  WaitForStorageX
  Check the state storage register for storage events and return storage data on change.
  This is best used with the ExecLoop command type.
*/

struct {
    uint16_t noChangeSince;
} _hal_WaitForStorageX_staticVars = {};

HAL_FUNCTION(_hal_WaitForStorageX)
{
    STATIC_VARS_START(_hal_WaitForStorageX);
    DECL_STATIC_VAR(noChangeSince);
    
    int16_t RetState = 2;
    uint16_t sStStorCtrl = 0;

    if(!jtagIdIsValid(cntrl_sig_capture()))
    {
        return 2;
    }

    eem_data_exchange32();
    SetReg_32Bits(0x9F);

    sStStorCtrl = (uint16_t)SetReg_32Bits(0);

    //Storage written bit
    if (sStStorCtrl & 0x100)
    {
        //Variable watch mode
        if ( (sStStorCtrl & 0x6) == 0x4 )
        {
            uint16_t readPos = 0;

            STREAM_put_word(VARIABLE_WATCH_FLAG);

            //Reset storage
            SetReg_32Bits(0x9E);
            SetReg_32Bits(sStStorCtrl | 0x40);

            for (readPos = 0; readPos < 8; ++readPos)
            {
                //Slice bits autoincrement on reading

                SetReg_32Bits(0x9A);
                SetReg_32Bits(readPos << 2);

                //Read MAB value
                SetReg_32Bits(0x9D);
                STREAM_put_long(SetReg_32Bits(0));

                //Read MDB value
                SetReg_32Bits(0x9D);
                STREAM_put_word(SetReg_32Bits(0) & 0xFFFF);
            }

            RetState = 1;
        }

        //Trace mode
        else
        {
            int16_t writePos = 0;
            int16_t newEntries = 0;

            SetReg_32Bits(0x9B);
            writePos = (int16_t)(SetReg_32Bits(0) >> 10);

            //store until full bit
            if (sStStorCtrl & 0x8)
            {
                //storage full bit (or writePos = 0, if written after polling storage control)
                if ((sStStorCtrl & 0x200) || (writePos == 0))
                {
                    //Disable and reset bits when full to avoid further events
                    //Enable bit will be set again with next reset from DLL
                    SetReg_32Bits(0x9E);
                    SetReg_32Bits((sStStorCtrl | 0x40) & ~0x1);

                    newEntries = 8 - lastTraceWritePos;
                }
                else if (writePos != lastTraceWritePos)
                {
                    newEntries = writePos - lastTraceWritePos;
                }
            }
            //Store continuously (only history mode: stop on trigger)
            else
            {
                if (writePos != lastTraceWritePos)
                {
                    noChangeSince = 0;
                }
                else if (++noChangeSince == 20)
                {
                    newEntries = (sStStorCtrl & 0x200) ? 8 : writePos;
                }
            }

            lastTraceWritePos = writePos;

            if (newEntries > 0)
            {
                uint16_t readPos = (writePos - newEntries) & 0x7;

                STREAM_put_word(STATE_STORAGE_FLAG);
                STREAM_put_word(newEntries);

                do
                {
                    //Slice bits autoincrement on reading

                    SetReg_32Bits(0x9A);
                    SetReg_32Bits(readPos << 2);

                    //Read MAB value
                    SetReg_32Bits(0x9D);
                    STREAM_put_long(SetReg_32Bits(0));

                    //Read MDB value
                    SetReg_32Bits(0x9D);
                    STREAM_put_word(SetReg_32Bits(0) & 0xFFFF);

                    //Read Ctrl value
                    SetReg_32Bits(0x9D);
                    STREAM_put_word(SetReg_32Bits(0) & 0xFFFF);

                    if (++readPos > 7)
                        readPos = 0;

                } while (readPos != writePos);

                RetState = 1;
            }
        }
    }

    return RetState;
}
/**
* \ingroup MODULMACROS
*
* \file WriteAllCpuRegs.c
*
* \brief Write CPU register values, except R0 and R2
*/


/**
  WriteAllCpuRegsXv2
  Write CPU register values, except R0 and R2. This function is for 20bit CPUs.
  inData:  <SP(32)> <Rn(32)>{12}
  outData: -
  SP: stack pointer register (R1)
  Rn: registers R4-R15
*/

//! \todo change API from 16 to 32 bit to be compatible to the X version
//        of this macro.
//! \todo function mapping in host DLL, currently remapped to WriteAllCpuRegsXv2
HAL_FUNCTION(_hal_WriteAllCpuRegs)
{
    uint16_t Registers;
    uint16_t  Rx;
    int16_t tmp;
    
    for (Registers = 1; Registers < 16; Registers++)
    {
        if(Registers == 2)
        {
            Registers += 2;
        }
        tmp = STREAM_get_word(&Rx);
        if(((Registers != 15) && (tmp != 0)) || ((Registers == 15) && (tmp < 0)))
        {
            return HALERR_WRITE_ALL_CPU_REGISTERS_STREAM;
        }
        WriteCpuReg(Registers, Rx);
    }    
    return 0;
}


/**
  WriteAllCpuRegsMSP432
  Write CPU register values
  inData: R0-R12, SP, LR, PC, SR, XPRS, SpecialRegs, MSP_SP, PSP_SP
  outData: -
*/
//#if defined(MSP430_UIF) || defined(MSP_FET)
//extern ARMConfigSettings armConfigSettings;
//#endif

int16_t writeCpuRegsArm(uint32_t data, uint32_t Rx)
{
    uint8_t retry = MAX_RETRY;
    uint32_t returnVal = 0;
    IHIL_Write_Read_Mem_Ap(0, DCRDR, &data, WRITE); // Write value
    data = Rx | REG_WnR;
    IHIL_Write_Read_Mem_Ap(0, DCRSR, &data, WRITE); // Request Write

    // check if wirte was sucessfully
    do
    {
        IHIL_Write_Read_Mem_Ap(0, DHCSR, &returnVal, READ);
    } while(--retry && !(returnVal & S_REGRDY));

    if(!retry)
    {
        return 0;
    }
    return 1;
}

HAL_FUNCTION(_hal_WriteAllCpuRegsArm)
{
#if defined(MSP430_UIF) || defined(MSP_FET)
    uint32_t data = 0;
    // Read the General Purpose registers.
    for (uint32_t Rx = 0; Rx < 16; ++Rx)
    {
        STREAM_get_long(&data);
        if(!writeCpuRegsArm(data, Rx))
        {
            return HALERR_UNDEFINED_ERROR;
        }
    }
    STREAM_get_long(&data);
    if(!writeCpuRegsArm(data, xPSR_REG))
    {
        return HALERR_UNDEFINED_ERROR;
    }

    STREAM_get_long(&data);
    if(!writeCpuRegsArm(data, spec_REG))
    {
        return HALERR_UNDEFINED_ERROR;
    }
    STREAM_get_long(&data);
    if(!writeCpuRegsArm(data, MSP_SP))
    {
        return HALERR_UNDEFINED_ERROR;
    }
    STREAM_get_long(&data);
    if(!writeCpuRegsArm(data, PSP_SP))
    {
        return HALERR_UNDEFINED_ERROR;
    }
#endif
    return 0;
}/**
* \ingroup MODULMACROSX
*
* \file WriteAllCpuRegsX.c
*
* \brief Write CPU register values, except R0 and R2
*/


HAL_FUNCTION(_hal_WriteAllCpuRegsX)
{
    uint16_t Registers;
    uint32_t  Rx = 0;
    int16_t tmp;
    
    for (Registers = 1; Registers < 16; Registers++)
    {
        if(Registers == 2)
        {
            Registers += 2;
        }
        STREAM_get_byte((uint8_t*)&Rx);
        STREAM_get_byte((uint8_t*)&Rx+1);
        tmp = STREAM_get_byte((uint8_t*)&Rx+2);

        if(((Registers != 15) && (tmp != 0)) || ((Registers == 15) && (tmp < 0)))
        {
            return HALERR_WRITE_ALL_CPU_REGISTERS_STREAM;
        }
        WriteCpuRegX(Registers, Rx);
    }   
    return 0;
}

/**
* \ingroup MODULMACROSXV2
*
* \file WriteAllCpuRegsXv2.c
*
* \brief Write CPU register values, except R0 and R2
*/

/**
  WriteAllCpuRegsXv2
  Write CPU register values, except R0 and R2. This function is for 20bit CPUs.
  inData:  <SP(24)> <Rn(24)>{12}
  outData: -
  SP: stack pointer register (R1)
  Rn: registers R4-R15
*/

HAL_FUNCTION(_hal_WriteAllCpuRegsXv2)
{
  uint16_t Registers;
  uint32_t  Rx;
  uint16_t Mova;
  uint16_t Rx_l;
  
  for (Registers = 1; Registers < 16; Registers++)
  {
    if(Registers == 2)
    {
      Registers += 2;
    }
    STREAM_get_byte((uint8_t*)&Rx);
    STREAM_get_byte((uint8_t*)&Rx+1);
    STREAM_get_byte((uint8_t*)&Rx+2);
    *((uint8_t*)&Rx+3) = 0;
    Mova  = 0x0080;
    Mova += (uint16_t)((Rx>>8) & 0x00000F00);
    Mova += (Registers & 0x000F);
    Rx_l  = (uint16_t)Rx;
    WriteCpuRegXv2(Mova, Rx_l);
  }

  return 0;
}

/**
* \ingroup MODULMACROSXV2
*
* \file WriteFramQuickXv2.c
*
* \brief <FILEBRIEF>
*/

/**
  WriteMemBytesXv2
  Write bytes to a memory mapped location.
  inData:  <addr(32)> <length(32)> <data(8)>{*}
  outData: -
  addr: the address to start writing to
  length: number of bytes to write
  data: data to write to the given location
*/

struct {
    uint32_t lLen;
    uint32_t Addr;
    uint16_t Mova;
    uint16_t ret_len;
    uint16_t id;
} _hal_WriteFramQuickXv2_staticVars = {};

HAL_FUNCTION(_hal_WriteFramQuickXv2)
{
    STATIC_VARS_START(_hal_WriteFramQuickXv2);
    DECL_STATIC_VAR(lLen);
    DECL_STATIC_VAR(Addr);
    DECL_STATIC_VAR(Mova);
    DECL_STATIC_VAR(ret_len);
    DECL_STATIC_VAR(id);
    
    int16_t ret_value = 0;
    uint16_t *pBuf;
    uint16_t sizeOfBuf;
    // fist message--------------------------------------------
    if(flags & MESSAGE_NEW_MSG)
    {
        if(STREAM_get_long(&Addr) != 0)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_ADDRESS);
        }
        // get length ot be flashed
        if(STREAM_get_long(&lLen) != 0)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_LENGTH);
        }

        id = cntrl_sig_capture();

        // i_SetPcRel
        Mova  = 0x0080;
        Mova += (uint16_t)((Addr>>8) & 0x00000F00);
        Addr  = (uint16_t)((Addr & 0xFFFF));

        SetPcXv2(Mova, Addr);

        cntrl_sig_16bit();
        SetReg_16Bits(0x0500);
        IHIL_Tclk(1);

        data_quick();
    }
    if(!(flags & MESSAGE_LAST_MSG ))
    {
      STREAM_out_change_type(RESPTYP_ACKNOWLEDGE);
      STREAM_flush();
    }
    
    // not first and not last message
    STREAM_get_buffer((void **)&pBuf,&sizeOfBuf);
    for(; lLen && sizeOfBuf; lLen--)
    {
         IHIL_Tclk(1);
         SetReg_16Bits(*pBuf++);
         sizeOfBuf -= 2;
         IHIL_Tclk(0);
    }
    
    // last message
    if(flags & MESSAGE_LAST_MSG )
    {
        cntrl_sig_16bit();
        SetReg_16Bits(0x0501);
        IHIL_Tclk(1);
        if(id == JTAGVERSION91 || id == JTAGVERSION99 || id == JTAGVERSION98)
        {
            SetPcXv2(0x80, SAFE_PC_ADDRESS); // Set PC to "safe" JMP $ address
            cntrl_sig_16bit();
            SetReg_16Bits(0x0501);
            IHIL_Tclk(1);
            addr_capture();
        }

        STREAM_put_word(ret_len);
    }
    else if(lLen != 0)
    {
        STREAM_out_change_type(RESPTYP_ACKNOWLEDGE);
        ret_value = 0;
    }
    else
    {
        cntrl_sig_16bit();
        SetReg_16Bits(0x0501);
        IHIL_Tclk(1);
        if(id == JTAGVERSION91 || id == JTAGVERSION99 || id == JTAGVERSION98)
        {
            SetPcXv2(0x80, SAFE_PC_ADDRESS); // Set PC to "safe" JMP $ address
            cntrl_sig_16bit();
            SetReg_16Bits(0x0501);
            IHIL_Tclk(1);
            addr_capture();
        }

        ret_value = HALERR_EXECUTE_FUNCLET_EXECUTION_ERROR;
    }
    return(ret_value);
}
/**
* \ingroup MODULMACROS
*
* \file WriteMemBytes.c
*
* \brief Write bytes to a memory mapped location
*/



/**
  WriteMemBytes
  Write bytes to a memory mapped location.
  inData:  <addr(32)> <length(32)> <data(8)>{*}
  outData: -
  addr: the address to start writing to
  length: number of bytes to write
  data: data to write to the given location
*/

struct {
    uint32_t lLen;
    uint32_t lAddr;
} _hal_WriteMemBytes_staticVars = {};

HAL_FUNCTION(_hal_WriteMemBytes)
{
    STATIC_VARS_START(_hal_WriteMemBytes);
    DECL_STATIC_VAR(lLen);
    DECL_STATIC_VAR(lAddr);
    
    int16_t ret_value = 0;
    uint8_t value;

    if(flags & MESSAGE_NEW_MSG)
    {
        if(STREAM_get_long(&lAddr) != 0)
        {
            return HALERR_WRITE_MEM_WORD_NO_RAM_ADDRESS;
        }
        if(STREAM_get_long(&lLen) == -1)
        {
            return HALERR_WRITE_MEM_WORD_NO_RAM_SIZE;
        }
        halt_cpu();
        IHIL_Tclk(0);
        cntrl_sig_16bit();
        SetReg_16Bits(0x2418);
    }

    lLen *= 2; //DLL always sends size in word
    for(; lLen && (ret_value == 0); lLen--)
    {
        ret_value = STREAM_get_byte(&value);
        addr_16bit();
        SetReg_16Bits((uint16_t)lAddr);
        data_to_addr();
        SetReg_8Bits(value);
        IHIL_Tclk(1);
        IHIL_Tclk(0);
        lAddr += 1;
    }

    if(flags & MESSAGE_LAST_MSG)
    {
        release_cpu();
    }
    else if(ret_value == 1)
    {
        STREAM_out_change_type(RESPTYP_ACKNOWLEDGE);
        ret_value = 0;
    }
    else
    {
        ret_value = HALERR_WRITE_MEM_WORD_UNKNOWN;
    }
    return(ret_value);
}
/**
* \ingroup MODULMACROSX
*
* \file WriteMemBytesX.c
*
* \brief Write bytes to a memory mapped location
*/

struct {
    uint32_t lLen;
    uint32_t lAddr;
} _hal_WriteMemBytesX_staticVars = {};

HAL_FUNCTION(_hal_WriteMemBytesX)
{
    STATIC_VARS_START(_hal_WriteMemBytesX);
    DECL_STATIC_VAR(lLen);
    DECL_STATIC_VAR(lAddr);
    
    int16_t ret_value = 0;
    uint8_t tmp_uchar;

    if(flags & MESSAGE_NEW_MSG)
    {
        if(STREAM_get_long(&lAddr) != 0)
        {
            return HALERR_WRITE_MEM_WORD_NO_RAM_ADDRESS;
        }
        if(STREAM_get_long(&lLen) == -1)
        {
            return HALERR_WRITE_MEM_WORD_NO_RAM_SIZE;
        }
        halt_cpu();
        IHIL_Tclk(0);
        cntrl_sig_low_byte();
        SetReg_16Bits(0x18);
    }

    lLen *= 2; //DLL always sends size in word
    for(; lLen && (ret_value == 0); lLen--)
    {
        ret_value = STREAM_get_byte(&tmp_uchar);
        addr_16bit();
        SetReg_20Bits(lAddr);
        data_to_addr();
        SetReg_8Bits(tmp_uchar);
        IHIL_Tclk(1);
        IHIL_Tclk(0);
        lAddr += 1;
    }

    if(flags & MESSAGE_LAST_MSG)
    {
        release_cpu();
    }
    else if(ret_value == 1)
    {
        STREAM_out_change_type(RESPTYP_ACKNOWLEDGE);
        ret_value = 0;
    }
    else
    {
        ret_value = HALERR_WRITE_MEM_WORD_UNKNOWN;
    }
    return(ret_value);
}

/**
* \ingroup MODULMACROSXV2
*
* \file WriteMemBytesXv2.c
*
* \brief Write bytes to a memory mapped location
*/

/**
  WriteMemBytesXv2
  Write bytes to a memory mapped location.
  inData:  <addr(32)> <length(32)> <data(8)>{*}
  outData: -
  addr: the address to start writing to
  length: number of bytes to write
  data: data to write to the given location
*/

HAL_FUNCTION(_hal_WriteMemBytesXv2)
{
  //! \not supported for Xv2
  return HALERR_UNDEFINED_ERROR;
}
/**
* \ingroup MODULMACROS
*
* \file WriteMemWords.c
*
* \brief Write words (16bit values) to a memory mapped location
*/



/**
  WriteMemWords
  Write words (16bit values) to a memory mapped location.
  inData:  <addr(32)> <length(32)> <data(16)>{*}
  outData: -
  addr: the address to start writing to
  length: number of words to write
  data: data to write to the given location
*/

struct {
    uint32_t lLen;
    uint32_t lAddr;
} _hal_WriteMemWords_staticVars = {};

HAL_FUNCTION(_hal_WriteMemWords)
{
    STATIC_VARS_START(_hal_WriteMemWords);
    DECL_STATIC_VAR(lLen);
    DECL_STATIC_VAR(lAddr);
    
    int16_t ret_value = 0;
    uint16_t tmp_uint;

    if(flags & MESSAGE_NEW_MSG)
    {
        if(STREAM_get_long(&lAddr) != 0)
        {
            return HALERR_WRITE_MEM_WORD_NO_RAM_ADDRESS;
        }
        if(STREAM_get_long(&lLen) == -1)
        {
            return HALERR_WRITE_MEM_WORD_NO_RAM_SIZE;
        }
        halt_cpu();
        IHIL_Tclk(0);
        cntrl_sig_16bit();
        SetReg_16Bits(0x2408);
    }

    for(; lLen && (ret_value == 0); lLen--)
    {
        ret_value = STREAM_get_word(&tmp_uint);
        addr_16bit();
        SetReg_16Bits((uint16_t)lAddr);
        data_to_addr();
        SetReg_16Bits(tmp_uint);
        IHIL_Tclk(1);
        IHIL_Tclk(0);
        lAddr += 2;
    }

    if(flags & MESSAGE_LAST_MSG)
    {
        release_cpu();
    }
    else if(ret_value == 1)
    {
        STREAM_out_change_type(RESPTYP_ACKNOWLEDGE);
        ret_value = 0;
    }
    else
    {
        ret_value = HALERR_WRITE_MEM_WORD_UNKNOWN;
    }
    return(ret_value);
}
/**
* \ingroup MODULMACROSX
*
* \file WriteMemWordsX.c
*
* \brief Write words (16bit values) to a memory mapped location
*/

struct {
    uint32_t lLen;
    uint32_t lAddr;
} _hal_WriteMemWordsX_staticVars = {};

HAL_FUNCTION(_hal_WriteMemWordsX)
{
    STATIC_VARS_START(_hal_WriteMemWordsX);
    DECL_STATIC_VAR(lLen);
    DECL_STATIC_VAR(lAddr);
    
    int16_t ret_value = 0;
    uint16_t tmp_uint;

    if(flags & MESSAGE_NEW_MSG)
    {
        if(STREAM_get_long(&lAddr) != 0)
        {
            return HALERR_WRITE_MEM_WORD_NO_RAM_ADDRESS;
        }
        if(STREAM_get_long(&lLen) == -1)
        {
            return HALERR_WRITE_MEM_WORD_NO_RAM_SIZE;
        }
        halt_cpu();
        IHIL_Tclk(0);
        cntrl_sig_low_byte();
        SetReg_16Bits(0x08);
    }

    for(; lLen && (ret_value == 0); lLen--)
    {
        ret_value = STREAM_get_word(&tmp_uint);
        addr_16bit();
        SetReg_20Bits(lAddr);
        data_to_addr();
        SetReg_16Bits(tmp_uint);
        IHIL_Tclk(1);
        IHIL_Tclk(0);
        lAddr += 2;
    }

    if(flags & MESSAGE_LAST_MSG)
    {
        release_cpu();
    }
    else if(ret_value == 1)
    {
        STREAM_out_change_type(RESPTYP_ACKNOWLEDGE);
        ret_value = 0;
    }
    else
    {
        ret_value = HALERR_WRITE_MEM_WORD_UNKNOWN;
    }
    return(ret_value);
}
/**
* \ingroup MODULMACROSXV2
*
* \file WriteMemWordsXv2.c
*
* \brief Write words (16bit values) to a memory mapped location
*/



/**
  WriteMemWordsXv2
  Write words (16bit values) to a memory mapped location.
  inData:  <addr(32)> <length(32)> <data(16)>{*}
  outData: -
  addr: the address to start writing to
  length: number of words to write
  data: data to write to the given location
*/

struct {
    uint32_t lLen;
    uint32_t lAddr;
} _hal_WriteMemWordsXv2_staticVars = {};

HAL_FUNCTION(_hal_WriteMemWordsXv2)
{
    STATIC_VARS_START(_hal_WriteMemWordsXv2);
    DECL_STATIC_VAR(lLen);
    DECL_STATIC_VAR(lAddr);
    
    uint16_t *pBuf;
    uint16_t sizeOfBuf;
    int16_t ret_value = 0;


    if(flags & MESSAGE_NEW_MSG)
    {
        if(STREAM_get_long(&lAddr) != 0)
        {
            return HALERR_WRITE_MEM_WORD_XV2_NO_RAM_ADDRESS;
        }
        if(STREAM_get_long(&lLen) == -1)
        {
            return HALERR_WRITE_MEM_WORD_XV2_NO_RAM_SIZE;
        }
    }
    
    STREAM_out_change_type(RESPTYP_ACKNOWLEDGE);
    STREAM_flush();
    
    STREAM_get_buffer((void **)&pBuf,&sizeOfBuf);
    for(; lLen && sizeOfBuf; lLen--)
    {
        //! \todo check correct loop cycle with design for performance optimization
        IHIL_Tclk(0);
        cntrl_sig_16bit();
        SetReg_16Bits(0x0500);
        addr_16bit();
        SetReg_20Bits(lAddr);
        IHIL_Tclk(1);
        data_to_addr();
        SetReg_16Bits(*pBuf++);
        IHIL_Tclk(0);
        cntrl_sig_16bit();
        SetReg_16Bits(0x0501);
        IHIL_Tclk(1);
        // one or more cycle, so CPU is driving correct MAB
        IHIL_TCLK();
        lAddr += 2;
        sizeOfBuf -= 2;
    }
    return(ret_value);
}
