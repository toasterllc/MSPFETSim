#pragma once
#include "Types.h"
#include "Stream.h"
#include "HIL.h"

using HalFuncInOut = int16_t (MSPProbeSim::*)(uint16_t);

#define HAL_FUNCTION(fn)                                \
    int16_t fn(uint16_t flags)

#define HAL_FUNCTION_UNIMP(fn)                          \
    int16_t fn(uint16_t flags) {                        \
        printf("### HAL: UNIMPLEMENTED: " #fn "\n");    \
        return 0;                                       \
    }

#define ACTIVE              1
#define LPM5_MODE           2

#define RSTHIGH 0
#define RSTLOW  1

#define BOOT_DATA_CRC_WRONG 0xC3C3

#define SR             2
#define	EEM_STOPPED    0x0080

#define CPUOFF              (0x0010u)
#define DBGJTAGON           (0x0080u)

enum {JTAG = 0, SPYBIWIRE = 1, SPYBIWIREJTAG = 2, JTAGUNDEF = 4, SPYBIWIRE_SUBMCU = 5, SPYBIWIRE_MSP_FET=6, JTAG_432 = 7, SWD_432 = 8};

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

ARMConfigSettings armConfigSettings = {};
DevicePowerSettings devicePowerSettings = {};
DeviceSettings deviceSettings = {};
int16_t intstate = 0;
int8_t traceBuffer[256];
uint16_t LPMx5_DEVICE_STATE = ACTIVE;
uint16_t TCE = 0;
uint16_t activeDevice = 0;
uint16_t altRomAddressForCpuRead = 0;
uint16_t assertBslValidBit = 0;
uint16_t enhancedPsa = 0;
uint16_t lastTraceWritePos = 0;
uint16_t setPCclockBeforeCapture = 0;
uint16_t wdtctlAddress5xx = 0x015C;
uint32_t JTAGLock5xx = 0xCACACACA;
uint32_t _hal_mclkCntrl0 = 0;
uint32_t cswValues[4] = {};
uint64_t prevJState = 0;
uint8_t mclk_modules[32] = {};
uint8_t numOfDevices = 0;
uint32_t writeMemAddr = 0;
uint32_t writeMemLen = 0;

inline static int16_t jtagIdIsValid(uint32_t id)
{
	return id == 0x89 || id == 0x8D || id == 0x91 || id == 0x95 || id == 0x98 || id == 0x99 || id == 0x4ba00477;
}

inline static int16_t jtagIdIsMSP432(uint32_t id)
{
	return id == 0x4ba00477;
}

inline static int16_t jtagIdIsMSP430(uint32_t id)
{
	return id == 0x89 || id == 0x8D || id == 0x91 || id == 0x95 || id == 0x98 || id == 0x99;
}

inline static int16_t jtagIdIsXv2(uint32_t id)
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

void globalVarsInit(void)
{
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

void* ResetFirmware()
{
    globalVarsInit();
    
    {
        // power up VCC as FET power supply
        IHIL_SetVcc(3300);
        IHIL_SwitchVccFET(1);
    }

    _hal_mclkCntrl0 = 0x040f;
    
    _hal_Init(0);
    
    return nullptr;
}

// For some reason there are two "hal_zero" functions: HAL_Zero and _hal_Zero
// HAL_Zero is a regular function, while _hal_Zero is in the HAL table.
// When the callAddr is 0, V3OP_Rx() calls both: it calls HAL_Zero() as a
// special case, and calls _hal_Zero during the normal HAL function dispatching.
int16_t HAL_Zero(const uint8_t* data)
{
    int16_t ret_value = 0;
    uint16_t i;
    if (data[MESSAGE_EXECUTE_ZERO_ADDR_POS] == STREAM_CORE_ZERO_VERSION)  // call for SW version
    {
        if(STREAM_out_init(data[MESSAGE_MSG_ID_POS], RESPTYP_DATA) >= 0)
        {
            uint32_t coreVersion_ToolId = Bios_getCore_version();
            coreVersion_ToolId =(coreVersion_ToolId<<16) + Bios_getTool_id();
            if(Bios_getHal_signature() == 0xBEEFBEEF && V3OP_HalCrcOk() &&
               Bios_getHil_signature() == 0xF00DF00D && V3OP_HilCrcOk())
            {
                STREAM_put_word(SW_0);
                STREAM_put_word(SW_1);
            }
            else
            {
                STREAM_put_word(Bios_getTool_id());
                STREAM_put_word(Bios_getTool_id());
            }
            STREAM_put_word(Bios_getInfo_hw_0());
            STREAM_put_word(Bios_getInfo_hw_1());

            STREAM_put_bytes((uint8_t*)&coreVersion_ToolId, sizeof(coreVersion_ToolId));
            STREAM_put_word(HIL_Version);
            
            if(Bios_getDcdc_signature() == 0xABBAABBA && V3OP_DcdcCrcOk())
            {
                STREAM_put_word(dcdc_getLayerVersion());
            }
            else
            {
                STREAM_put_word(-1);
            }
            if(Bios_getDcdc_signature() == 0xABBAABBA && V3OP_DcdcCrcOk() && Bios_getTool_id() != eZ_FET_NO_DCDC)
            {
                STREAM_put_word(dcdc_getSubMcuVersion());
            }
            else
            {
                STREAM_put_word(-1);
            }
             // return dummy value for Uart version module
            if(Bios_getCom_signature() == 0xACDCACDC && V3OP_ComChannelCrcOk())
            {
                STREAM_put_word(com_getLayerVersion());
            }
            else
            {
                STREAM_put_word(-1);
            }

            // return CRC's for core and all firmware modules
            STREAM_put_word(V3OP_GetHilCrc());
            STREAM_put_word(V3OP_GetHalCrc());
            STREAM_put_word(V3OP_GetDcdcCrc());
            STREAM_put_word(V3OP_GetCoreCrc());
            STREAM_put_word(V3OP_GetComChannelCrc());
            STREAM_put_word(FPGA_VERSION);

            STREAM_put_word(BIOS_RX_QUEUS);
            STREAM_put_word(BIOS_RX_SIZE);
            STREAM_put_long(0);

            // reset static vars
            STREAM_resetSharedVariables();
            ret_value = 1;
        }
        else
        {
            ret_value = -3;
        }
    }
    else if(data[MESSAGE_EXECUTE_ZERO_ADDR_POS] == STREAM_CORE_ZERO_MACRO_SIZE)
    {
        if(STREAM_out_init(data[MESSAGE_MSG_ID_POS], RESPTYP_DATA) >= 0)
        {
            STREAM_put_word(std::size(HALFns));
            ret_value =  1;
        }
        else
        {
            ret_value = -4;
        }
    }
    else if(data[MESSAGE_EXECUTE_ZERO_ADDR_POS] == STREAM_CORE_ZERO_MACRO_ADDR)
    {
        if(STREAM_out_init(data[MESSAGE_MSG_ID_POS], RESPTYP_DATA) >= 0)
        {
            for(i =0;i<std::size(HALFns);i++)
            {
                STREAM_put_word(i);
                STREAM_put_word(i);
            }
            ret_value = 1;
        }
        else
        {
            ret_value = -5;
        }
    }
    else if(data[MESSAGE_EXECUTE_ZERO_ADDR_POS] == STREAM_CORE_ZERO_PUC_RESET)
    {
        if(STREAM_out_init(data[MESSAGE_MSG_ID_POS], RESPTYP_ACKNOWLEDGE) >= 0)
        {
            STREAM_flush();
            ret_value = -6;
        }
        V3OP_HwReset();
        // there can't send a ret_value
    }
    if(ret_value == 1)
    {
        ret_value = MESSAGE_NO_RESPONSE;
        STREAM_flush();
    }
    return(ret_value);
}

HAL_FUNCTION(_hal_Zero) {
    return 0;
}

HAL_FUNCTION(_hal_Init) {
    IHIL_Init();
    return 0;
}

HAL_FUNCTION_UNIMP(_hal_SetVcc)
HAL_FUNCTION_UNIMP(_hal_GetVcc)

HAL_FUNCTION(_hal_StartJtag)
{
    uint8_t chainLen;
    uint8_t protocol;
    int16_t ret_value = -1;

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

HAL_FUNCTION_UNIMP(_hal_StartJtagActivationCode)

HAL_FUNCTION(_hal_StopJtag)
{
    RestoreTestRegs();
    IHIL_Close();

  return 0;
}

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
        armConfigSettings.scsBase = value;
        break;

    case (CONFIG_PARAM_FPB_BASE_ADDRESS):
        armConfigSettings.fpbBase = value;
        break;

    case (CONFIG_PARAM_INTERRUPT_OPTIONS):
        armConfigSettings.interruptOptions = value | 0x80;
        break;

    case (CONFIG_PARAM_ULP_MSP432):
        armConfigSettings.ulpDebug = value;
        break;

    case (CONFIG_PARAM_JTAG_LOCK_5XX):
        JTAGLock5xx = value;
        break;
        
    default:
        return (CONFIG_PARAM_UNKNOWN_PARAMETER);
    }

    return(0);
}

HAL_FUNCTION_UNIMP(_hal_GetFuses)
HAL_FUNCTION_UNIMP(_hal_BlowFuse)

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

    if(!jtagIdIsValid(cntrl_sig_capture_r()))
    {
        return 2;
    }

    // poll for bits in EEM GENCTRL register
    eem_read_control();
    lOut = IHIL_SetReg_16Bits_R(0x0000);

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

HAL_FUNCTION_UNIMP(_hal_BitSequence)

HAL_FUNCTION(_hal_GetJtagId)
{
    int16_t i;
    uint16_t JtagId = 0;

    for (i = 0; i < 4; ++i)
    {
        JtagId = cntrl_sig_capture_r();
        // now get device id pointer and core ip id
        if (jtagIdIsValid(JtagId))
        {
            STREAM_put_word(JtagId);
            return 0;
        }
    }// end of JTAG id scan
     return HALERR_UNDEFINED_ERROR;
}

HAL_FUNCTION_UNIMP(_hal_SetDeviceChainInfo)
HAL_FUNCTION_UNIMP(_hal_SetChainConfiguration)
HAL_FUNCTION_UNIMP(_hal_GetNumOfDevices)

HAL_FUNCTION(_hal_GetInterfaceMode)
{
    uint16_t id=0, loopCount=7,i=0, protocol =0;

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
        
        // Run  4wire/SBW entry Sequence & Reset high
        IHIL_Open(RSTHIGH);
        // Reset TAP state machine -> Run-Test/Idle
        IHIL_TapReset();
        // Run Fuse Check
        IHIL_CheckJtagFuse();
        // shift out JTAG ID
        id = cntrl_sig_capture_r();
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

HAL_FUNCTION(_hal_GetDeviceIdPtr)
{
    uint16_t JtagId = 0;
    uint16_t CoreIpId = 0;
    int32_t DeviceIpPointer = 0;
    int32_t IdDataAddr = 0;
    uint32_t lOut_long = 0;

    JtagId = cntrl_sig_capture_r();
    if (!jtagIdIsValid(JtagId))
    {
        return HALERR_UNDEFINED_ERROR;
    }
    else if (jtagIdIsXv2(JtagId))
    {
        // Get Core identification info
        core_ip_pointer();
        CoreIpId = IHIL_SetReg_16Bits_R(0);
        // Get device identification pointer
        if(JtagId == JTAGVERSION95)
        {
            IHIL_Delay_1ms(1500);
        }
        device_ip_pointer();
        lOut_long = IHIL_SetReg_20Bits_R(0);
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

//static int16_t _hal_GetInterfaceMode(uint16_t flags) {
//    uint16_t id=0, loopCount=7,i=0, protocol =0;
//    for (i = 0; i < loopCount; i++)
//    {
//        // set JTAG mode 1xx- 4xx
//        if(i == 0 || i == 3)// set JTAG
//        {
//            protocol = JTAG;
//        }
//        else
//	    {
//            // set SBW2 mode all devices
//			if(i == 2 || i == 5)
//			{
//                protocol = SPYBIWIRE;
//			}
//            // set SBW4 mode all devices
//			else if(i == 1|| i ==4 || i ==6)
//			{
//                protocol = SPYBIWIREJTAG;
//			}
//        }
//        IHIL_SetProtocol(protocol);
//
//        // Run  4wire/SBW entry Sequence & Reset high
//        IHIL_Open(RSTHIGH);
//        // Reset TAP state machine -> Run-Test/Idle
//        IHIL_TapReset();
//        // Run Fuse Check
//        IHIL_CheckJtagFuse();
//        // shift out JTAG ID
//        id = cntrl_sig_capture();
//        // now check for vaild JTAG ID
//        if (jtagIdIsValid(id))
//        {
//            STREAM_put_word(id);
//            STREAM_put_word(protocol);
//            return 0;
//        }
//    }
//    // Error no mode found
//    STREAM_put_word(0xFFFF);
//    STREAM_put_word(0xAAAA);
//    return HALERR_UNDEFINED_ERROR;
//}

HAL_FUNCTION_UNIMP(_hal_SyncJtag_AssertPor_SaveContext)
HAL_FUNCTION_UNIMP(_hal_SyncJtag_Conditional_SaveContext)
HAL_FUNCTION_UNIMP(_hal_RestoreContext_ReleaseJtag)
HAL_FUNCTION_UNIMP(_hal_ReadMemBytes)
HAL_FUNCTION_UNIMP(_hal_ReadMemWords)
HAL_FUNCTION_UNIMP(_hal_ReadMemQuick)
HAL_FUNCTION_UNIMP(_hal_WriteMemBytes)
HAL_FUNCTION_UNIMP(_hal_WriteMemWords)
HAL_FUNCTION_UNIMP(_hal_EemDataExchange)
HAL_FUNCTION_UNIMP(_hal_EemDataExchangeAFE2xx)
HAL_FUNCTION_UNIMP(_hal_SingleStep)
HAL_FUNCTION_UNIMP(_hal_ReadAllCpuRegs)
HAL_FUNCTION_UNIMP(_hal_WriteAllCpuRegs)
HAL_FUNCTION_UNIMP(_hal_Psa)
HAL_FUNCTION_UNIMP(_hal_ExecuteFunclet)
HAL_FUNCTION_UNIMP(_hal_ExecuteFuncletJtag)
HAL_FUNCTION_UNIMP(_hal_GetDcoFrequency)
HAL_FUNCTION_UNIMP(_hal_GetDcoFrequencyJtag)
HAL_FUNCTION_UNIMP(_hal_GetFllFrequency)
HAL_FUNCTION_UNIMP(_hal_GetFllFrequencyJtag)
HAL_FUNCTION_UNIMP(_hal_WaitForStorage)
HAL_FUNCTION_UNIMP(_hal_SyncJtag_AssertPor_SaveContextX)
HAL_FUNCTION_UNIMP(_hal_SyncJtag_Conditional_SaveContextX)
HAL_FUNCTION_UNIMP(_hal_RestoreContext_ReleaseJtagX)
HAL_FUNCTION_UNIMP(_hal_ReadMemBytesX)
HAL_FUNCTION_UNIMP(_hal_ReadMemWordsX)
HAL_FUNCTION_UNIMP(_hal_ReadMemQuickX)
HAL_FUNCTION_UNIMP(_hal_WriteMemBytesX)
HAL_FUNCTION_UNIMP(_hal_WriteMemWordsX)
HAL_FUNCTION_UNIMP(_hal_EemDataExchangeX)
HAL_FUNCTION_UNIMP(_hal_SingleStepX)
HAL_FUNCTION_UNIMP(_hal_ReadAllCpuRegsX)
HAL_FUNCTION_UNIMP(_hal_WriteAllCpuRegsX)
HAL_FUNCTION_UNIMP(_hal_PsaX)
HAL_FUNCTION_UNIMP(_hal_ExecuteFuncletX)
HAL_FUNCTION_UNIMP(_hal_GetDcoFrequencyX)
HAL_FUNCTION_UNIMP(_hal_GetFllFrequencyX)
HAL_FUNCTION_UNIMP(_hal_WaitForStorageX)
HAL_FUNCTION_UNIMP(_hal_BlowFuseXv2)
HAL_FUNCTION_UNIMP(_hal_BlowFuseFram)

HAL_FUNCTION(_hal_SyncJtag_AssertPor_SaveContextXv2)
{
    uint16_t MyOut[4];
    int16_t i;
    uint16_t address;
    uint16_t wdtVal;
    uint16_t id = cntrl_sig_capture_r();
    uint32_t lOut_long = 0;

    // -------------------------Power mode handling start ----------------------
    //Disable Lpmx5 and power gaiting settings
    if( id == JTAGVERSION99)
    {
        test_reg_3V();
        IHIL_SetReg_16Bits(0x40A0);
        test_reg();
        IHIL_SetReg_32Bits(0x00010000);
    }
    // -------------------------Power mode handling end ------------------------
    // enable clock control before sync
    // switch all functional clocks to JCK = 1 and stop them
    eem_data_exchange32();
    IHIL_SetReg_32Bits(GENCLKCTRL + WRITE);
    IHIL_SetReg_32Bits(MCLK_SEL3 + SMCLK_SEL3 + ACLK_SEL3 + STOP_MCLK + STOP_SMCLK + STOP_ACLK);
    // enable Emualtion Clocks
    eem_write_control();
    IHIL_SetReg_16Bits(EMU_CLK_EN + EEM_EN);

    cntrl_sig_16bit();
    // release RW and BYTE control signals in low byte, set TCE1 & CPUSUSP(!!) & RW
    IHIL_SetReg_16Bits(0x1501);

    if(wait_for_synch())
    {
        // provide one more clock to empty the pipe
        IHIL_TCLK();

        cntrl_sig_16bit();
        // release CPUFLUSH(=CPUSUSP) signal and apply POR signal
        IHIL_SetReg_16Bits(0x0C01);
        IHIL_Delay_1ms(40);

        // release POR signal again
        IHIL_SetReg_16Bits(0x0401); // disable fetch of CPU // changed from 401 to 501

        if(id == JTAGVERSION91 || id == JTAGVERSION99 || id == JTAGVERSION98)
        {   // Force PC so safe value memory location, JMP $
            data_16bit();
            IHIL_TCLK();
            IHIL_TCLK();
            IHIL_SetReg_16Bits(SAFE_PC_ADDRESS);
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
        IHIL_SetReg_16Bits(0x0501);
        IHIL_TCLK();

        // set EEM FEATURE enable now!!!
        eem_write_control();
        IHIL_SetReg_16Bits(EMU_FEAT_EN + EMU_CLK_EN + CLEAR_STOP);

        // Check that sequence exits on Init State
        cntrl_sig_capture();
        IHIL_SetReg_16Bits(0x0000);
    //    lout == 0x0301,0x3fd3
        // hold Watchdog Timer
        STREAM_get_word(&address);
        STREAM_get_word(&wdtVal);

        MyOut[0] = ReadMemWordXv2(address);
        wdtVal |= (MyOut[0] & 0xFF); // set original bits in addition to stop bit
        WriteMemWordXv2(address, wdtVal);

        // Capture MAB - the actual PC value is (MAB - 4)
        addr_capture();
        lOut_long = IHIL_SetReg_20Bits_R(0);
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
        IHIL_SetReg_32Bits(GENCLKCTRL + WRITE);
        IHIL_SetReg_32Bits(MCLK_SEL0 + SMCLK_SEL0 + ACLK_SEL0 + STOP_MCLK + STOP_SMCLK + STOP_ACLK);

        // reset Vacant Memory Interrupt Flag inside SFRIFG1
        if(id == JTAGVERSION91)
        {
            volatile uint16_t specialFunc = ReadMemWordXv2(0x0102);
            if(specialFunc & 0x8)
            {
                SetPcXv2(0x80, SAFE_PC_ADDRESS);
                cntrl_sig_16bit();
                IHIL_SetReg_16Bits(0x0501);
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

HAL_FUNCTION(_hal_SyncJtag_Conditional_SaveContextXv2)
{
  constexpr uint32_t MaxCyclesForSync = 10000; // must be defined, dependant on DMA (burst transfer)!!!
  uint8_t pipe_empty = 0;
  uint8_t cpuoff = 0;
  uint16_t irRequest = 0;
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
  if (!(IHIL_SetReg_16Bits_R(0x0000) & EEM_STOPPED))
  { // do this only if the device is NOT already stopped.
    // read out control signal register first
    cntrl_sig_capture();
    // check if CPUOFF bit is set
    if(!(IHIL_SetReg_16Bits_R(0x0000) & CNTRL_SIG_CPUOFF))
    { // do the following only if the device is NOT in Low Power Mode
      uint32_t tbValue;
      uint32_t tbCntrl;
      uint32_t tbMask;
      uint32_t tbComb;
      uint32_t tbBreak;

      // Trigger Block 0 value register
      eem_data_exchange32();
      IHIL_SetReg_32Bits(MBTRIGxVAL + READ + TB0);   // load address
      // shift in dummy 0
      tbValue = IHIL_SetReg_32Bits_R(0);               // Trigger Block 0 control register
      IHIL_SetReg_32Bits(MBTRIGxCTL + READ + TB0);   // load address
      // shift in dummy 0
      tbCntrl = IHIL_SetReg_32Bits_R(0);               // Trigger Block 0 mask register
      IHIL_SetReg_32Bits(MBTRIGxMSK + READ + TB0);   // load address
      // shift in dummy 0
      tbMask = IHIL_SetReg_32Bits_R(0);
      // Trigger Block 0 combination register
      IHIL_SetReg_32Bits(MBTRIGxCMB + READ + TB0);   // load address
      tbComb = IHIL_SetReg_32Bits_R(0);                         // shift in dummy 0
      // Trigger Block 0 combination register
      IHIL_SetReg_32Bits(BREAKREACT + READ);         // load address
      tbBreak = IHIL_SetReg_32Bits_R(0);                         // shift in dummy 0

      // now configure a trigger on the next instruction fetch
      IHIL_SetReg_32Bits(MBTRIGxCTL + WRITE + TB0);   // load address
      IHIL_SetReg_32Bits(CMP_EQUAL + TRIG_0 + MAB);
      IHIL_SetReg_32Bits(MBTRIGxMSK + WRITE + TB0);   // load address
      IHIL_SetReg_32Bits(MASK_ALL);
      IHIL_SetReg_32Bits(MBTRIGxCMB + WRITE + TB0);   // load address
      IHIL_SetReg_32Bits(EN0);
      IHIL_SetReg_32Bits(BREAKREACT + WRITE);         // load address
      IHIL_SetReg_32Bits(EN0);

      // enable EEM to stop the device
      IHIL_SetReg_32Bits(GENCLKCTRL + WRITE);         // load address
      IHIL_SetReg_32Bits(MCLK_SEL0 + SMCLK_SEL0 + ACLK_SEL0 + STOP_MCLK + STOP_SMCLK + STOP_ACLK);
      eem_write_control();
      IHIL_SetReg_16Bits(EMU_CLK_EN + CLEAR_STOP + EEM_EN);

      {
        int16_t lTimeout = 0;
        do
        {
          eem_read_control();
          lTimeout++;
        }
        while(!(IHIL_SetReg_16Bits_R(0x0000) & EEM_STOPPED) && lTimeout < 3000);
      }
      // restore the setting of Trigger Block 0 previously stored
      // Trigger Block 0 value register
      eem_data_exchange32();
      IHIL_SetReg_32Bits(MBTRIGxVAL + WRITE + TB0);   // load address
      IHIL_SetReg_32Bits(tbValue);
      IHIL_SetReg_32Bits(MBTRIGxCTL + WRITE + TB0);   // load address
      IHIL_SetReg_32Bits(tbCntrl);
      IHIL_SetReg_32Bits(MBTRIGxMSK + WRITE + TB0);   // load address
      IHIL_SetReg_32Bits(tbMask);
      IHIL_SetReg_32Bits(MBTRIGxCMB + WRITE + TB0);   // load address
      IHIL_SetReg_32Bits(tbComb);
      IHIL_SetReg_32Bits(BREAKREACT + WRITE);         // load address
      IHIL_SetReg_32Bits(tbBreak);
    }
  }
// End: special handling note 822
//------------------------------------------------------------------------------

  // enable clock control before sync
  eem_write_control();
  IHIL_SetReg_16Bits(EMU_CLK_EN + EEM_EN);

  // sync device to JTAG
  SyncJtagXv2();

  // reset CPU stop reaction - CPU is now under JTAG control
  // Note: does not work on F5438 due to note 772, State storage triggers twice on single stepping
  eem_write_control();
  IHIL_SetReg_16Bits(EMU_CLK_EN + CLEAR_STOP + EEM_EN);
  IHIL_SetReg_16Bits(EMU_CLK_EN + CLEAR_STOP);

  cntrl_sig_16bit();
  IHIL_SetReg_16Bits(0x1501);
  // clock system into Full Emulation State now...
  // while checking control signals CPUSUSP (pipe_empty), CPUOFF and HALT

  cntrl_sig_capture();
  lOut = IHIL_SetReg_16Bits_R(0);

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
        lOut = IHIL_SetReg_16Bits_R(0);
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
    (IHIL_SetReg_16Bits_R(0x000) & CNTRL_SIG_CPUSUSP) ? (pipe_empty = TRUE) : (pipe_empty = FALSE);
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
  IHIL_SetReg_16Bits(0x0501);

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
  lOut_long = IHIL_SetReg_20Bits_R(0);

  cntrl_sig_capture();
  lOut = IHIL_SetReg_16Bits_R(0);
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
  IHIL_SetReg_16Bits(EMU_FEAT_EN + EMU_CLK_EN + CLEAR_STOP);

  // check for Init State
  cntrl_sig_capture();
  IHIL_SetReg_16Bits(0);

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
        IHIL_SetReg_16Bits(0x0401);
        // preload the MDB before releasing the CPU
        data_16bit();
        IHIL_Tclk(1);
        IHIL_SetReg_16Bits(Mdb);
        IHIL_Tclk(0);
        // release the busses - otherwise modules/flash can't control the MDB
        // = get out of DATA_* instruction
        addr_capture();
        // here one more clock cycle is required to advance the CPU
        // otherwise enabling the EEM can stop the device again
        {
            cntrl_sig_capture();
            if(IHIL_SetReg_16Bits_R(0) & CNTRL_SIG_HALT)
            {
                cntrl_sig_16bit();
                IHIL_SetReg_16Bits(0x0403);
            }
        }
        IHIL_Tclk(1);
    }
    else
    {
        IHIL_Tclk(1);
        cntrl_sig_16bit();
        IHIL_SetReg_16Bits(0x0401);
        addr_capture();
        // here one more clock cycle is required to advance the CPU
        // otherwise enabling the EEM would stop the device again, because
        {
            cntrl_sig_capture();
            IHIL_Tclk(0);

            // shift out current control signal register value
            if(IHIL_SetReg_16Bits_R(0) & CNTRL_SIG_HALT)
            {
                cntrl_sig_16bit();
                IHIL_SetReg_16Bits(0x0403);
            }
            IHIL_Tclk(1);
        }
    }
    // now we can enable the EEM
    // --> this should work asynchronously
    eem_write_control();
    IHIL_SetReg_16Bits(control_mask);

    // -------------------------Power mode handling start ----------------------
    if(ulpDebug)
    {
        // Set LPMx.5 settings
        EnableLpmx5();
    }
    else
    {
        uint16_t id = cntrl_sig_capture_r();
        DisableLpmx5();
        //Manually disable DBGJTAGON bit
        if(id == JTAGVERSION99)
        {
            test_reg_3V();
            IHIL_SetReg_16Bits(IHIL_SetReg_16Bits_R(0x0000) & ~DBGJTAGON);
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
    IHIL_SetReg_20Bits(lAddr);
    IHIL_Tclk(1);
    IHIL_Tclk(0);
    data_capture();
    STREAM_put_word(IHIL_SetReg_16Bits_R(0));
    lAddr += 2;
  }
  IHIL_Tclk(1);
  // one or more cycle, so CPU is driving correct MAB
  IHIL_TCLK();

  return 0;
}

HAL_FUNCTION(_hal_ReadMemQuickXv2)
{
    uint32_t i;
    uint32_t lAddr;
    uint32_t lLen;
    uint32_t lPc;
    uint16_t Mova;
    uint16_t Pc_l;
    uint16_t id = cntrl_sig_capture_r();

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
    IHIL_SetReg_16Bits(0x0501);
    IHIL_Tclk(1);
    addr_capture();
    // END OF SETTING THE PROGRAM COUNTER
    data_quick();

    // DATA QUICK LOOP
    for (i = 0; i < lLen; i++)
    {
        IHIL_Tclk(1);
        IHIL_Tclk(0);
        STREAM_put_word( IHIL_SetReg_16Bits_R(0));
    }
    // Check save State
    cntrl_sig_capture();
    IHIL_SetReg_16Bits(0x0000);

    // Restore PC
    Mova  = 0x0080;
    Mova += (uint16_t)((lPc>>8) & 0x00000F00);
    Pc_l  = (uint16_t)((lPc & 0xFFFF));

    // SET PROGRAM COUNTER for Backup
    SetPcXv2(Mova, Pc_l);
    cntrl_sig_16bit();
    IHIL_SetReg_16Bits(0x0501);
    IHIL_Tclk(1);
    addr_capture();

    return 0;
}

HAL_FUNCTION(_hal_WriteMemWordsXv2)
{
    uint16_t *pBuf;
    uint16_t sizeOfBuf;
    int16_t ret_value = 0;

    if(flags & MESSAGE_NEW_MSG)
    {
        if(STREAM_get_long(&writeMemAddr) != 0)
        {
            return HALERR_WRITE_MEM_WORD_XV2_NO_RAM_ADDRESS;
        }
        if(STREAM_get_long(&writeMemLen) == -1)
        {
            return HALERR_WRITE_MEM_WORD_XV2_NO_RAM_SIZE;
        }
    }
    
    STREAM_out_change_type(RESPTYP_ACKNOWLEDGE);
    STREAM_flush();
    
    STREAM_get_buffer((void **)&pBuf,&sizeOfBuf);
    for(; writeMemLen && sizeOfBuf; writeMemLen--)
    {
        //! \todo check correct loop cycle with design for performance optimization
        IHIL_Tclk(0);
        cntrl_sig_16bit();
        IHIL_SetReg_16Bits(0x0500);
        addr_16bit();
        IHIL_SetReg_20Bits(writeMemAddr);
        IHIL_Tclk(1);
        data_to_addr();
        IHIL_SetReg_16Bits(*pBuf++);
        IHIL_Tclk(0);
        cntrl_sig_16bit();
        IHIL_SetReg_16Bits(0x0501);
        IHIL_Tclk(1);
        // one or more cycle, so CPU is driving correct MAB
        IHIL_TCLK();
        writeMemAddr += 2;
        sizeOfBuf -= 2;
    }
    return(ret_value);
}

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
                IHIL_SetReg_32Bits(tmp_char);   // load address
                lOut_long = IHIL_SetReg_32Bits_R(0);
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

                IHIL_SetReg_32Bits(tmp_char);           // load address
                IHIL_SetReg_32Bits(tmp_ulong);  // shift in value
            }
        }
        NumberOfExchanges--;
    }
    return 0;
}

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
    _hal_EemDataExchangeXv2(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG);
    STREAM_external_stream(&stream_tmp);
  }
  STREAM_internal_stream(stream_in_release, sizeof(stream_in_release), MESSAGE_NO_OUT, 0, &stream_tmp);
  _hal_RestoreContext_ReleaseJtagXv2(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG); // Data from DLL Stream
  STREAM_external_stream(&stream_tmp);

  if((!(stream_in_release[8] & CPUOFF)) || (irRequest & 0x4))
  {
    // poll for CPU stop reaction
    eem_read_control();
    do
    {
      i++;
    }
    while(!(IHIL_SetReg_16Bits_R(0x0000) & 0x0080) && (i < 500));

    if(i < 500)
    {
      // take target under JTAG control
      STREAM_internal_stream(stream_in_sync, sizeof(stream_in_sync), MESSAGE_OUT_TO_DLL, 0, &stream_tmp);
      RetState = _hal_SyncJtag_Conditional_SaveContextXv2(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG); // In
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
      _hal_EemDataExchangeXv2(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG);
      STREAM_external_stream(&stream_tmp);
    }
  }
  else
  {
      // First Check if we did not go into LPMx.5
      if (jtagIdIsValid(cntrl_sig_capture_r()))
      {
          // take target under JTAG control
          STREAM_internal_stream(stream_in_sync, sizeof(stream_in_sync), MESSAGE_OUT_TO_DLL, 0, &stream_tmp);
          RetState = _hal_SyncJtag_Conditional_SaveContextXv2(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG); // In
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

HAL_FUNCTION(_hal_ReadAllCpuRegsXv2)
{
    uint16_t Registers;
    uint16_t Mova;
    uint32_t Rx;
    uint16_t id = cntrl_sig_capture_r();

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
            IHIL_SetReg_16Bits(0x0501);
            IHIL_Tclk(1);
            addr_capture();
        }
    }
    // all CPU register values have been moved to the JMBOUT register address
    // -> JMB needs to be cleared again!!
    Rx = i_ReadJmbOut();
    return 0;
}

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

HAL_FUNCTION_UNIMP(_hal_PsaXv2)

#define startAddrOfs FlashWrite_o_[4]

#define REG_ADDRESS 5
#define REG_SIZE    6
#define REG_LOCKA   8
#define REG_TYPE    9

#define TIMEOUT_COUNT   300u

void setFuncletRegisters(const uint32_t* registerData)
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

struct {
    uint32_t lLen               = 0;
    uint16_t ret_len            = 0;
    uint32_t registerBackups[4] = {};
    uint16_t allignNeed         = 0;
    uint16_t dataL              = 0;
    uint16_t dataH              = 0;
} _hal_ExecuteFuncletXv2_staticVars;

HAL_FUNCTION(_hal_ExecuteFuncletXv2)
{
    auto& lLen              = _hal_ExecuteFuncletXv2_staticVars.lLen;
    auto& ret_len           = _hal_ExecuteFuncletXv2_staticVars.ret_len;
    auto& registerBackups   = _hal_ExecuteFuncletXv2_staticVars.registerBackups;
    auto& allignNeed        = _hal_ExecuteFuncletXv2_staticVars.allignNeed;
    auto& dataL             = _hal_ExecuteFuncletXv2_staticVars.dataL;
    auto& dataH             = _hal_ExecuteFuncletXv2_staticVars.dataH;
    
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
        // get start addres as long value
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
        IHIL_SetReg_16Bits(0x0401);
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
            _hal_SyncJtag_Conditional_SaveContextXv2(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG);
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
            _hal_SyncJtag_Conditional_SaveContextXv2(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG);
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
         _hal_SyncJtag_Conditional_SaveContextXv2(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG);
         STREAM_external_stream(&stream_tmp);

         ret_value = HALERR_EXECUTE_FUNCLET_EXECUTION_ERROR;
    }
    return(ret_value);
}

HAL_FUNCTION_UNIMP(_hal_UnlockDeviceXv2)

HAL_FUNCTION(_hal_MagicPattern)
{
    uint16_t id = 0;
    uint8_t chainLen = 1;
    uint16_t protocol = SPYBIWIRE;

    STREAM_get_word(&protocol);
    IHIL_SetProtocol(protocol);
    // run entry sequnce but pull rst low during the sequence to wake-up the device
    
    IHIL_Close();
    
    IHIL_RegulateVcc();
    
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

    id = cntrl_sig_capture_r() ;
    if (jtagIdIsXv2(id))
    {
        //Disable Lpmx5 and power gaiting settings
        if(id == JTAGVERSION98)
        {
            // just disable JTAG io lock
            test_reg_3V();
            IHIL_SetReg_16Bits(0x4020);
        }
        if(id == JTAGVERSION99)
        {
            test_reg_3V();
            IHIL_SetReg_16Bits(0x40A0);
            test_reg();
            IHIL_SetReg_32Bits(0x00010000);
        }
        STREAM_put_byte((uint8_t)chainLen);
        STREAM_put_byte((uint8_t)id);
        STREAM_put_byte((uint8_t)protocol);

        return 0;
    }
    return (HALERR_MAGIC_PATTERN);
}

HAL_FUNCTION_UNIMP(_hal_UnlockC092)
HAL_FUNCTION_UNIMP(_hal_HilCommand)
HAL_FUNCTION_UNIMP(_hal_PollJStateReg)
HAL_FUNCTION_UNIMP(_hal_PollJStateRegFR57xx)

HAL_FUNCTION(_hal_IsJtagFuseBlown)
{
    uint16_t  i = 0, lOut = 0;
    uint16_t jtagId = 0;

    for (i = 0; i < 3; i++) // First test could be negative.
    {
        jtagId = cntrl_sig_capture_r();
        if(jtagId != JTAGVERSION99 && jtagId != JTAGVERSION98 && jtagId != JTAGVERSION95 && jtagId != JTAGVERSION91 && jtagId != JTAGVERSION)
        {
            return -1;
        }

        lOut = IHIL_SetReg_16Bits_R(0xaaaa);
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

HAL_FUNCTION_UNIMP(_hal_ResetXv2)
HAL_FUNCTION_UNIMP(_hal_WriteFramQuickXv2)
HAL_FUNCTION_UNIMP(_hal_SendJtagMailboxXv2)
HAL_FUNCTION_UNIMP(_hal_SingleStepJStateXv2)
HAL_FUNCTION_UNIMP(_hal_PollJStateRegEt8)

//struct {
//    EnergyTraceRecordEt8_t buffer[NUM_RECORDS_TO_CAPTURE]   = {};
//    uint16_t currentIndex                                   = 0;
//} _hal_PollJStateRegEt8_staticVars;
//
//HAL_FUNCTION(_hal_PollJStateRegEt8) {
//    auto& buffer = _hal_PollJStateRegEt8_staticVars.buffer;
//    auto& currentIndex = _hal_PollJStateRegEt8_staticVars.currentIndex;
//    
//    double vcc = 0;
//    double extVcc= 0 ;
//    uint16_t mEtGatedMode = 0;
//    uint16_t* syncWithRunVarAddress = 0;
//
//    if(STREAM_discard_bytes(8) == -1)
//    {
//            return -1;
//    }
//    // request shared var from bios to sync with RestoreContextRun function
//
//    STREAM_get_word(&mEtGatedMode);
//
//    if(mEtGatedMode)
//    {
//        syncWithRunVarAddress = getTargetRunningVar();
//        if(syncWithRunVarAddress)
//        {
//             if(!(*syncWithRunVarAddress))
//             {
//                currentIndex = 0;
//                return 2;
//             }
//        }
//        else
//        {
//            return -1;
//        }
//    }
//
//    buffer[currentIndex].eventID = 8;
//
//    while(TA0R > 0xFFA0 || TA0R  < 2)
//    {
//        IHIL_Delay_1us(3);
//    }
//
//    _DINT_FET();
//
//    buffer[currentIndex].TimeStamp = getTimeStamp();
//    buffer[currentIndex].currentTicks = getIMeasure();
//
//    IHIL_GetVcc(&vcc, &extVcc);
//
//    _EINT_FET();
//    buffer[currentIndex].voltage = (unsigned int)(vcc ? vcc : extVcc);
//
//    // Energy trace data is available send it first -> don't check LPMx.5 or Breakpoint hit
//    if(++currentIndex == NUM_RECORDS_TO_CAPTURE)
//    {
//        currentIndex = 0;
//        STREAM_put_word(ENERGYTRACE_INFO);                  // Event ID
//        STREAM_put_byte(NUM_RECORDS_TO_CAPTURE);            // Number of records that is sent
//        STREAM_put_byte(sizeof(EnergyTraceRecordEt8_t));    // Size of Record
//        STREAM_put_bytes((void *)buffer, sizeof(EnergyTraceRecordEt8_t) * NUM_RECORDS_TO_CAPTURE);
//        return 1;
//    }
//
//    return 2;
//}

HAL_FUNCTION(_hal_ResetStaticGlobalVars)
{
    LPMx5_DEVICE_STATE = 1;
    intstate = 0;
    prevJState = 0x0000000000000000;
    lastTraceWritePos = 0;
    return 0;
}

HAL_FUNCTION_UNIMP(_hal_Reset430I)
HAL_FUNCTION_UNIMP(_hal_PollJStateReg430I)
HAL_FUNCTION_UNIMP(_hal_PollJStateReg20)
HAL_FUNCTION_UNIMP(_hal_SwitchMosfet)
HAL_FUNCTION_UNIMP(_hal_ResetL092)
HAL_FUNCTION_UNIMP(_hal_DummyMacro)
HAL_FUNCTION_UNIMP(_hal_Reset5438Xv2)
HAL_FUNCTION_UNIMP(_hal_LeaSyncConditional)
HAL_FUNCTION_UNIMP(_hal_GetJtagIdCodeArm)
HAL_FUNCTION_UNIMP(_hal_ScanApArm)
HAL_FUNCTION_UNIMP(_hal_MemApTransactionArm)
HAL_FUNCTION_UNIMP(_hal_ReadAllCpuRegsArm)
HAL_FUNCTION_UNIMP(_hal_WriteAllCpuRegsArm)
HAL_FUNCTION_UNIMP(_hal_EnableDebugArm)
HAL_FUNCTION_UNIMP(_hal_DisableDebugArm)
HAL_FUNCTION_UNIMP(_hal_RunArm)
HAL_FUNCTION_UNIMP(_hal_HaltArm)
HAL_FUNCTION_UNIMP(_hal_ResetArm)
HAL_FUNCTION_UNIMP(_hal_SingleStepArm)
HAL_FUNCTION_UNIMP(_hal_WaitForDebugHaltArm)
HAL_FUNCTION_UNIMP(_hal_MemApTransactionArmSwd)
HAL_FUNCTION_UNIMP(_hal_GetInterfaceModeArm)
HAL_FUNCTION_UNIMP(_hal_PollDStatePCRegEt)
HAL_FUNCTION_UNIMP(_hal_GetCpuIdArm)
HAL_FUNCTION_UNIMP(_hal_CheckDapLockArm)
HAL_FUNCTION_UNIMP(_hal_UnlockDap)
HAL_FUNCTION_UNIMP(_hal_UssSyncConditional)

static const inline HalFuncInOut HALFns[] = {
    &MSPProbeSim::_hal_Zero,
    &MSPProbeSim::_hal_Init,
    &MSPProbeSim::_hal_SetVcc,
    &MSPProbeSim::_hal_GetVcc,
    &MSPProbeSim::_hal_StartJtag,
    &MSPProbeSim::_hal_StartJtagActivationCode,
    &MSPProbeSim::_hal_StopJtag,
    &MSPProbeSim::_hal_Configure,
    &MSPProbeSim::_hal_GetFuses,
    &MSPProbeSim::_hal_BlowFuse,
    &MSPProbeSim::_hal_WaitForEem,
    &MSPProbeSim::_hal_BitSequence,
    &MSPProbeSim::_hal_GetJtagId,
    &MSPProbeSim::_hal_SetDeviceChainInfo,
    &MSPProbeSim::_hal_SetChainConfiguration,
    &MSPProbeSim::_hal_GetNumOfDevices,
    &MSPProbeSim::_hal_GetInterfaceMode,
    &MSPProbeSim::_hal_GetDeviceIdPtr,
    &MSPProbeSim::_hal_SyncJtag_AssertPor_SaveContext,
    &MSPProbeSim::_hal_SyncJtag_Conditional_SaveContext,
    &MSPProbeSim::_hal_RestoreContext_ReleaseJtag,
    &MSPProbeSim::_hal_ReadMemBytes,
    &MSPProbeSim::_hal_ReadMemWords,
    &MSPProbeSim::_hal_ReadMemQuick,
    &MSPProbeSim::_hal_WriteMemBytes,
    &MSPProbeSim::_hal_WriteMemWords,
    &MSPProbeSim::_hal_EemDataExchange,
    &MSPProbeSim::_hal_EemDataExchangeAFE2xx,
    &MSPProbeSim::_hal_SingleStep,
    &MSPProbeSim::_hal_ReadAllCpuRegs,
    &MSPProbeSim::_hal_WriteAllCpuRegs,
    &MSPProbeSim::_hal_Psa,
    &MSPProbeSim::_hal_ExecuteFunclet,
    &MSPProbeSim::_hal_ExecuteFuncletJtag,
    &MSPProbeSim::_hal_GetDcoFrequency,
    &MSPProbeSim::_hal_GetDcoFrequencyJtag,
    &MSPProbeSim::_hal_GetFllFrequency,
    &MSPProbeSim::_hal_GetFllFrequencyJtag,
    &MSPProbeSim::_hal_WaitForStorage,
    &MSPProbeSim::_hal_SyncJtag_AssertPor_SaveContextX,
    &MSPProbeSim::_hal_SyncJtag_Conditional_SaveContextX,
    &MSPProbeSim::_hal_RestoreContext_ReleaseJtagX,
    &MSPProbeSim::_hal_ReadMemBytesX,
    &MSPProbeSim::_hal_ReadMemWordsX,
    &MSPProbeSim::_hal_ReadMemQuickX,
    &MSPProbeSim::_hal_WriteMemBytesX,
    &MSPProbeSim::_hal_WriteMemWordsX,
    &MSPProbeSim::_hal_EemDataExchangeX,
    &MSPProbeSim::_hal_SingleStepX,
    &MSPProbeSim::_hal_ReadAllCpuRegsX,
    &MSPProbeSim::_hal_WriteAllCpuRegsX,
    &MSPProbeSim::_hal_PsaX,
    &MSPProbeSim::_hal_ExecuteFuncletX,
    &MSPProbeSim::_hal_GetDcoFrequencyX,
    &MSPProbeSim::_hal_GetFllFrequencyX,
    &MSPProbeSim::_hal_WaitForStorageX,
    &MSPProbeSim::_hal_BlowFuseXv2,
    &MSPProbeSim::_hal_BlowFuseFram,
    &MSPProbeSim::_hal_SyncJtag_AssertPor_SaveContextXv2,
    &MSPProbeSim::_hal_SyncJtag_Conditional_SaveContextXv2,
    &MSPProbeSim::_hal_RestoreContext_ReleaseJtagXv2,
    &MSPProbeSim::_hal_ReadMemWordsXv2,
    &MSPProbeSim::_hal_ReadMemQuickXv2,
    &MSPProbeSim::_hal_WriteMemWordsXv2,
    &MSPProbeSim::_hal_EemDataExchangeXv2,
    &MSPProbeSim::_hal_SingleStepXv2,
    &MSPProbeSim::_hal_ReadAllCpuRegsXv2,
    &MSPProbeSim::_hal_WriteAllCpuRegsXv2,
    &MSPProbeSim::_hal_PsaXv2,
    &MSPProbeSim::_hal_ExecuteFuncletXv2,
    &MSPProbeSim::_hal_UnlockDeviceXv2,
    &MSPProbeSim::_hal_MagicPattern,
    &MSPProbeSim::_hal_UnlockC092,
    &MSPProbeSim::_hal_HilCommand,
    &MSPProbeSim::_hal_PollJStateReg,
    &MSPProbeSim::_hal_PollJStateRegFR57xx,
    &MSPProbeSim::_hal_IsJtagFuseBlown,
    &MSPProbeSim::_hal_ResetXv2,
    &MSPProbeSim::_hal_WriteFramQuickXv2,
    &MSPProbeSim::_hal_SendJtagMailboxXv2,
    &MSPProbeSim::_hal_SingleStepJStateXv2,
    &MSPProbeSim::_hal_PollJStateRegEt8,
    &MSPProbeSim::_hal_ResetStaticGlobalVars,
    &MSPProbeSim::_hal_Reset430I,
    &MSPProbeSim::_hal_PollJStateReg430I,
    &MSPProbeSim::_hal_PollJStateReg20,
    &MSPProbeSim::_hal_SwitchMosfet,
    &MSPProbeSim::_hal_ResetL092,
    &MSPProbeSim::_hal_DummyMacro,
    &MSPProbeSim::_hal_Reset5438Xv2,
    &MSPProbeSim::_hal_LeaSyncConditional,
    &MSPProbeSim::_hal_GetJtagIdCodeArm,
    &MSPProbeSim::_hal_ScanApArm,
    &MSPProbeSim::_hal_MemApTransactionArm,
    &MSPProbeSim::_hal_ReadAllCpuRegsArm,
    &MSPProbeSim::_hal_WriteAllCpuRegsArm,
    &MSPProbeSim::_hal_EnableDebugArm,
    &MSPProbeSim::_hal_DisableDebugArm,
    &MSPProbeSim::_hal_RunArm,
    &MSPProbeSim::_hal_HaltArm,
    &MSPProbeSim::_hal_ResetArm,
    &MSPProbeSim::_hal_SingleStepArm,
    &MSPProbeSim::_hal_WaitForDebugHaltArm,
    &MSPProbeSim::_hal_MemApTransactionArmSwd,
    &MSPProbeSim::_hal_GetInterfaceModeArm,
    &MSPProbeSim::_hal_PollDStatePCRegEt,
    &MSPProbeSim::_hal_GetCpuIdArm,
    &MSPProbeSim::_hal_CheckDapLockArm,
    &MSPProbeSim::_hal_UnlockDap,
    &MSPProbeSim::_hal_UssSyncConditional,
};
