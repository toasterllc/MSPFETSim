#pragma once
#include "BIOS.h"
#include "Stream.h"
#include "HAL.h"

#define V3OP_LOOP_WAIT_FLAG   0x01
#define V3OP_LOOP_ARRAY_COUNT 4

#define V3OP_UNIMP          printf("### V3OP: UNIMPLEMENTED: %s\n", __FUNCTION__)
#define V3OP_UNIMP_RET0     V3OP_UNIMP; return 0

struct _V3opLoopArray_
{
  uint16_t addr;
  uint8_t  *indata;
  uint8_t  flags;
  uint8_t  msg_id;
  uint8_t  msg_type;
  uint8_t  active;
};
typedef struct _V3opLoopArray_ V3opLoopArray;

V3opLoopArray v3op_loop_array_[V3OP_LOOP_ARRAY_COUNT] = {
    { 0xFFFF, (uint8_t*)0xFFFF, 0x00, 0x00, 0x00, 1 },
    { 0xFFFF, (uint8_t*)0xFFFF, 0x00, 0x00, 0x00, 1 },
    { 0xFFFF, (uint8_t*)0xFFFF, 0x00, 0x00, 0x00, 1 },
    { 0xFFFF, (uint8_t*)0xFFFF, 0x00, 0x00, 0x00, 1 }
};

uint16_t v30p_stream_flags_ = 0;
uint16_t voltageSupervsion = 0;
uint8_t rx_queu_counter_public_ = 0;
uint8_t tempInData[4][100]= {0x0,0x0};

void IccMonitor_StartVoltageSupervision() {
    voltageSupervsion = 1;
}

void IccMonitor_StopVoltageSupervision() {
    voltageSupervsion = 0;
}

uint16_t Bios_getCore_version() {
    return CORE_VERSION;
}

uint32_t Bios_getHal_signature() {
    return HAL_SIGNATURE;
}

uint32_t Bios_getDcdc_signature() {
    return DCDC_SIGNATURE;
}

uint32_t Bios_getCom_signature() {
    return COM_SIGNATURE;
}

uint16_t Bios_getTool_id() {
    return TOOL_ID;
}

uint16_t Bios_getInfo_hw_0() {
    return INFO_U1_HW_0;
}

uint16_t Bios_getInfo_hw_1() {
    return INFO_U1_HW_1;
}

uint32_t Bios_getHil_signature() {
    return HIL_SIGNATURE;
}

int16_t dcdc_Restart(uint16_t fetType_) {
    V3OP_UNIMP_RET0;
}

int16_t dcdc_getSubMcuVersion() {
    return 0x0034; // Determined empirically by printing `expectedDcdcSubMcuVersion` in UpdateManagerFet.cpp
}

int16_t dcdc_getLayerVersion() {
    return DCDC_LAYER_VERSION;
}

int16_t dcdc_getLayerVersionCmp() {
    return DCDC_LAYER_VERSION_CMP;
}

int16_t dcdc_powerDown() {
    V3OP_UNIMP_RET0;
}

int16_t dcdc_setVcc(uint16_t vcc) {
    return 0;
}

int16_t com_getLayerVersion() {
    return COM_LAYER_VERSION;
}

int16_t com_getLayerVersionCmp() {
    return COM_LAYER_VERSION_CMP;
}







int16_t V3OP_SendException(uint8_t msg_id, uint16_t code, uint16_t* payload) {
    uint16_t i;

    if(STREAM_out_init(msg_id, RESPTYP_EXCEPTION) >= 0)
    {
        if(payload == NULL)
        {
            STREAM_put_word(code);
        }
        else
        {
            for(i = 0;(i < code) && (i < 126); i++)
            {
                STREAM_put_word(payload[i]);
            }
        }
        STREAM_flush();
        return(0);
    }
    return(-1);
}

int16_t V3OP_SetLoop(uint8_t *payload_incl_addr, uint8_t flags)
{
    uint16_t i, j;
    uint8_t payload_size;
    int16_t ret_value = -1;

    // search for same running function
    for(i = 0; i < V3OP_LOOP_ARRAY_COUNT; i++)
    {
        if((v3op_loop_array_[i].addr == *(uint16_t*)&payload_incl_addr[MESSAGE_EXECUTE_CALL_ADDR_POS]) &&
           (v3op_loop_array_[i].msg_id == payload_incl_addr[MESSAGE_MSG_ID_POS]))
        {
            ret_value = -2;
            break;
        }
    }
    // search free slot
    for(i = 0; i < V3OP_LOOP_ARRAY_COUNT; i++)
    {
        if(v3op_loop_array_[i].addr == 0xFFFF)
        {
            break;
        }
    }
    if(i < V3OP_LOOP_ARRAY_COUNT)   // free slot found
    {
        if(payload_incl_addr[0] >= MESSAGE_EXECUTE_PAYLOAD_POS)
        {
            payload_size = payload_incl_addr[0]+1;
            v3op_loop_array_[i].indata = tempInData[i];// (uint8_t*)malloc(payload_size);
            if(v3op_loop_array_[i].indata)
            {
                for(j = 0; j < payload_size; j++)
                {
                    v3op_loop_array_[i].indata[j] = payload_incl_addr[j];
                }
                v3op_loop_array_[i].flags = flags & V3OP_LOOP_WAIT_FLAG;
                v3op_loop_array_[i].msg_id = payload_incl_addr[2] | 0x40;
                v3op_loop_array_[i].msg_type = RESPTYP_DATA;
                v3op_loop_array_[i].addr = *(uint16_t*)&payload_incl_addr[MESSAGE_EXECUTE_CALL_ADDR_POS];
                v3op_loop_array_[i].active = 1;
                ret_value = 1;
            }
            else
            {
                ret_value = -4;
            }
        }
        else
        {
            v3op_loop_array_[i].indata = NULL;
            v3op_loop_array_[i].flags = flags & V3OP_LOOP_WAIT_FLAG;
            v3op_loop_array_[i].msg_id = payload_incl_addr[2] | 0x40;
            v3op_loop_array_[i].msg_type = RESPTYP_DATA;
            v3op_loop_array_[i].addr = *(uint16_t*)&payload_incl_addr[MESSAGE_EXECUTE_CALL_ADDR_POS];
            v3op_loop_array_[i].active = 1;
            ret_value = 1;
        }
    }
  else
  {
    ret_value = -3;
  }
  return(ret_value);
}

int16_t V3OP_KillLoop(uint8_t msg_id)
{
    uint16_t i;
    int16_t ret_value = -1;

    if(msg_id == 0)
    {
        ret_value = 0;
        for(i = 0; i < V3OP_LOOP_ARRAY_COUNT; i++)
        {
            if((v3op_loop_array_[i].addr != 0xFFFF) && v3op_loop_array_[i].addr < 0xFF00)
            {
                if(v3op_loop_array_[i].indata != NULL)
                {
                    //free(v3op_loop_array_[i].indata);
                }
                v3op_loop_array_[i].indata = NULL;
                v3op_loop_array_[i].addr = 0xFFFF;
                v3op_loop_array_[i].active = 0;

                ret_value++;
            }
        }
    }
    else
    {
        for(i = 0; i < V3OP_LOOP_ARRAY_COUNT; i++)
        {
            if(v3op_loop_array_[i].msg_id == msg_id) break;
        }
        if(i < V3OP_LOOP_ARRAY_COUNT)   // addr found
        {
            if(v3op_loop_array_[i].indata != NULL)
            {
//                free(v3op_loop_array_[i].indata);
            }
            v3op_loop_array_[i].indata = NULL;
            v3op_loop_array_[i].addr = 0xFFFF;
            v3op_loop_array_[i].active = 0;
        }
        ret_value = 1;
    }
    return(ret_value);
}

int16_t V3OP_PauseLoop(uint8_t msg_id)
{
    if ( msg_id != 0x40 )
    {
        int16_t i = 0;
        for(i = 0; i < V3OP_LOOP_ARRAY_COUNT; i++)
        {
            if(v3op_loop_array_[i].msg_id == msg_id)
            {
                v3op_loop_array_[i].active = 0;
                break;
            }
        }
    }
    return 1;
}

int16_t V3OP_ResumeLoop(uint8_t msg_id)
{
    if ( msg_id != 0x40 )
    {
        int16_t i = 0;
        for(i = 0; i < V3OP_LOOP_ARRAY_COUNT; i++)
        {
            if(v3op_loop_array_[i].msg_id == msg_id)
            {
                v3op_loop_array_[i].active = 1;
                break;
            }
        }
    }
    return 1;
}

void V3OP_KillAllLoops(void)
{
    int16_t i = 0;
    for(i = 0; i < V3OP_LOOP_ARRAY_COUNT; i++)
    {
        if(v3op_loop_array_[i].indata != NULL)
        {
//            free(v3op_loop_array_[i].indata);
        }
        v3op_loop_array_[i].indata = NULL;
        v3op_loop_array_[i].addr = 0xFFFF;
        v3op_loop_array_[i].active = 1;
    }
}

int16_t V3OP_HalInterfaceClear() {
    V3OP_UNIMP_RET0;
}

int16_t V3OP_HalInterfaceInit() {
    V3OP_UNIMP_RET0;
}

void V3OP_DcdcInterfaceClear() {
    V3OP_UNIMP;
}

int16_t V3OP_DcdcInterfaceInit() {
    V3OP_UNIMP_RET0;
}

int16_t V3OP_ComInterfaceInit() {
    V3OP_UNIMP_RET0;
}

void V3OP_ComInterfaceClear() {
    V3OP_UNIMP;
}

int16_t V3OP_HalFpgaUpdate() {
    V3OP_UNIMP_RET0;
}

uint8_t V3OP_DcdcCrcOk() {
    return 1;
}

uint8_t V3OP_ComChannelCrcOk() {
    return 1;
}

void V3OP_HwReset() {
    V3OP_UNIMP;
}

int16_t V3OP_CoreFlashFunctionInit(const uint8_t* payload) {
    V3OP_UNIMP_RET0;
}

int16_t V3OP_CoreFlashFunctionErase(const uint8_t* payload) {
    V3OP_UNIMP_RET0;
}

int16_t V3OP_CoreFlashFunctionWrite(const uint8_t* payload, uint16_t v30p_stream_flags_) {
    V3OP_UNIMP_RET0;
}

int16_t V3OP_CoreFlashFunctionRead(const uint8_t* payload) {
    V3OP_UNIMP_RET0;
}

void V3OP_UpCore() {
    V3OP_UNIMP;
}

int16_t dcdc_calibrate(uint16_t resistors[5], uint16_t resCount, uint16_t vcc) {
    return 0;
//    int16_t success = 0;
//    uint16_t count = CALIB_COUNT;
//    uint32_t internalTicks = 0;
//    uint32_t internalTime = 0;
//    uint16_t currentResCount = 0;
//
//    if((savedCalibrationValues.vcc == vcc) && (savedCalibrationValues.valid ==  1))
//    {
//        return 0;
//    }
//
//    savedCalibrationValues.vcc = vcc;
//    savedCalibrationValues.valid = 0;
//
//    for(currentResCount = 0; currentResCount < resCount; currentResCount++)
//    {
//        uint16_t resistor = resistors[currentResCount];
//
//        if (resistor == 0)
//        {
//            count = CALIB_COUNT*5;
//        }
//
//        if (resistor == 4)
//        {
//            count = CALIB_COUNT*2;
//        }
//
//        //  switch off Overcurrent detection
//        P5OUT |= BIT7;
//
//        success = dcdc_Send(CMD_CALLOAD, resistor);
//
//        if(!success)
//        {
//            return -1;
//        }
//        // Wait for some time
//        for(volatile uint32_t j = 0; j < DELAYCOUNT; ++j);
//        for(volatile uint32_t j = 0; j < DELAYCOUNT; ++j);
//        for(volatile uint32_t j = 0; j < DELAYCOUNT; ++j);
//        for(volatile uint32_t j = 0; j < DELAYCOUNT; ++j);
//
//        dcdc_RunCalibrate(&internalTicks, &internalTime, count);
//
//        savedCalibrationValues.resValues[currentResCount].ticks = internalTicks;
//        savedCalibrationValues.resValues[currentResCount].time = internalTime;
//        savedCalibrationValues.resValues[currentResCount].resistor = resistor;
//
//        // Reset the calibration resistor
//        success = dcdc_Send(CMD_CALLOAD, 0);
//        for(volatile uint32_t j = 0; j < DELAYCOUNT; ++j);
//
//        //  switch on Overcurrent detection
//        P5OUT &= ~BIT7;
//    }
//    if(!success)
//    {
//        savedCalibrationValues.valid = 0;
//        return -1;
//    }
//
//    savedCalibrationValues.valid = 1;
//    return 0;
}

void dcdc_getCalibrationValues(uint16_t vcc, uint16_t resistor, uint16_t resCount, uint32_t *ticks, uint32_t *time)
{
    uint16_t currentResCount = 0;

    for(currentResCount = 0; currentResCount < resCount; currentResCount++)
    {
        *time = 0;
        *ticks = 0;
//        if(savedCalibrationValues.resValues[currentResCount].resistor == resistor)
//        {
//            *time = savedCalibrationValues.resValues[currentResCount].time;
//            *ticks = savedCalibrationValues.resValues[currentResCount].ticks;
//        }
    }
}

int16_t V3OP_Calibrate(const uint8_t* str) {
    // Calibration values retreived form the DCDC
    uint32_t ticks;
    uint32_t time;
    // Start DCDC mcu calibaration function
    STREAM_out_init(str[MESSAGE_MSG_ID_POS], RESPTYP_DATA);

    uint16_t countRes = *(uint16_t*)&str[4];
    uint16_t res[5] = {0,0,0,0,0};
    uint16_t vcc = 0;
    int16_t ret_value = -1;
    // Resistors
    int16_t i, pos;
    for (i = 0, pos = 6; i < countRes; ++i, pos += 2)
    {
        res[i] = *(uint16_t*)&str[pos];
    }
    vcc = *(uint16_t*)&str[pos];

    ret_value = dcdc_calibrate(res, countRes, vcc);
    if(!ret_value)
    {
        int16_t y = 0;
        for(y =0; y < countRes; y++)
        {
            dcdc_getCalibrationValues(vcc, res[y], countRes, &ticks, &time);
            STREAM_put_long(ticks);
            STREAM_put_long(time);
        }
        STREAM_flush();
        ret_value = MESSAGE_NO_RESPONSE;
    }
    else
    {
        STREAM_put_long(0xDEAD);
        STREAM_put_long(0xBABE);
        STREAM_flush();
        ret_value = MESSAGE_NO_RESPONSE;
    }
    return ret_value;
}

uint8_t V3OP_HalCrcOk() {
    return 1;
}

uint8_t V3OP_HilCrcOk() {
    return 1;
}

uint16_t V3OP_GetHilCrc() {
    return 0x6153; // Determined empirically by printing `expectedHilCrc` in UpdateManagerFet.cpp
}

uint16_t V3OP_GetHalCrc() {
    return 0xF70B; // Determined empirically by printing `expectedHalCrc` in UpdateManagerFet.cpp
}

uint16_t V3OP_GetDcdcCrc() {
    return 0xF66F; // Determined empirically by printing `expectedDcdcCrc` in UpdateManagerFet.cpp
}

uint16_t V3OP_GetCoreCrc() {
    return 0x7aaa; // Determined empirically by printing `expectedCoreCrc` in UpdateManagerFet.cpp
}

uint16_t V3OP_GetComChannelCrc() {
    return 0x8457; // Determined empirically by printing `expectedFetComChannelCRC` in UpdateManagerFet.cpp
}

int16_t V3OP_Rx(uint8_t *str)
{
    uint8_t tmp_char;
    uint16_t call_addr;
    int16_t ret_value = -1;
    //int16_t ret_value = 0x8000;
	int16_t ret_value_tmp = 0;
    HalFuncInOut pCallAddr;

    if((bios_rx_record_.last_cmd_typ != str[MESSAGE_CMDTYP_POS]) || !(bios_rx_record_.last_msg_id & 0x80))
    {
      bios_rx_record_.last_cmd_typ = str[MESSAGE_CMDTYP_POS];
      bios_rx_record_.last_msg_call = str[MESSAGE_EXECUTE_CALL_ADDR_POS];
      v30p_stream_flags_ |= MESSAGE_NEW_MSG;
    }
    else
    {
        v30p_stream_flags_ &= ~MESSAGE_NEW_MSG;
    }
    if(!(str[MESSAGE_MSG_ID_POS] & 0x80))
    {
        v30p_stream_flags_ |= MESSAGE_LAST_MSG;
    }
    else
    {
        v30p_stream_flags_ &= ~MESSAGE_LAST_MSG;
    }
    bios_rx_record_.last_msg_id = str[MESSAGE_MSG_ID_POS];

    if(!bios_wb_control_)
    {
        BIOS_LedOn(BIOS_LED_MODE);
        bios_wb_control_ = 1;
    }
    if(str[MESSAGE_CMDTYP_POS] == CMDTYP_EXECUTE) // bypass CMDTYP_EXECUTE at switch instruction
    {
        if(v30p_stream_flags_ & MESSAGE_NEW_MSG)
        {
            call_addr = *(uint16_t*)&str[MESSAGE_EXECUTE_CALL_ADDR_POS]; // first int16_t is on a word boundary
            STREAM_discard_bytes(2);
        }
        else
        {
            call_addr = bios_rx_record_.last_msg_call;
        }
        if(call_addr == 0)
        {
            ret_value_tmp = HAL_Zero(str);
        }
        if(!ret_value_tmp)
        {
            // adjust to payload for HAL (payload without cmd-index)
            STREAM_out_init(str[MESSAGE_MSG_ID_POS], RESPTYP_DATA);
            #ifdef HAL_HISTORY
            if(hal_history_count_ >= sizeof(hal_history_)/2)
            {
                hal_history_count_=0;
            }
            hal_history_[hal_history_count_++] = call_addr;
            hal_history_last_ = call_addr;
            #endif
            if(call_addr < std::size(HALFns))
            {
                pCallAddr = HALFns[call_addr];
                if(pCallAddr != NULL)
                {
                    ret_value = (this->*pCallAddr)(v30p_stream_flags_);
                    if(!ret_value)
                    {
                        STREAM_flush();
                        ret_value = MESSAGE_NO_RESPONSE;
                    }
                }
            }
        }
        else
        {
            ret_value = ret_value_tmp;
        }
    }
    else
    {
        switch(str[MESSAGE_CMDTYP_POS])
        {
            case CMDTYP_EXECUTELOOP:
                ret_value = V3OP_SetLoop(str, V3OP_LOOP_WAIT_FLAG);
                break;
            case CMDTYP_EXECUTEEVER:
                ret_value = V3OP_SetLoop(str, 0);
                break;
            case CMDTYP_KILL:
                ret_value = V3OP_KillLoop(str[MESSAGE_EXECUTE_CALL_ADDR_POS+2]);
                break;
           case CMDTYP_KILL_ALL:
                V3OP_KillAllLoops();
                ret_value = 1;
                break;
            case CMDTYP_PAUSE_LOOP:
                {
                    uint8_t msgId = str[MESSAGE_EXECUTE_CALL_ADDR_POS+2];
                    ret_value = V3OP_PauseLoop( msgId | 0x40 );
                }
                break;
            case CMDTYP_RESUME_LOOP:
                {
                    uint8_t msgId = str[MESSAGE_EXECUTE_CALL_ADDR_POS+2];
                    ret_value = V3OP_ResumeLoop( msgId | 0x40 );
                }
                break;
            case CMDTYP_UPINIT:   // init update
                ret_value = V3OP_CoreFlashFunctionInit(str);
                break;
            case CMDTYP_UPERASE:  // erase HAL part
                ret_value = V3OP_CoreFlashFunctionErase(str);
                if(ret_value == 1) // tell Bios that Hal is dead
                {
                    V3OP_HalInterfaceClear();
                }
                break;
            case CMDTYP_UPWRITE:  // write into HAL part
                ret_value = V3OP_CoreFlashFunctionWrite(str,v30p_stream_flags_);
                break;
            case CMDTYP_UPREAD:   // read HAL part
                ret_value = V3OP_CoreFlashFunctionRead(str);
                break;
            case CMDTYP_UPCORE:
                // Erase Segment with Core Signature, to start install new core on restart
                V3OP_UpCore();
                break;
            case CMDTYP_DCDC_RESTART:
                // restart sub mcu
                ret_value =  dcdc_Restart(*(uint16_t*)&str[4]);
                break;
            case CMDTYP_DCDC_CALIBRATE:
                // Start DCDC mcu calibaration function
                ret_value = V3OP_Calibrate(str);
               break;
            case CMDTYP_DCDC_INIT_INTERFACE:
                V3OP_DcdcInterfaceInit();
                break;
            case CMDTYP_DCDC_SUB_MCU_VERSION:
                ret_value = dcdc_getSubMcuVersion();
                break;
            case CMDTYP_DCDC_LAYER_VERSION:
                ret_value = dcdc_getLayerVersion();
                break;
            case CMDTYP_DCDC_POWER_DOWN:
                ret_value = dcdc_powerDown();
                break;
            case CMDTYP_DCDC_SET_VCC:
                STREAM_out_init(str[MESSAGE_MSG_ID_POS], RESPTYP_DATA);
                ret_value = dcdc_setVcc(*(uint16_t*)&str[6]);
                break;
            case CMDTYP_OVER_CURRENT:
                {
                    uint16_t state = 0;
                    STREAM_out_init(str[MESSAGE_MSG_ID_POS], RESPTYP_DATA);
                    state = (*(uint16_t*)&str[6]);
                    if(state)
                    {
                        IccMonitor_StartVoltageSupervision();
                    }
                    else
                    {
                        IccMonitor_StopVoltageSupervision();
                    }
                    break;
                }
            case CMDTYP_HIL_SET_VCC:
              {
                    STREAM_out_init(str[MESSAGE_MSG_ID_POS], RESPTYP_DATA);
                    {
                        ret_value = IHIL_SetVcc(*(uint16_t*)&str[6]);
                    }
                    break;
              }
            case CMDTYP_HIL_GET_VCC:
              {
                    double vcc = 0;
                    double ext_vcc = 0;
                    {
                        ret_value = IHIL_GetVcc(&vcc, &ext_vcc);
                    }
                    STREAM_out_init(str[MESSAGE_MSG_ID_POS], RESPTYP_DATA);
                    STREAM_put_word((uint16_t)vcc);
                    STREAM_put_word((uint16_t)ext_vcc);
                    STREAM_flush();
                    break;
              }

            case CMDTYP_HIL_SWITCH_FET:
              {
                    STREAM_out_init(str[MESSAGE_MSG_ID_POS], RESPTYP_DATA);
                    {
                        IHIL_SwitchVccFET(*(uint16_t*)&str[6]);
                    }
                    ret_value = 0;
                    break;
              }
          case CMDTYP_CMP_VERSIONS:
              {
                    STREAM_out_init(str[MESSAGE_MSG_ID_POS], RESPTYP_DATA);
                    if(Bios_getHal_signature() == 0xBEEFBEEF && V3OP_HalCrcOk() &&
                    Bios_getHil_signature() == 0xF00DF00D && V3OP_HilCrcOk())
                    {
                        STREAM_put_word(SWCMP_0);
                        STREAM_put_word(SWCMP_1);
                        STREAM_put_word(HIL_VersionCMP);
                    }
                    else
                    {
                        STREAM_put_word(0);
                        STREAM_put_word(0);
                        STREAM_put_word(0);
                    }
                    STREAM_put_word((uint16_t) dcdc_getLayerVersionCmp());
                    STREAM_put_word((uint16_t) com_getLayerVersionCmp());
                    STREAM_flush();

                    ret_value = 0;
                    break;
              }
         }
    }

    if(ret_value != MESSAGE_NO_RESPONSE)
    {
        if(ret_value >= 0)
        {
            if(STREAM_out_init(str[MESSAGE_MSG_ID_POS], RESPTYP_ACKNOWLEDGE) >= 0)
            {
                STREAM_put_word(ret_value);
                STREAM_flush();
            }
        }
        else
        {
            STREAM_out_init(str[MESSAGE_MSG_ID_POS], RESPTYP_EXCEPTION);
            STREAM_put_word(ret_value);
            if(ret_value == EXCEPTION_MSGID_ERR)
            {
                tmp_char = (bios_rx_record_.last_msg_id + 1) & 0x3F;
                if(!tmp_char)
                {
                    tmp_char++;
                }
                STREAM_put_word(tmp_char);
            }
            STREAM_flush();
        }
    }
    return(0);
}

//int16_t V3OP_Rx(const uint8_t* str) {
//    // Advance to the payload
//    STREAM_discard_bytes(MESSAGE_PAYLOAD_POS);
//    
//    uint8_t tmp_char;
//    uint16_t call_addr;
//    int16_t ret_value = -1;
//	int16_t ret_value_tmp = 0;
//    HALFn pCallAddr;
//
//    if((lastMsg.cmd_typ != str[MESSAGE_CMDTYP_POS]) || !(lastMsg.msg_id & 0x80))
//    {
//      lastMsg.cmd_typ = str[MESSAGE_CMDTYP_POS];
//      lastMsg.msg_call = str[MESSAGE_EXECUTE_CALL_ADDR_POS];
//      streamFlags |= MESSAGE_NEW_MSG;
//    }
//    else
//    {
//        streamFlags &= ~MESSAGE_NEW_MSG;
//    }
//    if(!(str[MESSAGE_MSG_ID_POS] & 0x80))
//    {
//        streamFlags |= MESSAGE_LAST_MSG;
//    }
//    else
//    {
//        streamFlags &= ~MESSAGE_LAST_MSG;
//    }
//    lastMsg.msg_id = str[MESSAGE_MSG_ID_POS];
//
//    if(str[MESSAGE_CMDTYP_POS] == CMDTYP_EXECUTE) // bypass CMDTYP_EXECUTE at switch instruction
//    {
//        if(streamFlags & MESSAGE_NEW_MSG)
//        {
//            call_addr = *(uint16_t*)&str[MESSAGE_EXECUTE_CALL_ADDR_POS]; // first int16_t is on a word boundary
//            STREAM_discard_bytes(2);
//        }
//        else
//        {
//            call_addr = lastMsg.msg_call;
//        }
//        if(call_addr == 0)
//        {
//            ret_value_tmp = HAL_Zero(str);
//        }
//        if(!ret_value_tmp)
//        {
//            // adjust to payload for HAL (payload without cmd-index)
//            STREAM_out_init(str[MESSAGE_MSG_ID_POS], RESPTYP_DATA);
//            if(call_addr < std::size(HALFns))
//            {
//                pCallAddr = HALFns[call_addr];
//                if(pCallAddr != NULL)
//                {
//                    ret_value = (this->*pCallAddr)(streamFlags);
//                    if(!ret_value)
//                    {
//                        STREAM_flush();
//                        ret_value = MESSAGE_NO_RESPONSE;
//                    }
//                }
//            }
//        }
//        else
//        {
//            ret_value = ret_value_tmp;
//        }
//    }
//    else
//    {
//        switch(str[MESSAGE_CMDTYP_POS])
//        {
//            case CMDTYP_EXECUTELOOP:
//                ret_value = V3OP_SetLoop(str, V3OP_LOOP_WAIT_FLAG);
//                break;
//            case CMDTYP_EXECUTEEVER:
//                ret_value = V3OP_SetLoop(str, 0);
//                break;
//            case CMDTYP_KILL:
//                ret_value = V3OP_KillLoop(str[MESSAGE_EXECUTE_CALL_ADDR_POS+2]);
//                break;
//           case CMDTYP_KILL_ALL:
//                V3OP_KillAllLoops();
//                ret_value = 1;
//                break;
//            case CMDTYP_PAUSE_LOOP:
//                {
//                    uint8_t msgId = str[MESSAGE_EXECUTE_CALL_ADDR_POS+2];
//                    ret_value = V3OP_PauseLoop( msgId | 0x40 );
//                }
//                break;
//            case CMDTYP_RESUME_LOOP:
//                {
//                    uint8_t msgId = str[MESSAGE_EXECUTE_CALL_ADDR_POS+2];
//                    ret_value = V3OP_ResumeLoop( msgId | 0x40 );
//                }
//                break;
//            case CMDTYP_UPINIT:   // init update
//                ret_value = V3OP_CoreFlashFunctionInit(str);
//                break;
//            case CMDTYP_UPERASE:  // erase HAL part
//                ret_value = V3OP_CoreFlashFunctionErase(str);
//                if(ret_value == 1) // tell Bios that Hal is dead
//                {
//                    V3OP_HalInterfaceClear();
//                }
//                break;
//            case CMDTYP_UPWRITE:  // write into HAL part
//                ret_value = V3OP_CoreFlashFunctionWrite(str,streamFlags);
//                break;
//            case CMDTYP_UPREAD:   // read HAL part
//                ret_value = V3OP_CoreFlashFunctionRead(str);
//                break;
//            case CMDTYP_UPCORE:
//                // Erase Segment with Core Signature, to start install new core on restart
//                V3OP_UpCore();
//                break;
//            case CMDTYP_DCDC_RESTART:
//                // restart sub mcu
//                ret_value = dcdc_Restart(*(uint16_t*)&str[4]);
//                break;
//            case CMDTYP_DCDC_CALIBRATE:
//                // Start DCDC mcu calibaration function
//                ret_value = V3OP_Calibrate(str);
//               break;
//            case CMDTYP_DCDC_INIT_INTERFACE:
//                V3OP_DcdcInterfaceInit();
//                break;
//            case CMDTYP_DCDC_SUB_MCU_VERSION:
//                ret_value = dcdc_getSubMcuVersion();
//                break;
//            case CMDTYP_DCDC_LAYER_VERSION:
//                ret_value = dcdc_getLayerVersion();
//                break;
//            case CMDTYP_DCDC_POWER_DOWN:
//                ret_value = dcdc_powerDown();
//                break;
//            case CMDTYP_DCDC_SET_VCC:
//                STREAM_out_init(str[MESSAGE_MSG_ID_POS], RESPTYP_DATA);
//                ret_value = dcdc_setVcc(*(uint16_t*)&str[6]);
//                break;
//            case CMDTYP_OVER_CURRENT:
//                {
//                    uint16_t state = 0;
//                    STREAM_out_init(str[MESSAGE_MSG_ID_POS], RESPTYP_DATA);
//                    state = (*(uint16_t*)&str[6]);
//                    if(state)
//                    {
//                        IccMonitor_StartVoltageSupervision();
//                    }
//                    else
//                    {
//                        IccMonitor_StopVoltageSupervision();
//                    }
//                    break;
//                }
//            case CMDTYP_HIL_SET_VCC:
//              {
//                    STREAM_out_init(str[MESSAGE_MSG_ID_POS], RESPTYP_DATA);
//                    ret_value = IHIL_SetVcc(*(uint16_t*)&str[6]);
//                    break;
//              }
//            case CMDTYP_HIL_GET_VCC:
//              {
//                    double vcc = 0;
//                    double ext_vcc = 0;
//                    ret_value = IHIL_GetVcc(&vcc, &ext_vcc);
//                    STREAM_out_init(str[MESSAGE_MSG_ID_POS], RESPTYP_DATA);
//                    STREAM_put_word((uint16_t)vcc);
//                    STREAM_put_word((uint16_t)ext_vcc);
//                    STREAM_flush();
//                    break;
//              }
//
//            case CMDTYP_HIL_SWITCH_FET:
//              {
//                    STREAM_out_init(str[MESSAGE_MSG_ID_POS], RESPTYP_DATA);
//                    IHIL_SwitchVccFET(*(uint16_t*)&str[6]);
//                    ret_value = 0;
//                    break;
//              }
//          case CMDTYP_CMP_VERSIONS:
//              {
//                    STREAM_out_init(str[MESSAGE_MSG_ID_POS], RESPTYP_DATA);
//                    if(Bios_getHal_signature() == 0xBEEFBEEF && V3OP_HalCrcOk() &&
//                    Bios_getHil_signature() == 0xF00DF00D && V3OP_HilCrcOk())
//                    {
//                        STREAM_put_word(SWCMP_0);
//                        STREAM_put_word(SWCMP_1);
//                        STREAM_put_word(HIL_VersionCMP);
//                    }
//                    else
//                    {
//                        STREAM_put_word(0);
//                        STREAM_put_word(0);
//                        STREAM_put_word(0);
//                    }
//                    STREAM_put_word((uint16_t) dcdc_getLayerVersionCmp());
//                    STREAM_put_word((uint16_t) com_getLayerVersionCmp());
//                    STREAM_flush();
//
//                    ret_value = 0;
//                    break;
//              }
//         }
//    }
//
//    if(ret_value != MESSAGE_NO_RESPONSE)
//    {
//        if(ret_value >= 0)
//        {
//            if(STREAM_out_init(str[MESSAGE_MSG_ID_POS], RESPTYP_ACKNOWLEDGE) >= 0)
//            {
//                STREAM_put_word(ret_value);
//                STREAM_flush();
//            }
//        }
//        else
//        {
//            STREAM_out_init(str[MESSAGE_MSG_ID_POS], RESPTYP_EXCEPTION);
//            STREAM_put_word(ret_value);
//            if(ret_value == EXCEPTION_MSGID_ERR)
//            {
//                tmp_char = (lastMsg.msg_id + 1) & 0x3F;
//                if(!tmp_char)
//                {
//                    tmp_char++;
//                }
//                STREAM_put_word(tmp_char);
//            }
//            STREAM_flush();
//        }
//    }
//    return 0;
//}

bool V3OP_Scheduler(void)
{
    uint8_t loop_array_counter;
    uint8_t rx_queu_counter = 0;
    uint8_t rx_queu_counter_tmp;
    StreamSafe stream_tmp;
    HalFuncInOut pCallAddr;
    bool serviced = false;

    BIOS_ResetCts(); // release CTS line
    {
        // send error messages
        if(BIOS_IsUsbRxError())
        {
            V3OP_SendException(BIOS_getRxError().bios_rx_err_id_, BIOS_getRxError().bios_rx_err_code_, BIOS_getRxError().bios_rx_err_payload_);
            BIOS_ClearUsbRxError();
            BIOS_ResetCts();
        }
        // search for and execute "commands in loop"

        for(loop_array_counter = 0; loop_array_counter < V3OP_LOOP_ARRAY_COUNT; loop_array_counter++)
        {
            if((v3op_loop_array_[loop_array_counter].active) &&
               (v3op_loop_array_[loop_array_counter].addr < std::size(HALFns)))
            {
                serviced = true;
                
                pCallAddr = HALFns[v3op_loop_array_[loop_array_counter].addr];
                if(pCallAddr != NULL)
                {
                    if(STREAM_out_init(v3op_loop_array_[loop_array_counter].msg_id, v3op_loop_array_[loop_array_counter].msg_type) >= 0)
                    {
                        STREAM_internal_stream(&v3op_loop_array_[loop_array_counter].indata[MESSAGE_EXECUTE_PAYLOAD_POS], v3op_loop_array_[loop_array_counter].indata[0]-3, (uint8_t*)0x0001, 0, &stream_tmp);
                        if((this->*pCallAddr)(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG) == 1)
                        {
                            STREAM_flush();
                            if(v3op_loop_array_[loop_array_counter].flags & V3OP_LOOP_WAIT_FLAG)
                            {
                                V3OP_KillLoop(v3op_loop_array_[loop_array_counter].msg_id);
                            }
                        }
                        STREAM_external_stream(&stream_tmp);
                    }
                }
            }
        }

        // test on new messages from dll and execute
        rx_queu_counter_tmp = rx_queu_counter;
        do
        {
            if(bios_rx_record_.state[rx_queu_counter] & BIOS_RX_RDY)
            {
                serviced = true;
                
                BIOS_LedFlash(BIOS_LED_MODE,20);
                STREAM_in_init((BiosRxRecord*)&bios_rx_record_, rx_queu_counter);
                rx_queu_counter_public_ = rx_queu_counter;
                V3OP_Rx(bios_rx_record_.data[rx_queu_counter]);
                rx_queu_counter = rx_queu_counter_public_;
                bios_rx_record_.state[rx_queu_counter] &= ~BIOS_RX_RDY;
                break;
            }
            rx_queu_counter++;
            if(rx_queu_counter >= BIOS_RX_QUEUS)
            {
              rx_queu_counter = 0;
            }
        }
        while(rx_queu_counter_tmp != rx_queu_counter);
    }
    
    return serviced;
}
