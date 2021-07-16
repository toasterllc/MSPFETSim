/*
 * v3_0p.c
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
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

//! \ingroup MODULBIOS
//! \file v3_0p.c
//! \brief process and route the different message types
//! \li forwarding (execute) messages to HAL
//! \li upload of HAL macros
//! \li loop management

#include "fet/v3_0p_hw_fet.h"
#include "fet/bios.h"
#include "v3_0p.h"
#include "stream.h"
#include <stdlib.h>
#include "Fet_IccMonitor.h"
#include "USB_API/USB_Common/types.h"
#include "USB_API/USB_CDC_API/UsbCdc.h"
#include <string.h>
#include "../dcdc/FetDcdc.h"
#include "../Uart/FetCom.h"
#include "FetUsb.h"
#include "usbMain.h"
#include "usbConstructs.h"

 // module handling
typedef void *(*DcdcInit)(DCDC_INFOS_t* dcdcInfos_Pointer);
DCDC_INFOS_t dcdcInfos_;
#pragma required=dcdcInfos_

typedef void *(*ComInit)(COM_INFOS_t* comInfos_Pointer);


edt_common_methods_t  _edt_Common_Methods;
typedef void (*HilInitGetEdtCommenFunc)(edt_common_methods_t* edt_commen); // * hal_size, void * stream_adr)
HilInitGetEdtCommenFunc hilEdtCom = NULL;

typedef short (*HalFpgaUpdateFunc)(void);// FPGA update function

short _dummy_shortOutVoidIn(void) {return 0;}
void _dummy_voidOutVoidIn(void){};
short  _dummy_comConfig (unsigned long Baudrate, unsigned long MCLK_Frequency, unsigned short toolId){return 0;};
short _dummy_comTransmit(void){return 0;};
short _dummy_comReceive (unsigned char character){return 0;};
unsigned char *_dummy_comGetBuffer (){return 0;};

short _dummy_dcdcCalibrate(unsigned short resistor[4], unsigned short resCount, unsigned short vcc){return 0;}
short _dummy_dcdcSetVcc(unsigned short vcc ){return 0;}
short _dummy_dcdcRestart(unsigned short fetType_){return 0;};
void _dummy_dcdc_getCalibrationValues(unsigned short vcc, unsigned short resistor, unsigned short resCount, unsigned long *ticks, unsigned long *time){return;}

void _dummy_comSetHil(edt_common_methods_t* dummy){};
void _dummy_comSetDcdc(DCDC_INFOS_t* dummy){};
void _dummy_comSetUSB(FET_USB_INFOS_t* dummy){};
void _dummy_comLoop(void){};
short _dummy_comConfig2(unsigned long Baudrate){return 0;};

short _dummy_InitHil( void ){return 0;};
short _dummy_SetVcc(unsigned short Vcc){return 0;};
void _dummy_SwitchVccFET(unsigned short state) {return;};
short _dummy_GetVcc(double* Vcc, double* ExtVcc){return 0;};
void _dummy_setFpgaTimeOut(unsigned short state) {return;};
short _dummy_regulateVcc(void) { return 0; }

unsigned char rx_queu_counter_public_;

COM_INFOS_t comInfos_ =
{
    _dummy_shortOutVoidIn,
    _dummy_comConfig,
    _dummy_comTransmit,
    _dummy_comReceive,
    _dummy_voidOutVoidIn,
    _dummy_comSetHil,
    _dummy_comSetDcdc,
    _dummy_comSetUSB,
    _dummy_comLoop,
    _dummy_comConfig2,
    _dummy_comTransmit,
    _dummy_comTransmit,
    _dummy_voidOutVoidIn
};
#pragma required=comInfos_

FET_USB_INFOS_t fetUsbInfos_ =
{
    USBCDC_bytesInUSBBuffer,
    USBCDC_receiveData,
    cdcSendDataInBackground,
};
#pragma required=fetUsbInfos_

//------------------------------------------------------------------------------
// Global Variables - Data Memory used by this module
//
V3opLoopArray v3op_loop_array_[V3OP_LOOP_ARRAY_COUNT] = {
    { 0xFFFF, (unsigned char*)0xFFFF, 0x00, 0x00, 0x00, 1 },
    { 0xFFFF, (unsigned char*)0xFFFF, 0x00, 0x00, 0x00, 1 },
    { 0xFFFF, (unsigned char*)0xFFFF, 0x00, 0x00, 0x00, 1 },
    { 0xFFFF, (unsigned char*)0xFFFF, 0x00, 0x00, 0x00, 1 }
};

HAL_INFOS no_hal_infos_ = {NULL, 0, 0, 0, NULL, 0, 0};

HAL_INFOS_PTR hal_infos_ = &no_hal_infos_;
// ID from and pointer to all HAL functions
HAL_REC_ARRAY hal_ptr_ = NULL;
// informations about first and last measages
unsigned short v30p_stream_flags_ = 0;

short HAL_Zero(unsigned char *data);

void V3OP_HwReset(void)
{
    STREAM_resetSharedVariables();
    BIOS_UsbTxClear();
    BIOS_UsbRxClear();
    BIOS_InitCom();
}

#pragma optimize = low
short HAL_Zero(unsigned char *data)
{
    short ret_value = 0;
    unsigned short i;
    if(data[MESSAGE_EXECUTE_ZERO_ADDR_POS] == STREAM_CORE_ZERO_VERSION)  // call for SW version
    {
        if(STREAM_out_init(data[MESSAGE_MSG_ID_POS], RESPTYP_DATA) >= 0)
        {
            unsigned long coreVersion_ToolId = Bios_getCore_version();
            coreVersion_ToolId =(coreVersion_ToolId<<16) + Bios_getTool_id();
            if(Bios_getHal_signature() == 0xBEEFBEEF && V3OP_HalCrcOk() &&
               Bios_getHil_signature() == 0xF00DF00D && V3OP_HilCrcOk())
            {
                STREAM_put_word((*hal_infos_).sw_0);
                STREAM_put_word((*hal_infos_).sw_1);
            }
            else
            {
                STREAM_put_word(Bios_getTool_id());
                STREAM_put_word(Bios_getTool_id());
            }
            STREAM_put_word(Bios_getInfo_hw_0());
            STREAM_put_word(Bios_getInfo_hw_1());

            STREAM_put_bytes((unsigned char*)&coreVersion_ToolId,sizeof(coreVersion_ToolId));
            STREAM_put_word((*hal_infos_).hil_version);

            if(Bios_getDcdc_signature() == 0xABBAABBA && V3OP_DcdcCrcOk())
            {
                STREAM_put_word(dcdcInfos_.getLayerVersion());
            }
            else
            {
                STREAM_put_word(-1);
            }
            if(Bios_getDcdc_signature() == 0xABBAABBA && V3OP_DcdcCrcOk() && Bios_getTool_id() != eZ_FET_NO_DCDC)
            {
                STREAM_put_word(dcdcInfos_.getSubMcuVersion());

            }
            else
            {
                STREAM_put_word(-1);
            }
             // return dummy value for Uart version module
            if(Bios_getCom_signature() == 0xACDCACDC && V3OP_ComChannelCrcOk())
            {
                STREAM_put_word(comInfos_.comGetLayerVersion());
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
            STREAM_put_word((*hal_infos_).fpga_version);

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
        if((*hal_infos_).hal_size)
        {
            if(STREAM_out_init(data[MESSAGE_MSG_ID_POS], RESPTYP_DATA) >= 0)
            {
                STREAM_put_word((*hal_infos_).hal_size);
                ret_value =  1;
            }
            else
            {
                ret_value = -4;
            }
        }
        else
        {
            ret_value = -2;
        }
    }
    else if(data[MESSAGE_EXECUTE_ZERO_ADDR_POS] == STREAM_CORE_ZERO_MACRO_ADDR)
    {
        if(STREAM_out_init(data[MESSAGE_MSG_ID_POS], RESPTYP_DATA) >= 0)
        {
            for(i =0;i<(*hal_infos_).hal_size;i++)
            {
                STREAM_put_word(i);
                STREAM_put_word((*hal_ptr_)[i].id);
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

short V3OP_PauseLoop(unsigned char msg_id)
{
    if ( msg_id != 0x40 )
    {
        int i = 0;
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

short V3OP_ResumeLoop(unsigned char msg_id)
{
    if ( msg_id != 0x40 )
    {
        int i = 0;
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

//! \brief kill all loop functions, including vcc monitor
void V3OP_KillAllLoops(void)
{
    int i = 0;
    for(i = 0; i < V3OP_LOOP_ARRAY_COUNT; i++)
    {
        if(v3op_loop_array_[i].indata != NULL)
        {
            free(v3op_loop_array_[i].indata);
        }
        v3op_loop_array_[i].indata = NULL;
        v3op_loop_array_[i].addr = 0xFFFF;
        v3op_loop_array_[i].active = 1;
    }
}

//! \brief kill (stop) functions in loop
//! \param[in] addr index of function to kill
//! \param[in] id with there the function was install in the loop
//! \details if addr and id are zero, all function in loop are killed
//! \return  0 -> no function found in loop to kill (only posible if addr and id 0)
//! \return >0 -> count of functions in loop who killed
//! \return <0 -> no function to kill
short V3OP_KillLoop(unsigned char msg_id)
{
    unsigned short i;
    short ret_value = -1;

    if(msg_id == 0)
    {
        ret_value = 0;
        for(i = 0; i < V3OP_LOOP_ARRAY_COUNT; i++)
        {
            if((v3op_loop_array_[i].addr != 0xFFFF) && ((*hal_ptr_)[v3op_loop_array_[i].addr].id < 0xFF00))
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
                free(v3op_loop_array_[i].indata);
            }
            v3op_loop_array_[i].indata = NULL;
            v3op_loop_array_[i].addr = 0xFFFF;
            v3op_loop_array_[i].active = 0;
        }
        ret_value = 1;
    }
    return(ret_value);
}

//! \brief install a HAL function in loop
//! \param[in] *addr id (addr) to receive buffer
//! \param[in] flags 0 -> function in loop until kill
//! \param[in] flags 1 -> function in loop until function returns with !=0
//! \return <0 -> function not install
//! \return  1 -> function install in loop

unsigned static char tempInData[4][100]= {0x0,0x0};

short V3OP_SetLoop(unsigned char *payload_incl_addr, unsigned char flags)
{
    unsigned short i, j;
    unsigned char payload_size;
    short ret_value = -1;

    // search for same running function
    for(i = 0; i < V3OP_LOOP_ARRAY_COUNT; i++)
    {
        if((v3op_loop_array_[i].addr == *(unsigned short*)&payload_incl_addr[MESSAGE_EXECUTE_CALL_ADDR_POS]) &&
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
            v3op_loop_array_[i].indata = tempInData[i];// (unsigned char*)malloc(payload_size);
            if(v3op_loop_array_[i].indata)
            {
                for(j = 0; j < payload_size; j++)
                {
                    v3op_loop_array_[i].indata[j] = payload_incl_addr[j];
                }
                v3op_loop_array_[i].flags = flags & V3OP_LOOP_WAIT_FLAG;
                v3op_loop_array_[i].msg_id = payload_incl_addr[2] | 0x40;
                v3op_loop_array_[i].msg_type = RESPTYP_DATA;
                v3op_loop_array_[i].addr = *(unsigned short*)&payload_incl_addr[MESSAGE_EXECUTE_CALL_ADDR_POS];
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
            v3op_loop_array_[i].addr = *(unsigned short*)&payload_incl_addr[MESSAGE_EXECUTE_CALL_ADDR_POS];
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

short V3OP_Calibrate(unsigned char *str);

//! \brief process message type
//! \param[in] *str pointer on received buffer
//! \li str[1] message type
//! \li str[2] message id
//! \li str[3] function timeout (not used)
//! \li str[4...] message type (function) depended payload
//! \return 0
short V3OP_Rx (unsigned char *str)
{
    unsigned char tmp_char;
    unsigned short call_addr;
    short ret_value = -1;
    //short ret_value = 0x8000;
	short ret_value_tmp = 0;
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
            call_addr = *(unsigned short*)&str[MESSAGE_EXECUTE_CALL_ADDR_POS]; // first short is on a word boundary
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
            if(call_addr < (*hal_infos_).hal_size)
            {
                pCallAddr = (HalFuncInOut)(*hal_ptr_)[call_addr].function;
                if(pCallAddr != NULL)
                {
                    ret_value = pCallAddr(v30p_stream_flags_);
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
                    unsigned char msgId = str[MESSAGE_EXECUTE_CALL_ADDR_POS+2];
                    ret_value = V3OP_PauseLoop( msgId | 0x40 );
                }
                break;
            case CMDTYP_RESUME_LOOP:
                {
                    unsigned char msgId = str[MESSAGE_EXECUTE_CALL_ADDR_POS+2];
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
                ret_value =  dcdcInfos_.dcdcRestart(*(unsigned short*)&str[4]);
                break;
            case CMDTYP_DCDC_CALIBRATE:
                // Start DCDC mcu calibaration function
                ret_value = V3OP_Calibrate(str);
               break;
            case CMDTYP_DCDC_INIT_INTERFACE:
                V3OP_DcdcInterfaceInit();
                break;
            case CMDTYP_DCDC_SUB_MCU_VERSION:
                ret_value = dcdcInfos_.getSubMcuVersion();
                break;
            case CMDTYP_DCDC_LAYER_VERSION:
                ret_value = dcdcInfos_.getLayerVersion();
                break;
            case CMDTYP_DCDC_POWER_DOWN:
                ret_value = dcdcInfos_.dcdcPowerDown();
                break;
            case CMDTYP_DCDC_SET_VCC:
                STREAM_out_init(str[MESSAGE_MSG_ID_POS], RESPTYP_DATA);
                ret_value = dcdcInfos_.dcdcSetVcc(*(unsigned short*)&str[6]);
                break;
            case CMDTYP_OVER_CURRENT:
                {
                    unsigned short state = 0;
                    STREAM_out_init(str[MESSAGE_MSG_ID_POS], RESPTYP_DATA);
                    state = (*(unsigned short*)&str[6]);
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
                    if(_edt_Common_Methods.SetVcc)
                    {
                        ret_value = _edt_Common_Methods.SetVcc(*(unsigned short*)&str[6]);
                    }
                    break;
              }
            case CMDTYP_HIL_GET_VCC:
              {
                    double vcc = 0;
                    double ext_vcc = 0;
                    if(_edt_Common_Methods.GetVcc)
                    {
                        ret_value = _edt_Common_Methods.GetVcc(&vcc, &ext_vcc);
                    }
                    STREAM_out_init(str[MESSAGE_MSG_ID_POS], RESPTYP_DATA);
                    STREAM_put_word((unsigned short)vcc);
                    STREAM_put_word((unsigned short)ext_vcc);
                    STREAM_flush();
                    break;
              }

            case CMDTYP_HIL_SWITCH_FET:
              {
                    STREAM_out_init(str[MESSAGE_MSG_ID_POS], RESPTYP_DATA);
                    if(_edt_Common_Methods.SwitchVccFET)
                    {
                        _edt_Common_Methods.SwitchVccFET(*(unsigned short*)&str[6]);
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
                        STREAM_put_word((*hal_infos_).swCmp_0);
                        STREAM_put_word((*hal_infos_).swCmp_1);
                        STREAM_put_word((unsigned short)(*hal_infos_).hil_versionCmp);
                    }
                    else
                    {
                        STREAM_put_word(0);
                        STREAM_put_word(0);
                        STREAM_put_word(0);
                    }
                    STREAM_put_word((unsigned short) dcdcInfos_.getLayerVersionCmp());
                    STREAM_put_word((unsigned short) comInfos_.comGetLayerVersionCmp());
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

short V3OP_Calibrate(unsigned char *str)
{   // Calibration values retreived form the DCDC
    unsigned long ticks;
    unsigned long time;
    // Start DCDC mcu calibaration function
    STREAM_out_init(str[MESSAGE_MSG_ID_POS], RESPTYP_DATA);

    unsigned short countRes = *(unsigned short*)&str[4];
    unsigned short res[5] = {0,0,0,0,0};
    unsigned short vcc = 0;
    short ret_value = -1;
    // Resistors
    int i, pos;
    for (i = 0, pos = 6; i < countRes; ++i, pos += 2)
    {
        res[i] = *(unsigned short*)&str[pos];
    }
    vcc = *(unsigned short*)&str[pos];

    ret_value =  dcdcInfos_.dcdcCalibrate(res, countRes, vcc);
    if(!ret_value)
    {
        int y = 0;
        for(y =0; y < countRes; y++)
        {
            dcdcInfos_.dcdc_getCalibrationValues(vcc, res[y], countRes, &ticks, &time);
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

//! \brief sends exceptions with details in payload
//! \param[in] msg_id message id
//! \param[in] code if payload = NULL, 'code' is sendet in as payload
//! \param[in] *payload if !=NULL payload is sendet with the length 'code'
//! \return 0 exception sended
//! \return -1 exception not sended, no output buffer aviable
short V3OP_SendException(unsigned char msg_id, unsigned short code, unsigned short *payload)
{
    unsigned short i;

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

// Sw layer init

//*****************************************
short V3OP_HalInterfaceClear(void)
{
	V3OP_KillAllLoops();
	hal_infos_ = &no_hal_infos_;
    hal_ptr_ = NULL;
    return(0);
}

typedef void (*HilInitFunc)();

short V3OP_HalInterfaceInit(void)
{
    HalMainFunc halStartUpCode = NULL;
    HilInitFunc hilInit = NULL;

    unsigned char cmd[6] = {0x05, CMDTYP_EXECUTELOOP, 0, 0, 0, 0};
    unsigned char i;

    V3OP_HalInterfaceClear();
    //set shared mem Variables to 0x00
    STREAM_resetSharedVariables();


     // if hil is not loaded - did not make sence to init hal
    if(Bios_getHil_signature() != 0xF00DF00D || Bios_getHil_intvec() == 0xFFFF || !V3OP_HilCrcOk())
    {
         _edt_Common_Methods.Init = _dummy_InitHil;
         _edt_Common_Methods.SetVcc = _dummy_SetVcc;
         _edt_Common_Methods.GetVcc = _dummy_GetVcc;
         _edt_Common_Methods.setFpgaTimeOut = _dummy_setFpgaTimeOut;
         _edt_Common_Methods.SwitchVccFET = _dummy_SwitchVccFET;
         _edt_Common_Methods.regulateVcc = _dummy_regulateVcc;
         _edt_Common_Methods.SetToolID = _dummy_SwitchVccFET;
          return -1;
    }
    hilInit = (HilInitFunc)Bios_getHil_intvec();
    // call startup of HIL layer
    hilInit();
    hilEdtCom = (HilInitGetEdtCommenFunc)0x18A0;
    hilEdtCom(&_edt_Common_Methods);

    // check if rest vector is not FFFF and if a valid Hal/Programm signature was found
    if(Bios_getHal_intvec() == 0xFFFF || Bios_getHal_signature() != 0xBEEFBEEF || !V3OP_HalCrcOk())
    {
        return -1;
    }

    _edt_Common_Methods.SetToolID(Bios_getTool_id());

    halStartUpCode = (HalMainFunc)Bios_getHal_intvec(); // calls the (modified) startup code of HAL
    hal_infos_ = halStartUpCode((struct stream_funcs*)&_stream_Funcs, 0, V3OP_HilCrcOk(), V3OP_DcdcCrcOk()); // return HAL sw infos
    hal_ptr_ = (HAL_REC_ARRAY)(*hal_infos_).hal_list_ptr;

    IccMonitor_setHilIterface(&_edt_Common_Methods);
    comInfos_.comSetHil(&_edt_Common_Methods);

    if(hal_ptr_ != NULL)
    {
        //configure ICC monitor process
        (*hal_ptr_)[(*hal_infos_).hal_size-1].id = 0xFFFE;
        (*hal_ptr_)[(*hal_infos_).hal_size-1].function = (void*)IccMonitor_Process;

        for(i=0; i <(*hal_infos_).hal_size; i++)
        {
            if(((*hal_ptr_)[i].id != 0xFFFF) && ((*hal_ptr_)[i].id >= 0xFF00))
            {
                cmd[4] = i;
                cmd[5] = 0;
                V3OP_SetLoop(cmd, 0);
                break;
            }
        }
    }
    return 0;
}

void V3OP_DcdcInterfaceClear(void)
{
    dcdcInfos_.getSubMcuVersion = _dummy_shortOutVoidIn;
    dcdcInfos_.getLayerVersion = _dummy_shortOutVoidIn;
    dcdcInfos_.dcdcCalibrate =  _dummy_dcdcCalibrate;
    dcdcInfos_.dcdcPowerDown = _dummy_shortOutVoidIn;
    dcdcInfos_.dcdcSetVcc = _dummy_dcdcSetVcc;
    dcdcInfos_.dcdcRestart = _dummy_dcdcRestart;
    dcdcInfos_.dcdc_getCalibrationValues = _dummy_dcdc_getCalibrationValues;
    dcdcInfos_.getLayerVersionCmp = _dummy_shortOutVoidIn;
    //dcdcInfos_.getLayerVersion  = _dummy_shortOutVoidIn;
    comInfos_.comSetDcdc(&dcdcInfos_);

}

short V3OP_DcdcInterfaceInit(void)
{
    DcdcInit dcdc_Init_ = NULL;
    if(Bios_getDcdc_intvec() == 0xFFFF  || Bios_getDcdc_signature() != 0xABBAABBA || !V3OP_DcdcCrcOk())
    {
        V3OP_DcdcInterfaceClear();
        return -1;
    }
    dcdc_Init_ = (DcdcInit)Bios_getDcdc_intvec(); // calls the (modified) startup code of HAL
    dcdc_Init_(&dcdcInfos_);

    comInfos_.comSetDcdc(&dcdcInfos_);
    dcdcInfos_.dcdcRestart(Bios_getTool_id());

    return 0;
}

//******************** Module Implementation **************
short V3OP_ComInterfaceInit(void)
{
    ComInit com_Init_ = NULL;
    if((Bios_getCom_intvec() == 0xFFFF) || (Bios_getCom_signature() != 0xACDCACDC)|| ! V3OP_ComChannelCrcOk())
    {
        V3OP_ComInterfaceClear();
        return -1;
    }
    com_Init_ = (ComInit)Bios_getCom_intvec();
    com_Init_(&comInfos_);

    UsbMain_setComIterface(&comInfos_);
    comInfos_.comSetUSB(&fetUsbInfos_);

    comInfos_.comSetHil(&_edt_Common_Methods);
    comInfos_.comSetDcdc(&dcdcInfos_);

    return 0;
}

void V3OP_ComInterfaceClear(void)
{
    comInfos_.comGetLayerVersion      = _dummy_shortOutVoidIn;
    comInfos_.comGetLayerVersionCmp   = _dummy_shortOutVoidIn;
    comInfos_.comConfig               = _dummy_comConfig;
    comInfos_.comTransmit             = _dummy_comTransmit;
    comInfos_.comReceive              = _dummy_comReceive;
    comInfos_.comClose                = _dummy_voidOutVoidIn;
    comInfos_.comSetHil               = _dummy_comSetHil;
    comInfos_.comSetDcdc              = _dummy_comSetDcdc;
    comInfos_.comSetUSB               = _dummy_comSetUSB;
    comInfos_.comLoop                 = _dummy_comLoop;
    comInfos_.comSetCts               = _dummy_comTransmit;
    comInfos_.comClearCts             = _dummy_comTransmit;
    comInfos_.comSetRts               = _dummy_voidOutVoidIn;
    UsbMain_setComIterface(&comInfos_);
}

//*****************************************
short V3OP_HalFpgaUpdate(void)
{
    HalFpgaUpdateFunc halFpgaUpdateCode = NULL;
    short retVal;

    // check if rest vector is not FFFF and if a valid Hal/Programm signature was found
    if(Bios_getHal_intvec() == 0xFFFF || Bios_getHal_signature() != 0xADACADAC || !V3OP_HalFpgaCrcOk())
    {
        return -1;
    }

    halFpgaUpdateCode = (HalFpgaUpdateFunc)Bios_getHal_intvec(); // calls the (modified) startup code of HAL
    retVal = halFpgaUpdateCode(); // return update status, input parameters are don't care

    return retVal;
}

//******************************************
void V3OP_Scheduler(void)
{
    unsigned char loop_array_counter;
    unsigned char rx_queu_counter = 0;
    unsigned char rx_queu_counter_tmp;
    StreamSafe stream_tmp;
    HalFuncInOut pCallAddr;

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
               (v3op_loop_array_[loop_array_counter].addr <  (*hal_infos_).hal_size))
            {
                pCallAddr = (HalFuncInOut)(*hal_ptr_)[v3op_loop_array_[loop_array_counter].addr].function;
                if(pCallAddr != NULL)
                {
                    if(STREAM_out_init(v3op_loop_array_[loop_array_counter].msg_id, v3op_loop_array_[loop_array_counter].msg_type) >= 0)
                    {
                        STREAM_internal_stream(&v3op_loop_array_[loop_array_counter].indata[MESSAGE_EXECUTE_PAYLOAD_POS], v3op_loop_array_[loop_array_counter].indata[0]-3, (unsigned char*)0x0001, 0, &stream_tmp);
                        if(pCallAddr(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG) == 1)
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
}

// System events
typedef enum FET_UPDATE_CONFIG
{
    FET_INIT_ALL_MODULES = 0,
    FET_DE_INIT_ALL_MODULES,
    FET_DE_INIT_DCDC_MCU,
    FET_FPGA_UPDATE,
    FET_STOP_VOLTAGE_SUPERVISION,
    FET_START_VOLTAGE_SUPERVISION,
} FetUpdateConfig;


//! \brief lock/unlock write/erase to UIF (HAL) flash memory
//! \param[in] *payload pointer to receive buffer
//! \return 0 -> flash write/erase locked
//! \return 1 -> flash write/erase released
//! \return <0 -> error
short V3OP_CoreFlashFunctionInit(unsigned char *payload)
{
    short ret_value = -1;

    if(payload[4] == FET_INIT_ALL_MODULES)
    {
        // init communicaiton to DCDC sub mcu
        V3OP_DcdcInterfaceInit();
        dcdcInfos_.dcdcRestart(Bios_getTool_id());
        // will init hil layer as well if valid
        V3OP_HalInterfaceInit();
        V3OP_ComInterfaceInit();

        BIOS_LedAlternate(0);
        BIOS_LedOff(BIOS_LED_MODE);
        BIOS_LedOn(BIOS_LED_POWER);
        ret_value = 1;
    }
    else if(payload[4] == FET_DE_INIT_ALL_MODULES)//used for HAL HIL DCDC & COM Channel update
    {
        V3OP_KillAllLoops();
        V3OP_HalInterfaceClear();
        STREAM_resetSharedVariables();
        dcdcInfos_.dcdcPowerDown();
        V3OP_ComInterfaceClear();
        V3OP_DcdcInterfaceClear();
        BIOS_LedAlternate(30);
        ret_value = 1;
    }
    else if(payload[4] == FET_DE_INIT_DCDC_MCU) // Just for Sub MCU update don't clear Hal HIl you need it for SBW communication
    {
        V3OP_KillAllLoops();
        STREAM_resetSharedVariables();
        dcdcInfos_.dcdcPowerDown();
        V3OP_DcdcInterfaceClear();
        BIOS_LedAlternate(30);
        ret_value = 1;
    }
    else if(payload[4] == FET_FPGA_UPDATE) // Used for FPGA updates
    {
        V3OP_KillAllLoops();
        V3OP_HalInterfaceClear();
        STREAM_resetSharedVariables();
        dcdcInfos_.dcdcPowerDown();
        V3OP_ComInterfaceClear();
        V3OP_DcdcInterfaceClear();
        IccMonitor_StopDcdcOvercurrentDetection();
        BIOS_LedAlternate(5);
        ret_value = V3OP_HalFpgaUpdate() ? -1 : 1;
    }
    else if(payload[4] == FET_STOP_VOLTAGE_SUPERVISION) // Used for FPGA updates
    {
        V3OP_KillAllLoops();
        IccMonitor_StopVoltageSupervision();
        IccMonitor_StopDcdcOvercurrentDetection();
        BIOS_LedAlternate(30);
        ret_value = 1;
    }
    else if(payload[4] == FET_START_VOLTAGE_SUPERVISION) // Used for FPGA updates
    {
        IccMonitor_StartDcdcOvercurrentDetection();
        BIOS_LedOff(BIOS_LED_MODE);
        BIOS_LedOn(BIOS_LED_POWER);
        //~1.6 s delay
        __delay_cycles(40000000);
        IccMonitor_StartVoltageSupervision();
        __delay_cycles(40000000);
        ret_value = 1;
    }
    return(ret_value);
}

#ifdef MSP_FET
#pragma optimize = low
#pragma vector=WDT_VECTOR
__interrupt void TimeoutFpgaShift_(void)
{
    SFRIFG1 &= ~WDTIFG;

    if(STREAM_out_init(0, RESPTYP_STATUS) != EXCEPTION_TX_NO_BUFFER)
    {
        STREAM_put_word(FET_FPGA_TIMOUT);
        STREAM_flush();
    }
    _edt_Common_Methods.setFpgaTimeOut(1);
}
#endif
