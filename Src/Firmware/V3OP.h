#pragma once
#include "BIOS.h"
#include "Stream.h"
#include "HAL.h"

#define V3OP_LOOP_WAIT_FLAG   0x01
#define V3OP_LOOP_ARRAY_COUNT 4

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
typedef HalRec (*HAL_REC_ARRAY)[];

//! \ingroup MODULBIOS
//! \file v3_0p.c
//! \brief process and route the different message types
//! \li forwarding (execute) messages to HAL
//! \li upload of HAL macros
//! \li loop management


//typedef struct FET_USB_INFOS
//{
//    BYTE(*FetUSB_bytesInUSBBuffer)(BYTE intfNum);
//    BYTE(*FetUSB_receiveData)(BYTE* data, WORD size, BYTE intfNum);
//    BYTE(*FetUsb_CdcSendDataInBackground)(BYTE* dataBuf,WORD size,BYTE intfNum,ULONG ulTimeout);
//}FET_USB_INFOS_t;
//
//
//
#define CMD_CONFIGURE               0x01
#define CMD_SETPWM                  0x02
#define CMD_CALLOAD                 0x03
#define CMD_POWERDOWN               0x04

// Defines
#define SDIO_SLAVE_ADDRESS        (0x8200)
#define SDIO_POWERDOWN_KEY        (0x5a5a)
#define SDIO_DELAY_AFTER_SENDING  (1000u)

struct calibrationResistors
{
    uint32_t time;
    uint32_t ticks;
    uint16_t resistor;
};
typedef struct calibrationResistors calibrationResistors_t;

struct calibrationValues
{
    uint16_t vcc;
    calibrationResistors_t resValues[5];
    uint16_t valid;
};
typedef struct calibrationValues calibrationValues_t;

struct DCDC_INFOS
{
    int16_t (*getSubMcuVersion)(void);
    int16_t (*getLayerVersion)(void);
    int16_t (*dcdcCalibrate)(uint16_t resistor[4], uint16_t resCount, uint16_t vcc);
    int16_t (*dcdcPowerDown)(void);
    int16_t (*dcdcSetVcc)(uint16_t vcc);
    int16_t (*dcdcRestart)(uint16_t fetType_);
    void  (*dcdc_getCalibrationValues)(uint16_t vcc, uint16_t resistor,  uint16_t resCount, uint32_t *ticks, uint32_t *time);
    int16_t (*getLayerVersionCmp)(void);
};
typedef struct DCDC_INFOS DCDC_INFOS_t;

void dcdc_Init(DCDC_INFOS_t* dcdcInfos_Pointer);
int16_t dcdc_Calibrate(uint16_t resistor[5], uint16_t resCount, uint16_t vcc);
int16_t dcdc_PowerDown();
int16_t dcdc_SetVcc(uint16_t vcc);
int16_t dcdc_getSubMcuVersion();
int16_t dcdc_getLayerVersion();
int16_t dcdc_getLayerVersionCmp();
int16_t dcdc_Restart(uint16_t fetType_);
int16_t dcdc_Send(uint16_t cmd, uint16_t data);
int16_t dcdc_Receive(uint16_t *data_read);
void dcdc_getCalibrationValues(uint16_t vcc, uint16_t resistor, uint16_t resCount, uint32_t *ticks, uint32_t *time);
//
//
//
//
//#ifdef eZ_FET
//
//    // UART port direction
//    #define uart_CtsOut()       {P2DIR |= BIT7;}    // CTS output
//    #define uart_RtsIn()        {P2DIR &=~ BIT6;}   // RTS input
//
//    // UART port
//    #define uart_SetCtsBit()    {P2OUT |= BIT7;}    // Output
//    #define uart_ClearCtsBit()  {P2OUT &=~ BIT7;}   // Output clear
//    #define uart_RtsRen()       {P2REN |= BIT6;}    // RTS resistor
//    #define uart_CtsRen()       {P2REN |= BIT7;}    // CTS resistor
//
//    #define com_RtsIfgSet()            {P2IFG |= BIT6;}    // RTS Interrupt flag set (required for sending first byte)
//    #define com_RtsIfgClear()          {P2IFG &=~ BIT6;}   // RTS Interrupt flag clear
//    #define com_RtsIe()                {P2IE  |= BIT6;}    // RTS Interrupt enable
//    #define com_RtsInterruptDisable()  {P2IE &= ~BIT6;}    // RTS Interrupt disable
//    #define com_RtsIes()               {P2IES &=~ BIT6;}   // RTS Interrupt edge select (is set when a low-high transition)
//
//    // UART status bits
//    #define uart_RtsStatus()    ((P2IN & BIT6)==BIT6)
//
//    // UART port select
//    #define uart_TxdSel()       {P3SEL |= BIT3;}    // assign P3.3 to UCA0TXD
//    #define uart_RxdSel()       {P3SEL |= BIT4;}    // assign P3.4 to UCA0RXD
//
//
//    // UART Register and bit abstractions
//    #define TX_RX_PORT_SEL    ( P3SEL )
//    #define TX_RX_PORT_DIR    ( P3DIR )
//    #define TX_PORT_BIT       ( BIT3 )
//    #define RX_PORT_BIT       ( BIT4 )
//    #define UCA_CTRL_REGISTER ( UCA0CTL1 )
//    #define UCA_CTRL0_REGISTER ( UCA0CTL0 )
//    #define UCA_BAUD_CTRL_REGISTER ( UCA0BRW )
//    #define UCA_MOD_CTRL_REGISTER  ( UCA0MCTL )
//    #define UCA_IE_REGISTER   ( UCA0IE )
//    #define UCA_STATUS_REGISTER ( UCA0STAT )
//    #define UCA_INTERRUPT_FLAG_REGISTER  ( UCA0IFG )
//    #define UCA_TX_BUF          ( UCA0TXBUF )
//    #define UCA_RX_BUF          ( UCA0RXBUF )
//
//    #define UCA_BR0         ( UCA0BR0 )
//    #define UCA_BR1         ( UCA0BR1 )
//
//    #define RTS_PORT_OUT ( P2OUT )
//    #define RTS_PORT_DIR ( P2DIR )
//    #define RTS_PORT_BIT ( BIT7 )
//
//    #define RTS_PULLDOWN_OUT ( P2OUT )
//    #define RTS_PULLDOWN_REN ( P2REN )
//    #define RTS_PULLDOWN_BIT ( BIT6 )
//    #define rtsSetPullDownDir() (P2DIR &= ~BIT6)
//// CHECKME: Bit is cleared in both cases
//    #define rtsClearPullDownDir() (P2DIR &= ~BIT6)
//    #define RTS_PULLUP_REN ( P2REN )
//    #define RTS_PULLUP_OUT ( P2OUT )
//    #define RTS_PULLUP_BIT ( BIT6 )
//    #define rtsSetPullUpDir() (P2DIR &= ~BIT6)
//
//    #define CTS_PORT_OUT ( P2OUT )
//    #define CTS_PORT_DIR ( P2DIR )
//    #define CTS_PORT_REN ( P2REN )
//    #define CTS_PORT_BIT ( BIT6 )
//#endif
//
//#ifdef MSP_FET
//
//    // UART port direction
//    #define uart_CtsOut()       {P9DIR |= BIT5;}    // CTS output
//    #define uart_RtsIn()        {P2DIR &=~ BIT1;}   // "RTS" input (no RTS on MSPFET)
//
//
//    // UART port
//    #define uart_SetCtsBit()    {P9OUT |= BIT5;}    // Output
//    #define uart_ClearCtsBit()  {P9OUT &=~ BIT5;}   // Output clear
//    #define uart_RtsRen()       {P2REN |= BIT1;}    // RTS resistor
//    #define uart_CtsRen()       {P9REN |= BIT5;}    // CTS resistor
//
//    #define com_RtsIfgSet()            {P2IFG = 0x00; P2IFG |= BIT1;}    // RTS Interrupt flag set (required for sending first byte)
//    #define com_RtsIfgClear()          {P2IFG = 0x00; /*P2IFG &=~ BIT1;*/}   // RTS Interrupt flag clear
//    #define com_RtsIe()                {P2IE  = 0x00; P2IE  |= BIT1;}    // RTS Interrupt enable
//    #define com_RtsInterruptDisable()  {P2IE &= ~BIT1;}    // RTS Interrupt disable
//    #define com_RtsIes()               {P2IES = 0x00; P2IES &=~ BIT1;}   // RTS Interrupt edge select (is set when a low-high transition)
//
//    // UART status bits
//    #define uart_RtsStatus()    ((P2IN & BIT1)==BIT1)
//
//    // UART port select
//    #define uart_TxdSel()       {P8SEL |= BIT3;}    // assign P3.3 to UCA0TXD
//    #define uart_RxdSel()       {P8SEL |= BIT2;}    // assign P3.4 to UCA0RXD
//
//    // UART Register and bit abstractions
//    #define TX_RX_PORT_SEL    ( P8SEL )
//    #define TX_RX_PORT_DIR    ( P8DIR )
//    #define TX_PORT_BIT       ( BIT2 )
//    #define RX_PORT_BIT       ( BIT3 )
//    #define UCA_CTRL_REGISTER ( UCA1CTL1 )
//    #define UCA_CTRL0_REGISTER ( UCA1CTL0 )
//    #define UCA_BAUD_CTRL_REGISTER ( UCA1BRW )
//    #define UCA_MOD_CTRL_REGISTER ( UCA1MCTL )
//    #define UCA_IE_REGISTER   ( UCA1IE )
//    #define UCA_STATUS_REGISTER ( UCA1STAT )
//    #define UCA_INTERRUPT_FLAG_REGISTER  ( UCA1IFG )
//    #define UCA_TX_BUF          ( UCA1TXBUF )
//    #define UCA_RX_BUF          ( UCA1RXBUF )
//
//    #define UCA_BR0         ( UCA1BR0 )
//    #define UCA_BR1         ( UCA1BR1 )
//
//    #define RTS_PORT_OUT ( P2OUT )
//    #define RTS_PORT_DIR ( P2DIR )
//    #define RTS_PORT_BIT ( BIT1 )
//
//    #define RTS_PULLDOWN_OUT ( P2OUT )
//    #define RTS_PULLDOWN_REN ( P2REN )
//    #define RTS_PULLDOWN_BIT ( BIT1 )
//    #define rtsSetPullDownDir() (P2DIR |=  BIT1)
//// CHECKME: Bit is set in both cases
//    #define rtsClearPullDownDir() (P2DIR |=  BIT1)
//    #define RTS_PULLUP_REN ( P2REN )
//    #define RTS_PULLUP_OUT ( P2OUT )
//    #define RTS_PULLUP_BIT ( BIT1 )
//    #define rtsSetPullUpDir() (P2DIR |=  BIT1)
//
//    #define CTS_PORT_OUT ( P9OUT )
//    #define CTS_PORT_DIR ( P9DIR )
//    #define CTS_PORT_REN ( P9REN )
//    #define CTS_PORT_BIT ( BIT5 )
//#endif
//
//#define IO_CONFIG_HIGH_Z_UART           0x0
//#define IO_CONFIG_UART              	0x1
//#define IO_CONFIG_I2C               	0x2
//
//#define COM_SIGNATURE     0xACDCACDCul
//
//#ifdef MSP_FET
//    #define COM_FIFOSIZE      256ul
//#endif
//
//#ifdef eZ_FET
//    #define COM_FIFOSIZE      64ul
//#endif
//
////UART commands
//#define COM_CLOSE 9620ul
//#define UART_NO_HANDSHAKE 9621ul
//#define UART_NO_HANDSHAKE_PARITY_EVEN 9625ul
//#define UART_HANDSHAKE 9622ul
//#define COM_POWER_UP 9623ul
//#define COM_POWER_DOWN 9624ul
//
//#define PARITY_EVEN 1
//#define PARITY_NONE 0
//
//// BSL Commands
//#define BSL_UART_INVOKE_SEQUENCE 9601ul
//#define BSL_UART_SEQUENCE 9602ul
//#define BSL_DISABLE 8001ul
//#define BSL_MAX_DATA_LENGTH  (260*2)
//#define BSL_I2C_INVOKE_SEQUENCE1 100000ul
//#define BSL_I2C_INVOKE_SEQUENCE2 400000ul
//#define BSL_I2C_INVOKE_SEQUENCE3 100001ul
//#define BSL_I2C_INVOKE_SEQUENCE4 400001ul
//#define BSL_I2C_SEQUENCE1 100002ul
//#define BSL_I2C_SEQUENCE2 400002ul
//
//struct COM_INFOS
//{
//    int16_t (*comGetLayerVersion)(void);
//    int16_t (*comConfig)(uint32_t Baudrate, uint32_t MCLK_Frequency, uint16_t);
//    int16_t (*comTransmit)(void);
//    int16_t (*comReceive)(uint16_t character);
//    void  (*comClose)(void);
//    void  (*comSetHil)(edt_common_methods_t*);
//    void  (*comSetDcdc)(DCDC_INFOS_t*);
//    void  (*comSetUSB)(FET_USB_INFOS_t*);
//    void  (*comLoop)(void);
//    int16_t (*comConfigMode)(uint32_t Baudrate);
//    int16_t (*comSetCts)(void);
//    int16_t (*comClearCts)(void);
//    void  (*comSetRts)(void);
//    int16_t (*comGetLayerVersionCmp)(void);
//};
//typedef struct COM_INFOS COM_INFOS_t;
//
//int16_t COM_BASE_GetLayerVersion();
//int16_t COM_BASE_GetLayerVersionCmp();
//int16_t COM_BASE_Config(uint32_t Baudrate, uint32_t MCLK_Frequency,
//                      uint16_t fetType);
//int16_t COM_BASE_Receive(uint16_t character);
//void COM_BASE_Init(COM_INFOS_t* uartInfos_Pointer);
//void COM_BASE_SetHil(edt_common_methods_t* hil_Pointers);
//void COM_BASE_SetDcdc(DCDC_INFOS_t* dcdc_Pointers);
//void COM_BASE_SetUsb(FET_USB_INFOS_t* usb_Pointers);
//void COM_BASE_Close();
//void COM_BASE_Loop();





 // module handling
typedef void *(*DcdcInit)(DCDC_INFOS_t* dcdcInfos_Pointer);
DCDC_INFOS_t dcdcInfos_;
//#pragma required=dcdcInfos_

//typedef void *(*ComInit)(COM_INFOS_t* comInfos_Pointer);


edt_common_methods_t  _edt_Common_Methods_V3OP;
//typedef void (*HilInitGetEdtCommenFunc)(edt_common_methods_t* edt_commen); // * hal_size, void * stream_adr)
//HilInitGetEdtCommenFunc hilEdtCom = NULL;

typedef int16_t (*HalFpgaUpdateFunc)(void);// FPGA update function

int16_t _dummy_int16_tOutVoidIn(void) {return 0;}
void _dummy_voidOutVoidIn(void){};
int16_t  _dummy_comConfig (uint32_t Baudrate, uint32_t MCLK_Frequency, uint16_t toolId){return 0;};
int16_t _dummy_comTransmit(void){return 0;};
int16_t _dummy_comReceive (uint8_t character){return 0;};
uint8_t *_dummy_comGetBuffer (){return 0;};

int16_t _dummy_dcdcCalibrate(uint16_t resistor[4], uint16_t resCount, uint16_t vcc){return 0;}
int16_t _dummy_dcdcSetVcc(uint16_t vcc ){return 0;}
int16_t _dummy_dcdcRestart(uint16_t fetType_){return 0;};
void _dummy_dcdc_getCalibrationValues(uint16_t vcc, uint16_t resistor, uint16_t resCount, uint32_t *ticks, uint32_t *time){return;}

//void _dummy_comSetHil(edt_common_methods_t* dummy){};
//void _dummy_comSetDcdc(DCDC_INFOS_t* dummy){};
//void _dummy_comSetUSB(FET_USB_INFOS_t* dummy){};
//void _dummy_comLoop(void){};
//int16_t _dummy_comConfig2(uint32_t Baudrate){return 0;};

//int16_t _dummy_InitHil( void ){return 0;};
//int16_t _dummy_SetVcc(uint16_t Vcc){return 0;};
//void _dummy_SwitchVccFET(uint16_t state) {return;};
//int16_t _dummy_GetVcc(double* Vcc, double* ExtVcc){return 0;};
//void _dummy_setFpgaTimeOut(uint16_t state) {return;};
//int16_t _dummy_regulateVcc(void) { return 0; }

uint8_t rx_queu_counter_public_;

//COM_INFOS_t comInfos_ =
//{
//    _dummy_int16_tOutVoidIn,
//    _dummy_comConfig,
//    _dummy_comTransmit,
//    _dummy_comReceive,
//    _dummy_voidOutVoidIn,
//    _dummy_comSetHil,
//    _dummy_comSetDcdc,
//    _dummy_comSetUSB,
//    _dummy_comLoop,
//    _dummy_comConfig2,
//    _dummy_comTransmit,
//    _dummy_comTransmit,
//    _dummy_voidOutVoidIn
//};
//#pragma required=comInfos_
//
//FET_USB_INFOS_t fetUsbInfos_ =
//{
//    USBCDC_bytesInUSBBuffer,
//    USBCDC_receiveData,
//    cdcSendDataInBackground,
//};
//#pragma required=fetUsbInfos_

//------------------------------------------------------------------------------
// Global Variables - Data Memory used by this module
//
V3opLoopArray v3op_loop_array_[V3OP_LOOP_ARRAY_COUNT] = {
    { 0xFFFF, (uint8_t*)0xFFFF, 0x00, 0x00, 0x00, 1 },
    { 0xFFFF, (uint8_t*)0xFFFF, 0x00, 0x00, 0x00, 1 },
    { 0xFFFF, (uint8_t*)0xFFFF, 0x00, 0x00, 0x00, 1 },
    { 0xFFFF, (uint8_t*)0xFFFF, 0x00, 0x00, 0x00, 1 }
};

HAL_INFOS no_hal_infos_V3OP_V3OP_ = {NULL, 0, 0, 0, NULL, 0, 0};

HAL_INFOS_PTR hal_infos_V3OP_ = &no_hal_infos_V3OP_V3OP_;
// ID from and pointer to all HAL functions
HAL_REC_ARRAY hal_ptr_ = NULL;
// informations about first and last measages
uint16_t v30p_stream_flags_ = 0;

//int16_t HAL_Zero(uint8_t *data);

void V3OP_HwReset(void)
{
    STREAM_resetSharedVariables();
    BIOS_UsbTxClear();
    BIOS_UsbRxClear();
    BIOS_InitCom();
}

//#pragma optimize = low
// MSPProbeSim: renamed to not conflict with HAL.h `HAL_Zero` define
int16_t HAL_Zero_V3OP(uint8_t *data)
{
    int16_t ret_value = 0;
    uint16_t i;
    if(data[MESSAGE_EXECUTE_ZERO_ADDR_POS] == STREAM_CORE_ZERO_VERSION)  // call for SW version
    {
        if(STREAM_out_init(data[MESSAGE_MSG_ID_POS], RESPTYP_DATA) >= 0)
        {
            uint32_t coreVersion_ToolId = Bios_getCore_version();
            coreVersion_ToolId =(coreVersion_ToolId<<16) + Bios_getTool_id();
            if(Bios_getHal_signature() == 0xBEEFBEEF && V3OP_HalCrcOk() &&
               Bios_getHil_signature() == 0xF00DF00D && V3OP_HilCrcOk())
            {
                STREAM_put_word((*hal_infos_V3OP_).sw_0);
                STREAM_put_word((*hal_infos_V3OP_).sw_1);
            }
            else
            {
                STREAM_put_word(Bios_getTool_id());
                STREAM_put_word(Bios_getTool_id());
            }
            STREAM_put_word(Bios_getInfo_hw_0());
            STREAM_put_word(Bios_getInfo_hw_1());

            STREAM_put_bytes((uint8_t*)&coreVersion_ToolId,sizeof(coreVersion_ToolId));
            STREAM_put_word((*hal_infos_V3OP_).hil_version);

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
            STREAM_put_word((*hal_infos_V3OP_).fpga_version);

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
        if((*hal_infos_V3OP_).hal_size)
        {
            if(STREAM_out_init(data[MESSAGE_MSG_ID_POS], RESPTYP_DATA) >= 0)
            {
                STREAM_put_word((*hal_infos_V3OP_).hal_size);
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
            for(i =0;i<(*hal_infos_V3OP_).hal_size;i++)
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

//! \brief kill all loop functions, including vcc monitor
void V3OP_KillAllLoops(void)
{
    int16_t i = 0;
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
int16_t V3OP_KillLoop(uint8_t msg_id)
{
    uint16_t i;
    int16_t ret_value = -1;

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

uint8_t tempInData[4][100]= {0x0,0x0};

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

//int16_t V3OP_Calibrate(uint8_t *str);

//! \brief process message type
//! \param[in] *str pointer on received buffer
//! \li str[1] message type
//! \li str[2] message id
//! \li str[3] function timeout (not used)
//! \li str[4...] message type (function) depended payload
//! \return 0
int16_t V3OP_Rx (uint8_t *str)
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
            ret_value_tmp = HAL_Zero_V3OP(str);
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
            if(call_addr < (*hal_infos_V3OP_).hal_size)
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
                ret_value =  dcdcInfos_.dcdcRestart(*(uint16_t*)&str[4]);
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
                ret_value = dcdcInfos_.dcdcSetVcc(*(uint16_t*)&str[6]);
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
                    if(_edt_Common_Methods_V3OP.SetVcc)
                    {
                        ret_value = _edt_Common_Methods_V3OP.SetVcc(*(uint16_t*)&str[6]);
                    }
                    break;
              }
            case CMDTYP_HIL_GET_VCC:
              {
                    double vcc = 0;
                    double ext_vcc = 0;
                    if(_edt_Common_Methods_V3OP.GetVcc)
                    {
                        ret_value = _edt_Common_Methods_V3OP.GetVcc(&vcc, &ext_vcc);
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
                    if(_edt_Common_Methods_V3OP.SwitchVccFET)
                    {
                        _edt_Common_Methods_V3OP.SwitchVccFET(*(uint16_t*)&str[6]);
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
                        STREAM_put_word((*hal_infos_V3OP_).swCmp_0);
                        STREAM_put_word((*hal_infos_V3OP_).swCmp_1);
                        STREAM_put_word((uint16_t)(*hal_infos_V3OP_).hil_versionCmp);
                    }
                    else
                    {
                        STREAM_put_word(0);
                        STREAM_put_word(0);
                        STREAM_put_word(0);
                    }
                    STREAM_put_word((uint16_t) dcdcInfos_.getLayerVersionCmp());
                    STREAM_put_word((uint16_t) comInfos_.comGetLayerVersionCmp());
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

int16_t V3OP_Calibrate(uint8_t *str)
{   // Calibration values retreived form the DCDC
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

    ret_value =  dcdcInfos_.dcdcCalibrate(res, countRes, vcc);
    if(!ret_value)
    {
        int16_t y = 0;
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
int16_t V3OP_SendException(uint8_t msg_id, uint16_t code, uint16_t *payload)
{
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

// Sw layer init

//*****************************************
int16_t V3OP_HalInterfaceClear(void)
{
	V3OP_KillAllLoops();
	hal_infos_V3OP_ = &no_hal_infos_V3OP_V3OP_;
    hal_ptr_ = NULL;
    return(0);
}

//typedef void (*HilInitFunc)();

int16_t V3OP_HalInterfaceInit(void)
{
    HalMainFunc halStartUpCode = NULL;
    HilInitFunc hilInit = NULL;

    uint8_t cmd[6] = {0x05, CMDTYP_EXECUTELOOP, 0, 0, 0, 0};
    uint8_t i;

    V3OP_HalInterfaceClear();
    //set shared mem Variables to 0x00
    STREAM_resetSharedVariables();


     // if hil is not loaded - did not make sence to init hal
    if(Bios_getHil_signature() != 0xF00DF00D || Bios_getHil_intvec() == 0xFFFF || !V3OP_HilCrcOk())
    {
         _edt_Common_Methods_V3OP.Init = _dummy_InitHil;
         _edt_Common_Methods_V3OP.SetVcc = _dummy_SetVcc;
         _edt_Common_Methods_V3OP.GetVcc = _dummy_GetVcc;
         _edt_Common_Methods_V3OP.setFpgaTimeOut = _dummy_setFpgaTimeOut;
         _edt_Common_Methods_V3OP.SwitchVccFET = _dummy_SwitchVccFET;
         _edt_Common_Methods_V3OP.regulateVcc = _dummy_regulateVcc;
         _edt_Common_Methods_V3OP.SetToolID = _dummy_SwitchVccFET;
          return -1;
    }
    hilInit = (HilInitFunc)Bios_getHil_intvec();
    // call startup of HIL layer
    hilInit();
    HilInitGetEdtCommenFunc hilEdtCom = (HilInitGetEdtCommenFunc)0x18A0;
    hilEdtCom(&_edt_Common_Methods_V3OP);

    // check if rest vector is not FFFF and if a valid Hal/Programm signature was found
    if(Bios_getHal_intvec() == 0xFFFF || Bios_getHal_signature() != 0xBEEFBEEF || !V3OP_HalCrcOk())
    {
        return -1;
    }

    _edt_Common_Methods_V3OP.SetToolID(Bios_getTool_id());

    halStartUpCode = (HalMainFunc)Bios_getHal_intvec(); // calls the (modified) startup code of HAL
    hal_infos_V3OP_ = halStartUpCode((struct stream_funcs*)&_stream_Funcs, 0, V3OP_HilCrcOk(), V3OP_DcdcCrcOk()); // return HAL sw infos
    hal_ptr_ = (HAL_REC_ARRAY)(*hal_infos_V3OP_).hal_list_ptr;

    IccMonitor_setHilIterface(&_edt_Common_Methods_V3OP);
    comInfos_.comSetHil(&_edt_Common_Methods_V3OP);

    if(hal_ptr_ != NULL)
    {
        //configure ICC monitor process
        (*hal_ptr_)[(*hal_infos_V3OP_).hal_size-1].id = 0xFFFE;
        (*hal_ptr_)[(*hal_infos_V3OP_).hal_size-1].function = (void*)IccMonitor_Process;

        for(i=0; i <(*hal_infos_V3OP_).hal_size; i++)
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
    dcdcInfos_.getSubMcuVersion = _dummy_int16_tOutVoidIn;
    dcdcInfos_.getLayerVersion = _dummy_int16_tOutVoidIn;
    dcdcInfos_.dcdcCalibrate =  _dummy_dcdcCalibrate;
    dcdcInfos_.dcdcPowerDown = _dummy_int16_tOutVoidIn;
    dcdcInfos_.dcdcSetVcc = _dummy_dcdcSetVcc;
    dcdcInfos_.dcdcRestart = _dummy_dcdcRestart;
    dcdcInfos_.dcdc_getCalibrationValues = _dummy_dcdc_getCalibrationValues;
    dcdcInfos_.getLayerVersionCmp = _dummy_int16_tOutVoidIn;
    //dcdcInfos_.getLayerVersion  = _dummy_int16_tOutVoidIn;
    comInfos_.comSetDcdc(&dcdcInfos_);

}

int16_t V3OP_DcdcInterfaceInit(void)
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
int16_t V3OP_ComInterfaceInit(void)
{
    UNIMP_FN();
    return 0;
}

void V3OP_ComInterfaceClear(void)
{
    UNIMP_FN();
}

//*****************************************
int16_t V3OP_HalFpgaUpdate(void)
{
    HalFpgaUpdateFunc halFpgaUpdateCode = NULL;
    int16_t retVal;

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
    uint8_t loop_array_counter;
    uint8_t rx_queu_counter = 0;
    uint8_t rx_queu_counter_tmp;
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
               (v3op_loop_array_[loop_array_counter].addr <  (*hal_infos_V3OP_).hal_size))
            {
                pCallAddr = (HalFuncInOut)(*hal_ptr_)[v3op_loop_array_[loop_array_counter].addr].function;
                if(pCallAddr != NULL)
                {
                    if(STREAM_out_init(v3op_loop_array_[loop_array_counter].msg_id, v3op_loop_array_[loop_array_counter].msg_type) >= 0)
                    {
                        STREAM_internal_stream(&v3op_loop_array_[loop_array_counter].indata[MESSAGE_EXECUTE_PAYLOAD_POS], v3op_loop_array_[loop_array_counter].indata[0]-3, (uint8_t*)0x0001, 0, &stream_tmp);
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
int16_t V3OP_CoreFlashFunctionInit(uint8_t *payload)
{
    int16_t ret_value = -1;

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

//#ifdef MSP_FET
//#pragma optimize = low
//#pragma vector=WDT_VECTOR
//__interrupt void TimeoutFpgaShift_(void)
//{
//    SFRIFG1 &= ~WDTIFG;
//
//    if(STREAM_out_init(0, RESPTYP_STATUS) != EXCEPTION_TX_NO_BUFFER)
//    {
//        STREAM_put_word(FET_FPGA_TIMOUT);
//        STREAM_flush();
//    }
//    _edt_Common_Methods_V3OP.setFpgaTimeOut(1);
//}
//#endif
