#pragma once

#include "Bios/include/modules.h"
#include "Bios/include/protocol.h"
#include "Bios/include/error_def.h"
#include "Bios/include/ConfigureParameters.h"
#include "Bios/src/fw/fet/FetVersion.h"
#include "Bios/src/fw/fet/communicationDefs.h"
#include "Bios/src/hal/JTAG_defs.h"
#include "Bios/src/hal/EEM_defs.h"

#define FPGA_VERSION 0x016
#define FPGA_SIGNATURE 0xADACADAC

#define BIOS_RX_SIZE        258
#define BIOS_RX_QUEUS       4
#define BIOS_TX_SIZE        258
#define BIOS_TX_QUEUS       1
#define BIOS_TX_FLAGS       1

#define STREAM_CORE_ZERO_VERSION    0x00
#define STREAM_CORE_ZERO_MACRO_SIZE 0x01
#define STREAM_CORE_ZERO_MACRO_ADDR 0x02
#define STREAM_CORE_ZERO_PUC_RESET  0x03

#define HAL_SIGNATURE   0xBEEFBEEFul
#define HIL_SIGNATURE   0xF00DF00Dul
#define DCDC_SIGNATURE  0xABBAABBAul
#define COM_SIGNATURE   0xACDCACDCul

#define INFO_U1_HW_0 0xFF55
#define INFO_U1_HW_1 0x0140

#define MESSAGE_NEW_MSG 0x0001
#define MESSAGE_LAST_MSG 0x0002

#define eZ_FET_WITH_DCDC                0xAAAA
#define eZ_FET_NO_DCDC                  0xAAAB
#define eZ_FET_WITH_DCDC_NO_FLOWCTL     0xAAAC
#define MSP_FET_WITH_DCDC               0xBBBB
#define MSP_FET_WITH_DCDC_V2x           0xBBBC
#define eZ_FET_WITH_DCDC_V2x            0xAAAD

#define TOOL_ID      MSP_FET_WITH_DCDC_V2x

#define COM_CHANNEL 2
#define DEBUG_CHANNEL 0

#define _DINT_FET()
#define _EINT_FET()

#define REQUIRED(x)

#define INLINE(x)

#define MEMBER_FN_PTR(fn)       (&MSPProbeSim::fn)
#define CALL_MEMBER_FN_PTR(fn)  (this->*fn)

const int16_t SW_0 = (VERSION_MAJOR - 1) << 14 | (VERSION_MINOR << 8) | VERSION_PATCH;
const int16_t SW_1 = VERSION_BUILD;

const int16_t SWCMP_0 = (VERSION_MAJOR_CMP - 1) << 14 | (VERSION_MINOR_CMP << 8) | VERSION_PATCH_CMP;
const int16_t SWCMP_1 = VERSION_BUILD_CMP;

const uint16_t HIL_Version = MSP_FET_HIL_VERSION;
const uint16_t HIL_VersionCMP = MSP_FET_HIL_VERSION_CMP;

enum {JTAG = 0, SPYBIWIRE = 1, SPYBIWIREJTAG = 2, JTAGUNDEF = 4, SPYBIWIRE_SUBMCU = 5, SPYBIWIRE_MSP_FET=6, JTAG_432 = 7, SWD_432 = 8};

#define RSTHIGH 0
#define RSTLOW  1

#define Invalid_Manufactor_IdCode 0x000000FEul
#define Mask_Manufactor_IdCode 0x000000FEul

#define  A_VREFPLUS     2.5f         // conversion reference voltage Vref+=2.5V,

#define DCDC_STATUS     (P3IN & (BIT0+BIT1)) // Status bits from the DCDC Sub MCU

// Fuse blow related constants
#define VF_PWM_TIMEOUT  4000
#define VF_PWM_PERIOD   50
#define VF_PWM_ON_TIME  40
#define VF_PWM_OFF_TIME (VF_PWM_PERIOD - VF_PWM_ON_TIME)

const uint16_t ADC_CONV_RANGE = 4096;
const uint16_t ADC_AVERAGE = 1000;

struct jtag
{
  uint8_t  TCK;
  uint8_t  TMS;
  uint8_t  TDI;
  uint8_t  TDO;
  uint8_t* In;
  uint8_t* Out;
  uint8_t RST;
  uint8_t TST;
  uint8_t* DIRECTION;
};

struct vfuse_ctrl
{
  uint8_t  VF2TEST;
  uint8_t  VF2TDI;
  uint8_t  TDI_OFF;
  uint8_t* CTRL;
  uint8_t  PWM_SETVF;
  uint8_t* PWM_CTRL;
};

const struct jtag _Jtag_SubMcu =
{
    // UNIMPLEMENTED
};

const struct jtag _SBW_Back =
{
    // UNIMPLEMENTED
};

const struct jtag _Jtag_Target =
{
    // UNIMPLEMENTED
};

const struct jtag _Jtag_FPGA =
{
    // UNIMPLEMENTED
};

const struct vfuse_ctrl _Msp_Fet =
{
    // UNIMPLEMENTED
};

struct savedDacValues
{
    uint16_t DAC0;
    uint16_t DAC1;
    uint16_t Vcc;
    uint16_t valid;
};
typedef struct savedDacValues savedDacValues_t;

#define VALID_DATA   0x1A
#define SYNC_ONGOING 0x2A
#define INVALID_DATA 0x3A
#define JTAG_LOCKED  0x4A

#define BP_HIT_MASK_J              0x0400000000000000ull
#define LPMX5_MASK_J               0x4000000000000000ull
#define LPM4_1MASK_J               0x8000000000000000ull
#define LPM4_2MASK_J               0x8300000000000000ull
#define EIGHT_JSTATE_BITS          0x100000000000000ull
#define SYNC_BROKEN  0x5A

#define JSTATE_FLOW_CONTROL_BITS   0xC7
#define JSTATE_BP_HIT              0x4
#define JSTATE_SYNC_ONGOING        0x83
#define JSTATE_LOCKED_STATE        0x40
#define JSTATE_INVALID_STATE       0x81
#define JSTATE_LPM_ONE_TWO         0x82
#define JSTATE_LPM_THREE_FOUR      0x80
#define JSTATE_VALID_CAPTURE       0x03
#define JSTATE_LPM_X_FIVE          0xC0

#define JSTATE_SYNC_BROKEN_MASK         0xC3
#define JSTATE_SYNC_BROKEN_PGACT        0x02
#define JSTATE_SYNC_BROKEN_MCLK         0x01
#define JSTATE_SYNC_BROKEN_MCLK_PGACT   0x00

#define FET_TRUE                   0x1
#define FET_FALSE                  0x0

#define L092_MODE 0xA55AA55A
#define C092_MODE 0x5AA55AA5






// RESET
#define FPGA_RESET_BIT			    ( BIT1 )
#define FPGA_RESET_PORT_DIR			( P5DIR )
#define FPGA_RESET_PORT_OUT			( P5OUT )
#define FPGA_RESET_ASSERT           { FPGA_RESET_PORT_OUT &= ~FPGA_RESET_BIT; }
#define FPGA_RESET_DEASSERT         { FPGA_RESET_PORT_OUT |= FPGA_RESET_BIT; }

#define TRIGGER_LA                  { P6OUT |= BIT7; __delay_cycles(10); P6OUT &= ~BIT7; }
#define TRIGGER_ONCE                { P6OUT ^= BIT7; }

// DCDC output to target VCC
#define DCDC_VCC_BIT				( BIT7 )
#define DCDC_VCC_PORT_OUT			( P8OUT )
#define DCDC_VCC_ENABLE				{ DCDC_VCC_PORT_OUT |= DCDC_VCC_BIT; }
#define DCDC_VCC_DISABLE			{ DCDC_VCC_PORT_OUT &= ~DCDC_VCC_BIT; }

// DT voltage to target VCC
#define DT_VCC_BIT				    ( BIT6 )
#define DT_VCC_PORT_OUT				( P9OUT )
#define DT_VCC_ENABLE				{ DT_VCC_PORT_OUT |= DT_VCC_BIT; }
#define DT_VCC_DISABLE				{ DT_VCC_PORT_OUT &= ~DT_VCC_BIT; }

// DT voltage to level shifters
#define DT_SIGNALS_BIT			    ( BIT0 )
#define DT_SIGNALS_PORT_OUT			( P8OUT )
#define DT_SIGNALS_ENABLE			{ DT_SIGNALS_PORT_OUT |= DT_SIGNALS_BIT; }
#define DT_SIGNALS_DISABLE			{ DT_SIGNALS_PORT_OUT &= ~DT_SIGNALS_BIT; }

// SYS_CLK
#define FPGA_SYS_CLK_BIT		    ( BIT7 )
#define FPGA_SYS_CLK_PORT_DIR		( P2DIR )
#define FPGA_SYS_CLK_PORT_SEL		( P2SEL )
#define FPGA_SYS_CLK_START			{ FPGA_SYS_CLK_PORT_SEL |= FPGA_SYS_CLK_BIT; }
#define FPGA_SYS_CLK_STOP			{ FPGA_SYS_CLK_PORT_SEL &= ~FPGA_SYS_CLK_BIT; }

// WR_TRIG
#define FPGA_WR_TRIG_BIT		    ( BIT6 )
#define FPGA_WR_TRIG_PORT_DIR		( P2DIR )
#define FPGA_WR_TRIG_PORT_OUT		( P2OUT )

// RD_TRIG
#define FPGA_RD_TRIG_BIT			( BIT6 )
#define FPGA_RD_TRIG_PORT_DIR		( P5DIR )
#define FPGA_RD_TRIG_PORT_IN		( P5IN )
#define FPGA_RD_TRIG_IS_ASSERTED 	( FPGA_RD_TRIG_PORT_IN & FPGA_RD_TRIG_BIT )

// CMD
#define FPGA_CMD_BITS			    ( BIT2 + BIT3 + BIT4 + BIT5 )
#define FPGA_CMD_PORT_DIR			( P2DIR )
#define FPGA_CMD_PORT_OUT			( P2OUT )

// DATA0
#define FPGA_DATA0_BITS		        ( 0xFF )
#define FPGA_DATA0_PORT_DIR			( P1DIR )
#define FPGA_DATA0_PORT_IN			( P1IN )
#define FPGA_DATA0_PORT_OUT			( P1OUT )

// CMD, DATA0, WR_TRIG
#define FPGA_CMD_DATA0_PORT_OUT     ( PAOUT )

// DATA1
#define FPGA_DATA1_BITS		        ( 0xFFFF )
#define FPGA_DATA1_PORT_DIR			( PBDIR )
#define FPGA_DATA1_PORT_IN			( PBIN )
#define FPGA_DATA1_PORT_OUT			( PBOUT )

// IO_DIR
#define FPGA_IO_DIR_BIT				( BIT1 )
#define FPGA_IO_DIR_PORT_DIR		( P8SEL )
#define FPGA_IO_DIR_PORT_OUT		( P8OUT )

// UART_TXD / UART_TXD (MSP-FET RXD input, JTAG.12)
#define FPGA_UART_TXD_BIT			( BIT3 )
#define FPGA_UART_TXD_PORT_SEL		( P8SEL )

// UART_RXD / UART_RXD (MSP-FET TXD output, JTAG.14)
#define FPGA_UART_RXD_BIT			( BIT2 )
#define FPGA_UART_RXD_PORT_SEL		( P8SEL )
#define FPGA_UART_RXD_PORT_OUT		( P8OUT )
#define FPGA_UART_RXD_HIGH			{ FPGA_UART_RXD_PORT_OUT |= FPGA_UART_RXD_BIT; }
#define FPGA_UART_RXD_LOW			{ FPGA_UART_RXD_PORT_OUT &= ~FPGA_UART_RXD_BIT; }

// UART_CTS / UART_CTS (MSP-FET "clear to send" output, JTAG.10)
#define FPGA_UART_CTS_BIT			( BIT5 )
#define FPGA_UART_CTS_PORT_DIR		( P9DIR )
#define FPGA_UART_CTS_PORT_OUT		( P9OUT )
#define FPGA_UART_CTS_HIGH			{ FPGA_UART_CTS_PORT_OUT |= FPGA_UART_CTS_BIT; }
#define FPGA_UART_CTS_LOW			{ FPGA_UART_CTS_PORT_OUT &= ~FPGA_UART_CTS_BIT; }

// UART_RTR / UART_RTR (MSP-FET "ready to receive" input, JTAG.13)
#define FPGA_UART_RTR_BIT			( BIT1 )
#define FPGA_UART_RTR_PORT_DIR		( P2DIR )
#define FPGA_UART_RTR_PORT_IN		( P2IN )
#define FPGA_UART_RTR_IS_HIGH		( FPGA_UART_RTR_PORT_IN & FPGA_UART_RTR_BIT )

// JTAG signals in bypass mode
#define FPGA_BYPASS_DIR_CTRL_TEST_BIT		( BIT5 )
#define FPGA_BYPASS_DIR_CTRL_RST_BIT		( BIT4 )
#define FPGA_BYPASS_DIR_CTRL_TDO_BIT		( BIT3 )
#define FPGA_BYPASS_DIR_CTRL_TDI_BIT		( BIT2 )
#define FPGA_BYPASS_DIR_CTRL_TMS_BIT		( BIT1 )
#define FPGA_BYPASS_DIR_CTRL_TCK_BIT		( BIT0 )
#define FPGA_BYPASS_DIR_CTRL_PORT_DIR		( P4DIR )
#define FPGA_BYPASS_DIR_CTRL_PORT_OUT		( P4OUT )
#define FPGA_BYPASS_DIR_CTRL_PORT_IN		( P4IN )

#define FPGA_BYPASS_TEST_BIT		( BIT5 )
#define FPGA_BYPASS_RST_BIT			( BIT4 )
#define FPGA_BYPASS_TDO_BIT			( BIT3 )
#define FPGA_BYPASS_TDI_BIT			( BIT2 )
#define FPGA_BYPASS_TMS_BIT			( BIT1 )
#define FPGA_BYPASS_TCK_BIT			( BIT0 )
#define FPGA_BYPASS_JTAG_PORT_DIR	( P3DIR )
#define FPGA_BYPASS_JTAG_PORT_OUT	( P3OUT )
#define FPGA_BYPASS_JTAG_PORT_IN	( P3IN )

#define FPGA_BYPASS_TEST_HIGH		{ FPGA_BYPASS_JTAG_SBW_PORT_OUT |= FPGA_BYPASS_TEST_BIT; }
#define FPGA_BYPASS_TEST_LOW		{ FPGA_BYPASS_JTAG_SBW_PORT_OUT &= ~FPGA_BYPASS_TEST_BIT; }
#define FPGA_BYPASS_RST_HIGH		{ FPGA_BYPASS_JTAG_SBW_PORT_OUT |= FPGA_BYPASS_RST_BIT; }
#define FPGA_BYPASS_RST_LOW			{ FPGA_BYPASS_JTAG_SBW_PORT_OUT &= ~FPGA_BYPASS_RST_BIT; }
#define FPGA_BYPASS_TDI_HIGH		{ FPGA_BYPASS_JTAG_SBW_PORT_OUT |= FPGA_BYPASS_TDI_BIT; }
#define FPGA_BYPASS_TDI_LOW			{ FPGA_BYPASS_JTAG_SBW_PORT_OUT &= ~FPGA_BYPASS_TDI_BIT; }
#define FPGA_BYPASS_TMS_HIGH		{ FPGA_BYPASS_JTAG_SBW_PORT_OUT |= FPGA_BYPASS_TMS_BIT; }
#define FPGA_BYPASS_TMS_LOW			{ FPGA_BYPASS_JTAG_SBW_PORT_OUT &= ~FPGA_BYPASS_TMS_BIT; }
#define FPGA_BYPASS_TCK_HIGH		{ FPGA_BYPASS_JTAG_SBW_PORT_OUT |= FPGA_BYPASS_TCK_BIT; }
#define FPGA_BYPASS_TCK_LOW			{ FPGA_BYPASS_JTAG_SBW_PORT_OUT &= ~FPGA_BYPASS_TCK_BIT; }
#define FPGA_BYPASS_TDO_IS_ASSERTED	( FPGA_BYPASS_JTAG_SBW_PORT_IN & FPGA_BYPASS_TDO_BIT )

// UART direction signals in bypass mode
#define FPGA_BYPASS_DIR_CTRL_UART_RTR_BIT	( BIT7 )
#define FPGA_BYPASS_DIR_CTRL_UART_CTS_BIT	( BIT6 )
#define FPGA_BYPASS_DIR_CTRL_UART_RXD_BIT	( BIT5 )
#define FPGA_BYPASS_DIR_CTRL_UART_TXD_BIT	( BIT4 )
//#define FPGA_BYPASS_DIR_CTRL_PORT_OUT	      ( P1OUT )

// UART data signals in bypass mode
#define FPGA_BYPASS_UART_RTR_BIT		( BIT1 )
#define FPGA_BYPASS_UART_RTR_PORT_DIR	( P2DIR)
#define FPGA_BYPASS_UART_RTR_PORT_OUT	( P2OUT )
#define FPGA_BYPASS_UART_CTS_BIT		( BIT5 )
#define FPGA_BYPASS_UART_CTS_PORT_DIR	( P9DIR )
#define FPGA_BYPASS_UART_CTS_PORT_OUT	( P9OUT )
#define FPGA_BYPASS_UART_RXD_BIT		( BIT2 )
#define FPGA_BYPASS_UART_RXD_PORT_DIR	( P8DIR )
#define FPGA_BYPASS_UART_RXD_PORT_OUT	( P8OUT )
#define FPGA_BYPASS_UART_TXD_BIT		( BIT3 )
#define FPGA_BYPASS_UART_TXD_PORT_DIR	( P8DIR )
#define FPGA_BYPASS_UART_TXD_PORT_OUT	( P8OUT )

// FPGA commands
#define FPGA_CMD_CFG        0x00
#define FPGA_CMD_IR8_RD     0x01
#define FPGA_CMD_IR8        0x02
#define FPGA_CMD_IR4_RD     0x03
#define FPGA_CMD_IR4        0x04
#define FPGA_CMD_DR8_RD     0x05
#define FPGA_CMD_DR8        0x06
#define FPGA_CMD_DRX_RD     0x07
#define FPGA_CMD_DRX        0x08
#define FPGA_CMD_DRX0_RD    0x09
#define FPGA_CMD_BYPASS     0x0A
#define FPGA_CMD_RESET      0x0B
#define FPGA_CMD_ABORT      0x0C
#define FPGA_CMD_VERSION    0x0D
#define FPGA_CMD_CJTAG      0x0E

// FPGA cJTAG commands
#define FPGA_CJTAG_ZBS      0x00
#define FPGA_CJTAG_TCKIDLE  0x01

// Config Parameters
//FPGA registers
#define REG_RESPONSE_HANDSHAKE_OFF  0x00
#define REG_PROTOCOL		        0x01
#define REG_IRSCAN_PREAMBLE         0x02
#define REG_IRSCAN_POSTAMBLE	    0x03
#define REG_DRSCAN_PREAMBLE	        0x04
#define REG_DRSCAN_POSTAMBLE	    0x05
#define REG_TEST_CLK_FREQUENCY	    0x06
#define REG_TARGET_IO_CONFIGURATION 0x07
#define REG_TCLKset0		        0x08
#define REG_TCLKset1		        0x09
#define REG_TCLK_CLK_FREQUENCY	    0x0a
#define REG_START_FIFO		        0x0b
#define REG_STOP_FIFO		        0x0c
#define REG_JTAG_4_WIRE_FPGA_432    0x0D

#define JTAG_4_WIRE_FPGA            0
#define SBW_2_MSP_FET_FPGA          2
#define SBW_2_BACK_FPGA             1
#define TDI_TO_TDO_FPGA             3
#define TRI_STATE_FPGA_JTAG         4
#define TRI_STATE_FPGA_SBW          5


// PORTA bits
typedef union
{
  // Bit control
  struct
  {
    uint8_t DATA0		: 8;
    uint8_t UCA0TXD	: 1;
    uint8_t UCA0RXD	: 1;
    uint8_t CMD		: 4;
    uint8_t WR_TRIG	: 1;
    uint8_t RESERVED1	: 1;
  } bit;

  // Bypass control
  struct
  {
    uint8_t IO_GPIO0        : 1;
    uint8_t IO_GPIO1        : 1;
    uint8_t RESERVED0	      : 2;
    uint8_t DIR_CTRL_GPIO0  : 1;
    uint8_t DIR_CTRL_GPIO1  : 1;
    uint8_t DIR_CTRL_GPIO2  : 1;
    uint8_t DIR_CTRL_GPIO3  : 1;
    uint8_t IO_GPIO2        : 1;
    uint8_t IO_GPIO3        : 1;
    uint8_t RESERVED1	      : 6;
  } byp;

  // 16-bit access
  uint16_t all;

} s_FPGA_PA;

// PORTB bits
typedef union
{
   // Bypass control
  struct
  {
    uint8_t IO_TCK          : 1;
    uint8_t IO_TMS          : 1;
    uint8_t IO_TDI	      : 1;
    uint8_t IO_TDO          : 1;
    uint8_t IO_RST          : 1;
    uint8_t IO_TEST         : 1;
    uint8_t RESERVED0	      : 2;
    uint8_t DIR_CTRL_TCK    : 1;
    uint8_t DIR_CTRL_TMS    : 1;
    uint8_t DIR_CTRL_TDI    : 1;
    uint8_t DIR_CTRL_TDO    : 1;
    uint8_t DIR_CTRL_RST    : 1;
    uint8_t DIR_CTRL_TEST   : 1;
    uint8_t RESERVED1	      : 2;
  } byp;

  // 16-bit access
  uint16_t all;

} s_FPGA_PB;







struct _StreamSafe_
{
    uint8_t rx[12];
    uint16_t *ext_data;
    uint16_t ext_size;
    uint16_t ext_counter;
};
typedef struct _StreamSafe_ StreamSafe;

struct stream_funcs
{
    int16_t (MSPProbeSim::*out_init)(uint8_t, uint8_t);
    int16_t (MSPProbeSim::*flush)(void);
    int16_t (MSPProbeSim::*put_byte)(uint8_t);
    int16_t (MSPProbeSim::*put_bytes)(void*, uint16_t);
    int16_t (MSPProbeSim::*put_word)(uint16_t);
    int16_t (MSPProbeSim::*put_long)(uint32_t);
    int16_t (MSPProbeSim::*in_init)(void *, uint8_t);
    int16_t (MSPProbeSim::*internal_stream)(uint8_t *, uint16_t, uint8_t *, uint16_t, StreamSafe *);
    int16_t (MSPProbeSim::*external_stream)(StreamSafe *);
    uint8_t (MSPProbeSim::*change_type)(uint8_t);
    int16_t (MSPProbeSim::*get_byte)(uint8_t *);
    int16_t (MSPProbeSim::*get_word)(uint16_t *);
    int16_t (MSPProbeSim::*get_long)(uint32_t *);
    int16_t (MSPProbeSim::*get_buffer)(void **, uint16_t *);
    int16_t (MSPProbeSim::*discard_bytes)(uint16_t);
    int16_t (MSPProbeSim::*memcpy)(uint8_t*, uint8_t*, uint16_t);
    int16_t (MSPProbeSim::*biosLedOn)(uint8_t);
    int16_t (MSPProbeSim::*biosLedOff)(uint8_t);
    int16_t (MSPProbeSim::*biosLedBlink)(uint8_t, uint16_t);
    int16_t (MSPProbeSim::*biosLedFlash)(uint8_t, uint16_t);
    int16_t (MSPProbeSim::*biosLedAlternate)(uint16_t);
    int16_t (MSPProbeSim::*sendDebug)(int8_t *, uint16_t size);
    int16_t (MSPProbeSim::*getSharedVariable)(uint16_t IdMemoryType, uint16_t** Address);
    int16_t (MSPProbeSim::*deleteSharedVariable)(uint16_t IdMemoryType);
};









#define UNIMP_FN()  printf("### UNIMPLEMENTED: %s\n", __FUNCTION__)

#define CONST_AT(x, y) const x

#define HAL_FUNCTION(x) int16_t x (uint16_t flags)
using HalFuncInOut = int16_t (MSPProbeSim::*)(uint16_t);

#ifndef HAL_REC
#define HAL_REC
struct _HalRec_
{
  uint16_t id;
  HalFuncInOut function;
};
typedef struct _HalRec_ HalRec;
#endif

typedef void (MSPProbeSim::*HilInitFunc)();
typedef void *(MSPProbeSim::*HalMainFunc)(struct MSPProbeSim::stream_funcs* stream_adr, uint32_t, uint8_t v3opHilCrcOk, uint8_t v3opDcdcCrcOk);

struct _HAL_INFOS_
{
    HalMainFunc init;
    int16_t sw_0;
    int16_t sw_1;
    uint16_t hal_size;
    HalRec *hal_list_ptr;
    uint16_t hil_version;
    uint16_t fpga_version;
    int16_t swCmp_0;
    int16_t swCmp_1;
    uint16_t hil_versionCmp;
};
typedef struct _HAL_INFOS_ *HAL_INFOS_PTR;
typedef struct _HAL_INFOS_ HAL_INFOS;

#define RO_PLACEMENT_NO_INIT
#define DIAG_DEFAULT(x)
#define DIAG_SUPPRESS(x)
#define _NOP()
#define __delay_cycles(x)





struct DCDC_INFOS
{
    int16_t (MSPProbeSim::*getSubMcuVersion)(void);
    int16_t (MSPProbeSim::*getLayerVersion)(void);
    int16_t (MSPProbeSim::*dcdcCalibrate)(uint16_t resistor[4], uint16_t resCount, uint16_t vcc);
    int16_t (MSPProbeSim::*dcdcPowerDown)(void);
    int16_t (MSPProbeSim::*dcdcSetVcc)(uint16_t vcc);
    int16_t (MSPProbeSim::*dcdcRestart)(uint16_t fetType_);
    void    (MSPProbeSim::*dcdc_getCalibrationValues)(uint16_t vcc, uint16_t resistor,  uint16_t resCount, uint32_t *ticks, uint32_t *time);
    int16_t (MSPProbeSim::*getLayerVersionCmp)(void);
};
typedef struct DCDC_INFOS DCDC_INFOS_t;

typedef void *(MSPProbeSim::*DcdcInit)(DCDC_INFOS_t* dcdcInfos_Pointer);


#define _CONCAT2(x, y) x ## y
#define _CONCAT(x, y) _CONCAT2(x, y)
#define STATIC_VARS_START() auto& sv = _CONCAT(THISFN, _staticVars)
#define DECL_STATIC_VAR(n) auto& n = sv.n


#define PTR_FOR_CMP uintptr_t
