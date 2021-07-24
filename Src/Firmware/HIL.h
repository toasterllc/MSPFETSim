#pragma once

//#define HIL_VERSION        0x000C
#define HIL_SIGNATURE      0xF00DF00Dul
#define EXTERNAL_VCC_ON    1
#define EXTERNAL_VCC_OFF   0

#define REGULATION_ON    1
#define REGULATION_OFF   0

//! \brief version of bios code
const uint16_t hil_version_ = MSP_FET_HIL_VERSION;
//#pragma required=hil_version_

const uint16_t hil_versionCmp_ = MSP_FET_HIL_VERSION_CMP;
//#pragma required=hil_versionCmp_

const uint32_t hil_signature_ = HIL_SIGNATURE;
//#pragma required=hil_signature_

// MSPFETSim: not sure what this value is supposed to be
const HilInitFunc hil_Start_UP_ = MEMBER_FN_PTR(_hil_startUp);


uint16_t gTclkHighWhilePsa = 0;
uint16_t setValueVcc = 0;
uint16_t externalVccOn = 0;
uint16_t regulationOn = 0;

uint16_t jtagReleased = 0;
uint16_t setPCclockBeforeCapture = 0;

//// function prototypes for map initialization
//// common HIL configuration methods
//int16_t _hil_Init( void );
//int16_t _hil_SetVcc(uint16_t Vcc);
//int16_t _hil_GetVcc(double* Vcc, double* ExtVcc);
//int16_t _hil_SetProtocol(uint16_t protocol_id);
//void  _hil_SetPsaSetup(uint16_t enhanced);
//void  _hil_SetPsaTCLK(uint16_t tclkValue);
//
//int16_t _hil_Open( uint8_t state );
//int16_t _hil_Close( void );
//int16_t IccMonitor_Process(uint16_t flags); // flags: to be compatible with HIL calls
//void  _hil_EntrySequences(uint8_t states);
//
//void _hil_SetReset(uint8_t value);
//void _hil_SetTest(uint8_t value);
//void _hil_SetTMS(uint8_t value);
//void _hil_SetTDI(uint8_t value);
//void _hil_SetTCK(uint8_t value);
//
//void _hil_SetJtagSpeed(uint16_t jtagSpeed, uint16_t sbwSpeed);
//void _hil_ConfigureSetPc (uint16_t PCclockBeforeCapture);
//
//void _hil_setDacValues(uint16_t dac0, uint16_t dac1);
//int16_t _hil_regulateVcc(void);
//void _hil_switchVccFET(uint16_t switchVccFET);
//int16_t _hil_SetVcc(uint16_t Vcc);
//
//uint32_t hil_Jtag_read_idcode();
//uint32_t hil_swd_read_idcode();
//
//// SBW4
//extern int16_t _hil_4w_TapReset(void);
//extern int16_t _hil_4w_CheckJtagFuse(void);
//extern void  _hil_4w_StepPsa(uint32_t length);
//extern void  _hil_4w_StepPsaTclkHigh(uint32_t length);
//extern int16_t _hil_4w_BlowFuse(uint8_t targetHasTestVpp);
//extern uint8_t _hil_4w_Instr(uint8_t Instruction);
//
//// SBW2 DMA
//extern int16_t _hil_2w_TapReset_Dma(void);
//extern int16_t _hil_2w_CheckJtagFuse_Dma(void);
//extern uint8_t _hil_2w_Instr_Dma(uint8_t Instruction);
//extern uint8_t _hil_2w_SetReg_XBits08_Dma(uint8_t Data);
//extern uint16_t _hil_2w_SetReg_XBits16_Dma(uint16_t Data);
//extern uint32_t _hil_2w_SetReg_XBits20_Dma(uint32_t Data);
//extern uint32_t _hil_2w_SetReg_XBits32_Dma(uint32_t Data);
//extern uint64_t _hil_2w_SetReg_XBits64_Dma(uint64_t Data);
//extern uint64_t _hil_2w_SetReg_XBits8_64_Dma(uint64_t Data, uint16_t loopCount, uint16_t PG);
//extern void _hil_2w_Tclk_Dma(uint8_t state);
//extern void _hil_2w_StepPsa_Dma(uint32_t length);
//extern void _hil_2w_StepPsa_Dma_Xv2(uint32_t length);
//extern void _hil_2w_StepPsaTclkHigh_Dma(uint32_t length);
//extern int16_t _hil_2w_BlowFuse_Dma(uint8_t targetHasTestVpp);
//extern void _hil_2w_ConfigureSpeed_Dma(uint16_t speed);
//extern uint8_t _hil_2w_GetPrevInstruction_Dma();
//extern uint8_t DMA_TMSL_TDIL[];
//extern void setProtocolSbw2Dma(uint16_t id);

void _hil_Delay_1ms(uint16_t ms)
{
    _flush();
    usleep(ms*1000);
}

void _hil_Delay_1us(uint16_t us)
{
    _flush();
    usleep(us);
}

int16_t _hil_dummy_TapReset(void) {return 0;}
int16_t _hil_dummy_CheckJtagFuse(void){return 0;}
SBWShiftProxy<uint8_t> _hil_dummy_Instr(uint8_t Instruction){return 0;}
SBWShiftProxy<uint8_t> _hil_dummy_SetReg_XBits08(uint8_t Data){return 0;}
SBWShiftProxy<uint8_t> _hil_dummy_Instr_4(uint8_t Data){return 0;}

SBWShiftProxy<uint16_t> _hil_dummy_SetReg_XBits16(uint16_t Data){return 0;}
SBWShiftProxy<uint32_t,20> _hil_dummy_SetReg_XBits20(uint32_t Data){return 0;}
SBWShiftProxy<uint32_t> _hil_dummy_SetReg_XBits32(uint32_t Data){return 0;}
SBWShiftProxy<uint64_t> _hil_dummy_SetReg_XBits64(uint64_t Data){return 0;}
uint64_t _hil_dummy_SetReg_XBits8_64(uint64_t Data, uint16_t loopCount, uint16_t JStateVersion){return 0;}
uint64_t _hil_dummy_SetReg_XBits(uint64_t *data, uint16_t count){return 0;}
uint64_t _hil_dummy_SetReg_XBits35(uint64_t *data){return 0;}

void _hil_dummy_Tclk(uint8_t state){return;}
void _hil_dummy_StepPsa(uint32_t length){return;}
int16_t _hil_dummy_BlowFuse(uint8_t targetHasTestVpp){return 0;}
uint8_t _hil_dummy_GetPrevInstruction(){return 0;}

int16_t _hil_dummy_Write_Read_Ap(uint32_t address ,uint32_t *data, uint16_t rnw){return -1;};
int16_t _hil_dummy_Write_Read_Dp(uint8_t address ,uint32_t *data, uint16_t rnw){return -1;};
int16_t _hil_dummy_Write_Read_Mem_Ap(uint16_t ap_sel, uint32_t address, uint32_t *data, uint16_t rnw){return -1;};
uint32_t _hil_read_idcodeDummy(){return -1;}
uint8_t _hil_dummy_TransferData(uint8_t regiser, uint32_t* data, uint8_t rnw){return -1;};
void _hil_dummy_SetToolID(uint16_t id) {return;}

//// FPGA generic 430
//extern void initGeneric();
//extern uint8_t _hil_generic_Instr(uint8_t Instruction);
//
//extern uint8_t _hil_generic_SetReg_XBits08(uint8_t Data);
//extern uint16_t _hil_generic_SetReg_XBits16(uint16_t Data);
//extern uint32_t _hil_generic_SetReg_XBits20(uint32_t Data);
//extern uint32_t _hil_generic_SetReg_XBits32(uint32_t Data);
//extern uint64_t _hil_generic_SetReg_XBits64(uint64_t Data);
//extern uint64_t _hil_generic_XBits8_64(uint64_t Data, uint16_t loopCount, uint16_t PG);
//extern void  _hil_generic_Tclk(uint8_t state);
//extern void  _hil_generic_ConfigureSpeed(uint16_t speed);
//extern uint8_t _hil_generic_GetPrevInstruction();
//
//// FPGA generic 432
//extern uint8_t _hil_generic432_Instr_4(uint8_t Data);
////extern uint64_t _hil_generic432_SetReg_XBits(uint64_t *data, uint16_t count);
//extern uint8_t _hil_generic432_SetReg_XBits08(uint8_t Data);
//extern uint16_t _hil_generic432_SetReg_XBits16(uint16_t Data);
//extern uint32_t _hil_generic432_SetReg_XBits32(uint32_t Data);
//extern uint64_t _hil_generic432_SetReg_XBits64(uint64_t Data);
//extern uint64_t _hil_generic432_SetReg_XBits35(uint64_t *data);
//
//extern void hil_4w_432_Seq(uint16_t length, uint8_t *sequence);
//
//// FPGA generic
//extern uint16_t hil_fpga_get_version(void);
//extern void _hil_FpgaAccess_setTimeOut(uint16_t state);
//
//// protocol specific methods
//// must be implemeted in SWD mode
//extern void hil_Swd_InitJtag(struct jtag tmp);
//extern void hil_Swd_Seq(uint16_t length, uint8_t *sequence);
//extern uint8_t Swd_TransferData(uint8_t regiser, uint32_t* data, uint8_t rnw);
//
//extern void SWDTCKset1();
//extern void SWDTCKset0();
//
//void SWD_Entry();
//void JTAG_Entry();
//
//int16_t _hil_Write_Read_Ap_Jtag(uint32_t address ,uint32_t *data, uint16_t rnw);
//int16_t _hil_Write_Read_Dp_Jtag(uint8_t address ,uint32_t *data, uint16_t rnw);
//int16_t _hil_Write_Read_Mem_Ap_Jtag(uint16_t ap_sel, uint32_t address, uint32_t *data, uint16_t rnw);
//
//int16_t _hil_Write_Read_Dp_Swd(uint8_t address ,uint32_t *data, uint16_t rnw);
//int16_t _hil_Write_Read_Ap_Swd(uint32_t address, uint32_t *data, uint16_t rnw);
//int16_t _hil_Write_Read_Mem_Ap_Swd(uint16_t ap_sel, uint32_t address, uint32_t *data, uint16_t rnw);
//
//void _hil_Release(void);
//void  hil_initTimerA2(void);
//void _hil_BSL_EntrySequence(uint16_t);
//void _hil_BSL_EntrySequence1xx_4xx();
//void _hil_ReadADC12(void);
//void _hil_ConfigFpgaIoMode(uint16_t mode);
//
//static void _hil_Connect(uint8_t state);
//
//extern void initJtagSbw2Dma(struct jtag tmp);
//extern void initJtagSbw4(struct jtag tmp);

edt_common_methods_t   _Common_Methods = {};
edt_distinct_methods_t _Distinct_Methods = {};

struct jtag _Jtag =
{
  0,  // TCK, P4.4 (out) (high)
  0,  // TMS, P4.5 (out) (high)
  0,  // TDI, P4.6 (out) (high)
  0,  // TDO, P4.7 (in)
  0,
  0,
  0, //RST
  0, //TST
  0
};

// fuse blow control
struct vfuse_ctrl _VFuse =
{
  0,    // VF2TEST
  0,    // VF2TDI
  0,    // TDIOFF
  0,    // VF control register
  0,    // PWM_SETVF
  0,    // VF PWM control register
};

uint16_t gprotocol_id = 0;
uint16_t hil_sbw2Speed_ = 0;
uint16_t hil_jtagSpeed_ = 0;
int16_t dtVccSenseState = 0;

volatile uint8_t useTDI = 0;

//#pragma inline=forced
void RSTset1()
{
    _msp.sbwRstSet(1);
    _hil_Delay_1ms(5);
}
//#pragma inline=forced
void RSTset0()
{
    _msp.sbwRstSet(0);
    _hil_Delay_1ms(5);
}

//#pragma inline=forced
void TMSset1()
{
    UNIMP_FN();
//    { (*_Jtag.Out) |= _Jtag.TMS; }
//    _hil_Delay_1ms(5);
}
//#pragma inline=forced
void TMSset0()
{
    UNIMP_FN();
//    { (*_Jtag.Out) &= ~_Jtag.TMS; }
//    _hil_Delay_1ms(5);
}

//#pragma inline=forced
void TCKset1()
{
    _msp.sbwTestSet(1);
    _hil_Delay_1ms(5);
}
//#pragma inline=forced
void TCKset0()
{
    _msp.sbwTestSet(0);
    _hil_Delay_1ms(5);
}

//#pragma inline=forced
void TCKset1NoDelay()
{
    _msp.sbwTestSet(1);
}

//#pragma inline=forced
void TCKset0NoDelay()
{
    _msp.sbwTestSet(0);
}

//#pragma inline=forced
void TDIset1()
{
    UNIMP_FN();
//    { (*_Jtag.Out) |= _Jtag.TDI; }
//    _hil_Delay_1ms(5);
}
//#pragma inline=forced
void TDIset0()
{
    UNIMP_FN();
//    { (*_Jtag.Out) &= ~_Jtag.TDI; }
//     _hil_Delay_1ms(5);
}

//#pragma inline=forced
void TSTset1()
{
    _msp.sbwTestSet(1);
    _hil_Delay_1ms(5);
}
//#pragma inline=forced
void TSTset0()
{
    _msp.sbwTestSet(0);
    _hil_Delay_1ms(5);
}
//#pragma inline=forced
void TSTset1NoDelay()
{
    _msp.sbwTestSet(1);
}
//#pragma inline=forced
void TSTset0NoDelay()
{
    _msp.sbwTestSet(0);
}

////#pragma inline=forced
void TCLKset1()
{
    CALL_MEMBER_FN_PTR(_Distinct_Methods.Tclk)(1);
}

////#pragma inline=forced
void TCLKset0()
{
   CALL_MEMBER_FN_PTR(_Distinct_Methods.Tclk)(0);
}
//#pragma inline=forced
void TCLK()
{
    TCLKset0();
    TCLKset1();
}

void _hil_initEdtCommenMethods()
{
    _Common_Methods.Init                     = MEMBER_FN_PTR(_hil_Init);
    _Common_Methods.SetVcc                   = MEMBER_FN_PTR(_hil_SetVcc);
    _Common_Methods.SwitchVccFET             = MEMBER_FN_PTR(_hil_switchVccFET);
    _Common_Methods.GetVcc                   = MEMBER_FN_PTR(_hil_GetVcc);
    _Common_Methods.SetProtocol              = MEMBER_FN_PTR(_hil_SetProtocol);
    _Common_Methods.SetPsaTCLK               = MEMBER_FN_PTR(_hil_SetPsaTCLK);
    _Common_Methods.Open                     = MEMBER_FN_PTR(_hil_Open);
    _Common_Methods.Close                    = MEMBER_FN_PTR(_hil_Close);
    _Common_Methods.Delay_1us                = MEMBER_FN_PTR(_hil_Delay_1us);
    _Common_Methods.Delay_1ms                = MEMBER_FN_PTR(_hil_Delay_1ms);
    _Common_Methods.Loop                     = nullptr;
    _Common_Methods.EntrySequences           = MEMBER_FN_PTR(_hil_EntrySequences);
    _Common_Methods.SetReset                 = MEMBER_FN_PTR(_hil_SetReset);
    _Common_Methods.SetTest                  = MEMBER_FN_PTR(_hil_SetTest);
    _Common_Methods.SetTMS                   = MEMBER_FN_PTR(_hil_SetTMS);
    _Common_Methods.SetTCK                   = MEMBER_FN_PTR(_hil_SetTCK);
    _Common_Methods.SetTDI                   = MEMBER_FN_PTR(_hil_SetTDI);
    _Common_Methods.SetJtagSpeed             = MEMBER_FN_PTR(_hil_SetJtagSpeed);
    _Common_Methods.ConfigureSetPc           = MEMBER_FN_PTR(_hil_ConfigureSetPc);
    _Common_Methods.initDelayTimer           = MEMBER_FN_PTR(hil_initTimerA2);
    _Common_Methods.BSL_EntrySequence        = MEMBER_FN_PTR(_hil_BSL_EntrySequence);
    _Common_Methods.BSL_EntrySequence1xx_4xx = MEMBER_FN_PTR(_hil_BSL_EntrySequence1xx_4xx);
    _Common_Methods.SetToolID                = MEMBER_FN_PTR(_hil_dummy_SetToolID);
    _Common_Methods.regulateVcc              = MEMBER_FN_PTR(_hil_regulateVcc);
    _Common_Methods.setFpgaTimeOut           = MEMBER_FN_PTR(_hil_FpgaAccess_setTimeOut);
    _Common_Methods.getFpgaVersion           = MEMBER_FN_PTR(hil_fpga_get_version);
    _Common_Methods.ReadADC12                = MEMBER_FN_PTR(_hil_ReadADC12);
    _Common_Methods.ConfigFpgaIoMode         = MEMBER_FN_PTR(_hil_ConfigFpgaIoMode);
}

//void _hil_getEdtDistinct(edt_distinct_methods_t* edt_distinct);
//void _hil_getEdtCommen(edt_common_methods_t* edt_commen);
//void _hil_switchVccFET(uint16_t switchVccFET);

//#pragma required=_hil_getEdtDistinct
//#pragma required=_hil_getEdtCommen

void _hil_startUp()
{
    _hil_initEdtCommenMethods();
    CALL_MEMBER_FN_PTR(_Common_Methods.Init)();
    
    // DMA4 workaround
//    DMACTL4 |= DMARMWDIS;
    
    return;
}
//#pragma location="INFO_C_DISTINCT"
void _hil_getEdtDistinct(edt_distinct_methods_t* edt_distinct)
{
    if(edt_distinct == 0)
    {
      return;
    }
    
    *edt_distinct = _Distinct_Methods;
}

//#pragma location="INFO_C_COMMEN"
void _hil_getEdtCommen(edt_common_methods_t* edt_commen)
{
    if(edt_commen == 0)
    {
      return;
    }
    
    *edt_commen = _Common_Methods;
}

void hil_initFuseBlow(struct vfuse_ctrl tmp)
{
    _VFuse = tmp;
}

void hil_initTimerB0(void)
{
    UNIMP_FN();
}

// -----------------------------------------------------------------------------
void hil_initTimerA0(void)
{
    UNIMP_FN();
}

// -----------------------------------------------------------------------------
void hil_initTimerA2(void)
{
    UNIMP_FN();
}

//#pragma inline=forced
void qDriveJTAG(void)
{
    UNIMP_FN();
}

//#pragma inline=forced
void qDriveSbw(void)
{
    // TEST=0, RST=1
    _msp.sbwTestSet(0);
    _msp.sbwRstSet(1);
}

//#pragma inline=forced
void qDriveSwd(void)
{
    UNIMP_FN();
}

void _hil_ConfigFpgaIoMode(uint16_t mode)
{
    hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_TARGET_IO_CONFIGURATION, mode);
}


void _hil_BSL_EntrySequence1xx_4xx()
{
    hil_fpga_enable_bypass();

    if(gprotocol_id == SPYBIWIRE)
    {
        qDriveSbw();
    }
    else
    {
        qDriveJTAG();
    }

    RSTset0();    // set RST 0
    // INIT phase
    TSTset0();
    _hil_Delay_1ms(100);

    TSTset1();    // set test to 1
    _hil_Delay_1ms(100);

    //this is the first pulse keep it low for less than 15us for 5xx
    TSTset0NoDelay();

    TCKset0NoDelay();
    _hil_Delay_1us(5);

    TCKset1NoDelay();
    _hil_Delay_1us(5);

    TSTset1();     // set test 1;

    _hil_Delay_1ms(50);
    TCKset0();

    RSTset1();     // set RST 1;
    _hil_Delay_1ms(50);

    TCKset1();

    TSTset0();     // set test 0;
    _hil_Delay_1ms(50);

    hil_fpga_disable_bypass();
}


void _hil_BSL_EntrySequence(uint16_t switchBypassOff)
{
    hil_fpga_enable_bypass();

    if(gprotocol_id == SPYBIWIRE)
    {
        qDriveSbw();
    }
    else
    {
        qDriveJTAG();
    }

    RSTset0();    // set RST 0
    // INIT phase
    TSTset0();
    _hil_Delay_1ms(100);

    TSTset1();    // set test to 1
    _hil_Delay_1ms(100);

    //this is the first pulse keep it low for less than 15us for 5xx
    TSTset0NoDelay();
    if(!switchBypassOff)
    {
        TCKset0NoDelay();
        _hil_Delay_1us(5);
    }
    else
    {
        _hil_Delay_1us(10);
    }

    if(!switchBypassOff)
    {
        TCKset1NoDelay();
        _hil_Delay_1us(5);
    }
    TSTset1();     // set test 1;

    _hil_Delay_1ms(50);
    if(!switchBypassOff)
    {
        TCKset0();
    }
    RSTset1();     // set RST 1;
    _hil_Delay_1ms(50);
    if(!switchBypassOff)
    {
        TCKset1();
    }
    TSTset0();     // set test 0;
    _hil_Delay_1ms(50);

    if(switchBypassOff)
    {
        hil_fpga_disable_bypass();
    }
}


// -----------------------------------------------------------------------------

int16_t _hil_Init( void )
{
    hil_initTimerA0();              // TimeStamp
    hil_initTimerA2();              // Current pulse counter
    hil_initTimerB0();              // Delay loop timer

    hil_DacControlInitDAC0();
    hil_DacControlInitDAC1();
    hil_AdcControlInitADC();

//    DMACTL2 = 0;
//    DMACTL1 = 0;

    // default config
    externalVccOn = EXTERNAL_VCC_OFF;
    regulationOn = REGULATION_OFF;
    dtVccSenseState = 0;

   // set default to TCLK low when doing PSA
    gTclkHighWhilePsa = 0;
    // initialize function pointers to distinct functions

    // initialize FPGA
    hil_fpga_init();
    // set default debug protocol to JTAG
    gprotocol_id = SPYBIWIREJTAG;
    // set default to TCLK low when doing PSA
    gTclkHighWhilePsa = 0;
    // initialize function pointers to distinct functions
    _hil_SetProtocol(SPYBIWIREJTAG);
    jtagReleased = 1;

    // initialize fuse blow control
    hil_initFuseBlow(_Msp_Fet);

    // Default SBW2 speed --> 60MHz/(98+2) = 600 kHz
    hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_TEST_CLK_FREQUENCY, 35);
    hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_TCLK_CLK_FREQUENCY, 35);
    // set Io config to IO_CONFIG_HIGH_Z_UART
    _hil_ConfigFpgaIoMode(0x0);
    return 0;
}


int16_t _hil_regulateVcc(void)
{
    UNIMP_FN();
    return 0;
}

void _hil_setDacValues(uint16_t dac0, uint16_t dac1)
{
    hil_DacControlSetDAC1(dac1);
    hil_DacControlSetDAC0(dac0);
}

void _hil_ReadADC12(void)
{
    hil_AdcControlRead();
}

void _hil_switchVccFET(uint16_t switchVccFET)
{
    UNIMP_FN();
}

savedDacValues_t _savedDacValues = {};

//#pragma optimize = low
int16_t _hil_SetVccSupplyDt(uint16_t Vcc)
{
    UNIMP_FN();
    return 0;
}

//#pragma optimize = low
int16_t _hil_SetVccSupply(uint16_t Vcc)
{
    UNIMP_FN();
    return 0;
}

//#pragma optimize = low
// -----------------------------------------------------------------------------
int16_t _hil_SetVcc(uint16_t Vcc)
{
    UNIMP_FN();
    return 0;
}
// -----------------------------------------------------------------------------

int16_t _hil_GetVcc(double* Vcc, double* ExtVcc)
{
    UNIMP_FN();
    *Vcc = 3300;
    *ExtVcc = 0;
    return 0;
}

//#pragma optimize = low
// -----------------------------------------------------------------------------
int16_t _hil_SetProtocol(uint16_t protocol_id)
{
    int16_t ret_value = 0;

    if( protocol_id == SPYBIWIRE )
    {
        gprotocol_id = SPYBIWIRE;
        _Jtag = _SBW_Back;
        initJtagSbw2Dma(_Jtag);
        initJtagBypass(_Jtag);
        initGeneric();
        _hil_2w_ConfigureSpeed_Dma(SBW100KHz);
    }
    else if( protocol_id == SPYBIWIREJTAG || protocol_id == JTAG || protocol_id == JTAG_432)
    {
        gprotocol_id = protocol_id;
        _Jtag = _Jtag_Target;
        initJtagSbw4(_Jtag);
        initJtagBypass(_Jtag);
        initGeneric();
    }
    else if(protocol_id == SWD_432)
    {
        gprotocol_id = protocol_id;
        _Jtag = _Jtag_Target;
        hil_Swd_InitJtag(_Jtag);
        initJtagBypass(_Jtag);
        initGeneric();
    }
    else if( protocol_id == SPYBIWIRE_SUBMCU )
    {
        _Jtag = _Jtag_SubMcu;
        initJtagSbw2Dma(_Jtag);
        gprotocol_id = SPYBIWIRE_SUBMCU;
        setProtocolSbw2Dma(SPYBIWIRE_SUBMCU);
        _hil_2w_ConfigureSpeed_Dma(SBW600KHz);
    }
    else if( protocol_id == SPYBIWIRE_MSP_FET)
    {
        gprotocol_id = protocol_id;
        _Jtag = _Jtag_Target;
        initJtagSbw2Dma(_Jtag);
        initJtagBypass(_Jtag);
        initGeneric();
        _hil_2w_ConfigureSpeed_Dma(SBW100KHz);
    }
    else
    {
        ret_value = -1;
    }

    if( protocol_id == SPYBIWIRE || protocol_id == SPYBIWIRE_SUBMCU || protocol_id == SPYBIWIRE_MSP_FET)
    {
//        // load DMA1 with size just default
//        DMA1CTL = ( DMADT0 | DMASRCINCR1 | DMASRCINCR0 | DMASRCBYTE | DMADSTBYTE);
//        DMA1DA =  (_Jtag.Out); //JTAGOUT;       // set destination address
//        DMA2CTL = ( DMADT0 | DMASRCINCR1 | DMASRCINCR0 | DMASRCBYTE | DMADSTBYTE);
//        DMA2DA =  (_Jtag.Out); //JTAGOUT;       // set destination address

        // Functionality executed in FPGA bypass mode by Sub-MCU
        _Distinct_Methods.TapReset              = MEMBER_FN_PTR(_hil_2w_TapReset_Dma);
        _Distinct_Methods.CheckJtagFuse         = MEMBER_FN_PTR(_hil_2w_CheckJtagFuse_Dma);

        // SBW communication with Sub-MCU is executed in Bit-Banging mode
        if ( protocol_id == SPYBIWIRE_SUBMCU )
        {
            _Distinct_Methods.Instr              = MEMBER_FN_PTR(_hil_2w_Instr_Dma);
            _Distinct_Methods.SetReg_XBits08     = MEMBER_FN_PTR(_hil_2w_SetReg_XBits08_Dma);
            _Distinct_Methods.SetReg_XBits16     = MEMBER_FN_PTR(_hil_2w_SetReg_XBits16_Dma);
            _Distinct_Methods.SetReg_XBits20     = MEMBER_FN_PTR(_hil_2w_SetReg_XBits20_Dma);
            _Distinct_Methods.SetReg_XBits32     = MEMBER_FN_PTR(_hil_2w_SetReg_XBits32_Dma);
            _Distinct_Methods.SetReg_XBits64     = MEMBER_FN_PTR(_hil_2w_SetReg_XBits64_Dma);
            _Distinct_Methods.SetReg_XBits8_64   = MEMBER_FN_PTR(_hil_2w_SetReg_XBits8_64_Dma);
            _Distinct_Methods.Tclk               = MEMBER_FN_PTR(_hil_2w_Tclk_Dma);
            _Distinct_Methods.GetPrevInstruction = MEMBER_FN_PTR(_hil_2w_GetPrevInstruction_Dma);
            _Distinct_Methods.SetReg_XBits       = MEMBER_FN_PTR(_hil_dummy_SetReg_XBits);
            _Distinct_Methods.Instr04            = MEMBER_FN_PTR(_hil_dummy_Instr_4);
            _Distinct_Methods.write_read_Dp      = MEMBER_FN_PTR(_hil_dummy_Write_Read_Dp);
            _Distinct_Methods.write_read_Ap      = MEMBER_FN_PTR(_hil_dummy_Write_Read_Ap);
            _Distinct_Methods.write_read_mem_Ap  = MEMBER_FN_PTR(_hil_dummy_Write_Read_Mem_Ap);
            _Distinct_Methods.SetReg_XBits35     = MEMBER_FN_PTR(_hil_dummy_SetReg_XBits35);
            _Distinct_Methods.SwdTransferData    = MEMBER_FN_PTR(_hil_dummy_TransferData);
        }
        // SBW communication with Target-MCU is executed in FPGA mode
        else if( protocol_id == SPYBIWIRE || protocol_id == SPYBIWIRE_MSP_FET)
        {
            _Distinct_Methods.Instr              = MEMBER_FN_PTR(_hil_generic_Instr);
            _Distinct_Methods.SetReg_XBits08     = MEMBER_FN_PTR(_hil_generic_SetReg_XBits08);
            _Distinct_Methods.SetReg_XBits16     = MEMBER_FN_PTR(_hil_generic_SetReg_XBits16);
            _Distinct_Methods.SetReg_XBits20     = MEMBER_FN_PTR(_hil_generic_SetReg_XBits20);
            _Distinct_Methods.SetReg_XBits32     = MEMBER_FN_PTR(_hil_generic_SetReg_XBits32);
            _Distinct_Methods.SetReg_XBits64     = MEMBER_FN_PTR(_hil_generic_SetReg_XBits64);
            _Distinct_Methods.SetReg_XBits8_64   = MEMBER_FN_PTR(_hil_generic_XBits8_64);
            _Distinct_Methods.GetPrevInstruction = MEMBER_FN_PTR(_hil_generic_GetPrevInstruction);
            _Distinct_Methods.Tclk               = MEMBER_FN_PTR(_hil_generic_Tclk);
            _Distinct_Methods.SetReg_XBits       = MEMBER_FN_PTR(_hil_dummy_SetReg_XBits);
            _Distinct_Methods.Instr04            = MEMBER_FN_PTR(_hil_dummy_Instr_4);
            _Distinct_Methods.write_read_Dp      = MEMBER_FN_PTR(_hil_dummy_Write_Read_Dp);
            _Distinct_Methods.write_read_Ap      = MEMBER_FN_PTR(_hil_dummy_Write_Read_Ap);
            _Distinct_Methods.write_read_mem_Ap  = MEMBER_FN_PTR(_hil_dummy_Write_Read_Mem_Ap);
            _Distinct_Methods.SetReg_XBits35     = MEMBER_FN_PTR(_hil_dummy_SetReg_XBits35);
            _Distinct_Methods.SwdTransferData    = MEMBER_FN_PTR(_hil_dummy_TransferData);
        }
        if(gTclkHighWhilePsa == 1)
        {
            _Distinct_Methods.StepPsa            = MEMBER_FN_PTR(_hil_2w_StepPsaTclkHigh_Dma);
        }
        else if(gTclkHighWhilePsa == 2)
        {
            _Distinct_Methods.StepPsa            = MEMBER_FN_PTR(_hil_2w_StepPsa_Dma_Xv2);
        }
        else
        {
            _Distinct_Methods.StepPsa            = MEMBER_FN_PTR(_hil_2w_StepPsa_Dma);
        }
        _Distinct_Methods.BlowFuse               = MEMBER_FN_PTR(_hil_2w_BlowFuse_Dma);

//        DMA2SA                                 = (uint8_t*)DMA_TMSL_TDIL);
    }
    else if ( protocol_id == SPYBIWIREJTAG  || protocol_id == JTAG)
    {
        _Distinct_Methods.TapReset               = MEMBER_FN_PTR(_hil_4w_TapReset);
        _Distinct_Methods.CheckJtagFuse          = MEMBER_FN_PTR(_hil_4w_CheckJtagFuse);
        _Distinct_Methods.Instr                  = MEMBER_FN_PTR(_hil_generic_Instr);
        _Distinct_Methods.SetReg_XBits08         = MEMBER_FN_PTR(_hil_generic_SetReg_XBits08);
        _Distinct_Methods.SetReg_XBits16         = MEMBER_FN_PTR(_hil_generic_SetReg_XBits16);
        _Distinct_Methods.SetReg_XBits20         = MEMBER_FN_PTR(_hil_generic_SetReg_XBits20);
        _Distinct_Methods.SetReg_XBits32         = MEMBER_FN_PTR(_hil_generic_SetReg_XBits32);
        _Distinct_Methods.SetReg_XBits64         = MEMBER_FN_PTR(_hil_generic_SetReg_XBits64);
        _Distinct_Methods.SetReg_XBits8_64       = MEMBER_FN_PTR(_hil_generic_XBits8_64);
        _Distinct_Methods.GetPrevInstruction     = MEMBER_FN_PTR(_hil_generic_GetPrevInstruction);
        _Distinct_Methods.Tclk                   = MEMBER_FN_PTR(_hil_generic_Tclk);

        // load dummies for 432
        _Distinct_Methods.Instr04                = MEMBER_FN_PTR(_hil_dummy_Instr_4);
        _Distinct_Methods.write_read_Dp          = MEMBER_FN_PTR(_hil_dummy_Write_Read_Dp);
        _Distinct_Methods.write_read_Ap          = MEMBER_FN_PTR(_hil_dummy_Write_Read_Ap);
        _Distinct_Methods.write_read_mem_Ap      = MEMBER_FN_PTR(_hil_dummy_Write_Read_Mem_Ap);
        _Distinct_Methods.SetReg_XBits           = MEMBER_FN_PTR(_hil_dummy_SetReg_XBits);
        _Distinct_Methods.SetReg_XBits35         = MEMBER_FN_PTR(_hil_dummy_SetReg_XBits35);
        _Distinct_Methods.SwdTransferData        = MEMBER_FN_PTR(_hil_dummy_TransferData);

        if(gTclkHighWhilePsa == 0 || gTclkHighWhilePsa == 2)
        {
            _Distinct_Methods.StepPsa            = MEMBER_FN_PTR(_hil_4w_StepPsa);
        }
        else
        {
            _Distinct_Methods.StepPsa            = MEMBER_FN_PTR(_hil_4w_StepPsaTclkHigh);
        }
        _Distinct_Methods.BlowFuse               = MEMBER_FN_PTR(_hil_4w_BlowFuse);
    }
    else if (protocol_id == JTAG_432)
    {
        // load dummies for 430
        _Distinct_Methods.CheckJtagFuse          = MEMBER_FN_PTR(_hil_dummy_CheckJtagFuse);
        _Distinct_Methods.Instr                  = MEMBER_FN_PTR(_hil_dummy_Instr);
        _Distinct_Methods.SetReg_XBits20         = MEMBER_FN_PTR(_hil_dummy_SetReg_XBits20);
        _Distinct_Methods.GetPrevInstruction     = MEMBER_FN_PTR(_hil_dummy_GetPrevInstruction);
        _Distinct_Methods.Tclk                   = MEMBER_FN_PTR(_hil_dummy_Tclk);
        _Distinct_Methods.SetReg_XBits8_64       = MEMBER_FN_PTR(_hil_dummy_SetReg_XBits8_64);
        _Distinct_Methods.StepPsa                = MEMBER_FN_PTR(_hil_dummy_StepPsa);
        _Distinct_Methods.BlowFuse               = MEMBER_FN_PTR(_hil_dummy_BlowFuse);

        _Distinct_Methods.TapReset               = MEMBER_FN_PTR(_hil_4w_TapReset);
        _Distinct_Methods.SetReg_XBits08         = MEMBER_FN_PTR(_hil_generic432_SetReg_XBits08);
        _Distinct_Methods.SetReg_XBits16         = MEMBER_FN_PTR(_hil_generic432_SetReg_XBits16);
        _Distinct_Methods.SetReg_XBits32         = MEMBER_FN_PTR(_hil_generic432_SetReg_XBits32);
        _Distinct_Methods.SetReg_XBits64         = MEMBER_FN_PTR(_hil_generic432_SetReg_XBits64);
        _Distinct_Methods.Instr04                = MEMBER_FN_PTR(_hil_generic432_Instr_4);
        _Distinct_Methods.write_read_Dp          = MEMBER_FN_PTR(_hil_Write_Read_Dp_Jtag);
        _Distinct_Methods.write_read_Ap          = MEMBER_FN_PTR(_hil_Write_Read_Ap_Jtag);
        _Distinct_Methods.write_read_mem_Ap      = MEMBER_FN_PTR(_hil_Write_Read_Mem_Ap_Jtag);
        _Distinct_Methods.GetJtagIdCode          = MEMBER_FN_PTR(hil_Jtag_read_idcode);
        _Distinct_Methods.SetReg_XBits35         = MEMBER_FN_PTR(_hil_generic432_SetReg_XBits35);
        _Distinct_Methods.SetReg_XBits           = MEMBER_FN_PTR(_hil_dummy_SetReg_XBits);
        _Distinct_Methods.SetReg_XBits20         = MEMBER_FN_PTR(_hil_dummy_SetReg_XBits20);
        _Distinct_Methods.SwdTransferData        = MEMBER_FN_PTR(_hil_dummy_TransferData);
    }
    else if (protocol_id == SWD_432)
    {
        // load dummies for 430
        _Distinct_Methods.CheckJtagFuse          = MEMBER_FN_PTR(_hil_dummy_CheckJtagFuse);
        _Distinct_Methods.Instr                  = MEMBER_FN_PTR(_hil_dummy_Instr);
        _Distinct_Methods.SetReg_XBits20         = MEMBER_FN_PTR(_hil_dummy_SetReg_XBits20);
        _Distinct_Methods.GetPrevInstruction     = MEMBER_FN_PTR(_hil_dummy_GetPrevInstruction);
        _Distinct_Methods.Tclk                   = MEMBER_FN_PTR(_hil_dummy_Tclk);
        _Distinct_Methods.SetReg_XBits8_64       = MEMBER_FN_PTR(_hil_dummy_SetReg_XBits8_64);
        _Distinct_Methods.StepPsa                = MEMBER_FN_PTR(_hil_dummy_StepPsa);
        _Distinct_Methods.BlowFuse               = MEMBER_FN_PTR(_hil_dummy_BlowFuse);
        _Distinct_Methods.TapReset               = MEMBER_FN_PTR(_hil_dummy_TapReset);
        _Distinct_Methods.SetReg_XBits08         = MEMBER_FN_PTR(_hil_dummy_SetReg_XBits08);
        _Distinct_Methods.SetReg_XBits16         = MEMBER_FN_PTR(_hil_dummy_SetReg_XBits16);
        _Distinct_Methods.SetReg_XBits20         = MEMBER_FN_PTR(_hil_dummy_SetReg_XBits20);
        _Distinct_Methods.SetReg_XBits32         = MEMBER_FN_PTR(_hil_dummy_SetReg_XBits32);
        _Distinct_Methods.SetReg_XBits64         = MEMBER_FN_PTR(_hil_dummy_SetReg_XBits64);
        _Distinct_Methods.Instr04                = MEMBER_FN_PTR(_hil_dummy_Instr_4);
        _Distinct_Methods.write_read_Dp          = MEMBER_FN_PTR(_hil_Write_Read_Dp_Swd);
        _Distinct_Methods.write_read_Ap          = MEMBER_FN_PTR(_hil_Write_Read_Ap_Swd);
        _Distinct_Methods.write_read_mem_Ap      = MEMBER_FN_PTR(_hil_Write_Read_Mem_Ap_Swd);
        _Distinct_Methods.GetJtagIdCode          = MEMBER_FN_PTR(hil_swd_read_idcode);
        _Distinct_Methods.SetReg_XBits35         = MEMBER_FN_PTR(_hil_dummy_SetReg_XBits35);
        _Distinct_Methods.SetReg_XBits           = MEMBER_FN_PTR(_hil_dummy_SetReg_XBits);
        _Distinct_Methods.SwdTransferData        = MEMBER_FN_PTR(Swd_TransferData);
    }
    jtagReleased = 0;
    return(ret_value);
}

void _hil_SetPsaTCLK(uint16_t tclkValue)
{
    gTclkHighWhilePsa = tclkValue;
    if(gprotocol_id == SPYBIWIRE || gprotocol_id == SPYBIWIRE_SUBMCU || gprotocol_id == SPYBIWIRE_MSP_FET)
    {
        if(gTclkHighWhilePsa == 1)
        {
            _Distinct_Methods.StepPsa           = MEMBER_FN_PTR(_hil_2w_StepPsaTclkHigh_Dma);
        }
        else if(gTclkHighWhilePsa == 2)
        {
            _Distinct_Methods.StepPsa           = MEMBER_FN_PTR(_hil_2w_StepPsa_Dma_Xv2);
        }
        else
        {
            _Distinct_Methods.StepPsa           = MEMBER_FN_PTR(_hil_2w_StepPsa_Dma);
        }
    }
    if(gprotocol_id == SPYBIWIREJTAG  || gprotocol_id == JTAG)
    {
        if(gTclkHighWhilePsa == 1)
        {
            _Distinct_Methods.StepPsa           = MEMBER_FN_PTR(_hil_4w_StepPsaTclkHigh);
        }
        else
        {
            _Distinct_Methods.StepPsa           = MEMBER_FN_PTR(_hil_4w_StepPsa);
        }
    }
}
//extern uint8_t lastTestState;

void _hil_Release(void)
{
    if(gprotocol_id == SPYBIWIRE)
    {
        hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_PROTOCOL, TRI_STATE_FPGA_SBW);
    }
    else if(gprotocol_id == SWD_432)
    {
            hil_fpga_disable_bypass();
            hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_PROTOCOL, TRI_STATE_FPGA_JTAG);
    }
    else
    {
        hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_PROTOCOL, TRI_STATE_FPGA_JTAG);
    }
    hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_JTAG_4_WIRE_FPGA_432, 0);

    lastTestState = 0;

    _Distinct_Methods.TapReset                  = MEMBER_FN_PTR(_hil_dummy_TapReset);
    _Distinct_Methods.CheckJtagFuse             = MEMBER_FN_PTR(_hil_dummy_CheckJtagFuse);
    _Distinct_Methods.Instr                     = MEMBER_FN_PTR(_hil_dummy_Instr);
    _Distinct_Methods.SetReg_XBits08            = MEMBER_FN_PTR(_hil_dummy_SetReg_XBits08);
    _Distinct_Methods.SetReg_XBits16            = MEMBER_FN_PTR(_hil_dummy_SetReg_XBits16);
    _Distinct_Methods.SetReg_XBits20            = MEMBER_FN_PTR(_hil_dummy_SetReg_XBits20);
    _Distinct_Methods.SetReg_XBits32            = MEMBER_FN_PTR(_hil_dummy_SetReg_XBits32);
    _Distinct_Methods.SetReg_XBits64            = MEMBER_FN_PTR(_hil_dummy_SetReg_XBits64);
    _Distinct_Methods.SetReg_XBits8_64          = MEMBER_FN_PTR(_hil_dummy_SetReg_XBits8_64);
    _Distinct_Methods.Tclk                      = MEMBER_FN_PTR(_hil_dummy_Tclk);
    _Distinct_Methods.GetPrevInstruction        = MEMBER_FN_PTR(_hil_dummy_GetPrevInstruction);
    _Distinct_Methods.StepPsa                   = MEMBER_FN_PTR(_hil_dummy_StepPsa);
    _Distinct_Methods.BlowFuse                  = MEMBER_FN_PTR(_hil_dummy_BlowFuse);
    _Distinct_Methods.SetReg_XBits              = MEMBER_FN_PTR(_hil_dummy_SetReg_XBits);
    _Distinct_Methods.Instr04                   = MEMBER_FN_PTR(_hil_dummy_Instr_4);
    _Distinct_Methods.write_read_Dp             = MEMBER_FN_PTR(_hil_dummy_Write_Read_Dp);
    _Distinct_Methods.write_read_Ap             = MEMBER_FN_PTR(_hil_dummy_Write_Read_Ap);
    _Distinct_Methods.write_read_mem_Ap         = MEMBER_FN_PTR(_hil_dummy_Write_Read_Mem_Ap);
    _Distinct_Methods.GetJtagIdCode             = MEMBER_FN_PTR(_hil_read_idcodeDummy);
    _Distinct_Methods.SetReg_XBits35            = MEMBER_FN_PTR(_hil_dummy_SetReg_XBits35);
    _Distinct_Methods.SetReg_XBits              = MEMBER_FN_PTR(_hil_dummy_SetReg_XBits);
    _Distinct_Methods.SwdTransferData           = MEMBER_FN_PTR(_hil_dummy_TransferData);

    jtagReleased = 1;
}

/*-------------RstLow_JTAG----------------
            ________           __________
Test ______|        |_________|
                          _______________
Rst_____________________|
----------------------------------------*/

INLINE(forced)
void _hil_EntrySequences_RstLow_JTAG()
{
    UNIMP_FN();
//    _DINT_FET();
//    TSTset0();                    //1
//    _hil_Delay_1ms(4);           //reset TEST logic
//
//    RSTset0();                    //2
//
//    //TSTset1();                    //3
//    (*_Jtag.Out) |= _Jtag.TST;
//    _hil_Delay_1ms(50);         //activate TEST logic
//
//    //RSTset0();                    //4
//    (*_Jtag.Out) &= ~_Jtag.RST;
//    _hil_Delay_1us(40);
//
//     // for 4-wire JTAG clear Test pin Test(0)
//    //TSTset0();   //5
//    (*_Jtag.Out) &= ~_Jtag.TST;
//    _hil_Delay_1us(2);
//
//    // for 4-wire JTAG -dry  Reset(1)
//    //(*_Jtag.Out) |= _Jtag.RST;
//    (*_Jtag.Out) &= ~_Jtag.RST;  // This actually starts the BSL
//    _hil_Delay_1us(2);
//
//    // 4-wire JTAG - Test (1)
//    //TSTset1();  //7
//    (*_Jtag.Out) |= _Jtag.TST;
//    _hil_Delay_1ms(5);
//    _EINT_FET();
}

/*-------------RstHigh_JTAG--------------
            ________           __________
Test ______|        |_________|
         _______                   ______
Rst____|       |_________________|
----------------------------------------*/

INLINE(forced)
void _hil_EntrySequences_RstHigh_JTAG()
{
    UNIMP_FN();
//    TSTset0();                    //1
//    _hil_Delay_1ms(1);           //reset TEST logic
//
//    RSTset1();                    //2
//
//    TSTset1();                    //3
//    _hil_Delay_1ms(100);         //activate TEST logic
//
//    RSTset0();                    //4
//    _hil_Delay_1us(40);
//
//    // for 4-wire JTAG clear Test pin Test(0)
//    (*_Jtag.Out) &= ~_Jtag.TST;   //5
//    _hil_Delay_1us(1);
//
//    // for 4-wire JTAG - Test (1)
//    (*_Jtag.Out) |= _Jtag.TST;  //7
//    _hil_Delay_1us(40);
//
//    // phase 5 Reset(1)
//    RSTset1();
//    _hil_Delay_1ms(5);
}

/*-------------RstHigh_SBW---------------
            ________           __________
Test ______|        |_________|
        _________________________________
Rst____|
----------------------------------------*/
INLINE(forced)
void _hil_EntrySequences_RstHigh_SBW() {
    // TEST=0, RST=0
    _msp.sbwTestSet(0);
    _msp.sbwRstSet(0);
    _hil_Delay_1ms(5);
    
    // RST=1
    _msp.sbwRstSet(1);
    _hil_Delay_1ms(5);
    
    // TEST=1
    _msp.sbwTestSet(1);
    _hil_Delay_1ms(100);
    
    // Pulse TEST=[0,1]
    _msp.sbwTestPulse();
    _hil_Delay_1ms(5);
}

/*-------------RstLow_SBW----------------
            ________           __________
Test ______|        |_________|
               __________________________
Rst__________|
----------------------------------------*/
INLINE(forced)
void _hil_EntrySequences_RstLow_SBW() {
    // TEST=0, RST=0
    _msp.sbwTestSet(0);
    _msp.sbwRstSet(0);
    _hil_Delay_1ms(5);
    
    // TEST=1
    _msp.sbwTestSet(1);
    _hil_Delay_1ms(100);
    
    // RST=1
    _msp.sbwRstSet(1);
    _hil_Delay_1ms(5);
    
    // Pulse TEST=[0,1]
    _msp.sbwTestPulse();
    _hil_Delay_1ms(5);
}

void _hil_EntrySequences(uint8_t states)
{
    uint16_t protocol = JTAG_4_WIRE_FPGA;

    //This should probably never be called with SPYBIWIRE_SUBMCU anyway
    if(gprotocol_id == SPYBIWIRE_SUBMCU)
    {
        hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_PROTOCOL, TRI_STATE_FPGA_JTAG);
        gprotocol_id = SPYBIWIRE;
    }
    hil_fpga_enable_bypass();

    switch(gprotocol_id)
    {
    case SPYBIWIRE:
    case SPYBIWIRE_MSP_FET:
      {
        if (states == RSTLOW)
        {
            _hil_EntrySequences_RstLow_SBW();
        }
        if (states == RSTHIGH)
        {
            _hil_EntrySequences_RstHigh_SBW();
        }
        protocol = (gprotocol_id == SPYBIWIRE) ? SBW_2_BACK_FPGA : SBW_2_MSP_FET_FPGA;
        break;
      }
    case SPYBIWIREJTAG:
      {
        if (states == RSTLOW)
        {
            _hil_EntrySequences_RstLow_JTAG();
        }
        if (states == RSTHIGH)
        {
            _hil_EntrySequences_RstHigh_JTAG();
        }
        break;
      }
    case JTAG_432:
        {
            JTAG_Entry();
            RSTset1();
            break;
        }
    case SWD_432:
        {
            SWD_Entry();
            SWDTCKset1();
            break;
        }
    default:
        if (states == RSTLOW)
        {
            RSTset0();
            TSTset1();
        }
        if (states == RSTHIGH)
        {
            TSTset1();
        }
        break;
    }
    if(gprotocol_id != SWD_432)
    {
        hil_fpga_disable_bypass();
        hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_PROTOCOL, protocol);

        if(gprotocol_id == JTAG_432)
        {
            hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_JTAG_4_WIRE_FPGA_432, 1);
        }
        else
        {
             hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_JTAG_4_WIRE_FPGA_432, 0);
        }
    }
}

//#pragma optimize = medium
void _hil_Connect(uint8_t state)
{
    if(jtagReleased)
    {
        _hil_SetProtocol(gprotocol_id);
    }

    hil_fpga_enable_bypass();

    if(state == RSTHIGH && gprotocol_id != JTAG_432 && gprotocol_id != SWD_432)
    {
        if(gprotocol_id == SPYBIWIRE || gprotocol_id == SPYBIWIRE_MSP_FET)
        {
            _hil_Delay_1ms(10);
            qDriveSbw();
            _hil_Delay_1ms(30);
            _hil_EntrySequences_RstHigh_SBW();
        }
        else
        {
            _hil_Delay_1ms(10);
            qDriveJTAG();
            _hil_Delay_1ms(30);
            if(gprotocol_id == SPYBIWIREJTAG)
            {
                _hil_EntrySequences_RstHigh_JTAG();
            }
            else
            {
                TSTset1();
            }
        }
    }
    else if(state == RSTLOW && gprotocol_id != JTAG_432 && gprotocol_id != SWD_432)
    {
        if(gprotocol_id == SPYBIWIRE || gprotocol_id == SPYBIWIRE_MSP_FET)
        {
            qDriveSbw();
            _hil_Delay_1ms(1);
            _hil_EntrySequences_RstLow_SBW();
        }
        else
        {
            qDriveJTAG();
            _hil_Delay_1ms(1);
            if(gprotocol_id == SPYBIWIREJTAG)
            {
                _hil_EntrySequences_RstLow_JTAG();
            }
            else
            {
                RSTset0();
                TSTset1();
            }
        }
    }
    else if(gprotocol_id == JTAG_432)
    {
        _hil_Delay_1ms(10);
        qDriveJTAG();
        _hil_Delay_1ms(30);
        JTAG_Entry();
        RSTset1();
    }
    else if(gprotocol_id == SWD_432)
    {
        _hil_Delay_1ms(10);
        qDriveSwd();
        _hil_Delay_1ms(30);
        SWD_Entry();
        RSTset1();
        hil_swd_read_idcode();
    }

    if(gprotocol_id != SWD_432)
    {
        hil_fpga_disable_bypass();
    }

    jtagReleased = 0;
}

// -----------------------------------------------------------------------------
int16_t _hil_Open( uint8_t state)
{
    if(gprotocol_id == SPYBIWIRE_MSP_FET)
    {
        _hil_Connect(state);
        hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_PROTOCOL, SBW_2_MSP_FET_FPGA);
        hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_JTAG_4_WIRE_FPGA_432, 0);
        _hil_generic_ConfigureSpeed(hil_sbw2Speed_);
    }
    else if(gprotocol_id == SPYBIWIRE)
    {
        _hil_Connect(state);
         hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_PROTOCOL, SBW_2_BACK_FPGA);
         hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_JTAG_4_WIRE_FPGA_432, 0);
        _hil_generic_ConfigureSpeed(hil_sbw2Speed_);
    }
    else if(gprotocol_id == SPYBIWIREJTAG || gprotocol_id == JTAG)
    {
        _hil_Connect(state);
        hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_PROTOCOL, JTAG_4_WIRE_FPGA);
        hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_JTAG_4_WIRE_FPGA_432, 0);
        _hil_generic_ConfigureSpeed(hil_jtagSpeed_);
    }
    else if(gprotocol_id == JTAG_432)
    {
        _hil_Connect(state);
        hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_PROTOCOL, JTAG_4_WIRE_FPGA);
        hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_JTAG_4_WIRE_FPGA_432, 1);
        _hil_generic_ConfigureSpeed(hil_jtagSpeed_);
    }
    else if(gprotocol_id == SPYBIWIRE_SUBMCU)
    {
        hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_PROTOCOL, TRI_STATE_FPGA_JTAG);
        hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_JTAG_4_WIRE_FPGA_432, 0);
        gprotocol_id = SPYBIWIRE;
        _hil_Connect(state);
    }
    else if(gprotocol_id == SWD_432)
    {
        gprotocol_id = SWD_432;
        _hil_Connect(state);
    }
    return 0;
}
// -----------------------------------------------------------------------------
int16_t _hil_Close( void )
{
    externalVccOn = EXTERNAL_VCC_OFF;
    _hil_Release();
    return 0;
}

// -----------------------------------------------------------------------------

void _hil_SetJtagSpeed(uint16_t jtagSpeed, uint16_t sbwSpeed)
{
    if( sbwSpeed )
    {
        hil_sbw2Speed_ =  sbwSpeed;
    }
    if ( jtagSpeed )
    {
        hil_jtagSpeed_ =  jtagSpeed;
    }
}

// -----------------------------------------------------------------------------

void _hil_ConfigureSetPc (uint16_t PCclockBeforeCapture)
{
    setPCclockBeforeCapture = PCclockBeforeCapture;
}

// -----------------------------------------------------------------------------
void SetVFuse(void)
{
    UNIMP_FN();
}

uint8_t prevP4value = 0;
// -----------------------------------------------------------------------------
void setVpp(int32_t voltage)
{
    UNIMP_FN();
}

// -----------------------------------------------------------------------------
void testVpp(uint8_t mode)
{
    if(mode)
    {
        // Fuse blow via TST
        // Switch back to JTAG protocol
        hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_PROTOCOL, JTAG_4_WIRE_FPGA);
        hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_JTAG_4_WIRE_FPGA_432, 0);

    }
    else
    {
        // Fuse blow via TDI
        // Switch to JTAG fuse blow protocol, TDI is mapped to TDO
        hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_PROTOCOL, TDI_TO_TDO_FPGA);
        hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_JTAG_4_WIRE_FPGA_432, 0);
    }

    useTDI = !mode;
}


// -----------------------------------------------------------------------------
void _hil_SetReset(uint8_t value)
{

    hil_fpga_enable_bypass();
    if(value)
    {
        RSTset1();
    }
    else
    {
        RSTset0();
    }
    hil_fpga_disable_bypass();
}

// -----------------------------------------------------------------------------
void _hil_SetTest(uint8_t value)
{
    hil_fpga_enable_bypass();

    if(value)
    {
        TSTset1();
    }
    else
    {
        TSTset0();
    }
    hil_fpga_disable_bypass();
}

// -----------------------------------------------------------------------------
void _hil_SetTMS(uint8_t value)
{
    hil_fpga_enable_bypass();

    if(value)
    {
        TMSset1();
    }
    else
    {
        TMSset0();
    }
    hil_fpga_disable_bypass();
}

// -----------------------------------------------------------------------------
void _hil_SetTDI(uint8_t value)
{
    hil_fpga_enable_bypass();

    if(value)
    {
        TDIset1();
    }
    else
    {
        TDIset0();
    }
    hil_fpga_disable_bypass();
}

// -----------------------------------------------------------------------------
void _hil_SetTCK(uint8_t value)
{
    hil_fpga_enable_bypass();

    if(value)
    {
        TCKset1();
    }
    else
    {
        TCKset0();
    }
    hil_fpga_disable_bypass();
}
// --------------------------------------------ARM defines end------------------

void SWD_Entry()
{
    uint8_t sequence[8];
    uint8_t i = 0;
    uint16_t entry = 0xE79E;
    for (i = 0; i < 8; i++)
    {
        sequence[i] = 0xff;
    }
    hil_Swd_Seq(56, sequence);

    sequence[0] = entry & 0xff;
    sequence[1] = (entry >> 8) & 0xff;
    hil_Swd_Seq(16, sequence);

    for (i = 0; i < 8; i++)
    {
        sequence[i] = 0xff;
    }
    hil_Swd_Seq(56, sequence);
}

 void JTAG_Entry()
{
    uint8_t sequence[8];
    uint8_t i = 0;
    uint16_t entry = 0xE73C;
    for (i = 0; i < 8; i++)
    {
        sequence[i] = 0xff;
    }
    hil_4w_432_Seq(51, sequence);

    sequence[0] = entry & 0xff;
    sequence[1] = (entry >> 8) & 0xff;
    hil_4w_432_Seq(16, sequence);

    for (i = 0; i < 1; i++)
    {
        sequence[i] = 0xff;
    }
    hil_4w_432_Seq(8, sequence);
}

uint32_t hil_swd_read_idcode()
{
    uint8_t sequence[1] = {0};
    uint32_t id = 0;
    hil_Swd_Seq(8, sequence);

    if (CALL_MEMBER_FN_PTR(_Distinct_Methods.write_read_Dp)(0, &id, READ) != SWD_ACK)
    {
        return 0xFFFF;
    }
    return id;
}


uint32_t hil_Jtag_read_idcode()
{
    uint32_t JtagId = 0;

    JtagId = CALL_MEMBER_FN_PTR(_Distinct_Methods.Instr04)(IR4_IDCODE);

    if (JtagId == 0x1)
    {
        JtagId = CALL_MEMBER_FN_PTR(_Distinct_Methods.SetReg_XBits32)(0);
    }
    return JtagId;
}


int16_t _hil_Write_Read_Dp_Swd(uint8_t address ,uint32_t *data, uint16_t rnw)
{
    uint8_t retVal = 0;
    uint8_t i = 0;

    uint8_t regiser = SWD_DP | rnw<<1 | (address & 0x0c);

    for (i = 0; i < MAX_RETRY; i++)
    {
        retVal = Swd_TransferData(regiser, data, rnw);
        // if not wait for answer
        if (retVal != 0x2)
        {
            return SWD_ACK;
        }
    }
    if(i == MAX_RETRY || retVal != 0x1)
    {
        return -1;
    }
    return SWD_ACK;
}

////#pragma optimize = none
int16_t _hil_Write_Read_Ap_Swd(uint32_t address, uint32_t *data, uint16_t rnw)
{
    uint64_t retVal;
    uint8_t i = 0;
    uint32_t dummydata = 0;

    uint32_t apsel_bank_sel = address & DP_SELECT_APSEL_MASK;
    apsel_bank_sel |= address & DP_SELECT_APBANKSEL_MASK;

    if (_hil_Write_Read_Dp_Swd(DP_SELECT, &apsel_bank_sel, WRITE) != SWD_ACK)
    {
        return -1;
    }
    uint8_t regiser = SWD_AP | rnw<<1 | (address & 0x0c);
    do
    {
        retVal = Swd_TransferData(regiser, data, rnw);
    }
    while(i++ < MAX_RETRY && retVal != 0x1);

    if(i == MAX_RETRY)
    {
        return -1;
    }

    if(rnw == WRITE)
    {
        uint8_t regiser = SWD_DP | READ<<1 | (DP_RDBUFF & 0x0c);
        do
        {
             retVal = Swd_TransferData(regiser, &dummydata, READ);
        }
        while(i++ < MAX_RETRY && retVal != 0x1);

        if(i == MAX_RETRY)
        {
            return -1;
        }
    }
    else
    {
        do
        {
            Swd_TransferData(regiser, data, rnw);
        }
        while(i++ == MAX_RETRY && retVal != 0x1);

        if(i == MAX_RETRY)
        {
            return -1;
        }
    }
    return SWD_ACK;
}

int16_t _hil_Write_Read_Mem_Ap_Swd(uint16_t ap_sel, uint32_t address, uint32_t *data, uint16_t rnw)
{
    // First Read current CSW value
    uint32_t tmp = 0;
    uint32_t apsel = ((uint32_t)ap_sel);
    if (_hil_Write_Read_Ap_Swd(AP_CSW | apsel, &tmp, READ) != SWD_ACK)
    {
        return -1;
    }

    // Configure the transfer
     tmp = (tmp & ~(AP_CSW_SIZE_MASK | AP_CSW_ADDRINC_MASK)) | AP_CSW_ADDRINC_OFF | AP_CSW_SIZE_32BIT;
    if (_hil_Write_Read_Ap_Swd(AP_CSW | apsel, &tmp, WRITE) != SWD_ACK)
    {
        return -1;
    }

    // Write the address
    tmp = address;
    if (_hil_Write_Read_Ap_Swd(AP_TAR | apsel, &tmp, WRITE) != SWD_ACK)
    {
        return -1;
    }
    // Write/Read data
    return _hil_Write_Read_Ap_Swd(AP_DRW | apsel, data, rnw);
}

// -----------------------------------------------------------------------------
int16_t _hil_Write_Read_Dp_Jtag(uint8_t address ,uint32_t *data, uint16_t rnw)
{
    uint64_t retVal;
    uint8_t i = 0;
    uint64_t dataAlig = 0ull;
    dataAlig = ((uint64_t)*data) << 3;

    dataAlig |= (((address & 0xC) >> 1) + (rnw & 0x1));

    if(CALL_MEMBER_FN_PTR(_Distinct_Methods.Instr04)(DPACC) != 0x1)
    {
        return -1;
    }

    for (i = 0; i < MAX_RETRY; i++)
    {
        retVal = CALL_MEMBER_FN_PTR(_Distinct_Methods.SetReg_XBits35)(&dataAlig);
        // if ack == ACK
        if ((retVal & 0x3) == ACK)
        {
            *data = retVal >> 3;
            return ACK;
        }
    }
    return -1;
}

// -----------------------------------------------------------------------------
int16_t _hil_Write_Read_Ap_Jtag(uint32_t address, uint32_t *data, uint16_t rnw)
{
    uint64_t retVal;
    uint8_t i = 0;
    uint64_t dataAlig = 0ull;

    dataAlig = ((uint64_t)*data) << 3;

    uint32_t apsel_bank_sel = address & DP_SELECT_APSEL_MASK;
    apsel_bank_sel |= address & DP_SELECT_APBANKSEL_MASK;

    if (_hil_Write_Read_Dp_Jtag(DP_SELECT, &apsel_bank_sel, WRITE) != ACK)
    {
        return -1;
    }

    dataAlig |= (((address & 0xC) >> 1) + (rnw & 0x1));

    if(CALL_MEMBER_FN_PTR(_Distinct_Methods.Instr04)(APACC) != 0x1)
    {
        return -1;
    }

    do
    {
        retVal = CALL_MEMBER_FN_PTR(_Distinct_Methods.SetReg_XBits35)(&dataAlig);
    } while(((retVal & 0x3) != ACK) && (++i < MAX_RETRY));

    if(i == MAX_RETRY)
    {
        return -1;
    }
    else
    {
        if(rnw == READ)
        {
            return _hil_Write_Read_Dp_Jtag(DP_RDBUFF, data, READ);
        }
        else
        {
            return ACK;
        }
    }
}

// -----------------------------------------------------------------------------
// Currently only supports single 32-bit transfers
int16_t _hil_Write_Read_Mem_Ap_Jtag(uint16_t ap_sel, uint32_t address, uint32_t *data, uint16_t rnw)
{
    // First Read current CSW value
    uint32_t tmp = 0;
    uint32_t apsel = (((uint32_t)ap_sel) << 24);
    if (_hil_Write_Read_Ap_Jtag(AP_CSW | apsel, &tmp, READ) != ACK)
    {
        return -1;
    }

    // Configure the transfer
    tmp = (tmp & ~(AP_CSW_SIZE_MASK | AP_CSW_ADDRINC_MASK)) | AP_CSW_ADDRINC_OFF | AP_CSW_SIZE_32BIT;
    if (_hil_Write_Read_Ap_Jtag(AP_CSW | apsel, &tmp, WRITE) != ACK)
    {
        return -1;
    }

    // Write the address
    tmp = address;
    if (_hil_Write_Read_Ap_Jtag(AP_TAR | apsel, &tmp, WRITE) != ACK)
    {
        return -1;
    }
    // Write/Read data
    return _hil_Write_Read_Ap_Jtag(AP_DRW | apsel, data, rnw);
}












uint8_t lastTestState  = 0;
uint8_t lastResetState = 0;

void hil_fpga_init(void)
{
    UNIMP_FN();
}

void startWdtTimeOutCounter(uint32_t timeout)
{
    UNIMP_FN();
}

void stopWdtTimeOutCounter()
{
    UNIMP_FN();
}

void _hil_FpgaAccess_setTimeOut(uint16_t state)
{
    UNIMP_FN();
}

void initJtagBypass(struct jtag tmp)
{
    UNIMP_FN();
}

//enable bypass
void hil_fpga_enable_bypass()
{
    UNIMP_FN();
}

// Leave bypass
void hil_fpga_disable_bypass(void)
{
    UNIMP_FN();
}

void hil_fpga_write_cmd_data0(uint8_t cmd, uint8_t data0)
{
    UNIMP_FN();
}

void hil_fpga_write_cmd_data0_432(uint8_t cmd, uint8_t data0)
{
    UNIMP_FN();
}

// Write CMD, DATA0 and DATA1 to FPGA
void hil_fpga_write_cmd_data0_data1(uint8_t cmd, uint8_t data0, uint16_t data1)
{
    UNIMP_FN();
}

// Write CMD, DATA0 and DATA1 to FPGA
void hil_fpga_write_cmd_data0_data1_count_432(uint8_t cmd, uint8_t data0, uint16_t data1Buf[], uint8_t count)
{
    UNIMP_FN();
}

// Read data from FPGA
void hil_fpga_read_data_432(uint8_t count, uint16_t *buf)
{
    UNIMP_FN();
}

// Write CMD, DATA0 and DATA1 to FPGA
void hil_fpga_write_cmd_data0_data1_count(uint8_t cmd, uint8_t data0, uint16_t data1Buf[], uint8_t count)
{
    UNIMP_FN();
}

// Read data from FPGA
void hil_fpga_read_data1(uint8_t count, uint16_t *buf)
{
    UNIMP_FN();
}

// Hi-Z JTAG signals to target, except TEST and RST (actively drive 0)
void hil_fpga_power_up_target(void)
{
    UNIMP_FN();
}

uint16_t hil_fpga_get_version(void)
{
    return FPGA_VERSION;
}









void hil_DacControlInitDAC0(void)
{
    UNIMP_FN();
}

void hil_DacControlSetDAC0(uint16_t dacVal)
{
    UNIMP_FN();
}

void hil_DacControlInitDAC1(void)
{
    UNIMP_FN();
}

void hil_DacControlSetDAC1(uint16_t dacVal)
{
    UNIMP_FN();
}














uint16_t runningAverageBufferSpVcc[8] = {0,0,0,0,0,0,0,0};
uint16_t runningBufferVCCDtSense = 0;
uint16_t runningAverageBufferDtVcc[8] = {0,0,0,0,0,0,0,0};
uint16_t runningAverageBufferExtVcc[8] = {0,0,0,0,0,0,0,0};
uint16_t isVCC = 0, dtVCC = 0, iexVCC = 0;

/*  In normal operation mode, the  read function is triggered by the ADC12
    interrupt when a sample sequence has been completed.
    During Hil module init (FET startup)  and Set VCC the ADC12 interrupts are
    disabled and the readout is triggered manually by the calling function
*/

#define CAL_ADC_GAIN_FACTOR  *((uint16_t *)0x1A16)
#define CAL_ADC_OFFSET  *((int16_t *)0x1A18)

void hil_AdcControlRead()
{
    UNIMP_FN();
}

/* Init ADC, enable sequence conversion on A1, A3, A4 and A6 ñ Sequence is
    triggered once during main loop execution, only when HIL module is loaded and
    valid
*/
void hil_AdcControlInitADC()
{
    UNIMP_FN();
}

/*----------------------------------------------------------------------------
   This function performs a single AD conversion on the selected pin.
   Uses internal reference 2500mV (VR+ = VREF+).
    Only used during Fuse Blow to measure VCC Fuse
   Arguments: word pinmask (bit position of selected analog input Ax)
   Result:    word (12 bit ADC conversion result)
*/
float ConvertAD(int16_t channel)
{
    UNIMP_FN();
    return 0;
}

/* Only used during Fuse Blow to measure VCC Fuse  */
void hil_AdcControlInitADCvFuse()
{
    UNIMP_FN();
}

// Get 1 sample from A6
uint16_t hil_AdcControlGetDtVccSense(void)
{
    UNIMP_FN();
    return 0;
}

// Get 1 sample from A3
uint16_t hil_AdcControlGetExternalVcc(void)
{
    UNIMP_FN();
    return 0;
}
// Get 1 sample from A1
uint16_t hil_AdcControlGetSupplyVcc(void)
{
    UNIMP_FN();
    return 0;
}
// Get 1 sample from A2
uint16_t hil_AdcControlGetVFuse(void)
{
    UNIMP_FN();
    return 0;
}
// Get 1 sample from A4
uint16_t hil_AdcControlGetVccDt(void)
{
    UNIMP_FN();
    return 0;
}























void TMSH_DMA()
{
    UNIMP_FN();
}

void TMSL_DMA()
{
    UNIMP_FN();
}

void TMSLDH_DMA()
{
    UNIMP_FN();
}

void TDIH_DMA()
{
    UNIMP_FN();
}

void TDIL_DMA()
{
    UNIMP_FN();
}

void TDOsbwDma()
{
    UNIMP_FN();
}

void TDOsbwFuse()
{
    UNIMP_FN();
}

void TDO_RD_FUSE()
{
    UNIMP_FN();
}

void TDO_RD_DMA()
{
    UNIMP_FN();
}

void initJtagSbw2Dma(struct jtag tmp)
{
    UNIMP_FN();
}

#ifdef MSP_FET
void setProtocolSbw2Dma(uint16_t id)
{
    UNIMP_FN();
}
#endif

uint8_t _hil_2w_GetPrevInstruction_Dma()
{
    UNIMP_FN();
    return 0;
}

uint64_t sbw_ShiftDma(uint64_t Data, uint16_t Bits)
{
    UNIMP_FN();
    return 0;
}

int16_t _hil_2w_TapReset_Dma(void)
{
    // From msp_fet:_hil_2w_TapReset()
    uint16_t i;
    
    // Reset JTAG FSM
    for (i = 6; i > 0; i--)      // 6 is nominal
    {
        TMSH_TDIH();
    }
    // JTAG FSM is now in Test-Logic-Reset
    TMSL_TDIH();                 // now in Run/Test Idle
    return 0;
}

int16_t _hil_2w_CheckJtagFuse_Dma(void)
{
    // From _hil_2w_CheckJtagFuse()
    TMSL_TDIH();
    TMSH_TDIH();
    _hil_Delay_1ms(1);
    
    TMSL_TDIH();
    TMSH_TDIH();
    _hil_Delay_1ms(1);
    
    TMSL_TDIH();
    TMSH_TDIH();
    _hil_Delay_1ms(1);
    // In every TDI slot a TCK for the JTAG machine is generated.
    // Thus we need to get TAP in Run/Test Idle state back again.
    TMSH_TDIH();
    TMSL_TDIH();
    return 0;
}

SBWShiftProxy<uint8_t> _hil_2w_Instr_Dma(uint8_t Instruction)
{
    UNIMP_FN();
    return 0;
}

void hil_2w_SetReg_XBits8_64_Entry_DMA()
{
    UNIMP_FN();
}

void hil_2w_SetReg_XBits8_64_Exit_DMA()
{
    UNIMP_FN();
}

uint64_t hil_2w_SetReg_XBits8_64_Run_Dma(uint64_t Data, uint8_t * DataState, uint16_t JStateVersion)
{
    UNIMP_FN();
    return 0;
}

uint64_t _hil_2w_SetReg_XBits8_64_Dma(uint64_t Data, uint16_t loopCount, uint16_t JStateVersion)
{
    UNIMP_FN();
    return 0;
}

SBWShiftProxy<uint8_t> _hil_2w_SetReg_XBits08_Dma(uint8_t data)
{
    UNIMP_FN();
    return 0;
}

SBWShiftProxy<uint16_t> _hil_2w_SetReg_XBits16_Dma(uint16_t data)
{
    UNIMP_FN();
    return 0;
}

SBWShiftProxy<uint32_t,20> _hil_2w_SetReg_XBits20_Dma(uint32_t data)
{
    UNIMP_FN();
    return 0;
}

SBWShiftProxy<uint32_t> _hil_2w_SetReg_XBits32_Dma(uint32_t data)
{
    UNIMP_FN();
    return 0;
}

SBWShiftProxy<uint64_t> _hil_2w_SetReg_XBits64_Dma(uint64_t data)
{
    UNIMP_FN();
    return 0;
}

void _hil_2w_Tclk_Dma(uint8_t state)
{
    UNIMP_FN();
}

void _hil_2w_StepPsa_Dma_Xv2(uint32_t length)
{
    UNIMP_FN();
}

void _hil_2w_StepPsa_Dma(uint32_t length)
{
    UNIMP_FN();
}

void _hil_2w_StepPsaTclkHigh_Dma(uint32_t length)
{
    UNIMP_FN();
}

void _hil_2w_ConfigureSpeed_Dma(uint16_t speed)
{
    UNIMP_FN();
}

int16_t _hil_2w_BlowFuse_Dma(uint8_t targetHasTestVpp)
{
    UNIMP_FN();
    return 0;
}










uint16_t prevInstruction_hil_generic_ = 0;

//#pragma optimize=low
void _hil_generic_ConfigureSpeed(uint16_t speed)
{
    uint16_t tckFreq, tclkFreq;

    switch(speed)
        {
            case SBW100KHz:
                {
                    tckFreq = 298;
                    tclkFreq = 75;
                    break;
                }
            case SBW200KHz:
                {
                    tckFreq = 350;
                    tclkFreq = 75;
                    break;
                }
            case SBW400KHz:
                {
                    tckFreq = 137;
                    tclkFreq = 30;
                    break;
                }
            case SBW600KHz:
                {
                    tckFreq = 88;
                    tclkFreq = 21;
                    break;
                }
            case SBW1200KHz:
                {
                    tckFreq = 48;
                    tclkFreq = 12;
                    break;
                }
            case JTAG250KHz:
                {
                    tckFreq = 238;
                    tclkFreq = 238;
                    break;
                }
            case JTAG500KHz:
                {
                    tckFreq = 118;
                    tclkFreq = 28;
                    break;
                }
             case JTAG750KHz:
                {
                    tckFreq = 88;
                    tclkFreq = 21;
                    break;
                }
            case JTAG1MHz:
                {
                    tckFreq = 58;
                    tclkFreq = 14;
                    break;
                }
            case JTAG2MHz:
                {
                    tckFreq = 28;
                    tclkFreq = 7;
                    break;
                }
            case JTAG4MHz:
                {
                    tckFreq = 13;
                    tclkFreq = 4;
                    break;
                }
            case JTAG8MHz:
                {   // 7,5 MHz
                    tckFreq = 5;
                    tclkFreq = 2;
                    break;
                }
            case JTAG15MHz:
                {
                    tckFreq = 3;
                    tclkFreq = 3;
                    break;
                }
            default:
                {
                    // 500 kHz, but this should never happen
                    tckFreq = 118;
                    tclkFreq = 118;
                    break;
                }
        }
    hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_TEST_CLK_FREQUENCY, tckFreq);
    hil_fpga_write_cmd_data0_data1(FPGA_CMD_CFG, REG_TCLK_CLK_FREQUENCY, tclkFreq);
}

void initGeneric()
{
    prevInstruction_hil_generic_ = 0;
}

uint8_t _hil_generic_GetPrevInstruction()
{
    return prevInstruction_hil_generic_;
}

SBWShiftProxy<uint8_t> _hil_generic_Instr(uint8_t instruction)
{
    switch (gprotocol_id) {
    case SPYBIWIRE:
        // JTAG FSM state = Run-Test/Idle
        if (TCLK_saved) //PrepTCLK
        {
            TMSH_TDIH();
        }
        else
        {
            TMSH_TDIL();
        }
        // JTAG FSM state = Select DR-Scan
        TMSH_TDIH();
        // JTAG FSM state = Select IR-Scan
        TMSL_TDIH();
        // JTAG FSM state = Capture-IR
        TMSL_TDIH();
        // JTAG FSM state = Shift-IR, Shiftin TDI (8 bit)
        return SBWShiftProxy<uint8_t>(this, instruction);
        // JTAG FSM state = Run-Test/Idle
    default:
        BAD_PROTO(gprotocol_id);
        return 0;
    }
}

SBWShiftProxy<uint8_t> _hil_generic_SetReg_XBits08(uint8_t data)
{
    switch (gprotocol_id) {
    case SPYBIWIRE:
        // From msp_fet:_hil_2w_SetReg_XBits08()
        // JTAG FSM state = Run-Test/Idle
        if (TCLK_saved) //PrepTCLK
        {
            TMSH_TDIH();
        }
        else
        {
            TMSH_TDIL();
        }
        // JTAG FSM state = Select DR-Scan
        TMSL_TDIH();
        // JTAG FSM state = Capture-DR
        TMSL_TDIH();
        // JTAG FSM state = Shift-DR, Shiftin TDI (16 bit)
        return SBWShiftProxy<uint8_t>(this, data);
        // JTAG FSM state = Run-Test/Idle
    default:
        BAD_PROTO(gprotocol_id);
        return 0;
    }
}

SBWShiftProxy<uint16_t> _hil_generic_SetReg_XBits16(uint16_t data)
{
    switch (gprotocol_id) {
    case SPYBIWIRE:
        // From msp_fet:_hil_2w_SetReg_XBits16()
        // JTAG FSM state = Run-Test/Idle
        if (TCLK_saved) //PrepTCLK
        {
            TMSH_TDIH();
        }
        else
        {
            TMSH_TDIL();
        }
        // JTAG FSM state = Select DR-Scan
        TMSL_TDIH();
        // JTAG FSM state = Capture-DR
        TMSL_TDIH();
        // JTAG FSM state = Shift-DR, Shiftin TDI (16 bit)
        return SBWShiftProxy<uint16_t>(this, data);
        // JTAG FSM state = Run-Test/Idle
    default:
        BAD_PROTO(gprotocol_id);
        return 0;
    }
}

SBWShiftProxy<uint32_t,20> _hil_generic_SetReg_XBits20(uint32_t data)
{
    switch (gprotocol_id) {
    case SPYBIWIRE:
        // From msp_fet:_hil_2w_SetReg_XBits20()
        // JTAG FSM state = Run-Test/Idle
        if (TCLK_saved) //PrepTCLK
        {
            TMSH_TDIH();
        }
        else
        {
            TMSH_TDIL();
        }
        // JTAG FSM state = Select DR-Scan
        TMSL_TDIH();
        // JTAG FSM state = Capture-DR
        TMSL_TDIH();
        // JTAG FSM state = Shift-DR, Shiftin TDI (16 bit)
        return SBWShiftProxy<uint32_t,20>(this, data);
        // JTAG FSM state = Run-Test/Idle
    default:
        BAD_PROTO(gprotocol_id);
        return 0;
    }
}

// -----------------------------------------------------------------------------
SBWShiftProxy<uint32_t> _hil_generic_SetReg_XBits32(uint32_t data)
{
    switch (gprotocol_id) {
    case SPYBIWIRE:
        // From msp_fet:_hil_2w_SetReg_XBits32()
        // JTAG FSM state = Run-Test/Idle
        if (TCLK_saved) //PrepTCLK
        {
            TMSH_TDIH();
        }
        else
        {
            TMSH_TDIL();
        }
        // JTAG FSM state = Select DR-Scan
        TMSL_TDIH();
        // JTAG FSM state = Capture-DR
        TMSL_TDIH();
        // JTAG FSM state = Shift-DR, Shiftin TDI (16 bit)
        return SBWShiftProxy<uint32_t>(this, data);
        // JTAG FSM state = Run-Test/Idle
    default:
        BAD_PROTO(gprotocol_id);
        return 0;
    }
}

// -----------------------------------------------------------------------------
SBWShiftProxy<uint64_t> _hil_generic_SetReg_XBits64(uint64_t data)
{
    switch (gprotocol_id) {
    case SPYBIWIRE:
        // From msp_fet:_hil_2w_SetReg_XBits64()
        // JTAG FSM state = Run-Test/Idle
        if (TCLK_saved) //PrepTCLK
        {
            TMSH_TDIH();
        }
        else
        {
            TMSH_TDIL();
        }
        // JTAG FSM state = Select DR-Scan
        TMSL_TDIH();
        // JTAG FSM state = Capture-DR
        TMSL_TDIH();
        // JTAG FSM state = Shift-DR, Shiftin TDI (16 bit)
        return SBWShiftProxy<uint64_t>(this, data);
        // JTAG FSM state = Run-Test/Idle
    default:
        BAD_PROTO(gprotocol_id);
        return 0;
    }
}

//! \brief This function executes an SBW2 64BIT Data SHIFT (DR-SHIFT) in the case,
//! that the first 8 bits show a valid data capture from the JSTATE register. In case
//! of no valid capture the shift is ended after the first 8 bits. Timeout could be
//! used to set the function run count.
//! \param[in]  Data to be shifted into target device
//! \return Value shifted out of target device JTAG module
uint64_t _hil_generic_XBits8_64(uint64_t Data, uint16_t loopCount,uint16_t PG)
{
   uint64_t TDOvalue = 0x0;
    uint16_t timeout = loopCount;
    uint8_t DataState = 0;
    uint16_t currentDeviceState = 0;
    uint16_t syncBrokenCount = loopCount/2;

    do
    {
        TDOvalue = (_hil_generic_SetReg_XBits64(Data));
        // Mask out all not needed bits for device state detection
        currentDeviceState = ((TDOvalue >> 56) & JSTATE_FLOW_CONTROL_BITS);

        // check, if BIT[63, 62, 61, 60, 59  58, 57, 56, ] & 0x4 (BP_HIT) == TRUE
        if(currentDeviceState & JSTATE_BP_HIT)
        {
            // reload Jstate IR
            _hil_generic_Instr(IR_JSTATE_ID);
            DataState = VALID_DATA;
            return(TDOvalue);
        }

        // check, if BIT[63, 62, 61, 60, 59  58, 57, 56] (AM Sync. ongoing) == 0x83
        else if(currentDeviceState  == JSTATE_SYNC_ONGOING)
        {
            DataState = SYNC_ONGOING;
        }

        //check, if BIT[63, 62, 61, 60, 59  58, 57, 56] & 0x40 (Locked State) == 0x40
        else if((currentDeviceState & JSTATE_LOCKED_STATE) == JSTATE_LOCKED_STATE &&
                (currentDeviceState & JSTATE_LPM_X_FIVE) != JSTATE_LPM_X_FIVE)
        {
            DataState = JTAG_LOCKED;
            return(TDOvalue);
        }

        //check, if BIT[63, 62, 61, 60, 59  58, 57, 56] ( Invalid LPM) == 0x81)
        else if(currentDeviceState == JSTATE_INVALID_STATE)
        {
            // reload Jstate IR
            _hil_generic_Instr(IR_JSTATE_ID);
            DataState = INVALID_DATA;
        }
        /*PG2.0 && PG2.1 frozen Sync detection*/
       else if (((currentDeviceState & JSTATE_SYNC_BROKEN_MASK) ==  JSTATE_SYNC_BROKEN_MCLK)
                    ||((currentDeviceState & JSTATE_SYNC_BROKEN_MASK) ==  JSTATE_SYNC_BROKEN_MCLK_PGACT)
                    ||((currentDeviceState & JSTATE_SYNC_BROKEN_MASK) ==  JSTATE_SYNC_BROKEN_PGACT))
        {
           if(syncBrokenCount > 0 && PG >= 0x21)

           { // Only working for PG2.1, do not harm if executed on PG2.0 ? but will not create any effect only consume time
                uint32_t current3VtestReg = 0;

                // read current 3VtestReg value
                _hil_generic_Instr(IR_TEST_REG);
                current3VtestReg = _hil_generic_SetReg_XBits32(0);
                // set bit 25 high to rest sync
                current3VtestReg |= 0x2000000;
                _hil_generic_SetReg_XBits32(current3VtestReg);
                // set bit 25 low reset sync done
                current3VtestReg &= ~0x2000000;
                _hil_generic_SetReg_XBits32(current3VtestReg);

                _hil_generic_Instr(IR_JSTATE_ID);
                syncBrokenCount --;
                timeout = timeout + 5;
            }
            else
            {
                DataState = VALID_DATA;
                return(TDOvalue);
            }
        }
        // device is not in LPMx or AC read out mode just restart the shift but do not reload the JState IR
        else if(currentDeviceState != JSTATE_VALID_CAPTURE
                &&  currentDeviceState !=   JSTATE_LPM_ONE_TWO
                &&  currentDeviceState !=   JSTATE_LPM_THREE_FOUR
                && (currentDeviceState &    JSTATE_LPM_X_FIVE) != JSTATE_LPM_X_FIVE)
        {
            DataState = INVALID_DATA;
        }
        else
        {
            DataState = VALID_DATA;
        }

        if(!timeout)
        {
            return TDOvalue;
        }
        timeout--;
    }
    while(DataState == INVALID_DATA || DataState == SYNC_ONGOING);

    if(      currentDeviceState == JSTATE_LPM_ONE_TWO
        ||   currentDeviceState == JSTATE_LPM_THREE_FOUR
        ||   currentDeviceState == 0x00
        ||   currentDeviceState == 0x02)
    {
        // reload Jstate IR
        _hil_generic_Instr(IR_JSTATE_ID);
    }
    return(TDOvalue);
}


// -----------------------------------------------------------------------------
void _hil_generic_Tclk(uint8_t tclk)
{
    _sbwioTclk(0, tclk);
}










void initJtagSbw4(struct jtag tmp)
{
    UNIMP_FN();
}

// -----------------------------------------------------------------------------
int16_t _hil_4w_CheckJtagFuse(void)
{
    UNIMP_FN();
    return 0;
}

// -----------------------------------------------------------------------------
int16_t _hil_4w_TapReset(void)
{
    UNIMP_FN();
    return 0;
}

// -----------------------------------------------------------------------------
void _hil_4w_StepPsa(uint32_t length)
{
    UNIMP_FN();
}
// -----------------------------------------------------------------------------
void _hil_4w_StepPsaTclkHigh(uint32_t length)
{
    UNIMP_FN();
}

int16_t _hil_4w_BlowFuse(uint8_t targetHasTestVpp)
{
    UNIMP_FN();
    return 0;
}

void hil_4w_432_Seq(uint16_t length, uint8_t *sequence)
{
    UNIMP_FN();
}













void SWDTCKset1()
{
    UNIMP_FN();
}

void SWDTCKset0()
{
    UNIMP_FN();
}

void SWDIOset1TckCycle()
{
    UNIMP_FN();
}

void SWDIOset0TckCycle()
{
    UNIMP_FN();
}

void SWDTckCycle()
{
    UNIMP_FN();
}

void SWDIOwriteBit(uint8_t bit)
{
    UNIMP_FN();
}

void hil_Swd_InitJtag(struct jtag tmp)
{
    UNIMP_FN();
}

uint8_t Swd_TransferData(uint8_t regiser, uint32_t* data, uint8_t rnw)
{
    UNIMP_FN();
    return 0;
}

void hil_Swd_Seq(uint16_t length, uint8_t *sequence)
{
    UNIMP_FN();
}












SBWShiftProxy<uint8_t> _hil_generic432_Instr_4(uint8_t instruction)
{
    UNIMP_FN();
    return 0;
}

SBWShiftProxy<uint8_t> _hil_generic432_SetReg_XBits08(uint8_t data)
{
    UNIMP_FN();
    return 0;
}

SBWShiftProxy<uint16_t> _hil_generic432_SetReg_XBits16(uint16_t data)
{
    UNIMP_FN();
    return 0;
}

// -----------------------------------------------------------------------------
SBWShiftProxy<uint32_t> _hil_generic432_SetReg_XBits32(uint32_t data)
{
    UNIMP_FN();
    return 0;
}

// -----------------------------------------------------------------------------
SBWShiftProxy<uint64_t> _hil_generic432_SetReg_XBits64(uint64_t data)
{
    UNIMP_FN();
    return 0;
}

uint64_t _hil_generic432_SetReg_XBits(uint64_t *data, uint16_t count)
{
    UNIMP_FN();
    return 0;
}

uint64_t _hil_generic432_SetReg_XBits35(uint64_t *data)
{
    UNIMP_FN();
    return 0;
}












bool TCLK_saved = 1;

void TMSL_TDIL() { _sbwio(0,0); }
void TMSH_TDIL() { _sbwio(1,0); }
void TMSL_TDIH() { _sbwio(0,1); }
void TMSH_TDIH() { _sbwio(1,1); }

void TMSL_TDIL_TDOrd() { _sbwio_r(0,0); }
void TMSH_TDIL_TDOrd() { _sbwio_r(1,0); }
void TMSL_TDIH_TDOrd() { _sbwio_r(0,1); }
void TMSH_TDIH_TDOrd() { _sbwio_r(1,1); }

void _sbwio(bool tms, bool tdi) {
    // With no `tclk` specified, use the value for TMS, so that the line stays constant
    // between registering the TMS value and outputting the TDI value
    _msp.sbwIO(tms, tms, tdi, false);
}

void _sbwio_r(bool tms, bool tdi) {
    // With no `tclk` specified, use the value for TMS, so that the line stays constant
    // between registering the TMS value and outputting the TDI value
    _msp.sbwIO(tms, tms, tdi, true);
}

void _sbwioTclk(bool tms, bool tclk) {
    _msp.sbwIO(tms, TCLK_saved, tclk, false);
    TCLK_saved = tclk;
}

uint64_t _read(uint8_t w) {
    uint8_t b[4];
    assert(w <= sizeof(b));
    _msp.sbwRead(b, w);
    
         if (w == 1)    return                                 b[0]<<0  ;
    else if (w == 2)    return                       b[0]<<8 | b[1]<<0  ;
    else if (w == 3)    return            b[0]<<16 | b[1]<<8 | b[2]<<0  ;
    else if (w == 4)    return b[0]<<24 | b[1]<<16 | b[2]<<8 | b[3]<<0  ;
    else                abort(); // invalid width
}

//uint64_t _read(uint8_t w) {
//    uint8_t b[8];
//    _msp.sbwRead(&b, w/8+(w%8?1:0));
//    
//    if (w == 8) {
//        return b[0]<<0;
//    } else if (w == 16) {
//        return b[0]<<8 | b[1]<<0;
//    } else if (w == 20) {
//        return b[0]<<12 | b[1]<<4 | ((b[2]<<0)&0x0F);
//    } else if (w == 32) {
//        return b[0]<<24 | b[1]<<16 | b[2]<<8 | b[3]<<0;
//    } else {
//        abort();
//    }
//}

void sbw_Shift(uint64_t data, uint16_t width) {
    // MSPFETSim: custom implementation
    // <- Shift-DR
    for (uint64_t msb=(UINT64_C(1)<<(width-1)); msb; msb>>=1) {
        const bool tms = (msb & 1); // Last bit requires TMS=1
        const bool tdi = data&msb;
        _sbwio(tms, tdi);
    }
    // <- Exit1-DR
    
    // Return to Run-Test/Idle state
    _sbwio(1, TCLK_saved);
    _sbwio(0, TCLK_saved);
}

uint64_t sbw_Shift_R(uint64_t data, uint16_t width) {
    // MSPFETSim: custom implementation
    // <- Shift-DR
    for (uint64_t msb=(UINT64_C(1)<<(width-1)); msb; msb>>=1) {
        const bool tms = (msb & 1); // Last bit requires TMS=1
        const bool tdi = data&msb;
        _sbwio_r(tms, tdi);
    }
    // <- Exit1-DR
    
    // Return to Run-Test/Idle state
    // If we're reading a 20-bit address, read 4 extra dummy bits so we have
    // 3 full bytes of data enqueued (24 bits instead of 20 bits)
    switch (width) {
    case F_ADDR:
        _sbwio_r(1, TCLK_saved);
        _sbwio_r(0, TCLK_saved);
        // <- Run-Test/Idle
        // 2 extra cycles in the Run-Test/Idle state to read 2 more bits
        _sbwio_r(0, TCLK_saved);
        _sbwio_r(0, TCLK_saved);
        break;
    default:
        _sbwio(1, TCLK_saved);
        _sbwio(0, TCLK_saved);
        // <- Run-Test/Idle
        break;
    }
    
    uint64_t tdo = _read((width+7)/8);
    if (width == F_ADDR) {
        // For 20-bit addresses, scramble the result because the rest of
        // the TI codebase expects addresses to be scrambled for some
        // reason. When the caller unscrambles our result, they get the
        // value we started with, [1b]:
        // 
        //      [1a]        [1b]        [2]         [3]
        //   0xABCDEF -> 0x0ABCDE -> 0x0EABCD -> 0x0ABCDE
        // 
        // As shown above, we read [1a], remove the bottom 4 dummy bits
        // to get [1b] (see comment above), scramble and return [2], and
        // the caller de-scrambles to get [3].
        tdo = ((tdo&0x0000F0)<<12) | ((tdo&0xFFFF00)>>8);
    }
    return tdo;
}

void _flush() {
    _msp.sbwRead(nullptr, 0);
}
