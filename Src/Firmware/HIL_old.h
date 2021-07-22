#pragma once
#include "Types.h"
#include "MSPInterface.h"
#include "RuntimeError.h"

#define HIL_UNIMP           printf("### HIL: UNIMPLEMENTED: %s\n", __FUNCTION__)
#define HIL_UNIMP_RET0      HIL_UNIMP; return 0

#define EXTERNAL_VCC_ON    1
#define EXTERNAL_VCC_OFF   0

#define REGULATION_ON    1
#define REGULATION_OFF   0

#define RSTHIGH 0
#define RSTLOW  1

uint16_t gprotocol_id = JTAG;
uint16_t jtagReleased = 1;
uint16_t hil_sbw2Speed_ = 0;
uint16_t hil_jtagSpeed_ = 0;
int16_t dtVccSenseState = 0;
uint16_t gTclkHighWhilePsa = 0;
uint16_t setValueVcc = 0;
uint16_t externalVccOn = 0;
uint16_t regulationOn = 0;
uint16_t prevInstruction = 0;
bool TCLK_saved = 1;

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
    uint8_t b[8];
    _msp.sbwRead(&b, w/8+(w%8?1:0));
    
    if (w == 8) {
        return b[0]<<0;
    } else if (w == 16) {
        return b[0]<<8 | b[1]<<0;
    } else if (w == 20) {
        return b[0]<<12 | b[1]<<4 | ((b[2]<<0)&0x0F);
    } else if (w == 32) {
        return b[0]<<24 | b[1]<<16 | b[2]<<8 | b[3]<<0;
    } else {
        abort();
    }
}

#pragma push_macro("static")
#undef static
template <class...> static constexpr std::false_type _AlwaysFalse = {};
#pragma pop_macro("static")

template <typename T, size_t W=sizeof(T)*8>
class SBWShiftProxy {
public:
    // Default constructor returns a nop object that just returns 0
    SBWShiftProxy() {}
    SBWShiftProxy(MSPProbeSim* self, uint64_t data) : _self(self), _data(data) {}
    // Copy constructor: not allowed
    SBWShiftProxy(const SBWShiftProxy& x) = delete;
    // Move constructor: not allowed
    SBWShiftProxy(SBWShiftProxy&& x) = delete;
    
    ~SBWShiftProxy() {
        if (!_self) return; // Short-circuit if we're a nop object
        if (!_read) {
            // Perform non-read shift
                 if constexpr (W ==  8) _self->sbw_Shift(_data, F_BYTE);
            else if constexpr (W == 16) _self->sbw_Shift(_data, F_WORD);
            else if constexpr (W == 20) _self->sbw_Shift(_data, F_ADDR);
            else if constexpr (W == 32) _self->sbw_Shift(_data, F_LONG);
            else if constexpr (W == 64) _self->sbw_Shift(_data, F_LONG_LONG);
            else                        static_assert(_AlwaysFalse<T>);
        }
    }
    
    operator T() {
        if (!_self) return 0; // Short-circuit if we're a nop object
        
        // Perform read shift and return result
        _read = true;
             if constexpr (W ==  8) return _self->sbw_Shift_R(_data, F_BYTE);
        else if constexpr (W == 16) return _self->sbw_Shift_R(_data, F_WORD);
        else if constexpr (W == 20) return _self->sbw_Shift_R(_data, F_ADDR);
        else if constexpr (W == 32) return _self->sbw_Shift_R(_data, F_LONG);
        else if constexpr (W == 64) return _self->sbw_Shift_R(_data, F_LONG_LONG);
        else                        static_assert(_AlwaysFalse<T>);
    }
    
private:
    MSPProbeSim* _self = nullptr;
    uint64_t _data = 0;
    bool _read = false;
};

void TMSL_TDIL() { _sbwio(0,0); }
void TMSH_TDIL() { _sbwio(1,0); }
void TMSL_TDIH() { _sbwio(0,1); }
void TMSH_TDIH() { _sbwio(1,1); }

void TMSL_TDIL_TDOrd() { _sbwio_r(0,0); }
void TMSH_TDIL_TDOrd() { _sbwio_r(1,0); }
void TMSL_TDIH_TDOrd() { _sbwio_r(0,1); }
void TMSH_TDIH_TDOrd() { _sbwio_r(1,1); }

void sbw_Shift(uint64_t Data, int16_t Bits)
{
    uint64_t MSB = 0x0000000000000000;

    switch(Bits)
    {
        case F_BYTE: MSB = 0x00000080;
            break;
        case F_WORD: MSB = 0x00008000;
            break;
        case F_ADDR: MSB = 0x00080000;
            break;
        case F_LONG: MSB = 0x80000000;
            break;
        case F_LONG_LONG: MSB = 0x8000000000000000;
            break;
        default: // this is an unsupported format
            abort();
    }
    do
    {
        if ((MSB & 1) == 1)                       // Last bit requires TMS=1
        {
            if(Data & MSB)
            {
                TMSH_TDIH();
            }
            else
            {
                TMSH_TDIL();
            }
        }
        else
        {
            if(Data & MSB)
            {
                TMSL_TDIH();
            }
            else
            {
                TMSL_TDIL();
            }
        }
    }
    while(MSB >>= 1);

    if (TCLK_saved)
    {
        TMSH_TDIH();
        TMSL_TDIH();
    }
    else
    {
        TMSH_TDIL();
        TMSL_TDIL();
    }
}

uint64_t sbw_Shift_R(uint64_t Data, int16_t Bits)
{
    uint64_t MSB = 0x0000000000000000;

    switch(Bits)
    {
        case F_BYTE: MSB = 0x00000080;
            break;
        case F_WORD: MSB = 0x00008000;
            break;
        case F_ADDR: MSB = 0x00080000;
            break;
        case F_LONG: MSB = 0x80000000;
            break;
        case F_LONG_LONG: MSB = 0x8000000000000000;
            break;
        default: // this is an unsupported format
            abort();
    }
    do
    {
        if ((MSB & 1) == 1)                       // Last bit requires TMS=1
        {
            if(Data & MSB)
            {
                TMSH_TDIH_TDOrd();
            }
            else
            {
                TMSH_TDIL_TDOrd();
            }
        }
        else
        {
            if(Data & MSB)
            {
                TMSL_TDIH_TDOrd();
            }
            else
            {
                TMSL_TDIL_TDOrd();
            }
        }
    }
    while(MSB >>= 1);

    if (TCLK_saved)
    {
        TMSH_TDIH();
        TMSL_TDIH();
    }
    else
    {
        TMSH_TDIL();
        TMSL_TDIL();
    }
    uint64_t TDOvalue = _read(Bits);
    // de-scramble upper 4 bits if it was a 20bit shift
    if(Bits == F_ADDR)
    {
        TDOvalue = ((TDOvalue >> 4) | (TDOvalue << 16)) & 0x000FFFFF;
    }
    return(TDOvalue);
}

void _hil_Delay_1us(uint16_t us) {
    IHIL_Delay_1us(us);
}

void _hil_Delay_1ms(uint16_t ms) {
    IHIL_Delay_1ms(ms);
}

void RSTset1()
{
    _msp.sbwRstSet(1);
    _hil_Delay_1ms(5);
}

void RSTset0()
{
    _msp.sbwRstSet(0);
    _hil_Delay_1ms(5);
}

void TSTset1()
{
    _msp.sbwTestSet(1);
    _hil_Delay_1ms(5);
}

void TSTset0()
{
    _msp.sbwTestSet(0);
    _hil_Delay_1ms(5);
}

void TSTset1NoDelay()
{
    _msp.sbwTestSet(1);
}

void TSTset0NoDelay()
{
    _msp.sbwTestSet(0);
}

void TCLKset1()
{
    IHIL_Tclk(1);
}

void TCLKset0()
{
    IHIL_Tclk(0);
}

void TCKset1()
{
    // In SBW mode, TCK is pin 0, which is the same assignment TST.
    // See _SBW_Back definition.
    TSTset1();
}
void TCKset0()
{
    // In SBW mode, TCK is pin 0, which is the same assignment TST.
    // See _SBW_Back definition.
    TSTset0();
}

void TCKset1NoDelay()
{
    // In SBW mode, TCK is pin 0, which is the same assignment TST.
    // See _SBW_Back definition.
    TSTset1NoDelay();
}

void TCKset0NoDelay()
{
    // In SBW mode, TCK is pin 0, which is the same assignment TST.
    // See _SBW_Back definition.
    TSTset0NoDelay();
}

void TCLK()
{
    TCLKset0();
    TCLKset1();
}

void hil_fpga_enable_bypass() {
}

void hil_fpga_disable_bypass() {
}

void _flush() {
    _msp.sbwRead(nullptr, 0);
}

// HIL common methods (common = identical implementations for JTAG/SBW )
int16_t IHIL_Init() {
    // From msp_fet:_hil_Init()
    
    // default config
    externalVccOn = EXTERNAL_VCC_OFF;
    regulationOn = REGULATION_OFF;
    dtVccSenseState = 0;
    
    // set default to TCLK low when doing PSA
    gTclkHighWhilePsa = 0;
    // initialize function pointers to distinct functions
    
    // set default debug protocol to JTAG
    gprotocol_id = SPYBIWIREJTAG;
    // set default to TCLK low when doing PSA
    gTclkHighWhilePsa = 0;
    // initialize function pointers to distinct functions
    IHIL_SetProtocol(SPYBIWIRE);
    jtagReleased = 1;
    
    return 0;
};

int16_t IHIL_RegulateVcc() {
    return 0;
};

int16_t IHIL_SetVcc(uint16_t vcc) {
    return 0;
}

void IHIL_SwitchVccFET(uint16_t switchVccFET) {
}

int16_t IHIL_GetVcc(double* Vcc, double* ExtVcc) {
    // From msp_fet:_hil_GetVcc()
    float vccSupply = 3300, externalVcc = 0;

//    externalVcc = hil_AdcControlGetExternalVcc();
//    vccSupply = hil_AdcControlGetSupplyVcc();
//
//    if(externalVcc >= 1650 && externalVcc < 3800)
//    {
//        setValueVcc = (uint16_t)externalVcc;
//        externalVccOn = EXTERNAL_VCC_ON;
//        regulationOn = REGULATION_ON;
//
//        if(gprotocol_id != SWD_432)
//        {
//            hil_fpga_enable_bypass();
//        }
//
//        P8DIR |= (BIT0);
//        P8OUT |= (BIT0);
//
//        _hil_Delay_1ms(10);
//        if(gprotocol_id != SWD_432)
//        {
//            hil_fpga_disable_bypass();
//        }
//    }
//    else
//    {
//        externalVccOn = EXTERNAL_VCC_OFF;
//    }
    *Vcc = vccSupply;
    *ExtVcc = externalVcc;
    return 0;
};

int16_t IHIL_SetProtocol(uint16_t protocol_id) {
    // From msp_fet:_hil_SetProtocol()
    int16_t ret_value = 0;
    
    gprotocol_id = protocol_id;
    initGeneric();
    
    if (gprotocol_id == JTAG) {
//        _Distinct_Methods.TapReset = _hil_4w_TapReset;
//        _Distinct_Methods.CheckJtagFuse = _hil_4w_CheckJtagFuse;
//        _Distinct_Methods.Instr = _hil_generic_Instr;
//        _Distinct_Methods.SetReg_XBits08 = _hil_generic_SetReg_XBits08;
//        _Distinct_Methods.SetReg_XBits16 = _hil_generic_SetReg_XBits16;
//        _Distinct_Methods.SetReg_XBits20 = _hil_generic_SetReg_XBits20;
//        _Distinct_Methods.SetReg_XBits32 = _hil_generic_SetReg_XBits32;
//        _Distinct_Methods.SetReg_XBits64 = _hil_generic_SetReg_XBits64;
//        _Distinct_Methods.SetReg_XBits8_64 =  _hil_generic_XBits8_64;
//        _Distinct_Methods.GetPrevInstruction = _hil_generic_GetPrevInstruction;
//        _Distinct_Methods.Tclk = _hil_generic_Tclk;
//
//        // load dummies for 432
//        _Distinct_Methods.Instr04 = _hil_dummy_Instr_4;
//        _Distinct_Methods.write_read_Dp = _hil_dummy_Write_Read_Dp;
//        _Distinct_Methods.write_read_Ap = _hil_dummy_Write_Read_Ap;
//        _Distinct_Methods.write_read_mem_Ap = _hil_dummy_Write_Read_Mem_Ap;
//        _Distinct_Methods.SetReg_XBits  = _hil_dummy_SetReg_XBits;
//        _Distinct_Methods.SetReg_XBits35 = _hil_dummy_SetReg_XBits35;
//        _Distinct_Methods.SwdTransferData = _hil_dummy_TransferData;
//
//        if(gTclkHighWhilePsa == 0 || gTclkHighWhilePsa == 2)
//        {
//            _Distinct_Methods.StepPsa = _hil_4w_StepPsa;
//        }
//        else
//        {
//            _Distinct_Methods.StepPsa = _hil_4w_StepPsaTclkHigh;
//        }
//        _Distinct_Methods.BlowFuse = _hil_4w_BlowFuse;
    
    } else if (gprotocol_id == SPYBIWIRE) {
//        // Functionality executed in FPGA bypass mode by Sub-MCU
//        _Distinct_Methods.TapReset =            _hil_2w_TapReset_Dma;
//        _Distinct_Methods.CheckJtagFuse =       _hil_2w_CheckJtagFuse_Dma;
//
//        _Distinct_Methods.Instr =               _hil_generic_Instr;
//        _Distinct_Methods.SetReg_XBits08 =      _hil_generic_SetReg_XBits08;
//        _Distinct_Methods.SetReg_XBits16 =      _hil_generic_SetReg_XBits16;
//        _Distinct_Methods.SetReg_XBits20 =      _hil_generic_SetReg_XBits20;
//        _Distinct_Methods.SetReg_XBits32 =      _hil_generic_SetReg_XBits32;
//        _Distinct_Methods.SetReg_XBits64 =      _hil_generic_SetReg_XBits64;
//        _Distinct_Methods.SetReg_XBits8_64 =    _hil_generic_XBits8_64;
//        _Distinct_Methods.GetPrevInstruction =  _hil_generic_GetPrevInstruction;
//        _Distinct_Methods.Tclk =                _hil_generic_Tclk;
//        _Distinct_Methods.SetReg_XBits  =       _hil_dummy_SetReg_XBits;
//        _Distinct_Methods.Instr04       =       _hil_dummy_Instr_4;
//        _Distinct_Methods.write_read_Dp =       _hil_dummy_Write_Read_Dp;
//        _Distinct_Methods.write_read_Ap =       _hil_dummy_Write_Read_Ap;
//        _Distinct_Methods.write_read_mem_Ap =   _hil_dummy_Write_Read_Mem_Ap;
//        _Distinct_Methods.SetReg_XBits35 =      _hil_dummy_SetReg_XBits35;
//        _Distinct_Methods.SwdTransferData =     _hil_dummy_TransferData;
//        
//        if(gTclkHighWhilePsa == 1)
//        {
//            _Distinct_Methods.StepPsa = _hil_2w_StepPsaTclkHigh_Dma;
//        }
//        else if(gTclkHighWhilePsa == 2)
//        {
//            _Distinct_Methods.StepPsa = _hil_2w_StepPsa_Dma_Xv2;
//        }
//        else
//        {
//            _Distinct_Methods.StepPsa = _hil_2w_StepPsa_Dma;
//        }
//        _Distinct_Methods.BlowFuse = _hil_2w_BlowFuse_Dma;
    
    } else {
        printf("### Unsupported protocol: %d\n", gprotocol_id);
//        throw RuntimeError("invalid protocol: %d", protocol_id);
    }
    
    jtagReleased = 0;
    return(ret_value);
};

void IHIL_SetPsaTCLK(uint16_t tclkValue) {
    gTclkHighWhilePsa = tclkValue;
    if (gprotocol_id == JTAG) {
        // TODO: account for gTclkHighWhilePsa when StepPsa is invoked, instead of updating a function pointer here
//        if (gTclkHighWhilePsa == 1) {
//            _Distinct_Methods.StepPsa = _hil_4w_StepPsaTclkHigh;
//        } else {
//            _Distinct_Methods.StepPsa = _hil_4w_StepPsa;
//        }
    
    } else if (gprotocol_id == SPYBIWIRE) {
//        if (gTclkHighWhilePsa == 1) {
//            _Distinct_Methods.StepPsa = _hil_2w_StepPsaTclkHigh_Dma;
//        } else if (gTclkHighWhilePsa == 2) {
//            _Distinct_Methods.StepPsa = _hil_2w_StepPsa_Dma_Xv2;
//        } else {
//            _Distinct_Methods.StepPsa = _hil_2w_StepPsa_Dma;
//        }
    
    } else {
        printf("### Unsupported protocol: %d\n", gprotocol_id);
    }
}

int16_t IHIL_Open(uint8_t state) {
    _hil_Connect(state);
    return 0;
}

int16_t IHIL_Close() {
    // From msp_fet:_hil_Close()
    externalVccOn = EXTERNAL_VCC_OFF;
    _hil_Release();
    return 0;
}

void IHIL_Delay_1us(uint16_t usecs) {
    _flush();
    usleep(usecs);
};

void IHIL_Delay_1ms(uint16_t msecs) {
    _flush();
    usleep(msecs*1000);
};

int16_t IHIL_IccMonitor_Process(uint16_t flags)                 { HIL_UNIMP_RET0;   };
void IHIL_EntrySequences(uint8_t states)                        { HIL_UNIMP;        };
void IHIL_BSL_EntrySequence(uint16_t switchBypassOff)           { HIL_UNIMP;        };

void IHIL_BSL_EntrySequence1xx_4xx() {
    // From msp_fet:_hil_BSL_EntrySequence1xx_4xx()
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

void IHIL_SetReset(uint8_t value)                               { HIL_UNIMP;        };
void IHIL_SetTest(uint8_t value)                                { HIL_UNIMP;        };
void IHIL_SetTMS(uint8_t value)                                 { HIL_UNIMP;        };
void IHIL_SetTCK(uint8_t value)                                 { HIL_UNIMP;        };
void IHIL_SetTDI(uint8_t value)                                 { HIL_UNIMP;        };
void IHIL_InitDelayTimer()                                      { HIL_UNIMP;        };

void IHIL_SetJtagSpeed(uint16_t jtagSpeed, uint16_t sbwSpeed) {
}

void IHIL_ConfigureSetPc(uint16_t PCclockBeforeCapture)         { HIL_UNIMP;        };

// HIL distinct methods (distinct = different implementations for JTAG/SBW )
int16_t IHIL_TapReset() {
    if (gprotocol_id == JTAG) {
        HIL_UNIMP_RET0;
    
    } else {
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
}

int16_t IHIL_CheckJtagFuse() {
    if (gprotocol_id == JTAG) {
        HIL_UNIMP_RET0;
    
    } else {
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
}

void IHIL_Tclk(uint8_t tclk) {
    if (gprotocol_id == JTAG) {
        HIL_UNIMP;
    
    } else {
        _sbwioTclk(0, tclk);
    }
}

SBWShiftProxy<uint8_t> IHIL_Instr(uint8_t Instruction) {
    if (gprotocol_id == JTAG) {
        HIL_UNIMP;
        return {};
    
    } else {
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
        return SBWShiftProxy<uint8_t>(this, Instruction);
    }
}

int8_t IHIL_Instr4(uint8_t ir) {
    if (gprotocol_id == JTAG) {
        HIL_UNIMP_RET0;
    } else {
        HIL_UNIMP_RET0;
    }
}

void IHIL_StepPsa(uint32_t length) {
    if (gprotocol_id == JTAG) {
        HIL_UNIMP;
    } else {
        HIL_UNIMP;
    }
}

int16_t IHIL_BlowFuse(uint8_t targetHasTestVpp) {
    if (gprotocol_id == JTAG) {
        HIL_UNIMP_RET0;
    } else {
        HIL_UNIMP_RET0;
    }
}

uint8_t IHIL_GetPrevInstruction() {
    if (gprotocol_id == JTAG) {
        HIL_UNIMP_RET0;
    } else {
        HIL_UNIMP_RET0;
    }
}

void IHIL_TCLK() {
    if (gprotocol_id == JTAG) {
        HIL_UNIMP;
    
    } else {
        IHIL_Tclk(0);
        IHIL_Tclk(1);
    }
}

void SetReg_XBits(uint64_t *Data, uint16_t count) {
    if (gprotocol_id == JTAG) {
        HIL_UNIMP;
    } else {
        HIL_UNIMP;
    }
}

SBWShiftProxy<uint8_t> SetReg_8Bits(uint8_t Data) {
    if (gprotocol_id == JTAG) {
        HIL_UNIMP;
        return {};
    } else {
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
        return SBWShiftProxy<uint8_t>(this, Data);
        // JTAG FSM state = Run-Test/Idle
    }
}

SBWShiftProxy<uint16_t> SetReg_16Bits(uint16_t Data) {
    if (gprotocol_id == JTAG) {
        HIL_UNIMP;
        return {};
    
    } else {
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
        return SBWShiftProxy<uint16_t>(this, Data);
        // JTAG FSM state = Run-Test/Idle
    }
}

SBWShiftProxy<uint32_t,20> SetReg_20Bits(uint32_t Data) {
    if (gprotocol_id == JTAG) {
        HIL_UNIMP;
        return {};
    
    } else {
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
        return SBWShiftProxy<uint32_t,20>(this, Data);
        // JTAG FSM state = Run-Test/Idle
    }
}

SBWShiftProxy<uint32_t> SetReg_32Bits(uint32_t Data) {
    if (gprotocol_id == JTAG) {
        HIL_UNIMP;
        return {};
    
    } else {
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
        return SBWShiftProxy<uint32_t>(this, Data);
        // JTAG FSM state = Run-Test/Idle
    }
}

SBWShiftProxy<uint64_t> SetReg_35Bits(uint64_t *Data) {
    if (gprotocol_id == JTAG) {
        HIL_UNIMP;
        return {};
    } else {
        HIL_UNIMP;
        return {};
    }
}

SBWShiftProxy<uint64_t> SetReg_64Bits(uint64_t Data) {
    if (gprotocol_id == JTAG) {
        HIL_UNIMP;
        return {};
    } else {
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
        return SBWShiftProxy<uint64_t>(this, Data);
        // JTAG FSM state = Run-Test/Idle
    }
}

// ARM interface functions
int16_t IHIL_Write_Read_Dp(uint8_t address, uint32_t *data, uint16_t rnw)   { HIL_UNIMP_RET0; }
int16_t IHIL_Write_Read_Ap(uint32_t address, uint32_t *data, uint16_t rnw)  { HIL_UNIMP_RET0; }
int16_t IHIL_Write_Read_Mem_Ap(uint16_t ap_sel, uint32_t address,
    uint32_t *data, uint16_t rnw)                                           { HIL_UNIMP_RET0; }
uint8_t IHIL_SwdTransferData(uint8_t regiser, uint32_t* data, uint8_t rnw)  { HIL_UNIMP_RET0; }
uint64_t SetReg8_64Bits(uint64_t data, uint16_t loopCount, uint16_t PG)     { HIL_UNIMP_RET0; }

// JTAG instruction register access
void cntrl_sig_low_byte()                   { IHIL_Instr(IR_CNTRL_SIG_LOW_BYTE); }
SBWShiftProxy<uint8_t> cntrl_sig_capture()  { return IHIL_Instr(IR_CNTRL_SIG_CAPTURE); }
void cntrl_sig_high_byte()                  { IHIL_Instr(IR_CNTRL_SIG_HIGH_BYTE); }
void cntrl_sig_16bit()                      { IHIL_Instr(IR_CNTRL_SIG_16BIT); }
void cntrl_sig_release()                    { IHIL_Instr(IR_CNTRL_SIG_RELEASE); }
void addr_16bit()                           { IHIL_Instr(IR_ADDR_16BIT); }
void addr_capture()                         { IHIL_Instr(IR_ADDR_CAPTURE); }
void data_16bit()                           { IHIL_Instr(IR_DATA_16BIT); }
void data_capture()                         { IHIL_Instr(IR_DATA_CAPTURE); }
void data_to_addr()                         { IHIL_Instr(IR_DATA_TO_ADDR); }
void data_quick()                           { IHIL_Instr(IR_DATA_QUICK); }
void config_fuses()                         { IHIL_Instr(IR_CONFIG_FUSES); }
void eem_data_exchange()                    { IHIL_Instr(IR_EMEX_DATA_EXCHANGE); }
void eem_data_exchange32()                  { IHIL_Instr(IR_EMEX_DATA_EXCHANGE32); }
void eem_read_control()                     { IHIL_Instr(IR_EMEX_READ_CONTROL); }
void eem_write_control()                    { IHIL_Instr(IR_EMEX_WRITE_CONTROL); }
void eem_read_trigger()                     { IHIL_Instr(IR_EMEX_READ_TRIGGER); }
void data_psa()                             { IHIL_Instr(IR_DATA_PSA); }
void shift_out_psa()                        { IHIL_Instr(IR_SHIFT_OUT_PSA); }
void flash_16bit_update()                   { IHIL_Instr(IR_FLASH_16BIT_UPDATE); }
void jmb_exchange()                         { IHIL_Instr(IR_JMB_EXCHANGE); }
void device_ip_pointer()                    { IHIL_Instr(IR_DEVICE_ID); }
void core_ip_pointer()                      { IHIL_Instr(IR_COREIP_ID); }
void jstate_read()                          { IHIL_Instr(IR_JSTATE_ID); }
void test_reg()                             { IHIL_Instr(IR_TEST_REG); }
void test_reg_3V()                          { IHIL_Instr(IR_TEST_3V_REG); }
void prepare_blow()                         { IHIL_Instr(IR_PREPARE_BLOW); }
void ex_blow()                              { IHIL_Instr(IR_EX_BLOW); }

#define OUT1RDY 0x0008
#define OUT0RDY 0x0004
#define IN1RDY  0x0002
#define IN0RDY  0x0001

#define JMB32B  0x0010
#define OUTREQ  0x0004
#define INREQ   0x0001

// JTAG logic functions
int16_t isInstrLoad()
{
    cntrl_sig_capture();
    if((SetReg_16Bits(0) & (CNTRL_SIG_INSTRLOAD | CNTRL_SIG_READ)) != (CNTRL_SIG_INSTRLOAD | CNTRL_SIG_READ))
    {
        return -1;
    }
    return 0;
}

int16_t instrLoad()
{
    uint16_t i = 0;

    cntrl_sig_low_byte();
    SetReg_8Bits(CNTRL_SIG_READ);
    IHIL_Tclk(1);

    for(i = 0; i < 10; i++)
    {
        if(isInstrLoad() == 0)
        {
           return 0;
        }
        IHIL_TCLK();
    }
    return -1;
}

void halt_cpu()
{
    data_16bit();
    SetReg_16Bits(0x3FFF);
    IHIL_Tclk(0);
    cntrl_sig_16bit();
    SetReg_16Bits(0x2409);
    IHIL_Tclk(1);
}

void release_cpu()
{
    IHIL_Tclk(0);
    cntrl_sig_16bit();
    SetReg_16Bits(0x2401);
    addr_capture();
    IHIL_Tclk(1);
}

uint16_t ReadMemWord(uint16_t address)
{
    uint16_t data = 0;
    halt_cpu();
    IHIL_Tclk(0);
    addr_16bit();
    SetReg_16Bits(address);
    data_to_addr();
    IHIL_Tclk(1);
    IHIL_Tclk(0);
    data = SetReg_16Bits(0);
    release_cpu();
    return data;
}

uint16_t ReadMemWordX(uint32_t address)
{
    uint16_t data = 0;
    halt_cpu();
    IHIL_Tclk(0);
    addr_16bit();
    SetReg_20Bits(address & 0xFFFFF);
    data_to_addr();
    IHIL_Tclk(1);
    IHIL_Tclk(0);
    data = SetReg_16Bits(0);
    release_cpu();
    return data;
}

uint16_t ReadMemWordXv2(uint32_t address)
{
    uint16_t data = 0;
    IHIL_Tclk(0);
    addr_16bit();
    SetReg_20Bits(address & 0xFFFFF);
    IHIL_Tclk(1);
    IHIL_Tclk(0);
    data_capture();
    data = SetReg_16Bits(0);
    IHIL_Tclk(1);
    IHIL_Tclk(0);
    IHIL_Tclk(1);
    return data;
}

void WriteMemWord(uint16_t address, uint16_t data)
{
    halt_cpu();
    IHIL_Tclk(0);
    cntrl_sig_low_byte();
    SetReg_8Bits(0x08);
    addr_16bit();
    SetReg_16Bits(address);
    data_to_addr();
    SetReg_16Bits(data);
    IHIL_Tclk(1);
    release_cpu();
}

void WriteMemWordX(uint32_t address, uint16_t data)
{
    halt_cpu();
    IHIL_Tclk(0);
    cntrl_sig_low_byte();
    SetReg_8Bits(0x08);
    addr_16bit();
    SetReg_20Bits(address & 0xFFFFF);
    data_to_addr();
    SetReg_16Bits(data);
    IHIL_Tclk(1);
    release_cpu();
}

void WriteMemWordXv2(uint32_t address, uint16_t data)
{
    IHIL_Tclk(0);
    cntrl_sig_16bit();
    SetReg_16Bits(0x0500);
    addr_16bit();
    SetReg_20Bits(address);
    IHIL_Tclk(1);
    data_to_addr();
    SetReg_16Bits(data);
    IHIL_Tclk(0);
    cntrl_sig_16bit();
    SetReg_16Bits(0x0501);
    IHIL_Tclk(1);
    IHIL_Tclk(0);
    IHIL_Tclk(1);
}

void WriteMemByte(uint32_t address, uint16_t data)
{
    halt_cpu();
    IHIL_Tclk(0);
    addr_16bit();
    SetReg_16Bits(address);
    data_to_addr();
    SetReg_8Bits(data);
    IHIL_Tclk(1);
    IHIL_Tclk(0);
    release_cpu();
}

uint16_t ReadCpuReg_uint16_t(uint16_t reg)
{
    int16_t op = 0;
    uint16_t data = 0;

    cntrl_sig_16bit();
    SetReg_16Bits(0x3401);
    data_16bit();
    op = ((reg << 8) & 0x0F00) | 0x4082;
    SetReg_16Bits(op);
    IHIL_Tclk(0);
    data_capture();
    IHIL_Tclk(1);
    data_16bit();
    SetReg_16Bits(0x00fe);
    IHIL_Tclk(0);
    data_capture();
    IHIL_Tclk(1);
    IHIL_Tclk(0);
    IHIL_Tclk(1);
    SetReg_16Bits(0);
    data = SetReg_16Bits(0);
    IHIL_Tclk(0);
    cntrl_sig_16bit();
    SetReg_16Bits(0x2401);
    IHIL_Tclk(1);
    return data;
}

uint16_t ReadCpuReg(uint16_t reg)
{
    int16_t op = 0;
    uint16_t data = 0;

    cntrl_sig_16bit();
    SetReg_16Bits(0x3401);
    data_16bit();
    op = ((reg << 8) & 0x0F00) | 0x4082;
    SetReg_16Bits(op);
    IHIL_Tclk(0);
    data_capture();
    IHIL_Tclk(1);
    data_16bit();
    SetReg_16Bits(0x00fe);
    IHIL_Tclk(0);
    data_capture();
    IHIL_Tclk(1);
    IHIL_Tclk(0);
    IHIL_Tclk(1);
    data = SetReg_16Bits(0);
    IHIL_Tclk(0);
    cntrl_sig_16bit();
    SetReg_16Bits(0x2401);
    IHIL_Tclk(1);
    return data;
}

uint32_t ReadCpuRegX(uint16_t reg)
{
    int16_t op = 0;
    uint16_t Rx_l = 0;
    uint16_t Rx_h = 0;

    cntrl_sig_high_byte();
    SetReg_16Bits(0x34);
    op = ((reg << 8) & 0x0F00) | 0x60;
    data_16bit();
    IHIL_Tclk(1);
    SetReg_16Bits(op);
    IHIL_Tclk(0);
    data_capture();
    IHIL_Tclk(1);
    data_16bit();
    IHIL_Tclk(1);
    SetReg_16Bits(0x00fc);
    IHIL_Tclk(0);
    data_capture();
    IHIL_Tclk(1);
    Rx_l = SetReg_16Bits(0);
    IHIL_Tclk(0);
    IHIL_Tclk(1);
    Rx_h = SetReg_16Bits(0);
    IHIL_Tclk(0);
    cntrl_sig_high_byte();
    SetReg_8Bits(0x24);
    IHIL_Tclk(1);
    return ((uint32_t)Rx_h<<16) + Rx_l;
}

uint32_t ReadCpuRegXv2(uint16_t reg)
{
    uint16_t Rx_l = 0;
    uint16_t Rx_h = 0;
    const uint16_t jtagId = cntrl_sig_capture();
    const uint16_t jmbAddr = (jtagId == 0x98) ? 0x14c : 0x18c;

    IHIL_Tclk(0);
    data_16bit();
    IHIL_Tclk(1);
    SetReg_16Bits(reg);
    cntrl_sig_16bit();
    SetReg_16Bits(0x1401);
    data_16bit();
    IHIL_TCLK();
    if (altRomAddressForCpuRead)
    {
        SetReg_16Bits(0x0ff6);
    }
    else
    {
        SetReg_16Bits(jmbAddr);
    }
    IHIL_TCLK();
    SetReg_16Bits(0x3ffd);
    IHIL_Tclk(0);
    if (altRomAddressForCpuRead)
    {
        cntrl_sig_16bit();
        SetReg_16Bits(0x0501);
    }
    data_capture();
    IHIL_Tclk(1);
    Rx_l = SetReg_16Bits(0);
    IHIL_TCLK();
    Rx_h = SetReg_16Bits(0);
    IHIL_TCLK();
    IHIL_TCLK();
    IHIL_TCLK();
    if (!altRomAddressForCpuRead)
    {
        cntrl_sig_16bit();
        SetReg_16Bits(0x0501);
    }
    IHIL_Tclk(0);
    data_capture();
    IHIL_Tclk(1);
    return((uint32_t)Rx_h<<16) + Rx_l;
}

void WriteCpuReg(uint16_t reg, uint16_t data)
{
    uint16_t op = 0;
    cntrl_sig_16bit();
    SetReg_16Bits(0x3401);
    data_16bit();
    op = (0x4030 | reg);
    SetReg_16Bits(op);
    IHIL_Tclk(0);
    data_capture();
    IHIL_Tclk(1);
    data_16bit();
    SetReg_16Bits(data);
    IHIL_Tclk(0);
    data_capture();
    IHIL_Tclk(1);
    data_16bit();
    SetReg_16Bits(0x3ffd);
    IHIL_Tclk(0);
    data_capture();
    IHIL_Tclk(1);
    IHIL_Tclk(0);
    cntrl_sig_16bit();
    SetReg_16Bits(0x2401);
    IHIL_Tclk(1);
}

void WriteCpuRegX(uint16_t mova, uint32_t data)
{
    uint16_t op = 0x0080 | mova | ((data >> 8) & 0x0F00);
    cntrl_sig_high_byte();
    SetReg_8Bits(0x34);
    data_16bit();
    IHIL_Tclk(1);
    SetReg_16Bits(op);
    IHIL_Tclk(0);
    data_capture();
    IHIL_Tclk(1);
    data_16bit();
    IHIL_Tclk(1);
    SetReg_16Bits(data & 0xFFFF);
    IHIL_Tclk(0);
    data_capture();
    IHIL_Tclk(1);
    data_16bit();
    IHIL_Tclk(1);
    SetReg_16Bits(0x3ffd);
    IHIL_Tclk(0);
    data_capture();
    IHIL_Tclk(1);
    IHIL_Tclk(0);
    cntrl_sig_high_byte();
    SetReg_8Bits(0x24);
    IHIL_Tclk(1);
}

// BUGFIX BTT1722 added
void WriteCpuRegXv2(uint16_t mova, uint16_t data)
{
    IHIL_Tclk(0);
    data_16bit();
    IHIL_Tclk(1);
    SetReg_16Bits(mova);
    cntrl_sig_16bit();
    SetReg_16Bits(0x1401);
    data_16bit();
    IHIL_TCLK();
    SetReg_16Bits(data);
    IHIL_TCLK();
    SetReg_16Bits(0x3ffd);
    IHIL_TCLK();
    IHIL_Tclk(0);
    addr_capture();
    SetReg_20Bits(0x00000);
    IHIL_Tclk(1);
    cntrl_sig_16bit();
    SetReg_16Bits(0x0501);
    IHIL_TCLK();
    IHIL_Tclk(0);
    data_capture();
    IHIL_Tclk(1);
}

void SetPc(uint16_t pc)
{
    cntrl_sig_16bit();
    SetReg_16Bits(0x3401);
    data_16bit();
    SetReg_16Bits(0x4030);
    IHIL_TCLK();
    SetReg_16Bits(pc);
    IHIL_TCLK();
    addr_capture();
    IHIL_Tclk(0);
    cntrl_sig_16bit();
    SetReg_16Bits(0x2401);
    IHIL_Tclk(1);
}

void SetPcJtagBug(uint16_t pc)
{
    data_16bit();
    SetReg_16Bits(0x4030);
    IHIL_Tclk(1);
    IHIL_Tclk(0);
    SetReg_16Bits(pc);
    IHIL_Tclk(1);
    IHIL_Tclk(0);
    IHIL_Tclk(1);
    IHIL_Tclk(0);
}

void SetPcX(uint32_t pc)
{
    uint16_t pc_high = (uint16_t)(0x80 | (((pc)>>8) & 0x0F00));
    uint16_t pc_low  = (uint16_t)((pc) & 0xFFFF);
    cntrl_sig_high_byte();
    SetReg_8Bits(0x34);
    data_16bit();
    SetReg_16Bits(pc_high);
    IHIL_TCLK();
    SetReg_16Bits(pc_low);
    IHIL_TCLK();
    addr_capture();
    IHIL_Tclk(0);
    cntrl_sig_high_byte();
    SetReg_8Bits(0x24);
    IHIL_Tclk(1);
}

void SetPcXv2(uint16_t Mova, uint16_t pc)
{
    cntrl_sig_capture();
    SetReg_16Bits(0x0000);
    IHIL_Tclk(0);
    data_16bit();
    IHIL_Tclk(1);
    SetReg_16Bits(Mova);
    IHIL_Tclk(0);
    cntrl_sig_16bit();
    SetReg_16Bits(0x1400);
    data_16bit();
    IHIL_Tclk(0);
    IHIL_Tclk(1);
    SetReg_16Bits(pc);
    IHIL_Tclk(0);
    IHIL_Tclk(1);
    SetReg_16Bits(0x4303);
    IHIL_Tclk(0);
    addr_capture();
    SetReg_20Bits(0x00000);
}

uint16_t SyncJtag()
{
    uint16_t lOut = 0, i = 50;
    cntrl_sig_16bit();
    SetReg_16Bits(0x2401);
    cntrl_sig_capture();
    do
    {
        lOut = SetReg_16Bits(0x0000);
        i--;
    }
    while(((lOut == 0xFFFF) || !(lOut & 0x0200)) && i);
    if(!i)
    {
        return 0;
    }
    return lOut;
}

void SyncJtagX()
{
    uint16_t lOut = 0, i = 50;
    cntrl_sig_16bit();
    SetReg_16Bits(0x2401);
    cntrl_sig_capture();
    SetReg_16Bits(0x0000);
    IHIL_Tclk(1);
    if (!(lOut & 0x0200))
    {
        cntrl_sig_high_byte();
        SetReg_8Bits(0x34);
        eem_data_exchange32();
        SetReg_32Bits(0x89);
        SetReg_32Bits(0);
        eem_data_exchange32();
        SetReg_32Bits(0x88);
        SetReg_32Bits(lOut|0x40);
        eem_data_exchange32();
        SetReg_32Bits(0x88);
        SetReg_32Bits(lOut);
    }
    cntrl_sig_capture();
    do
    {
        lOut = SetReg_16Bits(0x0000);
        i--;
    }
    while(((lOut == 0xFFFF) || !(lOut & 0x0200)) && i);
}

void SyncJtagXv2()
{
    uint16_t i = 50, lOut = 0 ;
    cntrl_sig_16bit();
    SetReg_16Bits(0x1501);
    cntrl_sig_capture();
    do
    {
        lOut = SetReg_16Bits(0x0000);
        i--;
    }
    while(((lOut == 0xFFFF) || !(lOut & 0x0200)) && i);
}

uint16_t wait_for_synch()
{
    uint16_t i = 0;
    cntrl_sig_capture();

    while(!(SetReg_16Bits(0) & 0x0200) && i++ < 50);
    if(i >= 50)
    {
        return 0;
    }
    return 1;
}

void RestoreTestRegs()
{
    if(devicePowerSettings.powerTestReg3VMask)
    {
        test_reg_3V();
        SetReg_16Bits(devicePowerSettings.powerTestReg3VDefault);
        IHIL_Delay_1ms(20);
    }
    if(devicePowerSettings.powerTestRegMask)
    {
        test_reg();
        SetReg_32Bits(devicePowerSettings.powerTestRegDefault);
        IHIL_Delay_1ms(20);
    }
}

void EnableLpmx5()
{
    if(devicePowerSettings.powerTestReg3VMask)
    {
        uint16_t reg_3V = 0;
        test_reg_3V();
        reg_3V = SetReg_16Bits(devicePowerSettings.powerTestReg3VDefault);

        SetReg_16Bits((reg_3V & ~devicePowerSettings.powerTestReg3VMask)|
                       devicePowerSettings.enableLpmx5TestReg3V);

        IHIL_Delay_1ms(20);
    }

    if(devicePowerSettings.powerTestRegMask)
    {
        uint32_t reg_test = 0;
        test_reg();
        reg_test = SetReg_32Bits(devicePowerSettings.powerTestRegDefault);

        SetReg_32Bits((reg_test & ~devicePowerSettings.powerTestRegMask)|
        devicePowerSettings.enableLpmx5TestReg);

        IHIL_Delay_1ms(20);
    }
}

void DisableLpmx5()
{
    if(devicePowerSettings.powerTestReg3VMask)
    {
        uint16_t reg_3V = 0;
        test_reg_3V();
        reg_3V = SetReg_16Bits(devicePowerSettings.powerTestReg3VDefault);

        SetReg_16Bits((reg_3V & ~devicePowerSettings.powerTestReg3VMask)|
            devicePowerSettings.disableLpmx5TestReg3V);
        IHIL_Delay_1ms(20);
    }

    if(devicePowerSettings.powerTestRegMask)
    {
        uint32_t reg_test = 0;
        test_reg();
        SetReg_32Bits(devicePowerSettings.powerTestRegDefault);

        SetReg_32Bits((reg_test & ~devicePowerSettings.powerTestRegMask)|
            devicePowerSettings.disableLpmx5TestReg);

        IHIL_Delay_1ms(20);
    }
}



// -----------------------------------------------------------------------------
void JTAG_PsaSetup(uint32_t StartAddr)
{
    data_16bit();
    IHIL_Tclk(1);
    SetReg_16Bits(MOV_IMM_PC);
    IHIL_Tclk(0);
    IHIL_Tclk(1);
    SetReg_16Bits(StartAddr - 2);
    IHIL_Tclk(0);
    IHIL_Tclk(1);
    IHIL_Tclk(0);
    IHIL_Tclk(1);
    IHIL_Tclk(0);
    addr_capture();
    SetReg_16Bits(0x0000);
}

// -----------------------------------------------------------------------------
void JTAG_EnhancedPsaSetup(uint32_t StartAddr)
{
    halt_cpu();
    IHIL_Tclk(0);
    data_16bit();
    SetReg_16Bits(StartAddr - 2);
}

// -----------------------------------------------------------------------------
void JTAG_PsaEnd()
{
    // Intentionally does nothing
}

// -----------------------------------------------------------------------------
void JTAG_EnhancedPsaEnd()
{
    release_cpu();
    isInstrLoad();
}

void _hil_Connect(uint8_t state) {
    if (jtagReleased) {
        IHIL_SetProtocol(gprotocol_id);
    }
    
    if (state == RSTHIGH) {
        if (gprotocol_id == JTAG) {
            HIL_UNIMP;
        
        } else {
            IHIL_Delay_1ms(10);
            qDriveSbw();
            IHIL_Delay_1ms(30);
            _hil_EntrySequences_RstHigh_SBW();
        }
    
    } else if (state == RSTLOW) {
        if (gprotocol_id == JTAG) {
            HIL_UNIMP;
        
        } else {
            qDriveSbw();
            IHIL_Delay_1ms(1);
            _hil_EntrySequences_RstLow_SBW();
        }
    }
    jtagReleased = 0;
}

void _hil_Release() {
    jtagReleased = 1;
}

void qDriveSbw(void) {
    // TEST=0, RST=1
    _msp.sbwTestSet(0);
    _msp.sbwRstSet(1);
}

void qDriveJTAG(void) {
    HIL_UNIMP;
}

void _hil_EntrySequences_RstHigh_SBW() {
    // TEST=0, RST=0
    _msp.sbwTestSet(0);
    _msp.sbwRstSet(0);
    IHIL_Delay_1ms(5);
    
    // RST=1
    _msp.sbwRstSet(1);
    IHIL_Delay_1ms(5);
    
    // TEST=1
    _msp.sbwTestSet(1);
    IHIL_Delay_1ms(100);
    
    // TEST pulse 1->0->1
    // (Use a SBW IO cycle to pulse TEST)
    _msp.sbwTestPulse();
    IHIL_Delay_1ms(5);
}

void _hil_EntrySequences_RstLow_SBW() {
    // TEST=0, RST=0
    _msp.sbwTestSet(0);
    _msp.sbwRstSet(0);
    IHIL_Delay_1ms(5);
    
    // TEST=1
    _msp.sbwTestSet(1);
    IHIL_Delay_1ms(100);
    
    // RST=1
    _msp.sbwRstSet(1);
    IHIL_Delay_1ms(5);
    
    // TEST pulse 1->0->1
    // (Use a SBW IO cycle to pulse TEST)
    _msp.sbwTestPulse();
    IHIL_Delay_1ms(5);
}

void initGeneric() {
    prevInstruction = 0;
}
