#pragma once
#include "HIL.h"

typedef void (*HilInitGetEdtCommenFunc)(edt_common_methods_t* edt_commen);
typedef void (*HilInitGetEdtDistinctFunc)(edt_distinct_methods_t* edt_distinct);

//extern edt_common_methods_t  _edt_Common_Methods_HAL;
//extern edt_distinct_methods_t _edt_Distinct_Methods_HAL;
//
//extern uint16_t altRomAddressForCpuRead;
//extern uint16_t wdtctlAddress5xx;
//extern DevicePowerSettings devicePowerSettings;

// Hil common methods
//#pragma inline=forced
int16_t IHIL_Init(void)   { return CALL_MEMBER_FN_PTR(_edt_Common_Methods_HAL.Init)();}

//#pragma inline=forced
int16_t IHIL_SetVcc(uint16_t vcc){ return CALL_MEMBER_FN_PTR(_edt_Common_Methods_HAL.SetVcc)(vcc); }

//#pragma inline=forced
void IHIL_SwitchVccFET(uint16_t switchVccFET){ CALL_MEMBER_FN_PTR(_edt_Common_Methods_HAL.SwitchVccFET)(switchVccFET);}

//#pragma inline=forced
int16_t IHIL_GetVcc(double* Vcc, double* ExtVcc){ return CALL_MEMBER_FN_PTR(_edt_Common_Methods_HAL.GetVcc)(Vcc, ExtVcc);}

//#pragma inline=forced
int16_t IHIL_SetProtocol(uint16_t protocol_id){ return CALL_MEMBER_FN_PTR(_edt_Common_Methods_HAL.SetProtocol)(protocol_id);}

//#pragma inline=forced
void IHIL_SetPsaTCLK(uint16_t tclkValue){CALL_MEMBER_FN_PTR(_edt_Common_Methods_HAL.SetPsaTCLK)(tclkValue);}

//#pragma inline=forced
int16_t IHIL_Open(uint8_t state){ return CALL_MEMBER_FN_PTR(_edt_Common_Methods_HAL.Open)(state);}

//#pragma inline=forced
int16_t IHIL_Close(void){return CALL_MEMBER_FN_PTR(_edt_Common_Methods_HAL.Close)();}

//#pragma inline=forced
void IHIL_Delay_1us(uint16_t usecs){CALL_MEMBER_FN_PTR(_edt_Common_Methods_HAL.Delay_1us)(usecs);}

//#pragma inline=forced
void IHIL_Delay_1ms(uint16_t msecs){CALL_MEMBER_FN_PTR(_edt_Common_Methods_HAL.Delay_1ms)(msecs);}

//#pragma inline=forced
int16_t IHIL_IccMonitor_Process(uint16_t flags){ return CALL_MEMBER_FN_PTR(_edt_Common_Methods_HAL.Loop)(flags);}

//#pragma inline=forced
void IHIL_EntrySequences(uint8_t states){CALL_MEMBER_FN_PTR(_edt_Common_Methods_HAL.EntrySequences)(states);}

//#pragma inline=forced
void IHIL_BSL_EntrySequence(uint16_t switchBypassOff){CALL_MEMBER_FN_PTR(_edt_Common_Methods_HAL.BSL_EntrySequence)(switchBypassOff);}

//#pragma inline=forced
void IHIL_BSL_EntrySequence1xx_4xx(){ CALL_MEMBER_FN_PTR(_edt_Common_Methods_HAL.BSL_EntrySequence1xx_4xx)();}

//#pragma inline=forced
void IHIL_SetReset(uint8_t value){CALL_MEMBER_FN_PTR(_edt_Common_Methods_HAL.SetReset)(value);}

//#pragma inline=forced
void IHIL_SetTest(uint8_t value){CALL_MEMBER_FN_PTR(_edt_Common_Methods_HAL.SetTest)(value);}

//#pragma inline=forced
void IHIL_SetTMS(uint8_t value){CALL_MEMBER_FN_PTR(_edt_Common_Methods_HAL.SetTMS)(value);}

//#pragma inline=forced
void IHIL_SetTCK(uint8_t value){CALL_MEMBER_FN_PTR(_edt_Common_Methods_HAL.SetTCK)(value);}

//#pragma inline=forced
void IHIL_SetTDI(uint8_t value){CALL_MEMBER_FN_PTR(_edt_Common_Methods_HAL.SetTDI)(value);}

//#pragma inline=forced
void IHIL_InitDelayTimer(void){CALL_MEMBER_FN_PTR(_edt_Common_Methods_HAL.initDelayTimer)();}

//#pragma inline=forced
void IHIL_SetJtagSpeed(uint16_t jtagSpeed, uint16_t sbwSpeed){CALL_MEMBER_FN_PTR(_edt_Common_Methods_HAL.SetJtagSpeed)(jtagSpeed,sbwSpeed);}

//#pragma inline=forced
void IHIL_ConfigureSetPc(uint16_t PCclockBeforeCapture){CALL_MEMBER_FN_PTR(_edt_Common_Methods_HAL.ConfigureSetPc)(PCclockBeforeCapture);}
// Hil distinct methods
//#pragma inline=forced
int16_t IHIL_TapReset(void){return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.TapReset)();}

//#pragma inline=forced
int16_t IHIL_CheckJtagFuse(void){return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.CheckJtagFuse)();}

//#pragma inline=forced
void IHIL_Tclk(uint8_t tclk){CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.Tclk)(tclk);}

//#pragma inline=forced
SBWShiftProxy<uint8_t> IHIL_Instr4(uint8_t ir){return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.Instr04)(ir);}

//#pragma inline=forced
void IHIL_StepPsa(uint32_t length){CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.StepPsa)(length);}

//#pragma inline=forced
int16_t IHIL_BlowFuse(uint8_t targetHasTestVpp){return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.BlowFuse)(targetHasTestVpp);}

//#pragma inline=forced
uint8_t IHIL_GetPrevInstruction(){ return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.GetPrevInstruction)();}

//#pragma inline=forced
void IHIL_TCLK(void){IHIL_Tclk(0); IHIL_Tclk(1);}

//#pragma inline=forced
uint64_t SetReg_XBits(uint64_t *Data, uint16_t count){return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.SetReg_XBits)(Data,count);}

//#pragma inline=forced
uint64_t SetReg_35Bits(uint64_t *Data){return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.SetReg_XBits35)(Data);}

//#pragma inline=forced
SBWShiftProxy<uint8_t> SetReg_8Bits(uint8_t data){return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.SetReg_XBits08)(data);}

//#pragma inline=forced
SBWShiftProxy<uint16_t> SetReg_16Bits(uint16_t data){return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.SetReg_XBits16)(data);}

//#pragma inline=forced
SBWShiftProxy<uint32_t,20> SetReg_20Bits(uint32_t data) {return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.SetReg_XBits20)(data);}

//#pragma inline=forced
SBWShiftProxy<uint32_t> SetReg_32Bits(uint32_t data){return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.SetReg_XBits32)(data);}

//#pragma inline=forced
SBWShiftProxy<uint64_t> SetReg_64Bits(uint64_t data){return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.SetReg_XBits64)(data);}

//ARM interface functions
//#pragma inline=forced
int16_t IHIL_Write_Read_Dp(uint8_t address, uint32_t *data, uint16_t rnw)
    {return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.write_read_Dp)(address,data, rnw);}
//#pragma inline=forced
int16_t IHIL_Write_Read_Ap(uint32_t address, uint32_t *data, uint16_t rnw)
    {return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.write_read_Ap)(address, data, rnw);}
//#pragma inline=forced
int16_t IHIL_Write_Read_Mem_Ap(uint16_t ap_sel, uint32_t address, uint32_t *data, uint16_t rnw)
{ return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.write_read_mem_Ap)(ap_sel, address, data, rnw);}
//#pragma inline=forced
uint8_t IHIL_SwdTransferData(uint8_t regiser, uint32_t* data, uint8_t rnw)
{ return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.SwdTransferData)(regiser, data, rnw);}

//#pragma inline=forced
uint64_t SetReg8_64Bits(uint64_t data, uint16_t loopCount, uint16_t PG)
    {return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.SetReg_XBits8_64)(data, loopCount, PG);}

// JTAG instruction register access
SBWShiftProxy<uint8_t> cntrl_sig_low_byte()        { return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.Instr)(IR_CNTRL_SIG_LOW_BYTE);    }
//SBWShiftProxy<uint8_t> cntrl_sig_capture()         { return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.Instr)(IR_CNTRL_SIG_CAPTURE);     }

SBWShiftProxy<uint8_t> cntrl_sig_capture()         {
    const uint8_t jtagID = CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.Instr)(IR_CNTRL_SIG_CAPTURE);
    printf("JTAG ID: %x\n", jtagID);
    return jtagID;
}

SBWShiftProxy<uint8_t> cntrl_sig_high_byte()       { return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.Instr)(IR_CNTRL_SIG_HIGH_BYTE);   }
SBWShiftProxy<uint8_t> cntrl_sig_16bit()           { return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.Instr)(IR_CNTRL_SIG_16BIT);       }
SBWShiftProxy<uint8_t> cntrl_sig_release()         { return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.Instr)(IR_CNTRL_SIG_RELEASE);     }
SBWShiftProxy<uint8_t> addr_16bit()                { return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.Instr)(IR_ADDR_16BIT);            }
SBWShiftProxy<uint8_t> addr_capture()              { return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.Instr)(IR_ADDR_CAPTURE);          }
SBWShiftProxy<uint8_t> data_16bit()                { return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.Instr)(IR_DATA_16BIT);            }
SBWShiftProxy<uint8_t> data_capture()              { return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.Instr)(IR_DATA_CAPTURE);          }
SBWShiftProxy<uint8_t> data_to_addr()              { return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.Instr)(IR_DATA_TO_ADDR);          }
SBWShiftProxy<uint8_t> data_quick()                { return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.Instr)(IR_DATA_QUICK);            }
SBWShiftProxy<uint8_t> config_fuses()              { return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.Instr)(IR_CONFIG_FUSES);          }
SBWShiftProxy<uint8_t> eem_data_exchange()         { return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.Instr)(IR_EMEX_DATA_EXCHANGE);    }
SBWShiftProxy<uint8_t> eem_data_exchange32()       { return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.Instr)(IR_EMEX_DATA_EXCHANGE32);  }
SBWShiftProxy<uint8_t> eem_read_control()          { return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.Instr)(IR_EMEX_READ_CONTROL);     }
SBWShiftProxy<uint8_t> eem_write_control()         { return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.Instr)(IR_EMEX_WRITE_CONTROL);    }
SBWShiftProxy<uint8_t> eem_read_trigger()          { return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.Instr)(IR_EMEX_READ_TRIGGER);     }
SBWShiftProxy<uint8_t> data_psa()                  { return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.Instr)(IR_DATA_PSA);              }
SBWShiftProxy<uint8_t> shift_out_psa()             { return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.Instr)(IR_SHIFT_OUT_PSA);         }
SBWShiftProxy<uint8_t> flash_16bit_update()        { return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.Instr)(IR_FLASH_16BIT_UPDATE);    }
SBWShiftProxy<uint8_t> jmb_exchange()              { return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.Instr)(IR_JMB_EXCHANGE);          }
SBWShiftProxy<uint8_t> device_ip_pointer()         { return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.Instr)(IR_DEVICE_ID);             }
SBWShiftProxy<uint8_t> core_ip_pointer()           { return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.Instr)(IR_COREIP_ID);             }
SBWShiftProxy<uint8_t> jstate_read()               { return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.Instr)(IR_JSTATE_ID);             }
SBWShiftProxy<uint8_t> test_reg()                  { return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.Instr)(IR_TEST_REG);              }
SBWShiftProxy<uint8_t> test_reg_3V()               { return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.Instr)(IR_TEST_3V_REG);           }
SBWShiftProxy<uint8_t> prepare_blow()              { return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.Instr)(IR_PREPARE_BLOW);          }
SBWShiftProxy<uint8_t> ex_blow()                   { return CALL_MEMBER_FN_PTR(_edt_Distinct_Methods_HAL.Instr)(IR_EX_BLOW);               }

#define OUT1RDY 0x0008
#define OUT0RDY 0x0004
#define IN1RDY  0x0002
#define IN0RDY  0x0001

#define JMB32B  0x0010
#define OUTREQ  0x0004
#define INREQ   0x0001

// JTAG logic functions
//#pragma inline=forced
int16_t isInstrLoad()
{
    cntrl_sig_capture();
    if((SetReg_16Bits(0) & (CNTRL_SIG_INSTRLOAD | CNTRL_SIG_READ)) != (CNTRL_SIG_INSTRLOAD | CNTRL_SIG_READ))
    {
        return -1;
    }
    return 0;
}

//#pragma inline=forced
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

//#pragma inline=forced
void halt_cpu()
{
    data_16bit();
    SetReg_16Bits(0x3FFF);
    IHIL_Tclk(0);
    cntrl_sig_16bit();
    SetReg_16Bits(0x2409);
    IHIL_Tclk(1);
}

//#pragma inline=forced
void release_cpu()
{
    IHIL_Tclk(0);
    cntrl_sig_16bit();
    SetReg_16Bits(0x2401);
    addr_capture();
    IHIL_Tclk(1);
}

//#pragma inline=forced
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

//#pragma inline=forced
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

//#pragma inline=forced
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

//#pragma inline=forced
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

//#pragma inline=forced
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

//#pragma inline=forced
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

//#pragma inline=forced
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

//#pragma inline=forced
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

//#pragma inline=forced
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

//#pragma inline=forced
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

//#pragma inline=forced
uint32_t ReadCpuRegXv2(uint16_t reg)
{
    uint16_t Rx_l = 0;
    uint16_t Rx_h = 0;
    uint16_t jtagId = cntrl_sig_capture();
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

//#pragma inline=forced
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

//#pragma inline=forced
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
//#pragma inline=forced
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

//#pragma inline=forced
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

//#pragma inline=forced
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

//#pragma inline=forced
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

//#pragma inline=forced
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

//#pragma inline=forced
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

//#pragma inline=forced
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

//#pragma inline=forced
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

//#pragma inline=forced
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

//#pragma inline=forced
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

//#pragma inline=forced
void EnableLpmx5()
{
    if(devicePowerSettings.powerTestReg3VMask)
    {
        uint16_t reg_3V = 0;
        test_reg_3V();
        reg_3V= SetReg_16Bits(devicePowerSettings.powerTestReg3VDefault);

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

//#pragma inline=forced
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

//uint32_t i_ReadJmbOut();
//uint16_t i_ReadJmbOut16();
//
//int16_t i_WriteJmbIn(uint16_t data);
//int16_t i_WriteJmbIn32(uint16_t dataA, uint16_t dataB);
//
//int16_t checkWakeup();
//int16_t powerUpArm();


//#pragma inline=forced
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

//#pragma inline=forced
// -----------------------------------------------------------------------------
void JTAG_EnhancedPsaSetup(uint32_t StartAddr)
{
    halt_cpu();
    IHIL_Tclk(0);
    data_16bit();
    SetReg_16Bits(StartAddr - 2);
}

//#pragma inline=forced
// -----------------------------------------------------------------------------
void JTAG_PsaEnd(void)
{
    // Intentionally does nothing
}

//#pragma inline=forced
// -----------------------------------------------------------------------------
void JTAG_EnhancedPsaEnd(void)
{
    release_cpu();
    isInstrLoad();
}



uint32_t i_ReadJmbOut()
{
    uint16_t sJMBOUT0 = 0, sJMBOUT1 = 0, sJMBINCTL = 0;
    jmb_exchange();
    if(SetReg_16Bits(sJMBINCTL) & OUT1RDY)
    {
        sJMBINCTL |= JMB32B + OUTREQ;
        SetReg_16Bits(sJMBINCTL);
        sJMBOUT0 = SetReg_16Bits(0);
        sJMBOUT1 = SetReg_16Bits(0);
        return ((uint32_t)sJMBOUT1<<16) + sJMBOUT0;
    }
    return 0;
}

uint16_t i_ReadJmbOut16()
{
    uint16_t sJMBINCTL = 0;
    jmb_exchange();
    if(SetReg_16Bits(sJMBINCTL) & OUT0RDY)
    {
        sJMBINCTL |=  OUTREQ;
        SetReg_16Bits(sJMBINCTL);
        return SetReg_16Bits(0);
    }
    return 0;
}

int16_t i_WriteJmbIn(uint16_t data)
{
    volatile uint16_t sJMBINCTL = INREQ, sJMBIN0 = (uint16_t)(data & 0x0000FFFF);
    volatile uint16_t lOut = 0;
    volatile uint32_t Timeout = 0;

    jmb_exchange();
    do
    {
        lOut = SetReg_16Bits(0x0000);
        if(Timeout++ >= 3000)
        {
            return 1;
        }
    }
    while(!(lOut & IN0RDY) && Timeout < 3000);

    if(Timeout < 3000)
    {
        SetReg_16Bits(sJMBINCTL);
        SetReg_16Bits(sJMBIN0);
    }
    return 0;
}

int16_t i_WriteJmbIn32(uint16_t dataA, uint16_t dataB)
{
    volatile uint16_t sJMBINCTL =  JMB32B | INREQ;
    volatile uint16_t sJMBIN0 = 0 ,sJMBIN1 = 0, lOut = 0;
    volatile uint32_t Timeout = 0;

    sJMBIN0 = (uint16_t)(dataA & 0x0000FFFF);
    sJMBIN1 = (uint16_t)(dataB & 0x0000FFFF);

    jmb_exchange();
    do
    {
        lOut = SetReg_16Bits(0x0000);
        if(Timeout++ >= 3000)
        {
            return 1;
        }
    }
    while(!(lOut & IN1RDY) && Timeout < 3000);
    if(Timeout < 3000)
    {
        sJMBINCTL = 0x11;
        SetReg_16Bits(sJMBINCTL);
        SetReg_16Bits(sJMBIN0);
        SetReg_16Bits(sJMBIN1);
    }
    return 0;
}

int16_t checkWakeup()
{
    uint32_t UNLOCK_ADDRESS = 0xE0044000;
    uint32_t SYS_SYSTEM_STAT = 0xE0044020;
    uint32_t PWD = 0x695A;
    uint16_t DBG_SEC_ACT_MASK = 0x8;

    uint32_t val = 0;
    uint16_t  timeout = 100;

    //unlock Sys master
    IHIL_Write_Read_Mem_Ap(0, UNLOCK_ADDRESS, &PWD, WRITE);

    do
    {
        IHIL_Write_Read_Mem_Ap(0, SYS_SYSTEM_STAT, &val, READ);
        timeout--;
    }while((val & DBG_SEC_ACT_MASK) && --timeout);

    if(!timeout)
    {
        return -1;
    }
    return 0;
}

int16_t powerUpArm()
{
    uint32_t val = 0;
    uint16_t  timeout = 200;

    IHIL_Write_Read_Dp(DP_CTRL_STAT, &val, READ);
    // Request system and debug power up
    val |= DP_CTRL_STAT_CSYSPWRUPREQ | DP_CTRL_STAT_CDBGPWRUPREQ;
    IHIL_Write_Read_Dp(DP_CTRL_STAT, &val, WRITE);
    // Wait for acknowledge
    do
    {
        IHIL_Write_Read_Dp(DP_CTRL_STAT, &val, READ);
        timeout--;
    } while(((val & (DP_CTRL_STAT_CSYSPWRUPACK | DP_CTRL_STAT_CDBGPWRUPACK)) !=
                   (DP_CTRL_STAT_CSYSPWRUPACK | DP_CTRL_STAT_CDBGPWRUPACK)) && timeout);

    if(!timeout)
    {
        return -1;
    }
    return 0;
}
