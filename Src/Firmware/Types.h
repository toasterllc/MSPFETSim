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

#include "Bios/include/modules.h"
#include "Bios/include/protocol.h"
#include "Bios/include/error_def.h"
#include "Bios/include/ConfigureParameters.h"
#include "Bios/src/fw/fet/FetVersion.h"
#include "Bios/src/fw/fet/communicationDefs.h"
#include "Bios/src/hal/JTAG_defs.h"
#include "Bios/src/hal/EEM_defs.h"

using CHAR      = int8_t;
using UCHAR     = uint8_t;
using INT       = int16_t;
using UINT      = uint16_t;
using SHORT     = int16_t;
using USHORT    = uint16_t;
using LONG      = int32_t;
using ULONG     = uint32_t;
using VOID      = void;
using HANDLE    = uint32_t;
using PSTR      = int8_t*;
using BOOL      = int16_t;
using DOUBLE    = double;
using BYTE      = uint8_t;
using PBYTE     = uint8_t*;
using WORD      = uint16_t;
using DWORD     = uint32_t;
using PDWORD    = uint32_t*;

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
    int16_t (MSPFETSim::*out_init)(uint8_t, uint8_t);
    int16_t (MSPFETSim::*flush)(void);
    int16_t (MSPFETSim::*put_byte)(uint8_t);
    int16_t (MSPFETSim::*put_bytes)(void*, uint16_t);
    int16_t (MSPFETSim::*put_word)(uint16_t);
    int16_t (MSPFETSim::*put_long)(uint32_t);
    int16_t (MSPFETSim::*in_init)(void *, uint8_t);
    int16_t (MSPFETSim::*internal_stream)(uint8_t *, uint16_t, uint8_t *, uint16_t, StreamSafe *);
    int16_t (MSPFETSim::*external_stream)(StreamSafe *);
    uint8_t (MSPFETSim::*change_type)(uint8_t);
    int16_t (MSPFETSim::*get_byte)(uint8_t *);
    int16_t (MSPFETSim::*get_word)(uint16_t *);
    int16_t (MSPFETSim::*get_long)(uint32_t *);
    int16_t (MSPFETSim::*get_buffer)(void **, uint16_t *);
    int16_t (MSPFETSim::*discard_bytes)(uint16_t);
    int16_t (MSPFETSim::*memcpy)(uint8_t*, uint8_t*, uint16_t);
    int16_t (MSPFETSim::*biosLedOn)(uint8_t);
    int16_t (MSPFETSim::*biosLedOff)(uint8_t);
    int16_t (MSPFETSim::*biosLedBlink)(uint8_t, uint16_t);
    int16_t (MSPFETSim::*biosLedFlash)(uint8_t, uint16_t);
    int16_t (MSPFETSim::*biosLedAlternate)(uint16_t);
    int16_t (MSPFETSim::*sendDebug)(int8_t *, uint16_t size);
    int16_t (MSPFETSim::*getSharedVariable)(uint16_t IdMemoryType, uint16_t** Address);
    int16_t (MSPFETSim::*deleteSharedVariable)(uint16_t IdMemoryType);
};

#define UNIMP_FN()          printf("### [%s] UNIMPLEMENTED\n", __FUNCTION__)
#define BAD_PROTO(proto)    printf("### [%s] UNIMPLEMENTED PROTOCOL: %u\n", __FUNCTION__, (proto))

#define HAL_FUNCTION(x) int16_t x (uint16_t flags)
using HalFuncInOut = int16_t (MSPFETSim::*)(uint16_t);

#ifndef HAL_REC
#define HAL_REC
struct _HalRec_
{
  uint16_t id;
  HalFuncInOut function;
};
typedef struct _HalRec_ HalRec;
#endif

typedef void (MSPFETSim::*HilInitFunc)();
typedef void *(MSPFETSim::*HalMainFunc)(struct MSPFETSim::stream_funcs* stream_adr, uint32_t, uint8_t v3opHilCrcOk, uint8_t v3opDcdcCrcOk);

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




#pragma push_macro("static")
#undef static
template <class...> static constexpr std::false_type _AlwaysFalse = {};
#pragma pop_macro("static")

template <typename T, size_t W=sizeof(T)*8>
class SBWShiftProxy {
public:
    // Default constructor returns a nop object that just returns the argument
    // from the conversion operator
    SBWShiftProxy(uint64_t data=0) : _data(data) {}
    SBWShiftProxy(MSPFETSim* self, uint64_t data) : _self(self), _data(data) {}
    // Copy constructor: not allowed
    SBWShiftProxy(const SBWShiftProxy& x) = delete;
    SBWShiftProxy& operator=(const SBWShiftProxy& x) = delete;
    // Move constructor: not allowed
    SBWShiftProxy(SBWShiftProxy&& x) = delete;
    SBWShiftProxy& operator=(SBWShiftProxy&& x) = delete;
    
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
        if (!_self) return _data; // Short-circuit if we're a nop object
        
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
    MSPFETSim* _self = nullptr;
    uint64_t _data = 0;
    bool _read = false;
};






struct edt_common_methods
{
    int16_t (MSPFETSim::*Init)(void);
    int16_t (MSPFETSim::*SetVcc)(uint16_t);
    void  (MSPFETSim::*SwitchVccFET)(uint16_t);
    int16_t (MSPFETSim::*GetVcc)(double*, double*);
    int16_t (MSPFETSim::*SetProtocol)(uint16_t);
    void  (MSPFETSim::*SetPsaTCLK)(uint16_t);
    int16_t (MSPFETSim::*Open)(uint8_t state);
    int16_t (MSPFETSim::*Close)(void);
    void  (MSPFETSim::*Delay_1us)(uint16_t);
    void  (MSPFETSim::*Delay_1ms)(uint16_t);
    int16_t (MSPFETSim::*Loop)(uint16_t);
    void  (MSPFETSim::*EntrySequences)(uint8_t);
    void (MSPFETSim::*SetReset)(uint8_t);      // Set the Reset pin to the specified value
    void (MSPFETSim::*SetTest)(uint8_t);       // Set the Test pin to the specified value
    void (MSPFETSim::*SetJtagSpeed)(uint16_t, uint16_t);
    void (MSPFETSim::*ConfigureSetPc)(uint16_t);
    void (MSPFETSim::*initDelayTimer)(void);
    void (MSPFETSim::*BSL_EntrySequence)(uint16_t switchBypassOff);
    void (MSPFETSim::*SetTMS)(uint8_t);      // Set the TMS pin to the specified value
    void (MSPFETSim::*SetTCK)(uint8_t);      // Set the TCK pin to the specified value
    void (MSPFETSim::*SetTDI)(uint8_t);      // Set the TDI pin to the specified value
    int16_t (MSPFETSim::*regulateVcc)(void);
    void (MSPFETSim::*setFpgaTimeOut)(uint16_t state);
    uint16_t (MSPFETSim::*getFpgaVersion)(void);
    void (MSPFETSim::*ReadADC12)(void);
    void (MSPFETSim::*ConfigFpgaIoMode)(uint16_t mode);
    void (MSPFETSim::*BSL_EntrySequence1xx_4xx)(void);
    void (MSPFETSim::*SetToolID)(uint16_t id);
};
typedef struct edt_common_methods edt_common_methods_t;

struct edt_distinct_methods
{
    int16_t                     (MSPFETSim::*TapReset)(void);
    int16_t                     (MSPFETSim::*CheckJtagFuse)(void);
    SBWShiftProxy<uint8_t>      (MSPFETSim::*Instr)(uint8_t);
    SBWShiftProxy<uint8_t>      (MSPFETSim::*Instr04)(uint8_t);
    SBWShiftProxy<uint8_t>      (MSPFETSim::*SetReg_XBits08)(uint8_t);
    SBWShiftProxy<uint16_t>     (MSPFETSim::*SetReg_XBits16)(uint16_t);
    SBWShiftProxy<uint32_t,20>  (MSPFETSim::*SetReg_XBits20)(uint32_t);
    SBWShiftProxy<uint32_t>     (MSPFETSim::*SetReg_XBits32)(uint32_t);
    uint64_t                    (MSPFETSim::*SetReg_XBits35)(uint64_t *Data);
    SBWShiftProxy<uint64_t>     (MSPFETSim::*SetReg_XBits64)(uint64_t);
    uint64_t                    (MSPFETSim::*SetReg_XBits8_64)(uint64_t, uint16_t, uint16_t);
    uint64_t                    (MSPFETSim::*SetReg_XBits)(uint64_t *Data, uint16_t count);
    void                        (MSPFETSim::*Tclk)(uint8_t);
    void                        (MSPFETSim::*StepPsa)(uint32_t);
    int16_t                     (MSPFETSim::*BlowFuse)(uint8_t); // Blow the JTAG acces fuse
    uint8_t                     (MSPFETSim::*GetPrevInstruction)(void);
    int16_t                     (MSPFETSim::*write_read_Dp)(uint8_t address, uint32_t *data, uint16_t rnw);
    int16_t                     (MSPFETSim::*write_read_Ap)(uint32_t address, uint32_t *data, uint16_t rnw);
    int16_t                     (MSPFETSim::*write_read_mem_Ap)(uint16_t ap_sel, uint32_t address, uint32_t *data, uint16_t rnw);
    uint32_t                    (MSPFETSim::*GetJtagIdCode)();
    uint8_t                     (MSPFETSim::*SwdTransferData)(uint8_t regiser, uint32_t* data, uint8_t rnw);
};
typedef struct edt_distinct_methods edt_distinct_methods_t;






struct DCDC_INFOS
{
    int16_t (MSPFETSim::*getSubMcuVersion)(void);
    int16_t (MSPFETSim::*getLayerVersion)(void);
    int16_t (MSPFETSim::*dcdcCalibrate)(uint16_t resistor[4], uint16_t resCount, uint16_t vcc);
    int16_t (MSPFETSim::*dcdcPowerDown)(void);
    int16_t (MSPFETSim::*dcdcSetVcc)(uint16_t vcc);
    int16_t (MSPFETSim::*dcdcRestart)(uint16_t fetType_);
    void    (MSPFETSim::*dcdc_getCalibrationValues)(uint16_t vcc, uint16_t resistor,  uint16_t resCount, uint32_t *ticks, uint32_t *time);
    int16_t (MSPFETSim::*getLayerVersionCmp)(void);
};
typedef struct DCDC_INFOS DCDC_INFOS_t;

typedef struct FET_USB_INFOS
{
    BYTE(MSPFETSim::*FetUSB_bytesInUSBBuffer)(BYTE intfNum);
    BYTE(MSPFETSim::*FetUSB_receiveData)(BYTE* data, WORD size, BYTE intfNum);
    BYTE(MSPFETSim::*FetUsb_CdcSendDataInBackground)(BYTE* dataBuf,WORD size,BYTE intfNum,ULONG ulTimeout);
}FET_USB_INFOS_t;

struct COM_INFOS
{
    int16_t (MSPFETSim::*comGetLayerVersion)(void);
    int16_t (MSPFETSim::*comConfig)(uint32_t Baudrate, uint32_t MCLK_Frequency, uint16_t);
    int16_t (MSPFETSim::*comTransmit)(void);
    int16_t (MSPFETSim::*comReceive)(uint16_t character);
    void    (MSPFETSim::*comClose)(void);
    void    (MSPFETSim::*comSetHil)(edt_common_methods_t*);
    void    (MSPFETSim::*comSetDcdc)(DCDC_INFOS_t*);
    void    (MSPFETSim::*comSetUSB)(FET_USB_INFOS_t*);
    void    (MSPFETSim::*comLoop)(void);
    int16_t (MSPFETSim::*comConfigMode)(uint32_t Baudrate);
    int16_t (MSPFETSim::*comSetCts)(void);
    int16_t (MSPFETSim::*comClearCts)(void);
    void    (MSPFETSim::*comSetRts)(void);
    int16_t (MSPFETSim::*comGetLayerVersionCmp)(void);
};
typedef struct COM_INFOS COM_INFOS_t;

typedef void (MSPFETSim::*DcdcInit)(DCDC_INFOS_t* dcdcInfos_Pointer);
typedef void (MSPFETSim::*ComInit)(COM_INFOS_t* comInfos_Pointer);





#define STATIC_VARS_START(fn) auto& sv = fn ## _staticVars
#define DECL_STATIC_VAR(n) auto& n = sv.n

#define MEMBER_FN_PTR(fn)       (&MSPFETSim::fn)
#define CALL_MEMBER_FN_PTR(fn)  (this->*fn)




#define PTR_FOR_CMP uintptr_t

#define INLINE(x)
#define REQUIRED(x)
#define DIAG_DEFAULT(x)
#define DIAG_SUPPRESS(x)
#define _NOP()
#define _DINT_FET()
#define _EINT_FET()
#define CONST_AT(x, y) const x


#define eZ_FET_WITH_DCDC                0xAAAA
#define eZ_FET_NO_DCDC                  0xAAAB
#define eZ_FET_WITH_DCDC_NO_FLOWCTL     0xAAAC
#define MSP_FET_WITH_DCDC               0xBBBB
#define MSP_FET_WITH_DCDC_V2x           0xBBBC
#define eZ_FET_WITH_DCDC_V2x            0xAAAD



#define HAL_SIGNATURE   0xBEEFBEEFul
#define HIL_SIGNATURE   0xF00DF00Dul
#define DCDC_SIGNATURE  0xABBAABBAul
#define COM_SIGNATURE   0xACDCACDCul
