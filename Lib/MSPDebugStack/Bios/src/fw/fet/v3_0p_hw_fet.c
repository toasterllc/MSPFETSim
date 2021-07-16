/*
 * v3_0p_hw_nguif.c
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

//!  \ingroup MODULBIOS
//!  \file v3_0p_hw_uif.c
//!  \brief Bios included HAL (zero) and update function
//! \li forwarding (execute) messages to HAL
//! \li upload of HAL macros
//! \li loop management

#include "hw_compiler_specific.h"
#include <stdlib.h>

#include "bios.h"
#include "v3_0p.h"
#include "HAL_FLASH.h"
#include "MemorySegments.h"
#include "../../hil/msp_fet/archFpga.h"
#include "../fet/v3_0p_hw_fet.h"

#include "USB_API/USB_Common/types.h"
#include "USB_API/USB_Common/usb.h"                 //USB-specific functions
#include "F5xx_F6xx_Core_Lib/HAL_UCS.h"


unsigned short calculateCrc(unsigned short sum, unsigned short *adress, unsigned long segmentLength)
{
  //Initialize CRC register
  CRCINIRES = sum;

  //Compute CRC over the given segment
  while (segmentLength--)
  {
    CRCDIRB = *adress++;
  }
  //Return CRC result
  return CRCINIRES;
}

unsigned short V3OP_GetHilCrc()
{
    unsigned short hilCrc = 0x0000;
    unsigned long segmentLength = 0;
    unsigned short *address  = 0;

    //calculate CRC for HIL info segment 1 -------------------------------------
    segmentLength = (CHECKSUM_HIL[0] - INFO_SEGMENTS_HIL[0])/2;
    address = (unsigned short*)INFO_SEGMENTS_HIL[0];
    hilCrc = calculateCrc(hilCrc, address, segmentLength);

    //calculate CRC for HIL info segment 2
    segmentLength = (INFO_SEGMENTS_HIL[1] - CHECKSUM_HIL[1])/2;
    address = (unsigned short*)(CHECKSUM_HIL[1] + 1);
    hilCrc = calculateCrc(hilCrc, address, segmentLength);

    //calculate CRC for HIL main segment
    segmentLength = (HIL_SEGMENTS[1] - HIL_SEGMENTS[0]+1)/2;
    address = (unsigned short*)HIL_SEGMENTS[0];
    hilCrc = calculateCrc(hilCrc, address, segmentLength);

#ifdef MSP_FET
    //calculate CRC for Hil main segment  2
    segmentLength = (HIL_SEGMENTS[3] - HIL_SEGMENTS[2] + 1)/2;
    address = (unsigned short*)HIL_SEGMENTS[2];
    hilCrc = calculateCrc(hilCrc, address, segmentLength);
#endif
    //--------------------------------------------------------------------------

    return hilCrc;
}

unsigned short V3OP_GetHalFpgaCrc()
{
    unsigned short halCrc = 0x0000;
#ifdef MSP_FET
    unsigned long segmentLength = 0;
    unsigned short *address  = 0;

     //calculate CRC for Hal info segment 1 --------------------------------------
    segmentLength = (CHECKSUM_HAL[0] - INFO_SEGMENTS_HAL[0])/2;
    address = (unsigned short*)INFO_SEGMENTS_HAL[0];
    halCrc = calculateCrc(halCrc, address, segmentLength);
    //calculate CRC for Hal info segment 2
    segmentLength = (INFO_SEGMENTS_HAL[1] - CHECKSUM_HAL[1])/2;
    address = (unsigned short*)(CHECKSUM_HAL[1] + 1);
    halCrc = calculateCrc(halCrc, address, segmentLength);

     //calculate CRC for Hal main segment  1
    segmentLength = (HAL_FPGA_SEGMENTS[1] - HAL_FPGA_SEGMENTS[0] + 1)/2;
    address = (unsigned short*)HAL_FPGA_SEGMENTS[0];
    halCrc = calculateCrc(halCrc, address, segmentLength);
    //--------------------------------------------------------------------------
#endif
    return halCrc;
}

unsigned short V3OP_GetHalCrc()
{
    unsigned short halCrc = 0x0000;
    unsigned long segmentLength = 0;
    unsigned short *address  = 0;

     //calculate CRC for Hal info segment 1 --------------------------------------
    segmentLength = (CHECKSUM_HAL[0] - INFO_SEGMENTS_HAL[0])/2;
    address = (unsigned short*)INFO_SEGMENTS_HAL[0];
    halCrc = calculateCrc(halCrc, address, segmentLength);
    //calculate CRC for Hal info segment 2
    segmentLength = (INFO_SEGMENTS_HAL[1] - CHECKSUM_HAL[1])/2;
    address = (unsigned short*)(CHECKSUM_HAL[1] + 1);
    halCrc = calculateCrc(halCrc, address, segmentLength);

     //calculate CRC for Hal main segment  1
    segmentLength = (HAL_SEGMENTS[1] - HAL_SEGMENTS[0] + 1)/2;
    address = (unsigned short*)HAL_SEGMENTS[0];
    halCrc = calculateCrc(halCrc, address, segmentLength);
#ifdef eZ_FET
    //calculate CRC for Hal main segment  2
    segmentLength = (HAL_SEGMENTS[3] - HAL_SEGMENTS[2] + 1)/2;
    address = (unsigned short*)HAL_SEGMENTS[2];
    halCrc = calculateCrc(halCrc, address, segmentLength);
#endif
    //--------------------------------------------------------------------------

    return halCrc;
}

unsigned short V3OP_GetCoreCrc()
{
    unsigned short coreCrc = 0x0000;
    unsigned long segmentLength = 0;
    unsigned short *address  = 0;

    //calculate CRC for core info segment 1 ------------------------------------
    segmentLength = (CHECKSUM_CORE[0] - CORE_SEGMENTS[0])/2;
    address = (unsigned short*)CORE_SEGMENTS[0];
    coreCrc = calculateCrc(coreCrc, address, segmentLength);

    //calculate CRC for core main segment
    segmentLength = (CORE_SEGMENTS[1] - CHECKSUM_CORE[1] + 1)/2;
    address = (unsigned short*) (CHECKSUM_CORE[1] + 1);
    coreCrc = calculateCrc(coreCrc, address, segmentLength);

    //calculate CRC for reset vector segment
    segmentLength = (CORE_SEGMENTS_RESET[1] - CORE_SEGMENTS_RESET[0] + 1)/2;
    address = (unsigned short*)CORE_SEGMENTS_RESET[0];
    coreCrc = calculateCrc(coreCrc, address, segmentLength);
    //--------------------------------------------------------------------------

    return coreCrc;
}

unsigned short V3OP_GetDcdcCrc()
{
    unsigned short dcdcCrc = 0x0000;
    unsigned long segmentLength = 0;
    unsigned short *address  = 0;

    //calculate CRC for dcdc info segment 1 -------------------------------------
    segmentLength = (CHECKSUM_DCDC[0] - INFO_SEGMENTS_DCDC[0])/2;
    address = (unsigned short*)INFO_SEGMENTS_DCDC[0];
    dcdcCrc = calculateCrc(dcdcCrc, address, segmentLength);

    //calculate CRC for dcdc info segment 1
    segmentLength = (INFO_SEGMENTS_DCDC[1] - CHECKSUM_DCDC[1])/2;
    address = (unsigned short*)(CHECKSUM_DCDC[1] + 1);
    dcdcCrc = calculateCrc(dcdcCrc, address, segmentLength);

    //calculate CRC for dcdc main segment
    segmentLength = (DCDC_SEGMENTS[1] - DCDC_SEGMENTS[0] + 1)/2;
    address = (unsigned short*)DCDC_SEGMENTS[0];
    dcdcCrc = calculateCrc(dcdcCrc, address, segmentLength);
    //--------------------------------------------------------------------------

    return dcdcCrc;
}

unsigned short V3OP_GetComChannelCrc()
{
    unsigned short comChannelCrc = 0x0000;
    unsigned long segmentLength = 0;
    unsigned short *address  = 0;

    //calculate CRC for comchannel info segment 1 -------------------------------------
    segmentLength = (CHECKSUM_COMCHANNEL[0] - INFO_SEGMENTS_COMCHANNEL[0])/2;
    address = (unsigned short*)INFO_SEGMENTS_COMCHANNEL[0];
    comChannelCrc = calculateCrc(comChannelCrc, address, segmentLength);

    //calculate CRC for comchannel info segment 1
    segmentLength = (INFO_SEGMENTS_COMCHANNEL[1] - CHECKSUM_COMCHANNEL[1])/2;
    address = (unsigned short*)(CHECKSUM_COMCHANNEL[1] + 1);
    comChannelCrc = calculateCrc(comChannelCrc, address, segmentLength);

    //calculate CRC for comchannel main segment
    segmentLength = (COMCHANNEL_SEGMENTS[1] - COMCHANNEL_SEGMENTS[0] + 1)/2;
    address = (unsigned short*)COMCHANNEL_SEGMENTS[0];
    comChannelCrc = calculateCrc(comChannelCrc, address, segmentLength);
    //--------------------------------------------------------------------------

  return comChannelCrc;
}

unsigned char V3OP_HilCrcOk()
{
  return(V3OP_GetHilCrc() == *((unsigned short*)CHECKSUM_HIL[0]));
}

unsigned char V3OP_HalCrcOk()
{
  return(V3OP_GetHalCrc() == *((unsigned short*)CHECKSUM_HAL[0]));
}

unsigned char V3OP_HalFpgaCrcOk()
{
  return(V3OP_GetHalFpgaCrc() == *((unsigned short*)CHECKSUM_HAL[0]));
}

unsigned char V3OP_coreCrcOk()
{
  return(V3OP_GetCoreCrc() == *((unsigned short*)CHECKSUM_CORE[0]));
}

unsigned char V3OP_DcdcCrcOk()
{
  return(V3OP_GetDcdcCrc() == *((unsigned short*)CHECKSUM_DCDC[0]));
}

unsigned char V3OP_ComChannelCrcOk()
{
  return(V3OP_GetComChannelCrc() == *((unsigned short*)CHECKSUM_COMCHANNEL[0]));
}
//! \brief test address on memory type
//! \param[in] addr address to test
//! \return 0 -> flash memory
//! \return 1 -> info memory
//! \return 2 -> RAM
#pragma optimize = low
unsigned long V3OP_GetSegmentType(unsigned long addr)
{
    unsigned short i = 0;
    for(i = 0; i < (sizeof(INFO_SEGMENTS_HIL)/sizeof(unsigned long)); i+=2)
    {
        if((addr >= INFO_SEGMENTS_HIL[i]) && (addr <= INFO_SEGMENTS_HIL[i+1]))
        {
            return (INFO_SEGMENT_HIL);
        }
    }
    for(i = 0; i < (sizeof(INFO_SEGMENTS_HAL)/sizeof(unsigned long)); i+=2)
    {
        if((addr >= INFO_SEGMENTS_HAL[i]) && (addr <= INFO_SEGMENTS_HAL[i+1]))
        {
            return (INFO_SEGMENT_HAL);
        }
    }
    for(i = 0; i < (sizeof(HAL_SEGMENTS)/sizeof(unsigned long)); i+=2)
    {
        if((addr >= HAL_SEGMENTS[i]) && (addr <= HAL_SEGMENTS[i+1]))
        {
            return (HAL_SEGMENT);
        }
    }
    #ifdef MSP_FET
    for(i = 0; i < (sizeof(HAL_FPGA_SEGMENTS)/sizeof(unsigned long)); i+=2)
    {
        if((addr >= HAL_FPGA_SEGMENTS[i]) && (addr <= HAL_FPGA_SEGMENTS[i+1]))
        {
            return (HAL_SEGMENT_FPGA);
        }
    }
    #endif
    for(i = 0; i < (sizeof(HIL_SEGMENTS)/sizeof(unsigned long)); i+=2)
    {
        if((addr >= HIL_SEGMENTS[i]) && (addr <= HIL_SEGMENTS[i+1]))
        {
            return (HIL_SEGMENT);
        }
    }
    // dcdc segments
    for(i = 0; i < (sizeof(DCDC_SEGMENTS)/sizeof(unsigned long)); i+=2)
    {
        if((addr >= DCDC_SEGMENTS[i]) && (addr <= DCDC_SEGMENTS[i+1]))
        {
            return (DCDC_SEGMENT);
        }
    }
    for(i = 0; i < (sizeof(INFO_SEGMENTS_DCDC)/sizeof(unsigned long)); i+=2)
    {
        if((addr >= INFO_SEGMENTS_DCDC[i]) && (addr <= INFO_SEGMENTS_DCDC[i+1]))
        {
            return (INFO_SEGMENT_DCDC);
        }
    }
    // comchannel segments
    for(i = 0; i < (sizeof(COMCHANNEL_SEGMENTS)/sizeof(unsigned long)); i+=2)
    {
        if((addr >= COMCHANNEL_SEGMENTS[i]) && (addr <= COMCHANNEL_SEGMENTS[i+1]))
        {
            return (COMCHANNEL_SEGMENT);
        }
    }
    // INFO comchannel segments
    for(i = 0; i < (sizeof(INFO_SEGMENTS_COMCHANNEL)/sizeof(unsigned long)); i+=2)
    {
        if((addr >= INFO_SEGMENTS_COMCHANNEL[i]) && (addr <= INFO_SEGMENTS_COMCHANNEL[i+1]))
        {
            return (INFO_SEGMENT_COMCHANNEL);
        }
    }
    return 0;
}

//! \brief test address on write access
//! \param[in] addr address to test
//! \return 0 -> no write access
//! \return 1 -> write allow
unsigned char V3OP_WriteAllowed(unsigned short addr)
{
    return(0);
}
//! \brief test address on erase access
//! \param[in] addr address to test
//! \return 0 -> no erase access
//! \return 1 -> erase allow
unsigned char V3OP_EraseAllowed(unsigned short StartAddr)
{
    return(0);
}

//! \brief erase on UIF (HAL) all blocks which included in start address + size
//! \param[in] *payload pointer to receive buffer
//! \return 1 -> flash erase done
//! \return <0 -> error
#pragma optimize = low
short V3OP_CoreFlashFunctionErase(unsigned char *payload)
{
    unsigned long address_pointer;
    unsigned long  start_addr = (*(unsigned long*)&payload[4] & 0xFFFFF);
    unsigned long  end_addr   = start_addr + (*(unsigned long*)&payload[8] & 0xFFFFF)-2;
    unsigned char  segment_type;
    unsigned long segmentSize =0;

    segment_type = V3OP_GetSegmentType(start_addr);

    if(segment_type == NO_SEGMENT)
    {
        return -1;
    }

    // erase INFO mem
    if(segment_type == HAL_SEGMENT || segment_type == INFO_SEGMENT_HAL)
    {
        segmentSize = SEGMENT_SIZE_HAL_HIL;
        // First erase entry point into Hil Layer
        // IF firmware update is interrupted HIL could not be started
        Flash_SegmentErase((unsigned short*)INFO_SEGMENTS_HAL[1]);
    }

    if(segment_type == HIL_SEGMENT || segment_type == INFO_SEGMENT_HIL)
    {
        segmentSize = SEGMENT_SIZE_HAL_HIL;
        // First erase entry point into HAL Layer
        // IF firmware update is interrupted HAL could not be started
        Flash_SegmentErase((unsigned short*)INFO_SEGMENTS_HIL[0]);
    }

    if(segment_type == INFO_SEGMENT_DCDC || segment_type == DCDC_SEGMENT)
    {
        segmentSize = SEGMENT_SIZE_HAL_HIL;
        // First erase entry point into HAL Layer
        // IF firmware update is interrupted HAL could not be started
        Flash_SegmentErase((unsigned short*)INFO_SEGMENTS_DCDC[0]);
    }

    if(segment_type == COMCHANNEL_SEGMENT || segment_type == INFO_SEGMENT_COMCHANNEL)
    {
        segmentSize = SEGMENT_SIZE_HAL_HIL;
        // First erase entry point into HAL Layer
        // IF firmware update is interrupted HAL could not be started
        UnlockInfoA();
        Flash_SegmentErase((unsigned short*)INFO_SEGMENTS_COMCHANNEL[0]);
    }

    // Erase HIL/HAL/DCDC/COM
    for(address_pointer = start_addr; address_pointer < end_addr; address_pointer += segmentSize)
    {
        if(V3OP_GetSegmentType(address_pointer) != NO_SEGMENT)
        {
            Flash_SegmentErase((unsigned short*)address_pointer);
        }
    }

    // Do erase check of HIL/HAL/DCDC
    for(address_pointer = start_addr; address_pointer < end_addr; address_pointer += segmentSize)
    {
        if(V3OP_GetSegmentType(address_pointer) != NO_SEGMENT)
        {
            Flash_EraseCheck((unsigned char*)address_pointer, (segmentSize+1)); // (unsigned long *Flash_ptr, unsigned long len)
        }
    }
    LockInfoA();
    return(1);
}
//! \brief write payload to UIF (HAL) flash memory, RAM is allowed, too
//! \param[in] *payload pointer to receive buffer
//! \details on first function call the:
//! \li bytes 4 -7: the flash address
//! \li bytes 8 -11: data length
//! \li bytes 12 ... payload
//! \details if data length is larger then message size the function returns after writing the message. Direct follow calls
//! of this function continue writing.
//! \return 1 -> flash write done
//! \return <0 -> error
#pragma optimize = low
short V3OP_CoreFlashFunctionWrite(unsigned char *payload, unsigned short v30p_stream_flags_)
{
    static unsigned long address_pointer;
    static unsigned long  start_addr = 0; //(*(unsigned long*)&payload[4] & 0xFFFFF);
    static unsigned long  end_addr   = 0; //start_addr + (*(unsigned long*)&payload[8] & 0xFFFFF);
    unsigned char  segment_type;
    unsigned short *data_ptr = NULL;
    unsigned short *data_ptr_end = NULL;
    static unsigned long segmentSize =0;


    if(v30p_stream_flags_ & MESSAGE_NEW_MSG)
    {
        start_addr = (*(unsigned long*)&payload[4] & 0xFFFFF);
        end_addr   = start_addr + (*(unsigned long*)&payload[8] & 0xFFFFF);
        data_ptr = (unsigned short*)&payload[12];
        data_ptr_end = data_ptr + (payload[0] - 11) / sizeof(unsigned short); // write words /2
        segment_type = V3OP_GetSegmentType(start_addr);

        if(segment_type == NO_SEGMENT)
        {
            return -1;
        }
        // 2 bytes are written in one line
        segmentSize = 2;
        address_pointer = start_addr;
        UnlockInfoA();
    }
    else
    {
        data_ptr = (unsigned short*)&payload[4];
        data_ptr_end = data_ptr + (payload[0] - 3) / sizeof(unsigned short);
        LockInfoA();
    }
    for(;(address_pointer < end_addr) && (data_ptr < (data_ptr_end)); address_pointer += segmentSize)
    {
        // check if wirte is going into info or HAL/Hil segment
        // don't programm any core segment
        if(V3OP_GetSegmentType(address_pointer) != NO_SEGMENT)
        {
            FlashWrite_16(data_ptr, (unsigned short*)address_pointer, 1);
            data_ptr++;
        }
    }
    return 0;
}

//! \brief reads UIF memory, inclusive BIOS area
//! \param[in] *payload pointer to receive buffer
//! \li bytes 4 -7: the flash address
//! \li bytes 8 -11: data length
//! \return 0
short V3OP_CoreFlashFunctionRead(unsigned char *payload)
{
    return(0);
}

void V3OP_UpCore(void)
{
    _DINT_FET(); // Ensure no application interrupts fire during stop
    USB_disconnect();
    FPGA_RESET_ASSERT
    //~1.6 s delay
    __delay_cycles(40000000);

    USB_disable();
    XT2_Stop();
    //erase infoB
    Flash_SegmentErase((unsigned short*)0x197F);
    //erase infoC
    Flash_SegmentErase((unsigned short*)0x18FF);
    //erase infoD
    Flash_SegmentErase((unsigned short*)0x187F);
    // erase flash reset vector to invoke usb bsl by next startup
    Flash_SegmentErase((unsigned short*)0xFFFE);

    //~1.6 s delay
    __delay_cycles(40000000);
    PMMCTL0 = PMMPW | PMMSWBOR; // generate BOR for reseting device
}

unsigned short V3OP_SystemOk(void)
{
    if(!V3OP_coreCrcOk())
    {
        Flash_SegmentErase((unsigned short*)0xFFFE);
        return 0;
    }
    return 1;
}
