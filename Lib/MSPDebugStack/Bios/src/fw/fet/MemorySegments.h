/*
 * MemorySegments.h
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

// This header file defines the memory segments for each FET
// The values are used during the firmware update process

// Info memory
// Used for storing firmware module specific information (e.g. signature, checksum)
// These segments stay the same for both, eZ-FET and MSP-FET
const unsigned long INFO_SEGMENTS_COMCHANNEL[] = {0x1980,0x19FF}; // INFOA
const unsigned long INFO_SEGMENTS_HAL[] ={0x1900, 0x197F}; // INFOB
const unsigned long INFO_SEGMENTS_HIL[] ={0x1880, 0x18FF}; // INFOC
const unsigned long INFO_SEGMENTS_DCDC[] ={0x1800, 0x187F}; // INFOD


// Checksum address for each firmware module
const unsigned long CHECKSUM_COMCHANNEL[] = {0x19FA, 0x19FB};
const unsigned long CHECKSUM_HAL[] = {0x197A, 0x197B};
const unsigned long CHECKSUM_HIL[] = {0x18FA, 0x18FB};
const unsigned long CHECKSUM_DCDC[] = {0x187A, 0x187B};
// Core checksum is not FET independent because it's not stored in INFO memory
// but rather on the beginning of the core segment


// Flash segments for each firmware module
#ifdef eZ_FET
    // Core checksum is not FET independent because it's not stored in INFO memory
    // but rather on the beginning of the core segment
    const unsigned long CHECKSUM_CORE[] = {0x4402, 0x4403};

    const unsigned long HAL_SEGMENTS[] = {0x0E000, 0x0FDFF, 0x10000, 0x1DFFF};
    const unsigned long HIL_SEGMENTS[] = {0x8A00, 0xDFFF};
    const unsigned long DCDC_SEGMENTS[] = {0x23000,0x243FF};
    const unsigned long COMCHANNEL_SEGMENTS[] = {0x21000,0x22FFF};
    const unsigned long CORE_SEGMENTS[] = {0x4400,0x89FF};
    const unsigned long CORE_SEGMENTS_RESET[] = {0xFF80,0xFFFF};
#endif

#ifdef MSP_FET
    const unsigned long CHECKSUM_CORE[] = {0x8002, 0x8003};

    const unsigned long HAL_SEGMENTS[] = {0x18E00, 0x30DFF};
    const unsigned long HAL_FPGA_SEGMENTS[] = {0x18E00, 0x45FFF};
    const unsigned long HIL_SEGMENTS[] = {0xCC00, 0xFDFF, 0x10000, 0x18DFF};
    const unsigned long DCDC_SEGMENTS[] = {0x47200,0x47FFF};
    const unsigned long COMCHANNEL_SEGMENTS[] = {0x46000,0x471FF};
    const unsigned long CORE_SEGMENTS[] = {0x8000,0xCBFF};
    const unsigned long CORE_SEGMENTS_RESET[] = {0xFE00,0xFFFF};
#endif


// Segment types
// These are used during the update process to determine which segments to erase
// or write
const unsigned char NO_SEGMENT = 0;
const unsigned char INFO_SEGMENT_HIL = 1;
const unsigned char INFO_SEGMENT_HAL = 2;
const unsigned char HAL_SEGMENT = 3;
const unsigned char HIL_SEGMENT = 4;
const unsigned char INFO_SEGMENT_DCDC = 5;
const unsigned char DCDC_SEGMENT = 6;
const unsigned char INFO_SEGMENT_COMCHANNEL = 7;
const unsigned char COMCHANNEL_SEGMENT = 8;
const unsigned char HAL_SEGMENT_FPGA = 9;

// Segment size of different memory parts (e.g. INFO memory, Flash memory)
const unsigned short   SEGMENT_SIZE_INFO = 128; // Segment size in bytes of Flash
const unsigned short   SEGMENT_SIZE_HAL_HIL = 512; // Segment size in bytes of Flash
