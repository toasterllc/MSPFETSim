/*
 * \ingroup MODULMACROS
 *
 * \file MEMAPTransactionArm.c
 *
 * \brief <FILEBRIEF>
 *
 *
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
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

#include "arch.h"
#include "edt.h"
#include "hal.h"
#include "stream.h"
#include "JTAG_defs.h"
#include "error_def.h"
#include <string.h>

/**
  MEMAPTransaction
  Performs a memory transaction (Read/Write) on the specified port
  PortNum<8>: selects the port number
  RnW<8>: selects whether to perform a Read or a Write trasaction
  Datawidth<16>: Indicates the width of the data transaction (8-/16-/32-bits)
  Address<32>: Destination address
  Count<32>: The number of bytes to write
*/
#if defined(MSP430_UIF) || defined(MSP_FET)
    extern unsigned long cswValues[4];
#endif

#define ADDRESS_WRAPPING 0x00000FFFul

/// \brief Low-level write function with timeout
/// \return 0 if timeout occured and non-zero if success
#pragma inline=forced
unsigned char writeLowLevel(unsigned char address, unsigned long dataIn)
{
    unsigned char retry = MAX_RETRY;
    unsigned char ack = 0;
    unsigned long long dataIn64 = ((unsigned long long)dataIn) << 3;

    do
    {
        unsigned long long tmp = dataIn64 | (address >> 1) | WRITE;
        ack = SetReg_35Bits(&tmp) & 0x7;
    } while((ack != ACK) && (--retry));

    return retry;
}

/// \brief Low-level read function with timeout
/// \return 0 if timeout occured and non-zero if success
#pragma inline=forced
unsigned char readLowLevel(unsigned char address, unsigned long *dataInOut)
{
    unsigned char retry = MAX_RETRY;
    unsigned long long dataIn64 = ((unsigned long long)*dataInOut) << 3;
    unsigned long long dataOut64;

    do
    {
        unsigned long long tmp = dataIn64 | (address >> 1) | READ;
        dataOut64 = SetReg_35Bits(&tmp);
    }
    while((dataOut64 & 0x7 != ACK) && (--retry));

    *dataInOut = dataOut64 >> 3;

    return retry;
}

void SwdHandleError()
{
    unsigned long errorCheck = 0, resetError = 0;
    // hanlde locked out state
    IHIL_Write_Read_Dp(DP_DPIDR, &errorCheck, READ);
    // read
    IHIL_Write_Read_Dp(DP_CTRL_STAT, &errorCheck, READ);

    if(errorCheck & DP_CTRL_STAT_STICKYCMP)
    {
        resetError |= STKCMPCLR;
    }
    if(errorCheck & DP_CTRL_STAT_STICKYERR)
    {
        resetError |= STKERRCLR;
    }
    if(errorCheck & DP_CTRL_STAT_WDATAERR)
    {
        resetError |= WDERRCLR;
    }
    if(errorCheck & DP_CTRL_STAT_STICKYORUN)
    {
        resetError |= ORUNERRCLR;
    }
    if(resetError != 0)
    {
        IHIL_Write_Read_Dp(DP_DPIDR, &resetError, WRITE);
    }
}

short JtagHandleError()
{
    short retVal = 0;
    unsigned long errorCheck = 0, resetError = 0;
    // Check for errors
    //prepare read CTRL/STAT reg
    IHIL_Write_Read_Dp(DP_CTRL_STAT, &errorCheck, READ);
    //read CTRL/STAT reg
    IHIL_Write_Read_Dp(DP_CTRL_STAT, &errorCheck, READ);

    resetError = errorCheck;
    if(errorCheck & DP_CTRL_STAT_STICKYCMP)
    {
        resetError |= DP_CTRL_STAT_STICKYCMP;
    }
    if(errorCheck & DP_CTRL_STAT_STICKYERR)
    {
        resetError |= DP_CTRL_STAT_STICKYERR;
    }
    if(errorCheck & DP_CTRL_STAT_WDATAERR)
    {
        resetError |= DP_CTRL_STAT_WDATAERR;
    }
    if(errorCheck & DP_CTRL_STAT_STICKYORUN)
    {
        resetError |= DP_CTRL_STAT_STICKYORUN;
    }
    if(resetError != 0)
    {
        IHIL_Write_Read_Dp(DP_CTRL_STAT, &resetError, WRITE);

         //prepare read CTRL/STAT reg
        IHIL_Write_Read_Dp(DP_CTRL_STAT, &errorCheck, READ);
        //read CTRL/STAT reg
        IHIL_Write_Read_Dp(DP_CTRL_STAT, &errorCheck, READ);
        if(errorCheck & DP_CTRL_STAT_STICKYERR ||
           errorCheck & DP_CTRL_STAT_STICKYORUN ||
           errorCheck & DP_CTRL_STAT_STICKYCMP ||
           errorCheck & DP_CTRL_STAT_WDATAERR)
        {
            retVal = -1;
        }
        else
        {
            retVal = 0;
        }
    }
    else
    {
        retVal = 0;
    }
    return retVal;
}

/// \brief Stream the correct data size back to the DLL
#define STREAM_DATA(data)                                     \
    do                                                        \
    {                                                         \
        data >>= (address & 0x3) * 8;                         \
        switch(dataWidth)                                     \
        {                                                     \
        case AP_CSW_SIZE_8BIT:                                \
            STREAM_put_byte((unsigned char)data);             \
            break;                                            \
        case AP_CSW_SIZE_16BIT:                               \
            STREAM_put_word((unsigned short)data);            \
            break;                                            \
        case AP_CSW_SIZE_32BIT:                               \
            STREAM_put_long((unsigned long)data);             \
            break;                                            \
        }                                                     \
    } while(0)


HAL_FUNCTION(_hal_MemApTransactionArm)
{
    short retVal = 0;
#if defined(MSP430_UIF) || defined(MSP_FET)
    static unsigned long long apsel;
    static unsigned short rnw;
    static unsigned short dataWidth;
    static unsigned long address;
    static unsigned long count;

    unsigned char *pBuf;
    unsigned short sizeOfBuf;

    unsigned long data;

    if(flags & MESSAGE_NEW_MSG)
    {   // Do initial setup
        apsel = 0ull;
        if(STREAM_get_word((unsigned short *)&apsel) == -1)
        {
            return -1;
        }
        if(STREAM_get_word(&rnw) == -1)
        {
            return -1;
        }
        if(STREAM_get_word(&dataWidth) == -1)
        {
            return -1;
        }
        if(STREAM_get_long(&address) == -1)
        {
            return -1;
        }
        if(STREAM_get_long(&count) == -1)
        {
            return -1;
        }
        if(!count)
        { // Nothing to do, just return without failure
            return 0;
        }

        apsel <<= 24;
        count >>= dataWidth; // Calculate the number of transactions needed

        JtagHandleError();

        // Write CSW register
        data = cswValues[apsel & 0x3] | ((count == 1) ? AP_CSW_ADDRINC_OFF : AP_CSW_ADDRINC_SINGLE) | dataWidth;
        IHIL_Write_Read_Ap(AP_CSW, &data, WRITE);

        // Write TAR register
        IHIL_Write_Read_Ap(AP_TAR, &address, WRITE);

    }

    // Now that the setup is done, do the actual data transfer
    if(rnw == WRITE)
    {
        STREAM_out_change_type(RESPTYP_ACKNOWLEDGE);
        STREAM_flush();
        STREAM_get_buffer((void **)&pBuf,&sizeOfBuf);
        while(count && sizeOfBuf)
        {
            switch(dataWidth)
            {
            case 0:
              data = (*((unsigned char*) pBuf));
              break;
            case 1:
              data = (*((unsigned short*) pBuf));
              break;
            default:
              data = (*((unsigned long*) pBuf));
              break;
            }
            data <<= (address & 0x3) * 8;  // Move to correct byte lane

            writeLowLevel(AP_DRW, data);

            pBuf += (1 << dataWidth);
            address += (1 << dataWidth);
            if(sizeOfBuf >= (1 << dataWidth))
            {
                sizeOfBuf -= (1 << dataWidth);
            }
            else
            {
                sizeOfBuf = 0;
            }
            --count;

            // Check for address wrapping
            if(count && !(address & ADDRESS_WRAPPING))
            { // Write TAR register
                writeLowLevel(AP_TAR, address);
            }
        }
    }
    else
    { // READ
        data = 0;
        // Initiate read
        readLowLevel(AP_DRW,&data);

        while(--count)
        {
            readLowLevel(AP_DRW,&data);
            STREAM_DATA(data);
            // Check for address wrapping
            if(count)
            {
                address += (1 << dataWidth);
                if(!(address & ADDRESS_WRAPPING))
                { // Write TAR register
                    JtagHandleError();
                    IHIL_Instr4(IR4_APACC);
                    writeLowLevel(AP_TAR, address);
                    readLowLevel(AP_DRW,&data);
                }
            }
        }
        IHIL_Instr4(IR4_DPACC);
        readLowLevel(DP_RDBUFF,&data);
        STREAM_DATA(data);
    }

    if(count)
    { // More data expected
        retVal = 1;
    }
    else
    {
        data = 0;
        IHIL_Write_Read_Dp(DP_RDBUFF, &data, READ);
        retVal = JtagHandleError();
    }

#endif
    return retVal;
}



HAL_FUNCTION(_hal_MemApTransactionArmSwd)
{
    short retVal = 0;
#ifdef MSP_FET
    static unsigned long long apsel;
    static unsigned short rnw;
    static unsigned short dataWidth;
    static unsigned long address;
    static unsigned long count;

    unsigned char *pBuf;
    unsigned short sizeOfBuf;

    unsigned long data;

    if(flags & MESSAGE_NEW_MSG)
    {   // Do initial setup
        apsel = 0ull;
        if(STREAM_get_word((unsigned short *)&apsel) == -1)
        {
            return -1;
        }
        if(STREAM_get_word(&rnw) == -1)
        {
            return -1;
        }
        if(STREAM_get_word(&dataWidth) == -1)
        {
            return -1;
        }
        if(STREAM_get_long(&address) == -1)
        {
            return -1;
        }
        if(STREAM_get_long(&count) == -1)
        {
            return -1;
        }
        if(!count)
        { // Nothing to do, just return without failure
            return 0;
        }
        count >>= dataWidth; // Calculate the number of transactions needed

        // enalbe ORUNDETECT in CTRL/STAT
        IHIL_Write_Read_Dp(DP_CTRL_STAT, &data, READ);
        data |= 0x00000001;
        IHIL_Write_Read_Dp(DP_CTRL_STAT, &data, WRITE);


        // Write CSW register
        data = cswValues[apsel & 0x3] | ((count == 1) ? AP_CSW_ADDRINC_OFF : AP_CSW_ADDRINC_SINGLE) | dataWidth;
        IHIL_Write_Read_Ap(AP_CSW, &data, WRITE);

        // Write TAR register
        IHIL_Write_Read_Ap(AP_TAR, &address, WRITE);

        if(rnw == READ)
        {
            unsigned long long retVal = 0, i = 0;
            // dummy read
            unsigned char regiser  = SWD_AP | READ << 1 | (AP_DRW & 0x0c);
            do
            {
                retVal = IHIL_SwdTransferData(regiser, &data, READ);
                SwdHandleError();
            }
            while(++i < MAX_RETRY && retVal != SWD_ACK);

            if(i == MAX_RETRY)
            {
                return -1;
            }
            // dummy read end
        }
    }
    if(rnw == WRITE)
    {
        STREAM_get_buffer((void **)&pBuf,&sizeOfBuf);
        while(count && sizeOfBuf)
        {
            unsigned long long retVal = 0, i = 0;
            // Move to correct byte lane
            switch(dataWidth)
            {
            case 0:
              data = (*((unsigned char*) pBuf));
              break;
            case 1:
              data = (*((unsigned short*) pBuf));
              break;
            default:
              data = (*((unsigned long*) pBuf));
              break;
            }
            data <<= (address & 0x3) * 8;

            unsigned char regiser  = SWD_AP | WRITE << 1 | (AP_DRW & 0x0c);
            do
            {
                retVal = IHIL_SwdTransferData(regiser, &data, WRITE);
            }
            while(++i < MAX_RETRY && retVal != SWD_ACK);

            if(i == MAX_RETRY)
            {
                    return -1;
            }

            pBuf += (1 << dataWidth);
            address += (1 << dataWidth);
            if(sizeOfBuf >= (1 << dataWidth))
            {
                sizeOfBuf -= (1 << dataWidth);
            }
            else
            {
                sizeOfBuf = 0;
            }
            --count;

            // Check for address wrapping
            if(count && !(address & ADDRESS_WRAPPING))
            { // Write TAR register
                 IHIL_Write_Read_Ap(AP_TAR, &address, WRITE);
                 SwdHandleError();
            }
        }
    }
    else
    {
        //Read
        unsigned long long retVal = 0, i = 0;
        unsigned char regiser  = SWD_AP | READ << 1 | (AP_DRW & 0x0c);

        while(--count)
        {
            i = 0;
            data = 0;
            do
            {
                retVal = IHIL_SwdTransferData(regiser, &data, READ);
            }
            while(++i < MAX_RETRY && retVal != SWD_ACK);
            if(i == MAX_RETRY)
            {
                data = 0;
            }
            STREAM_DATA(data);

            if(count)
            {
                address += (1 << dataWidth);
                if(!(address & ADDRESS_WRAPPING))
                {
                    SwdHandleError();

                    // Write TAR register
                    IHIL_Write_Read_Ap(AP_TAR, &address, WRITE);
                    //Trigger new read
                    i = 0;
                    do
                    {
                        retVal = IHIL_SwdTransferData(regiser, &data, READ);
                        SwdHandleError();
                    }
                    while(++i < MAX_RETRY && retVal != SWD_ACK);
                }
            }
        }
    }

    if(count)
    { // More data expected
        retVal = 1;
    }
    else
    {
        IHIL_Write_Read_Dp(DP_RDBUFF, &data, READ);
        STREAM_DATA(data);
        SwdHandleError();

        // disable ORUNDETECT in CTRL/STAT
        IHIL_Write_Read_Dp(DP_CTRL_STAT, &data, READ);
        data &= ~0x00000001;
        IHIL_Write_Read_Dp(DP_CTRL_STAT, &data, WRITE);
    }
#endif
    return retVal;
}
