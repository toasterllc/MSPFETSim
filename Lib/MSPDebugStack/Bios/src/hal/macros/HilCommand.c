/**
* \ingroup MODULMACROS
*
* \file HilCommand.c
*
* \brief <FILEBRIEF>
*
*/
/*
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
#include "error_def.h"
#ifdef DB_PRINT
#include "debug.h"
#endif

/**
  HilCommand
  Low level Hil command to execute or instruction/data to shift.
  inData:  <command(32)> (<data(32)> <bits(32)>)
  outData: -
*/

typedef enum {
    HIL_CMD_RESET_JTAG_TAP,
    HIL_CMD_OPEN,
    HIL_CMD_CONNECT,
    HIL_CMD_CLOSE,
    HIL_CMD_JTAG_IR,
    HIL_CMD_JTAG_DR,
    HIL_TEST_REG,
    HIL_TEST_REG_3V,
    HIL_CMD_QUERY_JSTATE,
    HIL_CMD_WRITE_JMB,
    HIL_CMD_READ_JMB,
    HIL_CMD_CONFIGURE,
    HIL_CMD_BSL,
    HIL_CMD_FUSE_CHECK,
    HIL_CMD_JTAG_IR4,
    HIL_CMD_DPACC,
    HIL_CMD_APACC,
    HIL_CMD_MEM_AP,
    HIL_CMD_BSL_1XX_4XX,
    HIL_CMD_TCLK,
} HIL_COMMAND;

HAL_FUNCTION(_hal_HilCommand)
{
    unsigned long long shiftedOut = 0;
    unsigned long command = 0;
    unsigned long dataLow = 0;
    unsigned long dataHigh = 0;
    unsigned long long data = 0;
    unsigned long bits = 16;
    short streamResult = 0;
    short retVal = 0;

    while (streamResult == 0)
    {
        unsigned long long value = 0;

        if( STREAM_get_long(&command) < 0 )
        {
            return HALERR_NO_COMMAND;
        }
        if ( STREAM_get_long(&dataLow) < 0 )
        {
            return HALERR_NO_DATA;
        }
        if ( STREAM_get_long(&dataHigh) < 0 )
        {
            return HALERR_NO_DATA;
        }
        streamResult = STREAM_get_long(&bits);
        if (streamResult < 0)
        {
            return HALERR_NO_BIT_SIZE;
        }

        data = dataLow | ((unsigned long long)dataHigh << 32);

        if (command == HIL_CMD_JTAG_IR)
        {
            unsigned char bitCounter;

            // Force 8-bits for IR shifts
            // This is necessary because the instruction is shifted into the target MSB-first
            // by the SPI interface
            bits = 8;

            for(bitCounter = 0; bitCounter < bits; ++bitCounter)
            {
                value <<= 1;
                value |= (data >> bitCounter) & 1;
            }
        }
        else
        {
            value = data;
        }

        switch (command)
        {
        case HIL_CMD_RESET_JTAG_TAP:
            IHIL_TapReset();
            break;

        case HIL_CMD_FUSE_CHECK:
            IHIL_CheckJtagFuse();
            break;

        case HIL_CMD_OPEN:
        case HIL_CMD_CONNECT:
            IHIL_Open((unsigned char)dataLow);
            break;

        case HIL_CMD_BSL:
            IHIL_BSL_EntrySequence(1);
            break;

       case HIL_CMD_BSL_1XX_4XX:
            IHIL_BSL_EntrySequence1xx_4xx();
            break;

        case HIL_CMD_CLOSE:
            IHIL_Close();
            if (dataLow != 0)
            {
                IHIL_SetVcc(0);
            }
            break;

        case HIL_CMD_JTAG_IR:
            shiftedOut = _edt_Distinct_Methods.Instr((unsigned char)value);
            STREAM_put_long( (unsigned long)(shiftedOut &0xFFFFFFFF) );
            STREAM_put_long( (unsigned long)(shiftedOut >> 32) );
            break;

        case HIL_CMD_JTAG_IR4:
            shiftedOut = IHIL_Instr4((unsigned char)value);
            STREAM_put_long( (unsigned long)(shiftedOut &0xFFFFFFFF) );
            STREAM_put_long( (unsigned long)(shiftedOut >> 32) );
            break;

        case HIL_CMD_JTAG_DR:
            switch (bits)
            {
            case  8:
                shiftedOut = SetReg_8Bits((unsigned char)value & 0xFF);
                break;
            case 16:
                shiftedOut = SetReg_16Bits((unsigned short)value & 0xFFFF);
                break;
            case 20:
                shiftedOut = SetReg_20Bits((unsigned long)value & 0xFFFFF);
                break;
            case 32:
                shiftedOut = SetReg_32Bits((unsigned long)value & 0xFFFFFFFF);
                break;
            case 64:
                shiftedOut = SetReg_64Bits(value);
                break;
            case 35:
                shiftedOut = SetReg_35Bits(&value);
                break;
            default:
                return HALERR_INVALID_BIT_SIZE;
            }
            STREAM_put_long( (unsigned long)(shiftedOut &0xFFFFFFFF) );
            STREAM_put_long( (unsigned long)(shiftedOut >> 32) );
            break;

        case HIL_TEST_REG:
           test_reg();
           SetReg_32Bits((unsigned long)value);
           break;

        case HIL_TEST_REG_3V:
           test_reg_3V();
           SetReg_16Bits((unsigned short)value);
           break;

        case HIL_CMD_QUERY_JSTATE:
           jstate_read();
           shiftedOut = SetReg_64Bits(0x0000000000000000);
           STREAM_put_long( (unsigned long)(shiftedOut &0xFFFFFFFF) );
           STREAM_put_long( (unsigned long)(shiftedOut >> 32) );
           break;

        case HIL_CMD_TCLK:
           IHIL_Tclk((unsigned char)dataLow);
           break;
           
        case HIL_CMD_WRITE_JMB:
          {
            unsigned char jtagMailboxIn = 0;
            if(bits == 16)
            {
                jtagMailboxIn = i_WriteJmbIn((unsigned short)dataLow);
            }
            else if(bits == 32)
            {
                jtagMailboxIn = i_WriteJmbIn32((unsigned short)dataLow, (unsigned short)(dataLow >> 16));
            }
            else
            {
                return (HALERR_JTAG_MAILBOX_IN_TIMOUT);
            }

            if(jtagMailboxIn == 1)
            {
                return (HALERR_JTAG_MAILBOX_IN_TIMOUT);
            }
            break;
          }
        case HIL_CMD_READ_JMB:
            shiftedOut = i_ReadJmbOut();
            STREAM_put_long( (unsigned long)(shiftedOut &0xFFFFFFFF) );
            STREAM_put_long( (unsigned long)(shiftedOut >> 32) );
            break;

        case HIL_CMD_CONFIGURE:
            IHIL_SetProtocol(dataLow);

            {
                HilInitGetEdtDistinctFunc hilEdtDis = (HilInitGetEdtDistinctFunc)0x1880;
                hilEdtDis(&_edt_Distinct_Methods);
            }
            break;

        case HIL_CMD_DPACC:
            retVal = (IHIL_Write_Read_Dp((unsigned char)dataLow, // address
                                        &dataHigh,              // data
                                        (unsigned short)bits)   // rnw
                                         > 0) ? 0 : HALERR_DAP_NACK;
            STREAM_put_long(dataHigh);
            break;

        case HIL_CMD_APACC:
            retVal = (IHIL_Write_Read_Ap(dataLow,              // address | APSEL
                                        &dataHigh,            // data
                                        (unsigned short)bits) // rnw
                                         > 0) ? 0 : HALERR_DAP_NACK;
            STREAM_put_long(dataHigh);
            break;

        case HIL_CMD_MEM_AP:
            retVal = (IHIL_Write_Read_Mem_Ap((unsigned short)(bits >> 16),// ap_sel
                                            dataLow,                     // address
                                            &dataHigh,                   // data
                                            (unsigned short)bits)        // rnw
                                            > 0) ? 0 : HALERR_DAP_NACK;
              STREAM_put_long(dataHigh);

            break;

        default:
          retVal = HALERR_UNKNOWN_COMMAND;
        }
    }
    return retVal;
}

