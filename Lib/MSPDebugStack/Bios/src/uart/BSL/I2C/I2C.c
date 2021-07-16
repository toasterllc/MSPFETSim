/*
* I2C.c
*
* Base class for all memory classes.
*
* Copyright (C) 2007 - 2014 Texas Instruments Incorporated - http://www.ti.com/
*
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
*
* Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the
* distribution.
*
* Neither the name of Texas Instruments Incorporated nor the names of
* its contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
* A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
* OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
// Include section

// driver
#include "I2C.h"

//private
static void I2C_writeByte(unsigned char data);
static unsigned char I2C_readByte(unsigned char ack);
static unsigned char I2C_sda(unsigned char condition);
static void I2C_delay(void);
// *************************************************************************************************
// Extern section

// *************************************************************************************************
// @fn          bmp_ps_init
// @brief       Init I2C communication
// @param       none
// @return      none
// *************************************************************************************************
void I2C_init(void)
{
    // SCL and SDA are high by default
    I2C_OUT_SCL |= I2C_SCL_PIN;
    I2C_OUT_SDA |= I2C_SDA_PIN;

    // SCL and SDA are outputs by default
    I2C_DIR_SCL |= I2C_SCL_PIN;
    I2C_DIR_SDA |= I2C_SDA_PIN;

    I2C_DIR_SCL_LS &= ~I2C_DIR_SCL_LS_PIN; // signal inverted in
    I2C_DIR_SDA_LS |= I2C_DIR_SDA_LS_PIN;

    I2C_delay();
}


// *************************************************************************************************
// @fn          I2C_sda
// @brief       Control SDA line
// @param       unsigned char condition            I2C_SEND_START, I2C_SEND_RESTART, I2C_SEND_STOP
//                                      I2C_CHECK_ACK
// @return      unsigned char                      1=ACK, 0=NACK
// *************************************************************************************************
unsigned char I2C_sda(unsigned char condition)
{
    unsigned char  sda = 0;

    if (condition == I2C_SEND_START)
    {
        I2C_SDA_OUT;      // SDA is output
        I2C_SCL_HI;
        I2C_delay();
        I2C_SDA_LO;
        I2C_delay();
        I2C_SCL_LO;       // SDA 1-0 transition while SCL=1 (will be 0)
        I2C_delay();
    }
    else if (condition == I2C_SEND_RESTART)
    {
        I2C_SDA_OUT;      // SDA is output
        I2C_SCL_LO;
        I2C_SDA_HI;
        I2C_delay();
        I2C_SCL_HI;
        I2C_delay();
        I2C_SDA_LO;
        I2C_delay();
        I2C_SCL_LO;       // SDA 1-0 while SCL = 1 (was 0)
        I2C_delay();
    }
    else if (condition == I2C_SEND_STOP)
    {
        I2C_SDA_OUT;      // SDA is output
        I2C_SDA_LO;
        I2C_delay();
        I2C_SCL_LO;
        I2C_delay();
        I2C_SCL_HI;
        I2C_delay();
        I2C_SDA_HI;       // SDA 0-1 transition while SCL=1
        I2C_delay();
    }
    else if (condition == I2C_CHECK_ACK)
    {
        I2C_SDA_IN;       // SDA is input
        I2C_SCL_LO;
        I2C_delay();
        I2C_SCL_HI;
        I2C_delay();
        sda = I2C_IN_SDA & I2C_SDA_PIN; // ACK = SDA during ack clock pulse
        //I2C_SCL_LO;

        I2C_SCL_IN; // switch SCL to input
        I2C_delay();

        // wait if Slave holds SCL low after ack
        while(!(I2C_IN_SCL & I2C_SCL_PIN))
        {
            I2C_delay();
        }

        I2C_SCL_LO ;
        I2C_delay();
        I2C_SCL_OUT;
    }

    // Return value will only be evaluated when checking device ACK
    return (sda == 0);
}

// *************************************************************************************************
// @fn          I2C_writeByte
// @brief       Clock out bits through SDA.
// @param       unsigned char data                 Byte to send
// @return      none
// *************************************************************************************************
void I2C_writeByte(unsigned char data)
{
    unsigned char i, mask;

    // Set mask byte to 10000000b
    mask = BIT0 << 7;

    I2C_SDA_OUT;                    // SDA is output

    for (i = 8; i > 0; i--)
    {
        I2C_SCL_LO;                 // SCL=0
        if ((data & mask) == mask)
        {
            I2C_SDA_HI;             // SDA=1
        }
        else
        {
            I2C_SDA_LO;             // SDA=0
        }
        mask = mask >> 1;
        I2C_delay();
        I2C_SCL_HI;                 // SCL=1
        I2C_delay();
    }

    I2C_SCL_LO;                     // SCL=0
    I2C_SDA_IN;                     // SDA is input
}

// *************************************************************************************************
// @fn          I2C_readByte
// @brief       Read bits from SDA
// @param       unsigned char ack                  1=Send ACK after read, 0=Send NACK after read
// @return      unsigned char                      Bits read
// *************************************************************************************************
unsigned char I2C_readByte(unsigned char ack)
{
    unsigned char i = 0;
    unsigned char data = 0;
    unsigned char lastSDA = 0;

    I2C_SDA_IN;
    // SDA is input
    I2C_delay();

    for (i = 0; i < 8; i++)
    {
        I2C_delay();
        I2C_SCL_LO;                 // SCL=0
        I2C_delay();
        I2C_SCL_HI;                 // SCL=0

        // Shift captured bits to left
        data = data << 1;

        // Capture new bit
        if ((I2C_IN_SDA & I2C_SDA_PIN) == I2C_SDA_PIN)
        {
            data |= BIT0;
            lastSDA = 1;
        }
        else
        {
            lastSDA = 0;
        }
    }
    if(lastSDA)
    {
        I2C_SDA_HI;
    }

    I2C_SDA_OUT;                    // SDA is output

    // 1 aditional clock phase to generate master ACK
    I2C_SCL_LO;                     // SCL=0

    if (ack == 1)
    {
        I2C_SDA_LO;                  // Send ack -> continue read
    }
    else
    {
        I2C_SDA_HI;                 // Send nack -> stop read
    }

    I2C_delay();
    I2C_SCL_HI;                     // SCL=0
    I2C_delay();
    I2C_SCL_LO;

    return (data);
}

// *************************************************************************************************
// @fn          I2C_writeData
// @brief       Write a byte via I2C
// @param       unsigned char address   device address
//              unsigned char *data     Data to write
//              unsigned char dLength   data length
// @return      unsigned char
// *************************************************************************************************
unsigned short I2C_writeData(unsigned char address, unsigned char *data, unsigned short dLength, unsigned char sendStopBit, unsigned char sendI2cAddress)
{
    volatile unsigned char success = 0;
    unsigned int i = 0;

    if(sendI2cAddress)
    {
        I2C_sda(I2C_SEND_START);                        // Generate start condition

        I2C_writeByte((address << 1) | I2C_WRITE);      // Send 7bit device address + r/w bit '0' -> write
        success = I2C_sda(I2C_CHECK_ACK);               // Check ACK from device
        if (!success)
        {
            return (0);
        }
    }
    for(i = 0; i < dLength; i++)
    {
        I2C_writeByte(data[i]);                 // Send 8bit data to BSL
        success = I2C_sda(I2C_CHECK_ACK);    // Check ACK from device
        if (!success)
        {
            I2C_sda(I2C_SEND_STOP);
            return (0);
        }
    }
    if(sendStopBit)
    {
        I2C_sda(I2C_SEND_STOP);                // Generate stop condition
    }
    return 1;
}

// *************************************************************************************************
// @fn          I2C_readData
// @brief       Read a bytes fromn I2C
// @param       unsigned char address   device address
//              unsigned char *data     Data to write
//              unsigned char dLength   data length
// @return      void
// *************************************************************************************************
unsigned short I2C_readData(unsigned char address, unsigned char *data, unsigned short *dLength, unsigned char repeatedStart)
{
    unsigned char success = 0;
    unsigned int i = 0, dataLength = 4;

    if(repeatedStart)
    {
        I2C_sda(I2C_SEND_RESTART);
    }
    else
    {
        I2C_sda(I2C_SEND_START);                    // Generate restart condition
    }

    I2C_writeByte((address << 1) | I2C_READ);       // Send 7bit device address + r/w bit '1' -> read
    I2C_delay();
    I2C_delay();
    //2 delay cycles are needed here – I2C slave needs time to process send data
    success = I2C_sda(I2C_CHECK_ACK);               // Check ACK from device
    if (!success)
    {
        return (0);
    }

    // read BSL ACk
    data[i++] = I2C_readByte(1);
    //0x80
    data[i] = I2C_readByte(1);

    if(data[i] != 0x80)
    {
        //if no answer was received
        *dLength   = 1;

        // Read lst 8bit data from register
        data[i] = I2C_readByte(0);
        I2C_sda(I2C_SEND_STOP);                         // Generate stop condition
        return (1);
    }
    i++;
    // Length low byte
    data[i++] = I2C_readByte(1);
    // Length high byte
    data[i] = I2C_readByte(1);
    dataLength = (data[i] << 8 |  data[i-1]) + OFFSET_BSL_ACK + OFFSET_BSL_HEADER + OFFSET_BSL_CRC;// 2 + 4; // +2 because of crc
    *dLength   = (data[i] << 8 |  data[i-1]) + OFFSET_BSL_ACK + OFFSET_BSL_HEADER + OFFSET_BSL_CRC; // +2 because of crc
    i++;

    // read until data lenght -1
    for(; i < dataLength-1; i++)
    {
        data[i] = I2C_readByte(1);                  // Read 8bit data from register
    }

    // send nack if last byte is read
    data[i] = I2C_readByte(0);                // Read lst 8bit data from register

    I2C_sda(I2C_SEND_STOP);                         // Generate stop condition
    return (1);
}

// *************************************************************************************************
// @fn          I2C_delay
// @brief       Delay between I2C signal edges.
// @param       none
// @return      none
// *************************************************************************************************
void I2C_delay(void)
{
  asm ("	nop");
  asm ("	nop");
  asm ("	nop");
  asm ("	nop");
  asm ("	nop");
  asm ("	nop");
  asm ("	nop");
  asm ("	nop");
}
