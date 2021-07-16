// *************************************************************************************************
//
//      Copyright (C) 2009 Texas Instruments Incorporated - http://www.ti.com/
//
//
//        Redistribution and use in source and binary forms, with or without
//        modification, are permitted provided that the following conditions
//        are met:
//
//          Redistributions of source code must retain the above copyright
//          notice, this list of conditions and the following disclaimer.
//
//          Redistributions in binary form must reproduce the above copyright
//          notice, this list of conditions and the following disclaimer in the
//          documentation and/or other materials provided with the
//          distribution.
//
//          Neither the name of Texas Instruments Incorporated nor the names of
//          its contributors may be used to endorse or promote products derived
//          from this software without specific prior written permission.
//
//        THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//        "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//        LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//        A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//        OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//        SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//        LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//        DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//        THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//        (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//        OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *************************************************************************************************

#ifndef I2C_H_
#define I2C_H_

// *************************************************************************************************
// Include section

#include "msp430.h"

// *************************************************************************************************
// Prototypes section
void I2C_init(void);

//pulic
unsigned short I2C_writeData(unsigned char address, unsigned char *data, unsigned short dLength, unsigned char sendStopBit, unsigned char sendI2cAddress);
unsigned short I2C_readData(unsigned char address, unsigned char *data, unsigned short *dLength, unsigned char repeatedStart);

// *************************************************************************************************
// Defines section

// Port and pin resource for I2C interface
// SCL=P9.5, SDA=P8.3


// SCL config
#define I2C_IN_SCL           (P9IN)
#define I2C_OUT_SCL           (P9OUT)
#define I2C_REN_SCL           (P9REN)
#define I2C_DIR_SCL           (P9DIR)
#define I2C_DIR_SCL_LS        (P1OUT)
#define I2C_DIR_SCL_LS_PIN    (BIT6)

// SDA config
#define I2C_IN_SDA            (P8IN)
#define I2C_OUT_SDA           (P8OUT)
#define I2C_DIR_SDA           (P8DIR)
#define I2C_REN_SDA           (P8REN)
#define I2C_DIR_SDA_LS        (P1OUT)
#define I2C_DIR_SDA_LS_PIN    (BIT4)

// SCL / SDA bits
#define I2C_SCL_PIN           (BIT5)
#define I2C_SDA_PIN           (BIT3)

// I2C defines
#define I2C_WRITE         (0u)
#define I2C_READ          (1u)

#define I2C_SEND_START          (0u)
#define I2C_SEND_RESTART        (1u)
#define I2C_SEND_STOP           (2u)
#define I2C_CHECK_ACK           (3u)
#define I2C_CHECK_SCL_STRETCH   (4u)

#define I2C_8BIT_ACCESS   (0u)
#define I2C_16BIT_ACCESS  (1u)

#define I2C_SCL_HI        { I2C_OUT_SCL |=  I2C_SCL_PIN; }
#define I2C_SCL_LO        { I2C_OUT_SCL &= ~I2C_SCL_PIN; }

#define I2C_SDA_HI        { I2C_OUT_SDA |=  I2C_SDA_PIN; }
#define I2C_SDA_LO        { I2C_OUT_SDA &= ~I2C_SDA_PIN; }

#define I2C_SDA_IN        { I2C_OUT_SDA &=  ~I2C_SDA_PIN; I2C_DIR_SDA &= ~I2C_SDA_PIN; I2C_DIR_SDA_LS &= ~I2C_DIR_SDA_LS_PIN;}
#define I2C_SDA_OUT       { I2C_DIR_SDA |=  I2C_SDA_PIN; I2C_DIR_SDA_LS |= I2C_DIR_SDA_LS_PIN;}

#define I2C_SCL_IN        { I2C_DIR_SCL_LS |=  I2C_DIR_SCL_LS_PIN; I2C_DIR_SCL &= ~I2C_SCL_PIN;}
#define I2C_SCL_OUT       { I2C_DIR_SCL |=  I2C_SCL_PIN; I2C_DIR_SCL_LS &= ~I2C_DIR_SCL_LS_PIN;}

#define I2C_START          1
#define I2C_STOP           1
#define I2C_NO_STOP        0
#define I2C_NO_START       0
#define I2C_REPEATED_START 1


#define I2C_SLAVE_ADDRESS 0x48
#define OFFSET_BSL_HEADER 3
#define OFFSET_BSL_CRC 2
#define OFFSET_BSL_ACK 1

#endif
