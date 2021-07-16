/*
 * FetDcdc.c
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

#include "msp430.h"
#include "FetDcdc.h"
#include "hw_compiler_specific.h"
#include "../fw/fet/FetVersion.h"

#define DCDC_SIGNATURE     0xABBAABBAul

#define CALIB_COUNT 10
#define DELAYCOUNT 16384

const unsigned long dcdc_Signature_ @ "DCDCSIGNATURE" = DCDC_SIGNATURE;
#pragma required = dcdc_Signature_

const unsigned short dcdc_LayerVersion_ @ "DCDCLAYERVERSION" = DCDC_LAYER_VERSION;
#pragma required = dcdc_LayerVersion_

const unsigned short dcdc_LayerVersionCMP_ @ "DCDCLAYERVERSIONCMP" = DCDC_LAYER_VERSION_CMP;
#pragma required = dcdc_LayerVersionCMP_

DCDC_INFOS_t dcdcInfos_;
calibrationValues_t savedCalibrationValues;
const unsigned short countScV = 4;

unsigned short dcdcLayerVersion =0;

unsigned char* _UCBxCTL1 = 0 ;
unsigned char* _UCBxIFG   = 0 ;
unsigned char* _UCBxTXBUF  = 0 ;
unsigned char* _UCBxRXBUF = 0 ;
unsigned short _fetType = 0;

void dcdc_Init(DCDC_INFOS_t* dcdcInfos_Pointer)
{
    // map funciotn pointers in FET dcdc layer
    dcdcInfos_.dcdcCalibrate                = dcdc_Calibrate;
    dcdcInfos_.getSubMcuVersion             = dcdc_getSubMcuVersion;
    dcdcInfos_.getLayerVersion              = dcdc_getLayerVersion;
    dcdcInfos_.dcdcPowerDown                = dcdc_PowerDown;
    dcdcInfos_.dcdcSetVcc                   = dcdc_SetVcc;
    dcdcInfos_.dcdcRestart                  = dcdc_Restart;
    dcdcInfos_.dcdc_getCalibrationValues    = dcdc_getCalibrationValues;
    dcdcInfos_.getLayerVersionCmp           = dcdc_getLayerVersionCmp;
    // now copy getLayerVersion and retrun it to uper core layer
    *dcdcInfos_Pointer =  dcdcInfos_;

    _UCBxCTL1 = 0 ;
    _UCBxIFG   = 0  ;
    _UCBxTXBUF  = 0 ;
    _UCBxRXBUF = 0 ;
    _fetType = 0 ;
}

short dcdc_Restart(unsigned short fetType_)
{
    _fetType = fetType_;
    if(fetType_ == eZ_FET_WITH_DCDC || fetType_ == eZ_FET_WITH_DCDC_NO_FLOWCTL || fetType_ == eZ_FET_WITH_DCDC_V2x) // this is the eZ-FET tool id
    {
        P6DIR &= ~BIT4;

        P6DIR |= BIT5;
        P6OUT &= ~BIT5;

         // TEST pin sub mcu
         P6DIR &= ~BIT7;
         P6OUT &= ~BIT7;
         //togle RST pin to restart sub mcu
         P6DIR |= BIT6;
         __delay_cycles(600000);
         P6OUT &= ~BIT6;
         __delay_cycles(600000);
         P6OUT |= BIT6;
         P6DIR &= ~BIT6;
        // Port3
        //  P3.0 <-> HOST_SDA
        //  P3.1 -> HOST_SCL
        //  P3.2 -> N/C
        //  P3.3 -> UART_RXD N/C
        //  P3.4 <- UART_TXD N/C
        //  P3.5 -> n/c
        //  P3.6 -> n/c
        //  P3.7 -> n/c
        P3SEL = (BIT0+BIT1);
        // Configure I2C

        _UCBxCTL1   = (unsigned char*)&UCB0CTL1;
        _UCBxIFG    = (unsigned char*)&UCB0IFG;
        _UCBxTXBUF  = (unsigned char*)&UCB0TXBUF;
        _UCBxRXBUF  = (unsigned char*)&UCB0RXBUF;

        *_UCBxCTL1 |= UCSWRST;                      // Enable SW reset
        UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;     // I2C Master, synchronous mode
        *_UCBxCTL1 = UCSSEL_2 + UCSWRST;            // Use SMCLK, keep SW reset
        UCB0BR0 = 0xc8;//25;//12;                        // fSCL = SMCLK/25 = ~100kHz
        UCB0BR1 = 0;
        UCB0I2CSA = 0x48;                         // Slave Address is 048h
        *_UCBxCTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation
        __delay_cycles(600000);

        savedCalibrationValues.vcc = 0;
        savedCalibrationValues.valid = 0;
        return fetType_;
    }
    else if(fetType_ == eZ_FET_NO_DCDC) // else it the MSP-FET
    {
        return eZ_FET_NO_DCDC;
    }
    else if(fetType_ == MSP_FET_WITH_DCDC || fetType_ == MSP_FET_WITH_DCDC_V2x)
    {
        #ifdef MSP_FET
        // Port7
        //  P7.0 -> n/c
        //  P7.1 -> n/c
        //  P7.2 <- XT2IN
        //  P7.3 -> XT2OUT
        //  P7.4 -> DCDC_TEST
        //  P7.5 -> DCDC_RST (0 = reset)
        //  P7.6 -> VCC_DT_REF
        //  P7.7 -> VCC_DCDC_REF
        // TEST pin sub mcu low

        P7DIR &= ~BIT4;
        P7OUT &= ~BIT4;
        //togle RST pin to restart sub mcu
        P7DIR |= BIT5;
        __delay_cycles(600000);
        P7OUT &= ~BIT5;
        __delay_cycles(600000);
        P7OUT |= BIT5;
        P7DIR &= ~BIT5;
        // Port8
        //  P8.0 -> VCC_DT2TRGT_CTRL (control signal to switches to provide debug signals to target via JTAG.x)
        //  P8.1 -> UART_?
        //  P8.2 -> UART_TXD
        //  P8.3 <- UART_RXD
        //  P8.4 <- DCDC_IO0
        //  P8.5 -> HOST_SDA
        //  P8.6 -> HOST_SCL
        //  P8.7 -> VCC_SUPPLY2TRGT_CTRL (DCDC VCC to target VCC)

        P8DIR |= (BIT5+BIT6);                      // set pins to output direction
        P8SEL |= (BIT5+BIT6);                      // use USCI for i2c communication

        _UCBxCTL1   = (unsigned char*)&UCB1CTL1;
        _UCBxIFG    = (unsigned char*)&UCB1IFG;
        _UCBxTXBUF  = (unsigned char*)&UCB1TXBUF;
        _UCBxRXBUF  = (unsigned char*)&UCB1RXBUF;

        *_UCBxCTL1 |= UCSWRST;                      // Enable SW reset
        UCB1CTL0 = UCMST + UCMODE_3 + UCSYNC;     // I2C Master, synchronous mode
        *_UCBxCTL1 = UCSSEL_2 + UCSWRST;            // Use SMCLK, keep SW reset
        UCB1BR0 = 20;                             // fSCL = SMCLK/20 = ~100kHz
        UCB1BR1 = 0;
        UCB1I2CSA = 0x48;                         // Slave Address is 048h
       * _UCBxCTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation
        __delay_cycles(600000);

        //Configure port mapper and P2 for current measuring
        P2DIR &= ~BIT0;
        P2SEL |= BIT0;
        _DINT_FET();
        PMAPKEYID = PMAPKEY;
        PMAPCTL |= PMAPRECFG;
        P2MAP0 = PM_TB0CLK;
        PMAPKEYID = 0;
        _EINT_FET();

        dcdc_SetVcc(3300);

        savedCalibrationValues.vcc = 0;
        savedCalibrationValues.valid = 0;

       #endif
       return fetType_;
    }
    return -1;
}

#pragma optimize = low
short dcdc_SetVcc(unsigned short vcc)
{
    short success = dcdc_Send(CMD_CONFIGURE, vcc);
    if(!success)
    {
        return -1;
    }
    return 0;
}

/* TBD make ADC and timer config configurable via pointers like or I2c communication */
void dcdc_RunCalibrate(unsigned long *ticks, unsigned long *time, unsigned short count)
{
    unsigned short i0 = 0, i1 = 0;
    unsigned short t0 = 0, t1 = 0;
    unsigned short i = 0;

    *ticks = 0;
    *time = 0;

#ifdef MSP_FET
    #define PULSES TB0R
#else
    #define PULSES TA2R
#endif

    if(_fetType == eZ_FET_WITH_DCDC_NO_FLOWCTL || _fetType == eZ_FET_WITH_DCDC ||_fetType == MSP_FET_WITH_DCDC
       || _fetType == MSP_FET_WITH_DCDC_V2x ||  _fetType == eZ_FET_WITH_DCDC_V2x)
    {
        _DINT_FET();

        for(i = 0; i < count; ++i)
        {
            TA0CTL |= TACLR + MC__CONTINOUS + TAIE; // START the timer in free-running mode

            i0 = PULSES;
            t0 = TA0R;

            // Sample VCCout
            ADC12CTL0  &= ~ADC12ENC;              // Disable conversion, write controls
            ADC12MCTL0  = ADC12SREF_1 + 1;        // select Vref and analog channel Ax
            ADC12CTL0  |= ADC12ENC;               // Enable conversions

            ADC12IFG &= ~BIT0;
            ADC12CTL0  |= ADC12SC;                // Start conversions
            while ((ADC12IFG & BIT0) == 0);       // wait until conversion is done
            ADC12IFG &= ~BIT0;

            // Wait for some time
            for(volatile unsigned long j = 0; j < DELAYCOUNT; ++j);

            i1 = PULSES;
            t1 = TA0R;

            *time += (t1 - t0);
            *ticks += (i1 - i0);
        }
        _EINT_FET();
    }
}
#pragma optimize = low
void dcdc_getCalibrationValues(unsigned short vcc, unsigned short resistor,  unsigned short resCount, unsigned long *ticks, unsigned long *time)
{
    unsigned short currentResCount = 0;

    for(currentResCount = 0; currentResCount < resCount; currentResCount++)
    {
        if(savedCalibrationValues.resValues[currentResCount].resistor == resistor)
        {
            *time = savedCalibrationValues.resValues[currentResCount].time;
            *ticks = savedCalibrationValues.resValues[currentResCount].ticks;
        }
    }
}
#pragma optimize = low
short dcdc_Calibrate(unsigned short resistors[5], unsigned short resCount, unsigned short vcc)
{
    short success = 0;
    unsigned int count = CALIB_COUNT;
    unsigned long internalTicks = 0;
    unsigned long internalTime = 0;
    unsigned short currentResCount = 0;

    if((savedCalibrationValues.vcc == vcc) && (savedCalibrationValues.valid ==  1))
    {
        return 0;
    }

    savedCalibrationValues.vcc = vcc;
    savedCalibrationValues.valid = 0;

    for(currentResCount = 0; currentResCount < resCount; currentResCount++)
    {
        unsigned short resistor = resistors[currentResCount];

        if (resistor == 0)
        {
            count = CALIB_COUNT*5;
        }

        if (resistor == 4)
        {
            count = CALIB_COUNT*2;
        }

        //  switch off Overcurrent detection
#ifdef MSP_FET
        P5OUT |= BIT7;
#else
        P6OUT |= BIT5;
#endif

        success = dcdc_Send(CMD_CALLOAD, resistor);

        if(!success)
        {
            return -1;
        }
        // Wait for some time
        for(volatile unsigned long j = 0; j < DELAYCOUNT; ++j);
        for(volatile unsigned long j = 0; j < DELAYCOUNT; ++j);
        for(volatile unsigned long j = 0; j < DELAYCOUNT; ++j);
        for(volatile unsigned long j = 0; j < DELAYCOUNT; ++j);

        dcdc_RunCalibrate(&internalTicks, &internalTime, count);

        savedCalibrationValues.resValues[currentResCount].ticks = internalTicks;
        savedCalibrationValues.resValues[currentResCount].time = internalTime;
        savedCalibrationValues.resValues[currentResCount].resistor = resistor;

        // Reset the calibration resistor
        success = dcdc_Send(CMD_CALLOAD, 0);
        for(volatile unsigned long j = 0; j < DELAYCOUNT; ++j);

        //  switch on Overcurrent detection
#ifdef MSP_FET
        P5OUT &= ~BIT7;
#else
        P6OUT &= ~BIT5;
#endif
    }
    if(!success)
    {
        savedCalibrationValues.valid = 0;
        return -1;
    }

    savedCalibrationValues.valid = 1;
    return 0;
}
#pragma optimize = low
short dcdc_getSubMcuVersion()
{
    unsigned int version = 0;

    //togle RST pin to restart sub mcu
    #ifdef MSP_FET
    P7DIR |= BIT5;
    __delay_cycles(600000);
    P7OUT &= ~BIT5;
    __delay_cycles(600000);
    P7OUT |= BIT5;
    P7DIR &= ~BIT5;
    #endif

    dcdc_Receive(&version);
    return version;
}

short dcdc_getLayerVersion()
{
    return DCDC_LAYER_VERSION;
}

short dcdc_getLayerVersionCmp()
{
    return DCDC_LAYER_VERSION_CMP;
}

short dcdc_PowerDown()
{
    if(!dcdc_Send(CMD_POWERDOWN, SDIO_POWERDOWN_KEY))
    {
        return -1;
    }
    return 0;
}

#pragma optimize = low
// Generic send function
short dcdc_Send(unsigned int cmd, unsigned int data)
{
    unsigned char TxData[3];
    unsigned int timeout;

    TxData[0] = cmd;
    TxData[1] = (unsigned char)(data>>8);       // High byte
    TxData[2] = data&0xFF;                      // Low bytes

    // Send start condition and slave address
    *_UCBxCTL1 |= UCTR + UCTXSTT;                 // Master Tx and Start Condition
    __delay_cycles(5000);
    if(*_UCBxIFG & UCNACKIFG)
    {
        // Nack received
        return (0);
    }

    // Send byte 0
    *_UCBxTXBUF = TxData[0];
    __delay_cycles(5000);
    if(*_UCBxIFG & UCNACKIFG)
    {
        // Nack received
        return (0);
    }

    // Send byte 1
    *_UCBxTXBUF = TxData[1];
    __delay_cycles(5000);
    if(*_UCBxIFG & UCNACKIFG)
    {
        // Nack received
        return (0);
    }

    // Send byte 3
    *_UCBxTXBUF = TxData[2];

    //Poll for transmit interrupt flag.
    timeout = 10000;
    while ((!(*_UCBxIFG & UCTXIFG)) && (--timeout>0));
    if (timeout==0)
    {
        // Error
        return (0);
    }

    //Send stop condition.
    *_UCBxCTL1 |= UCTXSTP;

    return (1);

}

#pragma optimize = low
// Generic receive function
short dcdc_Receive(unsigned int *data_read)
{
    unsigned char RxBuffer[2];
    unsigned int timeout;
    unsigned int val;

    timeout = 10000;
    while ((*_UCBxCTL1 & UCTXSTP) && (--timeout>0));             // Ensure stop condition got sent
    if (timeout==0)
    {
        // Slave did not send in time
        return (0);
    }

    *_UCBxCTL1 &= ~UCTR;                      // Master receives
    *_UCBxCTL1 |= UCTXSTT;	                  // Start transmit enable

    // Receive byte 0
    timeout = 10000;
    while ((((*_UCBxIFG&UCRXIFG)==0)) && (--timeout>0));
    if (timeout==0)
    {
        // Slave did not send in time
        return (0);
    }
    *_UCBxIFG &= ~UCRXIFG;
    RxBuffer[0] = (unsigned char)*_UCBxRXBUF;	// Read RX buffer byte

    // Receive byte 1
    *_UCBxCTL1 |= UCTXSTP;            // Generate I2C stop condition
    timeout = 10000;
    while ((*_UCBxCTL1& UCTXSTP) && (--timeout>0));      // Wait for Stop
    if (timeout==0)
    {
        // Slave did not send in time
        return (0);
    }
    timeout = 10000;
    while ((((*_UCBxIFG&UCRXIFG)==0)) && (--timeout>0));  // Wait for byte RX
    if (timeout==0)
    {
        // Slave did not send in time
        return (0);
    }
    RxBuffer[1] = (unsigned char)*_UCBxRXBUF;	// Read RX buffer byte

    val  = ((unsigned int) RxBuffer[0])<<8 ;
    val += ((unsigned int) RxBuffer[1]);
    *data_read=val;

    return (1);
}
