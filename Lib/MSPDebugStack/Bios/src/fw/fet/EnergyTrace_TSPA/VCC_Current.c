/*
 * VCC_Current.c
 *
 * Copyright (c) 2007 - 2013 Texas Instruments Incorporated - http://www.ti.com/
 *
 * All rights reserved not granted herein.
 * Limited License.
 *
 * Texas Instruments Incorporated grants a world-wide, royalty-free,
 * non-exclusive license under copyrights and patents it now or hereafter
 * owns or controls to make, have made, use, import, offer to sell and sell ("Utilize")
 * this software subject to the terms herein.  With respect to the foregoing patent
 * license, such license is granted  solely to the extent that any such patent is necessary
 * to Utilize the software alone.  The patent license shall not apply to any combinations which
 * include this software, other than combinations with devices manufactured by or for TI (“TI Devices”).
 * No hardware patent is licensed hereunder.
 *
 * Redistributions must preserve existing copyright notices and reproduce this license (including the
 * above copyright notice and the disclaimer and (if applicable) source code license limitations below)
 * in the documentation and/or other materials provided with the distribution
 *
 * Redistribution and use in binary form, without modification, are permitted provided that the following
 * conditions are met:
 *
 *	* No reverse engineering, decompilation, or disassembly of this software is permitted with respect to any
 *     software provided in binary form.
 *	* any redistribution and use are licensed by TI for use only with TI Devices.
 *	* Nothing shall obligate TI to provide you with source code for the software licensed and provided to you in object code.
 *
 * If software source code is provided to you, modification and redistribution of the source code are permitted
 * provided that the following conditions are met:
 *
 *   * any redistribution and use of the source code, including any resulting derivative works, are licensed by
 *     TI for use only with TI Devices.
 *   * any redistribution and use of any object code compiled from the source code and any resulting derivative
 *     works, are licensed by TI for use only with TI Devices.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of its suppliers may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * DISCLAIMER.
 *
 * THIS SOFTWARE IS PROVIDED BY TI AND TI’S LICENSORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL TI AND TI’S LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*----------------------------------------------------------------------------+
|                                                                             |
| Texas Instruments
|                                                                             |
| MSP430G2452 software controlled DCDC converter
|                                                                             |
+-----------------------------------------------------------------------------+
|  Source: VCC_Current.h, File Version 1.0 2012/07/12                         |
|  Author: ANBR                                                               |
|                                                                             |
|  WHO          WHEN         WHAT                                             |
|  ---          ----------   ------------------------------------------------ |
|  ANBR         2012/07/12   born                                             |
+-----------------------------------------------------------------------------+
| Pin out table
+-----------------------------------------------------------------------------+
 MSP-FET 0.9 & eZ-FET 0.92 hardware

 Pin	MSP430G2xxx	Signal	        Note
 1	DVCC
 2	P1.0/CA0	    A_VCC_OUT	Target voltage feedback (divided by 2)
 3	P1.1	        DCDC_CAL1   Calibration resistor 1
 4	P1.2/TA0.1      DCDC_PULSE	FET driver output
 5	P1.3	        DCDC_IO0	Status bit0 (output, 1=overcurrent detected)
 6	P1.4	        DCDC_IO1	Status bit1 (input, 1=overcurrent shutdown enable)
 7	P1.5/CA5        A_VCC_REF	Reference voltage (divided by 2)
 8	P1.6/SCL        HOST_SCL	I2C clock
 9	P1.7/SDA        HOST_SDA	I2C data
 10	RST	            DCDC_RST	SBWTDIO i/o (external pull-up resistor)
 11	TEST	        DCDC_TEST	SBWTCK in (pull-down in host MCU)
 12	P2.7	        DCDC_CAL2   Calibration resistor 2
 13	P2.6	        DCDC_CAL0   Calibration resistor 0
 14	DVSS
+-----------------------------------------------------------------------------+
Version history

Version 1: Initial release
Version 2: Changed DCDC_CAL2 signal to output "1" on eZ-FET when inactive
           (bug fix for P2.6 being 2.83V when it should be pulled to VCCOUT)
Version 3: Added pwm on duty and overcurrent table
            Removed SDIO interface, timer-based PWM, eZ-FET ALPHA2 initialisation
            Changed variable names and function headers to meet coding guidelines
Version 4: Modified drive strength and overcurrent table
Version 5: Added overcurrent calibration when configuring output voltage
            Balanced DCDC regulation loop paths
Version 8: Reduced drive strength at 3600mV and increased regulation frequency
           Overcurrent detection VCC=3.6V: VBUS=5V ~90mA, VBUS=4.5V ~60mA
Version 9: Added highres PWM mode. Commented LOW_USB_FIX lines.
           Moved command handler to main loop. Added overcurrent shutdown after 15msec.
           Every ca. 300msec s/w tries to restart voltage regulation.
Version 10: Changed overcurrent shutdown pin function to enable function.
            Removed calibration for overcurrent detection. It will now trigger
            at ca. 70mA (VUSB=4.5V) - 100mA (VUSB=5V) and will turn off voltage
            regulation only if DCDC_IO1 input is high.
+----------------------------------------------------------------------------*/

#include "msp430.h"

// Software configuration
#define USE_OVERCURRENT_PROTECTION
#define OVERCURRENT_SHUTDOWN

// Sanity checks
#ifdef MSP_FET
  #ifdef EZ_FET
    #error "Wrong number of #defines"
  #endif
#endif

#ifndef MSP_FET
  #ifndef EZ_FET
    #error "Wrong number of #defines"
  #endif
#endif

// Software version (each version increments by 1)
#if defined(EZ_FET_V14) || defined(MSP_FET_V12)
    #define VERSION                     17
#endif

#ifdef EZ_FET_V2X
    #define VERSION                     80
#endif


#ifdef MSP_FET_V2X
    #define VERSION                     52
#endif


// Host commands
#define CMD_CONFIGURE               0x01
#define CMD_SETPWM                  0x02
#define CMD_CALLOAD                 0x03
#define CMD_POWERDOWN               0x04

// I2C communication
#define I2C_SLAVE_ADDRESS           0x90
#define I2C_PACKET_BYTES            3
#define I2C_CMD_BYTE                0
#define I2C_DATA_HI_BYTE            1
#define I2C_DATA_LO_BYTE            2

// Status output to Host MCU
#define STATUS_OC_OFF         { P1OUT &= ~BIT3; }
#define STATUS_OC_ON          { P1OUT |= BIT3; }
#define OC_SHUTDOWN_ENABLE    ((P1IN & BIT4)==0)

// pwmPowerdown key
#define POWERDOWN_KEY               0x5a5a

// Constants
const int fwVersion @0x1000 = VERSION;
const char i2cSlaveAddress = 0x90;                   // Address is 0x90<<1 for R/W

// Global variables
__no_init unsigned int __regvar pwmTimerReg @ 4;
#ifdef USE_OVERCURRENT_PROTECTION
__no_init unsigned int __regvar pwmOffCount @ 5;
//__no_init unsigned int pwmTimeOut;
__no_init unsigned char pwmOcEvents;
#define OVERCURRENT_TIMEOUT       1
#define OVERCURRENT_EVENT_LIMIT   3
#endif
__no_init unsigned char i2cReceived[I2C_PACKET_BYTES];
__no_init unsigned int i2cState;                     // State variable
__no_init unsigned i2cRxIndex;                       // Pointer to i2cReceived[] while receiving
__no_init int i2cCmd;                                // Received 8-bit command
__no_init unsigned int i2cData;                      // Received 16-bit data

// Startup voltages
#define EZ_FET_VOLTAGE      3600
#define MSP_FET_VOLTAGE     3000

// Configuration settings, in 100mV steps
// Format: pwm on duty, overcurrent limit
#define PWM_ARRAY_VCC_MIN   1200
#define PWM_ARRAY_VCC_MAX   3600

#if defined(EZ_FET_V14) || defined(MSP_FET_V12)
const unsigned char pwmConfigTable[] = {  3, 2,     // 1200mV
                                          3, 2,     // 1300mV
                                          3, 2,     // 1400mV
                                          3, 2,     // 1500mV
                                          4, 2,     // 1600mV
                                          4, 2,     // 1700mV
                                          5, 2,     // 1800mV
                                          5, 2,     // 1900mV
                                          5, 2,     // 2000mV
                                          5, 2,     // 2100mV
                                          5, 2,     // 2200mV
                                          5, 2,     // 2300mV
                                          6, 2,     // 2400mV
                                          6, 3,     // 2500mV
                                          6, 3,     // 2600mV
                                          6, 3,     // 2700mV
                                          7, 3,     // 2800mV
                                          7, 3,     // 2900mV
                                          7, 3,     // 3000mV
                                          8, 3,     // 3100mV
                                          8, 3,     // 3200mV
                                          8, 3,     // 3300mV
                                          8, 3,     // 3400mV
                                          9, 3,     // 3500mV
                                          9, 3 };   // 3600mV
#endif


#ifdef EZ_FET_V2X
const unsigned char pwmConfigTable[] = {  3, 2,     // 1200mV
                                          3, 2,     // 1300mV
                                          3, 2,     // 1400mV
                                          3, 2,     // 1500mV
                                          4, 2,     // 1600mV
                                          4, 2,     // 1700mV
                                          5, 2,     // 1800mV
                                          5, 2,     // 1900mV
                                          5, 2,     // 2000mV
                                          5, 2,     // 2100mV
                                          5, 2,     // 2200mV
                                          5, 2,     // 2300mV
                                          6, 2,     // 2400mV
                                          6, 3,     // 2500mV
                                          6, 3,     // 2600mV
                                          6, 3,     // 2700mV
                                          7, 3,     // 2800mV
                                          7, 3,     // 2900mV
                                          7, 3,     // 3000mV
                                          8, 3,     // 3100mV
                                          9, 3,     // 3200mV
                                          10, 3,     // 3300mV
                                          11, 3,     // 3400mV
                                          11, 3,     // 3500mV
                                          11, 3 };   // 3600mV
#endif

#ifdef MSP_FET_V2X
const unsigned char pwmConfigTable[] = {  3, 2,  // 1200mV
                                          3, 2,     // 1300mV
                                          3, 2,     // 1400mV
                                          3, 2,     // 1500mV
                                          4, 2,     // 1600mV
                                          4, 2,     // 1700mV
                                          6, 3,     // 1800mV
                                          6, 3,     // 1900mV
                                          6, 3,     // 2000mV
                                          6, 3,     // 2100mV
                                          7, 3,     // 2200mV
                                          7, 3,     // 2300mV
                                          7, 3,     // 2400mV
                                          7, 3,     // 2500mV
                                          7, 3,     // 2600mV
                                          7, 3,     // 2700mV
                                          8, 3,     // 2800mV
                                          8, 3,     // 2900mV
                                          8, 3,     // 3000mV
                                          9, 3,     // 3100mV
                                          9, 3,     // 3200mV
                                          10, 3,     // 3300mV
                                          11, 3,     // 3400mV
                                          11, 3,     // 3500mV
                                          11, 3 };   // 3600mV
#endif

// Function prototypes
void calibrationSetLoad(int i2cData);

/**
* \brief pwmConfigure
*
* PWM regulation and overcurrent detection requires voltage-dependend
* parameters.
*
* The pwmConfigure function accesses a configuration table
* to set the voltage-dependent PWM on duty.
*
* The calibration resistor #0 is then used to finetune the overcurrent
* detection threshold. Target power switch needs to be disconnected during
* this calibration!
*
* @param int i2cData [IN] Output voltage in mV (1200 - 3600, 100mV steps)
*
* @return void
*/
int voltage;
unsigned char highresMode;

void pwmConfigure(int i2cData)
{
  unsigned int index = 0;
  voltage = i2cData&0x7FFF;                 // Mask MSB
  highresMode = ((i2cData>>8)>>8)&0x01;     // Extract MSB

  if ((voltage >= PWM_ARRAY_VCC_MIN) && (voltage <= PWM_ARRAY_VCC_MAX))
  {
    // Find PWM on duty in configuration table
    index = voltage - PWM_ARRAY_VCC_MIN;
    index = index / 100;
    TA0CCR0 = pwmConfigTable[index*2 + highresMode] + 1;
  }
}

/**
* \brief pwmSetOnTime
*
* Manually override the PWM on duty setting done by
* pwmConfigure()
*
* @param int i2cData [IN] 0 = PWM off, 1..10 = PWM on duty
*
* @return void
*/
void pwmSetOnTime(int i2cData)
{
  if (i2cData == 0)
  {
    pwmTimerReg = TACLR;                    // Turn off PWM
  }
  else if (i2cData < 10)
  {
    TA0CCR0 = i2cData;
    pwmTimerReg = TASSEL_2 + MC_2 + TACLR;  // using register is faster than accessing a Flash-stored value
  }
}


/**
* \brief calibrationSetLoad
*
* Connect one or more calibration resistors to the output voltage
*
* @param int i2cData [IN] 0 = disconnect all resistors
*                         1 = connect resistor #0
*                         2 = connect resistor #1
*                         4 = connect resistor #2
*
* @return void
*/
#ifdef EZ_FET
void calibrationSetLoad(int i2cData)
{
  // eZ-FET: Set pin to output "0"
  P1OUT &= ~BIT1;
  P2OUT &= ~(BIT6+BIT7);

  // Calibration resistor 0
  if (i2cData & BIT0) { P2DIR |= BIT6;  }
  else	            { P2DIR &= ~BIT6; }

  // Calibration resistor 1
  if (i2cData & BIT1) { P1DIR |= BIT1;  }
  else              { P1DIR &= ~BIT1; }

  // Calibration resistor 2
  if (i2cData & BIT2) { P2DIR |= BIT7;  }
  else	            { P2DIR &= ~BIT7; }
}
#endif
#ifdef MSP_FET
void calibrationSetLoad(int i2cData)
{
    // MSP-FET: Turn on calibration resistor switch
    P1DIR |= BIT1;
    P2DIR |= BIT6+BIT7;

    // Calibration resistor 0
    if (i2cData & BIT0) { P2OUT |= BIT6; }
    else	              { P2OUT &= ~BIT6; }

    // Calibration resistor 1
    if (i2cData & BIT1) { P1OUT |= BIT1;  }
    else              { P1OUT &= ~BIT1; }

    // Calibration resistor 2
    if (i2cData & BIT2) { P2OUT |= BIT7;  }
    else              { P2OUT &= ~BIT7; }
}
#endif


/**
* \brief pwmPowerdown
*
* Go to LPM3 and wait there until I2C communication
* resumes
*
* @param int i2cData [IN] Powerdown key
*
* @return void
*/
void pwmPowerdown(int i2cData)
{
    if (i2cData == POWERDOWN_KEY)
    {
        // To LPM - exit through next communication packet
#ifdef USE_OVERCURRENT_PROTECTION
        IE1 &= ~WDTIE;
#endif
        _BIS_SR(LPM3_bits+GIE);
        __no_operation();
#ifdef USE_OVERCURRENT_PROTECTION
        IE1 |= WDTIE;
#endif
    }
}


/**
* \brief commandHandler
*
* Evaluates command received through I2C
*
* @param int i2cData [IN] Data received through I2C
*
* @return void
*/
void commandHandler(int i2cData)
{
    switch(i2cCmd)
    {
        // pwmConfigure PWM
        case CMD_CONFIGURE:     pwmConfigure(i2cData);
                                break;
        // Set PWM manually
        case CMD_SETPWM:		pwmSetOnTime(i2cData);
                                break;
        // Enable calibration loads
        case CMD_CALLOAD:		calibrationSetLoad(i2cData);
                                break;
        // Powerdown device
        case CMD_POWERDOWN:		pwmPowerdown(i2cData);
                                break;
    }
    // Void command
    i2cCmd = -1;
}


/**
* \brief main
*
* Initialises system, sets initial PWM parameters
* starts voltage regulation
*
* @param none
*
* @return none
*/
int main( void )
{
    // Stop watchdog timer to prevent time out reset
    WDTCTL = WDTPW + WDTHOLD;

#if defined MSP_FET
    // Hardware initialisation

    // PORT1
    // P1.0 = A_VCC_OUT
    // P1.1 = DCDC_CAL1
    // P1.2 = DCDC_PULSE
    // P1.3 = DCDC_IO0
    // P1.4 = DCDC_IO1
    // P1.5 = A_VCCPOD
    // P1.6 = HOST_SCL
    // P1.7 = HOST_SDA
    P1SEL |= BIT2;                       // DCDC_PULSE = TA0.1 output
    P1OUT &= ~(BIT1+BIT3);               // DCDC_CAL0, DCDC_IO0 are "0"
    P1DIR = BIT1+BIT2+BIT3;              // DCDC_PULSE, DCDC_IO0 are output

    // PORT2
    // P2.6 = DCDC_CAL1
    // P2.7 = DCDC_CAL2
    P2SEL  &= ~(BIT6+BIT7);               // XT1 pins to GPIO functionality
    P2SEL2 &= ~(BIT6+BIT7);
    P2OUT = 0;
    P2DIR   = BIT0+BIT1+BIT2+BIT3+BIT4+BIT5+BIT6+BIT7;
#endif
#ifdef EZ_FET
    // Hardware initialisation

    // PORT1
    // P1.0 = A_VCC_OUT
    // P1.1 = DCDC_CAL1
    // P1.2 = DCDC_PULSE
    // P1.3 = DCDC_IO0
    // P1.4 = DCDC_IO1
    // P1.5 = A_VCCPOD
    // P1.6 = HOST_SCL
    // P1.7 = HOST_SDA
    P1SEL |= BIT2;                       // DCDC_PULSE = TA0.1 output
    P1OUT &= ~(BIT1+BIT3);               // DCDC_CAL0, DCDC_IO0 are "0"
    P1DIR = BIT2+BIT3;                   // DCDC_PULSE, DCDC_IO0, DCDC_IO1 are output
    P1REN = 0;

    // PORT2
    // P2.6 = DCDC_CAL1
    // P2.7 = DCDC_CAL2
    P2SEL  &= ~(BIT6+BIT7);               // XT1 pins to GPIO functionality
    P2SEL2 &= ~(BIT6+BIT7);
    P2OUT  &= ~(BIT6+BIT7);               // DCDC_CAL0, DCDC_CAL1 are "0"
    P2REN   = 0;
    P2DIR   = BIT0+BIT1+BIT2+BIT3+BIT4+BIT5;
#endif

    // P1.0 & P1.5 = comparator inputs for regulation loop
    CAPD   = CAPD0 + CAPD5;
    CACTL2 = P2CA0 + P2CA3 + P2CA1;
    CACTL1 = CAON;

    // Set DCO to 16MHz
    DCOCTL = CALDCO_16MHZ;
    BCSCTL1 = CALBC1_16MHZ;

    // Setup timer (set at TA0CCR1, clear at TA0CCR0)
    TA0CCTL1  = OUTMOD_3;
    TA0CCR0   = 8;
    TA0CCR1   = 2;
    pwmTimerReg = TASSEL_2 + MC_2 + TACLR;  // using register is faster than accessing a Flash-stored value

#ifdef USE_OVERCURRENT_PROTECTION
    // Start Watchdog timer to periodically check off count
    // Watchdog timer runs off VLOCLK (12kHz), results in an ISR every 5.3msec
    pwmOffCount = 0;
    BCSCTL3 |= LFXT1S_2;
    WDTCTL = WDTPW + WDTNMI + WDTTMSEL + WDTCNTCL + WDTSSEL + WDTIS1 + WDTIS0;
    IE1 |= WDTIE;
#endif

#ifdef OVERCURRENT_SHUTDOWN
    pwmOcEvents = 0;
#endif

    // Enable I2C interface
    i2cCmd = -1;
    i2cState = 0;
    i2cRxIndex  = 0;
    USICTL0  = USIPE6+USIPE7+USISWRST;    // Port & USI mode setup
    USICTL1  = USII2C+USIIE+USISTTIE;     // Enable I2C mode & USI interrupts
    USICKCTL = USIDIV_7+USISSEL_3+USICKPL;// SMCLK/128, inactive state is high
    USICNT  |= USIIFGCC;                  // Disable automatic clear control
    USICTL0 &= ~USISWRST;                // Enable USI
    USICTL1 &= ~USIIFG;                  // Clear pending flag
    _EINT();

#ifdef EZ_FET
    // pwmConfigure PWM for 3.6V
    pwmConfigure(EZ_FET_VOLTAGE);
#endif
#ifdef MSP_FET
    // pwmConfigure PWM for 3.0V
    pwmConfigure(MSP_FET_VOLTAGE);
#endif

    while(1)
    {
        // Main regulation loop - 12 cycles = 750nsec @ 16MHz (1.31MHz)
        if ((CACTL2&CAOUT)==0)
        {
            // VCC_OUT too low? Trigger "one-shot" charge pulse
            // 3.6V high=410nsec=7cycles, low=350nsec=5cycles
            TA0CTL = pwmTimerReg;
            _NOP();
        }
        else
        {
            // VCC_OUT too high? Skip pulse!
            TA0CTL = TACLR;
            // Increase PWM off counter - used for overcurrent detection
    #ifdef USE_OVERCURRENT_PROTECTION
            pwmOffCount++;
    #endif
        }

#define CMD_HANDLER
#ifdef CMD_HANDLER
        if (i2cCmd != -1)
        {
           commandHandler(i2cData);
        }
#endif
    }
}


/**
* \brief wdtIsr
*
* Interrupt service routine for watchdog timer
* Checks every 5.3msec if the pwmOffCount is below a
* voltage dependent threshold. If TRUE, then the output
* current is too high, and the overcurrent detection signal
* is asserted
*
* @param none
*
* @return void
*/
#ifdef USE_OVERCURRENT_PROTECTION
#pragma vector=WDT_VECTOR
__interrupt void wdtIsr(void)
{
    if (OC_SHUTDOWN_ENABLE)
    {
        // No off phases anymore? Then we have overcurrent!
        if (pwmOffCount == 0) //pwmTimeOut)
        {
          if (++pwmOcEvents > OVERCURRENT_EVENT_LIMIT)
          {
            // Assert overcurrent detection
            STATUS_OC_ON;
            // Wait for overcurrent condition to heal
            __delay_cycles(5000000);
            pwmOcEvents = 0;
          }
        }
        else
        {
            // Deassert overcurrent detection
            STATUS_OC_OFF;
            pwmOcEvents = 0;
        }
    }

    // Reset counter
    pwmOffCount = 0;
}
#endif




/**
* \brief usiTxrxIsr
*
* Interrupt service routine for I2C communication
*
* All I2C packets to the DCDC MCU are 4 x 8bit long
* 1: slave address
* 2: command
* 3: dataH
* 4: dataL
*
* When reading from the DCDC MCU, it returns 2 x 8bit
* 1: software fwVersion H
* 2: software fwVersion L
*
* @param none
*
* @return void
*/
#pragma vector = USI_VECTOR
__interrupt void usiTxrxIsr (void)
{
    // Start entry?
    if (USICTL1 & USISTTIFG)
    {
        i2cState = 2;                          // Enter 1st state on start
        i2cRxIndex = 0;
    }

    switch(i2cState)
    {
        case 0: // Idle, should not get here
                break;

        case 2: // RX Address
                USICNT = (USICNT & 0xE0) + 0x08; // Bit counter = 8, RX address
                USICTL1 &= ~USISTTIFG;   // Clear start flag
                i2cState = 4;           // Go to next state: check address
                break;

        case 4: // Process Address and send (N)Ack
                USICTL0 |= USIOE;        // SDA = output
                if ((USISRL&0xFE) == i2cSlaveAddress)  // Address match?
                {
                    if (USISRL & 0x01) i2cState = 18;        // Go to next state: TX data
                    else               i2cState = 8;         // Go to next state: RX data
                    USISRL = 0x00;         // Send Ack
                }
                else
                {
                    USISRL = 0xFF;         // Send NAck
                    i2cState = 6;         // Go to next state: prep for next Start
                }
                USICNT |= 0x01;          // Bit counter = 1, send (N)Ack bit
                break;

        case 6: // Prep for Start condition
                USICTL0 &= ~USIOE;       // SDA = input
                i2cState = 0;           // Reset state machine
                break;

        case 8: // Receive data byte
                USICTL0 &= ~USIOE;       // SDA = input
                USICNT |=  0x08;         // Bit counter = 8, RX data
                i2cState = 10;          // Go to next state: Test data and (N)Ack
                break;

        case 10:// Check Data & TX (N)Ack
                USICTL0 |= USIOE;             // SDA = output
                if (i2cRxIndex < I2C_PACKET_BYTES)
                {
                    i2cReceived[i2cRxIndex++] = USISRL;
                }
                USISRL = 0x00;                // Send Ack
                if (i2cRxIndex >= I2C_PACKET_BYTES)
                {
                    // Evaluate command
                    i2cCmd = i2cReceived[I2C_CMD_BYTE];
                    i2cData = ((unsigned int)i2cReceived[I2C_DATA_HI_BYTE])<<8;
                    i2cData += i2cReceived[I2C_DATA_LO_BYTE];
                    i2cState = 6;           // Go to next state: prep for next Start
                    i2cRxIndex = 0;
                }
                else
                {
                    i2cState = 8;           // Receive next byte
                }
                USICNT |= 0x01;          // Bit counter = 1, send (N)Ack bit
                break;

        case 18: // Send Version high byte
                USICTL0 |= USIOE;         // SDA = output
                USISRL = fwVersion>>8;     // Send data byte
                USICNT |=  0x08;          // Bit counter = 8, TX data
                i2cState = 20;           // Go to next state: receive (N)Ack
                break;

        case 20:// Receive Data (N)Ack
                USICTL0 &= ~USIOE;       // SDA = input
                USICNT |= 0x01;          // Bit counter = 1, receive (N)Ack
                i2cState = 22;          // Go to next state: check (N)Ack
                break;

        case 22:// Process Data Ack/NAck
                // Prep for Start condition
                if(USISRL & 0X01)
                {
                    USICTL0 &= ~USIOE;       // SDA = input
                    i2cState = 0;           // Reset state machine
                }
                else                        // Send NextByte
                {
                    // Send Version low byte
                    USICTL0 |= USIOE;         // SDA = output
                    USISRL = fwVersion&0XFF;   // Send data byte
                    USICNT |=  0x08;          // Bit counter = 8, TX data
                    i2cState = 20;           // Go to next state: receive (N)Ack
                }
                break;
    }

    USICTL1 &= ~USIIFG;                  // Clear pending flags
    LPM3_EXIT;                          // Clear LPM3 request
}
