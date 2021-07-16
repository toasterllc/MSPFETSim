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
#include "msp430.h"

// Vendor ID (VID) for device, TI (MSP430)=0x2047
//    You can use is only in this example.
//    You can order your own VID at www.usb.org
#define USB_VID 0x2047        // Vendor ID, 0x2047 for Texas Instruments Incorporated (MSP430 Group)

#ifdef MSP_FET
    #define USB_PID 0x0204         // Product ID (PID), 0x0203 ez-FET, 0x0204 MSP FET
#endif

#ifdef eZ_FET
    #define USB_PID 0x0203         // Product ID (PID), 0x0203 ez-FET, 0x0204 MSP FET
#endif

// The following defines are here so that the supported crystals can be
// both easily seen, and changed without having to recompute any TBCCR6 values
// When changing these values, be sure to also change the matching SPEED_x_PLL values
// NOTE: Speeds must be in order from highest to lowest!
// NOTE: If only one external frequency is used,
//       all SPEED_x_PLL values must be changed to that value!

//24MHz will be detected
#define SPEED_1          24000000
#define SPEED_1_PLL      USBPLL_SETCLK_24_0;

//12MHz will be detected
#define SPEED_2          12000000
#define SPEED_2_PLL      USBPLL_SETCLK_12_0;

//8MHz will be detected
#define SPEED_3          8000000
#define SPEED_3_PLL      USBPLL_SETCLK_8_0;

//4MHz will be detected
#define SPEED_4          4000000
#define SPEED_4_PLL      USBPLL_SETCLK_4_0;


//Device Specific Definitions
#define MASS_ERASE_DELAY 0x8000
#define INTERRUPT_VECTOR_START 0xFFE0
#define INTERRUPT_VECTOR_END   0xFFFF
#define SECURE_RAM_START 0x2400

//******************* CHANGE BASED ON RAM/FLASH
#define DCO_SPEED 8000000
#define ACLK_SPEED 32768

#define TIMER_CTL           TBCTL
#define TIMER_CTL_SETTINGS  TBSSEL_2 + MC_2
#define TIMER_CTL_CLR       TBCLR

#define TIMER_CCTL          TBCCTL6
#define TIMER_CCTL_SETTINGS CM_1 + CCIS_1+ CAP
#define TIMER_CCTL_IFG      CCIFG
#define TIMER_CCTL_CM       CM_1

#define TIMER_CCR           TBCCR6


#ifdef MSP_FET
    //define xt2 pins fo fets
    #define XT2SEL_PORT             P7SEL
    #define XT2SEL_PINS             (BIT2 + BIT3)

    //define LEDS pins fo fets
    #define GREEN_LED               BIT3
    #define GREEN_LED_PORT          P5OUT
    #define GREEN_LED_PORT_DIR      P5DIR

    #define RED_LED                 BIT7
    #define RED_LED_PORT            P6OUT
    #define RED_LED_PORT_DIR        P6DIR

    #define TIME_OUT_COUNT          1000
#endif

#ifdef eZ_FET
    //define xt2 pins fo fets
    #define XT2SEL_PORT             P5SEL
    #define XT2SEL_PINS             (BIT2 + BIT3)

    //define LEDS pins fo fets
    #define GREEN_LED               BIT3
    #define GREEN_LED_PORT          P1OUT
    #define GREEN_LED_PORT_DIR      P1DIR

    #define RED_LED                 BIT2
    #define RED_LED_PORT            P1OUT
    #define RED_LED_PORT_DIR        P1DIR

    #define TIME_OUT_COUNT          1000
#endif

//Device Specific Definitions

//Device Specific Definitions for commands and bugs
#ifdef RAM_WRITE_ONLY_BSL_DEF
    #define RAM_WRITE_ONLY_BSL 0x80
#endif

#ifdef FLASH_WRITE_BSL
    #define FULL_FLASH_BSL
    #define RAM_BASED_BSL      1
#endif

#define DO_NOT_CHECK_VPE
#define USBKEYPID_STARTUP_BUGFIX
#define LOCK_USB_CORRECTLY
// Bugfix not implimented, but left in on purpose

// standard command includes
#ifdef RAM_WRITE_ONLY_BSL
 #define SUPPORTS_RX_DATA_BLOCK_FAST
 #define SUPPORTS_RX_PASSWORD
 #define SUPPORTS_LOAD_PC
#endif

#ifdef FULL_FLASH_BSL
 #define FULL_GENERIC_COMMANDS
 #define FLASH_COMMANDS
#endif

#ifdef FULL_FRAM_BSL
 #define FULL_GENERIC_COMMANDS
#endif

#ifdef FULL_GENERIC_COMMANDS
 #define SUPPORTS_RX_DATA_BLOCK_FAST
 #define SUPPORTS_RX_PASSWORD
 #define SUPPORTS_LOAD_PC
 #define SUPPORTS_RX_DATA_BLOCK
 #define SUPPORTS_MASS_ERASE
 #define SUPPORTS_CRC_CHECK
 #define SUPPORTS_TX_DATA_BLOCK
 #define SUPPORTS_TX_BSL_VERSION
#endif

#ifdef FLASH_COMMANDS
 #define SUPPORTS_ERASE_SEGMENT
 #define SUPPORTS_TOGGLE_INFO
#endif
