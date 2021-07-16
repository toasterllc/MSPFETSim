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
#include "BSL_Device_File.h"
#include "types.h"      // Basic Type declarations
#include "Proj_Settings.h"
#include "usb.h"        // USB-specific Data Structures
#include "hal_UCS.h"
#include "USBAPI.h"
#include "USBAPI.h"
#include "BSL430_Command_Interpreter.h"
#include "BSL430_PI.h"
#include "BSL430_API.h"

void interpretPI_Command();
#define MAX_BUFFER_SIZE 62
extern __no_init BYTE bFunctionSuspended;
__no_init char *BSL430_ReceiveBuffer;
__no_init char *BSL430_SendBuffer;
__no_init unsigned int BSL430_BufferSize;
char RAM_Buf[MAX_BUFFER_SIZE+1];

// TA PI Commands
#define USBDISCONNECT (PI_COMMAND_UPPER + 0x03)

#ifndef USBKEYPID_STARTUP_BUGFIX
#error ERROR: Ensure that the USBKEYPID is closed correctly in the low level init file!
#endif

/*******************************************************************************
Change Log:
--------------------------------------------------------------------------------
Version 6 work begins
11.10.12 LCW USB bugfixes
         Reintroduced USB conf lock bug to keep device behavior the same
--------------------------------------------------------------------------------
Version 5 work begins
02.03.12 LCW  Correctly lock USB conf registers in low level init
         LCW  Added bugfix check to C file to ensure other devices get fix
--------------------------------------------------------------------------------
Version 4 work begins
16.03.10 LCW  DCORSEL_3 -> DCORSEL_4
              Inlined XT2 startup code
              Source code cleanup
*******************************************************************************/

#define USB_PI 0x30
#define USB_PI_VERSION 0x10
#define PI_VERSION (USB_PI+USB_PI_VERSION) // 0x36
const unsigned char BSL430_PI_Version @ "BSL430_VERSION_PI" = PI_VERSION;
/*******************************************************************************
*Function:    init
*Description: Initialize the peripheral and ports to begin TX/RX
*******************************************************************************/
#pragma required=BSL430_PI_Version
void PI_init()
{
#ifdef RAM_BASED_BSL
    __disable_interrupt();
    BSL430_ReceiveBuffer = (char*)(ReceiveBuffer)+1;
    BSL430_SendBuffer = &RAM_Buf[1];
#else
    volatile unsigned short timeOut = 0;
    __disable_interrupt();

    // Init Leds
    GREEN_LED_PORT_DIR |= GREEN_LED;
    RED_LED_PORT_DIR  |= RED_LED;

    GREEN_LED_PORT &= ~GREEN_LED;
    RED_LED_PORT &= ~RED_LED;

    BSL430_ReceiveBuffer = (char*)(ReceiveBuffer)+1;
    BSL430_SendBuffer = &RAM_Buf[1];

    //Set VCore for 1.8 Volt - required by USB module!

    // Open PMM registers for write access
    PMMCTL0_H = 0xA5;

    unsigned short level = PMMCTL0_L & (PMMCOREV1 | PMMCOREV0);
    while(level < 4)
	{
        // Set SVM highside to new level and check if a VCore increase is possible
        SVSMHCTL = SVMHE | SVSHE | (SVSMHRRL0 * level);
        // Wait until SVM highside is settled
        while ((PMMIFG & SVSMHDLYIFG) == 0);
        // Clear flag
        PMMIFG &= ~SVSMHDLYIFG;
        // Set also SVS highside to new level
        // Vcc is high enough for a Vcore increase
        SVSMHCTL |= (SVSHRVL0 * level);
        // Wait until SVM highside is settled
        while ((PMMIFG & SVSMHDLYIFG) == 0);
        // Clear flag
        PMMIFG &= ~SVSMHDLYIFG;
        //**************flow change for errata workaround ************
        // Set VCore to new level
        PMMCTL0_L = PMMCOREV0 * level;
        // Set SVM, SVS low side to new level
        SVSMLCTL = SVMLE | (SVSMLRRL0 * level)| SVSLE | (SVSLRVL0 * level);
        // Wait until SVM, SVS low side is settled
        while ((PMMIFG & SVSMLDLYIFG) == 0);
        // Clear flag
        PMMIFG &= ~SVSMLDLYIFG;
        //**************flow change for errata workaround ************

        // Increment next VCore level
        level++;
    }
    // Lock PMM registers for write access
    PMMCTL0_H = 0x00;

    //XT2 Startup
    XT2SEL_PORT |= XT2SEL_PINS;
    UCSCTL6 |=  XT1OFF;

    UCSCTL6 &= ~(XT2OFF + XT2DRIVE0 + XT2DRIVE1);               // enalbe XT2 even if not used
    while (UCSCTL7 & (DCOFFG + XT2OFFG) && timeOut++ < TIME_OUT_COUNT)
    {
        UCSCTL7 &= ~(DCOFFG + XT2OFFG);                         // Clear OSC flaut Flags fault flags
        SFRIFG1 &= ~OFIFG;                                      // Clear OFIFG fault flag
    }
    while(timeOut >= TIME_OUT_COUNT)
    {
        GREEN_LED_PORT ^= GREEN_LED;
        RED_LED_PORT ^= RED_LED;
        __delay_cycles(800000);
    }

    UCSCTL3 = SELREF__REFOCLK;                                  // REFO for FLL reference
    UCSCTL4 = SELA__REFOCLK + SELS__XT2CLK + SELM__DCOCLK;      // ACLK = REFOCLK, SMCLK = XT2 ?? used??, MCLK = DCO
    UCSCTL5 = DIVA_2;                                           // ACLK/4
    __delay_cycles(800000);

    wUSBPLL = USBPLL_SETCLK_4_0;

    UCSCTL0 = 0x000;                                            // Set DCO to lowest Tap
    UCSCTL2= FLLD__2 | ((DCO_SPEED/ACLK_SPEED) - 1);
    UCSCTL1= DCORSEL_4;
    UCSCTL4 = SELM__DCOCLKDIV + SELS__DCOCLKDIV + SELA__REFOCLK;

    //init USB
    USB_init();

    if (USBPWRCTL & USBBGVBV)
    {
        USB_enable();
        USB_reset();
        USBCNF |= PUR_EN; // generate rising edge on DP -> the host enumerates our device as full speed device
    }
#endif

}
#ifdef RAM_BASED_BSL
void USB_disconnect()
{
    USBKEYPID = 0x9628;                                                             //set KEY and PID to 0x9628 -> access to
                                                                                    //configuration registers enabled
    USBCNF &= ~PUR_EN;                                                              //disconnect pull up resistor - logical
                                                                                    //disconnect from HOST
    USBPWRCTL &= ~VBOFFIE;                                                          //disable interrupt VUSBoff
    USBKEYPID = 0x9600;                                                             //access to configuration registers disabled
}

void USB_disable()
{
    USBKEYPID = 0x9628;                                                         //set KEY and PID to 0x9628 -> access to
                                                                                //configuration registers enabled
    USBCNF    = 0;                                                              //disable USB module
    USBPLLCTL &= ~UPLLEN;                                                       //disable PLL
    USBKEYPID = 0x9600;                                                         //access to configuration registers disabled
}
#endif

/*******************************************************************************
*Function:    PI_receivePacket
*Description: Reads an entire packet, verifies it, and sends it to the core to be interpreted
*Returns:
*             DATA_RECEIVED         A packet has been received and can be processed
*             RX_ERROR_RECOVERABLE  An error has occured, the function can be called again to
*                                   receive a new packet
*******************************************************************************/
char PI_receivePacket()
{
    BSL430_BufferSize = 0;
    UsbClearReceiveBuffer();
    while( BSL430_BufferSize == 0 )
    {
        UsbHandler();
        if (bEnumerationStatus == ENUMERATION_COMPLETE) // if enumeration completed
        {
            GREEN_LED_PORT |= GREEN_LED;                // turn on Green LED with enumeration

            if (bFunctionSuspended == FALSE)            // and device not suspended
            {
                BSL430_BufferSize = UsbReceiveHID();
            }
        }
    }
    if( BSL430_BufferSize > 0 )
    {

        RED_LED_PORT ^= RED_LED; // Toggle RED LED with each packet RX

        BSL430_BufferSize = *ReceiveBuffer;   // first byte is size
        if( (BSL430_ReceiveBuffer[0]) == USBDISCONNECT)
        {
            #ifdef RAM_BASED_BSL
                USB_disconnect();
                __delay_cycles(8000001);
                USB_disable();
                __delay_cycles(8000001);
                UCSCTL6 |= XT2OFF;                  // Switch off XT2 oscillator
            #else
                USBKEYPID = 0x9628;                 // set KEY and PID to 0x9628 -> access to configuration registers enabled
                USBPWRCTL = 0;                      //~VBOFFIE+~UPLLEN;  // disable interrupt VUSBoff
                USBCNF =    0;
                UCSCTL6 |= XT2OFF;                  // Switch off XT2 oscillator
            #endif
            PMMCTL0 = PMMPW | PMMSWBOR;             // generate BOR for reseting device
        }
        else
        {
            return DATA_RECEIVED;
        }
    }
    return 0x00;
}
/*******************************************************************************
*Function:    interpretPI_Command
*Description: interprets a PI command
*Parameters:
              none
*******************************************************************************/
void interpretPI_Command()
{}

/*******************************************************************************
*Function:    PI_sendData
*Description: Sends the data in the data buffer
*Parameters:
              int bufSize - the number of bytes to send
*******************************************************************************/
void PI_sendData(int bufSize)
{
  RAM_Buf[0] = bufSize;
  while( UsbSendHID((BYTE const*)RAM_Buf) == 0 );
}
/*******************************************************************************
*Function:    PI_getBufferSize
*Description: Returns the max Data Buffer Size for this PI (size of BSL430_SendBuffer)
*Returns:     size in bytes of BSL430_SendBuffer
*******************************************************************************/
int PI_getBufferSize()
{
  return MAX_BUFFER_SIZE;
}
