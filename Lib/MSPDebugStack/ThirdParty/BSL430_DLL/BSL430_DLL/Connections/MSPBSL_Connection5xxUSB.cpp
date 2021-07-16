/*
 * MSPBSL_Connection5xxUSB
 *
 * A subclass to add USB specific connection functions
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
 *
*/

#include <pch.h>

#include "MSPBSL_Connection5xxUSB.h"
#include "MSPBSL_PhysicalInterfaceUSB.h"
#include "MSPBSL_PacketHandler.h"

#include "RAM_BSL_FLASH_MODE_eZ_FET.h"
#include "RAM_BSL_FLASH_MODE_MSP_FET.h"

using namespace std;

/***************************************************************************//**
* MSPBSL_Connection5xxUSB Constructor.
*    
* \param initString an initialization string for the connection
*
* \return a MSPBSL_Connection5xxUSB class
******************************************************************************/
MSPBSL_Connection5xxUSB::MSPBSL_Connection5xxUSB(string initString) : MSPBSL_Connection5xx( initString)
{
}

/***************************************************************************//**
* MSPBSL_Connection5xxUSB Destructor.
*        
******************************************************************************/
MSPBSL_Connection5xxUSB::~MSPBSL_Connection5xxUSB(void)
{
}

void MSPBSL_Connection5xxUSB:: closeBslconnection()
{
	uint8_t usbDisconnct = USB_DISCONNECT_COMMAND;
	this->thePacketHandler->TX_Packet(&usbDisconnct,1);
	std::this_thread::sleep_for(std::chrono::seconds(5));
}

/***************************************************************************//**
* Loads the USB BSL into RAM using the default password
*
* Creates a default 5/6xx password array (32x 0xFF) and calls LoadRAM_BSL with
* that as a parameter
*        
* \return the result of the LoadRAM_BSL function call
******************************************************************************/
uint16_t MSPBSL_Connection5xxUSB::loadRAM_BSL(uint16_t toolPid)
{
	uint8_t buf_array[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	return loadRAM_BSL( buf_array, toolPid );
}

/***************************************************************************//**
* USB-specific helper function to load the RAM BSL and start it
*
* This function is included in the 5xx USB class as a convenience function
* it's purpose is to make it simple to unlock a device, load and start the RAM
* BSL, and re-connect to this RAM BSL
*
* \param pass the password to submit to the connected device's BSL
*        
* \return an unsigned byte describing the success or failure of the entire operation
*         0: No error, RAM based BSL is loaded, and further communication can 
*            continue using this class
*        >0: one or more errors occured
******************************************************************************/
#define MSPBSL_EZ_FET_USB_PID 0x0203
#define MSPBSL_MSP_FET_USB_PID 0x0204

uint16_t MSPBSL_Connection5xxUSB::loadRAM_BSL(uint8_t* password, uint16_t toolPid)
{	
	uint16_t retValue = RX_Password( password );
	if( retValue != ACK )
	{
		return retValue;
	}

	if(MSPBSL_EZ_FET_USB_PID == toolPid)
	{
	retValue = RX_DataBlockFast(RAM_BSL_FLASH_MODE_eZ_FETImage, 0x2500, sizeof RAM_BSL_FLASH_MODE_eZ_FETImage);
		if( retValue != ACK )
		{
			return retValue;
		}
	}
	if(MSPBSL_MSP_FET_USB_PID == toolPid)
	{
		retValue = RX_DataBlockFast(RAM_BSL_FLASH_MODE_MSP_FETImage, 0x2500, sizeof RAM_BSL_FLASH_MODE_MSP_FETImage);
		if( retValue != ACK )
		{
			return retValue;
		}
	}

	retValue = setPC(0x2504);
	if( retValue != ACK )
	{
		return retValue;
	}
	std::this_thread::sleep_for(std::chrono::seconds(1));

	return ACK;
}

/***************************************************************************//**
* An error description function
*
* This function is meant to return a string which fully describes an error code
* which could be returned from any function within this class
* 
* \param err the 16 bit error code
*
* \return A string describing the error code
******************************************************************************/
string MSPBSL_Connection5xxUSB::getErrorInformation( uint16_t err )
{
	return MSPBSL_Connection5xx::getErrorInformation( err );
}
