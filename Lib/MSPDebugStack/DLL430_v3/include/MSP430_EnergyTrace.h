/*
 * MSP430_EnergyTrace.h
 *
 * API for accessing Energy Trace functionality of MSP430 library.
 *
 * Copyright (c) 2004 - 2016 Texas Instruments Incorporated - http://www.ti.com/
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

#include "MSP430.h"

/*
Record format
-------------

The default record format for operation mode 1 for the MSP430FR5859 is 22bytes for each record consisting of:
[8byte header][8byte device State][4byte current I in nA][2byte voltage V in mV][4byte energy E in uWsec= 100 nJ]

Where the header consists of:
[1byte eventID][7byte timestamp in usec]

The eventID defines the number of arguments following the header.
  eventID = 1 : I     value,  		    32 bits current
  eventID = 2 : V     value,  		    16 bits voltage
  eventID = 3 : I & V values, 	        32 bits current, 16 bits voltage
  eventID = 4 : S     value,  		    64 bits state                                                       (default type for ET_PROFILING_DSTATE)
  eventID = 5 : S & I values, 		    64 bits state, 32 bits current
  eventID = 6 : S & V values, 		    64 bits state, 16 bits voltage
  eventID = 7 : S & I & V & E values,	64 bits state, 32 bits current, 16 bits voltage, 32 bits energy     (default type for ET_PROFILING_ANALOG_DSTATE)
  eventID = 8 : I & V & E values,       32 bits current, 16 bits voltage, 32 bits energy                    (default type for ET_PROFILING_ANALOG)
  eventID = 9 : S & I & V values,	    64 bits state, 32 bits current, 16 bits voltage


Recommended initialisation settings & sequence
----------------------------------------------

1. For MSP430 microcontrollers with JSTATE register (e.g. MSP430FR5969)

EnergyTraceSetup = {  ET_PROFILING_ANALOG_DSTATE,         // Gives callbacks of with eventID 7
                      ET_PROFILING_10K,                   // N/A
                      ET_ALL,                             // Gives all JSTATE information
                      ET_EVENT_WINDOW_100,                // N/A
                      ET_CALLBACKS_ONLY_DURING_RUN }      // Callbacks are sent only when target MCU is executing code

Recommended sequence
                      MSP430_Initialize()
                      MSP430_VCC(XYZ)
                      MSP430_Configure(INTERFACE_MODE, XYZ)
                      MSP430_OpenDevice("","",0,0x00000000,0x00000000)
                      MSP430_EnableEnergyTrace(EnergyTraceSetup, EnergyTraceCallbacks, Handle)
                      MSP430_Run(RUN_TO_BREAKPOINT, 0) or MSP430_Run(FREE_RUN, 0)
                      ..
                      <process EnergyTraceCallbacks>
                      ..
                      MSP430_State(State,1,CPUCycles)
                      MSP430_DisableEnergyTrace(Handle)
                      MSP430_Close(0)

2. For MSP430 microcontrollers without JSTATE register

EnergyTraceSetup = {  ET_PROFILING_ANALOG,                // Gives callbacks of with eventID 8
                      ET_PROFILING_10K,                   // N/A
                      ET_ALL,                             // N/A
                      ET_EVENT_WINDOW_100,                // N/A
                      ET_CALLBACKS_ONLY_DURING_RUN }      // Callbacks are sent only when target MCU is executing code

Recommended sequence
                      MSP430_Initialize()
                      MSP430_VCC(XYZ)
                      MSP430_EnableEnergyTrace(EnergyTraceSetup, EnergyTraceCallbacks, Handle)
                      MSP430_Run(RUN_TO_BREAKPOINT, 0) or MSP430_Run(FREE_RUN, 0) or MSP430_Run(FREE_RUN, 1)
                      ..
                      <process EnergyTraceCallbacks>
                      ..
                      MSP430_State(State,1,CPUCycles)
                      MSP430_DisableEnergyTrace(Handle)
                      MSP430_Close(0)

3. For analog sampling without target code download and execution

EnergyTraceSetup = {  ET_PROFILING_ANALOG,                // Gives callbacks of with eventID 8
                      ET_PROFILING_10K,                   // N/A
                      ET_ALL,                             // N/A
                      ET_EVENT_WINDOW_100,                // N/A
                      ET_CALLBACKS_CONTINUOUS }           // Callbacks are continuously

Recommended sequence
                      MSP430_Initialize()
                      MSP430_VCC(XYZ)
                      MSP430_EnableEnergyTrace(EnergyTraceSetup, EnergyTraceCallbacks, Handle)
                      ..
                      <process EnergyTraceCallbacks>
                      ..
                      MSP430_DisableEnergyTrace(Handle)
                      MSP430_Close(0)

Optional
--------
To enable higher resolution current sampling, set "MSP430_Configure(ENERGYTRACE_CURRENTDRIVE, 1)" before calling "MSP430_VCC(XYZ)".

*/

#ifndef _MSP430_ENERGYTRACE_H_
#define _MSP430_ENERGYTRACE_H_

#ifdef __cplusplus
extern "C" {
#endif

// Push style interface

// EnergyTrace mode selector
typedef enum ETMode
{
  ET_PROFILING_ANALOG,  		    // Time-driven device state profiling (analog measurement only)
  ET_PROFILING_DSTATE,  		    // Time-driven device state profiling (digital state only)
  ET_PROFILING_ANALOG_DSTATE,  	    // Time-driven device state profiling (analog measurement and digital state)
  ET_EVENT_ANALOG,       		    // Event-triggered device state capturing (analog measurement only)
  ET_EVENT_DSTATE,      		    // Event-triggered device state capturing (digital state only)
  ET_EVENT_ANALOG_DSTATE,     	    // Event-triggered device state capturing (analog measurement and digital state)
} ETMode_t;

// Sample frequency (callbacks will be triggered according to this frequency)
typedef enum ETProfiling_samplingFreq
{
  ET_PROFILING_OFF,   // no sampling of device state
  ET_PROFILING_100,   // 100Hz
  ET_PROFILING_1K,    // 1kHz
  ET_PROFILING_5K,    // 5kHz
  ET_PROFILING_10K,   // 10kHz
  ET_PROFILING_50K,   // 50kHz
  ET_PROFILING_100K,  // 100kHz
} ETProfiling_samplingFreq_t;

// EnergyTrace recording format selector
typedef enum ETProfilingDState_recFormat
{
	// defines how much Device State information is reported.
	// For Wolverine the Device State is the content of the
	// JTAG Device State Register (64bits in total)
	// Comment below are true for Wolverine, but might differ for newer devices in the future.
	// The record format needs to be synchronized with the information in the device specific XML files.
	ET_POWER_MODE_ONLY,           // bits 63 - 52
	ET_POWER_MODE_CODE_PROFILING, // bits 63 - 33
	ET_ALL                        // all 64 bits (bit 32 and not available mod bits must be masked out for further processing)
} ETProfilingDState_recFormat_t;

// EnergyTrace event-triggered profiling window size
typedef enum ETEvent_window
{
	// the number of samples (plus/minus) around the event trigger which will be pushed
	// to the debugger (actually the size of the ringbuffer for analog measurement)
	ET_EVENT_WINDOW_25,   // 25 samples of current and voltage
	ET_EVENT_WINDOW_50,   // 50 -"-
	ET_EVENT_WINDOW_100,  // 100 -"-
	ET_EVENT_WINDOW_500,  // 500 -"-
	ET_EVENT_WINDOW_1000, // 1000 -"-
} ETEvent_window_t;


// EnergyTrace callback mode
typedef enum ETCallback_mode
{
	// Callbacks can be sent either continuous (e.g. for analog sampling)
	// or only when the target MCU is executing code (e.g. run to breakpoint or free run)
	ET_CALLBACKS_CONTINUOUS,        // Callbacks are sent after MSP430_EnableEnergyTrace is called
	ET_CALLBACKS_ONLY_DURING_RUN,   // Callbacks are sent after MSP430_EnableEnergyTrace is called
                                    // and the target MCU is executing code
} ETCallback_mode_t;


// EnergyTrace setup information
typedef struct EnergyTraceSetup_tag
{
    // Time-driven device state profiling
    ETMode_t 						    ETMode; 	    // defines the operation mode
    ETProfiling_samplingFreq_t		    ETFreq; 	    // device state sampling frequency
    ETProfilingDState_recFormat_t 	    ETFormat;  	    // format of the device state information

    // Event-triggered device state capturing
    ETEvent_window_t 				    ETSampleWindow;

    // Generic
    ETCallback_mode_t                   ETCallback;     // callback style

} EnergyTraceSetup;

typedef enum EnergyTraceEventID_tag
{
    ET_EVENT_CURR             = 1,
    ET_EVENT_VOLT	          = 2,
    ET_EVENT_CURR_VOLT        = 3,
    ET_EVENT_STATE	          = 4,
    ET_EVENT_STATE_CURR       = 5,
    ET_EVENT_STATE_VOLT       = 6,
    ET_EVENT_STATE_VOLT_CURR  = 7,
    ET_EVENT_CURR_VOLT_ENERGY = 8,
    ET_EVENT_ALL              = 9,
} EnergyTraceEventID;

typedef void( *PushDataFn )( void* pContext, const uint8_t* pBuffer, uint32_t nBufferSize );

// Currently implemented error messages (in order of priority, since several errors could be present in parallel)
// "Unsupported debugger"				  Debugger does not support analog or dstate profiling
// "Unsupported device"					  Device or device variant does not support dstate profiling
// "USB voltage out of specification"	  USB voltage not sufficient for target power supply
// "Debugger overcurrent detected"		  Debugger overcurrent condition detected
// "Generic debugger error"				  Generic debugger malfunction detected
typedef void( *ErrorOccurredFn )( void* pContext, const char* pszErrorText );

typedef struct EnergyTraceCallbacks_tag
{
	// Context data defined by the client to be passed to the callback functions
	void* pContext;

	// Called to push new data to the client
	PushDataFn pPushDataFn;

	// Called when an error has occurred
	ErrorOccurredFn pErrorOccurredFn;

} EnergyTraceCallbacks;

typedef void* EnergyTraceHandle;

// Enables EnergyTrace with the given setup configuration and callback functions.
// Returns a handle that can be used to disable profiling
// If this returns an error, then MSP430_Error_Number()/MSP430_Error_String() can fetch
// the error message
DLL430_SYMBOL STATUS_T WINAPI MSP430_EnableEnergyTrace(
	const EnergyTraceSetup* setup, 			// in
	const EnergyTraceCallbacks* callbacks, 	// in
	EnergyTraceHandle* handle ); 			// out

// Reset EnergyTrace session associated with the passed in handle
DLL430_SYMBOL STATUS_T WINAPI MSP430_DisableEnergyTrace( const EnergyTraceHandle handle );

// Reset EnergyTrace Buffers and Processors
// Function should be called before collecting new EnergyTrace data
DLL430_SYMBOL STATUS_T WINAPI MSP430_ResetEnergyTrace( const EnergyTraceHandle handle );

#ifdef __cplusplus
}
#endif

#endif // _MSP430_ENERGYTRACE_H_
