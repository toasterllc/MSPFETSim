/*
 * Exceptions.cpp
 *
 * Emulation module exceptions
 *
 * Copyright (C) 2007 - 2011 Texas Instruments Incorporated - http://www.ti.com/
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

#include <pch.h>
#include "Exceptions.h"


using namespace TI::DLL430;


DLL430_FileOpenException::DLL430_FileOpenException()
	: DLL430_Exception(FILE_OPEN_ERR, "Can't open file") {}


EM_NoEmulationManagerException::EM_NoEmulationManagerException()
	: EM_Exception(INTERNAL_ERR, "No emulation manager") {}


EM_NoTriggerConditionManagerException::EM_NoTriggerConditionManagerException()
	: EM_Exception(INTERNAL_ERR, "Module does not exist") {}


EM_NoBreakpointManagerException::EM_NoBreakpointManagerException()
	: EM_Exception(BREAKPOINT_ERR, "Module does not exist") {}


EM_NoClockControlException::EM_NoClockControlException()
	: EM_Exception(CLK_CTRL_ERR, "Module does not exist") {}


EM_NoCycleCounterException::EM_NoCycleCounterException()
	: EM_Exception(RESOURCE_ERR, "Module does not exist") {}


EM_NoSequencerException::EM_NoSequencerException()
	: EM_Exception(SEQUENCER_ERR, "Module does not exist") {}


EM_NoTraceException::EM_NoTraceException()
	: EM_Exception(STATE_STOR_ERR, "Module does not exist") {}


EM_NoVariableWatchException::EM_NoVariableWatchException()
	: EM_Exception(STATE_STOR_ERR, "Module does not exist") {}


EM_NoSoftwareBreakpointsException::EM_NoSoftwareBreakpointsException()
	: EM_Exception(RESOURCE_ERR, "Module does not exist") {}


EM_NotSupportedException::EM_NotSupportedException()
	: EM_Exception(RESOURCE_ERR, "Function is not supported") {}


EM_TriggerResourceException::EM_TriggerResourceException()
	: EM_Exception(RESOURCE_ERR, "Out of trigger resources") {}


EM_TriggerParameterException::EM_TriggerParameterException()
	: EM_Exception(PARAMETER_ERR, "Option not supported") {}


EM_RegisterWriteException::EM_RegisterWriteException()
	: EM_Exception(WRITE_REGISTER_ERR, "Error writing EEM registers") {}


EM_RegisterReadException::EM_RegisterReadException()
	: EM_Exception(READ_REGISTER_ERR, "Error reading EEM registers") {}


EM_SequencerException::EM_SequencerException()
	: EM_Exception(PARAMETER_ERR, "Invalid state or transition") {}


EM_StateStorageConflictException::EM_StateStorageConflictException()
	: EM_Exception(RESOURCE_ERR, "Trace and Variable watch can't be used simultaneously") {}


EM_NotVariableWatchModeException::EM_NotVariableWatchModeException()
	: EM_Exception(VAR_WATCH_EN_ERR, "Not in variable watch mode") {}


EM_TriggerConfigurationException::EM_TriggerConfigurationException()
	: EM_Exception(RESOURCE_ERR, "No valid trigger configuration possible") {}


EM_EemNotAccessibleException::EM_EemNotAccessibleException()
	: EM_Exception(EEM_NOT_ACCESSIBLE, "EEM module not accessible while running in Low Power Debug Mode - Activate High Power Debug Mode to enable this feature") {}


EM_TriggerConflictException::EM_TriggerConflictException()
	: EM_Exception(TRIGGER_CONFLICT_ERR, "Trigger conflicts with existing trigger") {}


EM_SwbpCriticalInstruction::EM_SwbpCriticalInstruction()
	: EM_Exception(SWBP_CRITICAL_INSTRUCTION, "Software breakpoint can't be set (followed by critical value)") {}


EM_SoftwareBreakpointsNotEnabledException::EM_SoftwareBreakpointsNotEnabledException()
	: EM_Exception(RESOURCE_ERR, "Software breakpoints are not enabled") {}


EM_MemoryAccessFunctionException::EM_MemoryAccessFunctionException()
	: EM_Exception(INTERNAL_ERR, "Memory access functions not configured") {}


EM_InstructionEntryMissingException::EM_InstructionEntryMissingException()
	: EM_Exception(INTERNAL_ERR, "No instruction stored for this address") {}


EM_MemoryReadErrorException::EM_MemoryReadErrorException()
	: EM_Exception(READ_MEMORY_ERR, "Failed reading device memory") {}


EM_MemoryWriteErrorException::EM_MemoryWriteErrorException()
	: EM_Exception(WRITE_MEMORY_ERR, "Failed writing device memory") {}
