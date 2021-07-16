/*
 * IMemoryManager.h
 *
 * Handles access to different memory modules by means of address.
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

#pragma once


#include "MemoryArea.h"

namespace TI
{
	namespace DLL430
	{
		class CpuRegisters;

		/** \brief manages global address space or all memory areas */
		class IMemoryManager
		{
		public:
			virtual ~IMemoryManager() {}

			/** \brief get a memory area by name and sub-index
			 *
			 * \param name the name of the area
			 * \param subIndex addresses multiple areas of the same name (usually 0)
			 * \return pointer to a memory area if the combination of name and subIndex exists, else 0
			 */
			virtual MemoryArea* getMemoryArea(MemoryArea::Name type, size_t subIndex = 0) = 0;

			/** \brief get the index by memory area by name and address
			*
			* \param name the name of the area
			* \param  addresses of memory area to be accessed 
			* \param  size lenght of memory access
			* \return index of area
			*/
			virtual int16_t getMemoryAreaIndex(MemoryArea::Name type, uint32_t address = 0, uint32_t size = 0) = 0;

			/** \brief get a memory area by index
			 *
			 * \param index the index of the area
			 * \return pointer to a memory area if index is valid, else 0
			 */
			virtual MemoryArea* getMemoryArea(size_t index) = 0;

			virtual CpuRegisters* getCpuRegisters(size_t index = 0) = 0;

			/** \brief get the number of memory areas
			 *
			 * \return the number of memory areas
			 */
			virtual size_t count() const = 0;

			/** \brief read from memory
			 *
			 * This registers a read call. The content of buffer is undefined
			 * until you call the sync() method.
			 *
			 * \param address the first address
			 * \param buffer the buffer to write the read data to
			 * \param count the number of elements to read
			 * \return true if the read was registered successfully, else false
			 */
			virtual bool read(uint32_t address, uint8_t* buffer, size_t count) = 0;

			/** \brief overwrite memory
			 *
			 * Convenience function for flash memory (internally handles erase, etc.)
			 *
			 * \param address the first address
			 * \param buffer the buffer to read the data to write from
			 * \param count the number of elements to write
			 * \return true if the write was registered successfully, else false
			 */
			virtual bool overwrite(uint32_t address, const uint8_t* buffer, size_t count) = 0;

			/** \brief write to memory
			 *
			 * This registers a write call. The content of buffer must stay valid
			 * until you call the sync() method.
			 *
			 * \param address the first address
			 * \param buffer the buffer to read the data to write from
			 * \param count the number of elements to write
			 * \return true if the write was registered successfully, else false
			 */
			virtual bool write(uint32_t address, const uint8_t* buffer, size_t count) = 0;

			/** \brief write to memory
			 *
			 * This registers a write call. The transfer is delayed
			 * until you call the sync() method.
			 *
			 * \param address the first address
			 * \param value the value to write to address
			 * \return true if the write was registered successfully, else false
			 */
			virtual bool write(uint32_t address, uint32_t value) = 0;

			/** \brief This function must be called after using read() or write()
			 *
			 * This function MUST be called to complete a read or write call.
			 * Reads and writes can be combined in any order unless they are inter-dependent.
			 *
			 * \return true if the sync was successfull, else false
			 */
			virtual bool sync() = 0;

			/** \brief erase a flash area completely
			 *
			 * This function erases a complete flash area. It will do nothing
			 * if the area is not a flash.
			 *
			 * \return true if flash was erased successfully, else false
			 */
			virtual bool erase() = 0;

			/** \brief erase a flash segment
			 *
			 * This function erases a flash segment. It will do nothing
			 * if the area is not a flash. The given start and end address
			 * mark all segments that any of the addresses in this range
			 * is in as to be erased. The addresses to no have to be aligned
			 * to segment size.
			 *
			 * \param start the first address
			 * \param end the last address
			 * \param if true forces to unlock any memory protection (for example disables MPU lock bit)
			 * \return true if flash segments were erased successfully, else false
			 */
			virtual bool erase(uint32_t start, uint32_t end, bool forceUnlock = false) = 0;

			/** \brief verify memory content
			 *
			 * This function verifies the contents of memory by using the
			 * Pseudo Signature Algorithm (PSA). This is a lot faster then reading
			 * all memory since the PSA is executed on the target device itself.
			 * Note that this will flush all affected cache contents.
			 *
			 * \param address the first address
			 * \param buffer the data to verify against (one byte per buffer element)
			 * \param count the number of buffer elements
			 * \return true if the PSA values matched, else false
			 */
			virtual bool verify(uint32_t address, const uint8_t* buffer, size_t count) = 0;

			/** \brief check of the area is cacheable
			 *
			 * \return true if the area is readOnly, else false
			 */
			virtual bool isReadOnly() const = 0;

			/** \brief	Locks/unlocks desired memory module.
			 *			Memory Module is identified by name.
			 *			Action == true locks module,
			 *			Action == false unlocks module.
			 * \return	true if module isProtected and the state switched actually from
			 *			locked to unlocked or vice versa, false otherwise.
			 */
			virtual bool lock(MemoryArea::Name name, bool action) = 0;

			/** \brief get last error code
			 *
			 * Read and reset the last error code that occured during a memory operation.
			 *
			 * \return Error code
			 */
			virtual MemoryError getLastError() = 0;

			/** \brief Enable/disable preservation of Ram
			 *
			 * Ram will be saved and restored for operations that can modify Ram content
			 * if set to true.
			 *
			 */
			virtual void setRamPreserveMode(bool enabled) = 0;

			/** \brief Current Ram preservation mode
			 *
			 * Get current setting for Ram preservation
			 *
			 * \return Current state (enabled or not)
			 */
			virtual bool getRamPreserveMode() const = 0;

			/** \brief Check voltage for flash programming
			 *
			 * Check internal and external voltage against minimum voltage
			 *
			 * \return Voltage sufficient
			 */
			virtual bool checkMinFlashVoltage() const = 0;
		};

	}
}
