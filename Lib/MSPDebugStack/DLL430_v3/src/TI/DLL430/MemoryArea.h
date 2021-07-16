/*
 * MemoryArea.h
 *
 * Interface for memory access
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


namespace TI
{
	namespace DLL430
	{
		enum MemoryError { MEMORY_NO_ERROR, MEMORY_READ_ERROR, MEMORY_WRITE_ERROR, MEMORY_LOCKED_ERROR, MEMORY_UNLOCK_ERROR };

		/** \brief memory area management */
		class MemoryArea
		{
		public:
			enum Name { None, Main, Info, Bsl, BootCode, Ram, UsbRam, Cpu, Eem, Peripheral8bit, Peripheral16bit, Lcd, IrVec, Lib, LeaPeripheral, LeaRam, MidRom, TinyRam, UssPeripheral};

			virtual ~MemoryArea() {}

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
			 * \param mode for selection of erase mode all/segments
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

			/** \brief get the memory start address
			 *
			 * \return the memory start address value
			 */
			virtual uint32_t getStart() const = 0;

			/** \brief get the memory end address
			 *
			 * \return the memory end address value
			 */
			virtual uint32_t getEnd() const = 0;

			/** \brief get the memory size
			 *
			 * \return the memory size value
			 */
			virtual uint32_t getSize() const = 0;

			/** \brief get the memory segment size
			 *
			 * This value only makes sense if the memory is a flash.
			 *
			 * \return the memory segment size value
			 */
			virtual uint32_t getSegmentSize() const = 0;

			/** \brief get number of memory banks
			 *
			 * This value only makes sense if the memory is a flash.
			 *
			 * \return the memory segment size value
			 */
			virtual uint32_t getBanks() const = 0;

			/** \brief check if a memory is mapped to global address space
			 *
			 * \return true if the memory is mapped, else false
			 */
			virtual bool isMapped() const = 0;

			/** \brief get the name of the area
			 *
			 * \return the memory area name
			 */
			virtual MemoryArea::Name getName() const = 0;

			/** \brief set whether the area is currently accessible
			*
			*/
			virtual void setAccessible(bool accessible) = 0;

			/** \brief query if area is currently accessible
			*
			* \return true if accessible, else false
			*/
			virtual bool isAccessible() const = 0;

			virtual bool fillCache() = 0;
			/** \brief query if area is currently accessible
			*
			* \return true if cache was filled 
			*/
			virtual bool flushCache() const = 0;
			/** \brief query if area is currently accessible
			*
			* \return true if cache was flushed and written back to target device
			*/
			virtual bool wakeup() = 0;
			/** \brief query if area is currently accessible
			*
			* \return true if cache was flushed and written back to target device
			*/

			virtual bool isProtectionEnabled() = 0;
		};

	}
}
