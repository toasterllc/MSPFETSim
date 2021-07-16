/*
 * EnergyTraceProcessor.h
 *
 * Process incoming serial data into EnergyTrace records
 *
 * Copyright (c) 2007 - 2016 Texas Instruments Incorporated - http://www.ti.com/
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

#pragma once

#include "EnergyTraceProcessor.h"
#include <DoubleBuffer.h>

//#define ETLOG

namespace TI
{
	namespace DLL430
	{
        class IDeviceHandle;
		class EnergyTraceProcessorId8 : public EnergyTraceProcessor
		{
		public:
			/**
			 * \brief Constructor creates the two buffers
			 * \param size The desired size of the buffers
			 */
			EnergyTraceProcessorId8(uint32_t calibrationPoints, size_t dataSize, IDeviceHandle *devHandle);
			~EnergyTraceProcessorId8();

			/**
			 * \brief Resets the internal state of the processor
			 */
			void Reset();

			/**
			 * \brief Add data to be processed
			 * \param data The data element to be written
			 * \return Indicates whether new output data is available for reading
			 */
			bool AddData(void *data, size_t size);

			/**
			 * \brief Get the pointer for reading data from the buffer
			 * \return A pointer to the current read buffer
			 */
			void* GetReadBufferPtr();

			/**
			 * \brief Get the buffer size
			 * \return the size
			 */
			size_t GetReadBufferSize();
#ifdef ETLOG
			void Log(char * str);
			void LogCalls(char * str);
#endif

		private:
			DoubleBuffer<EnergyRecordEt8> mBuffer; ///< The Double buffer that stores all the records
			uint32_t mPrevCurrent;
			double mEnergyMicroWsec;
			double mEnergyMicroWsecAdder;
			double mAccumulatedNDiv;

			double mAccumulatedT;
			uint32_t mAccumulatedN;
			uint32_t mCurrent;
			bool mCurrentValid;
			IDeviceHandle *mDevice;

#ifdef ETLOG
			char logStr [500];
			FILE *filePtr;
			FILE *filePtrCalls;
#endif
		};
	}
}
