/*
 * DoubleBuffer.h
 *
 * Template class for doing double-buffering
 *
 * Copyright (C) 2007 - 2012 Texas Instruments Incorporated - http://www.ti.com/
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
		template<typename T>
		class DoubleBuffer
		{
		public:
			/**
			 * \brief Constructor creates the two buffers
			 * \param size The desired size of the buffers
			 */
			DoubleBuffer(size_t size)
			 : mSize(size)
			{
				mBuffers[0] = new T[size];
				mBuffers[1] = new T[size];
				Reset();
			}

			~DoubleBuffer()
			{
				delete[] mBuffers[0];
				delete[] mBuffers[1];
			}

			/**
			 * \brief Resets the current state of the double buffer
			 */
			void Reset()
			{
				mCurrentBufferIndex = 0;
				mCurrentBufferLevel = 0;
				mDataAvailable = false;
			}

			/**
			 * \brief Add data to the buffer and indicate that a swap has happened
			 * \param data The data element to be written
			 * \return Indicates whether a swap between buffers has occured and the user
			 *		   may start reading from the other buffer
			 */
			bool AddData(const T &data)
			{
				mBuffers[mCurrentBufferIndex][mCurrentBufferLevel++] = data;

				return doSwap();
			}

			/**
			 * \brief Add an array of data to the buffer and indicate that a swap has happened
			 * \param data The data element to be written
			 * \return Indicates whether a swap between buffers has occured and the user
			 *		   may start reading from the other buffer
			 */
			bool AddData(const T *data, size_t size = 1)
			{
				bool retVal = false;

				size_t copySize;
				size_t sizeInBytes = size * sizeof(T);

				while (sizeInBytes)
				{
					if ((mCurrentBufferLevel + sizeInBytes) > mSize)
					{
						copySize = mSize - mCurrentBufferLevel;
					}
					else
					{
						copySize = sizeInBytes;
					}
					memcpy(&mBuffers[mCurrentBufferIndex][mCurrentBufferLevel], data, copySize);

					mCurrentBufferLevel += copySize;
					sizeInBytes -= copySize;

					retVal = retVal || doSwap();
				}

				return retVal;
			}

			/**
			 * \brief Get the pointer for reading data from the buffer
			 * \return A pointer to the current read buffer
			 */
			T* GetReadBufferPtr()
			{
				T* retVal = 0;

				if (mDataAvailable)
				{
					retVal = mBuffers[(mCurrentBufferIndex + 1) % 2];
				}

				return retVal;
			}

			/**
			 * \brief Get the buffer size
			 * \return the size
			 */
			size_t GetBufferSize() const
			{
				return mSize;
			}

		private:
			/**
			 * \brief Swap buffers if needed
			 */
			bool doSwap()
			{
				bool retVal = false;
				if (mCurrentBufferLevel >= mSize)
				{
					// swap buffers
					mCurrentBufferIndex = (mCurrentBufferIndex + 1) % 2;
					mCurrentBufferLevel = 0;
					mDataAvailable = true;
					retVal = true;
				}
				return retVal;
			}

			T* mBuffers[2];						/**< Actual buffers where data is stored */
			unsigned char mCurrentBufferIndex;	/**< Indicates which buffer is currently being written to */
			size_t mCurrentBufferLevel;			/**< Inidcates the fill-level of the current write buffer */
			size_t mSize;						/**< The size of the double buffer */
			bool mDataAvailable;				/**< Indicates whether data is available for reading */
		};
	}
}
