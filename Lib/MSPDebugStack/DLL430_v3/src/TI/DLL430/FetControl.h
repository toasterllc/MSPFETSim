/*
 * FetControl.h
 *
 * Flash Emulation Tool Control.
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

#include "PortInfo.h"
#include "WatchdogControl.h"
#include "IoChannel.h"
#include "FetControlThread.h"
#include "HalResponseHandler.h"

#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>

namespace TI
{
	namespace DLL430
	{

		class HalCommand;

		class FetControl
		{
		public:
			explicit FetControl (IoChannel* channel);
			~FetControl ();

			FetControl(const FetControl&) = delete;
			FetControl& operator=(const FetControl&) = delete;

			bool send (HalExecCommand& command);
			bool sendData(const std::vector<uint8_t>& data);

			void shutdown();

			bool resetCommunication();
			bool resetFetState();

			bool kill (uint8_t id);
			bool pauseLoopCmd (uint8_t id);
			bool resumeLoopCmd (uint8_t id);

			uint8_t createResponseId (bool reserveId = false);
			bool registerResponseHandler (uint8_t id, HalResponseHandlerPtr h);
			void unregisterResponseHandler (uint8_t id, HalResponseHandlerPtr h);
			void unregisterResponseHandler (HalResponseHandlerPtr h);

			HalResponseHandlerPtr findResponseHandler (uint8_t id);
			HalResponseHandlerPtr findResponseHandler (HalResponseHandlerPtr h);
			void clearResponse();

			uint8_t getNextId(uint8_t ref);

			std::string getSerial() const;
			void provideSystemErrorMsg(HalResponse& resp);
			void provideSystemConnectMsg(bool connect);

			typedef std::function<void(uint32_t)> NotifyCallback;
			void addSystemNotifyCallback(const NotifyCallback& notifyCallback);

			bool hasCommunication() const;

			const std::vector<uint8_t>* getHwVersion();
			const std::vector<uint8_t>* getSwVersion() const;

			uint16_t getFetCoreVersion() const;

			uint16_t getFetToolId() const;
			uint16_t getHilVersion() const;
			uint16_t getDcdcLayerVersion() const;
			uint16_t getDcdcSubMcuVersion() const;
			uint16_t getFetComChannelVersion() const;

			uint16_t getFetCoreCrc() const;
			uint16_t getFetHalCrc() const;
			uint16_t getFetHilCrc() const;
			uint16_t getFetDcdcCrc() const;
			uint16_t getFetComChannelCrc() const;

			uint16_t getFetFpgaVersion() const;

			bool communication;

		private:
			IoChannel* channel;
			std::vector<uint8_t> fetSwVersion;
			std::vector<uint8_t> fetHwVersion;

			uint16_t fetCoreVersion;
			uint16_t fetHilVersion;
			uint16_t fetFpgaVersion;
			uint16_t fetToolId;

			uint16_t fetDcdcLayerVersion;
			uint16_t fetDcdcSubMcuVersion;
			uint16_t fetComChannelVersion;

			uint16_t fetHilCrc;
			uint16_t fetHalCrc;
			uint16_t fetDcdcCrc;
			uint16_t fetCoreCrc;
			uint16_t fetComChannelCrc;

			uint8_t currentId;

			friend class FetControlThread;
			FetControlThread* reader;

			typedef std::map<uint32_t, HalResponseHandlerPtr> ResponseHandlerTable;
			ResponseHandlerTable responseHandlers;

			/* this mutex protects the reponseHandlers list, so iterators keep stable */
			boost::mutex rhMutex;
			boost::mutex CriMutex;
			boost::recursive_mutex sendMutex;

			NotifyCallback lNotifyCallback;

			std::map<uint8_t, bool> reservedIds;
			std::map<uint16_t, std::vector<uint8_t> > fetHardwareVersions;


		};

	};
};
