/*
 * UsbCdcIoChannel.cpp
 *
 * IOChannel via CDC (VCOM) over USB communication.
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

#include "UsbCdcIoChannel.h"
#include "logging/Logging.h"

#include <boost/asio/read.hpp>
#include <boost/asio/write.hpp>
#include <boost/asio/io_service.hpp>

#if defined(_WIN32) || defined(_WIN64)

extern "C" {
	#include <setupapi.h>
	#include <dbt.h>
}

#elif defined(__APPLE__)

#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/IOCFPlugIn.h>
#include <IOKit/usb/IOUSBLib.h>
#include <IOKit/IOMessage.h>
#include <mach/mach_port.h>
#include <IOKit/serial/IOSerialKeys.h>
#include <sys/stat.h>
#define MAXPATHLEN 128
#define MAXNAMELEN 64

#else

	#include <unistd.h>
	#include <boost/filesystem.hpp>

	using namespace boost::filesystem;

#endif

using namespace TI::DLL430;
using namespace std;
using namespace std::placeholders;
using namespace boost::asio;

#define  XOFF		0x13
#define  XON		0x11
#define  XMASK		0x10
#define  XMASK_OFF	0x03
#define  XMASK_ON	0x01
#define  XMASK_MASK	0x00


UsbCdcIoChannel::UsbCdcIoChannel(const PortInfo& portInfo)
 : UsbIoChannel(portInfo)
 , inputBuffer(260)
 , ioService(0)
 , port(0)
 , comState(ComStateRcv)
 , bytesReceived(0)
 , timerEvent(false)
 , readEvent(false)
 , cancelled(false)
{
	retrieveStatus();
}

UsbCdcIoChannel::~UsbCdcIoChannel()
{
	this->cleanup();
}

void UsbCdcIoChannel::createCdcPortList(const uint16_t vendorId, const uint16_t productId, PortMap& portList)
{
#if defined(_WIN32) || defined(_WIN64)
	stringstream cdcIdStream;
	cdcIdStream << hex << setfill('0') << "USB\\VID_" << setw(4) << vendorId << "&PID_" << setw(4) << productId;

	const int BUFFER_SIZE = 128;

	HDEVINFO hDevInfo = ::SetupDiGetClassDevs(nullptr, "USB", 0, DIGCF_PRESENT | DIGCF_ALLCLASSES );
	SP_DEVINFO_DATA devInfoData;
	devInfoData.cbSize = sizeof(SP_DEVINFO_DATA);

	for (int i = 0; ::SetupDiEnumDeviceInfo(hDevInfo, i, &devInfoData); ++i )
	{
		char deviceId[BUFFER_SIZE] = {0};
		BOOL result = ::SetupDiGetDeviceInstanceId(hDevInfo, &devInfoData, deviceId, BUFFER_SIZE, nullptr);

		//not TI and/or not CDC
		if (result && string(deviceId).find(cdcIdStream.str()) != string::npos)
		{
			DWORD propertyType = 0;
			BYTE property[BUFFER_SIZE] = {0};

			::SetupDiGetDeviceRegistryProperty(hDevInfo, &devInfoData, SPDRP_FRIENDLYNAME, &propertyType, property, BUFFER_SIZE, nullptr);

			stringstream sstr;
			for (int k = 0; k < BUFFER_SIZE && property[k] != 0; ++k)
			{
				sstr << property[k];
			}

			const size_t idBegin = sstr.str().find_last_of('(') + 1;
			const size_t idEnd = sstr.str().find_last_of(')');
			assert(idEnd > idBegin);

			const string name = sstr.str().substr(idBegin, idEnd - idBegin);

			if ((name[0] && (sstr.str().compare(0, 19, "MSP Debug Interface") == 0 ))|| (name[0] && (sstr.str().compare(0, 19, "MSP-FET430UIF - CDC") == 0 )))
			{
				PortInfo portInfo(name, string("\\\\.\\")+name, PortInfo::CDC, retrieveSerialFromId(deviceId));
				if (name[0] && (sstr.str().compare(0, 19, "MSP Debug Interface") == 0 ))
				{
					portInfo.useFlowControl = false;
					portInfo.useCrc = false;
				}
				else if (name[0] && (sstr.str().compare(0, 19, "MSP-FET430UIF - CDC") == 0 ))
				{
					portInfo.useFlowControl = true;
					portInfo.useCrc = true;
				}

				//if (open)
				{
					portInfo.status = UsbCdcIoChannel(portInfo).getStatus();
				}
				portList[portInfo.name] = portInfo;
			}
		}
	}
	::SetupDiDestroyDeviceInfoList(hDevInfo);//free resources

#elif defined(__APPLE__)
	CFMutableDictionaryRef matchingDict;
	kern_return_t kernResult;
	io_iterator_t iterator;

	matchingDict = IOServiceMatching(kIOSerialBSDServiceValue);
	CFDictionarySetValue(matchingDict, CFSTR(kIOSerialBSDTypeKey), CFSTR(kIOSerialBSDAllTypes));
	kernResult = IOServiceGetMatchingServices(kIOMasterPortDefault, matchingDict, &iterator);

	if (kernResult != KERN_SUCCESS) {
		return;
	}

	for (;;)
	{
		io_service_t device = IOIteratorNext(iterator);
		if (device == 0)
		{
			if (!IOIteratorIsValid(iterator))
			{
				/*
				 * Apple documentation advises resetting the iterator if
				 * it should become invalid during iteration.
				 */
				IOIteratorReset(iterator);
				device = IOIteratorNext(iterator);
				if (device == 0)
				{
					break;
				}
			}
			else
			{
				break;
			}
		}

		CFTypeRef bsdPathAsCFString = nullptr;
		CFTypeRef vendorIdAsCFNumber = nullptr;
		CFTypeRef productIdAsCFNumber = nullptr;
		CFTypeRef ttyDeviceAsCFString = nullptr;
		CFTypeRef interfaceNumberAsCFNumber = nullptr;

		char ttyDevice[MAXNAMELEN];
		SInt32 interfaceNumber;
		char path[MAXPATHLEN];
		SInt32 vID = 0;
		SInt32 pID = 0;

		// Get the name of the modem's callout device
		bsdPathAsCFString = IORegistryEntryCreateCFProperty(device, CFSTR(kIOCalloutDeviceKey),
		                                                    kCFAllocatorDefault, 0);

		ttyDeviceAsCFString = IORegistryEntryCreateCFProperty(device, CFSTR(kIOTTYDeviceKey),
		                                                      kCFAllocatorDefault, 0);

		io_name_t name;
		IORegistryEntryGetName(device, name);

		// wander up the hierarchy until we find the level that can give us the
		// vendor/product IDs and the product name, if available
		io_registry_entry_t parent;
		kern_return_t kernResult = IORegistryEntryGetParentEntry(device, kIOServicePlane, &parent);
		IOObjectRelease(device);

        while ( kernResult == KERN_SUCCESS && ( !vendorIdAsCFNumber || !productIdAsCFNumber || !interfaceNumberAsCFNumber) )
		{
			if (!vendorIdAsCFNumber)
			{
				vendorIdAsCFNumber = IORegistryEntrySearchCFProperty(parent,
				                                                     kIOServicePlane,
				                                                     CFSTR(kUSBVendorID),
				                                                     kCFAllocatorDefault, 0);
			}

			if (!productIdAsCFNumber)
			{
				productIdAsCFNumber = IORegistryEntrySearchCFProperty(parent,
				                                                      kIOServicePlane,
				                                                      CFSTR(kUSBProductID),
				                                                      kCFAllocatorDefault, 0);
			}

			if (!interfaceNumberAsCFNumber)
			{
				interfaceNumberAsCFNumber = IORegistryEntrySearchCFProperty(parent,
				                                                            kIOServicePlane,
				                                                            CFSTR(kUSBInterfaceNumber),
				                                                            kCFAllocatorDefault, 0);
			}

			io_registry_entry_t oldparent = parent;
			kernResult = IORegistryEntryGetParentEntry(parent, kIOServicePlane, &parent);
			IOObjectRelease(oldparent);
		}

		if (interfaceNumberAsCFNumber)
		{
			CFNumberGetValue((CFNumberRef)interfaceNumberAsCFNumber, kCFNumberSInt32Type, &interfaceNumber);
			CFRelease(interfaceNumberAsCFNumber);
			if (interfaceNumber != 1)
			{
				continue;
			}
		}

		if (ttyDeviceAsCFString)
		{
			CFStringGetCString((CFStringRef)ttyDeviceAsCFString, ttyDevice, PATH_MAX, kCFStringEncodingUTF8);
			CFRelease(ttyDeviceAsCFString);
		}

		if (bsdPathAsCFString)
		{
			CFStringGetCString((CFStringRef)bsdPathAsCFString, path, PATH_MAX, kCFStringEncodingUTF8);
			CFRelease(bsdPathAsCFString);
		}

		if (vendorIdAsCFNumber)
		{
			CFNumberGetValue((CFNumberRef)vendorIdAsCFNumber, kCFNumberSInt32Type, &vID);
			CFRelease(vendorIdAsCFNumber);
		}

		if (productIdAsCFNumber)
		{
			CFNumberGetValue((CFNumberRef)productIdAsCFNumber, kCFNumberSInt32Type, &pID);
			CFRelease(productIdAsCFNumber);
		}

		if ((vID == vendorId) && (pID == productId))
		{
			PortInfo portInfo(ttyDevice, path, PortInfo::CDC);
			if (productId == 0x0010)
			{
				portInfo.useFlowControl = true;
				portInfo.useCrc = true;
			}
			portInfo.status = UsbCdcIoChannel(portInfo).getStatus();
			portList[portInfo.name] = portInfo;
		}
	}
#else
	stringstream cdcIdStream;
	cdcIdStream << hex << setfill('0') << "usb:v" << setw(4) << vendorId << "p" << setw(4) << productId;

	path p("/sys/class/tty/");
	if (exists(p) && is_directory(p))
	{
		const directory_iterator end;
		for (directory_iterator it(p); it != end; ++it)
		{
			string dir = it->path().string();
			if (dir.find("ttyACM") != string::npos)
			{
				string modalias;
				int interfaceNumber = -1;

				std::ifstream modAliasStream((it->path()/"device/modalias").string().c_str());
				modAliasStream >> modalias;

				std::ifstream ifNumStream((it->path()/"device/bInterfaceNumber").string().c_str());
				ifNumStream >> interfaceNumber;
				if (modalias.find(cdcIdStream.str()) == 0 && interfaceNumber == 0)
				{
					const string filename = it->path().filename().string();
					const string portPath = string("/dev/") + filename;

					PortInfo portInfo(filename, portPath, PortInfo::CDC);

					if (productId == 0x0010)
					{
						portInfo.useFlowControl = true;
						portInfo.useCrc = true;
					}

					//if (open)
					{
						portInfo.status = UsbCdcIoChannel(portInfo).getStatus();
					}
					portList[portInfo.name] = portInfo;
				}
			}
		}
	}
#endif
}


void UsbCdcIoChannel::enumeratePorts (PortMap& portList, bool open)
{
	createCdcPortList(0x2047, 0x0013, portList); //eZ-FET
	createCdcPortList(0x2047, 0x0014, portList); //MSP-FET
	createCdcPortList(0x2047, 0x0010, portList); //UIF
}

std::string UsbCdcIoChannel::retrieveSerialFromId(const std::string& id)
{
#if defined(_WIN32) || defined(_WIN64)
	const size_t idBegin = id.find_last_of('\\') + 1;
	return id.substr(idBegin, 16);
#else
	const size_t begin = id.find_last_of('_') + 1;
	const size_t end = id.find_last_of('-');
	return id.substr( begin, end - begin );
#endif
}

bool UsbCdcIoChannel::openPort()
{
	ioService = new boost::asio::io_service;
	port = new boost::asio::serial_port(*ioService);
	timer = new boost::asio::deadline_timer(*ioService);

	boost::system::error_code ec;
	ec = port->open(portInfo.path, ec);
	if (ec != boost::system::error_condition(boost::system::errc::success))
	{
		int retry = 5;
		while ((ec != boost::system::error_condition(boost::system::errc::success))
			&& (--retry ))
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
			ec = port->open(portInfo.path, ec);
		}

		if (ec == boost::system::error_condition(boost::system::errc::permission_denied))
		{
			portInfo.status = PortInfo::inUseByAnotherInstance;
		}
		if (ec != boost::system::error_condition(boost::system::errc::success))
		{
			close();
			return false;
		}
	}
	return true;
}

void UsbCdcIoChannel::retrieveStatus()
{
	portInfo.status = PortInfo::freeForUse;

	if (!isOpen())
	{
		openPort();
		//Seeing issues on some platforms (eg. Ubuntu) when port is immediately closed again
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		close();
	}
}


bool UsbCdcIoChannel::open()
{
	if (!isOpen() && !openPort())
	{
		return false;
	}

	portInfo.status = PortInfo::freeForUse;

	try
	{
		const int baudrate = 460800;

#if defined(__APPLE__)
		// Vanilla boost does not provide method to set non-standard baudrates,
		// so we have to set it low-level
		if (ioctl (port->native_handle(), _IOW('T', 2, speed_t), &baudrate, 1) < 0)
		{
			return false;
		}
#else
		port->set_option( serial_port::baud_rate( baudrate ) );
#endif
		port->set_option( serial_port::flow_control( serial_port::flow_control::none ) );
		port->set_option( serial_port::parity( serial_port::parity::none ) );
		port->set_option( serial_port::stop_bits( serial_port::stop_bits::one ) );
		port->set_option( serial_port::character_size(8) );
	}
	catch (const boost::system::system_error&)
	{
		return false;
	}

	return true;
}

void UsbCdcIoChannel::cleanup()
{
	boost::system::error_code ec;
	if (isOpen())
	{
		port->close(ec);
	}
	delete timer;
	timer = 0;
	delete port;
	port = 0;
	delete ioService;
	ioService = 0;
}

bool UsbCdcIoChannel::close()
{
	cleanup();
	return true;
}


bool UsbCdcIoChannel::isOpen() const
{
	return port && port->is_open();
}


void UsbCdcIoChannel::cancel()
{
	cancelled = true;

	boost::system::error_code ec;
	if (timer && timer->expires_from_now(boost::posix_time::milliseconds(0), ec) > 0)
	{
		timer->async_wait(std::bind(&UsbCdcIoChannel::onTimer, this, std::placeholders::_1));
	}
}


void UsbCdcIoChannel::setTimer(uint32_t duration)
{
	timerEvent = false;

	if (timer)
	{
		boost::system::error_code ec;
		timer->expires_from_now(boost::posix_time::milliseconds(duration), ec);
		timer->async_wait(bind(&UsbCdcIoChannel::onTimer, this, std::placeholders::_1));
	}
}


void UsbCdcIoChannel::startRead(size_t offset, size_t numBytes)
{
	bytesReceived = 0;
	readEvent = false;
	async_read(*port, buffer(&inputBuffer[offset], numBytes), bind(&UsbCdcIoChannel::onRead, this, std::placeholders::_1, std::placeholders::_2));
}


void UsbCdcIoChannel::onTimer(const boost::system::error_code& ec)
{
	timerEvent = (ec != error::operation_aborted);
}


void UsbCdcIoChannel::onRead(const boost::system::error_code& ec, size_t numBytes)
{
	readEvent = (ec != error::operation_aborted);
	bytesReceived = numBytes;
}


size_t UsbCdcIoChannel::read(HalResponse& resp)
{
	if (!isOpen())
		return 0;

	size_t actSize = 0;
	size_t expSize = 1;

	setTimer(1000);
	startRead(0, expSize);

	boost::system::error_code ec;

	while (ioService->run_one(ec))
	{
		if (readEvent)
		{
			if (bytesReceived > 0)
			{
				if (actSize == 0)
					expSize = inputBuffer[0] + ( (inputBuffer[0] & 0x01) ? 3 : 4);

				actSize += bytesReceived;

				if (actSize == expSize)
				{
					timer->cancel(ec);
					break;
				}
			}

			startRead(actSize, expSize - actSize);
		}

		else if (timerEvent)
		{
			if (wasUnplugged() || cancelled)
			{
				cancelled = false;
				port->cancel(ec);
				break;
			}

			setTimer(1000);
		}

		if (ioService->stopped())
		{
			ioService->reset();
		}
	}

	//Let cancelled tasks finish
	ioService->run(ec);
	ioService->reset();


	if (actSize == expSize)
	{
		processMessage(actSize, resp);
		return actSize;
	}
	return 0;
}


bool UsbCdcIoChannel::wasUnplugged()
{
#if defined(_WIN32) || defined(_WIN64)
	boost::system::error_code ec;
	ec = serial_port(*ioService).open(portInfo.path, ec);

	if (ec == boost::system::error_condition(boost::system::errc::no_such_file_or_directory))
	{
		comState = ComStateDisconnect;
	}
#else
	/*
	 * Workaround for El Capitan with UIF: the above branch triggers sending of SET_LINE_CODING
	 * Do not trigger SET_LINE_CODING here
	 */

	/* Also workaround for Ubuntu 16 with UIF: the above branch triggers sending of SetControlLineState, which we don't want here */

	struct stat dev;
	if (stat(portInfo.path.c_str(), &dev) != 0)
	{
		comState = ComStateDisconnect;
	}
#endif
	return comState == ComStateDisconnect;
}


void UsbCdcIoChannel::processMessage(size_t msgSize, HalResponse& resp)
{
#ifdef DB_PRINT
	Logging::DefaultLogger().PrintReceiveBuffer(&inputBuffer[0], static_cast<long>(msgSize));
#endif // DB_PRINT

	if (portInfo.useCrc)
	{
		const uint16_t expCrc = createCrc(&inputBuffer[0]);
		const uint16_t actCrc = (inputBuffer[msgSize-1] << 8) | inputBuffer[msgSize-2];
		if (actCrc != expCrc)
		{
			resp.setError(HalResponse::Error_CRC);
		}
	}
	resp.setType(inputBuffer[1]);
	resp.setId(inputBuffer[2] & 0x7f); //Don't mask async bit (0x40)
	resp.setIsComplete(inputBuffer[2]);

	if (msgSize >= 2)
	{
		resp.append(&inputBuffer[1], inputBuffer[0]);
	}
}


enum ComState UsbCdcIoChannel::poll()
{
	return comState;
}


size_t UsbCdcIoChannel::write(const uint8_t* payload, size_t len)
{
	if (!isOpen())
		return 0;

	const size_t ret_len = len;

	uint8_t report[256] = {0};

	if (payload)
		memcpy(report, payload, len);

	// test for fill byte
	if (!(report[0] & 0x01))
		report[len++] = 0x00;

	if (portInfo.useCrc)
	{
		//create crc and append it to data
		uint16_t crc = createCrc(report);

		report[len++] = crc & 0x00ff;
		report[len++] = (crc & 0xff00) >> 8;
	}

	size_t n_write = 0;
	uint8_t send_buf[512];

	if (portInfo.useFlowControl)
	{
		//mask XOFF, XON and MASK in data stream
		size_t j = 0;

		for (size_t i = 0; i < len; i++)
		{
			const uint8_t ch = report[i];
			switch (ch)
			{
			case XOFF:
			case XON:
			case XMASK:
				send_buf[j] = XMASK;
				j++;
				send_buf[j] = ch & 0x3;
				break;
			default:
				send_buf[j] = ch;
			}
			j++;
		}
		n_write = j;
	}
	else
	{
		n_write = len;
		memcpy(send_buf, report, n_write);
	}

#ifdef DB_PRINT
	Logging::DefaultLogger().PrintSendBuffer(send_buf, static_cast<long>(n_write));
#endif // DB_PRINT

	boost::system::error_code ec;
	const size_t nWritten = boost::asio::write(*port, buffer(send_buf, n_write), transfer_all(), ec);

	if (nWritten != n_write)
	{
		return 0;
	}

	return ret_len;
}

const char* UsbCdcIoChannel::getName() const
{
	return portInfo.name.c_str();
}

string UsbCdcIoChannel::getSerial() const
{
	return portInfo.serial;
}

PortInfo::Status UsbCdcIoChannel::getStatus() const
{
 	return portInfo.status;
}
