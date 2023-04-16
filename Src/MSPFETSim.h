#pragma once
#include <climits>
#include <chrono>
#include "Toastbox/RuntimeError.h"
#include "Toastbox/USB.h"
#include "VirtualUSBDevice.h"
#include "Drivers/MSPDebugDriver.h"

class MSPFETSim {
public:
    MSPFETSim(MSPDebugDriver& msp) :
    _msp(msp),
    _usb(_usbDeviceInfo) {
    }
    
    // Copy constructor: illegal
    MSPFETSim(const MSPFETSim& x) = delete;
    MSPFETSim& operator=(const MSPFETSim& x) = delete;
    // Move constructor: illegal
    MSPFETSim(MSPFETSim&& x) = delete;
    MSPFETSim& operator=(MSPFETSim&& x) = delete;
    
    void run() {
        try {
            _usb.start();
        } catch (std::exception& e) {
            throw Toastbox::RuntimeError(
                "Failed to start VirtualUSBDevice: %s"                                  "\n"
                "Make sure:"                                                            "\n"
                "  - you're root"                                                       "\n"
                "  - the vhci-hcd kernel module is loaded: `sudo modprobe vhci-hcd`"    "\n",
                e.what()
            );
        }
        
        Main();
    }
    
    void stop() {
        _usb.stop();
    }
    
public:
    
    #include "Firmware/Main.h"
    
    struct _Msg {
        uint8_t data[512];
        size_t len = 0;
    };
    
    void _dequeueUSBRequest(std::chrono::milliseconds timeout=std::chrono::milliseconds::max()) {
        auto xfer = _usb.read(timeout);
        if (xfer) _handleUSBXfer(std::move(*xfer));
    }
    
    void _handleUSBXfer(VirtualUSBDevice::Xfer&& xfer) {
        // Endpoint 0
        if (xfer.ep == 0) _handleUSBXferEP0(std::move(xfer));
        // Other endpoints
        else _handleUSBXferEPX(std::move(xfer));
    }
    
    void _handleUSBXferEP0(VirtualUSBDevice::Xfer&& xfer) {
        using namespace Toastbox;
        
        const USB::SetupRequest req = xfer.setupReq;
        const uint8_t* payload = xfer.data.get();
        const size_t payloadLen = xfer.len;
        USB::CDC::LineCoding lineCoding = {};
        
        // Verify that this request is a Class request
        if ((req.bmRequestType&USB::RequestType::TypeMask) != USB::RequestType::TypeClass)
            throw Toastbox::RuntimeError("invalid request bmRequestType (TypeClass)");
        
        // Verify that this request is intended for the interface
        if ((req.bmRequestType&USB::RequestType::RecipientMask) != USB::RequestType::RecipientInterface)
            throw Toastbox::RuntimeError("invalid request bmRequestType (RecipientInterface)");
        
        switch (req.bmRequestType&USB::RequestType::DirectionMask) {
        case USB::RequestType::DirectionOut:
            switch (req.bRequest) {
            case USB::CDC::Request::SET_LINE_CODING: {
                if (payloadLen != sizeof(lineCoding))
                    throw Toastbox::RuntimeError("SET_LINE_CODING: payloadLen doesn't match sizeof(USB::CDC::LineCoding)");
                
                memcpy(&lineCoding, payload, sizeof(lineCoding));
                lineCoding = {
                    .dwDTERate      = Endian::HFL_U32(lineCoding.dwDTERate),
                    .bCharFormat    = Endian::HFL_U8(lineCoding.bCharFormat),
                    .bParityType    = Endian::HFL_U8(lineCoding.bParityType),
                    .bDataBits      = Endian::HFL_U8(lineCoding.bDataBits),
                };
                
                printf("SET_LINE_CODING:\n");
                printf("  dwDTERate: %08x\n", lineCoding.dwDTERate);
                printf("  bCharFormat: %08x\n", lineCoding.bCharFormat);
                printf("  bParityType: %08x\n", lineCoding.bParityType);
                printf("  bDataBits: %08x\n", lineCoding.bDataBits);
                return;
            }
            
            case USB::CDC::Request::SET_CONTROL_LINE_STATE: {
                const bool dtePresent = req.wValue&1;
                printf("SET_CONTROL_LINE_STATE:\n");
                printf("  dtePresent=%d\n", dtePresent);
                return;
            }
            
            case USB::CDC::Request::SEND_BREAK: {
                printf("SEND_BREAK:\n");
                return;
            }
            
            default:
                throw Toastbox::RuntimeError("invalid request (DirectionHostToDevice): %x", req.bRequest);
            }
        
        case USB::RequestType::DirectionIn:
            switch (req.bRequest) {
            case USB::CDC::Request::GET_LINE_CODING: {
                printf("GET_LINE_CODING\n");
                if (payloadLen != sizeof(lineCoding))
                    throw Toastbox::RuntimeError("SET_LINE_CODING: payloadLen doesn't match sizeof(USB::CDC::LineCoding)");
                _usb.write(USB::Endpoint::Default, &lineCoding, sizeof(lineCoding));
                return;
            }
            
            default:
                throw Toastbox::RuntimeError("invalid request (DirectionDeviceToHost): %x", req.bRequest);
            }
        
        default:
            throw Toastbox::RuntimeError("invalid request direction");
        }
    }
    
    void _handleUSBXferEPX(VirtualUSBDevice::Xfer&& xfer) {
        switch (xfer.ep) {
        case CDC0_OUTEP_ADDR:
//            printf("Endpoint CDC0_OUTEP_ADDR\n");
            _handleUSBXferData(std::move(xfer));
            break;
        case CDC1_OUTEP_ADDR:
//            printf("Endpoint CDC1_OUTEP_ADDR\n");
            break;
        default:
            throw Toastbox::RuntimeError("invalid endpoint: %02x", xfer.ep);
        }
    }
    
    void _handleUSBXferData(VirtualUSBDevice::Xfer&& xfer) {
//        printf("HOST: <");
//        for (size_t i=0; i<xfer.len; i++) {
//            printf(" %02x", xfer.data[i]);
//        }
//        printf(" >\n\n");
        
        _msgs.push_back({});
        _Msg& msg = _msgs.back();
        
        assert(sizeof(msg.data) >= xfer.len);
        memcpy(msg.data, xfer.data.get(), xfer.len);
        msg.len = xfer.len;
        
        // Notify firmware that USB data is available
        USBCDC_handleDataReceived(DEBUG_CHANNEL);
    }
    
    void _writeReply(const void* data, size_t len) {
//        printf("DEVICE: <");
//        const uint8_t* dataU8 = (const uint8_t*)data;
//        for (size_t i=0; i<len; i++) {
//            printf(" %02x", dataU8[i]);
//        }
//        printf(" >\n\n");
        
        _usb.write(CDC0_INEP_ADDR, data, len);
    }
    
    MSPDebugDriver& _msp;
    VirtualUSBDevice _usb;
    
    std::deque<_Msg> _msgs = {}; // Messages host->device
    
    static const inline Toastbox::USB::DeviceDescriptor* _DeviceDescriptor =
        (const Toastbox::USB::DeviceDescriptor*)Descriptor::abromDeviceDescriptor;
    
    static const inline Toastbox::USB::ConfigurationDescriptor** _ConfigurationDescriptors = Descriptor::configDescs;
    static const inline size_t _ConfigurationDescriptorsCount = std::size(Descriptor::configDescs);
    
    static const inline Toastbox::USB::StringDescriptor** _StringDescriptors = Descriptor::stringDescs;
    static const inline size_t _StringDescriptorsCount = std::size(Descriptor::stringDescs);
    
    static const inline VirtualUSBDevice::Info _usbDeviceInfo = {
        .deviceDesc             = _DeviceDescriptor,
        .deviceQualifierDesc    = nullptr,
        .configDescs            = _ConfigurationDescriptors,
        .configDescsCount       = _ConfigurationDescriptorsCount,
        .stringDescs            = _StringDescriptors,
        .stringDescsCount       = _StringDescriptorsCount,
        .throwOnErr             = true,
    };
};
