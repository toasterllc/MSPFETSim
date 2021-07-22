#pragma once
#include <climits>
#include <chrono>
#include "VirtualUSBDevice.h"
#include "MSPInterface.h"

class MSPProbeSim {
public:
    MSPProbeSim(MSPInterface& msp) :
    _msp(msp),
    _usb(_usbDeviceInfo) {
    }
    
    // Copy constructor: illegal
    MSPProbeSim(const MSPProbeSim& x) = delete;
    MSPProbeSim& operator=(const MSPProbeSim& x) = delete;
    // Move constructor: illegal
    MSPProbeSim(MSPProbeSim&& x) = delete;
    MSPProbeSim& operator=(MSPProbeSim&& x) = delete;
    
    void run() {
        try {
            _usb.start();
        } catch (std::exception& e) {
            throw RuntimeError(
                "Failed to start VirtualUSBDevice: %s"                                  "\n"
                "Make sure:"                                                            "\n"
                "  - you're root"                                                       "\n"
                "  - the vhci-hcd kernel module is loaded: `sudo modprobe vhci-hcd`"    "\n",
                e.what()
            );
        }
        
//        ResetFirmware();
//        BIOS_InitCom();
//        
//        for (;;) {
//            // While V3OP_Scheduler has work to do (return=true), let it run for many iterations
//            // before checking for USB data (without blocking).
//            // When V3OP_Scheduler out of work (return=false), wait indefinitely for USB data.
//            bool serviced = true;
//            for (int i=0; i<100 && serviced; i++) {
//                serviced = V3OP_Scheduler();
//            }
//            _dequeueUSBRequest(serviced ? 1ms : 0ms);
//        }
    }
    
    void stop() {
        _usb.stop();
    }
    
public:
    
    #include "Firmware/Firmware.h"
    
    struct _Msg {
        uint8_t data[512];
        size_t len = 0;
    };
    
//    using _Rep = _Msg;
    
    void _dequeueUSBRequest(std::chrono::milliseconds timeout=0ms) {
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
        const USB::SetupRequest req = xfer.setupReq;
        const uint8_t* payload = xfer.data.get();
        const size_t payloadLen = xfer.len;
        USB::CDC::LineCoding lineCoding = {};
        
        // Verify that this request is a Class request
        if ((req.bmRequestType&USB::RequestType::TypeMask) != USB::RequestType::TypeClass)
            throw RuntimeError("invalid request bmRequestType (TypeClass)");
        
        // Verify that this request is intended for the interface
        if ((req.bmRequestType&USB::RequestType::RecipientMask) != USB::RequestType::RecipientInterface)
            throw RuntimeError("invalid request bmRequestType (RecipientInterface)");
        
        switch (req.bmRequestType&USB::RequestType::DirectionMask) {
        case USB::RequestType::DirectionHostToDevice:
            switch (req.bRequest) {
            case USB::CDC::Request::SET_LINE_CODING: {
                if (payloadLen != sizeof(lineCoding))
                    throw RuntimeError("SET_LINE_CODING: payloadLen doesn't match sizeof(USB::CDC::LineCoding)");
                
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
                throw RuntimeError("invalid request (DirectionHostToDevice): %x", req.bRequest);
            }
        
        case USB::RequestType::DirectionDeviceToHost:
            switch (req.bRequest) {
            case USB::CDC::Request::GET_LINE_CODING: {
                printf("GET_LINE_CODING\n");
                if (payloadLen != sizeof(lineCoding))
                    throw RuntimeError("SET_LINE_CODING: payloadLen doesn't match sizeof(USB::CDC::LineCoding)");
                _usb.write(USB::Endpoint::DefaultIn, &lineCoding, sizeof(lineCoding));
                return;
            }
            
            default:
                throw RuntimeError("invalid request (DirectionDeviceToHost): %x", req.bRequest);
            }
        
        default:
            throw RuntimeError("invalid request direction");
        }
    }
    
    void _handleUSBXferEPX(VirtualUSBDevice::Xfer&& xfer) {
        switch (xfer.ep) {
        case CDC0_OUTEP_ADDR:
            printf("Endpoint CDC0_OUTEP_ADDR\n");
            _handleUSBXferData(std::move(xfer));
            break;
        case CDC1_OUTEP_ADDR:
            printf("Endpoint CDC1_OUTEP_ADDR\n");
            break;
        default:
            throw RuntimeError("invalid endpoint: %02x", xfer.ep);
        }
        
//        _usb.reply(msg, nullptr, 0);
    }
    
//    void _handleInXfer(VirtualUSBDevice::Xfer&& xfer) {
//        _inXfers.push_back(std::move(xfer));
//        // If a reply is already available, send it now
//        _replyIfPossible();
//    }
    
    void _handleUSBXferData(VirtualUSBDevice::Xfer&& xfer) {
        printf("_handleUSBXferData: <");
        for (size_t i=0; i<xfer.len; i++) {
            printf(" %02x", xfer.data[i]);
        }
        printf(" >\n\n");
        
        _msgs.push_back({});
        _Msg& msg = _msgs.back();
        
        assert(sizeof(msg.data) >= xfer.len);
        memcpy(msg.data, xfer.data.get(), xfer.len);
        msg.len = xfer.len;
        
        // Notify firmware that USB data is available
        USBCDC_handleDataReceived(DEBUG_CHANNEL);
    }
    
    void _writeReply(const void* data, size_t len) {
        _usb.write(CDC0_INEP_ADDR, data, len);
//        _reps.push_back({});
//        _Rep& rep = _reps.back();
//        
//        assert(sizeof(rep.data) >= len);
//        memcpy(rep.data, data, len);
//        rep.len = len;
//        
//        // If the host requested data, send it now
//        _replyIfPossible();
    }
    
//    void _replyIfPossible() {
//        // Match in-xfers with replies, and call `_usb.reply()` with the pairs,
//        // until we run out of one (or both)
//        while (!_inXfers.empty() && !_reps.empty()) {
//            VirtualUSBDevice::Xfer& xfer = _inXfers.front();
//            _Rep& rep = _reps.front();
//            _usb.reply(xfer, rep.data, rep.len);
//            
//            _inXfers.pop_front();
//            _reps.pop_front();
//        }
//    }
    
    MSPInterface& _msp;
    VirtualUSBDevice _usb;
    
    std::deque<_Msg> _msgs = {}; // Messages host->device
//    std::deque<_Rep> _reps; // Replies device->host
//    std::deque<VirtualUSBDevice::Xfer> _inXfers; // USB host requests for data from device
    
    static const inline USB::DeviceDescriptor* _DeviceDescriptor =
        (const USB::DeviceDescriptor*)Descriptor::abromDeviceDescriptor;
    
    static const inline USB::ConfigurationDescriptor** _ConfigurationDescriptors = Descriptor::configDescs;
    static const inline size_t _ConfigurationDescriptorsCount = std::size(Descriptor::configDescs);
    
    static const inline USB::StringDescriptor** _StringDescriptors = Descriptor::stringDescs;
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
