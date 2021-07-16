#pragma once
#include <climits>
#include "VirtualUSBDevice.h"
#include "ti/Descriptor.h"
#include "ti/USBTypes.h"

class MSPProbeSim {
public:
    MSPProbeSim() : _device(_deviceInfo) {
    }
    
    void run() {
        try {
            _device.start();
        } catch (std::exception& e) {
            throw RuntimeError(
                "Failed to start VirtualUSBDevice: %s"                                  "\n"
                "Make sure:"                                                            "\n"
                "  - you're root"                                                       "\n"
                "  - the vhci-hcd kernel module is loaded: `sudo modprobe vhci-hcd`"    "\n",
                e.what()
            );
        }
        
        for (;;) {
            VirtualUSBDevice::Msg msg = _device.read();
            _handleMessage(std::move(msg));
        }
    }
    
    void stop() {
        _device.stop();
    }
    
private:
    void _handleMessage(VirtualUSBDevice::Msg&& msg) {
        // Endpoint 0
        if (!msg.cmd.base.ep) _handleRequestEP0(std::move(msg));
        // Other endpoints
        else _handleRequestEPX(std::move(msg));
    }
    
    void _handleRequestEP0(VirtualUSBDevice::Msg&& msg) {
        const USB::SetupRequest req = msg.getSetupRequest();
//        const uint8_t* payload = msg.payload.get();
        const size_t payloadLen = msg.payloadLen;
        
        // Verify that this is a Class request intended for an Interface
        if (req.bmRequestType != (USB::RequestType::TypeClass|USB::RequestType::RecipientInterface))
            throw RuntimeError("invalid request");
        
        switch (req.bRequest) {
        case USB::CDC::Request::SET_LINE_CODING: {
            if (payloadLen != sizeof(USB::CDC::LineCoding))
                throw RuntimeError("SET_LINE_CODING: payloadLen doesn't match sizeof(USB::CDC::LineCoding)");
            printf("SET_LINE_CODING\n");
            return _device.reply(msg, nullptr, 0);
        }
        
        case USB::CDC::Request::GET_LINE_CODING: {
            printf("GET_LINE_CODING\n");
            if (payloadLen != sizeof(USB::CDC::LineCoding))
                throw RuntimeError("SET_LINE_CODING: payloadLen doesn't match sizeof(USB::CDC::LineCoding)");
            abort(); // TODO: fix
//            return _device.reply(msg, &LineCoding, sizeof(LineCoding));
        }
        
        case USB::CDC::Request::SET_CONTROL_LINE_STATE: {
            const bool dtePresent = req.wValue&1;
            printf("SET_CONTROL_LINE_STATE:\n");
            printf("  dtePresent=%d\n", dtePresent);
            return _device.reply(msg, nullptr, 0);
        }
        
        default:
            throw RuntimeError("invalid request");
        }
    }
    
    bool _handleRequestEPX(VirtualUSBDevice::Msg&& msg) {
        const uint8_t ep = msg.getEndpointAddress();
        switch (ep) {
        case CDC0_INTEP_ADDR:
            printf("Endpoint CDC0_INTEP_ADDR\n");
            break;
        case CDC0_OUTEP_ADDR:
            printf("Endpoint CDC0_OUTEP_ADDR\n");
            break;
        case CDC0_INEP_ADDR:
            printf("Endpoint CDC0_INEP_ADDR\n");
            break;
        case CDC1_INTEP_ADDR:
            printf("Endpoint CDC1_INTEP_ADDR\n");
            break;
        case CDC1_OUTEP_ADDR:
            printf("Endpoint CDC1_OUTEP_ADDR\n");
            break;
        case CDC1_INEP_ADDR:
            printf("Endpoint CDC1_INEP_ADDR\n");
            break;
        default:
            throw RuntimeError("invalid endpoint: %02x", ep);
        }
        
        _device.reply(msg, nullptr, 0);
        return true;
    }
    
    static inline const USB::DeviceDescriptor* _DeviceDescriptor =
        (const USB::DeviceDescriptor*)TIDescriptor::abromDeviceDescriptor;
    
    static inline const USB::ConfigurationDescriptor* _ConfigurationDescriptors[] = {
        (USB::ConfigurationDescriptor*)&TIDescriptor::abromConfigurationDescriptorGroup,
    };
    
    static inline const USB::StringDescriptor* _StringDescriptors[] = {
        &TIDescriptor::String0,
        &TIDescriptor::String1,
        &TIDescriptor::String2,
        &TIDescriptor::String3,
        &TIDescriptor::String4,
        &TIDescriptor::String5,
        &TIDescriptor::String6,
    };
    
    static inline const VirtualUSBDevice::Info _deviceInfo = {
        .deviceDesc             = _DeviceDescriptor,
        .deviceQualifierDesc    = nullptr,
        .configDescs            = _ConfigurationDescriptors,
        .configDescsCount       = std::size(_ConfigurationDescriptors),
        .stringDescs            = _StringDescriptors,
        .stringDescsCount       = std::size(_StringDescriptors),
        .throwOnErr             = true,
    };
    
    VirtualUSBDevice _device;
};
