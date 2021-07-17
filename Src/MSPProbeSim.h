#pragma once
#include <climits>
#include "VirtualUSBDevice.h"
#include "MSPInterface.h"

class MSPProbeSim {
public:
    MSPProbeSim(MSPInterface& msp) :
    _msp(msp),
    _jtag(dynamic_cast<MSPInterfaceJTAG*>(&_msp)),
    _sbw(dynamic_cast<MSPInterfaceSBW*>(&_msp)),
    _usb(_usbDeviceInfo) {
    }
    
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
        
        ResetFirmware();
        BIOS_InitCom();
        
        for (;;) {
            _dequeueUSBRequest();
            // Call V3OP_Scheduler until there's no work to do
            while (V3OP_Scheduler());
        }
    }
    
    void stop() {
        _usb.stop();
    }
    
private:
    
    #include "FW/FW.h"
    
    struct _Msg {
        uint8_t data[512];
        size_t len = 0;
    };
    
    using _Rep = _Msg;
    
    void _dequeueUSBRequest() {
        VirtualUSBDevice::Xfer xfer = _usb.read();
        _handleUSBRequest(std::move(xfer));
    }
    
    void _handleUSBRequest(VirtualUSBDevice::Xfer&& xfer) {
        // Endpoint 0
        if (!xfer.cmd.base.ep) _handleUSBRequestEP0(std::move(xfer));
        // Other endpoints
        else _handleUSBRequestEPX(std::move(xfer));
    }
    
    void _handleUSBRequestEP0(VirtualUSBDevice::Xfer&& xfer) {
        const USB::SetupRequest req = xfer.getSetupRequest();
//        const uint8_t* payload = xfer.payload.get();
        const size_t payloadLen = xfer.payloadLen;
        
        // Verify that this is a Class request intended for an Interface
        if (req.bmRequestType != (USB::RequestType::TypeClass|USB::RequestType::RecipientInterface))
            throw RuntimeError("invalid request");
        
        switch (req.bRequest) {
        case USB::CDC::Request::SET_LINE_CODING: {
            if (payloadLen != sizeof(USB::CDC::LineCoding))
                throw RuntimeError("SET_LINE_CODING: payloadLen doesn't match sizeof(USB::CDC::LineCoding)");
            printf("SET_LINE_CODING\n");
            return _usb.reply(xfer, nullptr, 0);
        }
        
        case USB::CDC::Request::GET_LINE_CODING: {
            printf("GET_LINE_CODING\n");
            if (payloadLen != sizeof(USB::CDC::LineCoding))
                throw RuntimeError("SET_LINE_CODING: payloadLen doesn't match sizeof(USB::CDC::LineCoding)");
            abort(); // TODO: fix
//            return _usb.reply(xfer, &LineCoding, sizeof(LineCoding));
        }
        
        case USB::CDC::Request::SET_CONTROL_LINE_STATE: {
            const bool dtePresent = req.wValue&1;
            printf("SET_CONTROL_LINE_STATE:\n");
            printf("  dtePresent=%d\n", dtePresent);
            return _usb.reply(xfer, nullptr, 0);
        }
        
        default:
            throw RuntimeError("invalid request");
        }
    }
    
    void _handleUSBRequestEPX(VirtualUSBDevice::Xfer&& xfer) {
        const uint8_t ep = xfer.getEndpointAddress();
        switch (ep) {
        case CDC0_INTEP_ADDR:
            printf("Endpoint CDC0_INTEP_ADDR\n");
            break;
        case CDC0_OUTEP_ADDR:
            printf("Endpoint CDC0_OUTEP_ADDR\n");
            _handleMsg(std::move(xfer));
            break;
        case CDC0_INEP_ADDR:
            printf("Endpoint CDC0_INEP_ADDR\n");
            _handleInXfer(std::move(xfer));
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
        
//        _usb.reply(msg, nullptr, 0);
    }
    
    void _handleInXfer(VirtualUSBDevice::Xfer&& xfer) {
        _inXfers.push_back(std::move(xfer));
        // If a reply is already available, send it now
        _replyIfPossible();
    }
    
    void _handleMsg(VirtualUSBDevice::Xfer&& xfer) {
        printf("_handleMsg: <");
        for (size_t i=0; i<xfer.payloadLen; i++) {
            printf(" %02x", xfer.payload[i]);
        }
        printf(" >\n\n");
        
        _msgs.push_back({});
        _Msg& msg = _msgs.back();
        
        assert(sizeof(msg.data) >= xfer.payloadLen);
        memcpy(msg.data, xfer.payload.get(), xfer.payloadLen);
        msg.len = xfer.payloadLen;
        
        // Notify firmware that USB data is available
        USBCDC_handleDataReceived(DEBUG_CHANNEL);
        
        // Reply to the USB request to let the host know we received the data
        _usb.reply(xfer, nullptr, 0);
    }
    
    void _handleReply(const void* data, size_t len) {
        _reps.push_back({});
        _Rep& rep = _reps.back();
        
        assert(sizeof(rep.data) >= len);
        memcpy(rep.data, data, len);
        rep.len = len;
        
        // If the host requested data, send it now
        _replyIfPossible();
    }
    
    void _replyIfPossible() {
        // Match in-xfers with replies, and call `_usb.reply()` with the pairs,
        // until we run out of one (or both)
        while (!_inXfers.empty() && !_reps.empty()) {
            VirtualUSBDevice::Xfer& xfer = _inXfers.front();
            _Rep& rep = _reps.front();
            _usb.reply(xfer, rep.data, rep.len);
            
            _inXfers.pop_front();
            _reps.pop_front();
        }
    }
    
    MSPInterface& _msp;
    MSPInterfaceJTAG*const _jtag = nullptr;
    MSPInterfaceSBW*const _sbw = nullptr;
    VirtualUSBDevice _usb;
    
    std::deque<_Msg> _msgs; // Messages host->device
    std::deque<_Rep> _reps; // Replies device->host
    std::deque<VirtualUSBDevice::Xfer> _inXfers; // USB host requests for data from device
    
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
