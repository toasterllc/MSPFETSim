#pragma once
#include <libusb-1.0/libusb.h>
#include "RuntimeError.h"
#include "MSPInterface.h"
#include "Defer.h"
#include "USBDevice.h"
#include "../../MDC/Code/STM32/Shared/STLoaderTypes.h"
>>>>>>> ffcc230... Add USBDevice class. Started work on FTDI driver.:Src/Drivers/STMBridge.h

class STMBridge : public MSPInterface {
public:
    
    static std::vector<USBDevice> GetMatchingDevices() {
        std::vector<USBDevice> r;
        for (USBDevice& d : USBDevice::GetAllDevices()) {
            if (_DeviceMatches(d)) {
                r.push_back(std::move(d));
            }
        }
        return r;
    }
    
    STMBridge(USBDevice&& dev) {
        assert(dev);
        _s.dev = std::move(dev);
        _s.dev.open();
        _s.dev.claimInterface(_USBInterfaceIdx);
    }
    
//    // Copy constructor: not allowed
//    STMBridge(const STMBridge& x) = delete;
//    // Move constructor: use move assignment operator
//    STMBridge(STMBridge&& x) { *this = std::move(x); }
//    // Move assignment operator
//    STMBridge& operator=(STMBridge&& x) {
//        _s = x._s;
//        x._s = {};
//        return *this;
//    }
//    
//    ~STMBridge() {
//        _reset();
//    }
    
    // Verify that MSPInterface::PinState values == MSPDebugCmd::PinState values,
    // so we can use the types interchangeably
    static_assert((uint8_t)MSPInterface::PinState::Out0 ==
                  (uint8_t)STLoader::MSPDebugCmd::PinStates::Out0);
    static_assert((uint8_t)MSPInterface::PinState::Out1 ==
                  (uint8_t)STLoader::MSPDebugCmd::PinStates::Out1);
    static_assert((uint8_t)MSPInterface::PinState::In ==
                  (uint8_t)STLoader::MSPDebugCmd::PinStates::In);
    static_assert((uint8_t)MSPInterface::PinState::Pulse01 ==
                  (uint8_t)STLoader::MSPDebugCmd::PinStates::Pulse01);
    
    void sbwPins(MSPInterface::PinState test, MSPInterface::PinState rst) override {
        _s.cmds.emplace_back((STLoader::MSPDebugCmd::PinState)test, (STLoader::MSPDebugCmd::PinState)rst);
    }
    
    void sbwIO(bool tms, bool tclk, bool tdi, bool tdoRead) override {
        _s.cmds.emplace_back(tms, tclk, tdi, tdoRead);
        if (tdoRead) _s.readLen++;
    }
    
    void sbwRead(void* buf, size_t len) override {
        if (len > _s.readLen) throw RuntimeError("too much data requested");
        // Short-circuit if we don't have any commands, and we're not reading any data
        if (_s.cmds.empty() && !len) return;
        
        // Write our MSPDebugCmds and read back the queued data
        const STLoader::Cmd cmd = {
            .op = STLoader::Op::MSPDebug,
            .arg = {
                .MSPDebug = {
                    .writeLen = (uint32_t)_s.cmds.size(),
                    .readLen = (uint32_t)len,
                },
            },
        };
        
        // Write the STLoader::Cmd
        _s.dev.bulkWrite(STLoader::Endpoints::CmdOut, &cmd, sizeof(cmd));
        
        // Write the MSPDebugCmds
        if (_s.cmds.size()) {
            _s.dev.bulkWrite(STLoader::Endpoints::DataOut, _s.cmds.data(), _s.cmds.size());
            _s.cmds.clear();
        }
        
        // Read back the queued data
        if (len) {
            _s.dev.bulkRead(STLoader::Endpoints::DataIn, buf, len);
        }
        
        // Check our status
        STLoader::Status status = STLoader::Status::Error;
        _s.dev.bulkRead(STLoader::Endpoints::DataIn, &status, sizeof(status));
        if (status != STLoader::Status::OK) throw RuntimeError("MSPDebug commands failed");
    }
    
private:
    static constexpr uint32_t _USBInterfaceIdx = 0;
    
    struct {
        USBDevice dev;
        std::vector<STLoader::MSPDebugCmd> cmds;
        size_t readLen = 0;
    } _s;
    
    static bool _DeviceMatches(USBDevice& dev) {
        struct libusb_device_descriptor desc = dev.getDeviceDescriptor();
        return desc.idVendor==0x0483 && desc.idProduct==0xDF11;
    }
    
//    template <typename T>
//    void _bulkXfer(uint8_t ep, T* data, size_t len) {
//        int xferLen = 0;
//        int ir = libusb_bulk_transfer(_s.dev, ep, (uint8_t*)data, (int)len, &xferLen, 0);
//        _CheckUSBErr(ir, "libusb_bulk_transfer failed");
//        if ((size_t)xferLen != len)
//            throw RuntimeError("libusb_bulk_transfer short transfer (tried: %zu, got: %zu)", len, (size_t)xferLen);
//    }
    
//    void _checkUSBXferErr(int ir, int xferLen, size_t expectedXferLen) {
//        if (ir < 0) throw RuntimeError("libusb_bulk_transfer failed: %s", libusb_error_name(ir));
//        if ((size_t)xferLen != sizeof(status))
//            throw RuntimeError("libusb_bulk_transfer short transfer (tried: %zu, got: %zu)", expectedXferLen, (size_t)xferLen);
//    }
//    
//    void _waitOrThrow(const char* errMsg) {
//        STLoader::Status status = STLoader::Status::Error;
//        _bulkXfer(STLoader::Endpoints::DataIn, &status, sizeof(status));
//        if (status != STLoader::Status::OK) throw RuntimeError("%s: status=%d", errMsg, ir);
//    }
};
