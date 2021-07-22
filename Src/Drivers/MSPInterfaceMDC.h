#pragma once
#include <libusb-1.0/libusb.h>
#include "RuntimeError.h"
#include "MSPInterface.h"
#include "Defer.h"
#include "USBDevice.h"
#include "../../MDC/Code/STM32/Shared/STLoaderTypes.h"
>>>>>>> ffcc230... Add USBDevice class. Started work on FTDI driver.:Src/Drivers/STMBridge.h

class MSPInterfaceMDC : public MSPInterface {
public:
    
    static std::vector<USBDevice> GetDevices() {
        std::vector<USBDevice> r;
        for (USBDevice& d : USBDevice::GetDevices()) {
            if (_DeviceMatches(d)) {
                r.push_back(std::move(d));
            }
        }
        return r;
    }
    
    MSPInterfaceMDC(USBDevice&& dev) : _dev(std::move(dev)) {
        assert(_dev);
        _dev.open();
        _dev.claimInterface(_USBInterfaceIdx);
    }
    
//    // Copy constructor: not allowed
//    MSPInterfaceMDC(const MSPInterfaceMDC& x) = delete;
//    // Move constructor: use move assignment operator
//    MSPInterfaceMDC(MSPInterfaceMDC&& x) { *this = std::move(x); }
//    // Move assignment operator
//    MSPInterfaceMDC& operator=(MSPInterfaceMDC&& x) {
//        _s = x._s;
//        x._s = {};
//        return *this;
//    }
//    
//    ~MSPInterfaceMDC() {
//        _reset();
//    }
    
    void sbwTestSet(bool val) {
        _cmds.emplace_back(STLoader::MSPDebugCmd::TestSet, val);
    }
    
    void sbwRstSet(bool val) {
        _cmds.emplace_back(STLoader::MSPDebugCmd::RstSet, val);
    }
    
    void sbwTestPulse() {
        _cmds.emplace_back(STLoader::MSPDebugCmd::TestPulse);
    }
    
    void sbwIO(bool tms, bool tclk, bool tdi, bool tdoRead) override {
        _cmds.emplace_back(STLoader::MSPDebugCmd::SBWIO, tms, tclk, tdi, tdoRead);
        if (tdoRead) _readLen++;
    }
    
    void sbwRead(void* buf, size_t len) override {
        if (len > _readLen) throw RuntimeError("too much data requested");
        // Short-circuit if we don't have any commands, and we're not reading any data
        if (_cmds.empty() && !len) return;
        
        // Write our MSPDebugCmds and read back the queued data
        const STLoader::Cmd cmd = {
            .op = STLoader::Op::MSPDebug,
            .arg = {
                .MSPDebug = {
                    .writeLen = (uint32_t)_cmds.size(),
                    .readLen = (uint32_t)len,
                },
            },
        };
        
        // Write the STLoader::Cmd
        _dev.bulkWrite(STLoader::Endpoints::CmdOut, &cmd, sizeof(cmd));
        
        // Write the MSPDebugCmds
        if (_cmds.size()) {
            _dev.bulkWrite(STLoader::Endpoints::DataOut, _cmds.data(), _cmds.size());
            _cmds.clear();
        }
        
        // Read back the queued data
        if (len) {
            _dev.bulkRead(STLoader::Endpoints::DataIn, buf, len);
        }
        
        // Check our status
        STLoader::Status status = STLoader::Status::Error;
        _dev.bulkRead(STLoader::Endpoints::DataIn, &status, sizeof(status));
        if (status != STLoader::Status::OK) throw RuntimeError("MSPDebug commands failed");
    }
    
private:
    static constexpr uint32_t _USBInterfaceIdx = 0;
    
    USBDevice _dev;
    std::vector<STLoader::MSPDebugCmd> _cmds = {};
    size_t _readLen = 0;
    
    static bool _DeviceMatches(USBDevice& dev) {
        struct libusb_device_descriptor desc = dev.getDeviceDescriptor();
        return desc.idVendor==0x0483 && desc.idProduct==0xDF11;
    }
    
//    template <typename T>
//    void _bulkXfer(uint8_t ep, T* data, size_t len) {
//        int xferLen = 0;
//        int ir = libusb_bulk_transfer(_dev, ep, (uint8_t*)data, (int)len, &xferLen, 0);
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
