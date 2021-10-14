#pragma once

#if __has_include("MDCDevice.h")

#include <libusb-1.0/libusb.h>
#include "Toastbox/RuntimeError.h"
#include "Toastbox/Defer.h"
#include "Toastbox/USBDevice.h"
#include "MSPDebugDriver.h"
#include "MDCDevice.h"
#define MSPDebugDriverMDCExists 1

// This driver is for a custom device and isn't generally useful, however
// it may be instructive as an example of a simple MSPFETSim driver.
class MSPDebugDriverMDC : public MSPDebugDriver {
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
    
    MSPDebugDriverMDC(USBDevice&& dev) : _dev(std::move(dev)) {
        _dev.flushEndpoints();
    }
    
    void sbwTestSet(bool val) override {
        _cmds.emplace_back(STM::MSPDebugCmd::TestSet, val);
    }
    
    void sbwRstSet(bool val) override {
        _cmds.emplace_back(STM::MSPDebugCmd::RstSet, val);
    }
    
    void sbwTestPulse() override {
        _cmds.emplace_back(STM::MSPDebugCmd::TestPulse);
    }
    
    void sbwIO(bool tms, bool tclk, bool tdi, bool tdoRead) override {
        _cmds.emplace_back(STM::MSPDebugCmd::SBWIO, tms, tclk, tdi, tdoRead);
        if (tdoRead) _readLen++;
    }
    
    void sbwRead(void* buf, size_t len) override {
        if (len > _readLen) throw RuntimeError("too much data requested");
        // Short-circuit if we don't have any commands, and we're not reading any data
        if (_cmds.empty() && !len) return;
        
        _dev.mspDebug(_cmds.data(), _cmds.size(), buf, len);
        _cmds.clear();
    }
    
private:
    static constexpr uint32_t _USBInterfaceIdx = 0;
    
    MDCDevice _dev;
    std::vector<STM::MSPDebugCmd> _cmds = {};
    size_t _readLen = 0;
    
    static bool _DeviceMatches(USBDevice& dev) {
        const USB::DeviceDescriptor desc = dev.deviceDescriptor();
        return desc.idVendor==0x0483 && desc.idProduct==0xDF11;
    }
};

#endif // __has_include("MDCDevice.h")
