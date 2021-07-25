#pragma once

#if __has_include("../../MDC/Code/STM32/Shared/STLoaderTypes.h")

#include <libusb-1.0/libusb.h>
#include "MSPDebugDriver.h"
#include "RuntimeError.h"
#include "Defer.h"
#include "USBDevice.h"
#include "../../MDC/Code/STM32/Shared/STLoaderTypes.h"
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
        assert(_dev);
        _dev.open();
        _dev.claimInterface(_USBInterfaceIdx);
    }
    
    void sbwTestSet(bool val) override {
        _cmds.emplace_back(STLoader::MSPDebugCmd::TestSet, val);
    }
    
    void sbwRstSet(bool val) override {
        _cmds.emplace_back(STLoader::MSPDebugCmd::RstSet, val);
    }
    
    void sbwTestPulse() override {
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
};

#endif // __has_include("../../MDC/Code/STM32/Shared/STLoaderTypes.h")
