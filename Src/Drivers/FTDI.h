#pragma once
#include <libftdi1/ftdi.h>

class FTDI : public MSPInterface {
public:
    
    static libusb_device* FindFTDIDevice() {
        ftdi_usb_find_all();
        
        
        libusb_device** devs = nullptr;
        Defer( if (devs) libusb_free_device_list(devs, true); );
        
        ssize_t devsCount = libusb_get_device_list(_USBContext(), &devs);
        _CheckUSBErr((int)devsCount, "libusb_get_device_list failed");
        
        libusb_device* dev = nullptr;
        for (size_t i=0; i<(size_t)devsCount; i++) {
            libusb_device* d = devs[i];
            if (_DeviceMatches(d)) {
                dev = d;
                break;
            }
        }
        
        // Retain the device since it'd otherwise be destroyed when we call libusb_free_device_list
        if (dev) libusb_ref_device(dev);
        return dev;
    }
    
    FTDI(libusb_device* dev) {
        assert(dev);
        try {
            _CtxCreate();
            
            int ir = libusb_open(dev, &_s.dev);
            _CheckUSBErr(ir, "libusb_open failed");
            
            ir = libusb_claim_interface(_s.dev, _USBInterfaceIdx);
            _CheckUSBErr(ir, "libusb_claim_interface failed");
            
        } catch (const std::exception& e) {
            _reset();
            throw;
        }
    }
    
    ~FTDI() {
        _reset();
    }
    
    void sbwPins(MSPInterface::PinState test, MSPInterface::PinState rst) override {
    }
    
    void sbwIO(bool tms, bool tclk, bool tdi, bool tdoRead) override {
    }
    
    void sbwRead(void* buf, size_t len) override {
    }
    
private:
    static libusb_context* _FTDICtx = nullptr;
    
    static void _FTDICtxCreate() {
        static std::once_flag Once;
        std::call_once(Once, [](){
            int ir = ftdi_init(&_FTDICtx);
            _CheckFTDIErr(ir, "ftdi_init failed");
        });
    }
    
    static void _CheckFTDIErr(int ir, const char* errMsg) {
        if (ir < 0) throw RuntimeError("%s: %s", errMsg, ftdi_get_error_string(_FTDIContext()));
    }
    
    struct ftdi_context _ftdi;
};
