#pragma once
#include <libftdi1/ftdi.h>

class MSPInterfaceFTDI : public MSPInterface {
public:
    
//    static int ftdi_usb_find_all_ours(struct ftdi_context *ftdi, struct ftdi_device_list **devlist, int vendor, int product)
//    {
//        struct ftdi_device_list **curdev;
//        libusb_device *dev;
//        libusb_device **devs;
//        int count = 0;
//        int i = 0;
//        
//        libusb_init(&ftdi->usb_ctx);
//        
//        if (libusb_get_device_list(ftdi->usb_ctx, &devs) < 0)
//            abort();
//        
//        curdev = devlist;
//        *curdev = NULL;
//
//        while ((dev = devs[i++]) != NULL)
//        {
//            struct libusb_device_descriptor desc;
//
//            if (libusb_get_device_descriptor(dev, &desc) < 0)
//                abort();
//
//            if (((vendor || product) &&
//                    desc.idVendor == vendor && desc.idProduct == product) ||
//                    (!(vendor || product) &&
//                     (desc.idVendor == 0x403) && (desc.idProduct == 0x6001 || desc.idProduct == 0x6010
//                                                  || desc.idProduct == 0x6011 || desc.idProduct == 0x6014
//                                                  || desc.idProduct == 0x6015)))
//            {
//                *curdev = (struct ftdi_device_list*)malloc(sizeof(struct ftdi_device_list));
//                if (!*curdev)
//                    abort();
//
//                (*curdev)->next = NULL;
//                (*curdev)->dev = dev;
//                libusb_ref_device(dev);
//                curdev = &(*curdev)->next;
//                count++;
//            }
//        }
//        libusb_free_device_list(devs,1);
//        return count;
//    }
    
    static std::vector<USBDevice> GetDevices() {
//        auto vers = ftdi_get_library_version();
//        printf("libftdi version:\n");
//        printf("  major = %d\n", vers.major);
//        printf("  minor = %d\n", vers.minor);
//        printf("  micro = %d\n", vers.micro);
//        printf("  version_str = %s\n", vers.version_str);
//        printf("  snapshot_str = %s\n", vers.snapshot_str);
//        printf("\n\n");
        
        struct ftdi_context ctx = {};
        int ir = ftdi_init(&ctx);
        if (ir < 0) throw RuntimeError("ftdi_init failed: %s", ftdi_get_error_string(&ctx));
        Defer( ftdi_deinit(&ctx); );
        
        struct ftdi_device_list* devs = nullptr;
        ir = ftdi_usb_find_all(&ctx, &devs, 0, 0);
        if (ir < 0) throw RuntimeError("ftdi_usb_find_all failed: %s", ftdi_get_error_string(&ctx));
        Defer( if (devs) ftdi_list_free(&devs); );
        
        std::vector<USBDevice> r;
        while (devs) {
            r.push_back(devs->dev);
            devs = devs->next;
        }
        return r;
    }
    
    MSPInterfaceFTDI(USBDevice&& dev) : _dev(std::move(dev)) {
    }
    
    ~MSPInterfaceFTDI() {}
    
    void sbwPins(MSPInterface::PinState test, MSPInterface::PinState rst) override {
    }
    
    void sbwIO(bool tms, bool tclk, bool tdi, bool tdoRead) override {
    }
    
    void sbwRead(void* buf, size_t len) override {
    }
    
private:
    
    class _FTDIDevice {
    public:
        _FTDIDevice(USBDevice&& usbDev) :
        _usbDev(std::move(usbDev)) {
            int ir = ftdi_init(&_ctx);
            _checkErr(ir, "ftdi_init failed");
        }
        
//        // Copy constructor: not allowed
//        _FTDIDevice(const _FTDIDevice& x) = delete;
//        // Move constructor: use move assignment operator
//        _FTDIDevice(_FTDIDevice&& x) { *this = std::move(x); }
//        // Move assignment operator
//        _FTDIDevice& operator=(_FTDIDevice&& x) {
//            _reset();
//            _s = x._s;
//            x._s = {};
//            return *this;
//        }
        
        ~_FTDIDevice() {
            close();
            ftdi_deinit(&_ctx);
        }
        
        void open() {
            int ir = ftdi_usb_open_dev(&_ctx, _usbDev);
            _checkErr(ir, "ftdi_usb_open_dev failed");
        }
        
        void close() {
            int ir = ftdi_usb_close(&_ctx);
            _checkErr(ir, "ftdi_usb_close failed");
        }
        
//        operator bool() const { return _s.dev; }
//        operator libusb_device*() const { return _s.dev; }
//        operator libusb_device_handle*() const { return _s.devHandle; }
        
    private:
        void _checkErr(int ir, const char* errMsg) {
            if (ir < 0) throw RuntimeError("%s: %s", errMsg, ftdi_get_error_string(&_ctx));
        }
        
//        void _reset() {
//            
//        }
//        
    //    void _reset() {
    //        close();
    //    }
        
    //    void _setDevHandle(libusb_device_handle* devHandle) {
    //        libusb_close(_s.dev);
    //        _s.dev = nullptr;
    //        
    //        if (_s.devHandle) 
    //        if (dev) libusb_ref_device(dev);
    //        if (_dev) libusb_unref_device(_dev);
    //        _dev = dev;
    //        _s.devHandle = devHandle;
    //    }
        
//        struct {
//            struct ftdi_context ctx;
//            USBDevice usbDev;
//        } _s;
        
        struct ftdi_context _ctx;
        USBDevice _usbDev;
    };
    
    
    
//    static libusb_context* _FTDICtx = nullptr;
//    
//    static void _FTDICtxCreate() {
//        static std::once_flag Once;
//        std::call_once(Once, [](){
//            int ir = ftdi_init(&_FTDICtx);
//            _checkErr(ir, "ftdi_init failed");
//        });
//    }
    
//    static void _checkErr(struct ftdi_context& ctx, int ir, const char* errMsg) {
//        if (ir < 0) throw RuntimeError("%s: %s", errMsg, ftdi_get_error_string(&ctx));
//    }
    
    _FTDIDevice _dev;
};