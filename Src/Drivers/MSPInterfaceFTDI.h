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
        
        static struct {
            std::mutex lock;
            struct ftdi_context* ctx = nullptr;
        } FTDICtx;
        
        auto lock = std::unique_lock(FTDICtx.lock);
        if (!FTDICtx.ctx) {
            FTDICtx.ctx = ftdi_new();
            if (!FTDICtx.ctx) throw RuntimeError("ftdi_new failed");
        }
        
        struct ftdi_device_list* devs = nullptr;
        int ir = ftdi_usb_find_all(FTDICtx.ctx, &devs, 0, 0);
        if (ir < 0) throw RuntimeError("ftdi_usb_find_all failed: %s", ftdi_get_error_string(FTDICtx.ctx));
        Defer( if (devs) ftdi_list_free(&devs); );
        
        std::vector<USBDevice> r;
        while (devs) {
            r.push_back(devs->dev);
            devs = devs->next;
        }
        return r;
    }
    
    MSPInterfaceFTDI(uint8_t testPin, uint8_t rstPin, USBDevice&& dev) :
    _testPin(testPin),
    _rstPin(rstPin),
    _dev(std::move(dev)) {
        _dev.open();
        _dev.usbReset();
        _dev.setEventChar(0, 0);
        
        _dev.setBitmode(BITMODE_RESET, 0);
        _dev.setBitmode(BITMODE_MPSSE, 0);
        
        // TODO: these commands aren't all supported by all hardware
        {
            constexpr uint8_t initCmd[] = {
//                MPSSE::DisableClkDivideBy5,        // Use 60MHz master clock
//                MPSSE::DisableAdaptiveClocking,    // Disable adaptive clocking
//                MPSSE::Disable3PhaseDataClocking,  // Disable three-phase clocking
//                MPSSE::DisconnectTDITDOLoopback,   // Disable loopback
                MPSSE::SetClkDivisor, 0, 0,         // Set TCK frequency to 6MHz
            };
            
            _dev.write(initCmd, sizeof(initCmd));
        }
        
        // Clear our receive buffer
        // For some reason this needs to happen after our first write,
        // otherwise we don't receive anything.
        // This is necessary in case an old process was doing IO and crashed, in which
        // case there could still be data in the buffer.
        for (;;) {
            uint8_t tmp[128];
            size_t ir = _dev.readAny(tmp, sizeof(tmp));
            if (!ir) break;
        }
        
        {
            constexpr uint8_t initCmd[] = {
                MPSSE::BadCommand,
            };
            _dev.write(initCmd, sizeof(initCmd));
        }
        
        // Synchronize with FTDI by sending a bad command and ensuring we get the expected error
        uint8_t resp[2];
        _dev.read(resp, sizeof(resp));
        printf("resp: %x %x\n", resp[0], resp[1]);
//            if (!(resp[0]==MPSSE::BadCommandResp && resp[1]==MPSSE::BadCommand))
//                throw RuntimeError("init sync failed (%x %x)", resp[0], resp[1]);
    }
    
    ~MSPInterfaceFTDI() {}
    
    struct MPSSE {
        static constexpr uint8_t SetDataBitsL               = 0x80;
        static constexpr uint8_t ReadDataBitsL              = 0x81;
        static constexpr uint8_t SetDataBitsH               = 0x82;
        static constexpr uint8_t ReadDataBitsH              = 0x83;
        static constexpr uint8_t DisconnectTDITDOLoopback   = 0x85;
        static constexpr uint8_t SetClkDivisor              = 0x86;
        static constexpr uint8_t DisableClkDivideBy5        = 0x8A;
        static constexpr uint8_t Disable3PhaseDataClocking  = 0x8D;
        static constexpr uint8_t DisableAdaptiveClocking    = 0x97;
        static constexpr uint8_t BadCommandResp             = 0xFA;
        static constexpr uint8_t BadCommand                 = 0xAB;
    };
    
    PinState _testState = PinState::In;
    PinState _rstState = PinState::In;
    void _setPins(PinState t, PinState r) {
        assert(t != PinState::Pulse01);
        assert(r != PinState::Pulse01);
        // Short-circuit if the state hasn't changed
        if (t==_testState && r==_rstState) return;
        _testState = t;
        _rstState = r;
        
        const uint8_t dir =
            (t!=PinState::In ? _testPin : 0) |
            (r!=PinState::In ? _rstPin  : 0) ;
        
        const uint8_t val =
            (t==PinState::Out1 ? _testPin : 0) |
            (r==PinState::Out1 ? _rstPin  : 0) ;
        
        _cmds.push_back(MPSSE::SetDataBitsL);
        _cmds.push_back(val); // Value
        _cmds.push_back(dir); // Direction
    }
    
    void _setTest(PinState t) {
        _setPins(t, _rstState);
    }
    
    void _setRst(PinState r) {
        _setPins(_testState, r);
    }
    
    void _read() {
        _cmds.push_back(MPSSE::ReadDataBitsL);
        _readLen++;
    }
    
    void sbwTest(PinState test) override {
        _setTest(test);
    }
    
    void sbwRst(PinState rst) override {
        _setRst(rst);
    }
    
    void sbwPins(PinState test, PinState rst) override {
        const PinState t0 = (test!=PinState::Pulse01 ? test : PinState::Out0);
        const PinState r0 = (rst !=PinState::Pulse01 ? rst  : PinState::Out0);
        const PinState t1 = (test!=PinState::Pulse01 ? test : PinState::Out1);
        const PinState r1 = (rst !=PinState::Pulse01 ? rst  : PinState::Out1);
        
        _setPins(t0, r0);
        _setPins(t1, r1);
        
//        if (pulse) {
//            // TODO: flush existing commands if we're pulsing, to ensure that these commands are atomically received, otherwise the pulse signal could be stretched
//        }
//        
//        
//        const uint8_t dir =
//            (test!=PinState::In ? _testPin : 0) |
//            ( rst!=PinState::In ? _rstPin  : 0) ;
//        
//        const uint8_t val0 =
//            (test==PinState::Out1 ? _testPin : 0) |
//            ( rst==PinState::Out1 ? _rstPin  : 0) ;
//        
//        _cmds.push_back(MPSSE::SetDataBitsL);
//        _cmds.push_back(val0); // Value
//        _cmds.push_back(dir); // Direction
//        
//        if (pulse) {
//            const uint8_t val1 =
//                (test==PinState::Out1||test==PinState::Pulse01 ? _testPin : 0) |
//                ( rst==PinState::Out1|| rst==PinState::Pulse01 ? _rstPin  : 0) ;
//            
//            _cmds.push_back(MPSSE::SetDataBitsL);
//            _cmds.push_back(val1); // Value
//            _cmds.push_back(dir); // Direction
//        }
    }
    
//    RST::Write(tms);
//    TEST::Write(0);
//    tdo = RST::Read();
//    RST::Write(tclk);
//    TEST::Write(1);
    
    
    
    void sbwIO(bool tms, bool tclk, bool tdi, bool tdoRead) override {
        _flushIfNeeded();
        
        // Write TMS
        {
            _setRst(tms ? PinState::Out1 : PinState::Out0);
            _setTest(PinState::Out0);
            _setRst(tclk ? PinState::Out1 : PinState::Out0);
            _setTest(PinState::Out1);
        }
        
        // Write TDI
        {
            _setRst(tdi ? PinState::Out1 : PinState::Out0);
            _setTest(PinState::Out0);
            _setTest(PinState::Out1);
            _setRst(PinState::In);
        }
        
        // Read TDO
        {
            _setTest(PinState::Out0);
            if (tdoRead) _read();
            _setTest(PinState::Out1);
            _setRst(tdi ? PinState::Out1 : PinState::Out0);
        }
    }
    
    void _flushIfNeeded() {
        if (_cmds.size() >= 512) {
            _flush();
        }
    }
    
    void _flush() {
        if (_cmds.empty()) return; // Short-circuit if there aren't any commands to flush
        // Write the commands
        _dev.write(_cmds.data(), _cmds.size());
        _cmds.clear();
        
        // Read expected amount of data into the end of `_readData`
        const size_t off = _readData.size();
        _readData.resize(_readData.size() + _readLen);
        _dev.read(_readData.data()+off, _readLen);
        _readLen = 0;
    }
    
    void sbwRead(void* buf, size_t len) override {
        _flush();
        
        // Verify that the requested length isn't greater than
        // the amount of available data
        assert(len*8 <= _readData.size());
        
//        const size_t tmpBufLen = len*8;
//        auto tmpBufUnique = std::make_unique<uint8_t[]>(tmpBufLen);
//        uint8_t* tmpBuf = tmpBufUnique.get();
//        _dev.read(tmpBuf, tmpBufLen);
        
        uint8_t* buf8 = (uint8_t*)buf;
        for (size_t ireadData=0, ibit=0, ibuf=0; ireadData<len*8; ireadData++) {
            const bool bit = _readData[ireadData] & _rstPin;
            buf8[ibuf] <<= 1;
            buf8[ibuf] |= bit;
            ibit++;
            if (ibit == 8) {
                ibuf++;
                ibit = 0;
            }
        }
        
        _readData.erase(_readData.begin(), _readData.begin()+len*8);
        
//        uint8_t* buf8 = (uint8_t*)buf;
//        for (size_t ibit=0, ibyte=0, byteLen=0; ibit<tmpBufLen; ibit++) {
//            const bool bit = tmpBuf[ibit] & _rstPin;
//            buf8[ibyte] <<= 1;
//            buf8[ibyte] |= bit;
//            byteLen++;
//            if (byteLen == 8) {
//                ibyte++;
//                byteLen = 0;
//            }
//        }
//        
//        _readLen -= len;
    }
    
private:
    
    class _FTDIDevice {
    public:
        _FTDIDevice(USBDevice&& usbDev) :
        _usbDev(std::move(usbDev)) {
            int ir = ftdi_init(&_ctx);
            _checkErr(ir, "ftdi_init failed");
            
//            ftdi_set_interface(&_ctx, INTERFACE_ANY);
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
        
//        void setInterface(uint8_t mode, uint8_t pinDirs) {
//            int ir = ftdi_set_bitmode(&_ctx, pinDirs, mode);
//            _checkErr(ir, "setBitmode failed");
//        }
        
        void open() {
//            int ir = ftdi_set_interface(&_ctx, INTERFACE_A);
//            if (ir) {
//                abort();
//            }
//            
//            // If `device` wasn't specified, use the first one with the default VID/PID
//            struct ftdi_device_list* devices = NULL;
//            int devicesCount = ftdi_usb_find_all(&_ctx, &devices, 0x403, 0x6014);
//            if (devicesCount < 1) {
//                abort();
//            }
//            
//            // Open FTDI USB device
//            ir = ftdi_usb_open_dev(&_ctx, devices[0].dev);
//            if (ir) {
//                abort();
//            }
            
            int ir = ftdi_usb_open_dev(&_ctx, _usbDev);
            _checkErr(ir, "ftdi_usb_open_dev failed");
            
//            printf("OPENED\n");
//            
//            ir = ftdi_tcioflush(&_ctx);
//            _checkErr(ir, "ftdi_tcioflush failed");
        }
        
        void close() {
            int ir = ftdi_usb_close(&_ctx);
            _checkErr(ir, "ftdi_usb_close failed");
        }
        
        void usbReset() {
            int ir = ftdi_usb_reset(&_ctx);
            _checkErr(ir, "ftdi_usb_reset failed");
        }
        
        void setBitmode(uint8_t mode, uint8_t pinDirs) {
            int ir = ftdi_set_bitmode(&_ctx, pinDirs, mode);
            _checkErr(ir, "ftdi_set_bitmode failed");
        }
        
        void read(void* data, size_t len) {
            size_t off = 0;
            while (off < len) {
                int ir = ftdi_read_data(&_ctx, (uint8_t*)data+off, len-off);
                _checkErr(ir, "ftdi_read_data failed");
                printf("ftdi_read_data returned: %d\n", ir);
                off += ir;
            }
        }
        
        size_t readAny(void* data, size_t len) {
            int ir = ftdi_read_data(&_ctx, (uint8_t*)data, len);
            _checkErr(ir, "ftdi_read_data failed");
            return (size_t)ir;
        }
        
        void write(const void* data, size_t len) {
            int ir = ftdi_write_data(&_ctx, (const uint8_t*)data, len);
            _checkErr(ir, "ftdi_write_data failed");
            if ((size_t)ir != len)
                throw RuntimeError("short write (attempted=%zu, wrote=%zu)", len, (size_t)ir);
        }
        
        void setEventChar(uint8_t eventch, uint8_t enable) {
            int ir = ftdi_set_event_char(&_ctx, eventch, enable);
            _checkErr(ir, "ftdi_set_event_char failed");
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
    
//    static struct ftdi_context _FTDIContextCreate() {
//        struct ftdi_context ctx;
//        int ir = ftdi_init(&ctx);
//        _checkErr(ir, "ftdi_init failed");
//        return ctx;
//    }
    
    
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
    
    const uint8_t _testPin = 0;
    const uint8_t _rstPin = 0;
    _FTDIDevice _dev;
    std::vector<uint8_t> _cmds;
    std::vector<uint8_t> _readData;
    size_t _readLen = 0;
};
