#pragma once
#include <libftdi1/ftdi.h>
#include "USBDevice.h"

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
            std::mutex lock = {};
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
    
    static size_t _GetMaxPacketSize(USBDevice& dev) {
        const auto configDesc = dev.getConfigDescriptor(0);
        if (configDesc->bNumInterfaces < 1) throw RuntimeError("configuration descriptor has no interfaces");
        const auto iface = configDesc->interface[0];
        if (iface.num_altsetting < 1) throw RuntimeError("interface has no interfaces");
        const auto ifaceDesc = iface.altsetting[0];
        if (ifaceDesc.bNumEndpoints < 1) throw RuntimeError("interface descriptor has no endpoints");
        return ifaceDesc.endpoint[0].wMaxPacketSize;
    }
    
    static size_t _GetFlushThreshold(size_t maxPacketSize) {
        // Flush 2 bytes before _cmds hits _maxPacketSize.
        // This needs to be at least 2 bytes to account for the 2 extra MPSSE
        // commands we send when flushing.
        return maxPacketSize-2;
    }
    
    MSPInterfaceFTDI(USBDevice&& dev) :
    _maxPacketSize(_GetMaxPacketSize(dev)),
    _flushThreshold(_GetFlushThreshold(_maxPacketSize)),
    _dev(std::move(dev))
    {
        _dev.open();
        _dev.usbReset();
        _dev.setEventChar(0, 0);
        
        _dev.setBitmode(BITMODE_RESET, 0);
        _dev.setBitmode(BITMODE_MPSSE, 0);
        _dev.setLatencyTimer(1); // Improves performance significantly
        // TODO: does setting the baud rate apply to MPSSE?
//        _dev.setBaudRate(1<<21);
        
        constexpr uint8_t initCmd[] = {
//            MPSSE::SetClkDivisor, 0, 0,         // Set TCK frequency to 6MHz
            MPSSE::SetClkDivisor, 14, 0,        // Set TCK frequency to 400kHz
//            MPSSE::SetClkDivisor, 29, 0,        // Set TCK frequency to 200kHz
//            MPSSE::SetClkDivisor, 59, 0,        // Set TCK frequency to 100kHz
//            MPSSE::SetClkDivisor, 119, 0,       // Set TCK frequency to 50kHz
        };
        
        _dev.write(initCmd, sizeof(initCmd));
        
        // TODO: in the future we should use ftdi_tcioflush(), but that's only available in
        // TODO: the latest libftdi, and we don't want to require that yet
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
        
        // Synchronize with FTDI by sending a bad command and ensuring we get the expected error
        _dev.write(&MPSSE::BadCommand, 1);
        uint8_t resp[2];
        _dev.read(resp, sizeof(resp));
        if (!(resp[0]==MPSSE::BadCommandResp && resp[1]==MPSSE::BadCommand))
            throw RuntimeError("FTDI sync failed (expected: <%x %x> got: <%x %x>)", MPSSE::BadCommandResp, MPSSE::BadCommand, resp[0], resp[1]);
        
        // Configure pins
        // They need to default to the output state because our internal functions (eg _testPulse())
        // assume that's the state of the pins, and otherwise they won't work.
        _pinsSet(_PinState::Out1, _PinState::Out1);
    }
    
    ~MSPInterfaceFTDI() {}
    
    struct MPSSE {
        static constexpr uint8_t ClockBitsOutPosEdgeMSB     = 0x12;
        static constexpr uint8_t ClockBitsInPosEdgeMSB      = 0x22;
        static constexpr uint8_t SetDataBitsL               = 0x80;
        static constexpr uint8_t ReadDataBitsL              = 0x81;
        static constexpr uint8_t SetDataBitsH               = 0x82;
        static constexpr uint8_t ReadDataBitsH              = 0x83;
        static constexpr uint8_t DisconnectTDITDOLoopback   = 0x85;
        static constexpr uint8_t SetClkDivisor              = 0x86;
        static constexpr uint8_t SendImmediate              = 0x87;
        static constexpr uint8_t DisableClkDivideBy5        = 0x8A;
        static constexpr uint8_t Disable3PhaseDataClocking  = 0x8D;
        static constexpr uint8_t DisableAdaptiveClocking    = 0x97;
        static constexpr uint8_t BadCommandResp             = 0xFA;
        static constexpr uint8_t BadCommand                 = 0xAB;
    };
    
    enum _PinState {
        Out0,
        Out1,
        In,
    };
    
    _PinState _testState = _PinState::In;
    _PinState _rstState = _PinState::In;
    void _pinsSet(_PinState t, _PinState r) {
        // Short-circuit if the state hasn't changed
        if (t==_testState && r==_rstState) return;
        _testState = t;
        _rstState = r;
        
        const uint8_t dir =
            (t!=_PinState::In ? _TestPin : 0) |
            (r!=_PinState::In ? _RstPin  : 0) ;
        
        const uint8_t val =
            (t==_PinState::Out1 ? _TestPin : 0) |
            (r==_PinState::Out1 ? _RstPin  : 0) ;
        
        _cmds.push_back(MPSSE::SetDataBitsL);
        _cmds.push_back(val); // Value
        _cmds.push_back(dir); // Direction
    }
    
    void _testSet(_PinState t) {
        _pinsSet(t, _rstState);
    }
    
    void _rstSet(_PinState r) {
        _pinsSet(_testState, r);
    }
    
    void _testPulse(bool tdoRead) {
        // If we're reading: pulse TEST [0,1] and read RST on rising edge
        if (tdoRead) {
            _cmds.push_back(MPSSE::ClockBitsInPosEdgeMSB);
            _cmds.push_back(0x00);
            _readLen++;
        
        // If we're not reading, just pulse TEST [0,1] (this also writes
        // TDI, but that's a nop since TDI is an input)
        } else {
            _cmds.push_back(MPSSE::ClockBitsOutPosEdgeMSB);
            _cmds.push_back(0x00);
            _cmds.push_back(0x00);
        }
    }
    
    void sbwTestSet(bool test) override {
        _testSet(test ? _PinState::Out1 : _PinState::Out0);
        _flush();
    }
    
    void sbwTestPulse() override {
        _testPulse(false);
        _flush();
    }
    
    void sbwRstSet(bool rst) override {
        _rstSet(rst ? _PinState::Out1 : _PinState::Out0);
        _flush();
    }
    
//    void sbwPins(PinState test, PinState rst) override {
//        const PinState t0 = (test!=PinState::Pulse01 ? test : PinState::Out0);
//        const PinState r0 = (rst !=PinState::Pulse01 ? rst  : PinState::Out0);
//        const PinState t1 = (test!=PinState::Pulse01 ? test : PinState::Out1);
//        const PinState r1 = (rst !=PinState::Pulse01 ? rst  : PinState::Out1);
//        
//        _pinsSet(t0, r0);
//        _pinsSet(t1, r1);
//        
////        if (pulse) {
////            // TODO: flush existing commands if we're pulsing, to ensure that these commands are atomically received, otherwise the pulse signal could be stretched
////        }
////        
////        
////        const uint8_t dir =
////            (test!=PinState::In ? _TestPin : 0) |
////            ( rst!=PinState::In ? _RstPin  : 0) ;
////        
////        const uint8_t val0 =
////            (test==PinState::Out1 ? _TestPin : 0) |
////            ( rst==PinState::Out1 ? _RstPin  : 0) ;
////        
////        _cmds.push_back(MPSSE::SetDataBitsL);
////        _cmds.push_back(val0); // Value
////        _cmds.push_back(dir); // Direction
////        
////        if (pulse) {
////            const uint8_t val1 =
////                (test==PinState::Out1||test==PinState::Pulse01 ? _TestPin : 0) |
////                ( rst==PinState::Out1|| rst==PinState::Pulse01 ? _RstPin  : 0) ;
////            
////            _cmds.push_back(MPSSE::SetDataBitsL);
////            _cmds.push_back(val1); // Value
////            _cmds.push_back(dir); // Direction
////        }
//    }
    
//    RST::Write(tms);
//    TEST::Write(0);
//    tdo = RST::Read();
//    RST::Write(tclk);
//    TEST::Write(1);
    
//    void _flushPoint() {
//        // If we're over the flush threshold, flush outstanding commands
//        if (_cmds.size() > _flushThreshold) {
//            _flush();
//        }
//        _flushCmdLen = _cmds.size();
//    }
    
//    void sbwIO(bool tms, bool tclk, bool tdi, bool tdoRead) override {
//        // Write TMS
//        {
//            _rstSet(tms ? _PinState::Out1 : _PinState::Out0);
//            _testSet(_PinState::Out0);
//            _rstSet(tclk ? _PinState::Out1 : _PinState::Out0);
//            _testSet(_PinState::Out1);
//        }
//        
//        // Write TDI
//        {
//            _rstSet(tdi ? _PinState::Out1 : _PinState::Out0);
//            _testSet(_PinState::Out0);
//            _testSet(_PinState::Out1);
//            _rstSet(_PinState::In);
//        }
//        
//        // Read TDO
//        {
//            _testSet(_PinState::Out0);
//            if (tdoRead) _read();
//            _testSet(_PinState::Out1);
//            _rstSet(tdi ? _PinState::Out1 : _PinState::Out0);
//        }
//        
//        _flush();
//    }
    
    void sbwIO(bool tms, bool tclk, bool tdi, bool tdoRead) {
        // Write TMS
        {
            _rstSet(tms ? _PinState::Out1 : _PinState::Out0);
            _testSet(_PinState::Out0);
            _rstSet(tclk ? _PinState::Out1 : _PinState::Out0);
            _testSet(_PinState::Out1);
        }
        
        // Write TDI
        {
            _rstSet(tdi ? _PinState::Out1 : _PinState::Out0);
            _testPulse(false);
            _rstSet(_PinState::In);
        }
        
        // Read TDO
        {
            _testPulse(tdoRead);
            _rstSet(tdi ? _PinState::Out1 : _PinState::Out0);
        }
        
        _flush();
    }
    
    // _Swap(): Swap `len` elements in vector `a`, at offset `off`, with the entire vector `b`.
    // As a convenience, `off+len` can extend past the end of `a`, in which case
    // `len` will be clamped so that `off+len` is at the end of `a`.
    template <typename T>
    void _Swap(std::vector<T>& a, size_t off, size_t len, std::vector<T>& b) {
        len = std::min(len, a.size()-off); // Allow `off+len` to extend past the end of `a`
        
        assert(off <= a.size());
        assert(len <= a.size()-off);
        
        const size_t aOldSize = a.size();
        const size_t bOldSize = b.size();
        const size_t aNewSize = aOldSize+bOldSize-len;
        const size_t bNewSize = len;
        
        if (aNewSize > aOldSize) {
            a.resize(aNewSize);
            // Shift all elements in `a` after off+len
            std::move(a.begin()+off+len, a.begin()+aOldSize, a.begin()+off+bOldSize);
        }
        
        if (bNewSize > bOldSize) b.resize(bNewSize);
        
        // Swap elements
        std::swap_ranges(a.begin()+off, a.begin()+off+std::max(bNewSize,bOldSize), b.begin());
        
        if (aNewSize < aOldSize) {
            // Shift all elements in `a` after off+len
            std::move(a.begin()+off+len, a.begin()+aOldSize, a.begin()+off+bOldSize);
        }
        
        a.resize(aNewSize);
        b.resize(bNewSize);
    }
    
    void _flush(bool required=false) {
        if (_cmds.size() <= _flushThreshold) {
            // Remember the last flush point
            _flushCmdLen = _cmds.size();
            _flushReadLen = _readLen;
            
            // Short-circuit if the flush isn't required, and we're below the threshold
            if (!required) return;
        }
        
        // Short-circuit if there aren't any commands to flush
        if (!_flushCmdLen) return;
        assert(_cmds.size() >= _flushCmdLen);
        assert(_readLen >= _flushReadLen);
        
        std::vector<uint8_t> extraCmds = {
            MPSSE::BadCommand,
            MPSSE::SendImmediate,
        };
        
        const size_t extraCmdsLen = extraCmds.size();
        const size_t writeLen = _flushCmdLen+extraCmdsLen;
        
//        printf("[FTDI] flushing %zu commands (_flushThreshold=%zu, _maxPacketSize=%zu)\n",
//            writeLen, _flushThreshold, _maxPacketSize);
        
        // Logic error if our commands are larger than FTDI's USB max packet size
        assert(writeLen <= _maxPacketSize);
        
        // Write the commands
        _Swap(_cmds, _flushCmdLen, extraCmdsLen, extraCmds); // Swap the extra commands into the buffer
        _dev.write(_cmds.data(), writeLen);
        _Swap(_cmds, _flushCmdLen, extraCmdsLen, extraCmds); // Swap the extra commands out of the buffer
        
        // Shift remaining commands to the beginning of _cmds,
        // resize _cmds, and update _flushCmdLen
        std::move(_cmds.begin()+_flushCmdLen, _cmds.end(), _cmds.begin());
        _cmds.resize(_cmds.size()-_flushCmdLen);
        _flushCmdLen = _cmds.size();
        
        // Read expected amount of data into the end of `_readData`
        // We expect 2 extra bytes: {MPSSE::BadCommandResp, MPSSE::BadCommand}
        constexpr size_t ResponseExtraByteCount = 2;
        const size_t readLen = _flushReadLen+ResponseExtraByteCount;
        const size_t off = _readData.size();
        _readData.resize(_readData.size() + readLen);
        
        printf("[FTDI] reading %zu bytes (_flushReadLen=%zu, _readLen=%zu)\n",
            readLen-ResponseExtraByteCount, _flushReadLen, _readLen);
        
        _dev.read(_readData.data()+off, readLen);
        _readLen -= _flushReadLen;
        _flushReadLen = _readLen;
        
        // Verify that the 2 extra response bytes are what we
        // expect: {MPSSE::BadCommandResp, MPSSE::BadCommand}
        const uint8_t* back = &(*(_readData.end()-ResponseExtraByteCount));
        if (!(back[0]==MPSSE::BadCommandResp && back[1]==MPSSE::BadCommand)) {
            throw RuntimeError("FTDI sync failed (expected: <%x %x>, got: <%x %x>)",
                MPSSE::BadCommandResp, MPSSE::BadCommand, back[0], back[1]);
        }
        
        // Remove 2 extra response bytes
        _readData.pop_back();
        _readData.pop_back();
    }
    
    void sbwRead(void* buf, size_t len) override {
        _flush(true);
        
        // Verify that the requested length isn't greater than
        // the amount of available data
        assert(len*8 <= _readData.size());
        
//        const size_t tmpBufLen = len*8;
//        auto tmpBufUnique = std::make_unique<uint8_t[]>(tmpBufLen);
//        uint8_t* tmpBuf = tmpBufUnique.get();
//        _dev.read(tmpBuf, tmpBufLen);
        
        // Read every 8th byte, which contains all 8 shifted bits
        printf("Read data (available=%zu, read len=%zu):\n", _readData.size(), len);
        for (size_t i=0; i<_readData.size(); i++) {
            printf("[%zu] %02x\n", i, _readData[i]);
        }
        printf("\n\n\n");
        
        uint8_t* buf8 = (uint8_t*)buf;
        for (size_t i=7; i<len*8; i+=8) {
//            printf("Read data: 0x%x\n", _readData[i]);
            *buf8 = _readData[i];
            buf8++;
        }
        
//        for (size_t ireadData=0, ibit=0, ibuf=0; ireadData<len*8; ireadData++) {
//            printf("Read data: 0x%x\n", _readData[ireadData]);
//            const bool bit = _readData[ireadData] & 0x1; // We read one bit at a time (via _testPulse()), which FTDI puts at bit 0
//            buf8[ibuf] <<= 1;
//            buf8[ibuf] |= bit;
//            ibit++;
//            if (ibit == 8) {
//                ibuf++;
//                ibit = 0;
//            }
//        }
        
        _readData.erase(_readData.begin(), _readData.begin()+len*8);
        
//        uint8_t* buf8 = (uint8_t*)buf;
//        for (size_t ibit=0, ibyte=0, byteLen=0; ibit<tmpBufLen; ibit++) {
//            const bool bit = tmpBuf[ibit] & _RstPin;
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
        
        void setLatencyTimer(uint8_t timer) {
            int ir = ftdi_set_latency_timer(&_ctx, timer);
            _checkErr(ir, "ftdi_set_latency_timer failed");
        }
        
        void setBaudRate(uint32_t baud) {
            int ir = ftdi_set_baudrate(&_ctx, (int)baud);
            _checkErr(ir, "ftdi_set_bitmode failed");
        }
        
        void read(void* data, size_t len) {
            size_t off = 0;
            while (off < len) {
                int ir = ftdi_read_data(&_ctx, (uint8_t*)data+off, len-off);
                _checkErr(ir, "ftdi_read_data failed");
//                printf("ftdi_read_data returned: %d\n", ir);
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
        
        struct ftdi_context _ctx = {};
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
    
    static constexpr uint8_t _TestPin   = 1<<0; // FTDI's TCK
    static constexpr uint8_t _RstPin    = 1<<2; // FTDI's TDO
    const size_t _maxPacketSize = 0;
    const size_t _flushThreshold = 0;
    _FTDIDevice _dev;
    std::vector<uint8_t> _cmds = {};
    std::vector<uint8_t> _readData = {};
    size_t _flushCmdLen = 0;
    size_t _flushReadLen = 0;
    size_t _readLen = 0;
};
