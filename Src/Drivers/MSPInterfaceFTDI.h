#pragma once
#include <libftdi1/ftdi.h>
#include "USBDevice.h"

class MSPInterfaceFTDI : public MSPInterface {
public:
    
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
        _dev.setInterface(INTERFACE_A);
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
        // If we're reading: pulse TEST=[0,1] and read RST on rising edge
        if (tdoRead) {
            _cmds.push_back(MPSSE::ClockBitsInPosEdgeMSB);
            _cmds.push_back(0x00);
            _readLen++;
        
        // If we're not reading, just pulse TEST=[0,1] (this also writes
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
    
    void sbwIO(bool tms, bool tclk, bool tdi, bool tdoRead) {
        // Write TMS
        {
            // This TMS sequence is the trickiest part of this driver, purely
            // because MSP430 requires that TCLK be restored during the
            // low-phase of TEST (SBWTCK).
            // 
            // Unfortunately there's no way to do that with the MPSSE clocking
            // command set (eg ClockBitsOutPosEdgeMSB). This is because the MPSSE
            // clocking commands only allow TDO to be read or TDI to be written.
            // Since RST=TDO, it can only be read by the clocking commands, and
            // therefore we have to manually bit-bang the pins.
            // 
            // The problem with bit-banging the pins is that the low phase of TEST
            // (SBWTCK) is timing critical; if it's too long (>7us), the MSP430 will
            // exit the SBW debug mode. Therefore we go to great lengths to ensure
            // that the commands written to FTDI will be handled 'atomically'. To do
            // so, the flush scheme:
            // 
            //   - groups commands so that the TEST=[0,1] pulse doesn't get split up
            //     (implemented by having all MSPInterface functions call _flush()
            //     after enqueueing their commands), and
            //   
            //   - ensures that it never writes a set of commands larger than the
            //     FTDI's USB max packet length, to ensure that they'll be delivered
            //     as one packet.
            //
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
        }
        
        // Short-circuit if the flush isn't required, and we're below the threshold
        if (!required && (_cmds.size() <= _flushThreshold)) return;
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
        // We use _Swap() to temporarily replace the last 2 commands in _cmds with `extraCmds`.
        // This is a little gross, but it beats copying the whole buffer or calling
        // _dev.write() twice.
        _Swap(_cmds, _flushCmdLen, extraCmdsLen, extraCmds);
        _dev.write(_cmds.data(), writeLen);
        _Swap(_cmds, _flushCmdLen, extraCmdsLen, extraCmds);
        
        // Shift remaining commands to the beginning of _cmds, resize _cmds,
        // and update _flushCmdLen
        std::move(_cmds.begin()+_flushCmdLen, _cmds.end(), _cmds.begin());
        _cmds.resize(_cmds.size()-_flushCmdLen);
        _flushCmdLen = _cmds.size();
        
        // Read expected amount of data into the end of `_readData`
        // We expect 2 extra bytes: {MPSSE::BadCommandResp, MPSSE::BadCommand}
        constexpr size_t ResponseExtraByteCount = 2;
        const size_t readLen = _flushReadLen+ResponseExtraByteCount;
        const size_t off = _readData.size();
        _readData.resize(_readData.size() + readLen);
        
//        printf("[FTDI] reading %zu bytes (_flushReadLen=%zu, _readLen=%zu)\n",
//            readLen-ResponseExtraByteCount, _flushReadLen, _readLen);
        
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
        
        // Logic error if the requested length exceeds available data
        assert(len*8 <= _readData.size());
        
        uint8_t* buf8 = (uint8_t*)buf;
        for (size_t i=7; i<len*8; i+=8) {
            *buf8 = _readData[i];
            buf8++;
        }
        
        _readData.erase(_readData.begin(), _readData.begin()+len*8);
    }
    
private:
    
    class _FTDIDevice {
    public:
        _FTDIDevice(USBDevice&& usbDev) :
        _usbDev(std::move(usbDev)) {
            int ir = ftdi_init(&_ctx);
            _checkErr(ir, "ftdi_init failed");
        }
        
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
        
        void usbReset() {
            int ir = ftdi_usb_reset(&_ctx);
            _checkErr(ir, "ftdi_usb_reset failed");
        }
        
        void setInterface(enum ftdi_interface iface) {
            int ir = ftdi_set_interface(&_ctx, iface);
            _checkErr(ir, "ftdi_set_interface failed");
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
        
    private:
        void _checkErr(int ir, const char* errMsg) {
            if (ir < 0) throw RuntimeError("%s: %s", errMsg, ftdi_get_error_string(&_ctx));
        }
        
        struct ftdi_context _ctx = {};
        USBDevice _usbDev;
    };
    
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
