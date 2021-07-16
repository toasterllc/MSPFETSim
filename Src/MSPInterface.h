#pragma once

class MSPInterface {
public:
    enum class PinState : uint8_t {
        Out0,
        Out1,
        In,
        Pulse01,
    };
    
    void open() {}
    void close() {}
    
    size_t readLenMax() const { return 0; }
    
    // sbwPins(): Sets the state of the SBW pins
    void sbwPins(PinState test, PinState rst) { _unimplemented(); }
    
    // sbwIO(): Performs a single SBW IO cycle
    //   If tdoRead=true, the TDO output bit should be shifted into persistent storage
    //   for later retrieval via `sbwRead()`.
    void sbwIO(bool tms, bool tclk, bool tdi, bool tdoRead) { _unimplemented(); }
    
    // sbwRead(): Retrieves data previously stored via sbwIO()
    //   For optimal performance, IO operations should be queued until sbwRead() is called, at
    //   which point the queued operations should be flushed to the device.
    //   
    //   len==0 is valid and must flush outstanding IO operations without returning any data.
    //   
    //   The maximum length that the device can store/read can be configured via `readLenMax`.
    void sbwRead(void* buf, size_t len) { _unimplemented(); }
    
private:
    void _unimplemented() const {
        throw std::runtime_error("unimplemented function");
    }
};
