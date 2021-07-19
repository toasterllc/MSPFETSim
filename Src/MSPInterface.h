#pragma once

class MSPInterface {
public:
    enum class PinState : uint8_t {
        Out0,
        Out1,
        In,
        Pulse01,
    };
    
    virtual ~MSPInterface() {}
    
    virtual size_t readLenMax() const { return 0; }
    
//    virtual void open() {}
//    virtual void close() {}
    
    virtual void sbwTest(PinState test) {}
    
    virtual void sbwRst(PinState rst) {}
    
    // pins(): Sets the state of the SBW pins
    virtual void sbwPins(PinState test, PinState rst) {}
    
    // io(): Performs a single SBW IO cycle
    //   If tdoRead=true, the TDO output bit should be shifted into persistent storage
    //   for later retrieval via `read()`.
    virtual void sbwIO(bool tms, bool tclk, bool tdi, bool tdoRead) {}
    
    // read(): Retrieves data previously stored via io()
    //   For optimal performance, IO operations should be queued until read() is called, at
    //   which point the queued operations should be flushed to the device.
    //   
    //   len==0 is valid and must flush outstanding IO operations without returning any data.
    //   
    //   The maximum length that the device can store/read can be configured via `readLenMax`.
    virtual void sbwRead(void* buf, size_t len) {}
};
