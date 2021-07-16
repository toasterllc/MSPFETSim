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
    virtual void open() {}
    virtual void close() {}
    virtual size_t readLenMax() const { return 0; }
};

class MSPInterfaceJTAG : public MSPInterface {
};

class MSPInterfaceSBW : public MSPInterface {
public:
    // pins(): Sets the state of the SBW pins
    virtual void pins(PinState test, PinState rst) = 0;
    
    // io(): Performs a single SBW IO cycle
    //   If tdoRead=true, the TDO output bit should be shifted into persistent storage
    //   for later retrieval via `read()`.
    virtual void io(bool tms, bool tclk, bool tdi, bool tdoRead) = 0;
    
    // read(): Retrieves data previously stored via io()
    //   For optimal performance, IO operations should be queued until read() is called, at
    //   which point the queued operations should be flushed to the device.
    //   
    //   len==0 is valid and must flush outstanding IO operations without returning any data.
    //   
    //   The maximum length that the device can store/read can be configured via `readLenMax`.
    virtual void read(void* buf, size_t len) = 0;
};
