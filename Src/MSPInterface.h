#pragma once

class MSPInterface {
public:
    virtual ~MSPInterface() {}
    
    // sbwTestSet() / sbwRstSet(): Set output value of a pin
    virtual void sbwTestSet(bool val) {}
    virtual void sbwRstSet(bool val) {}
    
    // sbwTestPulse(): pulse Test pin ->0->1
    //   Semantically this is identical to: sbwTestSet(0), sbwTestSet(1), but the timing
    //   of this operation is critical, so the driver must ensure that the duration of
    //   the low (0) state is short enough (<7us) to prevent the MSP430 from exiting SBW
    //   mode.
    virtual void sbwTestPulse() {}
    
    // sbwIO(): Performs a Spy-bi-wire IO cycle
    //   If tdoRead=true, the TDO output bit should be shifted into persistent storage
    //   for later retrieval via `read()`.
    virtual void sbwIO(bool tms, bool tclk, bool tdi, bool tdoRead) {}
    
    // read(): Retrieves data previously stored via sbwIO()
    //   For optimal performance, IO operations should be queued until read() is called, at
    //   which point the queued operations should be flushed to the device.
    //   
    //   len==0 is valid and must flush outstanding IO operations without returning any data.
    virtual void sbwRead(void* buf, size_t len) {}
};
