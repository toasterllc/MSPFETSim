#pragma once

class MSPDebugDriver {
public:
    virtual ~MSPDebugDriver() {}
    
    // sbwTestSet() / sbwRstSet(): Set output value of a pin
    virtual void sbwTestSet(bool val) = 0;
    virtual void sbwRstSet(bool val) = 0;
    
    // sbwTestPulse(): pulse Test pin ->0->1
    //   Semantically this is identical to: sbwTestSet(0), sbwTestSet(1), but the timing
    //   of this operation is critical, so the driver must ensure that the duration of
    //   the low (0) state is short enough (<7us) to prevent the MSP430 from exiting SBW
    //   mode.
    virtual void sbwTestPulse() = 0;
    
    // sbwIO(): Performs a Spy-bi-wire IO cycle
    //   If tdoRead=true, the TDO output bit should be shifted into persistent storage
    //   for later retrieval via `read()`.
    virtual void sbwIO(bool tms, bool tclk, bool tdi, bool tdoRead) = 0;
    
    // sbwRead(): Retrieves data previously stored via sbwIO()
    //   For optimal performance, IO operations should be queued until sbwRead() is called, at
    //   which point the queued operations should be flushed to the device.
    //   
    //   When sbwRead() is called, the amount of queued data (ie the number of sbwIO operations
    //   where tdoRead=1) is guaranteed to be a byte multiple, so the implementation doesn't
    //   need to concern itself with returing partial bytes.
    //   
    //   len==0 is valid and must flush outstanding IO operations without returning any data.
    virtual void sbwRead(void* buf, size_t len) = 0;
};
