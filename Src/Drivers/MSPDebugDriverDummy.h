#pragma once
#include "MSPDebugDriver.h"

class MSPDebugDriverDummy : public MSPDebugDriver {
    void sbwTestSet(bool val) {
        printf("sbwTestSet(%d)\n", val);
    }
    
    void sbwRstSet(bool val) {
        printf("sbwRstSet(%d)\n", val);
    }
    
    void sbwTestPulse() {
        printf("sbwTestPulse()\n");
    }
    
    void sbwIO(bool tms, bool tclk, bool tdi, bool tdoRead) override {
        printf("sbwIO(%d %d %d %d)\n", tms, tclk, tdi, tdoRead);
    }
    
    void sbwRead(void* buf, size_t len) override {
        printf("sbwRead(%p, %zu)\n", buf, len);
    }
};
