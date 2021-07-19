#pragma once
#include "MSPInterface.h"

class MSPInterfaceDummy : public MSPInterface {
    void sbwPins(PinState test, PinState rst) override {
        printf("SBW pins()\n");
    }
    
    void sbwIO(bool tms, bool tclk, bool tdi, bool tdoRead) override {
        printf("SBW io()\n");
    }
    
    void sbwRead(void* buf, size_t len) override {
        printf("SBW read()\n");
    }
};
