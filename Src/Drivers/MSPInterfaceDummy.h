#pragma once
#include "MSPInterface.h"

class MSPInterfaceDummy : public MSPInterface {
    void sbwTestSet(bool val) {
        printf("sbwTestSet(%d)\n", val);
    }
    
    void sbwRstSet(bool val) {
        printf("sbwRstSet(%d)\n", val);
    }
    
    void sbwIO(bool tms, bool tclk, bool tdi, bool tdoRead) override {
        printf("sbwIO(%d %d %d %d)\n", tms, tclk, tdi, tdoRead);
    }
    
    void sbwRead(void* buf, size_t len) override {
        printf("sbwRead(%p, %zu)\n", buf, len);
    }
};
