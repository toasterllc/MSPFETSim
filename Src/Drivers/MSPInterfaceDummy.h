#pragma once
#include "MSPInterface.h"

class MSPInterfaceDummy : public MSPInterface {
    
    void sbwPin(Pin pin, Op op) override {
        printf("sbwPin(%d, %d)\n", (int)pin, (int)op);
    }
    
    void sbwFlush() override {
        printf("sbwFlush()\n");
    }
    
    void sbwRead(void* buf, size_t len) override {
        printf("sbwRead(%p, %zu)\n", buf, len);
    }
};
