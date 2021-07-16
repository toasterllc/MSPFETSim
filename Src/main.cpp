#include "MSPProbeSim.h"
#include "MSPInterface.h"

class MSPInterfaceSBWDummy : public MSPInterfaceSBW {
    void pins(PinState test, PinState rst) override {
        printf("SBW pins()\n");
    }
    
    void io(bool tms, bool tclk, bool tdi, bool tdoRead) override {
        printf("SBW io()\n");
    }
    
    void read(void* buf, size_t len) override {
        printf("SBW read()\n");
    }
};

int main(int argc, const char* argv[]) {
    MSPInterfaceSBWDummy msp;
    MSPProbeSim probeSim(msp);
    try {
        probeSim.run();
    
    } catch (const std::exception& e) {
        fprintf(stderr, "Error: %s\n", e.what());
        fprintf(stderr, "Exiting...\n");
        return 1;
    }
    
    return 0;
}
