#include "MSPProbeSim.h"
#include "MSPInterface.h"

class MSPInterfaceDummy : public MSPInterface {
    void sbwPins(PinState test, PinState rst) {
        printf("sbwPins\n");
    }
    
    void sbwIO(bool tms, bool tclk, bool tdi, bool tdoRead) {
        printf("sbwIO\n");
    }
    
    void sbwRead(void* buf, size_t len) { {
        printf("sbwRead\n");
    }
};

int main(int argc, const char* argv[]) {
    MSPInterfaceDummy msp;
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
