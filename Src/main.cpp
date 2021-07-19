#include "MSPProbeSim.h"
#include "MSPInterface.h"
#include "Drivers/STMBridge.h"

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

int main(int argc, const char* argv[]) {
    try {
        auto devices = STMBridge::GetMatchingDevices();
        if (devices.empty()) throw RuntimeError("no matching USB devices");
        if (devices.size() > 1) throw RuntimeError("more than one matching USB device");
        
//        MSPInterfaceDummy msp;
        STMBridge msp(std::move(devices[0]));
        MSPProbeSim probeSim(msp);
        probeSim.run();
    
    } catch (const std::exception& e) {
        fprintf(stderr, "Error: %s\n", e.what());
        fprintf(stderr, "Exiting...\n");
        return 1;
    }
    
    return 0;
}
