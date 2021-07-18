#include "MSPProbeSim.h"
#include "MSPInterface.h"
#include "STMBridge.h"

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
        libusb_device* usbDevice = STMBridge::FindUSBDevice();
        if (!usbDevice) throw RuntimeError("no matching USB devices");
        Defer(libusb_unref_device(usbDevice));
        
//        MSPInterfaceDummy msp;
        STMBridge msp(usbDevice);
        MSPProbeSim probeSim(msp);
        probeSim.run();
    
    } catch (const std::exception& e) {
        fprintf(stderr, "Error: %s\n", e.what());
        fprintf(stderr, "Exiting...\n");
        return 1;
    }
    
    return 0;
}
