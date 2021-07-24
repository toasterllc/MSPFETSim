#include "MSPFETSim.h"
#include "Drivers/MSPDebugDriver.h"
#include "Drivers/MSPDebugDriverDummy.h"
#include "Drivers/MSPDebugDriverMDC.h"
#include "Drivers/MSPDebugDriverFTDI.h"

std::unique_ptr<MSPDebugDriver> CreateDriver() {
    // Dummy
//    return std::make_unique<MSPDebugDriverDummy>();
    
//    // MDC
//#if MSPDebugDriverMDCExists
//    auto devices = MSPDebugDriverMDC::GetDevices();
//    if (devices.empty()) throw RuntimeError("no matching USB devices");
//    if (devices.size() > 1) throw RuntimeError("more than one matching USB device");
//    return std::make_unique<MSPDebugDriverMDC>(std::move(devices[0]));
//#endif
    
    // FTDI
    auto devices = MSPDebugDriverFTDI::GetDevices();
    if (devices.empty()) throw RuntimeError("no matching FTDI devices");
    if (devices.size() > 1) throw RuntimeError("more than one matching FTDI device");
    return std::make_unique<MSPDebugDriverFTDI>(std::move(devices[0]));
}

int main(int argc, const char* argv[]) {
    try {
        auto msp = CreateDriver();
        MSPFETSim probeSim(*msp);
        probeSim.run();
    
    } catch (const std::exception& e) {
        fprintf(stderr, "Error: %s\n", e.what());
        return 1;
    }
    
    return 0;
}
