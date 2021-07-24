#include "MSPProbeSim.h"
#include "MSPInterface.h"
#include "Drivers/MSPInterfaceDummy.h"
#include "Drivers/MSPInterfaceMDC.h"
#include "Drivers/MSPInterfaceFTDI.h"

std::unique_ptr<MSPInterface> CreateMSPInterface() {
    // Dummy
//    return std::make_unique<MSPInterfaceDummy>();
    
    // MDC
#if MSPInterfaceMDCExists
    auto devices = MSPInterfaceMDC::GetDevices();
    if (devices.empty()) throw RuntimeError("no matching USB devices");
    if (devices.size() > 1) throw RuntimeError("more than one matching USB device");
    return std::make_unique<MSPInterfaceMDC>(std::move(devices[0]));
#endif
    
//    // FTDI
//    auto devices = MSPInterfaceFTDI::GetDevices();
//    if (devices.empty()) throw RuntimeError("no matching FTDI devices");
//    if (devices.size() > 1) throw RuntimeError("more than one matching FTDI device");
//    return std::make_unique<MSPInterfaceFTDI>(std::move(devices[0]));
}

int main(int argc, const char* argv[]) {
    try {
        auto msp = CreateMSPInterface();
        MSPProbeSim probeSim(*msp);
        probeSim.run();
    
    } catch (const std::exception& e) {
        fprintf(stderr, "Error: %s\n", e.what());
        return 1;
    }
    
    return 0;
}
