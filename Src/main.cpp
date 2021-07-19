#include "MSPProbeSim.h"
#include "MSPInterface.h"
#include "Drivers/MSPInterfaceDummy.h"
#include "Drivers/MSPInterfaceMDC.h"
#include "Drivers/MSPInterfaceFTDI.h"

int main(int argc, const char* argv[]) {
    try {
//        MSPInterfaceDummy msp;
        
//        auto devices = MSPInterfaceMDC::GetDevices();
//        if (devices.empty()) throw RuntimeError("no matching USB devices");
//        if (devices.size() > 1) throw RuntimeError("more than one matching USB device");
//        MSPInterfaceMDC msp(std::move(devices[0]));
        
        auto devices = MSPInterfaceFTDI::GetDevices();
        if (devices.empty()) throw RuntimeError("no matching FTDI devices");
        if (devices.size() > 1) throw RuntimeError("more than one matching FTDI device");
        MSPInterfaceFTDI msp(std::move(devices[0]));
        
        MSPProbeSim probeSim(msp);
        probeSim.run();
    
    } catch (const std::exception& e) {
        fprintf(stderr, "Error: %s\n", e.what());
        fprintf(stderr, "Exiting...\n");
        return 1;
    }
    
    return 0;
}
