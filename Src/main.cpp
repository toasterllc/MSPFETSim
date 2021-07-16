#include "MSPProbeSim.h"

int main(int argc, const char* argv[]) {
    MSPProbeSim probeSim;
    try {
        probeSim.run();
    
    } catch (const std::exception& e) {
        fprintf(stderr, "Error: %s\n", e.what());
        fprintf(stderr, "Exiting...\n");
        return 1;
    }
    
    return 0;
}
