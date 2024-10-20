#include <Configuration.h>
#include "PrestoeMJSimulationBridge.hpp"

int main(int argc, char** argv) {
    bool sim = true;

    if(sim){ // Simulation
        MujocoSimulationBridge *sim_bridge = new PrestoeMJSimulationBridge();
        sim_bridge->run();
    }else{ // Robot Control
        // HardwareBridge *hw_bridge = new PrestoeHardwareBridge();
        // hw_bridge->run();
    }
    return 0;
}
