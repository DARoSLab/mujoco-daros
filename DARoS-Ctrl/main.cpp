#include <Configuration.h>
#include <MujocoSimulationBridge.hpp>

// #include <stdexcept>

int main(int argc, char** argv) {
    // MujocoSimulationBridge* sim_bridge = new PrestoeMJSimulationBridge();
    MujocoSimulationBridge* sim_bridge = new StaccatoeMJSimulationBridge();
    sim_bridge->setCtrl();
    sim_bridge->run();
    
    return 0;
}
