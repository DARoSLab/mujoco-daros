#include <Configuration.h>
#include "PrestoeMJSimulationBridge.hpp"
#include "PrestoeSystem.hpp"

int main(int argc, char** argv) {
  bool sim = true;

  if(sim){ // Simulation
    PrestoeSystem<double> prestoe_sys(THIS_COM"/Systems/PrestoeSystem/Configs/prestoe_sim_setup.yaml");
    PrestoeMJSimulationBridge sim_bridge(&prestoe_sys, 
     THIS_COM"/Systems/PrestoeSystem/Robot/prestoe.xml");
    sim_bridge.run();

  }else{ // Robot Control
    PrestoeSystem<float> prestoe_sys("prestoe_experiment_setup.yaml");
      // HardwareBridge *hw_bridge = new PrestoeHardwareBridge();
      // hw_bridge->run();
  }

  return 0;
}
