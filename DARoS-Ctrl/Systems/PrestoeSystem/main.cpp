#include <Configuration.h>
#include "PrestoeMJSimulationBridge.hpp"
#include "PrestoeSystem.hpp"

int main(int argc, char** argv) {
  bool sim = true;
  bool is_record = false;
  double duration = 10.0;

  // Select mode s: simulation, or e: experiment
  if(argc < 1){
    std::cout << "Usage: ./PrestoeSystem [s/e] [record duration]" << std::endl;
    return 0;
  }
  else{
    if(argv[1][0] == 's'){ sim = true; }
    else if(argv[1][0] == 'e'){  sim = false; }
  }


  if(sim){ // Simulation
    PrestoeSystem<double> prestoe_sys(THIS_COM"/Systems/PrestoeSystem/Configs/prestoe_system_config.yaml");
    PrestoeMJSimulationBridge sim_bridge(&prestoe_sys, THIS_COM"/Systems/PrestoeSystem/Configs/prestoe_sim_config.yaml");
    sim_bridge.run();

  }else{ // Robot Control
    PrestoeSystem<float> prestoe_sys("prestoe_experiment_setup.yaml");
      // HardwareBridge *hw_bridge = new PrestoeHardwareBridge();
      // hw_bridge->run();
  }

  return 0;
}
