#include <Configuration.h>
#include "MiniArmMJSimulationBridge.hpp"
#include "MiniArmSystem.hpp"

int main(int argc, char** argv) {

  MiniArmSystem<double> miniarm_sys(THIS_COM"/Systems/MiniArmSystem/Configs/miniarm_sim_setup.yaml");
  MiniArmMJSimulationBridge sim_bridge(&miniarm_sys);
  sim_bridge.run();

  return 0;
}
