/*!
 * @file main.cpp
 * @brief Main Function for the System running
 *
 * The main function parses command line arguments and starts the appropriate
 * driver.
 */

#include "TelloSystem.hpp"
#include "TelloSimulationBridge.hpp"

void printUsage() {
  printf(
      "Usage: robot [sim-or-robot] [parameters-from-file]\n"
      "\t      sim-or-robot: s for sim, r for robot\n"
      "\t      param-file:   f for loading parameters from file, l (or nothing) for LCM\n"
      "                      this option can only be used in robot mode\n");
}

int main(int argc, char** argv) {
  // if (argc != 2 && argc != 3) {
  //   printUsage();
  //   return EXIT_FAILURE;
  // }

  // if(argv[1][0] == 's'){
  System<double>* Tello_sys = new TelloSystem<double>(true);
  SimulationBridge* sim_bridge = 
    new TelloSimulationBridge(Tello_sys,
        "ConfigFiles/simulator-defaults.yaml",
        "ConfigFiles/default-terrain.yaml");
  sim_bridge->run();

  delete sim_bridge;
  delete Tello_sys;

  // }else if(argv[2][0] == 'r'){
  //   // TODO
  // }else{
  //   throw std::runtime_error("select simulation (s), training (t), or robot (r)");
  // }

  return 0;
}
