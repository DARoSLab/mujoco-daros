/*!
 * @file main.cpp
 * @brief Main Function for the WBC Controller
 *
 * The main function parses command line arguments and starts the appropriate
 * driver.
 */

#include "PatSystem.hpp"
#include "PatSimulationBridge.hpp"
#include "PatHardwareBridge.hpp"
#include "PatHardwareTestBridge.hpp"
#include <HardwareBridge.hpp>

void printUsage() {
  printf(
      "Usage: robot [robot-id] [sim-or-robot] [parameters-from-file]\n"
      "\twhere robot-id:     [0- ] \n"
      "\t      sim-or-robot: s for sim, r for robot\n"
      "\t      param-file:   f for loading parameters from file, l (or nothing) for LCM\n"
      "                      this option can only be used in robot mode\n");
}

int main(int argc, char** argv) {
  if (argc != 3 && argc != 4) {
    printUsage();
    return EXIT_FAILURE;
  }

  if(argv[2][0] == 's'){
    System<double>* Pat_sys = new PatSystem<double>(argv[1], true);
    SimulationBridge* sim_bridge =
      new PatSimulationBridge(Pat_sys,
          "config/simulator-defaults.yaml",
          "config/default-terrain.yaml");
           // "config/terrains/stairs-terrain.yaml");
    if(argc == 4){

      if(argv[3][0] == 'd') // disable visualization
        sim_bridge->disable_visualization();
    }
    sim_bridge->run();

    delete sim_bridge;
    delete Pat_sys;

  }else if(argv[2][0] == 'r'){
#ifdef __linux
    System<float> * Pat_sys = new PatSystem<float>(argv[1], false);
    HardwareBridge* hardware_bridge = new PatHardwareBridge(Pat_sys);
    hardware_bridge->run();

    delete hardware_bridge;
#endif
}else if(argv[2][0] == 't'){
#ifdef __linux
    System<float> * Pat_sys = new PatSystem<float>(argv[1], false);
    HardwareBridge* hardware_bridge = new PatHardwareTestBridge(Pat_sys);
    hardware_bridge->run();

    delete hardware_bridge;
#endif
  }else{
    throw std::runtime_error("select simulation (s) or robot (r)");
  }
  return 0;
}
