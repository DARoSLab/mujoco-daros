#ifndef PRESTOE_MJ_SIMULATION_BRIDGE_H
#define PRESTOE_MJ_SIMULATION_BRIDGE_H

#include <MujocoSimulationBridge.hpp>
#include <unistd.h>
#include <PrestoeBipedSystem.hpp>

class PrestoeBipedMJSimulationBridge: public MujocoSimulationBridge{
  public:
    PrestoeBipedMJSimulationBridge(System<double> * system, const std::string & config_file);
    virtual ~PrestoeBipedMJSimulationBridge(){}
 
  protected:
    PrestoeBipedSystem<double> * _prestoe_sys;
    virtual void _UpdateSystemObserver();
    virtual void _UpdateControlCommand();
    virtual void _UpdateSystemVisualInfo();

};

#endif
