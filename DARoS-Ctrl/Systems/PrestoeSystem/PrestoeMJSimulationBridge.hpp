#ifndef PRESTOE_MJ_SIMULATION_BRIDGE_H
#define PRESTOE_MJ_SIMULATION_BRIDGE_H

#include <MujocoSimulationBridge.hpp>
#include <unistd.h>
#include <PrestoeSystem.hpp>

class PrestoeMJSimulationBridge: public MujocoSimulationBridge{
  public:
    PrestoeMJSimulationBridge(System<double> * system);
    virtual ~PrestoeMJSimulationBridge(){}
 
  protected:
    PrestoeSystem<double> * _prestoe_sys;
    virtual void _UpdateSystemObserver();
    virtual void _UpdateControlCommand();
    virtual void _UpdateSystemVisualInfo();

};

#endif
