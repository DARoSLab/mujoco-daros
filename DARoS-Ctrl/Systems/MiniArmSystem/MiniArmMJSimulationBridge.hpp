#ifndef MINIARM_MJ_SIMULATION_BRIDGE_H
#define MINIARM_MJ_SIMULATION_BRIDGE_H

#include <MujocoSimulationBridge.hpp>
#include <unistd.h>
#include <MiniArmSystem.hpp>

class MiniArmMJSimulationBridge: public MujocoSimulationBridge{
  public:
    MiniArmMJSimulationBridge(System<double> * system);
    virtual ~MiniArmMJSimulationBridge(){}
 
  protected:
    MiniArmSystem<double> * _miniarm_sys;
    virtual void _UpdateSystemObserver();
    virtual void _UpdateControlCommand();
    virtual void _UpdateSystemVisualInfo();

};

#endif
