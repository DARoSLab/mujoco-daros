#ifndef PRESTOE_MJ_SIMULATION_BRIDGE_H
#define PRESTOE_MJ_SIMULATION_BRIDGE_H

#include <MujocoSimulationBridge.hpp>
#include <unistd.h>

class PrestoeMJSimulationBridge: public MujocoSimulationBridge{
  public:
    PrestoeMJSimulationBridge(System<double> * system, const std::string & mj_xml_file);
    virtual ~PrestoeMJSimulationBridge(){}
 
  protected:
    virtual void _UpdateSystemObserver();
    virtual void _UpdateControlCommand();
    virtual void _UpdateSystemVisualInfo();

};

#endif
