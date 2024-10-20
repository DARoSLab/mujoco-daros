#ifndef PRESTOE_MJ_SIMULATION_BRIDGE_H
#define PRESTOE_MJ_SIMULATION_BRIDGE_H

#include <MujocoSimulationBridge.hpp>
#include <unistd.h>

class PrestoeMJSimulationBridge: public MujocoSimulationBridge{
  public:
    PrestoeMJSimulationBridge();
    virtual ~PrestoeMJSimulationBridge(){}
 
  protected:
    double _ctrl_time = 0.;
    double _ctrl_dt = 0.002;

    static PrestoeMJSimulationBridge* instance;

    virtual void _onestep_simulation();
    
    int _iter=0;

    int freejoint_qpos_addr;
    int freejoint_qvel_addr;
    static void control_callback(const mjModel* m,mjData* d);
};

#endif
