#ifndef TELLO_SIMULATION_BRIDGE
#define TELLO_SIMULATION_BRIDGE

#include <SimulationBridge.hpp>
#include "TelloSystem.hpp"
#include <SimUtilities/IMU_Simulator.h>


class TelloSimulationBridge: public SimulationBridge{
  public:
    TelloSimulationBridge(System<double>* sys,
        const std::string sim_params_file, const std::string & terrain_file);
    virtual ~TelloSimulationBridge(){
      delete _imuSimulator;
    }
    TelloSystem<double>* _tello_sys;
    tello_joint_data_lcmt _j_data;
    tello_joint_command_lcmt _j_cmd;
    IMU_Data _imuData;
    CheaterState _cheaterState;
    //GamepadCommand _gamepadCommand;

  protected:
  	double _sim_time = 0.;
    double _ctrl_time = 0.;
    double _sim_dt = 0.;
    double _ctrl_dt = 0.;

    IMU_Simulator<double>* _imuSimulator = nullptr;

    virtual void _onestep_simulation();
    virtual void _onestep_system();
};

#endif
