#ifndef Pat_SIMULATION_BRIDGE
#define Pat_SIMULATION_BRIDGE

#include <SimulationBridge.hpp>
#include "PatSystem.hpp"
#include <Utilities/PeriodicTask.h>

#include <SimUtilities/ImuSimulator.h>
#include <SimUtilities/realSenseSimulator.h>


class PatSimulationBridge: public SimulationBridge{
  public:
    PatSimulationBridge(System<double>* sys,
        const std::string & sim_params_file, const std::string & terrain_file);
    virtual ~PatSimulationBridge(){

      delete _imuSimulator;
      delete _rsSimulator;
    }
    PatSystem<double>* _pat_sys;
    can_data_lcmt _canData;
    can_command_lcmt _spiCommand;
    VectorNavData _vectorNavData;
    CheaterState _cheaterState;
    LocalizationData _localizationData;
    GamepadCommand _gamepadCommand;

  protected:
    double _sim_time = 0.;
    double _ctrl_time = 0.;
    double _sim_dt = 0.;
    double _ctrl_dt = 0.;

    std::vector<double> _ext_force_time;
    std::vector<double> _ext_force_duration;
    std::vector<std::vector<double> > _ext_force_list;
    std::vector<double> _enable_ext_force;
    std::vector<double> _ext_force_kp, _ext_force_kd;
    std::vector<double> _des_body_pose;

    std::vector<double> _ext_force;
    double _ext_force_init_time;
    double _ext_force_duration_time;

    size_t _num_impulse = 0;
    size_t _force_idx = 0;
    bool _b_no_more_force = true;
    bool _enable_support = false;
    double _enable_fixture_support;
    double _enable_push_test;
    bool _enable_disable_counter = false;
    int _wait_disable_counter = 0;
    int _max_disable_count = 0;

    ImuSimulator<double>* _imuSimulator = nullptr;
    realSenseSimulator<double>* _rsSimulator = nullptr;

    virtual void _onestep_simulation();
    virtual void _onestep_system();


    PeriodicTaskManager taskManager;
    void runXSR();

    void _ParamExtractor(const std::string & file_name);
};

#endif
