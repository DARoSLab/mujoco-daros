#ifndef Pat_SYSTEM
#define Pat_SYSTEM

#include <System.hpp>
#include <robots/Patroclus.h>
#include <robots/PatroclusControl.h>

#include <can_data_lcmt.hpp>
#include <can_command_lcmt.hpp>
#include <pat_leg_control_command_lcmt.hpp>
#include <pat_leg_control_data_lcmt.hpp>

#include <rt/rt_rc_interface.h>
#include <rt/rt_xsr_interface.h>

#include <state_machine/ControlFSM.h>
#include <controllers/auxillary/LegControllerPat.h>
#include <controllers/auxillary/JPosInitializerPat.h>
#include <control/ctrl_utils/DesiredStateCommand.h>
#include <PatParameters.h>
#include <SimUtilities/VisualizationData.h>
#include <SimUtilities/GamepadCommand.h>

template<typename T> class PatLCMCommunicator;
template<typename T> class PatMocapManager;

template<typename T>
class PatSystem: public System<T>{
  friend class PatLCMCommunicator<T>;
  friend class PatMocapManager<T>;

  public:
    PatSystem(const char* robot_type, bool is_sim);
    virtual ~PatSystem(){
      delete _controlFSM;
      delete _legController;
      delete _gaitScheduler;
      delete _stateEstimator;
      delete _desiredStateCommand;
      delete _visualizationData;
    }

    virtual bool initialization();
    virtual void renderSystem() {
      _communicator->VisualInfoSendingURDF(_estimated_state, true);
      // Simulation
      if(this->_is_sim){
        _communicator->VisualInfoSendingURDF(this->_state, false);
        _communicator->debugVisualInfoSending();
      }
    }
    virtual void clearVisualization(){
      _visualizationData->clear();
    }

    virtual bool Estop();
    virtual void updateFBModelStateEstimate();

    // Must be specified in
    // SimulationBridge or HardwareBridge
    can_data_lcmt* _sys_canData;
    can_command_lcmt* _sys_canCommand;
    VectorNavData* _sys_vectorNavData;
    MOCAPData<float>* _sys_mocapData;
    LocalizationData* _sys_localizationData;
    CheaterState* _sys_cheaterState;
    GamepadCommand* _sys_gamepadCommand;
    ///////////////////////////////////////

    VisualizationData* _visualizationData;
    MoCapData<float>* _mocapData;


    virtual void onestep_forward();
    virtual void onestep_forward_train(bool & b_render){}
    const PatParameters & getParameters(){ return _sys_parameters; }
    PatLCMCommunicator<T>* getCommunicator(){return _communicator;}
    int getRobotId(){return _robot_id;}
  protected:
    T _ave_time = 0.;
    int num_skip = 800;
    unsigned long iter = 0;
    bool _initialized = false;
    bool _cheaterModeEnabled = false;
    int _simulation_test = 0;

    void _initializeStateEstimator(bool cheaterMode = false);

    pat_leg_control_command_lcmt leg_control_command_lcm;
    pat_leg_control_data_lcmt leg_control_data_lcm;
    state_estimator_lcmt state_estimator_lcm;

    RobotType _robot_type;
    int _robot_id = 0;
    FBModelState<T> _estimated_state;

    rc_control_settings rc_control;
    PatBiped<float>* _pat_ctrl;
    ControlFSM<float>* _controlFSM;
    LegControllerPat<float>* _legController;
    GaitSchedulerPat<float>* _gaitScheduler;

    StateEstimatorContainer<float>* _stateEstimator;
    DesiredStateCommand<float>* _desiredStateCommand;

    PatParameters _sys_parameters;
    EstimatorParameters _sys_estimator_parameters;
    // VisualizationData* _visualizationData;
    PatLCMCommunicator<T>* _communicator;
    // PatMocapManager<T>* _mocap_manager;

    JPosInitializerPat<float>* _jpos_initializer;

};

#endif
