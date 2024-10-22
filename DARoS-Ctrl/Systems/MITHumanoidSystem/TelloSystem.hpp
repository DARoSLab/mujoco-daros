#ifndef Tello_SYSTEM
#define Tello_SYSTEM

#include <System.hpp>
#include <tello_joint_command_lcmt.hpp>
#include <tello_joint_data_lcmt.hpp>
#include <state_estimator_lcmt.hpp>

#include <TelloParameters.h>
#include <controllers/auxillary/TelloJointController.h>
#include <controllers/auxillary/UserInputHandler.h>
#include <tello/state_machine/ControlFSM_Tello.h>
#include <observers/StateEstimatorContainer.h>
#include <observers/EstimatorParameters.h>
#include <SimUtilities/VisualizationData.h>
#include <Configuration.h>
#include <robots/Tello.h>
#include <cppTypes.h>
#include <SimUtilities/VisualizationData.h>

template<typename T> class TelloLCMCommunicator;

template<typename T>
class TelloSystem: public System<T>{
	friend class TelloLCMCommunicator<T>;

  public:
    TelloSystem(bool is_sim);
    virtual ~TelloSystem(){
      delete _stateEstimator;
      delete _visualizationData;
    }

    virtual bool initialization();
    virtual void renderSystem() {
      _communicator->VisualInfoSending(_estimated_state, true);
      if(this->_is_sim){
        _communicator->VisualInfoSending(this->_state, false);
        _communicator->debugVisualInfoSending();
      }
    }
    virtual void clearVisualization(){  _visualizationData->clear(); }
    virtual void updateFBModelStateEstimate();

    // // Must be specified in
    // // SimulationBridge or HardwareBridge
    tello_joint_data_lcmt* _sys_data;
    tello_joint_command_lcmt* _sys_cmd;
    IMU_Data* _sys_imuData;
    CheaterState* _sys_cheaterState;
    // ///////////////////////////////////////

    state_estimator_lcmt state_estimator_lcm;
    VisualizationData* _visualizationData;

    virtual void onestep_forward();
    const TelloParameters & getParameters(){ return _sys_parameters; }
    T vel = 0.0;
    ControlFSM_Tello<float>* _controlFSM;


  protected:
    bool _initialized = false;
    void _initializeStateEstimator(bool cheaterMode = false);

    FBModelState<T> _estimated_state;
    Tello<float> _robot_ctrl;

    TelloJointController<float> * _jointController;

    
    StateEstimatorContainer<float>* _stateEstimator;
    UserInputHandler<float>* _userInputHandler;

    TelloParameters _sys_parameters;
    EstimatorParameters _sys_estimator_parameters;
    TelloLCMCommunicator<T>* _communicator;
};

#endif
