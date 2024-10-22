#include "TelloSystem.hpp"
#include <TelloLCMCommunicator.hpp>
#include <robots/TelloRobotParams.h>
#include <Utilities/pretty_print.h>
#include <observers/CheaterPositionVelocityEstimator.h>
#include <observers/OrientationEstimator.h>

template <typename T>
TelloSystem<T>::TelloSystem(bool is_sim):System<T>(is_sim)
{
  this->_robot = new Tello<T>();
  Tello<T> * tello = dynamic_cast<Tello<T>* > (this->_robot);
  setTelloRobotParams(tello);
  setTelloRobotParams(&_robot_ctrl);

  this->_robot->getInitialState(this->_state);
  this->_robot->getInitialState(_estimated_state);

  _sys_parameters.initializeFromYamlFile(THIS_COM"Systems/MIT_Humanoid/tello_config/tello-parameters.yaml");
  _jointController = new TelloJointController<float>(_robot_ctrl);
  this->_update_dt = _sys_parameters.controller_dt;

  _visualizationData = new VisualizationData();
  //_sys_gamepadCommand = new GamepadCommand();

  if(this->_is_sim){
    _communicator = new TelloLCMCommunicator<T>(this, 0);
  }else{
    _communicator = new TelloLCMCommunicator<T>(this, 255);
  }
  printf("[Tello System] Constructed\n");
  vel = _sys_parameters.CAS_v_des[0];
}

template <typename T>
bool TelloSystem<T>::initialization(){
// Must be initialized (construct estimator, command, control fsm) after
// _sys_hData, _sys_hCommand,  _sys_vectorNavData,  _sys_localizationData,
// _sys_cheaterState, _sys_gamepadCommand
// are allocated
  if(_initialized){
    printf("[Tello System - Warning] Multiple Initialization Attempts\n");
    return _initialized;
  }else{
    _stateEstimator = new StateEstimatorContainer<float>(
        _sys_cheaterState, _sys_imuData, 
        _jointController->_datas,
        &_sys_estimator_parameters,
        &_robot_ctrl);

    _userInputHandler = new UserInputHandler<float>(); // &user

    _controlFSM = new ControlFSM_Tello<float>(&_robot_ctrl, _jointController, 
        _stateEstimator, 
        _userInputHandler, &_sys_parameters, _visualizationData);

    _initializeStateEstimator(true); // for tello, always use cheater mode
    _jointController->zeroCommand();
    _jointController->setEnabled(true);

     //get_rc_control_settings(&rc_control);

    _initialized = true;
    return _initialized;
  }
}

template <typename T>
void TelloSystem<T>::onestep_forward(){
  //get_rc_control_settings(&rc_control);

  // Update Jpos, Jvel
  _jointController->updateData(_sys_data);
  // Run State Estmiator
  _stateEstimator->run();

  //_userInputHandler->convertToStateCommands();
  _controlFSM->runFSM();

  _jointController->updateCommand(_sys_cmd);
  _jointController->setLcm(_sys_data, _sys_cmd);
  _stateEstimator->setLcm(&state_estimator_lcm);
  _communicator->finalizeStep(_sys_data, _sys_cmd, &state_estimator_lcm);
  _sys_parameters.CAS_v_des[0] = vel;
  

}


template<typename T>
void TelloSystem<T>::_initializeStateEstimator(bool cheaterMode) {
  _stateEstimator->removeAllEstimators();

  _stateEstimator->addEstimator<CheaterOrientationEstimator<float>>();
  _stateEstimator->addEstimator<CheaterPositionVelocityEstimator<float>>();
}

template<typename T>
void TelloSystem<T>::updateFBModelStateEstimate(){
  _estimated_state.bodyOrientation = _stateEstimator->getResult().orientation.template cast<T>();
  _estimated_state.bodyPosition = _stateEstimator->getResult().position.template cast<T>();

  for(size_t leg(0); leg<tello::num_leg; ++leg){
    for(size_t jidx(0); jidx<tello::num_leg_joint; ++jidx){
      _estimated_state.q[tello::num_leg_joint*leg + jidx] = _jointController->_datas[leg]->q[jidx];
    }
  }
  size_t start_idx = tello::num_leg * tello::num_leg_joint;
  _estimated_state.q[start_idx] = _jointController->_datas[tello::num_leg]->q[0];

  for(size_t arm(0); arm<tello::num_arm; ++arm){
    for(size_t jidx(0); jidx<tello::num_arm_joint; ++jidx){
      _estimated_state.q[start_idx + tello::num_arm_joint*arm + jidx] = 
        _jointController->_datas[tello::num_leg+arm]->q[jidx];
    }
  }
}




template class TelloSystem<double>;
template class TelloSystem<float>;
