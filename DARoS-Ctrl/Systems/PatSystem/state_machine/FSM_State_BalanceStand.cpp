/*=========================== Balance Stand ===========================*/
/**
 * FSM State that forces all legs to be on the ground and uses the QP
 * Balance controller for instantaneous balance control.
 */

#include "FSM_State_BalanceStand.h"
#include <wbc_ctrl/LocomotionCtrl/LocomotionCtrl.hpp>

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_BalanceStand<T>::FSM_State_BalanceStand(
    ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::BALANCE_STAND,"BALANCE_STAND") {
  // Set the pre controls safety checks
  this->turnOnAllSafetyChecks();
  // Turn off Foot pos command since it is set in WBC as operational task
  this->checkPDesFoot = false;

  // Initialize GRF to 0s
  this->footFeedForwardForces = Mat32<T>::Zero();
  _wbc_ctrl = new LocomotionCtrl<T>(_controlFSMData);
  _wbc_data = new LocomotionCtrlData<T>();

  _wbc_ctrl->setFloatingBaseWeight(1000.);
}

template <typename T>
void FSM_State_BalanceStand<T>::onEnter() {
  // Always set the gait to be standing in this state
  std::cout << "[Balance Stand] OnEnter" << '\n';
  this->_data->_gaitScheduler->gaitData._nextGait = GaitType::STAND;

  _ini_body_pos = (this->_data->_stateEstimator->getResult()).position;

  if(_ini_body_pos[2] < 0.2) {
    _ini_body_pos[2] = 0.3;
  }

  last_height_command = _ini_body_pos[2];

  _ini_body_ori_rpy = (this->_data->_stateEstimator->getResult()).rpy;
  _body_weight = this->_data->_pat->_bodyMass * 9.81;

  _wbc_data->vBody_Ori_des.setZero();

  for(size_t i(0); i<pat_biped::num_legs; ++i){
    _wbc_data->pFoot_des[i].setZero();
    _wbc_data->vFoot_des[i].setZero();
    _wbc_data->aFoot_des[i].setZero();
    _wbc_data->Fr_des[i][2] = _body_weight/pat_biped::num_legs;
    _wbc_data->contact_state[i] = true;
  }
  _wbc_data->vBody_des.setZero();
  _wbc_data->aBody_des.setZero();

  Vec4<T> contactState;
  contactState<< 0.5, 0.5, 0.5, 0.5;
  this->_data->_stateEstimator->setContactPhase(contactState);
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_BalanceStand<T>::run() {
  BalanceStandStep();
}

/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_BalanceStand<T>::onExit() {
  _iter = 0;
}

/**
 * Calculate the commands for the leg controllers for each of the feet.
 */
template <typename T>
void FSM_State_BalanceStand<T>::BalanceStandStep() {

  _ini_body_pos[2] = 0.38 + 0.04*sin(2*M_PI*0.002*_iter);
  // _ini_body_ori_rpy[0] = 0.3*sin(2*M_PI*0.001*_iter);
  // _ini_body_ori_rpy[1] = 0.5*sin(2*M_PI*0.001*_iter++);
  _wbc_data->pBody_des = _ini_body_pos;
  _wbc_data->pBody_RPY_des = _ini_body_ori_rpy;

  if(this->_data->userParameters->use_rc){
    const rc_control_settings* rc_cmd = this->_data->_desiredStateCommand->rcCommand;
    // Orientation
    _wbc_data->pBody_RPY_des[0] = rc_cmd->rpy_des[0]*1.4;
    _wbc_data->pBody_RPY_des[1] = rc_cmd->rpy_des[1]*0.46;
    _wbc_data->pBody_RPY_des[2] -= rc_cmd->rpy_des[2];

    // Height
    _wbc_data->pBody_des[2] += 0.12 * rc_cmd->height_variation;
  }else{
    // Orientation
    // _wbc_data->pBody_RPY_des[0] =
    //  0.6* this->_data->_desiredStateCommand->gamepadCommand->leftStickAnalog[0];
    //  _wbc_data->pBody_RPY_des[1] =
    //   0.6*this->_data->_desiredStateCommand->gamepadCommand->rightStickAnalog[0];
    // _wbc_data->pBody_RPY_des[2] -=
    //   this->_data->_desiredStateCommand->gamepadCommand->rightStickAnalog[1];
    //
    // // Height
    // _wbc_data->pBody_des[2] +=
    //   0.12 * this->_data->_desiredStateCommand->gamepadCommand->rightStickAnalog[0];
  }

  if(this->_data->_desiredStateCommand->trigger_pressed) {
    _wbc_data->pBody_des[2] = 0.05;

    if(last_height_command - _wbc_data->pBody_des[2] > 0.001) {
      _wbc_data->pBody_des[2] = last_height_command - 0.001;
    }
  }
  last_height_command = _wbc_data->pBody_des[2];

  _wbc_ctrl->run(_wbc_data, *this->_data);
  _iter++;
}

// template class FSM_State_BalanceStand<double>;
template class FSM_State_BalanceStand<float>;
