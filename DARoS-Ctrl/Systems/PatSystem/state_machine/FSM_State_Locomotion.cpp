/*============================ Locomotion =============================*/
/**
 * FSM State for robot locomotion. Manages the contact specific logic
 * and handles calling the interfaces to the controllers. This state
 * should be independent of controller, gait, and desired trajectory.
 */

#include "FSM_State_Locomotion.h"
#include <Utilities/Timer.h>
#include <wbc_ctrl/LocomotionCtrl/LocomotionCtrl.hpp>
//#include <rt/rt_interface_lcm.h>

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
  template <typename T>
FSM_State_Locomotion<T>::FSM_State_Locomotion(ControlFSMData<T>* _controlFSMData)
  : FSM_State<T>(_controlFSMData, FSM_StateName::LOCOMOTION, "LOCOMOTION")
{
  _wbc_ctrl = new LocomotionCtrl<T>(_controlFSMData);
  _wbc_data = new LocomotionCtrlData<T>();
  _wbc_ctrl->setFloatingBaseWeight(1000.);
  // _model = this->_data->_pat->buildModel();

  _model = _wbc_ctrl->getModelPtr();
  #ifdef USE_CMPC
    cMPC_biped = new cMPC_BipedLocomotion(_controlFSMData->userParameters->controller_dt,
      (int)33 / (1000. * _controlFSMData->userParameters->controller_dt),
      _controlFSMData->userParameters, _model);
      std::cout << "[LOCOMOTION] Using MPC" << '\n';
  #else
    std::cout << "[LOCOMOTION] Using WBC" << '\n';
    cMPC_biped = new WBC_BipedLocomotion(_controlFSMData->userParameters, _model);
  #endif
  this->turnOnAllSafetyChecks();
  // Turn off Foot pos command since it is set in WBC as operational task
  this->checkPDesFoot = false;

  // Initialize GRF and footstep locations to 0s
  this->footFeedForwardForces = Mat34<T>::Zero();
  this->footstepLocations = Mat34<T>::Zero();

}

template <typename T>
void FSM_State_Locomotion<T>::onEnter() {
  cMPC_biped->initialize();
  iter = 0;
  printf("[FSM LOCOMOTION] On Enter\n");
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_Locomotion<T>::run() {
  LocomotionControlStep();
  ++iter;
}


/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_Locomotion<T>::onExit() {
  // Nothing to clean up when exiting
  iter = 0;
}

/**
 * Calculate the commands for the leg controllers for each of the feet by
 * calling the appropriate balance controller and parsing the results for
 * each stance or swing leg.
 */
template <typename T>
void FSM_State_Locomotion<T>::LocomotionControlStep() {

  cMPC_biped->run(*this->_data);

  _wbc_data->pBody_des = cMPC_biped->pBody_des.template cast<T>();
  _wbc_data->vBody_des = cMPC_biped->vBody_des.template cast<T>();
  _wbc_data->aBody_des = cMPC_biped->aBody_des.template cast<T>();

  _wbc_data->pBody_RPY_des = cMPC_biped->pBody_RPY_des.template cast<T>();
  _wbc_data->vBody_Ori_des = cMPC_biped->vBody_Ori_des.template cast<T>();
  _wbc_data->pBody_des[0] += _wbc_data->vBody_des[0]*(this->_data->userParameters->controller_dt);
  _wbc_data->pBody_des[1] += _wbc_data->vBody_des[1]*(this->_data->userParameters->controller_dt);

  for(size_t i(0); i<pat_biped::num_legs; ++i){
  _wbc_data->pFoot_des[i] = cMPC_biped->pFoot_des[i];
  _wbc_data->vFoot_des[i] = cMPC_biped->vFoot_des[i];
  _wbc_data->aFoot_des[i] = cMPC_biped->aFoot_des[i];
  _wbc_data->Fr_des[i] << 0.0, 0.0, 0.0; //cMPC_biped->Fr_des[i];
  }

  _wbc_data->pBody_des = cMPC_biped->pBody_des.template cast<T>();
  _wbc_data->contact_state = cMPC_biped->contact_state;

  _wbc_ctrl->run(_wbc_data, *this->_data);

  Vec4<T> se_contactState; se_contactState.setZero();
  se_contactState << cMPC_biped->contact_state, 0, 0;
  this->_data->_stateEstimator->setContactPhase(se_contactState);
}

// template class FSM_State_Locomotion<double>;
template class FSM_State_Locomotion<float>;
