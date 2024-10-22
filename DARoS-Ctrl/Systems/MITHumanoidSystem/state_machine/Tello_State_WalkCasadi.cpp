/*=========================== Balance Stand Tello ===========================*/
/**
 * FSM State that forces all legs to be on the ground and uses the QP
 * Balance controller for instantaneous balance control.
 */

#include "Tello_State_WalkCasadi.h"
#include <wbc_ctrl/TelloLocomotionCtrl/TelloLocoCtrl.hpp>

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
Tello_State_WalkCasadi<T>::Tello_State_WalkCasadi(
    ControlFSMData_Tello<T>* _controlFSMData)
    : Tello_State<T>(_controlFSMData, Tello_StateName::WALK_CASADI,"WALK_CASADI") {
  // TEST
  this->turnOffAllSafetyChecks();

  _casadiMPC = new CASADI_Walk_Tello<T>(_controlFSMData->_userParameters->controller_dt,
    50./(1000. *_controlFSMData->_userParameters->controller_dt ),
    _controlFSMData->_userParameters,
    this->_model , this->_fsm_data);

  // Initialize GRF to 0s
  //this->footFeedForwardForces = Mat34<T>::Zero();

  _wbc_ctrl = new TelloLocoCtrl<T>(this->_model, _controlFSMData);
  _wbc_data = new TelloLocoCtrlData<T>();

  // Do Body Posture control almost ignore Fr des
  _wbc_ctrl->setFloatingBaseWeight(_controlFSMData->_userParameters->FloatingBaseWeight);
  _ini_jpos = DVec<T>::Zero(this->_model->_nDof-6);
  printf("[Casadi Walk State] Constructed\n");
}

template <typename T>
void Tello_State_WalkCasadi<T>::onEnter() {
  this->UpdateModel();

  this->_casadiMPC->init_casadi();

  // Default is to not transition
  this->nextStateName = this->stateName;

  // Reset the transition data
  this->transitionData.zero();

  // Initial state
  _ini_body_pos = (this->_fsm_data->_stateEstimator->getResult()).position;
  _ini_com_pos = (this->_fsm_data->_stateEstimator->getResult()).position + this->_model->getComPos();
  _ini_body_ori_rpy = (this->_fsm_data->_stateEstimator->getResult()).rpy;
  _body_weight = this->_fsm_data->_tello_model->massMatrix()(3,3)*9.81;

  _mid_pos_cps.setZero();


  for(size_t i(0); i<tello_contact::num_foot_contact; ++i){
    _mid_pos_cps += 
      this->_model->_pGC[tello_contact::R_heel + i]/tello_contact::num_foot_contact;
      std::cout<<"contact point \n "<< this->_model->_pGC[tello_contact::R_heel + i] << std::endl;
  }
  // exit(0);
  _ini_jpos = this->get_current_jpos();
  std::cout<<"_ini_jpos \n "<< _ini_jpos << std::endl;
  pretty_print(_ini_body_ori_rpy, std::cout, "body rpy");
  pretty_print(_ini_com_pos, std::cout, "[Tello WALK_CASADI] ini com pos");
  pretty_print(_mid_pos_cps, std::cout, "[Tello WALK_CASADI middle of cps");
  std::cout << "[Tello WALK_CASADI] On Enter" << std::endl;
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void Tello_State_WalkCasadi<T>::run() {
  //Vec3<T> body_pos = (this->_fsm_data->_stateEstimator->getResult()).position;
  //Quat<T> body_ori = (this->_fsm_data->_stateEstimator->getResult()).orientation;

  //DVec<T> jpos = DVec<T>::Zero(this->_model._nDof-6);
  //jpos = this->get_current_jpos();
  //pretty_print(body_pos, std::cout, "body pos");
  //pretty_print(body_ori, std::cout, "body ori");
  this->UpdateModel();

  this->viz_heel_toe();
  KeepPostureStep();
  // for(size_t i(tello::num_leg ); i<tello::num_joint_group; ++i){
  //   this->_fsm_data->_jointController->_commands[i]->zero();
  // }
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template <typename T>
Tello_StateName Tello_State_WalkCasadi<T>::checkTransition() {
  this->nextStateName = this->stateName;
  _iter++;

  // Switch FSM control mode
  switch ((int)this->_fsm_data->_userParameters->control_mode) {
    case Tello_StateName::BALANCE_STAND:  // normal c (0)
      // Normal operation for state based transitions
      break;

    case Tello_StateName::STAND_CASADI:  // normal c (0)
      // Normal operation for state based transitions
      break;

    case Tello_StateName::WALK_CASADI:  // normal c (0)
      // Normal operation for state based transitions
      break;

    case Tello_StateName::PASSIVE:
      this->nextStateName = Tello_StateName::PASSIVE;
      break;

    default:
      std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
        << (int)Tello_StateName::BALANCE_STAND << " to "
        << this->_fsm_data->_userParameters->control_mode << std::endl;
  }

  // Return the next state name to the FSM
  return this->nextStateName;

}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template <typename T>
TransitionData_Tello<T> Tello_State_WalkCasadi<T>::transition() {
  // Finish Transition
  this->transitionData.done = true;
  // Return the transition data to the FSM
  return this->transitionData;
}

/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void Tello_State_WalkCasadi<T>::onExit() {
  _iter = 0;
}

template <typename T>
void Tello_State_WalkCasadi<T>::KeepPostureStep() {
  T curr_time = 0.002*_iter;
  _wbc_data->pBody_RPY_des.setZero();
  _wbc_data->jpos_des = _ini_jpos;


  _wbc_data->pBody_des[0] = _ini_com_pos[0];
  _wbc_data->pBody_des[1] = _ini_com_pos[1]; //0.0;
  _wbc_data->pBody_des[2] = _ini_com_pos[2];

  _wbc_data->vBody_des.setZero();
  _wbc_data->aBody_des.setZero();
  _wbc_data->vBody_Ori_des.setZero();


  _casadiMPC->run(*this->_fsm_data);
  // T _body_weight = this->_fsm_data->_tello_model->massMatrix()(3,3)*9.81;
  _wbc_data->pBody_RPY_des = _casadiMPC->pBody_RPY_des;
  _wbc_data->vBody_Ori_des = _casadiMPC->vBody_Ori_des;
  _wbc_data->vBody_des     = _casadiMPC->vBody_des;
  _wbc_data->pBody_des     = _casadiMPC->pBody_des;

  for(size_t i(0); i<tello_contact::num_foot_contact; ++i){
    _wbc_data->Fr_des[i] = _casadiMPC->Fr_des[i];
    // std::cout<<"Fr_des[i] \n "<< _wbc_data->Fr_des[i] << std::endl;
    _wbc_data->pFoot_des[i] = _casadiMPC->pFoot_des[i];
    _wbc_data->vFoot_des[i] = _casadiMPC->vFoot_des[i];
    _wbc_data->aFoot_des[i] = _casadiMPC->aFoot_des[i];
  }
  

  _wbc_data->contact_state = _casadiMPC->contact_state;

  // for(size_t i(0); i<tello_contact::num_foot_contact; ++i){
  //   _wbc_data->Fr_des[i].setZero();
  //   _wbc_data->Fr_des[i][2] = _body_weight/tello_contact::num_foot_contact;
  // }
  _wbc_ctrl->run(_wbc_data, *this->_fsm_data);
}

// template class FSM_State_BalanceStand<double>;
template class Tello_State_WalkCasadi<float>;
