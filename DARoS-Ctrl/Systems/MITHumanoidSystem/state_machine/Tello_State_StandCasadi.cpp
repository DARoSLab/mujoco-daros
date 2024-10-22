/*=========================== Balance Stand Tello ===========================*/
/**
 * FSM State that forces all legs to be on the ground and uses the QP
 * Balance controller for instantaneous balance control.
 */

#include "Tello_State_StandCasadi.h"
#include <wbc_ctrl/TelloStandCtrl/TelloStandCtrl.hpp>

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
Tello_State_StandCasadi<T>::Tello_State_StandCasadi(
    ControlFSMData_Tello<T>* _controlFSMData)
    : Tello_State<T>(_controlFSMData, Tello_StateName::STAND_CASADI,"STAND_CASADI") {
  // TEST
  this->turnOffAllSafetyChecks();

  _casadiMPC = new CASADI_Stand_Tello(_controlFSMData->_userParameters->controller_dt,
    50./(1000. *_controlFSMData->_userParameters->controller_dt ),
    _controlFSMData->_userParameters,
    this->_model);

  // Initialize GRF to 0s
  //this->footFeedForwardForces = Mat34<T>::Zero();

  _wbc_ctrl = new TelloStandCtrl<T>(this->_model);
  _wbc_data = new TelloStandCtrlData<T>();

  // Do Body Posture control almost ignore Fr des
  _wbc_ctrl->setFloatingBaseWeight(0.1);
  _ini_jpos = DVec<T>::Zero(this->_model->_nDof-6);
  printf("[Casadi Stand State] Constructed\n");
}

template <typename T>
void Tello_State_StandCasadi<T>::onEnter() {
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

  pretty_print(_ini_body_ori_rpy, std::cout, "body rpy");
  pretty_print(_ini_com_pos, std::cout, "[Tello STAND_CASADI] ini com pos");
  pretty_print(_mid_pos_cps, std::cout, "[Tello STAND_CASADI middle of cps");
  std::cout << "[Tello STAND_CASADI] On Enter" << std::endl;
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void Tello_State_StandCasadi<T>::run() {
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
Tello_StateName Tello_State_StandCasadi<T>::checkTransition() {
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
TransitionData_Tello<T> Tello_State_StandCasadi<T>::transition() {
  // Finish Transition
  this->transitionData.done = true;
  // Return the transition data to the FSM
  return this->transitionData;
}

/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void Tello_State_StandCasadi<T>::onExit() {
  _iter = 0;
}

template <typename T>
void Tello_State_StandCasadi<T>::KeepPostureStep() {
  T curr_time = 0.002*_iter;
  T amp(0.2);
  T freq(0.2);
  _wbc_data->pBody_RPY_des.setZero();
  // _wbc_data->pBody_RPY_des[1] = 0.25*sin(2.*M_PI*curr_time);
  _wbc_data->pBody_RPY_des[2] = _ini_body_ori_rpy[2];
  _wbc_data->pBody_RPY_des[1] = _ini_body_ori_rpy[1] - 0.2*(cos(curr_time*2*M_PI*freq)-1) - 0.05;
  _wbc_data->jpos_des = _ini_jpos;
  _wbc_data->jpos_des[tello::shoulder_roll_R] += (-0.8*(cos(curr_time*2*M_PI*freq)-1));
  _wbc_data->jpos_des[tello::shoulder_roll_L] += (0.8*(cos(curr_time*2*M_PI*freq)-1));

  _wbc_data->jpos_des[tello::shoulder_pitch_R] += (0.8*(cos(curr_time*2*M_PI*freq)-1));
  _wbc_data->jpos_des[tello::shoulder_pitch_L] += (0.8*(cos(curr_time*2*M_PI*freq)-1));

  _wbc_data->jpos_des[tello::elbow_R] += (1.2*(cos(curr_time*2*M_PI*freq)-1));
  _wbc_data->jpos_des[tello::elbow_L] += (1.2*(cos(curr_time*2*M_PI*freq)-1));

  _wbc_data->pBody_des[0] = _ini_com_pos[0];
  _wbc_data->pBody_des[1] = _ini_com_pos[1]; //0.0;
  _wbc_data->pBody_des[2] = _ini_com_pos[2] + amp*(cos(curr_time*2*M_PI*freq) - 1); // USE THIS NORMALLY

  _wbc_data->vBody_des.setZero();
  _wbc_data->aBody_des.setZero();
  _wbc_data->vBody_Ori_des.setZero();


  _casadiMPC->run(*this->_fsm_data);
  // T _body_weight = this->_fsm_data->_tello_model->massMatrix()(3,3)*9.81;


  for(size_t i(0); i<tello_contact::num_foot_contact; ++i){
    _wbc_data->Fr_des[i] = _casadiMPC->Fr_des[i];
  }

  // for(size_t i(0); i<tello_contact::num_foot_contact; ++i){
  //   _wbc_data->Fr_des[i].setZero();
  //   _wbc_data->Fr_des[i][2] = _body_weight/tello_contact::num_foot_contact;
  // }
  _wbc_ctrl->run(_wbc_data, *this->_fsm_data);
}

// template class FSM_State_BalanceStand<double>;
template class Tello_State_StandCasadi<float>;
