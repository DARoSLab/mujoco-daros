/*=========================== Balance Stand ===========================*/
/**
 * FSM State that forces all legs to be on the ground and uses the QP
 * Balance controller for instantaneous balance control.
 */

#include "FSM_State_Stepping.h"
#include <wbc_ctrl/LocomotionCtrl/LocomotionCtrl.hpp>
#include <convexMPC_Biped/LiftSwingTrajectory.hpp>

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_Stepping<T>::FSM_State_Stepping(
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
void FSM_State_Stepping<T>::onEnter() {
  // Always set the gait to be standing in this state
  std::cout << "[STEPPING] OnEnter" << '\n';
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
    _wbc_data->contact_state[i] = false;
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
void FSM_State_Stepping<T>::run() {
  BalanceStandStep();
}

/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_Stepping<T>::onExit() {
  _iter = 0;
  _phase[0] = 0.0;
  _phase[1] = M_PI;
}

/**
 * Calculate the commands for the leg controllers for each of the feet.
 */
template <typename T>
void FSM_State_Stepping<T>::BalanceStandStep() {
  UpdateGaitInfo();
  auto seResult = this->_data->_stateEstimator->getResult();
  _ini_body_pos = seResult.position;
  // T z_ref = SwingTrajectory(_phase[0], 0.2);
  // std::cout<<"Z_ref: "<< z_ref << " \n";
  _ini_body_pos[2] = 0.42; // + 0.04*sin(2*M_PI*0.002*_iter);
  // _ini_body_ori_rpy[0] = 0.5*sin(2*M_PI*0.001*_iter);
  // _ini_body_ori_rpy[1] = 0.5*sin(2*M_PI*0.001*_iter);
  _wbc_data->pBody_des = _ini_body_pos;
  _wbc_data->pBody_RPY_des = _ini_body_ori_rpy;
  _body_weight = this->_data->_pat->_bodyMass * 9.81;
  for(size_t leg(0); leg<pat_biped::num_legs; ++leg){
    if(_phase[leg]<M_PI){
      auto foot_pos = seResult.position + seResult.rBody.transpose()*(this->_data->_pat->getHipLocation(leg)+
      this->_data->_legController->datas[leg]->p);
      _wbc_data->pFoot_des[leg][0] = foot_pos[0];
      _wbc_data->pFoot_des[leg][1] = foot_pos[1];
      _wbc_data->pFoot_des[leg][2] = SwingTrajectory(_phase[leg], 0.05);
      auto* debugPath = this->_data->visualizationData->addPath();
      if(debugPath) {
          debugPath->num_points = 100;
          debugPath->color = {1.0, 0.2, 0.2, 0.5};
          float step = (M_PI - _phase[leg]) / 100.f;
          for(int j = 0; j < 100; j++) {
            debugPath->position[j] << _wbc_data->pFoot_des[leg][0],
                                      _wbc_data->pFoot_des[leg][1],
                                      SwingTrajectory(_phase[leg] + j * step, 0.05);
          }
       }
    }else{
      _wbc_data->pFoot_des[leg][0] = 0.0;
      _wbc_data->pFoot_des[leg][1] = _ini_body_pos[1] + 0.1*pow(-1, leg+1);
      _wbc_data->pFoot_des[leg][2] = 0.0;
      _wbc_data->Fr_des[leg][2] = _body_weight;


    }
  }

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
template <typename T>
T FSM_State_Stepping<T>::SwingTrajectory(T phase, T swing_height){
  assert(phase>=0.0 && phase<=2*M_PI);
  /*
  Cubic Hermite Swing Trajectory
  */
  T z_ref = 0.0;
  T t;
  if(phase<M_PI/2){ //Swing up
    t  = (2.0/M_PI)*phase;
    z_ref = swing_height*(-2*pow(t, 3) + 3*pow(t, 2));
  }
  else if(phase<M_PI) //Swing Down
  {
    t  = (2.0/M_PI)*phase - 1;
    z_ref = swing_height*(2*pow(t, 3) - 3*pow(t, 2) + 1);
  }
  else{ //Stance
  }
  return z_ref;
}

template <typename T>
void FSM_State_Stepping<T>::UpdateGaitInfo(){
  _phase[0] = 2*M_PI*(fmod(_iter*0.002, _gait_period)/_gait_period);//left leg
  _phase[0] = fmod(_phase[0], 2*M_PI);
  _phase[1] = fmod(_phase[0]+ M_PI, 2*M_PI);//right leg
  // _phase[1] = M_PI+0.01;
  Vec4<T> conphase;conphase.setZero();
  Vec4<T> contactState;contactState.setZero();

  for(size_t foot(0); foot < pat_biped::num_legs; foot++){
    conphase[foot] = _phase[foot]>M_PI? (_phase[foot]-M_PI)/ M_PI : 0.0;
    contactState[foot] = conphase[foot] > 0.0 ? 0.5 : 0.0;
  }


  this->_data->_stateEstimator->setContactPhase(conphase);

  _wbc_data->contact_state[0] = contactState[0];
  _wbc_data->contact_state[1] = contactState[1];
}
// template class FSM_State_Stepping<double>;
template class FSM_State_Stepping<float>;
