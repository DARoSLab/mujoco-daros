#ifdef MACHINE_LEARNING_BUILD

#include "FSM_State_RL.h"
#include <Utilities/Timer.h>

template <typename T>
FSM_State_RL<T>::FSM_State_RL(ControlFSMData<T>* _controlFSMData):
FSM_State<T>(_controlFSMData, FSM_StateName::LOCOMOTION_RL, "RL"),
_data(_controlFSMData){
  vRLCon = new PatRL(_data, _data->userParameters);
  _dof_pos.resize(6);
  _dof_vel.resize(6);

}

template <typename T>
void FSM_State_RL<T>::onEnter() {
  vRLCon->initialize();
  iter = 0;
  _ready_for_policy = false;

  printf("[FSM RL] On Enter\n");
}
template <typename T>
void FSM_State_RL<T>::onExit() {
  printf("[FSM RL] On Exit\n");
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
// void FSM_State_RL<T>::run(bool b_first_visit) {
void FSM_State_RL<T>::run() {
  // if(b_first_visit) { onEnter(); }
  vRLCon->run();
  //updateLegCMD();
  iter++;
}
template <typename T>
void FSM_State_RL<T>::updateLegCMD(){
  _data->_legController->commands->zero(); // Zero the leg controller
  auto bodyPosition = _data->_stateEstimator->getResult().position;
  Vec3<float> bodyOri = ori::quatToRPY(_data->_stateEstimator->getResult().orientation);
  bodyOri(2) = 0.0;
  if (bodyPosition(2) < vRLCon->getMaxHeightError() || bodyOri.norm()>vRLCon->getMaxOriError()){
  	ESTOP = true;
  }

  for(int leg=0; leg<2; leg++){
   for(int j=0; j<3; j++){
     _dof_pos[3*leg + j] = _data->_legController->datas[RS_IDX(leg)]->q[j];
     _dof_vel[3*leg + j] = _data->_legController->datas[RS_IDX(leg)]->qd[j];
   }
  }
  _tau_ff.setZero();
  if(!ESTOP)
    _tau_ff = vRLCon->getPGain()*(vRLCon->getActionScale()*vRLCon->getPolicyCMD().cast<float>() + vRLCon->getDefaultDofPos() - _dof_pos) - vRLCon->getDGain()*_dof_vel;
  else
    printf("ESTOP!\n");

  for(int leg=0; leg<2; leg++){
    _data->_legController->commands[RS_IDX(leg)].tauFeedForward = _tau_ff.block<3, 1>(3*leg, 0);
  }
}
template <typename T>
void FSM_State_RL<T>::bringToInitPosture(){
  /*
  Order of indices in Robot Software and Isaacgym is d/nt
  Robot software FR->FL->RR->RL where as in isaac gym its FL->FR->RL->RR
  0->1 1->0 2->3 3->2  leg + (-1)^leg
 */

  _default_dof_pos = vRLCon->getDefaultDofPos();
  _data->_legController->commands->zero(); // Zero the leg controller
  for(int leg =0; leg<2; leg ++){//isaac leg
    _data->_legController->commands[RS_IDX(leg)].qDes = _default_dof_pos.block<3, 1>(3*leg, 0);
    _data->_legController->commands[RS_IDX(leg)].kpJoint= 10.0*Mat3<float>::Identity();
    _data->_legController->commands[RS_IDX(leg)].kdJoint= 1.5*Mat3<float>::Identity();
  }

}

template <typename T>
bool FSM_State_RL<T>::isInitPostureDone(T tolerance){

  _dof_pos_error = vRLCon->getDefaultDofPos() - _dof_pos;
  return (_dof_pos_error.transpose()*_dof_pos_error)<tolerance;

}
template class FSM_State_RL<float>;
#endif
