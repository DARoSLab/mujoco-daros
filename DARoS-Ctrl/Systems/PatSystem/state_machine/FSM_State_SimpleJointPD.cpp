/*============================= Joint PD ==============================*/
/**
 * FSM State that allows PD control of the joints.
 */

#include "FSM_State_SimpleJointPD.h"
#include <Configuration.h>

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_SimpleJointPD<T>::FSM_State_SimpleJointPD(ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::JOINT_PD, "JOINT_PD"),
_ini_jpos(pat_biped::num_act_joints){
  // Do nothing here yet
}

template <typename T>
void FSM_State_SimpleJointPD<T>::onEnter() {
  // Reset counter
  _iter = 0;

  for(size_t leg(0); leg<pat_biped::num_legs; ++leg){
    for(size_t jidx(0); jidx <3; ++jidx){
      _ini_jpos[3*leg + jidx] = FSM_State<T>::_data->_legController->datas[leg]->q[jidx];
    }
  }

}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_SimpleJointPD<T>::run() {
  // This is just a test, should be running whatever other code you want
  Vec3<T> qDes, qdDes;
  float kp(0.);
  float kd(0.0);

  qDes << 0.0, 0.0, 0.0;
  qdDes << 0, 0, 0;


  float curr_time(0.0f);
  float omega(0.3f);
  float pi(3.141592f);
  // float amp_a(0.2f);
  // float amp_h(0.5f);
  // float amp_k(1.7f);
  float amp_a(0.0f);
  float amp_h(0.0f);
  float amp_k(0.0f);

  float p_des_a, p_des_h, p_des_k;
  float v_des_a, v_des_h, v_des_k;


  curr_time = 0.002*_iter++;
  // p_des =amp*sin(2*pi*omega*curr_time);

  p_des_a = amp_a + amp_a*sin(2*pi*omega*curr_time-pi/2);
  v_des_a =2*amp_a*pi*omega*cos(2*pi*omega*curr_time-pi/2);

  p_des_h = amp_h + amp_h*sin(2*pi*omega*curr_time-pi/2);
  v_des_h =2*amp_h*pi*omega*cos(2*pi*omega*curr_time-pi/2);

  p_des_k = amp_k + amp_k*sin(2*pi*omega*curr_time-pi/2);
  v_des_k =2*amp_k*pi*omega*cos(2*pi*omega*curr_time-pi/2);

  // p_des = 0.5;
  //v_des = 0.0*curr_time;

  for(int leg=0; leg<2; leg++){


   qDes<<  p_des_a, pow(-1, leg)*p_des_h, pow(-1, leg+1)*p_des_k;
   qdDes<<  pow(-1, leg+1)*v_des_a,pow(-1, leg)*v_des_h, pow(-1, leg+1)*v_des_k;
  }
  for( int leg =0; leg<2; leg++){
    FSM_State<T>::_data->_legController->commands[leg].kpJoint = kp*Mat3<float>::Identity();
    FSM_State<T>::_data->_legController->commands[leg].kdJoint = kd*Mat3<float>::Identity();

    FSM_State<T>::_data->_legController->commands[leg].tauFeedForward.setZero();
    FSM_State<T>::_data->_legController->commands[leg].qDes = qDes;
    FSM_State<T>::_data->_legController->commands[leg].qdDes = qdDes;
  }
  //
  // static double progress(0.);
  // progress += this->_data->userParameters->controller_dt;
  // double movement_duration(3.0);
  // double ratio = progress/movement_duration;
  // if(ratio > 1.) ratio = 1.;
  //
  // this->jointPDControl(0, ratio*qDes + (1. - ratio)*_ini_jpos.head(3), qdDes, 200, 2.0);
  // this->jointPDControl(1, ratio*qDes + (1. - ratio)*_ini_jpos.segment(3, 3), qdDes, 200, 2.0);

}

/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_SimpleJointPD<T>::onExit() {
  // Nothing to clean up when exiting
}

//template class FSM_State_SimpleJointPD<double>;
template class FSM_State_SimpleJointPD<float>;
