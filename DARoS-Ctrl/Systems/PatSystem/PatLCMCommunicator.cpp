#include "PatLCMCommunicator.hpp"
#include <unistd.h>
#include <PatSystem.hpp>
#include <stdio.h>

template<typename T>
PatLCMCommunicator<T>::PatLCMCommunicator(PatSystem<T> * system, int ttl):
  _pat_system(system),
  _pat_lcm(getLcmUrl(ttl))
{
  _dyn_model = _pat_system->getRobot()->buildModel();
  _visualizationData =_pat_system->_visualizationData;

  _pat_lcm.subscribe("quadruped_parameters", &PatLCMCommunicator<T>::handleParameterLCM, this);
  _pat_lcm.subscribe("quadruped_menu_data", &PatLCMCommunicator<T>::handleMenuDataLCM, this);
  _pat_lcm_thread = std::thread(&PatLCMCommunicator<T>::_pat_LCMThread, this);
}

template <typename T>
void PatLCMCommunicator<T>::VisualInfoSendingURDF(const FBModelState<T> & _state, bool _is_estimate){
  Quat<T> body_quat = _state.bodyOrientation;
  Quat<T> rot_y, rot_z;
  Quat<T> rot_x_half_neg;
  rot_y<< cos(M_PI/2.), 0., sin(M_PI/2.), 0.;
  rot_z<< cos(-M_PI/4.), 0., 0. , sin(-M_PI/4.);
  rot_x_half_neg<< cos(-M_PI/4.), sin(-M_PI/4.), 0., 0.;

  body_quat = ori::quatProduct(rot_y, body_quat);
  body_quat = ori::quatProduct(rot_x_half_neg, body_quat);

  Quat<T> rot_x_half; rot_x_half<< cos(-M_PI/4.),  sin(-M_PI/4.), 0., 0.;
  Quat<T> rot_y_half; rot_y_half<< cos(-M_PI/4.), 0., sin(-M_PI/4.), 0.;
  body_quat = ori::quatProduct(body_quat, rot_x_half);
  body_quat = ori::quatProduct(body_quat, rot_y_half);

  for(size_t i(0); i<3; ++i){
    _lcmt.body_pos[i] = _state.bodyPosition[i];
    _lcmt.body_ori_quat_visual[i] = body_quat[i];
    _lcmt.body_ori_quat_est[i] = _state.bodyOrientation[i];
  }
  _lcmt.body_ori_quat_visual[3] = body_quat[3];
  _lcmt.body_ori_quat_est[3] = _state.bodyOrientation[3];


  for(size_t i(0); i < pat_biped::num_act_joints; ++i){
    _lcmt.jpos[i] = _state.q[i];
  }

  // TEST
  // _lcmt.body_pos[0] = 0.0;
  // _lcmt.body_pos[1] = 0.0;
  // _lcmt.body_pos[2] = 0.0;

  //_lcmt.body_ori_quat_visual[0] = 1.;
  //_lcmt.body_ori_quat_visual[1] = 0.;
  //_lcmt.body_ori_quat_visual[2] = 0.;
  //_lcmt.body_ori_quat_visual[3] = 0.;

  //static int iter(0);
  //++iter;
  //T t = 0.002*iter;
  //_lcmt.jpos[0] = -0.3;// + sin(2*M_PI*t);
  //_lcmt.jpos[1] = -1.0;// + sin(2*M_PI*t);
  //_lcmt.jpos[2] = 2.;
  //_lcmt.jpos[3] = 0.3; // - sin(2*M_PI*t);
  //_lcmt.jpos[4] = -1.0;
  //_lcmt.jpos[5] = 2.;


  if(_is_estimate){
    _pat_lcm.publish("pat_state_est_info", &_lcmt);
  } else {
    _pat_lcm.publish("pat_state_info", &_lcmt);
  }
}

template<typename T>
void PatLCMCommunicator<T>::debugVisualInfoSending(){

  //Build Debug Visualization LCM Message
  _visualizationData->buildMessageLCM(_debug_lcmt);
  _pat_lcm.publish("debug_visualization", &_debug_lcmt);
}

template<typename T>
void PatLCMCommunicator<T>::finalizeStep(
    pat_leg_control_data_lcmt* data,
    pat_leg_control_command_lcmt* command,
    state_estimator_lcmt* se) {

  _pat_lcm.publish("leg_control_command", command);
  _pat_lcm.publish("leg_control_data", data);
  _pat_lcm.publish("state_estimator_data", se);
}

template<typename T>
void PatLCMCommunicator<T>::handleMenuDataLCM(const lcm::ReceiveBuffer *rbuf,
    const std::string & chan, const quadruped_menu_data_lcmt * msg){
  (void)rbuf;
  (void)chan;

  _pat_system->_sys_parameters.control_mode =  msg->control_mode;
  printf("Control mode %f \n", _pat_system->_sys_parameters.control_mode);
  if(msg->cheater_mode == 0 || msg->cheater_mode == 1){
    //std::cout << "Updated *new* cheater mode: " << msg->cheater_mode<< std::endl;
    _pat_system->_sys_parameters.cheater_mode = msg->cheater_mode;
  }

  _pat_system->_sys_parameters.use_wbc = msg->use_wbc;
  _pat_system->_sys_parameters.use_rc = 0; //msg->use_rc;[FIX this]
    //std::cout << "use_wbc msg: " << (int)msg->use_wbc<< std::endl;
    //std::cout << "use_rc msg: " << (int)msg->use_rc<< std::endl;
    //std::cout << "use_rc: " << _pat_system->_sys_parameters.use_rc<< std::endl;
  // Added joint gains
  //_pat_system->_sys_parameters.Kp_joint[0] = msg->Kp_joint[0];
  //_pat_system->_sys_parameters.Kp_joint[1] = msg->Kp_joint[1];
  //_pat_system->_sys_parameters.Kp_joint[2] = msg->Kp_joint[2];
  //_pat_system->_sys_parameters.Kd_joint[0] = msg->Kd_joint[0];
  //_pat_system->_sys_parameters.Kd_joint[1] = msg->Kd_joint[1];
  //_pat_system->_sys_parameters.Kd_joint[2] = msg->Kd_joint[2];
  //_pat_system->_sys_parameters.Kd_joint = msg->Kd_joint;
}

template<typename T>
void PatLCMCommunicator<T>::handleParameterLCM(const lcm::ReceiveBuffer *rbuf,
    const std::string & chan, const quadruped_parameters_lcmt * msg){
  (void)rbuf;
  (void)chan;

  if(msg->xbox_ctrl){
    _pat_system->_sys_gamepadCommand->leftStickAnalog[0] = msg->stick_left_horizontal*0.8;
    _pat_system->_sys_gamepadCommand->leftStickAnalog[1] = -msg->stick_left_vertical*0.8;
    _pat_system->_sys_gamepadCommand->rightStickAnalog[0] = -msg->stick_right_horizontal*1.3;
    _pat_system->_sys_gamepadCommand->rightStickAnalog[1] = msg->stick_right_vertical;
  } else{
    _pat_system->_sys_gamepadCommand->leftStickAnalog[1] =  msg->key_vertical*0.7; //forward & backward
    _pat_system->_sys_gamepadCommand->leftStickAnalog[0] =  msg->key_horizontal* 0.3; //right & left

    _pat_system->_sys_gamepadCommand->rightStickAnalog[0] =  msg->key_turn*1.0;
    _pat_system->_sys_gamepadCommand->rightStickAnalog[1] = msg->key_pitch*0.1;

    _pat_system->_sys_gamepadCommand->rightTriggerButton = false;
    if(msg->jump_trigger){
      _pat_system->_sys_gamepadCommand->rightTriggerButton = true;
    }
    //printf("key vertial, turn: %f, %f\n", msg->key_vertical, msg->key_turn );
 }
}

template class PatLCMCommunicator<double>;
template class PatLCMCommunicator<float>;
