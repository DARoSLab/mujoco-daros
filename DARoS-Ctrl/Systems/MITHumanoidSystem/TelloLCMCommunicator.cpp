#include "TelloLCMCommunicator.hpp"
#include <unistd.h>
#include <TelloSystem.hpp>

template<typename T>
TelloLCMCommunicator<T>::TelloLCMCommunicator(TelloSystem<T> * system, int ttl):
  _tello_system(system),
  _tello_lcm(getLcmUrl(ttl))
{
  std::cout<<"@@@@@@@@@@@@@@@@@@"<<std::endl;
  _model = _tello_system->getRobot()->buildModel();
  std::cout<<"--------------"<<std::endl;
  _visualizationData =_tello_system->_visualizationData;
  _tello_lcm.subscribe("tello_parameters", &TelloLCMCommunicator<T>::handleParameterLCM, this);
  _tello_lcm_thread = std::thread(&TelloLCMCommunicator<T>::_tello_LCMThread, this);
}

template <typename T>
void TelloLCMCommunicator<T>::VisualInfoSending(FBModelState<T> _state, bool _is_estimate){
  _model.setState(_state);
  _model.forwardKinematics();

  Quat<T> body_quat = _model._state.bodyOrientation;

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
    _lcmt.body_pos[i] = _model._state.bodyPosition[i];
    _lcmt.body_ori_quat_visual[i] = body_quat[i];
    _lcmt.body_ori_quat_est[i] = _model._state.bodyOrientation[i];
  }
  _lcmt.body_ori_quat_visual[3] = body_quat[3];
  _lcmt.body_ori_quat_est[3] = _model._state.bodyOrientation[3];

  for(size_t i(0); i < tello::num_act_joint; ++i){
    _lcmt.jpos[i] = _model._state.q[i]; 
  }

  if(_is_estimate){
    _tello_lcm.publish("tello_est_state_info", &_lcmt);
  } else {
    _tello_lcm.publish("humanoid_visualization_info", &_lcmt);
  }

}

template<typename T>
void TelloLCMCommunicator<T>::debugVisualInfoSending(){
  //Build Debug Visualization LCM Message
  _visualizationData->buildMessageLCM(_debug_lcmt);
  _tello_lcm.publish("debug_visualization", &_debug_lcmt);
}

template<typename T>
void TelloLCMCommunicator<T>::finalizeStep(tello_joint_data_lcmt* data,
 tello_joint_command_lcmt* command,
 state_estimator_lcmt* se) {
  _tello_lcm.publish("joint_control_command", command);
  _tello_lcm.publish("joint_control_data", data);
  _tello_lcm.publish("state_estimator_data", se);
}


template<typename T>
void TelloLCMCommunicator<T>::handleParameterLCM(const lcm::ReceiveBuffer *rbuf,
    const std::string & chan, const tello_control_lcmt * msg){
  (void)rbuf;
  (void)chan;

  
  _tello_system->vel += msg->vel;
  std::cout<<"current vel: " << _tello_system->vel << std::endl;
  //if(msg->xbox_ctrl){
    //_tello_system->_sys_gamepadCommand->leftStickAnalog[0] = msg->stick_left_horizontal*0.8;
    //_tello_system->_sys_gamepadCommand->leftStickAnalog[1] = -msg->stick_left_vertical*0.8;
    //_tello_system->_sys_gamepadCommand->rightStickAnalog[0] = -msg->stick_right_horizontal*1.3;
    //_tello_system->_sys_gamepadCommand->rightStickAnalog[1] = msg->stick_right_vertical;
   //}else{
    //_tello_system->_sys_gamepadCommand->rightStickAnalog[0] =  msg->turn_right*1.5;
    //_tello_system->_sys_gamepadCommand->leftStickAnalog[1] =  msg->key_up*0.2;
    //if(!(msg->key_up > 0)){
      //_tello_system->_sys_gamepadCommand->leftStickAnalog[1] =  -msg->key_down*0.2;
    //}
    //_tello_system->_sys_gamepadCommand->leftStickAnalog[0] =  msg->key_right * 0.2;
    //if(!(msg->key_right > 0)){
      //_tello_system->_sys_gamepadCommand->leftStickAnalog[0] =  -msg->key_left* 0.2;
   //}
 //}

}
template class TelloLCMCommunicator<double>;
template class TelloLCMCommunicator<float>;
