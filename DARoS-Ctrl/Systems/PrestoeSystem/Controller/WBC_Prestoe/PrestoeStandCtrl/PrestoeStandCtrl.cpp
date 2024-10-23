#include "PrestoeStandCtrl.hpp"
#include <WBIC_FB/ContactSet/SingleContact.hpp>
#include <WBIC_FB/TaskSet/BodyOriTask.hpp>
#include <WBIC_FB/TaskSet/JPosTask.hpp>
#include <WBIC_FB/TaskSet/BodyPosTask.hpp>
#include <WBIC_FB/TaskSet/LinkPosTask.hpp>

template<typename T>
PrestoeStandCtrl<T>::PrestoeStandCtrl(const FloatingBaseModel<T> * model): WBC_Ctrl<T>(model)
{
  _body_ori_task = new BodyOriTask<T>(this->_model);
  _jpos_task = new JPosTask<T>(this->_model);
  _body_pos_task = new BodyPosTask<T>(this->_model);

  this->_wbic_data->_W_rf = DVec<T>::Constant(3*_num_contact, 100.);
  for(size_t i(0); i<_num_contact; ++i){
    _foot_contact[i] = new SingleContact<T>(this->_model, prestoe_contact::rheel + i);
    // this->_wbic_data->_W_rf[3*i+2] = 1;
  }
}

template<typename T>
PrestoeStandCtrl<T>::~PrestoeStandCtrl(){
  delete _body_ori_task;
  delete _jpos_task;
  delete _body_pos_task;
  for(size_t i(0); i < _num_contact; ++i){
    delete _foot_contact[i];
  }
}

template<typename T>
void PrestoeStandCtrl<T>::_ContactTaskUpdate(void* input){
  _input_data = static_cast<PrestoeStandCtrlData<T>* >(input);

  _ParameterSetup();
  _CleanUp();
  Vec3<T> zero_vec3; zero_vec3.setZero();

   // Body ori task
  _quat_des = ori::rpyToQuat(_input_data->pBody_RPY_des);
  _body_ori_task->UpdateTask(&_quat_des, _input_data->vBody_Ori_des, zero_vec3);
  this->_task_list.push_back(_body_ori_task);
 
  // body pos task
  _body_pos_task->UpdateTask(&(_input_data->pBody_des), 
      _input_data->vBody_des, _input_data->aBody_des);
  this->_task_list.push_back(_body_pos_task);

   // Contact     
  for(size_t i(0); i<_num_contact; ++i){
      _foot_contact[i]->setRFDesired((DVec<T>)(_input_data->Fr_des[i]));
      _foot_contact[i]->UpdateContactSpec();
      this->_contact_list.push_back(_foot_contact[i]);
  }
    // joint position task
  DVec<T> zero_vec(prestoe::num_act_joint);zero_vec.setZero();
  _jpos_task->UpdateTask(&(_input_data->jpos_des), zero_vec, zero_vec);
  this->_task_list.push_back(_jpos_task);
}

template<typename T>
void PrestoeStandCtrl<T>::_ParameterSetup(){

  // for(size_t i(0); i<3; ++i){
    
  //   ((BodyOriTask<T>*)_body_ori_task)->_Kp[i] = param->Kp_ori[i];
  //   ((BodyOriTask<T>*)_body_ori_task)->_Kd[i] = param->Kd_ori[i];

  //   ((CentroidMomentumTask<T>*)_centroid_mom_task)->_Kp[i] = param->Kp_cam[i];
  //   ((CentroidMomentumTask<T>*)_centroid_mom_task)->_Kd[i] = param->Kd_cam[i];

  //   ((CentroidMomentumTask<T>*)_centroid_mom_task)->_Kp[i+3] = param->Kp_clm[i];
  //   ((CentroidMomentumTask<T>*)_centroid_mom_task)->_Kd[i+3] = param->Kd_clm[i];

  //   ((BodyPosTask<T>*)_body_pos_task)->_Kp[i] = param->Kp_body[i];
  //   ((BodyPosTask<T>*)_body_pos_task)->_Kd[i] = param->Kd_body[i];

  //  }

  // for(size_t jindx(0); jindx<prestoe::num_act_joint; ++jindx){
  //   ((JPosTask<T>*)_jpos_task)->_Kp[jindx] = 105.;
  //   ((JPosTask<T>*)_jpos_task)->_Kd[jindx] = 10;
  // }

}


template<typename T>
void PrestoeStandCtrl<T>::_CleanUp(){
  this->_contact_list.clear();
  this->_task_list.clear();
}

template<typename T>
void PrestoeStandCtrl<T>::_LCM_PublishData() {

  for(size_t i(0); i<3; ++i){
    this->_wbc_data_lcm.body_pos_cmd[i] = _input_data->pBody_des[i];
    this->_wbc_data_lcm.body_vel_cmd[i] = _input_data->vBody_des[i];
    this->_wbc_data_lcm.body_ori_cmd[i] = _quat_des[i];
    this->_wbc_data_lcm.body_rpy_cmd[i] = _input_data->pBody_RPY_des[i];
    this->_wbc_data_lcm.body_ang_vel_cmd[i] = _input_data->vBody_Ori_des[i];

    Quat<T> quat = this->_state->bodyOrientation;
    Mat3<T> Rot = ori::quaternionToRotationMatrix(quat);
    Vec3<T> global_body_vel = Rot.transpose() * this->_state->bodyVelocity.tail(3);

    this->_wbc_data_lcm.body_pos[i] = this->_state->bodyPosition[i];

    this->_wbc_data_lcm.body_vel[i] = global_body_vel[i];
    this->_wbc_data_lcm.body_ori[i] = this->_state->bodyOrientation[i];
    this->_wbc_data_lcm.body_rpy[i] = quatToRPY(this->_state->bodyOrientation)[i];
    this->_wbc_data_lcm.body_ang_vel[i] = this->_state->bodyVelocity[i];
    this->_wbc_data_lcm.com_pos[i] = this->_state->bodyPosition[i] + this->_model->getComPosWorld()[i];
  }
  this->_wbc_data_lcm.body_ori_cmd[3] = _quat_des[3];
  this->_wbc_data_lcm.body_ori[3] = this->_state->bodyOrientation[3];

  this->_wbcLCM.publish("wbc_lcm_data", &(this->_wbc_data_lcm) );
}

template class PrestoeStandCtrl<float>;
template class PrestoeStandCtrl<double>;
