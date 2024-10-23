#include "StaccatoeLocomotionCtrl.hpp"
#include <ContactSet/SingleContact.hpp>
#include <TaskSet/BodyOriTask.hpp>
#include <TaskSet/CentroidMomentumTask.hpp>
#include <TaskSet/CentroidMomentumTask2.hpp>
#include <TaskSet/JPosTask.hpp>
#include <TaskSet/LinkPosTask.hpp>
#include <TaskSet/CentroidAngMomentumTask.hpp>
#include <TaskSet/CentroidAngMomentumTask2.hpp>
#include <TaskSet/CentroidLinearMomentumTask.hpp>
#include <TaskSet/BodyPosTask.hpp>

template<typename T>
StaccatoeLocomotionCtrl<T>::StaccatoeLocomotionCtrl(const FloatingBaseModel<T> * model):
  WBC_Ctrl<T>(model)
{
  _centroid_mom_task = new CentroidMomentumTask<T>(this->_model);
  _body_ori_task = new BodyOriTask<T>(this->_model);
  _jpos_task = new JPosTask<T>(this->_model);
  //_cam_task = new CentroidAngMomentumTask<T>(this->_model);
  _cam_task = new CentroidAngMomentumTask2<T>(this->_model);
  //_body_pos_task = new BodyPosTask<T>(this->_model);
  _body_pos_task = new CentroidLinearMomentumTask<T>(this->_model);
  _foot_task = new LinkPosTask<T>(this->_model, staccatoe_link::foot);

  this->_wbic_data->_W_rf = DVec<T>::Constant(3*_num_contact, 1.);
  for(size_t i(0); i<_num_contact; ++i){
    _foot_contact[i] = new SingleContact<T>(this->_model, staccatoe_contact::heel + i);
    this->_wbic_data->_W_rf[3*i+2] = 0.001;
  }
}

template<typename T>
StaccatoeLocomotionCtrl<T>::~StaccatoeLocomotionCtrl(){
  delete _body_ori_task;
  delete _centroid_mom_task;
  delete _jpos_task;
  delete _cam_task;
  delete _body_pos_task;
  delete _foot_task;

  for(size_t i(0); i < _num_contact; ++i){
    delete _foot_contact[i];
  }
}

template<typename T>
void StaccatoeLocomotionCtrl<T>::_ContactTaskUpdate(void* input, ControlFSMData_Staccatoe<T> & data){
  _input_data = static_cast<StaccatoeLocomotionCtrlData<T>* >(input);

  _ParameterSetup(data._userParameters);

  // Wash out the previous setup
  _CleanUp();
  Vec3<T> zero_vec3; zero_vec3.setZero();
// centroid ang momentum task
  //_cam_task->UpdateTask(&_quat_des, zero_vec3, zero_vec3);
  //this->_task_list.push_back(_cam_task);

   // Body ori task
  _quat_des = ori::rpyToQuat(_input_data->pBody_RPY_des);
  _body_ori_task->UpdateTask(&_quat_des, _input_data->vBody_Ori_des, zero_vec3);
  this->_task_list.push_back(_body_ori_task);

 
  // body pos task
  _body_pos_task->UpdateTask(&(_input_data->pBody_des), 
      _input_data->vBody_des, _input_data->aBody_des);
  this->_task_list.push_back(_body_pos_task);


  // joint position task
  DVec<T> zero_vec(staccatoe::num_act_joint);zero_vec.setZero();
  _jpos_task->UpdateTask(&(_input_data->jpos_des), zero_vec, zero_vec);
  this->_task_list.push_back(_jpos_task);

  // centroid momentum task
  //_centroid_mom_pos_des.head(4) = _quat_des;
  //_centroid_mom_pos_des.tail(3) = _input_data->pBody_des;
  //DVec<T> zero_vec6(6); zero_vec6.setZero();
  //_centroid_mom_task->UpdateTask(&_centroid_mom_pos_des, zero_vec6, zero_vec6);
  //this->_task_list.push_back(_centroid_mom_task);


   
   // Contact     
   for(size_t i(0); i<_num_contact; ++i){
    _foot_contact[i]->setRFDesired((DVec<T>)(_input_data->Fr_des[i]));
    _foot_contact[i]->UpdateContactSpec();
    this->_contact_list.push_back(_foot_contact[i]);
  }

}

template<typename T>
void StaccatoeLocomotionCtrl<T>::_ParameterSetup(const StaccatoeParameters* param){

  for(size_t i(0); i<3; ++i){
    
    ((BodyOriTask<T>*)_body_ori_task)->_Kp[i] = _Kp_ori[i];
    ((BodyOriTask<T>*)_body_ori_task)->_Kd[i] = _Kd_ori[i];

    ((CentroidMomentumTask<T>*)_centroid_mom_task)->_Kp[i] = _Kp_cam[i];
    ((CentroidMomentumTask<T>*)_centroid_mom_task)->_Kd[i] = _Kd_cam[i];

    ((CentroidMomentumTask<T>*)_centroid_mom_task)->_Kp[i+3] = _Kp_clm[i];
    ((CentroidMomentumTask<T>*)_centroid_mom_task)->_Kd[i+3] = _Kd_clm[i];

    ((CentroidAngMomentumTask<T>*)_cam_task)->_Kp[i] = _Kp_cam[i];
    ((CentroidAngMomentumTask<T>*)_cam_task)->_Kd[i] = _Kd_cam[i];

    ((BodyPosTask<T>*)_body_pos_task)->_Kp[i] = _Kp_body[i];
    ((BodyPosTask<T>*)_body_pos_task)->_Kd[i] = _Kd_body[i];

   }

  for(size_t jindx(0); jindx<staccatoe::num_act_joint; ++jindx){
    ((JPosTask<T>*)_jpos_task)->_Kp[jindx] = 100.;
    ((JPosTask<T>*)_jpos_task)->_Kd[jindx] = 5.0;
  }
}


template<typename T>
void StaccatoeLocomotionCtrl<T>::_CleanUp(){
  this->_contact_list.clear();
  this->_task_list.clear();
}

template<typename T>
void StaccatoeLocomotionCtrl<T>::_LCM_PublishData() {

  for(size_t i(0); i<3; ++i){
    this->_wbc_data_lcm.body_pos_cmd[i] = _input_data->pBody_des[i];
    this->_wbc_data_lcm.body_vel_cmd[i] = _input_data->vBody_des[i];
    this->_wbc_data_lcm.body_ori_cmd[i] = _quat_des[i];

    Quat<T> quat = this->_state.bodyOrientation;
    Mat3<T> Rot = ori::quaternionToRotationMatrix(quat);
    Vec3<T> global_body_vel = Rot.transpose() * this->_state.bodyVelocity.tail(3);

    this->_wbc_data_lcm.body_pos[i] = this->_state.bodyPosition[i];
    this->_wbc_data_lcm.body_vel[i] = global_body_vel[i];
    this->_wbc_data_lcm.body_ori[i] = this->_state.bodyOrientation[i];
    this->_wbc_data_lcm.body_ang_vel[i] = this->_state.bodyVelocity[i];
  }
  this->_wbc_data_lcm.body_ori_cmd[3] = _quat_des[3];
  this->_wbc_data_lcm.body_ori[3] = this->_state.bodyOrientation[3];

  this->_wbcLCM.publish("wbc_lcm_data", &(this->_wbc_data_lcm) );
}

template class StaccatoeLocomotionCtrl<float>;
//template class StaccatoeLocomotionCtrl<double>;
