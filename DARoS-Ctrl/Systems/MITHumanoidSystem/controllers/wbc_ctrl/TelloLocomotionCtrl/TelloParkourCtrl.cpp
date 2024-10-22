#include "TelloParkourCtrl.hpp"
#include <ContactSet/SingleContact.hpp>
#include <TaskSet/BodyOriTask.hpp>
#include <TaskSet/CentroidMomentumTask.hpp>
#include <TaskSet/CentroidMomentumTask2.hpp>
#include <TaskSet/JPosTask.hpp>
#include <TaskSet/CentroidAngMomentumTask.hpp>
#include <TaskSet/CentroidAngMomentumTask2.hpp>
#include <TaskSet/CentroidLinearMomentumTask.hpp>
#include <TaskSet/BodyPosTask.hpp>
#include <TaskSet/LinkPosTask.hpp>

template<typename T>
TelloParkourCtrl<T>::TelloParkourCtrl(const FloatingBaseModel<T> * model, const ControlFSMData_Tello<T>* _controlFSMData):
  WBC_Ctrl<T>(model)
{
  _body_ori_task = new BodyOriTask<T>(this->_model);
  _body_pos_task = new CentroidLinearMomentumTask<T>(this->_model);
  _jpos_task = new JPosTask<T>(this->_model);
  this->_wbic_data->_W_rf = DVec<T>::Constant(3*_num_contact, 1.);
  for(size_t i(0); i<_num_contact; ++i){
    _foot_contact[i] = new SingleContact<T>(this->_model, tello_contact::R_heel + i);
    this->_wbic_data->_W_rf[3*i+2] = _controlFSMData->_userParameters->REACTION_FORCE_Z_WEIGHT;
    this->_wbic_data->_W_rf[3*i] = _controlFSMData->_userParameters->REACTION_FORCE_X_WEIGHT;

  }

  _foot_task[0] = new LinkPosTask<T>(this->_model, tello_contact::R_heel);
  _foot_task[1] = new LinkPosTask<T>(this->_model, tello_contact::R_toe);
  _foot_task[2] = new LinkPosTask<T>(this->_model, tello_contact::L_heel);
  _foot_task[3] = new LinkPosTask<T>(this->_model, tello_contact::L_toe);  
}

template<typename T>
TelloParkourCtrl<T>::~TelloParkourCtrl(){
  delete _body_ori_task;
  delete _jpos_task;
  delete _body_pos_task;
  for(size_t i(0); i < _num_contact; ++i){
    delete _foot_contact[i];
  }
  for(size_t i(0); i<_num_contact; ++i){
    delete _foot_task[i];
  }
}

template<typename T>
void TelloParkourCtrl<T>::_ContactTaskUpdate(void* input, ControlFSMData_Tello<T> & data){
  _input_data = static_cast<TelloParkourCtrlData<T>* >(input);

  _ParameterSetup(data._userParameters);

  // Wash out the previous setup
  _CleanUp();
 
  // Body ori task
  _quat_des = ori::rpyToQuat(_input_data->pBody_RPY_des);
  Vec3<T> zero_vec3; zero_vec3.setZero();
  _body_ori_task->UpdateTask(&_quat_des, _input_data->vBody_Ori_des, zero_vec3);
  this->_task_list.push_back(_body_ori_task);

  // body pos task
  _body_pos_task->UpdateTask(&(_input_data->pBody_des), 
      _input_data->vBody_des, _input_data->aBody_des);
  this->_task_list.push_back(_body_pos_task);


  for (size_t i(0); i < _num_contact; ++i) {
    if (_input_data->contact_state[i] > 0.5){ //if it's 1 then it is swinging, if it's 0 then it is contact
      _foot_task[i]->UpdateTask(&(_input_data->pFoot_des[i]), _input_data->vFoot_des[i], _input_data->aFoot_des[i]);
      this->_task_list.push_back(_foot_task[i]);
    } else{
    _foot_contact[i]->setRFDesired((DVec<T>)(_input_data->Fr_des[i]));
    _foot_contact[i]->UpdateContactSpec();
    this->_contact_list.push_back(_foot_contact[i]);
    }
  }


  // joint position task
  DVec<T> zero_jpos(tello::num_act_joint);zero_jpos.setZero();
  _jpos_task->UpdateTask(&(_input_data->jpos_des), zero_jpos, zero_jpos);
  this->_task_list.push_back(_jpos_task);

}

template<typename T>
void TelloParkourCtrl<T>::_ParameterSetup(const TelloParameters* param){

  for(size_t i(0); i<3; ++i){
    
    ((BodyOriTask<T>*)_body_ori_task)->_Kp[i] = param->Kp_ori[i];
    ((BodyOriTask<T>*)_body_ori_task)->_Kd[i] = param->Kd_ori[i];

    ((CentroidLinearMomentumTask<T>*)_body_pos_task)->_Kp[i] = param->Kp_body[i];
    ((CentroidLinearMomentumTask<T>*)_body_pos_task)->_Kd[i] = param->Kd_body[i];

    for(size_t j(0); j<_num_contact; ++j){
      ((LinkPosTask<T>*)_foot_task[j])->_Kp[i] = param->Kp_foot[i];
      ((LinkPosTask<T>*)_foot_task[j])->_Kd[i] = param->Kd_foot[i];
    }

   }

  for(size_t jindx(0); jindx<tello::num_act_joint; ++jindx){
    ((JPosTask<T>*)_jpos_task)->_Kp[jindx] = param->Kp_joint;
    ((JPosTask<T>*)_jpos_task)->_Kd[jindx] = param->Kd_joint;
  }

}


template<typename T>
void TelloParkourCtrl<T>::_CleanUp(){
  this->_contact_list.clear();
  this->_task_list.clear();
}

template<typename T>
void TelloParkourCtrl<T>::_LCM_PublishData() {

  //for(size_t i(0); i<3; ++i){
    //this->_wbc_data_lcm.body_pos_cmd[i] = _input_data->pBody_des[i];
    //this->_wbc_data_lcm.body_vel_cmd[i] = _input_data->vBody_des[i];
    //this->_wbc_data_lcm.body_ori_cmd[i] = _quat_des[i];

    //Quat<T> quat = this->_state.bodyOrientation;
    //Mat3<T> Rot = ori::quaternionToRotationMatrix(quat);
    //Vec3<T> global_body_vel = Rot.transpose() * this->_state.bodyVelocity.tail(3);

    //this->_wbc_data_lcm.body_pos[i] = this->_state.bodyPosition[i];
    //this->_wbc_data_lcm.body_vel[i] = global_body_vel[i];
    //this->_wbc_data_lcm.body_ori[i] = this->_state.bodyOrientation[i];
    //this->_wbc_data_lcm.body_ang_vel[i] = this->_state.bodyVelocity[i];
  //}
  //this->_wbc_data_lcm.body_ori_cmd[3] = _quat_des[3];
  //this->_wbc_data_lcm.body_ori[3] = this->_state.bodyOrientation[3];

  //this->_wbcLCM.publish("wbc_lcm_data", &(this->_wbc_data_lcm) );
}

template class TelloParkourCtrl<float>;
