#include "LocomotionCtrl.hpp"
#include <ContactSet/SingleContact.hpp>
#include <TaskSet/BodyOriTask.hpp>
#include <TaskSet/BodyRPZTask.hpp>
#include <TaskSet/BodyZTask.hpp>
#include <TaskSet/BodyPosTask.hpp>
#include <TaskSet/LinkPosTask.hpp>
#include <ParamHandler/ParamHandler.hpp>

template<typename T>
LocomotionCtrl<T>::LocomotionCtrl(const ControlFSMData<T> * data):WBC_Ctrl<T>(data)
{
  //_body_pos_task = new BodyPosAdaptiveTask<T>(&(this->_model), this->_data->userParameters->controller_dt );
  _body_ori_task = new BodyOriTask<T>(&(this->_model));
  _body_pos_task = new BodyPosTask<T>(&(this->_model));
  _body_rpz_task = new BodyRPZTask<T>(&(this->_model));
  _body_z_task = new BodyZTask<T>(&(this->_model));
  _foot_contact[0] = new SingleContact<T>(&(this->_model), pat_biped_linkID::RF);
  _foot_contact[1] = new SingleContact<T>(&(this->_model), pat_biped_linkID::LF);

  _foot_task[0] = new LinkPosTask<T>(&(this->_model), pat_biped_linkID::RF);
  _foot_task[1] = new LinkPosTask<T>(&(this->_model), pat_biped_linkID::LF);
  _ParameterSetup(THIS_COM"config/pat-wbc-parameters.yaml");
}

template<typename T>
LocomotionCtrl<T>::~LocomotionCtrl(){
  delete _body_pos_task;
  delete _body_ori_task;

  for(size_t i (0); i< pat_biped::num_legs; ++i){
    delete _foot_contact[i];
    delete _foot_task[i];
  }
}

template<typename T>
void LocomotionCtrl<T>::_ContactTaskUpdate(void* input){
  //this->_EstimateContactForces(); // sehwan - this prints out gabriel's estimated contact forces

  _input_data = static_cast<LocomotionCtrlData<T>* >(input);

  // _ParameterSetup(this->_data->userParameters);


  // Wash out the previous setup
  _CleanUp();

  _rpy_des  = _input_data->pBody_RPY_des;
  _rpy_vel_des = _input_data->vBody_Ori_des;

  _rpz_des[0] = _rpy_des[0];
  _rpz_des[1] = _rpy_des[1];
  _rpz_des[2] = _input_data->pBody_des[2];

  _rpz_vel_des[0] = _rpy_vel_des[0];
  _rpz_vel_des[1] = _rpy_vel_des[1];
  _rpz_vel_des[2] = _input_data->vBody_des[2];

  _rpz_acc_des[0] = 0.0;
  _rpz_acc_des[1] = 0.0;
  _rpz_acc_des[2] = _input_data->aBody_des[2];


  Vec3<T> zero_vec3; zero_vec3.setZero();



  // // ((BodyPosAdaptiveTask<T>*)_body_pos_task)->setAccCompensationInput(this->_F_ext_est_DOB);
  // // ((BodyPosAdaptiveTask<T>*)_body_pos_task)->setAccCompensationInput(this->_F_ext_est);
  if(_input_data->contact_state[0] > 0. && _input_data->contact_state[1] > 0.){//Double stance
    _quat_des = ori::rpyToQuat(_input_data->pBody_RPY_des);
    _body_ori_task->UpdateTask(&_quat_des, _input_data->vBody_Ori_des, zero_vec3);
    // Body pos task
    _body_pos_task->UpdateTask(
        &(_input_data->pBody_des),
        _input_data->vBody_des,
        _input_data->aBody_des);
    this->_task_list.push_back(_body_ori_task);
    this->_task_list.push_back(_body_pos_task);

  }else{//Single stance

    _body_rpz_task->UpdateTask(&_rpz_des, _rpz_vel_des, _rpz_acc_des);
    this->_task_list.push_back(_body_rpz_task);

  }
  _body_z_task->UpdateTask(&_rpz_des, _rpz_vel_des, _rpz_acc_des);
  // this->_task_list.push_back(_body_z_task);

  size_t Jc_row(0);
  for (size_t i(0); i <pat_biped::num_legs; ++i) {
    if(_input_data->contact_state[i] > 0.){ Jc_row += 3; }
  }
  this->_Jc_full = DMat<T>(Jc_row, this->_model._nDof);


  size_t ct_idx(0);
  for(size_t leg(0); leg<pat_biped::num_legs; ++leg){
    if(_input_data->contact_state[leg] > 0.){ // Contact
      _foot_contact[leg]->setRFDesired((DVec<T>)(_input_data->Fr_des[leg]));
      _foot_contact[leg]->UpdateContactSpec();
      this->_contact_list.push_back(_foot_contact[leg]);

      int link_idx= 3*leg+10;
      this->_Jc_full.block(3*ct_idx, 0, 3, this->_model._nDof)  = this->_model._Jc[link_idx];
      ++ct_idx;

      //printf("Jc row: %zu\n", Jc_row);

    }else{ // No Contact (swing)
      _foot_task[leg]->UpdateTask(
          &(_input_data->pFoot_des[leg]),
          _input_data->vFoot_des[leg],
          _input_data->aFoot_des[leg]);
          //zero_vec3);
      this->_task_list.push_back(_foot_task[leg]);
    }
  }
}
template<typename T>
void LocomotionCtrl<T>::_ParameterSetup(const std::string & file){
  ParamHandler handler(file);


  handler.getVector("Kp_body", _Kp_body);
  handler.getVector("Kd_body", _Kd_body);
  handler.getVector("Kp_foot", _Kp_foot);
  handler.getVector("Kd_foot", _Kd_foot);
  handler.getVector("Kp_ori", _Kp_ori);
  handler.getVector("Kd_ori", _Kd_ori);
  handler.getVector("Kp_joint", _Kp_joint);
  handler.getVector("Kd_joint", _Kd_ori);

  for(size_t i(0); i<3; ++i){
    ((BodyPosTask<T>*)_body_pos_task)->_Kp[i] = _Kp_body[i];
    ((BodyPosTask<T>*)_body_pos_task)->_Kd[i] = _Kd_body[i];


    ((BodyOriTask<T>*)_body_ori_task)->_Kp[i] = _Kp_ori[i];
    ((BodyOriTask<T>*)_body_ori_task)->_Kd[i] = _Kd_ori[i];

    for(size_t j(0); j<pat_biped::num_legs; ++j){
      ((LinkPosTask<T>*)_foot_task[j])->_Kp[i] = _Kp_foot[i];
      ((LinkPosTask<T>*)_foot_task[j])->_Kd[i] = _Kd_foot[i];
    }

    this->_Kp_joint[i] = _Kp_joint[i];
    this->_Kd_joint[i] = _Kd_joint[i];


   }

  printf("[WBC_biped LOCOMOTION] Parameter Setup is completed\n");


}
template<typename T>
void LocomotionCtrl<T>::_ParameterSetup(const PatParameters* param){

  for(size_t i(0); i<3; ++i){
    ((BodyPosTask<T>*)_body_pos_task)->_Kp[i] = param->Kp_body[i];
    ((BodyPosTask<T>*)_body_pos_task)->_Kd[i] = param->Kd_body[i];

    //((BodyPosAdaptiveTask<T>*)_body_pos_task)->_Kp[i] = param->Kp_body[i];
    //((BodyPosAdaptiveTask<T>*)_body_pos_task)->_Kd[i] = param->Kd_body[i];


    ((BodyOriTask<T>*)_body_ori_task)->_Kp[i] = param->Kp_ori[i];
    ((BodyOriTask<T>*)_body_ori_task)->_Kd[i] = param->Kd_ori[i];

    for(size_t j(0); j<pat_biped::num_legs; ++j){
      ((LinkPosTask<T>*)_foot_task[j])->_Kp[i] = param->Kp_foot[i];
      ((LinkPosTask<T>*)_foot_task[j])->_Kd[i] = param->Kd_foot[i];
      //((LinkPosTask<T>*)_foot_task[j])->_Kp_kin[i] = 1.5;
    }

    this->_Kp_joint[i] = param->Kp_joint[i];
    this->_Kd_joint[i] = param->Kd_joint[i];

    //this->_Kp_joint_swing[i] = param->Kp_joint_swing[i];
    //this->_Kd_joint_swing[i] = param->Kd_joint_swing[i];
   }
}


template<typename T>
void LocomotionCtrl<T>::_CleanUp(){
  this->_contact_list.clear();
  this->_task_list.clear();
}

template<typename T>
void LocomotionCtrl<T>::_LCM_PublishData() {
  int iter(0);
  for(size_t leg(0); leg<pat_biped::num_legs; ++leg){
    _Fr_result[leg].setZero();

    if(_input_data->contact_state[leg]>0.){
      for(size_t i(0); i<3; ++i){
        _Fr_result[leg][i] = this->_wbic_data->_Fr[3*iter + i];
      }
      ++iter;
    }

    if(_input_data->contact_state[leg] > 0.){ // Contact
      this->_wbc_data_lcm.contact_est[leg] = 1;
    }else{
      this->_wbc_data_lcm.contact_est[leg] = 0;
    }
  }

  auto body_rpy = ori::quatToRPY(this->_state.bodyOrientation);

  for(size_t i(0); i<3; ++i){
    this->_wbc_data_lcm.Fext_est[i] = this->_F_ext_est[i];
    this->_wbc_data_lcm.Fext_est_DOB[i] = this->_F_ext_est_DOB[i];

    this->_wbc_data_lcm.foot_pos[i] = this->_model._pGC[pat_biped_linkID::RF][i];
    this->_wbc_data_lcm.foot_vel[i] = this->_model._vGC[pat_biped_linkID::RF][i];

    this->_wbc_data_lcm.foot_pos[i + 3] = this->_model._pGC[pat_biped_linkID::LF][i];
    this->_wbc_data_lcm.foot_vel[i + 3] = this->_model._vGC[pat_biped_linkID::LF][i];

    for(size_t leg(0); leg<pat_biped::num_legs; ++leg){
      this->_wbc_data_lcm.Fr_des[3*leg + i] = _input_data->Fr_des[leg][i];
      this->_wbc_data_lcm.Fr[3*leg + i] = _Fr_result[leg][i];

      this->_wbc_data_lcm.foot_pos_cmd[3*leg + i] = _input_data->pFoot_des[leg][i];
      this->_wbc_data_lcm.foot_vel_cmd[3*leg + i] = _input_data->vFoot_des[leg][i];
      this->_wbc_data_lcm.foot_acc_cmd[3*leg + i] = _input_data->aFoot_des[leg][i];

      this->_wbc_data_lcm.jpos_cmd[3*leg + i] = this->_des_jpos[3*leg + i];
      this->_wbc_data_lcm.jvel_cmd[3*leg + i] = this->_des_jvel[3*leg + i];

      this->_wbc_data_lcm.jpos[3*leg + i] = this->_state.q[3*leg + i];
      this->_wbc_data_lcm.jvel[3*leg + i] = this->_state.qd[3*leg + i];
    }

    this->_wbc_data_lcm.body_pos_cmd[i] = _input_data->pBody_des[i];
    this->_wbc_data_lcm.body_vel_cmd[i] = _input_data->vBody_des[i];
    this->_wbc_data_lcm.body_ori_cmd[i] = _quat_des[i];

    Quat<T> quat = this->_state.bodyOrientation;
    Mat3<T> Rot = ori::quaternionToRotationMatrix(quat);
    Vec3<T> global_body_vel = Rot.transpose() * this->_state.bodyVelocity.tail(3);

    this->_wbc_data_lcm.body_pos[i] = this->_state.bodyPosition[i];
    this->_wbc_data_lcm.body_vel[i] = global_body_vel[i];
    this->_wbc_data_lcm.body_ori[i] = this->_state.bodyOrientation[i];
    this->_wbc_data_lcm.body_rpy[i] = body_rpy[i];
    this->_wbc_data_lcm.body_ang_vel[i] = this->_state.bodyVelocity[i];
  }
  this->_wbc_data_lcm.body_ori_cmd[3] = _quat_des[3];
  this->_wbc_data_lcm.body_ori[3] = this->_state.bodyOrientation[3];

  if(_input_data->lcm_publish_channel == 0){
    this->_wbcLCM.publish("wbc_lcm_data", &(this->_wbc_data_lcm) );
  }
  else{
    this->_wbcLCM.publish("wbc_loco_jump_lcm_data", &(this->_wbc_data_lcm) );
  }
}

template class LocomotionCtrl<float>;
//template class LocomotionCtrl<double>;
