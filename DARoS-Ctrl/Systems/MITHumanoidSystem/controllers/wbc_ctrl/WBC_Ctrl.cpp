#include "WBC_Ctrl.hpp"
#include <pretty_print.h>
#include <Utilities/Timer.h>

template<typename T>
WBC_Ctrl<T>::WBC_Ctrl(const FloatingBaseModel<T> * model):
  _full_config(model->_nDof+1),
  _tau_ff(model->_nDof-6),
  _des_jpos(model->_nDof-6),
  _des_jvel(model->_nDof-6),
  _wbcLCM(getLcmUrl(255))
{
  _iter = 0;
  _full_config.setZero();

  _model = const_cast<FloatingBaseModel<T> *>(model) ;
  _state.q = DVec<T>::Zero(_model->_nDof-6);
  _state.qd = DVec<T>::Zero(_model->_nDof-6);

  _kin_wbc = new KinWBC<T>(model->_nDof);

  _wbic = new WBIC<T>(model->_nDof, &(_contact_list), &(_task_list));
  //_wbic = new WBIC_test<T>(model->_nDof, &(_contact_list), &(_task_list));
  _wbic_data = new WBIC_ExtraData<T>();

  _wbic_data->_W_floating = DVec<T>::Constant(6, 0.1);

  _Kp_joint.resize(_model->_nDof-6, 0);       // humanoid 5
  _Kd_joint.resize(_model->_nDof-6, 0);       // humanoid 1.5
  //_Kp_joint.resize(_model->_nDof-6, 0.);
  //_Kd_joint.resize(_model->_nDof-6, 0.);
  _wbic_data->_W_rf = DVec<T>::Constant(12, 1.);

  _pold = DVec<T>::Zero(model->_nDof);
  _taudistold = DVec<T>::Zero(model->_nDof);
  _CorMat = DMat<T>::Zero(model->_nDof,model->_nDof);
}

template<typename T>
WBC_Ctrl<T>::~WBC_Ctrl(){
  delete _kin_wbc;
  delete _wbic;
  delete _wbic_data;

  typename std::vector<Task<T> *>::iterator iter = _task_list.begin();
  while (iter < _task_list.end()) {
    delete (*iter);
    ++iter;
  }
  _task_list.clear();

  typename std::vector<ContactSpec<T> *>::iterator iter2 = _contact_list.begin();
  while (iter2 < _contact_list.end()) {
    delete (*iter2);
    ++iter2;
  }
  _contact_list.clear();
}

template <typename T>
void WBC_Ctrl<T>::_ComputeWBC() {
  //Timer timerWBC;
  _kin_wbc->FindConfiguration(_full_config, _task_list, _contact_list,
                              _des_jpos, _des_jvel);
  _wbic->UpdateSetting(_A, _Ainv, _coriolis, _grav);
  _wbic->MakeTorque(_tau_ff, _wbic_data);
  // std::cout<<"tau_ff: "<<_tau_ff.transpose()<<std::endl;
  // std::cout<<"q: "  <<_state.q.transpose()<<std::endl;
  // std::cout<<"qd: " <<_state.qd.transpose()<<std::endl;
 //std::cout<<"_ComputeLineWBC: "<<timerWBC.getMs()<<std::endl;
 // simon added for preventing knee from reaching singularity
 if (_state.q[3]<0.15 && _state.qd[3]<0) {
    _tau_ff[3] = 20;
 }
 if (_state.q[8]<0.15 && _state.qd[8]<0) {
    _tau_ff[8] = 20;
  }


  //T dt(0.002);
  //for(size_t i(0); i<humanoid::num_act_joint; ++i){
    //_des_jvel[i] += _wbic_data->_qddot[i+6]* dt*dt;
    //_des_jpos[i] += _des_jvel[i]*dt + 0.5*_wbic_data->_qddot[i+6]* dt*dt;
  //}
}

template<typename T>
void WBC_Ctrl<T>::run(void* input, ControlFSMData_Tello<T> & data){
  ++_iter;
  // Update Model
  _UpdateModel(data._stateEstimator->getResult(), data);

  // Task & Contact Update
  _ContactTaskUpdate(input, data);

  // WBC Computation
  _ComputeWBC();
  
  // Update Limb Command
  _UpdateJointCMD(data);
  
  // LCM publish
  _LCM_PublishData();

}

template<typename T>
void WBC_Ctrl<T>::_UpdateJointCMD(ControlFSMData_Tello<T> & fsm_data){
    //printf("******************************\n");
  int joint_count = 0;
  for(size_t idx(0); idx < tello::num_joint_group; ++idx){
    for(int jidx(0); jidx < fsm_data._jointController->_commands[idx]->_num_joints;++jidx){
      fsm_data._jointController->_commands[idx]->tauFeedForward[jidx] = _tau_ff[joint_count];
      fsm_data._jointController->_commands[idx]->qDes[jidx] = _des_jpos[joint_count];
      fsm_data._jointController->_commands[idx]->qdDes[jidx] = _des_jvel[joint_count];

      fsm_data._jointController->_commands[idx]->kpJoint(jidx, jidx) = _Kp_joint[joint_count];
      fsm_data._jointController->_commands[idx]->kdJoint(jidx, jidx) = _Kd_joint[joint_count];
      joint_count++;
    }
  }

  //pretty_print(cmd->qDes, std::cout, "q des");
  //pretty_print(cmd->qdDes, std::cout, "qd des");
  //printf("\n");

  //pretty_print(cmd->tauFeedForward, std::cout, "tau feedforward");
  //pretty_print(cmd->kpJoint, std::cout, "Kp Joint");
  //pretty_print(cmd->kdJoint, std::cout, "Kd Joint");
  //pretty_print(_full_config, std::cout, "full config");
}


template<typename T>
void WBC_Ctrl<T>::_UpdateModel(const StateEstimate<T> & state_est, 
    ControlFSMData_Tello<T> & fsm_data){
  _A = _model->getMassMatrix();
  _grav = _model->getGravityForce();
  _coriolis = _model->getCoriolisForce();
  _Ainv = _A.inverse();

  _state.bodyOrientation = state_est.orientation;
  _state.bodyPosition = state_est.position;
  for(size_t i(0); i<3; ++i){
    _state.bodyVelocity[i] = state_est.omegaBody[i];
    _state.bodyVelocity[i+3] = state_est.vBody[i];
  }

  int joint_count = 0;
  for(size_t idx(0); idx < tello::num_joint_group; ++idx){
    for(int jidx(0); jidx < fsm_data._jointController->_datas[idx]->_num_joints;++jidx){
      _state.q[joint_count] = fsm_data._jointController->_datas[idx]->q[jidx];
      _state.qd[joint_count] = fsm_data._jointController->_datas[idx]->qd[jidx];
      _full_config[joint_count+6] = fsm_data._jointController->_datas[idx]->q[jidx];

      if(_iter < 2){ // initial
        _des_jpos[joint_count] = _state.q[joint_count];
        _des_jvel[joint_count] = _state.qd[joint_count];
      }
      joint_count++;
    }
  }
  //pretty_print(_A, std::cout, "A");
}

template class WBC_Ctrl<float>;
template class WBC_Ctrl<double>;
