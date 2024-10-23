#include "WBC_Ctrl.hpp"
#include <Utilities/pretty_print.h>
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

  _Kp_joint.resize(_model->_nDof-6, 10);
  _Kd_joint.resize(_model->_nDof-6, 0.5);
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
 //std::cout<<"_ComputeLineWBC: "<<timerWBC.getMs()<<std::endl;
  // std::cout<<"_qddot: "<<_wbic_data->_qddot.transpose()<<std::endl;
  // std::cout<<"_Fr: "<<_wbic_data->_Fr.transpose()<<std::endl;
}

template<typename T>
void WBC_Ctrl<T>::run(void* input, ControlFSMData_Staccatoe<T> & data){
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
void WBC_Ctrl<T>::_UpdateJointCMD(ControlFSMData_Staccatoe<T> & fsm_data){
  for(size_t jidx(0); jidx < staccatoe::num_act_joint; ++jidx){
    fsm_data._jointController->_command.tauFeedForward[jidx] = _tau_ff[jidx];
    fsm_data._jointController->_command.qDes[jidx] = _des_jpos[jidx];
    fsm_data._jointController->_command.qdDes[jidx] = _des_jvel[jidx];

    fsm_data._jointController->_command.kpJoint(jidx) = _Kp_joint[jidx];
    fsm_data._jointController->_command.kdJoint(jidx) = _Kd_joint[jidx];
  }

  // pretty_print(fsm_data._jointController->_command.qDes, std::cout, "q des");
  // pretty_print(fsm_data._jointController->_command.qdDes, std::cout, "qd des");
  // printf("\n");

  // pretty_print(fsm_data._jointController->_command.tauFeedForward, std::cout, "tau feedforward");
  // pretty_print(fsm_data._jointController->_command.kpJoint, std::cout, "Kp Joint");
  // pretty_print(fsm_data._jointController->_command.kdJoint, std::cout, "Kd Joint");
  // pretty_print(_full_config, std::cout, "full config");
}


template<typename T>
void WBC_Ctrl<T>::_UpdateModel(const StateEstimate<T> & state_est, 
    ControlFSMData_Staccatoe<T> & fsm_data){

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

  for(size_t jidx(0); jidx < staccatoe::num_act_joint; ++jidx){
    _state.q[jidx] = fsm_data._jointController->_data.q[jidx];
    _state.qd[jidx] = fsm_data._jointController->_data.qd[jidx];
    _full_config[jidx+6] = fsm_data._jointController->_data.q[jidx];

    if(_iter < 2){ // initial
      _des_jpos[jidx] = _state.q[jidx];
      _des_jvel[jidx] = _state.qd[jidx];
    }
  }
  //pretty_print(_A, std::cout, "A");
}

template class WBC_Ctrl<float>;
template class WBC_Ctrl<double>;
