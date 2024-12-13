#include "WBC_Ctrl.hpp"
#include <pretty_print.h>
#include <Timer.h>
#include <utilities.h>

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
  _state = &_model->_state;

  _kin_wbc = new KinWBC<T>(model->_nDof);

  _wbic = new WBIC<T>(model->_nDof, &(_contact_list), &(_task_list));
  _wbic_data = new WBIC_ExtraData<T>();

  _wbic_data->_W_floating = DVec<T>::Constant(6, 0.1);
  _wbic_data->_W_rf = DVec<T>::Constant(12, 1.);

  _Kp_joint.resize(_model->_nDof-6, 10);
  _Kd_joint.resize(_model->_nDof-6, 0.5);
  //_Kp_joint.resize(_model->_nDof-6, 0.);
  //_Kd_joint.resize(_model->_nDof-6, 0.);

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
//  std::cout<<"_ComputeLineWBC: "<<timerWBC.getMs()<<std::endl;
  // std::cout<<"_qddot: "<<_wbic_data->_qddot.transpose()<<std::endl;
  // std::cout<<"_Fr: "<<_wbic_data->_Fr.transpose()<<std::endl;
}

template<typename T>
void WBC_Ctrl<T>::run(void* input){
  ++_iter;
  // Assume that fb model is updated by state machine 
  _UpdateParams();

  // Task & Contact Update
  _ContactTaskUpdate(input);

  // WBC Computation
  _ComputeWBC();
  
  // LCM publish
  _LCM_PublishData();
}

template<typename T>
void WBC_Ctrl<T>::_UpdateParams(){

  _A = _model->getMassMatrix();
  _grav = _model->getGravityForce();
  _coriolis = _model->getCoriolisForce();
  _Ainv = _A.inverse();

  for(size_t jidx(0); jidx < prestoe::num_act_joint; ++jidx){
    _full_config[jidx+6] = _state->q[jidx];
  }
  //pretty_print(_A, std::cout, "A");
}

template class WBC_Ctrl<float>;
template class WBC_Ctrl<double>;
