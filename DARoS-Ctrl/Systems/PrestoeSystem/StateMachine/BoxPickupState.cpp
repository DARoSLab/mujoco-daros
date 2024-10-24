#include "BoxPickupState.hpp"
#include <pretty_print.h>
#include <utilities.h>
#include <ParamHandler/ParamHandler.hpp>
#include <Command.hpp>
#include <PrestoeFBModel.h>

#include <WBC_Prestoe/PrestoeBoxPickupCtrl/PrestoeBoxPickupCtrl.hpp>
#include <PrestoeObsManager.hpp>

template <typename T>
BoxPickupState<T>::BoxPickupState(ObserverManager<T>* obs_manager, PrestoeSystem<T>* prestoe_system):
  _obs_manager(obs_manager),
  State<T>(prestoe_system){

  PrestoeFBModel<T>::buildFBModel(_fb_model, false);
  // PrestoeFBModel<T>::buildFBModel(_fb_model, true);
  _fb_state = _fb_model._state;
  std::string config_file = THIS_COM"/Systems/PrestoeSystem/Configs/box-pickup_state.yaml";

  _ReadConfig(config_file);
  _jtorque_pos_cmd = new JTorquePosCommand<T>(prestoe::num_act_joint);

  _wbc_ctrl = new PrestoeBoxPickupCtrl<T>(&_fb_model, config_file);
  _wbc_data = new PrestoeBoxPickupCtrlData<T>();

  _wbc_ctrl->setFloatingBaseWeight(10000.);
  _ini_jpos = DVec<T>::Zero(prestoe::num_act_joint);

  printf("[Box Pickup State] Constructed\n");
}

template <typename T>
void BoxPickupState<T>::OnEnter() {
  CheaterModeObserver<T>* cheater_mode_obs = 
    dynamic_cast<CheaterModeObserver<T>*>(
      _obs_manager->_observers[PrestoeObsList::CheaterMode]);
  _ini_jpos = cheater_mode_obs->_q.tail(prestoe::num_act_joint);

  _UpdateModel();

  // Initial state
  _ini_body_pos = cheater_mode_obs->_q.head(3); 
  Quat<T> quat = cheater_mode_obs->_q.segment(3,4);
  _ini_body_ori_rpy = quatToRPY(quat);

  _ini_com_pos = _ini_body_pos + _fb_model.getComPosWorld();

  // Hand
  _ini_rhand_pos = _fb_model._pGC[prestoe_contact::rhand];
  _ini_lhand_pos = _fb_model._pGC[prestoe_contact::lhand];

  _mid_pos_cps.setZero();
  for(size_t i(0); i<prestoe_contact::num_foot_contact; ++i){
    // pretty_print(_fb_model._pGC[prestoe_contact::rheel + i], std::cout, "contact pos");
    _mid_pos_cps += _fb_model._pGC[prestoe_contact::rheel + i]/prestoe_contact::num_foot_contact;
  }

  this->_state_time = 0.0;
  _seq_idx = 0;
  _accumulated_seq_time = 0.0;

  _pre_CoM_target = _ini_com_pos;
  _pre_RPY_target = _ini_body_ori_rpy;
  _pre_RHand_target = _ini_rhand_pos;
  _pre_LHand_target = _ini_lhand_pos;

  // pretty_print(_ini_body_ori_rpy, std::cout, "body rpy");
  // pretty_print(_mid_pos_cps, std::cout, "[Box Pick Up] middle of cps");
  printf("[Box Pick Up] On Enter\n");
}

template <typename T>
void BoxPickupState<T>::RunNominal() {
  _UpdateModel();

  if(_RunSequence(_seq_idx, this->_state_time - _accumulated_seq_time, _pre_CoM_target, _pre_RPY_target, _pre_RHand_target, _pre_LHand_target)){
    // If the sequence is done, move to the next sequence
    _accumulated_seq_time += _seq_duration[_seq_idx];

    _pre_RPY_target += _seq_rpy_delta[_seq_idx];
    _pre_CoM_target += _seq_com_delta[_seq_idx];
    _pre_RHand_target += _seq_rhand_delta[_seq_idx];
    _pre_LHand_target += _seq_lhand_delta[_seq_idx];
    ++_seq_idx;
  }

  _UpdateCommand();

  this->_state_time += this->_sys_info._ctrl_dt;
}

template <typename T>
bool BoxPickupState<T>::_RunSequence(size_t idx, T seq_time, const Vec3<T> & com_ini, 
  const Vec3<T> & rpy_ini, const Vec3<T> & rhand_ini, const Vec3<T> & lhand_ini){

  _wbc_data->pBody_RPY_des = smooth_change<T>(rpy_ini, rpy_ini + _seq_rpy_delta[idx], _seq_duration[idx], seq_time);
  _wbc_data->pCoM_des = smooth_change<T>(com_ini, com_ini + _seq_com_delta[idx], _seq_duration[idx], seq_time);
  _wbc_data->rHand_pos_des = smooth_change<T>(rhand_ini, rhand_ini + _seq_rhand_delta[idx], _seq_duration[idx], seq_time);
  _wbc_data->lHand_pos_des = smooth_change<T>(lhand_ini, lhand_ini + _seq_lhand_delta[idx], _seq_duration[idx], seq_time);

  _wbc_data->jpos_des = _ini_jpos;
  for (size_t i(0); i<prestoe_contact::num_foot_contact; ++i) _wbc_data->Fr_des[i].setZero();

  // Run WBC controller
  _wbc_ctrl->run(_wbc_data);

  // If the sequence is not done yet or the sequence index is out of bound, then keep the current sequence 
  if(seq_time <= _seq_duration[idx] || idx >= _seq_duration.size()){ return false; }
  else{ return true; } // If the sequence is done, move to the next sequence
}


template<typename T>
void BoxPickupState<T>::_UpdateModel(){
  CheaterModeObserver<T>* cheater_mode_obs = 
    dynamic_cast<CheaterModeObserver<T>*>(
      _obs_manager->_observers[PrestoeObsList::CheaterMode]);

  _fb_state.bodyPosition = cheater_mode_obs->_q.head(3); 
  _fb_state.bodyOrientation = cheater_mode_obs->_q.segment(3,4); 

  for(size_t i(0); i<3; ++i){
    _fb_state.bodyVelocity[i] = cheater_mode_obs->_dq[i]; 
    _fb_state.bodyVelocity[i+3] = cheater_mode_obs->_dq[i+3]; 
  }

  for(size_t jidx(0); jidx < prestoe::num_act_joint; ++jidx){
    _fb_state.q[jidx] = cheater_mode_obs->_q[jidx+7]; 
    _fb_state.qd[jidx] = cheater_mode_obs->_dq[jidx+6]; 
  }

  // pretty_print(cheater_mode_obs->_q, std::cout, "q");
  // pretty_print(_fb_state.bodyPosition, std::cout, "body position");
  // pretty_print(_fb_state.bodyOrientation, std::cout, "body orientation");
  // pretty_print(_fb_state.bodyVelocity, std::cout, "body velocity");
  // pretty_print(_fb_state.q, std::cout, "joint positions");
  // pretty_print(_fb_state.qd, std::cout, "joint velocities");

  _fb_model.setState(_fb_state);
  _fb_model.contactJacobians();
  _fb_model.FullcontactJacobians();
  _fb_model.massMatrix();//Comment if using massandCoriolisMatrix()
  _fb_model.generalizedGravityForce();
  _fb_model.generalizedCoriolisForce();
  _fb_model.centroidMomentumMatrix();
  _fb_model.massandCoriolisMatrix();//Comment if not needed

  // DMat<T> A = _fb_model.getMassMatrix();
  // std::cout<<A<<std::endl;
  // std::cout<<A.block(0,0, 6,6)<<std::endl;
}

template<typename T>
void BoxPickupState<T>::_ReadConfig(const std::string & file_name) {
  ParamHandler param_handler(file_name);

  param_handler.getEigenVec("Kp", _Kp);
  param_handler.getEigenVec("Kd", _Kd);

  param_handler.getEigenVec("seq_duration", _seq_duration);
  param_handler.getVec3Sequence("seq_CoM_delta", _seq_com_delta);
  param_handler.getVec3Sequence("seq_RPY_delta", _seq_rpy_delta);
  param_handler.getVec3Sequence("seq_RHand_delta", _seq_rhand_delta);
  param_handler.getVec3Sequence("seq_LHand_delta", _seq_lhand_delta);

  // print sequence
  // for(size_t i(0); i<_seq_duration.size(); ++i){
  //   printf("seq duration: %f\n", _seq_duration[i]);
  //   pretty_print(_seq_com_delta[i], std::cout, "seq CoM delta");
  //   pretty_print(_seq_rpy_delta[i], std::cout, "seq RPY delta");
  //   pretty_print(_seq_rhand_delta[i], std::cout, "seq RHand delta");
  //   pretty_print(_seq_lhand_delta[i], std::cout, "seq LHand delta");
  // }
}

template<typename T>
void BoxPickupState<T>::_UpdateCommand(){
  CheaterModeObserver<T>* cheater_mode_obs = 
    dynamic_cast<CheaterModeObserver<T>*>(
      _obs_manager->_observers[PrestoeObsList::CheaterMode]);

  JTorquePosCommand<T>* cmd = dynamic_cast<JTorquePosCommand<T>*>(_jtorque_pos_cmd);

  cmd->SetJointCommand(_wbc_ctrl->_tau_ff, 
      _wbc_ctrl->_des_jpos, _wbc_ctrl->_des_jvel, _Kp, _Kd);
}
template class BoxPickupState<float>;
template class BoxPickupState<double>;
