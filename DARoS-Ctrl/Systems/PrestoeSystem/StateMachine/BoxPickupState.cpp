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

  _ini_com = _ini_body_pos + _fb_model.getComPosWorld();

  _mid_pos_cps.setZero();
  for(size_t i(0); i<prestoe_contact::num_foot_contact; ++i){
    pretty_print(_fb_model._pGC[prestoe_contact::rheel + i], std::cout, "contact pos");
    _mid_pos_cps += _fb_model._pGC[prestoe_contact::rheel + i]/prestoe_contact::num_foot_contact;
  }

  this->_state_time = 0.0;

  // pretty_print(_ini_body_ori_rpy, std::cout, "body rpy");
  pretty_print(_mid_pos_cps, std::cout, "[Box Pick Up] middle of cps");
  printf("[Box Pick Up] On Enter\n");
}

template <typename T>
void BoxPickupState<T>::RunNominal() {
  _UpdateModel();
  _KeepPostureStep();
  _UpdateCommand();
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

template <typename T>
void BoxPickupState<T>::_KeepPostureStep() {
    this->_state_time += this->_sys_info._ctrl_dt;
    T curr_time = this->_state_time;

    _wbc_data->jpos_des = _ini_jpos;
    //setting orientation targets
    _wbc_data->pBody_RPY_des.setZero();
    _wbc_data->pBody_RPY_des[1] = _ini_body_ori_rpy[1];

    //setting COM target
    _wbc_data->pCoM_des = _mid_pos_cps;
    _wbc_data->pCoM_des[2] = _ini_com[2];

    for (size_t i(0); i<prestoe_contact::num_foot_contact; ++i) _wbc_data->Fr_des[i].setZero();

    _wbc_ctrl->run(_wbc_data);
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
}


template class BoxPickupState<float>;
template class BoxPickupState<double>;