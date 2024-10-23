#include "BalanceStandState.hpp"
#include <pretty_print.h>
#include <utilities.h>
#include <ParamHandler/ParamHandler.hpp>
#include <Command.hpp>
#include <PrestoeFBModel.h>

#include <WBC_Prestoe/PrestoeStandCtrl/PrestoeStandCtrl.hpp>
#include <PrestoeObsManager.hpp>

template <typename T>
BalanceStandState<T>::BalanceStandState(ObserverManager<T>* obs_manager, PrestoeSystem<T>* prestoe_system):
  _obs_manager(obs_manager),
  State<T>(prestoe_system){

  Prestoe<T> dummy;
  dummy.buildFBModel(_fb_model, true);
  _fb_state = _fb_model._state;

  _ReadConfig(THIS_COM"/PrestoeSystem/Configs/standing_state.yaml");
  _jtorque_pos_cmd = new JTorquePosCommand<T>(prestoe::num_act_joint);

  _wbc_ctrl = new PrestoeStandCtrl<T>(&_fb_model, THIS_COM"/PrestoeSystem/Configs/standing_ctrl.yaml");
  _wbc_data = new PrestoeStandCtrlData<T>();

  _wbc_ctrl->setFloatingBaseWeight(10000.);
  _ini_jpos = DVec<T>::Zero(prestoe::num_act_joint);

  printf("[Balance Stand State] Constructed\n");
}

template <typename T>
void BalanceStandState<T>::OnEnter() {
  printf("[Balance Stand] On Enter begin\n");
  CheaterModeObserver<T>* cheater_mode_obs = 
    dynamic_cast<CheaterModeObserver<T>*>(
      _obs_manager->_observers[PrestoeObsList::CheaterMode]);
  _ini_jpos = cheater_mode_obs->_q.tail(prestoe::num_act_joint);

  _UpdateModel();

  // Initial state
  _ini_body_pos = cheater_mode_obs->_q.head(3); 
  Quat<T> quat = cheater_mode_obs->_q.segment(3,4);
  _ini_body_ori_rpy = quatToRPY(quat);

  _mid_pos_cps.setZero();

  std::cout<<_fb_model._pGC.size()<<std::endl;
  for(size_t i(0); i<prestoe_contact::num_foot_contact; ++i){
    printf("in\n");
    pretty_print(_fb_model._pGC[prestoe_contact::rheel + i], std::cout, "contact pos");
    _mid_pos_cps += _fb_model._pGC[prestoe_contact::rheel + i]/prestoe_contact::num_foot_contact;
  }
printf("1\n");

  this->_state_time = 0.0;

  pretty_print(_ini_body_ori_rpy, std::cout, "body rpy");
  pretty_print(_mid_pos_cps, std::cout, "[Balance Stand] middle of cps");
  std::cout << "[Balance Stand] On Enter is done" << std::endl;
}

template <typename T>
void BalanceStandState<T>::RunNominal() {
  _UpdateModel();
  _KeepPostureStep();
  _UpdateCommand();
}

template<typename T>
void BalanceStandState<T>::_UpdateCommand(){
  CheaterModeObserver<T>* cheater_mode_obs = 
    dynamic_cast<CheaterModeObserver<T>*>(
      _obs_manager->_observers[PrestoeObsList::CheaterMode]);

  JTorquePosCommand<T>* cmd = dynamic_cast<JTorquePosCommand<T>*>(_jtorque_pos_cmd);

  cmd->SetJointCommand(_wbc_ctrl->_tau_ff, 
      _wbc_ctrl->_des_jpos, _wbc_ctrl->_des_jvel, _Kp, _Kd);
}

template <typename T>
void BalanceStandState<T>::_KeepPostureStep() {
    this->_state_time += this->_sys_info._ctrl_dt;
    T curr_time = this->_state_time;

    T standingDuration= 5;

    //joints and orientation initialization
    _wbc_data->pBody_RPY_des.setZero();
    _wbc_data->jpos_des = _ini_jpos;
    //setting orientation targets
    // _wbc_data->pBody_RPY_des[1] = _ini_body_ori_rpy[1];
    T ini_pitch = _ini_body_ori_rpy[1];
    T target_pitch = 0.;
    _wbc_data->pBody_RPY_des[1] = smooth_change(ini_pitch, target_pitch, standingDuration, curr_time);

    //setting COM targets
    _wbc_data->pBody_des = _mid_pos_cps;

    //stand up to a target height
    // _wbc_data->pBody_des[2] = _ini_body_pos[2];
    _wbc_data->pBody_des[2] = smooth_change(_ini_body_pos[2], _targetHeight, standingDuration, curr_time);

    _wbc_data->vBody_des.setZero();
    _wbc_data->aBody_des.setZero();
    _wbc_data->vBody_Ori_des.setZero();

    for (size_t i(0); i<prestoe_contact::num_foot_contact; ++i) _wbc_data->Fr_des[i].setZero();

    _wbc_ctrl->run(_wbc_data);

}

template<typename T>
void BalanceStandState<T>::_UpdateModel(){
  printf("[Balance Stand] Update Model\n");
  CheaterModeObserver<T>* cheater_mode_obs = 
    dynamic_cast<CheaterModeObserver<T>*>(
      _obs_manager->_observers[PrestoeObsList::CheaterMode]);

  pretty_print(cheater_mode_obs->_q, std::cout, "q");
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

  pretty_print(_fb_state.bodyPosition, std::cout, "body position");
  pretty_print(_fb_state.bodyOrientation, std::cout, "body orientation");
  pretty_print(_fb_state.bodyVelocity, std::cout, "body velocity");
  pretty_print(_fb_state.q, std::cout, "joint positions");
  pretty_print(_fb_state.qd, std::cout, "joint velocities");

  _fb_model.setState(_fb_state);
  _fb_model.contactJacobians();
  _fb_model.FullcontactJacobians();
  _fb_model.massMatrix();//Comment if using massandCoriolisMatrix()
  _fb_model.generalizedGravityForce();
  _fb_model.generalizedCoriolisForce();
  _fb_model.centroidMomentumMatrix();
  _fb_model.massandCoriolisMatrix();//Comment if not needed

  DMat<T> A = _fb_model.getMassMatrix();
  std::cout<<A.block(0,0, 6,6)<<std::endl;
}

template<typename T>
void BalanceStandState<T>::_ReadConfig(const std::string & file_name) {
  ParamHandler param_handler(file_name);
  param_handler.getValue("target_height", _targetHeight);
  param_handler.getEigenVec("Kp", _Kp);
  param_handler.getEigenVec("Kd", _Kd);
}


template class BalanceStandState<float>;
template class BalanceStandState<double>;
