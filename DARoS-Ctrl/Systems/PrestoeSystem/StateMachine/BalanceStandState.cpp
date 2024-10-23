#include "BalanceStandState.hpp"
#include <pretty_print.h>
#include <ParamHandler/ParamHandler.hpp>
#include <PrestoeObsManager.hpp>
#include <Command.hpp>
#include <WBC_Prestoe/PrestoeStandCtrl/PrestoeStandCtrl.hpp>
#
template <typename T>
BalanceStandState<T>::BalanceStandState(ObserverManager<T>* obs_manager, PrestoeSystem<T>* prestoe_system):
  _obs_manager(obs_manager),
  State<T>(prestoe_system){

  _ReadConfig(THIS_COM"/PrestoeSystem/Configs/standing_state.yaml");
  _jtorque_cmd = new JTorqueCommand<T>(prestoe::num_act_joint);

  _wbc_ctrl = new PrestoeStandCtrl<T>(this->_model);
  _wbc_data = new PrestoeStandCtrlData<T>();

  _wbc_ctrl->setFloatingBaseWeight(10000.);
  _ini_jpos = DVec<T>::Zero(prestoe::num_act_joint);

  buildFloatingBaseModelFromURDF(_fb_model, 
    THIS_COM"/Systems/PrestoeSystem/Robot/prestoe.urdf", false);
  printf("[Balance Stand State] Constructed\n");
}

template <typename T>
void BalanceStandState<T>::OnEnter() {
  CheaterModeObserver<T>* cheater_mode_obs = 
    dynamic_cast<CheaterModeObserver<T>*>(
      _obs_manager->_observers[PrestoeObsList::CheaterMode]);
  _ini_jpos = cheater_mode_obs->_q.tail(prestoe::num_act_joint);

  _UpdateModel();

  // Initial state
  _ini_body_pos = (this->_fsm_data->_stateEstimator->getResult()).position;
  _ini_com_pos = (this->_fsm_data->_stateEstimator->getResult()).position + this->_model->getComPosWorld();
  _ini_body_ori_rpy = (this->_fsm_data->_stateEstimator->getResult()).rpy;
  _body_weight = this->_fsm_data->_staccatoe_model->massMatrix()(3,3)*9.81;

  _mid_pos_cps.setZero();

  for(size_t i(0); i<staccatoe_contact::num_foot_contact; ++i){
    _mid_pos_cps += 
      this->_model->_pGC[staccatoe_contact::heel + i]/staccatoe_contact::num_foot_contact;
  }

  this->_state_time = 0.0;

  pretty_print(_ini_body_ori_rpy, std::cout, "body rpy");
  pretty_print(_ini_com_pos, std::cout, "[Balance Stand] ini com pos");
  pretty_print(_mid_pos_cps, std::cout, "[Balance Stand] middle of cps");
  std::cout << "[Balance Stand] On Enter" << std::endl;
}

template <typename T>
void BalanceStandState<T>::RunNominal() {
  _UpdateModel();
  _KeepPostureStep();
}

template <typename T>
void BalanceStandState<T>::_KeepPostureStep() {
    double targetHeight(0.5);
    this->_state_time += this->_sys_info._ctrl_dt;
    T curr_time = this->_state_time;
    T amp(0.02);
    T freq(0.2);
    static auto iniTime=high_resolution_clock::now();
    auto curTime=high_resolution_clock::now();
    double standingDuration= 5;
    auto curTimeSec =duration_cast<duration<double>>(curTime-iniTime);

    //joints and orientation initialization
    _wbc_data->pBody_RPY_des.setZero();
    _wbc_data->jpos_des = _ini_jpos;
    //setting orientation targets
    _wbc_data->pBody_RPY_des[1] = _ini_body_ori_rpy[1];
    // _wbc_data->pBody_RPY_des[0] = smooth_change((double)_ini_body_ori_rpy[0],(double)0,1.,curTimeSec.count());
    // _wbc_data->pBody_RPY_des[1] = smooth_change((double)_ini_body_ori_rpy[1],(double)0.3,2.,curTimeSec.count());
    // _wbc_data->pBody_RPY_des[2] = smooth_change((double)_ini_body_ori_rpy[2],(double)0,1.,curTimeSec.count());

    //updating the middle of contact points (optional?)
    // for (size_t i(0); i < staccatoe_contact::num_foot_contact; ++i)
    // {
    // _mid_pos_cps +=
    //     this->_model->_pGC[staccatoe_contact::heel + i] / staccatoe_contact::num_foot_contact;
    // }

    //setting COM targets
    _wbc_data->pBody_des = _mid_pos_cps;
    // _wbc_data->pBody_des = _ini_com_pos;

    // _wbc_data->pBody_des[0] =smooth_change((double)_ini_com_pos[0],(double)_mid_pos_cps[0],1.,curTimeSec.count());
    // _wbc_data->pBody_des[1] = smooth_change((double)_ini_com_pos[1],(double)_mid_pos_cps[1],1.,curTimeSec.count());

    //stand up to a target height
    //  auto _height=smooth_change((double)_ini_body_pos[2],targetHeight,standingDuration,curTimeSec.count());
    _wbc_data->pBody_des[2] = _ini_body_pos[2];


    _wbc_data->vBody_des.setZero();
    _wbc_data->aBody_des.setZero();
    _wbc_data->vBody_Ori_des.setZero();
   

    for (size_t i(0); i < staccatoe_contact::num_foot_contact; ++i)
    {
      _wbc_data->Fr_des[i].setZero();
      _wbc_data->Fr_des[i][2] = _body_weight / staccatoe_contact::num_foot_contact;
    }
    _wbc_ctrl->run(_wbc_data, *this->_fsm_data);

}

template<typename T>
void BalanceStandState<T>::_UpdateModel(){
  CheaterModeObserver<T>* cheater_mode_obs = 
    dynamic_cast<CheaterModeObserver<T>*>(
      _obs_manager->_observers[PrestoeObsList::CheaterMode]);

    
  cheater_mode_obs->_q;

  fb_state.bodyPosition = state_est.position;
  fb_state.bodyOrientation = state_est.orientation;

  for(size_t i(0); i<3; ++i){
    fb_state.bodyVelocity[i] = state_est.omegaBody[i];
    fb_state.bodyVelocity[i+3] = state_est.vBody[i];
  }

  for(size_t jidx(0); jidx < staccatoe::num_act_joint; ++jidx){
    fb_state.q[jidx] = _fsm_data->_jointController->_data.q[jidx];
    fb_state.qd[jidx] = _fsm_data->_jointController->_data.qd[jidx];
  }

  // pretty_print(fb_state.bodyPosition, std::cout, "body position");
  // pretty_print(fb_state.bodyOrientation, std::cout, "body orientation");
  // pretty_print(fb_state.bodyVelocity, std::cout, "body velocity");
  // pretty_print(fb_state.q, std::cout, "joint orientations");
  // pretty_print(fb_state.qd, std::cout, "joint velocities");

  _model->setState(fb_state);
  _model->contactJacobians();
  _model->FullcontactJacobians();
  _model->massMatrix();//Comment if using massandCoriolisMatrix()
  _model->generalizedGravityForce();
  _model->generalizedCoriolisForce();
  _model->centroidMomentumMatrix();
  _model->massandCoriolisMatrix();//Comment if not needed

  //DMat<T> A = _model->getMassMatrix();
  //std::cout<<A.block(0,0, 6,6)<<std::endl;

}
template class BalanceStandState<float>;
template class BalanceStandState<double>;
