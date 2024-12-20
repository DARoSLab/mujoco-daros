#include "JPosCtrlState.hpp"
#include <pretty_print.h>
#include <ParamHandler/ParamHandler.hpp>
#include <MiniArmSystem.hpp>
#include <MiniArmObsManager.hpp>
#include <Command.hpp>

template <typename T>
JPosCtrlState<T>::JPosCtrlState(ObserverManager<T>* ob_man, MiniArmSystem<T>* sys):
_obs_manager(ob_man),
State<T>(sys){
  _ReadConfig(THIS_COM"/Systems/MiniArmSystem/Configs/jpos_state.yaml");
  _jpos_cmd = new JPosCommand<T>(miniarm::num_act_joint);
  printf("[JPosCtrlState] Constructed\n");
}

template <typename T>
void JPosCtrlState<T>::OnEnter() {
  CheaterModeObserver<T>* cheater_mode_obs = 
    dynamic_cast<CheaterModeObserver<T>*>(
      _obs_manager->_observers[MiniArmObsList::CheaterMode]);

  this->_jpos_ini = cheater_mode_obs->_q.tail(miniarm::num_act_joint);
  this->_state_time = 0.0;
  pretty_print(_jpos_ini, std::cout, "JPosCtrlState: _jpos_ini");
}

template <typename T>
void JPosCtrlState<T>::RunNominal() {
  T curr_time = this->_state_time;
  T init_move_duration = 3.0; // 3 seconds
  DVec<T> target_jpos;
  DVec<T> target_vel;

  if(curr_time < init_move_duration) {
    // Move to the default joint position
    target_jpos = (1.0 - curr_time / init_move_duration) * this->_jpos_ini + 
                    (curr_time / init_move_duration) * this->_jpos_default;
    target_vel = DVec<T>::Zero(miniarm::num_act_joint);

  }else{
    T swing_time = curr_time - init_move_duration;
    DVec<T> sin_vec = (2.0*M_PI*_swing_freq * swing_time).array().sin();
    DVec<T> cos_vec = (2.0*M_PI*_swing_freq * swing_time).array().cos();
    target_jpos = _jpos_default + _swing_amp.cwiseProduct(sin_vec);
    target_vel = (2.0*M_PI*_swing_amp).cwiseProduct(cos_vec);
  }
  CheaterModeObserver<T>* cheater_mode_obs =
   dynamic_cast<CheaterModeObserver<T>*>(_obs_manager->_observers[MiniArmObsList::CheaterMode]);
  JPosCommand<T>* jpos_cmd = dynamic_cast<JPosCommand<T>*>(_jpos_cmd);

  jpos_cmd->SetJointCommand(target_jpos, target_vel, _Kp, _Kd);

  this->_state_time += this->_sys_info._ctrl_dt;
}

template <typename T>
void JPosCtrlState<T>::_ReadConfig(const std::string & file_name) {
  ParamHandler param_handler(file_name);
  param_handler.getEigenVec("jpos_default", _jpos_default);
  param_handler.getEigenVec("swing_freq", _swing_freq);
  param_handler.getEigenVec("swing_amp", _swing_amp);
  param_handler.getEigenVec("Kp", _Kp);
  param_handler.getEigenVec("Kd", _Kd);
}

template class JPosCtrlState<float>;
template class JPosCtrlState<double>;