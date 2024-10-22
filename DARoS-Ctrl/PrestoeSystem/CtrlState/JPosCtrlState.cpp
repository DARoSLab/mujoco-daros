#include "JPosCtrlState.hpp"
#include <pretty_print.h>

template <typename T>
JPosCtrlState<T>::JPosCtrlState(ObserverManager<T>* ob_man):
_obs_manager(ob_man)
{
}

template <typename T>
void JPosCtrlState<T>::Initialize() {
}

template <typename T>
void JPosCtrlState<T>::OnEnter() {
  CheaterModeObserver<T>* cheater_mode_obs = 
    dynamic_cast<CheaterModeObserver<T>*>(
      _obs_manager->_observers[PrestoeObsList::CheaterMode]);

  this->_jpos_ini = cheater_mode_obs->_q.tail(prestoe::num_act_joint);
  pretty_print(_jpos_ini, std::cout, "JPosCtrlState: _jpos_ini");
}

template <typename T>
void JPosCtrlState<T>::RunNominal() {
  // std::cout << "[JPosCtrlState] RunNominal" << std::endl;
}

template class JPosCtrlState<float>;
template class JPosCtrlState<double>;