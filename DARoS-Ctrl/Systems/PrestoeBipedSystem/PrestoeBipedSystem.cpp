#include "PrestoeBipedSystem.hpp"
#include <ParamHandler/ParamHandler.hpp>
#include <Configuration.h>

template <typename T>
PrestoeBipedSystem<T>::PrestoeBipedSystem(const std::string & setup_file): System<T>(setup_file)
{
  _ReadConfig(setup_file);
}

template <typename T>
void PrestoeBipedSystem<T>::runCtrl()
{
  if(_initial_run){
    _initial_run = false;
    _state_ctrl->Initialize(this->_test_state_id);
  }
  _state_ctrl->RunState();
}

template <typename T>
bool PrestoeBipedSystem<T>::Initialization()
{
  // _obs_manager.Initialize();
  // _user_input_manager.Initialize();
  // _vis_manager.Initialize();

  return true;
}
template<typename T>
void PrestoeBipedSystem<T>::_ReadConfig(const std::string & file_name) {
  ParamHandler param_handler(file_name);
  param_handler.getValue("controller_dt", this->_ctrl_dt);
  T input;
  param_handler.getValue("test_state_id", input);
  this->_test_state_id = static_cast<StateList>(input);
}

template class PrestoeBipedSystem<float>;
template class PrestoeBipedSystem<double>;