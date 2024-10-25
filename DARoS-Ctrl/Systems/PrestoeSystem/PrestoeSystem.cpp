#include "PrestoeSystem.hpp"
#include <ParamHandler/ParamHandler.hpp>
#include <Configuration.h>

template <typename T>
PrestoeSystem<T>::PrestoeSystem(const std::string & setup_file): System<T>(setup_file)
{
  _ReadConfig(setup_file);
  _initialized = Initialization();
}

template <typename T>
void PrestoeSystem<T>::runCtrl()
{
  _state_ctrl->RunState();
}

template <typename T>
bool PrestoeSystem<T>::Initialization()
{
  // _obs_manager.Initialize();
  // _user_input_manager.Initialize();
  // _vis_manager.Initialize();

  return true;
}
template<typename T>
void PrestoeSystem<T>::_ReadConfig(const std::string & file_name) {
  ParamHandler param_handler(file_name);
  param_handler.getValue("controller_dt", this->_ctrl_dt);
  std::string mujoco_file_name;
  param_handler.getString("Mujoco_XML_FileName", mujoco_file_name);
  this->_mujoco_file = THIS_COM"/Systems/PrestoeSystem/Robot/" + mujoco_file_name;
}

template class PrestoeSystem<float>;
template class PrestoeSystem<double>;