#include "MiniArmSystem.hpp"
#include <ParamHandler/ParamHandler.hpp>
#include <Configuration.h>

template <typename T>
MiniArmSystem<T>::MiniArmSystem(const std::string & setup_file): System<T>(setup_file)
{
  _ReadConfig(setup_file);
}

template <typename T>
void MiniArmSystem<T>::runCtrl()
{
  _state_ctrl->RunState();
}

template<typename T>
void MiniArmSystem<T>::_ReadConfig(const std::string & file_name) {
  ParamHandler param_handler(file_name);
  param_handler.getValue("controller_dt", this->_ctrl_dt);
  std::string mujoco_file_name;
  param_handler.getString("Mujoco_XML_FileName", mujoco_file_name);
  this->_mujoco_file = THIS_COM"/Systems/MiniArmSystem/Robot/" + mujoco_file_name;
}

template class MiniArmSystem<float>;
template class MiniArmSystem<double>;