#include "PrestoeSystem.hpp"

template <typename T>
PrestoeSystem<T>::PrestoeSystem(const std::string & setup_file): System<T>(setup_file)
{
  _initialized = Initialization();
}

template <typename T>
void PrestoeSystem<T>::runCtrl()
{
}

template <typename T>
bool PrestoeSystem<T>::Initialization()
{
  // _obs_manager.Initialize();
  // _user_input_manager.Initialize();
  // _vis_manager.Initialize();

  return true;
}


template class PrestoeSystem<float>;
template class PrestoeSystem<double>;