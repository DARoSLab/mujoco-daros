#include "StateMachineCtrl.hpp"
#include <MiniArmObsManager.hpp>
#include "JPosCtrlState.hpp"

template <typename T>
StateMachineCtrl<T>::StateMachineCtrl(ObserverManager<T>* obs_manager, MiniArmSystem<T>* sys) {
  // Initialize and add all of the FSM States to the state list
  _state_list.resize(StateList::NUM_STATE);
  _state_list[StateList::JOINT_PD] = new JPosCtrlState<T>(obs_manager, sys);
  
  printf("[State Machine Control] Constructed\n");
  _Initialize();
}

template <typename T>
void StateMachineCtrl<T>::_Initialize() {
  _curr_State = _state_list[StateList::JOINT_PD];
  _next_State = _curr_State;
}

template <typename T>
void StateMachineCtrl<T>::RunState() {
  if(_b_first_visit) {
    _curr_State->OnEnter();
    _b_first_visit = false;
  }
  _curr_State->RunNominal();
}

template class StateMachineCtrl<float>;
template class StateMachineCtrl<double>;
