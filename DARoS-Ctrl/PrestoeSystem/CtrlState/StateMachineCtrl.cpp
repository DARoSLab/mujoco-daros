#include "StateMachineCtrl.hpp"
#include "JPosCtrlState.hpp"

template <typename T>
StateMachineCtrl<T>::StateMachineCtrl(ObserverManager<T>* obs_manager):
_obs_manager(obs_manager)
{
  // Initialize and add all of the FSM States to the state list
  _state_list.resize(StateList::NUM_STATE);
  _state_list[StateList::JOINT_PD] = new JPosCtrlState<T>(obs_manager);
  
  printf("[State Machine Control] Constructed\n");
  _Initialize();
}

template <typename T>
void StateMachineCtrl<T>::_Initialize() {
  // Initialize a new FSM State with the Passive FSM State
  // because this function called before the system state is updated
  _curr_State = _state_list[StateList::JOINT_PD];
  // Initialize to not be in transition
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
