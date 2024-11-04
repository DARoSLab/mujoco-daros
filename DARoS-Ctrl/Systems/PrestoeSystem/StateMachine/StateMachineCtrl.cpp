#include "StateMachineCtrl.hpp"
#include <PrestoeObsManager.hpp>
#include "JPosCtrlState.hpp"
#include "BalanceStandState.hpp"
#include "BoxPickupState.hpp"

template <typename T>
StateMachineCtrl<T>::StateMachineCtrl(ObserverManager<T>* obs_manager, PrestoeSystem<T>* sys) {
  // Initialize and add all of the FSM States to the state list
  _state_list.resize(StateList::NUM_STATE);
  _state_list[StateList::JOINT_PD] = new JPosCtrlState<T>(obs_manager, sys);
  _state_list[StateList::BALANCE_STAND] = new BalanceStandState<T>(obs_manager, sys);
  _state_list[StateList::BOX_PICKUP] = new BoxPickupState<T>(obs_manager, sys);
  
  printf("[State Machine Control] Constructed\n");
}

template <typename T>
void StateMachineCtrl<T>::Initialize(StateList test_state) {
  // Initialize a new FSM State with the Passive FSM State
  // because this function called before the system state is updated
  _curr_State = _state_list[test_state];

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
