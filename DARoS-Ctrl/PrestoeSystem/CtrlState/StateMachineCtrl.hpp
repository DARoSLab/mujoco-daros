#ifndef __STATE_MACHINE_CTRL_H__
#define __STATE_MACHINE_CTRL_H__

#include <iostream>

enum StateList {
  PASSIVE = 0,
  JOINT_PD = 1, 
  BALANCE_STAND = 2,
  BOX_PICKUP = 3,
  NUM_STATE
};

template <typename T> class ObserverManager;
template <typename T> class PrestoeSystem;
template <typename T> class State;

template <typename T>
class StateMachineCtrl{
  public:
    // Initialize for Quadruped
    StateMachineCtrl(ObserverManager<T>* obs_manager, 
                     PrestoeSystem<T>* prestoe_system); 
    void RunState();
    void _Initialize();

    int iter = 0;
    std::vector<State<T> *> _state_list;
    State<T>* _curr_State;    // current FSM state
    State<T>* _next_State;       // next FSM state

    bool _b_first_visit = true;
};

#endif  // CONTROLFSM_H
