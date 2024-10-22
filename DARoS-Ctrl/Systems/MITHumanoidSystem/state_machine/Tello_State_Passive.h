#ifndef FSM_STATE_Tello_PASSIVE_H
#define FSM_STATE_Tello_PASSIVE_H

#include "Tello_State.h"

/**
 *
 */
template <typename T>
class Tello_State_Passive : public Tello_State<T> {
 public:
  Tello_State_Passive(ControlFSMData_Tello<T>* _controlFSMData);

  // Behavior to be carried out when entering a state
  void onEnter();

  // Run the normal behavior for the state
  void run();

  // Checks for any transition triggers
  Tello_StateName checkTransition();

  // Manages state specific transitions
  TransitionData_Tello<T> transition();

  // Behavior to be carried out when exiting a state
  void onExit();

 private:
  // Keep track of the control iterations
  int iter = 0;
};

#endif  // FSM_STATE_PASSIVE_H
