#ifndef FSM_STATE_PASSIVE_H
#define FSM_STATE_PASSIVE_H

#include <FSM_State.h>

/**
 *
 */
template <typename T>
class FSM_State_Passive : public FSM_State<T> {
 public:
  FSM_State_Passive(ControlFSMData<T>* _controlFSMData);

  // Behavior to be carried out when entering a state
  void onEnter();

  // Run the normal behavior for the state
  void run();

  // Behavior to be carried out when exiting a state
  void onExit();

 private:
  // Keep track of the control iterations
  int iter = 0;
  Vec3<float> pFeetVec;
};

#endif  // FSM_STATE_PASSIVE_H
