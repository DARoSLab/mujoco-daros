#ifndef FSM_STATE_SIMPLE_JOINTPD_H
#define FSM_STATE_SIMPLE_JOINTPD_H

#include <FSM_State.h>

template <typename T>
class FSM_State_SimpleJointPD : public FSM_State<T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FSM_State_SimpleJointPD(ControlFSMData<T>* _controlFSMData);

  // Behavior to be carried out when entering a state
  void onEnter();

  // Run the normal behavior for the state
  void run();

  // Behavior to be carried out when exiting a state
  void onExit();

 private:
  // Keep track of the control iterations
  int _iter = 0;
  DVec<T> _ini_jpos;

};

#endif  // FSM_STATE_JOINTPD_H
