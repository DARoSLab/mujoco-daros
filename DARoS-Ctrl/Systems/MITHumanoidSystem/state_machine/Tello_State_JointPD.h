#ifndef Tello_STATE_JOINTPD_H
#define Tello_STATE_JOINTPD_H

#include <Tello_State.h>

/**
 *
 */
template <typename T>
class Tello_State_JointPD : public Tello_State<T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Tello_State_JointPD(ControlFSMData_Tello<T>* _controlFSMData);

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
  DVec<T> _ini_jpos;
};

#endif  // Tello_STATE_JOINTPD_H
