#ifndef FSM_STATE_STEPPING_H
#define FSM_STATE_STEPPING_H

#include <FSM_State.h>

template<typename T> class WBC_Ctrl;
template<typename T> class LocomotionCtrlData;

/**
 *
 */
template <typename T>
class FSM_State_Stepping : public FSM_State<T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FSM_State_Stepping(ControlFSMData<T>* _controlFSMData);

  // Behavior to be carried out when entering a state
  void onEnter();

  // Run the normal behavior for the state
  void run();

  // Behavior to be carried out when exiting a state
  void onExit();

 private:
  // Keep track of the control iterations
  int _iter = 0;
  T _phase[2];
  T _gait_period = 3*0.66;

  // Parses contact specific controls to the leg controller
  void BalanceStandStep();
  void UpdateGaitInfo();
  T SwingTrajectory(T phase, T swing_height);


  WBC_Ctrl<T> * _wbc_ctrl;
  LocomotionCtrlData<T> * _wbc_data;

  T last_height_command = 0;

  Vec3<T> _ini_body_pos;
  Vec3<T> _ini_body_ori_rpy;
  T _body_weight;
};

#endif  // FSM_STATE_STEPPING_H
