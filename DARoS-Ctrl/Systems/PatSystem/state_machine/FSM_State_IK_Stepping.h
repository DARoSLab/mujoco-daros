#ifndef FSM_STATE_IK_STEPPING_H
#define FSM_STATE_IK_STEPPING_H

#include <FSM_State.h>
#include <ctrl_utils/FootSwingTrajectory.h>
#include <lcm-cpp.hpp>
#include <FBModel/FloatingBaseModel.h>
#include "pat_ik_tracking_lcmt.hpp"

template<typename T> class WBC_Ctrl;
template<typename T> class LocomotionCtrlData;

/**
 *
 */
template <typename T>
class FSM_State_IK_Stepping : public FSM_State<T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FSM_State_IK_Stepping(ControlFSMData<T>* _controlFSMData);

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
  T _gait_period = 0.66;
  T _contactPhase[2];
  lcm::LCM _lcm;
  pat_ik_tracking_lcmt _ik_lcm;

  FootSwingTrajectory<float> footSwingTrajectories[2];
  FloatingBaseModel<T> _model;
  FBModelState<T> _state;
  // Parses contact specific controls to the leg controller
  void BalanceStandStep();
  void UpdateGaitInfo();
  void _UpdateModel();
  Vec3<T> SwingTrajectory(T phase, T swing_height, int leg);
  Vec3<T> Analytical_IK(Vec3<T> foot_pos);
};

#endif  // FSM_STATE_STEPPING_H
