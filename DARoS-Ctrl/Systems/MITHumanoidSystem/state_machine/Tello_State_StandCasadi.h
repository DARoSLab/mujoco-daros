#ifndef FSM_STATE_Tello_STANDCASADI_H
#define FSM_STATE_Tello_STANDCASADI_H

#include "Tello_State.h"
#include <MPC_Casadi/CASADI_Stand_Tello.h>

template<typename T> class WBC_Ctrl;
template<typename T> class TelloStandCtrlData;

/**
 *
 */
template <typename T>
class Tello_State_StandCasadi : public Tello_State<T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Tello_State_StandCasadi(ControlFSMData_Tello<T>* _controlFSMData);

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
   CASADI_Stand_Tello* _casadiMPC;
  bool _b_standing_up = true;
  // Keep track of the control iterations
  int _iter = 0;

  // Parses contact specific controls to the leg controller
  void KeepPostureStep();

  WBC_Ctrl<T> * _wbc_ctrl;
  TelloStandCtrlData<T> * _wbc_data;

  Vec3<T> _des_com_pos; 
  Vec3<T> _ini_com_pos;
  Vec3<T> _ini_body_ori_rpy;
  Vec3<T> _ini_body_pos;
  DVec<T> _ini_jpos;
  Vec3<T> _mid_pos_cps;
  T _body_weight;
};

#endif  // FSM_STATE_BALANCESTAND_BIPED_H
