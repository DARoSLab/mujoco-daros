#ifndef PAT_DOG_LOCOMOTION_H
#define PAT_DOG_LOCOMOTION_H

#include <convexMPC_Biped/cMPC_BipedLocomotion.h>
#include <convexMPC_Biped/WBC_BipedLocomotion.h>
#include <FSM_State.h>
// #define USE_CMPC
template<typename T> class WBC_Ctrl;
template<typename T> class LocomotionCtrlData;
/**
 *
 */
template <typename T>
class FSM_State_Locomotion : public FSM_State<T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FSM_State_Locomotion(ControlFSMData<T>* _controlFSMData);

  // Behavior to be carried out when entering a state
  void onEnter();

  // Run the normal behavior for the state
  void run();

  // Behavior to be carried out when exiting a state
  void onExit();

 private:
  // Keep track of the control iterations
  int iter = 0;
  #ifdef USE_CMPC
    cMPC_BipedLocomotion* cMPC_biped;
  #else
    WBC_BipedLocomotion* cMPC_biped;
  #endif
  bool ESTOP = false;
  WBC_Ctrl<T> * _wbc_ctrl;
  LocomotionCtrlData<T> * _wbc_data;
  const FloatingBaseModel<T> * _model;
  // FloatingBaseModel<T> _model;
  Vec3<T> _ini_body_ori_rpy;

  Vec3<T> rpy_ini;
  Vec4<T> _ori_ini_inv;
  bool first_visit = true;
  // Parses contact specific controls to the leg controller
  void LocomotionControlStep();

};

#endif  // FSM_STATE_LOCOMOTION_H
