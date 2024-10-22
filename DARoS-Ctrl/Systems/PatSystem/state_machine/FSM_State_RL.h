#ifndef PAT_FSM_STATE_LOCOMOTION_RL_H
#define PAT_FSM_STATE_LOCOMOTION_RL_H

#ifdef MACHINE_LEARNING_BUILD

#include "controllers/PatRL/PatRL.hpp"
#include <FSM_State.h>
/**
 *
 */
template <typename T>
class FSM_State_RL : public FSM_State<T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FSM_State_RL(ControlFSMData<T>* _controlFSMData);

  // Behavior to be carried out when entering a state
  void onEnter();
  void onExit();
  // Run the normal behavior for the state
  // void run(bool b_first_visit);
  void run();


 private:
  // Keep track of the control iterations
  int iter = 0;
  PatRL* vRLCon;
  ControlFSMData<T>* _data;
  Vec6<float> _tau_ff;
  Vec6<float> _default_dof_pos;
  Vec6<float> _dof_pos_error;
  bool _ready_for_policy = false;
  DVec<T> _dof_pos;
  DVec<T> _dof_vel;
  bool ESTOP = false;

  void updateLegCMD();
  void bringToInitPosture();
  bool isInitPostureDone(T tolerance);


}
;
#endif

#endif  // FSM_STATE_LOCOMOTION_H
