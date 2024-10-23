#ifndef GUIDE_DOG_RECOVERY_STANDUP_H
#define GUIDE_DOG_RECOVERY_STANDUP_H

#include <FSM_State.h>
#include <FBModel/FloatingBaseModel.h>
#include <robots/PatBiped.h>
/**
 *
 */
template <typename T>
class FSM_State_RecoveryStand : public FSM_State<T> {
 public:
  FSM_State_RecoveryStand(ControlFSMData<T>* _controlFSMData);

  // Behavior to be carried out when entering a state
  void onEnter();

  // Run the normal behavior for the state
  void run();

  // Behavior to be carried out when exiting a state
  void onExit();


 private:
  // Keep track of the control iterations
  int iter = 0;
  int _motion_start_iter = 0;
  FloatingBaseModel<T> _model;
  FBModelState<T> _state;

  static constexpr int StandUp = 0;
  static constexpr int FoldLegs = 1;
  static constexpr int RollOver = 2;

  unsigned long long _state_iter;
  int _flag = FoldLegs;
  bool roll_left = true; //Keeps track of last rolling direction

  // JPos
  Vec3<T> fold_jpos[2];
  Vec3<T> stand_jpos[2];
  Vec3<T> rolling_jpos[2];
  Vec3<T> initial_jpos[2];
  Vec3<T> zero_vec3;

  std::vector<double> _ll_stand_jpos, _rl_stand_jpos; 


  Vec3<T> f_ff;

  // iteration setup
  //const int rollover_ramp_iter = 300;
  //const int rollover_settle_iter = 300;

  //const int fold_ramp_iter = 1000;
  //const int fold_settle_iter = 1000;

  //const int standup_ramp_iter = 500;
  //const int standup_settle_iter = 500;

  // 0.5 kHz
  const int rollover_ramp_iter = 150;
  const int rollover_settle_iter = 150;

  //const int fold_ramp_iter = 500;
  //const int fold_settle_iter = 500;
  const int fold_ramp_iter = 800; //400;
  const int fold_settle_iter = 700;

  const int standup_ramp_iter = 500; //250;
  const int standup_settle_iter = 450;
  bool b_standup_settled = false;
  void _UpdateModel();
  void _RollOver(const int & iter);
  void _StandUp(const int & iter);
  void _FoldLegs(const int & iter);

  bool _UpsideDown();
  void _AssignRollDirection();
  void _SetJPosInterPts(
      const size_t & curr_iter, size_t max_iter, int leg,
      const Vec3<T> & ini, const Vec3<T> & fin);
  void _ParameterSetup(const std::string & file);

};

#endif  // FSM_STATE_RECOVERY_STANDUP_H
