#ifndef TVRLOCOMOTION_PAT_H
#define TVRLOCOMOTION_PAT_H

#include "cppTypes.h"
#include "BipedGait.h"
#include "cMPC_BipedInterface.h"
#include "Reversal_LIPM_Planner.hpp"
#include "LiftSwingTrajectory.hpp"
#include "minjerk_one_dim.hpp"
#include "BSplineBasic.h"

#include <cstdio>

class TVRLocomotion {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TVRLocomotion(PatParameters* parameters,
      const FloatingBaseModel<float> * model );
  void initialize();

  void run(ControlFSMData<float>& data);

  const FloatingBaseModel<float> * _model;
  static constexpr int _num_cp = 2;

  Vec3<float> pBody_des;
  Vec3<float> vBody_des;
  Vec3<float> aBody_des;

  Vec3<float> pBody_RPY_des;
  Vec3<float> vBody_Ori_des;

  Vec3<float> pFoot_des[_num_cp];
  Vec3<float> vFoot_des[_num_cp];
  Vec3<float> aFoot_des[_num_cp];

  Vec3<float> Fr_des[_num_cp];
  float _yaw_des =0.f;
  Vec2<float> contact_state;
  int fsm_state;
  float fsm_phase;

private:

  PatParameters* _parameters = nullptr;
  Reversal_LIPM_Planner* _planner;

  float _yaw_turn_rate;
  float _roll_turn_rate;
  float _pitch_turn_rate;

  float _roll_des;
  float _pitch_des;

  float _x_vel_des = 0.;
  float _y_vel_des = 0.;
  float _body_height = 0.35;


  int _iter = 0;
  int _step_count = -1;


  LiftSwingTrajectory<float> liftSwingTrajectories[_num_cp];
  FootSwingTrajectory<float> footSwingTrajectory;

  Vec3<float> _pFoot[_num_cp];
  Vec3<float> _com_pos, _com_vel;
  std::vector<double> _default_foot_loc;



  //LCM
  lcm::LCM _tvrLCM, _gaitLCM;
  tvr_data_lcmt _tvr_lcm;
  pat_gait_lcmt _gait_lcm;

  //Gait
  SimpleGait* _sgait;
  double _alpha, _beta;
  Vec2<float> _swing_offset;
  Vec2<float> _swing_states;
  Vec2<float> _contact_states;
  float _swing_time, _swing_height;
  int _previous_cs[2], _current_cs[2];

  int _swing_leg, _stance_leg;
  //TVR
  Vec3<float> _com_pos_ini;
  Vec3<float> _com_vel_ini;
  Vec3<float> _stance_foot_loc_ini;
  Vec3<float> _init_target_foot_loc;
  Vec3<float> _target_foot_loc;
  float _x_com_des = 0.0;
  float _y_com_des = 0.0;
  bool _enable_swing_planning = false;
  double _tvr_plan_time;


  std::vector<MinJerk_OneDimension*> _min_jerk_offset;
  BS_Basic<3, 3, 1, 2, 2> _foot_traj;


  void _setupCommand(ControlFSMData<float> & data);
  void _planSwingLegTrajectory();
  void _runTVRPlanning();
  void _updateWBCCMD();
  void _LoadLocomotionParams(const std::string & file);

  void _SetBspline(const Vec3<float> & st_pos, const Vec3<float> & des_pos);
  void _SetMinJerkOffset(const Vec3<float> & offset);
  void _SetBazier(const Vec3<float> & st_pos, const Vec3<float> & des_pos);
  void _GetBsplineSwingTrajectory();
  void _GetBazierSwingTrajectory();
};


#endif //CHEETAH_SOFTWARE_CONVEXMPCLOCOMOTION_H
