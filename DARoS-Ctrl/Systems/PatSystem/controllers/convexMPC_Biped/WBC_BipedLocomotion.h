#ifndef BIPED_WBCLOCOMOTION_PAT_H
#define BIPED_WBCLOCOMOTION_PAT_H

#include <ctrl_utils/FootSwingTrajectory.h>
#include <ControlFSMData.h>
#include "cppTypes.h"
#include "BipedGait.h"
#include "cMPC_BipedInterface.h"
#include "Reversal_LIPM_Planner.hpp"
#include "LiftSwingTrajectory.hpp"
#include "tvr_data_lcmt.hpp"
#include "pat_gait_lcmt.hpp"

#include <cstdio>

class WBC_BipedLocomotion {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  WBC_BipedLocomotion(PatParameters* parameters,
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

private:
  void _setupCommand(ControlFSMData<float> & data);
  void _updateWBCCMD();

  Reversal_LIPM_Planner* _planner;

  float _yaw_turn_rate;


  float _roll_turn_rate;
  float _pitch_turn_rate;

  float _roll_des;
  float _pitch_des;

  float _x_vel_des = 0.;
  float _y_vel_des = 0.;
  float _body_height = 0.40;


  int _iter = 0;
  float dt;
  float swingTimes[_num_cp];
  LiftSwingTrajectory<float> liftSwingTrajectories[_num_cp];
  OffsetDurationGait standing, walking, walkingDS, walkingDS2, running;
  bool firstRun = true;
  bool firstSwing[_num_cp];
  float swingTimeRemaining[_num_cp];

  BipedGait* gait;
  Vec3<float> pFoot[_num_cp];
  SimpleGait* sgait;

  PatParameters* _parameters = nullptr;
  vectorAligned<Vec12<double>> _sparseTrajectory;
  std::vector<Vec3<float>>init_hip_loc  = std::vector<Vec3<float>>(2);
  std::vector<Vec3<float>>init_foot_loc = std::vector<Vec3<float>>(2);
  Vec3<float> _com_pos, _com_vel;
  FBModelState<float> _state;
  bool _enable_fp_planning[2]={false, false};
  bool _done_fp_planning[2] = {false, false};
  float _swing_time, _swing_height;
  std::vector<double> _default_foot_loc;
  double _enable_double_stance;
  float _x_com_des = 0.0;
  float _y_com_des = 0.0;

  Vec3<float> _com_pos_ini;
  Vec3<float> _com_vel_ini;
  Vec3<float> _stance_foot_loc_ini;

  Vec2<float> _com_pos_state;
  Vec2<float> _com_vel_state;
  int _step_count = 0;

  lcm::LCM _tvrLCM, _gaitLCM;
  tvr_data_lcmt _tvr_lcm;
  pat_gait_lcmt _gait_lcm;




  void tvrParameterInitialization(const std::string & file);
};


#endif //CHEETAH_SOFTWARE_CONVEXMPCLOCOMOTION_H
