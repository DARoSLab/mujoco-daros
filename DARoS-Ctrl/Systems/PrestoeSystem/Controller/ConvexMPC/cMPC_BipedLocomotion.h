#ifndef BIPED_CONVEXMPCLOCOMOTION_PAT_H
#define BIPED_CONVEXMPCLOCOMOTION_PAT_H

#include <ctrl_utils/FootSwingTrajectory.h>
#include <ControlFSMData.h>
#include "cppTypes.h"
#include "BipedGait.h"
#include "cMPC_BipedInterface.h"
#include "Reversal_LIPM_Planner.hpp"
#include "LiftSwingTrajectory.hpp"

#include <cstdio>

class cMPC_BipedLocomotion {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  cMPC_BipedLocomotion(
      float _dt, int _iterations_between_mpc, PatParameters* parameters,
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
  void _SetupCommand(ControlFSMData<float> & data);
  void _updateWBC_CMD();

  Reversal_LIPM_Planner* _planner;

  float pNextFoot[_num_cp][3];

  float _yaw_turn_rate;


  float _roll_turn_rate;
  float _pitch_turn_rate;

  float _roll_des;
  float _pitch_des;

  float _x_vel_des = 0.;
  float _y_vel_des = 0.;
  float _body_height = 0.40;

  void updateMPCIfNeeded(int* mpcTable, ControlFSMData<float>& data);
  void solveDenseMPC(int *mpcTable, ControlFSMData<float> &data);


  int iterationsBetweenMPC;
  int horizonLength;
  int default_iterations_between_mpc;
  float dt;
  float dtMPC;
  int iterationCounter = 0;
  Vec3<float> f_ff[_num_cp];
  float swingTimes[_num_cp];
  FootSwingTrajectory<float> footSwingTrajectories[_num_cp];
  LiftSwingTrajectory<float> liftSwingTrajectories[_num_cp];
  OffsetDurationGait standing, walking, running;
  Mat3<float> Kp, Kd, Kp_stance, Kd_stance;
  bool firstRun = true;
  bool firstSwing[_num_cp];
  float swingTimeRemaining[_num_cp];
  float stand_traj[6];
  int current_gait;
  int gaitNumber;

  BipedGait* gait;
  Vec3<float> v_des_world;
  Vec3<float> world_position_desired;
  Vec3<float> rpy_int;
  Vec3<float> rpy_comp;
  Vec3<float> pFoot[_num_cp];
  float trajAll[12*36];

  PatParameters* _parameters = nullptr;
  vectorAligned<Vec12<double>> _sparseTrajectory;
  std::vector<Vec3<float>>init_hip_loc  = std::vector<Vec3<float>>(2);
  std::vector<Vec3<float>>init_foot_loc = std::vector<Vec3<float>>(2);
  Vec3<float> com_pos, com_vel;
  FBModelState<float> _state;
  bool _enable_fp_planning[2]={false, false};
  bool _done_fp_planning[2] = {false, false};
  float _swing_time, _swing_height;
  std::vector<double> _default_foot_loc;
  void cMPCParameterInitialization(const std::string & file);
};


#endif //CHEETAH_SOFTWARE_CONVEXMPCLOCOMOTION_H
