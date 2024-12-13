#ifndef BIPED_CONVEXMPCLOCOMOTION_H
#define BIPED_CONVEXMPCLOCOMOTION_H

#include <ctrl_utils/FootSwingTrajectory.h>
#include <ControlFSMData_Humanoid.h>
#include "cppTypes.h"
#include "BipedGait.h"
#include "cMPC_BipedInterface.h"


#include <cstdio>

using Eigen::Array4f;
using Eigen::Array4i;


class cMPC_BipedLocomotion {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  cMPC_BipedLocomotion(
      float _dt, int _iterations_between_mpc, HumanoidParameters* parameters,
      const FloatingBaseModel<float> * model );
  void initialize();

  void run(ControlFSMData_Humanoid<float>& data);

  const FloatingBaseModel<float> * _model;
  Vec3<float> pBody_des;
  Vec3<float> vBody_des;
  Vec3<float> aBody_des;

  Vec3<float> pBody_RPY_des;
  Vec3<float> vBody_Ori_des;

  Vec3<float> pFoot_des[4];
  Vec3<float> vFoot_des[4];
  Vec3<float> aFoot_des[4];

  Vec3<float> Fr_des[4];
  float _yaw_des =0.f; 
  Vec4<float> contact_state;

private:
  void _SetupCommand(ControlFSMData_Humanoid<float> & data);
  void _updateWBC_CMD();

  float pNextFoot[4][3];

  float _yaw_turn_rate;
  

  float _roll_turn_rate;
  float _pitch_turn_rate;

  float _roll_des;
  float _pitch_des;

  float _x_vel_des = 0.;
  float _y_vel_des = 0.;
  float _body_height = 0.5;

  void updateMPCIfNeeded(int* mpcTable, ControlFSMData_Humanoid<float>& data);
  void solveDenseMPC(int *mpcTable, ControlFSMData_Humanoid<float> &data);

  static constexpr int _num_cp = 4;

  int iterationsBetweenMPC;
  int horizonLength;
  int default_iterations_between_mpc;
  float dt;
  float dtMPC;
  int iterationCounter = 0;
  Vec3<float> f_ff[_num_cp];
  float swingTimes[_num_cp];
  FootSwingTrajectory<float> footSwingTrajectories[_num_cp];
  OffsetDurationGait standing, walking, running;
  Mat3<float> Kp, Kd, Kp_stance, Kd_stance;
  bool firstRun = true;
  bool firstSwing[4];
  float swingTimeRemaining[4];
  float stand_traj[6];
  int current_gait;
  int gaitNumber;

  BipedGait* gait;
  Vec3<float> v_des_world;
  Vec3<float> world_position_desired;
  Vec3<float> rpy_int;
  Vec3<float> rpy_comp;
  Vec3<float> pFoot[4];
  float trajAll[12*36];

  HumanoidParameters* _parameters = nullptr;
  vectorAligned<Vec12<double>> _sparseTrajectory;
};


#endif //CHEETAH_SOFTWARE_CONVEXMPCLOCOMOTION_H
