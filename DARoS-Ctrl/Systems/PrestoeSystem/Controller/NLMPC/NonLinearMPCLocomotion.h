#ifndef SUMMER_NONLINEARMPCLOCOMOTION_H
#define SUMMER_NONLINEARMPCLOCOMOTION_H

#include <TrajGeneration/FootSwingTrajectory.h>
// #include <auxillary/FootTrajGen.h>
// #include <ControlFSMData.h>
#include "cppTypes.h"
#include "Gait.h"
#include <thread>

#include <cstdio>
#include "NMPC.hpp"
using Eigen::Array4f;
using Eigen::Array4i;
using namespace Eigen;


template<typename T>
struct NLMPC_Result {
  T commands[10];
  Vec10<T> contactPhase;
};

class NonLinearMPCLocomotion {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  NonLinearMPCLocomotion(float _dt, int _iterations_between_mpc);
  void initialize();
  void terminate();
  void run();

  Vec3<float> pBody_des;
  Vec3<float> vBody_des;
  Vec3<float> aBody_des;

  Vec3<float> pBody_RPY_des;
  Vec3<float> vBody_Ori_des;

  Vec3<float> pFoot_des[2];
  Vec3<float> vFoot_des[2];
  Vec3<float> aFoot_des[2];
  Vec3<float> Fr_des[10];
  Vec3<float> mpc_pBody_des;
  Vec3<float> mpc_vBody_des;


  Vec4<float> contact_state;

  Gait* gait; // TODO: is it safe to have gait as a public pointer?

  int getIterCount(){return iterationCounter;}

private:
  void _setCommand();
  void _updateWBC();


  float _yaw_turn_rate;
  float _yaw_des;


  float _x_vel_des = 0.;
  float _y_vel_des = 0.;

  // High speed running
  float _body_height = 0.3;

  float _default_body_height = 0.3;

  void updateMPCIfNeeded();

  int iterationsBetweenMPC;
  int horizonLength;
  int default_iterations_between_mpc;
  float dt;
  float dtMPC;
  int iterationCounter = 0;

  Vec4<float> swingTimes;
  Vec3<float> v_des_world;
  FootSwingTrajectory<float> footSwingTrajectories[4];

  OffsetDurationGait standing, walking;
  bool firstRun = true;
  bool firstSwing[4];
  float swingTimeRemaining[4];


  Vec3<float> world_position_desired;
  float x_comp_integral = 0;
  Vec3<float> pFoot[4];
  Vec3<float> pNextFoot[4];
  NLMPC_Result<float> result;

  vectorAligned<Vec12<double>> _sparseTrajectory;

  MatrixXf get_desireR(float yaw, float yaw_rate);
  MatrixXf get_desireRPY(float yaw, float yaw_rate);
  MatrixXf get_desireX(Vector3f startPos, Vector3f desVel);
  MatrixXf get_desireF(MatrixXf mpcTable);
  // MPC_output output;
  NMPC nmpc;
  std::vector<OSQPFloat> z0;
  std::vector<OSQPFloat> params;
  void computeInitialNMPCGuess();
  void updateNMPCParams(MatrixXf contactLoc, MatrixXf mpcTable, VectorXf ref, MatrixXf Xref, MatrixXf Rref, MatrixXf Fref);
  std::thread mpc_thread;

  bool mpc_can_run = false;
  bool terminate_thread = false;
};

#endif
