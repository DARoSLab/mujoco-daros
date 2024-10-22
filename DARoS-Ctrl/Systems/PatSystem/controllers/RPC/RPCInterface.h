#ifndef RPCINTERFACE_H
#define RPCINTERFACE_H

#include <thread>
#include <iostream>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>

// Contains all of the control related data
#include <state_machine/ControlFSMData.h>

#include <lcm/lcm-cpp.hpp>
#include "rpc_inputs_lcmt.hpp"
#include "rpc_outputs_lcmt.hpp"

// Normal robot states
#define K_THREADED 0
#define K_LCM 1
#define K_NEURAL_NET 2
#define K_PERIODIC 3

using namespace std;

/**
 * Enumerate all of the FSM states so we can keep track of them.
 */
enum class RPCInterfaceName {
  THREADED,
  LCM,
  NEURAL_NET,
  PERIODIC
};

class RPCInterface {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RPCInterface(ControlFSMData<float>* _controlFSM);

  void setRPCInputs();
  void getResult();

  void SendRPCInputsLCM();

  double BoundDesiredState(int k_curr, int state_num, double x_curr, double x_des_curr, double dx_curr, double error_max);

  Vec3<float> getFootFeedForwardForces(int leg);
  Vec3<float> getFootstepLocations(int leg);
  double GetPredictedState(int state);
  Mat3<float> OmegaTodRPY(Vec3<float> rpy);

  ControlFSMData<float>* data;


  std::thread _solveThread;

  bool INITIALIZED = false;
  bool FIRST_RUN = false;
  bool RUN_RPC = false;

  // Store the current force and step location results for the filter
  Mat34<float> footFeedForwardForces = Mat34<float>::Zero();
  Mat34<float> footstepLocations = Mat34<float>::Zero();

  // Store the previous force and step location results for the filter
  double alpha_forces = 0.9;
  double alpha_footsteps = 0.9;
  Mat34<float> footFeedForwardForcesPrev = Mat34<float>::Zero();
  Mat34<float> footstepLocationsPrev = Mat34<float>::Zero();


  Eigen::Vector4i contact_state_scheduled;
  Eigen::Vector4i contact_state_result;
  Eigen::Vector4i contact_state_last_valid;

  // Store the decision variable results
  Mat34<float> footFeedForwardForces_last_valid = Mat34<float>::Zero();
  Mat34<float> footstepLocations_last_valid = Mat34<float>::Zero();;

  double dt_solve_k = 0;
  double dt_solve_pred = 0;
  double alpha_solve_pred = 0.2;

  // Position and yaw get updated
  double px_des = 0;
  double py_des = 0;
  double yaw_des = 0;

  // Max error bounds so robot does not have too big of an error
  double px_error_max = 0.05;
  double py_error_max = 0.05;
  double yaw_error_max = 0.1745;

  // Outputs: Result
  double X_result[180] = {0};
  double x_opt[60] = {0};
  double u_opt[120] = {0};
  double p_opt[12] = {0};
  double dt_pred[5] = {0};
  double opt_time = 0;

  // Inputs
  double current_state[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  double p_foot0[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  double r_hip[12] = {0};
  double x_desired[72] = 
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  double gait_enabled[4] = {0};
  double phase_variable[4] = {0};
  double period_time[4] = {0};
  double time_swing[4] = {0};
  double mass = 0;          // lumped total mass, [kg]
  double inertia[3] = {0};  // body inertia, {Ixx, Iyy, Izz}, [km/s]
  double gravity[3] = {0};  // gravity, [m/s^2]
  double mu[4] = {0};       // ground friction coefficient
  double z_g[4] = {0};      // ground height. [m]
  double Q[12] = {0};
  double R[24] = {0};
  double K_HX[30] = {1.25, 0, -0.3, 0.02, 0, 0, 0, 0, 0, 0,
                     1.0, 1.5, 1, 1, 1, 0, 0, 0, 0, 0,
                     1.0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  int iter = 0;
  int success_test = 1;

  // LCM parameters setup
  lcm::LCM * lcm;
  rpc_inputs_lcmt input_data;

private:

  bool CheckSafeSolve();
};

#endif
