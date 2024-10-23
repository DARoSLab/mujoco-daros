#ifndef UserInputHandler_H
#define UserInputHandler_H

#include <iostream>
#include <queue>
#include <thread>

#include "cppTypes.h"
#include <lcm/lcm-cpp.hpp>

//#include <SimUtilities/GamepadCommand.h>
//#include <rt/rt_rc_interface.h>

template <typename T>
struct UserInputData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  UserInputData() {
    zero();
  }

  // Zero out all of the data
  void zero(){
    stateDes = Vec12<T>::Zero();
    stateTrajDes = Eigen::Matrix<T, 12, 10>::Zero();
  }

  // Instantaneous desired state command
  Vec12<T> stateDes;
  Vec12<T> pre_stateDes;

  void print() { printf("state des: %f, %f, %f ...\n", stateDes[0], stateDes[1], stateDes[2]); }
  // Desired future state trajectory (for up to 10 timestep MPC)
  Eigen::Matrix<T, 12, 10> stateTrajDes;
};

template <typename T>
class UserInputHandler {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Initialize with the GamepadCommand struct
  UserInputHandler(){}

  // TODO
/*
  UserInputHandler(GamepadCommand* command,
                      rc_control_settings* rc_command,
                      StateEstimatorContainer<T>* _stateEstimator,
                      double* _move_waypoint, double* _use_rc, double _dt) :
    _waypointLCMStream(getLcmUrl(255)), velCmdLcm(getLcmUrl(255)) {
    gamepadCommand = command;
    rcCommand = rc_command;
    stateEstimator = _stateEstimator;

    move_waypoint = _move_waypoint; // 0: rc guided, 1: move waypoint, 2: reset waypoint, 3: move/follow waypoint
    use_rc = _use_rc;

    leftAnalogStick.setZero();
    rightAnalogStick.setZero();

    dt = _dt;

    // Reset the state commands
    data.zero();

    // LCM path planning stuff
    _waypointLCMStream.subscribe("PATH", &UserInputHandler<T>::handlePathLCM, this);
    _waypointLCMThread = std::thread(&UserInputHandler::waypointLCMThread, this);

    velCmdLcm.subscribe("VEL_CMD", &UserInputHandler<T>::handleVelCmdLcm, this);
    _velCmdLcmThread = std::thread(&UserInputHandler::velCmdLcmThread, this);
  }

  void convertToStateCommands();
  //void gamepadHotkeys();
  void gamepadCommandToStateDes();
  void rcCommandToStateDes();
  void commandPxdPydYawd();
  void commandPzd();
  void setAndPublishWaypoint();
  void trackPlannedPath();
  void setCommandLimits(T minVelX_in, T maxVelX_in,
                        T minVelY_in, T maxVelY_in,
                        T minTurnRate_in, T maxTurnRate_in);
  void desiredStateTrajectory(int N, Vec10<T> dtVec);
  void printRawInfo();
  void printStateCommandInfo();
  float deadband(float command, T minVal, T maxVal);

  // These should come from the inferface
  T maxRoll = 0.4;
  T minRoll = -0.4;
  T maxPitch = 0.4;
  T minPitch = -0.4;
  T maxVelX = 2.0;
  T minVelX = -2.0;
  //T maxVelX = 5.0;
  //T minVelX = -5.0;
  T maxVelY = 0.8;
  T minVelY = -0.8;
  //T maxVelY = 0.5;
  //T minVelY = -0.5;
  T maxTurnRate = 2.5;
  T minTurnRate = -2.5;
  //T maxTurnRate = 0.5;
  //T minTurnRate = -0.5;


  Vec2<float> leftAnalogStick;
  Vec2<float> rightAnalogStick;

  // Holds the instantaneous desired state and future desired state trajectory
  UserInputData<T> data;

  const rc_control_settings* rcCommand;
  const GamepadCommand* gamepadCommand;

  bool trigger_pressed = false;

 private:
  StateEstimatorContainer<T>* stateEstimator;

  // Dynamics matrix for discrete time approximation
  Mat12<T> A;

  // Control loop timestep change
  T dt;

  // RC or Gamepad
  double* use_rc;

  // Body height command
  const T MC_BODY_HEIGHT = 0.29;

  // Value cutoff for the analog stick deadband
  T deadbandRegion = 0.075;
  //const T filter = 0.01;
  const T filter = 0.02;
  double pitch_filtered = 0.0;

  // Choose how often to print info, every N iterations
  int printNum = 5;  // N*(0.001s) in simulation time

  // Track the number of iterations since last info print
  int printIter = 0;

  // Tom trajectory tracking - waypoint based
  lcm::LCM _waypointLCMStream;
  waypoint_lcmt _waypointLCM;
  desired_state_command_lcmt _dscLCM;
  Vec3<double> _waypoint = {0.0, 0.0, 0.26};
  std::thread _waypointLCMThread;
  double* move_waypoint;
  const double PATH_x_max = 0.5;
  const double PATH_y_max = 0.1;
  const double PATH_yaw_max = 1.25;
  Eigen::Matrix<double, 3, 6> K;
  std::deque<Vec2<double>> path_;

  void waypointLCMThread() {
    while (true) {
      _waypointLCMStream.handle();
    }
  }
  void handlePathLCM(const lcm::ReceiveBuffer* rbuf, const std::string& chan,
                     const path_lcmt* msg);

  // Savva's Trajectory Tracking - velocity command based
  lcm::LCM velCmdLcm;
  vel_cmd_lcmt velCmd;
  std::thread _velCmdLcmThread;
  int update_iter = 0;

  void velCmdLcmThread() {
    while (true) {
      velCmdLcm.handle();
    }
  }
  void handleVelCmdLcm(const lcm::ReceiveBuffer* rbuf, const std::string& chan,
                     const vel_cmd_lcmt* msg);
*/
};

#endif
