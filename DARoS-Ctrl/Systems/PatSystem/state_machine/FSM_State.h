#ifndef QUADRUPED_FSM_State_H
#define QUADRUPED_FSM_State_H

#include <stdio.h>

#include "ControlFSM.h"
#include "ControlFSMData.h"
#include <auxillary/GaitSchedulerPat.h>
#include <ctrl_utils/FootSwingTrajectory.h>
#include <wbc_ctrl/LocomotionCtrl/LocomotionCtrl.hpp>
#include <wbc_ctrl/LocomotionCtrl/LocomotionCtrl.hpp>
#include <RPC/RPCInterface.h>
#include <RPC/ThreadedRPCInterface.h>
#include <RPC/LCMRPCInterface.h>


/**
 * Enumerate all of the natural gait states so we can modify the gait scheduler.
 */
enum class NaturalGaitState {
  STANCE,
  NORMAL_LOCOMOTION,
  TRANSITIONING_TO_STANCE,
  DISTURBANCE
};

/**
 * Enumerate all of the acrobatic motions so we can specify for the data reader.
 */
enum class AcrobaticMotions3D {
  HIGH_JUMP,
  SPIN_180,
  BARREL_ROLL,
  BALANCE_STAND,
  TABLE_JUMP,
  JUMP_RIGHT,
  JUMP_LEFT,
  LATERAL_JUMP_LEFT,
  HIND_LEGS
};


/**
 *
 */
template <typename T>
class FSM_State {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Generic constructor for all states
  FSM_State(ControlFSMData<T>* _controlFSMData, FSM_StateName stateNameIn, std::string stateStringIn);

  // Behavior to be carried out when entering a state
  virtual void onEnter() = 0;// {}

  // Run the normal behavior for the state
  virtual void run() = 0; //{}

  // Behavior to be carried out when exiting a state
  virtual void onExit() = 0; // {}

  //
  void jointPDControl(int leg, Vec3<T> qDes, Vec3<T> qdDes, T kp = 100, T kd = 0.5);
  void cartesianImpedanceControl(int leg, Vec3<T> pDes, Vec3<T> vDes,
                                 Vec3<double> kp_cartesian,
                                 Vec3<double> kd_cartesian);
  void footstepHeuristicPlacement(int leg);

  //
  void runWholeBodyController();
  void runConvexModelPredictiveController();
  void runRegularizedPredictiveController();


  Vec3<T> ForceWorldToHip(Vec3<T> forceWorld);
  Vec3<T> transformPositionWorldToHip(int leg, Vec3<T> positionWorld);
  Vec3<T> VelocityWorldToHip(int leg, Vec3<T> velocityWorld);
  Vec3<T> GetFootPositionWorld(int leg);
  Vec3<T> GetAverageFootPositionWorld();
  bool getRenderingPreference(){ return _b_render; }
  void StanceLegImpedanceControl(int leg);
  Vec3<T> FootstepWorldPosition(int leg, Vec3<T> footstepLocation);
  void SetFootswingTrajectories(int leg, Vec3<T> initialPos, Vec3<T> finalPos, T height_des);
  void SetLegControllerCommands();
  void ZeroLegControllerCommands();

  void NaturalGaitModification();

  //
  void turnOnAllSafetyChecks();
  void turnOffAllSafetyChecks();
  bool locomotionSafe();

  // Visualizations
  void VisualizeSwingLegTrajectory();
  void VisualizeInputForce(Vec3<T> position, Vec3<T> direction, T color[3], T brightness);
  void VisualizeStepPos(Vec3<T> position, T color[3], T brightness);
  void VisualizeArrow(Vec3<T> position, Vec3<T> direction, T color[3], T brightness, T scale);
  void VisualizeSphere(Vec3<T> position, T color[3], T brightness);
  void VisualizeBox(Vec3<T> position, Vec3<T> dimension, Vec3<T> rpy, T color[3], T brightness);
  void VisualizePredictionRPC();


  // Rendering Preference (Send Visual information to Unity)
  bool _b_render = true;
  // Holds all of the relevant control data
  ControlFSMData<T>* _data;

  // FSM State info
  FSM_StateName stateName;      // enumerated name of the current state
  std::string stateString;      // state name string

  // Track the natural gait state
  NaturalGaitState naturalGaitState = NaturalGaitState::STANCE;
  int gaitTransitionIter = 0;
  int recoveryIter = 0;
  T period_time_natural = 0.5;
  T switching_phase_natural = 0.5;
  T swing_time_natural = 0.25;

  // Pre controls safety checks
  bool checkSafeOrientation = false;  // check roll and pitch

  // Post control safety checks
  bool checkPDesFoot = false;          // do not command footsetps too far
  bool checkForceFeedForward = false;  // do not command huge forces
  bool checkLegSingularity = false;    // do not let leg

  // Number of feet to use for balance
  int NUM_FEET = 4;

  // Stores the swing trajectories for each foot | TODO: move these to the FSM_State parent class
  FootSwingTrajectory<float> footSwingTrajectories[4];

  // Leg controller command placeholders for the whole robot (3xNUM_FEET matrices)
  D3Mat<T> jointFeedForwardTorques;  // feed forward joint torques
  D3Mat<T> jointPositions;           // joint angle positions
  D3Mat<T> jointVelocities;          // joint angular velocities
  D3Mat<T> footFeedForwardForces;    // feedforward forces at the feet
  D3Mat<T> footPositions;            // cartesian foot positions
  D3Mat<T> footVelocities;           // cartesian foot velocities
  Mat3<T> kpMat;                     // cartesian P gain matrix
  Mat3<T> kdMat;                     // cartesian D gain matrix
  Mat3<T> kpMatAll[pat_biped::num_legs];               // cartesian P gain matrix
  Mat3<T> kdMatAll[pat_biped::num_legs];               // cartesian D gain matrix

  // Store the impedance control gains for swing and stance
  Mat3<T> Kp_swing, Kd_swing, Kp_stance, Kd_stance;

  // Footstep locations
  D3Mat<T> footstepLocations;  // Footstep locations for next step
  D3Mat<T> footPositionsCurr;  // Footstep locations at the current stance location
  D3Mat<T> footPositionsTD;    // Footstep locations at touchdown

  Vec12<T> x_desired_wbc;

  // ModelPredictiveController cMPC
  // RegularizedPredictiveController RPC
  WBC_Ctrl<T> * _wbc_ctrl;
  LocomotionCtrlData<T> * _wbc_data;

  Vec3<T> _world_des_wbc;

  // The interface to the actual optimization problem
  //  - ThreadedRPCInterface: Creates a thread within control code to run
  //      the optimization as fast as possible
  //  - LCMRPCInterface: Sends inputs and listens for outputs from LCM from
  //      a separate program running the optimization
  RPCInterface* RPC_Interface;

  // Store the colors used for each of the legs
  T legColors[12] = {1, 0, 0,
                     0, 0, 1,
                     0, 1, 0,
                     0.9098, 0.4667, 0.1333
                    };

private:
  T alpha_ngm = 0.0025;

  T pError_act_norm_filt = 0.0;
  T v_act_norm_filt = 0.0;
  T v_des_norm_filt = 0.0;

  // Get the actual and desired turning rates (approximated by omega body Z)
  T yaw_rate_act_filt = 0.0;
  T yaw_rate_des_filt = 0.0;
};

#endif  // FSM_State_H
