#ifndef CONTROL_FSM_PAT_H
#define CONTROL_FSM_PAT_H

#include <iostream>

// Checks the robot state and commands for safety
#include "SafetyChecker.h"
#include <RPC/ThreadedRPCInterface.h>

template<typename T> class FSM_State;
/**
 * Enumerate all of the FSM states so we can keep track of them.
 */
enum FSM_StateName {
  PASSIVE = 0,
  RECOVERY_STAND = 1,
  BALANCE_STAND = 2,
  LOCOMOTION = 3,
  LOCOMOTION_RL = 4,
  LOCOMOTION_RPC = 5,
  JOINT_PD = 6,
  STEPPING = 7,
  IK_STEPPING = 8,
  #ifdef RMPFLOW_BUILD
    COLLISION_RMP = 9,
    LOCOMOTION_RMP = 10,
  #endif
  NUM_STATE
};

template <typename T>
struct FSM_ControllerList {
  ThreadedRPCInterface* RPC_Interface;
};


/**
 * Enumerate all of the operating modes
 */
enum class FSM_OperatingMode { NORMAL, ESTOP };


/**
 * Control FSM handles the FSM states from a higher level
 */
template <typename T>
class ControlFSM {
 public:
  // Initialize for Pat
  ControlFSM(PatBiped<T>* pat,
             StateEstimatorContainer<T>* _stateEstimator,
             LegControllerPat<T>* _legController,
             GaitSchedulerPat<T>* _gaitScheduler,
             DesiredStateCommand<T>* _desiredStateCommand,
             PatParameters* userParameters,
             VisualizationData* visualizationData,
             MoCapData<T>* mocapDat, int robot_id=0);

  // Initializes the Control FSM instance
  void initialize();

  // Runs the FSM logic and handles the state transitions and normal runs
  void runFSM();

  // Runs the FSM logic and handles the state transitions and normal runs when running training simulations
  void runTrainingFSM(bool & b_render);

  // This will be removed and put into the SafetyChecker class
  FSM_OperatingMode safetyPreCheck();

  //
  FSM_OperatingMode safetyPostCheck();

  // Gets the next FSM_State from the list of created states when requested
  FSM_State<T>* getNextState(FSM_StateName stateName);

  // Prints the current FSM status
  void printInfo(int opt);

  // Contains all of the control related data
  ControlFSMData<T> _fsm_data;

  // FSM state information
  std::vector<FSM_State<T> *> _state_list;
  FSM_State<T>* currentState;    // current FSM state

  // Checks all of the inputs and commands for safety
  SafetyChecker<T>* safetyChecker;
  int getRobotId(){return _robot_id;}
 private:
  int _previous_state;
  int _current_state;
  bool _b_first_visit = true;
  // Operating mode of the FSM
  FSM_OperatingMode operatingMode;

  // Choose how often to print info, every N iterations
  int printNum = 10000;  // N*(0.001s) in simulation time

  // Track the number of iterations since last info print
  int printIter = 0;  // make larger than printNum to not print

  int iter = 0;
  int _robot_id = 0;

  void _RC_DataUpdate();
};

#endif  // CONTROLFSM_H
