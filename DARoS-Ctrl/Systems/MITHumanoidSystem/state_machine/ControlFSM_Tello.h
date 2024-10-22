#ifndef CONTROLFSM_TELLO_H
#define CONTROLFSM_TELLO_H

#include <iostream>

// Checks the robot state and commands for safety
#include "SafetyChecker_Tello.h"
#include "TransitionData_Tello.h"

template<typename T> class Tello_State;
/**
 * Enumerate all of the FSM states so we can keep track of them.
 */
enum Tello_StateName {
  PASSIVE = 0,
  JOINT_PD = 1, 
  BALANCE_STAND = 2,
  WALK_CASADI = 3,
  RECOVERY_STAND = 4,
  PARKOUR = 5,
  STAND_CASADI = 6,
  NUM_STATE
};

/**
 * Enumerate all of the operating modes
 */
enum class Tello_OperatingMode { NORMAL, TRANSITIONING, ESTOP };

/**
 * Control FSM handles the FSM states from a higher level
 */
template <typename T>
class ControlFSM_Tello {
 public:
  // Initialize for Quadruped
  ControlFSM_Tello(Tello<T>* _tello,
             TelloJointController<T>* _jointController,
             StateEstimatorContainer<T>* _stateEstimator,
             UserInputHandler<T> * _userInputHandler,
             TelloParameters* userParameters,
             VisualizationData* visualizationData);

  // Initializes the Control FSM instance
  void initialize();

  // Runs the FSM logic and handles the state transitions and normal runs
  void runFSM();

  Tello_OperatingMode safetyPreCheck();
  Tello_OperatingMode safetyPostCheck();

  // Gets the next FSM_State from the list of created states when requested
  Tello_State<T>* getNextState(Tello_StateName stateName);

  // Prints the current FSM status
  void printInfo(int opt);

  // Contains all of the control related data
  ControlFSMData_Tello<T> data;
  FloatingBaseModel<T> _tello_model;

  // FSM state information
  std::vector<Tello_State<T> *> _state_list;
  Tello_State<T>* currentState;    // current FSM state
  Tello_State<T>* nextState;       // next FSM state
  Tello_StateName nextStateName;   // next FSM state name

  // Checks all of the inputs and commands for safety
  SafetyChecker_Tello<T>* safetyChecker;
  TransitionData_Tello<T> transitionData;
  int iter = 0;
 private:
  // Operating mode of the FSM
  Tello_OperatingMode operatingMode;

  // Choose how often to print info, every N iterations
  int printNum = 10000;  // N*(0.001s) in simulation time

  // Track the number of iterations since last info print
  int printIter = 0;  // make larger than printNum to not print

  

};

#endif  // CONTROLFSM_H
