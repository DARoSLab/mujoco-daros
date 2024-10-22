/*============================ Control FSM ============================*/
/**
 * The Finite State Machine that manages the robot's controls. Handles
 * calls to the FSM State functions and manages transitions between all
 * of the states.
 */

#include "ControlFSM.h"
#include <rt/rt_rc_interface.h>

// FSM States
#include "FSM_State.h"
#include "FSM_State_BalanceStand.h"
#include "FSM_State_Locomotion.h"
#include "FSM_State_Passive.h"
#include "FSM_State_RecoveryStand.h"
#include "FSM_State_LocomotionRPC.h"
#include "FSM_State_JointPD.h"
#include "FSM_State_RL.h"
#include "FSM_State_Stepping.h"
#include "FSM_State_IK_Stepping.h"
#include "FSM_State_RMP.h"
#include "FSM_State_LocomotionRMP.h"


/**
 * Constructor for the Control FSM. Passes in all of the necessary
 * data and stores it in a struct. Initializes the FSM with a starting
 * state and operating mode.
 *
 * @param _quadruped the quadruped information
 * @param _stateEstimator contains the estimated states
 * @param _legController interface to the leg controllers
 * @param _gaitScheduler controls scheduled foot contact modes
 * @param _desiredStateCommand gets the desired COM state trajectories
 * @param userParameters passes in the control parameters from the GUI
 */
template <typename T>
ControlFSM<T>::ControlFSM(PatBiped<T>* pat,
                          StateEstimatorContainer<T>* _stateEstimator,
                          LegControllerPat<T>* _legController,
                          GaitSchedulerPat<T>* _gaitScheduler,
                          DesiredStateCommand<T>* _desiredStateCommand,
                          PatParameters* userParameters,
                          VisualizationData* visualizationData,
                          MoCapData<T>* mocapData, int robot_id)
{
  // Add the pointers to the ControlFSMData struct
  _fsm_data._pat = pat;
  _fsm_data._stateEstimator = _stateEstimator;
  _fsm_data._legController = _legController;
  _fsm_data._gaitScheduler = _gaitScheduler;
  _fsm_data._desiredStateCommand = _desiredStateCommand;
  _fsm_data.userParameters = userParameters;
  _fsm_data.visualizationData = visualizationData;
  _fsm_data.mocapData = mocapData;
  _fsm_data._robot_id = robot_id;
  // Initialize and add all of the FSM States to the state list
  _state_list.resize(FSM_StateName::NUM_STATE);
  _state_list[FSM_StateName::PASSIVE] = new FSM_State_Passive<T>(&_fsm_data);
  _state_list[FSM_StateName::BALANCE_STAND]= new FSM_State_BalanceStand<T>(&_fsm_data);
  _state_list[FSM_StateName::LOCOMOTION] = new FSM_State_Locomotion<T>(&_fsm_data);
  _state_list[FSM_StateName::RECOVERY_STAND] = new FSM_State_RecoveryStand<T>(&_fsm_data);
  _state_list[FSM_StateName::LOCOMOTION_RPC] = new FSM_State_LocomotionRPC<T>(&_fsm_data);
  _state_list[FSM_StateName::JOINT_PD] = new FSM_State_JointPD<T>(&_fsm_data);
  _state_list[FSM_StateName::STEPPING] = new FSM_State_Stepping<T>(&_fsm_data);
  _state_list[FSM_StateName::IK_STEPPING] = new FSM_State_IK_Stepping<T>(&_fsm_data);
  #ifdef RMPFLOW_BUILD
  _state_list[FSM_StateName::COLLISION_RMP] = new FSM_State_RMP<T>(&_fsm_data);
  _state_list[FSM_StateName::LOCOMOTION_RMP] = new FSM_State_LocomotionRMP<T>(&_fsm_data);
  #endif
#ifdef MACHINE_LEARNING_BUILD
  _state_list[FSM_StateName::LOCOMOTION_RL] = new FSM_State_RL<T>(&_fsm_data);
#endif

  safetyChecker = new SafetyChecker<T>(&_fsm_data);
  // Initialize the FSM with the Passive FSM State
  initialize();
}

/**
 * Initialize the Control FSM with the default settings. SHould be set to
 * Passive state and Normal operation mode.
 */
template <typename T>
void ControlFSM<T>::initialize() {
  // Initialize a new FSM State with the control data
  // currentState = _state_list[FSM_StateName::PASSIVE];
  operatingMode = FSM_OperatingMode::NORMAL;
  _previous_state = FSM_StateName::PASSIVE;
  _current_state = FSM_StateName::PASSIVE;
  // Enter the new current state cleanly
  // currentState->onEnter();

  // Initialize FSM mode to normal operation
}

template<typename T>
void ControlFSM<T>::_RC_DataUpdate(){
  if(_fsm_data.userParameters->use_rc){
    int rc_mode = _fsm_data._desiredStateCommand->rcCommand->mode;

    if(rc_mode == RC_mode::RECOVERY_STAND){
      _fsm_data.userParameters->control_mode = FSM_StateName::RECOVERY_STAND;

    } else if(rc_mode == RC_mode::LOCOMOTION){
      _fsm_data.userParameters->control_mode = FSM_StateName::LOCOMOTION;
      // _fsm_data.userParameters->control_mode = FSM_StateName::IK_STEPPING;
    } else if(rc_mode == RC_mode::IK_STEPPING){
      // _fsm_data.userParameters->control_mode = FSM_StateName::BALANCE_STAND;
      _fsm_data.userParameters->control_mode = FSM_StateName::IK_STEPPING;
    } else if(rc_mode == RC_mode::RPC_LOCOMOTION){
      _fsm_data.userParameters->control_mode = FSM_StateName::IK_STEPPING;

    }else if(rc_mode == RC_mode::RL_LOCOMOTION){
      _fsm_data.userParameters->control_mode = FSM_StateName::IK_STEPPING;

    }

  }
}

/**
 * Called each control loop iteration. Decides if the robot is safe to
 * run controls and checks the current state for any transitions. Runs
 * the regular state behavior if all is normal.
 */
template <typename T>
void ControlFSM<T>::runFSM() {
  // Check the robot state for safe operation
  // operatingMode = safetyPreCheck();

  // Remote Controller Data Update (control mode)
  _RC_DataUpdate();
  // Run the robot control code if operating mode is not unsafe
  if (operatingMode != FSM_OperatingMode::ESTOP) {
    // Run normal controls if no transition is detected
    _current_state = (int)_fsm_data.userParameters->control_mode;
    // if(_current_state>2){
    //   std::cout << "Hardware not ready!!!" << '\n';
    //   exit(0);
    // }
    _b_first_visit = (_current_state != _previous_state);
    if (_b_first_visit)
      _state_list[_current_state]->onEnter();
    else
      _state_list[_current_state]->run();

    // if (operatingMode == FSM_OperatingMode::NORMAL) {
    //
    //   // Detect a commanded transition
    // if ((int)_fsm_data.userParameters->control_mode != currentState->stateName) {
    //     printf("[FSM State] Mode Change: from %d to %d\n",
    //         currentState->stateName, (int)_fsm_data.userParameters->control_mode);
    //     currentState->onExit();
    //     // Get the next FSM State by name
    //     currentState = _state_list[(int)_fsm_data.userParameters->control_mode];
    //     currentState->onEnter();

    //   } else {
    //     // Run the iteration for the current state normally
    //     currentState->run();
    //   }
    // }
    // // Check the robot state for safe operation
    // safetyPostCheck();

  } else { // if ESTOP
    _state_list[FSM_StateName::PASSIVE]->run();
  }

  // Print the current state of the FSM
  printInfo(0);
  _previous_state = _current_state;
// Increase the iteration counter
  iter++;
}

/**
 * Checks the robot state for safe operation conditions. If it is in
 * an unsafe state, it will not run the normal control code until it
 * is safe to operate again.
 *
 * @return the appropriate operating mode
 */
template <typename T>
FSM_OperatingMode ControlFSM<T>::safetyPreCheck() {
  // Check for safe orientation if the current state requires it
  if (currentState->checkSafeOrientation &&
      _fsm_data.userParameters->control_mode != FSM_StateName::RECOVERY_STAND) {
    if (!safetyChecker->checkSafeOrientation()) {
      operatingMode = FSM_OperatingMode::NORMAL;
      _fsm_data.userParameters->control_mode = FSM_StateName::RECOVERY_STAND;
      std::cout << "[CONTROL_FSM] broken: Orientation Safety Check FAIL" << std::endl;
    }
  }

  // Default is to return the current operating mode
  return operatingMode;
}

/**
 * Checks the robot state for safe operation commands after calculating the
 * control iteration. Prints out which command is unsafe. Each state has
 * the option to enable checks for commands that it cares about.
 *
 * Should this EDamp / EStop or just continue?
 * Should break each separate check into its own function for clarity
 *
 * @return the appropriate operating mode
 */
template <typename T>
FSM_OperatingMode ControlFSM<T>::safetyPostCheck() {
  // Check for safe desired foot positions
  if (currentState->checkPDesFoot) {
    safetyChecker->checkPDesFoot();
  }

  // Check for safe desired feedforward forces
  if (currentState->checkForceFeedForward) {
    safetyChecker->checkForceFeedForward();
  }

  // Default is to return the current operating mode
  return operatingMode;
}

/**
 * Prints Control FSM info at regular intervals and on important events
 * such as transition initializations and finalizations. Separate function
 * to not clutter the actual code.
 *
 * @param printing mode option for regular or an event
 */
template <typename T>
void ControlFSM<T>::printInfo(int opt) {
  switch (opt) {
    case 0:  // Normal printing case at regular intervals
      // Increment printing iteration
      printIter++;

      // Print at commanded frequency
      if (printIter == printNum) {
        std::cout << "[CONTROL FSM] Printing FSM Info...\n";
        std::cout
            << "---------------------------------------------------------\n";
        std::cout << "Iteration: " << iter << "\n";
        if (operatingMode == FSM_OperatingMode::NORMAL) {
          std::cout << "Operating Mode: NORMAL in " << _state_list[_current_state]->stateString
                    << "\n";
        } else if (operatingMode == FSM_OperatingMode::ESTOP) {
          std::cout << "Operating Mode: ESTOP\n";
        }
        std::cout << "Gait Type: " << _fsm_data._gaitScheduler->gaitData.gaitName
                  << "\n";
        std::cout << std::endl;

        // Reset iteration counter
        printIter = 0;
      }

      // Print robot info about the robot's status
      // _fsm_data._gaitScheduler->printGaitInfo();
      // _fsm_data._desiredStateCommand->printStateCommandInfo();

      break;
  }
}

// template class ControlFSM<double>; This should be fixed... need to make
// RobotRunner a template
template class ControlFSM<float>;
//template class ControlFSM<double>;
