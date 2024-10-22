/*============================ Control FSM ============================*/
/**
 * The Finite State Machine that manages the robot's controls. Handles
 * calls to the FSM State functions and manages transitions between all
 * of the states.
 */

#include "ControlFSM_Tello.h"
//#include <rt/rt_rc_interface.h>

// FSM States
#include "Tello_State.h"
#include "Tello_State_Passive.h"
#include "Tello_State_JointPD.h"
#include "Tello_State_BalanceStand.h"
#include "Tello_State_StandCasadi.h"
#include "Tello_State_WalkCasadi.h"
#include "Tello_State_Parkour.h"


/**
 * Constructor for the Control FSM. Passes in all of the necessary
 * data and stores it in a struct. Initializes the FSM with a starting
 * state and operating mode.
 *
 * @param _tello the tello information
 * @param _stateEstimator contains the estimated states
 * @param _limbController interface to the limb controllers
 * @param userParameters passes in the control parameters from the GUI
 */
template <typename T>
ControlFSM_Tello<T>::ControlFSM_Tello(Tello<T>* _tello,
                          TelloJointController<T>* jointController,
                          StateEstimatorContainer<T>* stateEstimator,
                          UserInputHandler<T> * userInputHandler,
                          TelloParameters* userParameters,
                          VisualizationData* visualizationData)
{
  // Add the pointers to the ControlFSMData struct
  _tello_model = _tello->buildModel();
  data._tello_model = &_tello_model;
  data._stateEstimator = stateEstimator;
  data._jointController = jointController;
  data._userParameters = userParameters;
  data._visualizationData = visualizationData;
  data._userInputHandler = userInputHandler;

  // Initialize and add all of the FSM States to the state list
  _state_list.resize(Tello_StateName::NUM_STATE);

  _state_list[Tello_StateName::PASSIVE] = new Tello_State_Passive<T>(&data);
  _state_list[Tello_StateName::JOINT_PD] = new Tello_State_JointPD<T>(&data);
  _state_list[Tello_StateName::BALANCE_STAND]= new Tello_State_BalanceStand<T>(&data);
  _state_list[Tello_StateName::STAND_CASADI]= new Tello_State_StandCasadi<T>(&data);
  _state_list[Tello_StateName::WALK_CASADI]= new Tello_State_WalkCasadi<T>(&data);
  _state_list[Tello_StateName::PARKOUR]= new Tello_State_Parkour<T>(&data);

  //_state_list[Tello_StateName::LOCOMOTION] = new Tello_State_Locomotion<T>(&data);
  
  safetyChecker = new SafetyChecker_Tello<T>(&data);

  printf("[Control FSM Tello] Constructed\n");
  // Initialize the FSM with the Passive FSM State
  initialize();
}

/**
 * Initialize the Control FSM with the default settings. SHould be set to
 * Passive state and Normal operation mode.
 */
template <typename T>
void ControlFSM_Tello<T>::initialize() {
  // Initialize a new FSM State with the Passive FSM State
  // because this function called before the system state is updated
  currentState = _state_list[Tello_StateName::PASSIVE];

  // Enter the new current state cleanly
  currentState->onEnter();

  // Initialize to not be in transition
  nextState = currentState;

  // Initialize FSM mode to normal operation
  operatingMode = Tello_OperatingMode::NORMAL;
}
/**
 * Called each control loop iteration. Decides if the robot is safe to
 * run controls and checks the current state for any transitions. Runs
 * the regular state behavior if all is normal.
 */
template <typename T>
void ControlFSM_Tello<T>::runFSM() {
  // Check the robot state for safe operation
  operatingMode = safetyPreCheck();

  // Remote Controller Data Update (control mode)
  //_RC_DataUpdate();

  // Run the robot control code if operating mode is not unsafe
  if (operatingMode != Tello_OperatingMode::ESTOP) {
    // Run normal controls if no transition is detected
    if (operatingMode == Tello_OperatingMode::NORMAL) {
      // Check the current state for any transition
      nextStateName = currentState->checkTransition();
      // Detect a commanded transition
      if (nextStateName != currentState->stateName) {
        // Set the FSM operating mode to transitioning
        operatingMode = Tello_OperatingMode::TRANSITIONING;

        // Get the next FSM State by name
        nextState = _state_list[nextStateName];

        // Print transition initialized info
        //printInfo(1);

      } else {
        // Run the iteration for the current state normally
        currentState->run();
      }
    }

    // Run the transition code while transition is occuring
    if (operatingMode == Tello_OperatingMode::TRANSITIONING) {
      transitionData = currentState->transition();

      // Check the robot state for safe operation
      safetyPostCheck();

      // Run the state transition
      if (transitionData.done) {
        // Exit the current state cleanly
        currentState->onExit();

        // Print finalizing transition info
        //printInfo(2);

        // Complete the transition
        currentState = nextState;

        // Enter the new current state cleanly
        currentState->onEnter();

        // Return the FSM to normal operation mode
        operatingMode = Tello_OperatingMode::NORMAL;
      }
    } else {
      // Check the robot state for safe operation
      safetyPostCheck();
    }

  } else { // if ESTOP
    currentState = _state_list[Tello_StateName::PASSIVE];
    currentState->onEnter();
    nextStateName = currentState->stateName;
  }

  // Print the current state of the FSM
  printInfo(0);

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
Tello_OperatingMode ControlFSM_Tello<T>::safetyPreCheck() {
  // Check for safe orientation if the current state requires it
  if (currentState->checkSafeOrientation) {
    if (!safetyChecker->checkSafeOrientation()) {
      operatingMode = Tello_OperatingMode::NORMAL;
      data._userParameters->control_mode = Tello_StateName::PASSIVE;
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
Tello_OperatingMode ControlFSM_Tello<T>::safetyPostCheck() {
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
void ControlFSM_Tello<T>::printInfo(int opt) {
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
        if (operatingMode == Tello_OperatingMode::NORMAL) {
          std::cout << "Operating Mode: NORMAL in " << currentState->stateString
                    << "\n";

        } else if (operatingMode == Tello_OperatingMode::TRANSITIONING) {
          std::cout << "Operating Mode: TRANSITIONING from "
                    << currentState->stateString << " to "
                    << nextState->stateString << "\n";

        } else if (operatingMode == Tello_OperatingMode::ESTOP) {
          std::cout << "Operating Mode: ESTOP\n";
        }

        // Reset iteration counter
        printIter = 0;
      }

      // Print robot info about the robot's status
      // data._gaitScheduler->printGaitInfo();
       //data._desiredStateCommand->printStateCommandInfo();

      break;

    case 1:  // Initializing FSM State transition
      std::cout << "[CONTROL FSM] Transition initialized from "
                << currentState->stateString << " to " << nextState->stateString
                << "\n"
                << std::endl;

      break;

    case 2:  // Finalizing FSM State transition
      std::cout << "[CONTROL FSM] Transition finalizing from "
                << currentState->stateString << " to " << nextState->stateString
                << "\n"
                << std::endl;

      break;
  }
}

// template class ControlFSM<double>; This should be fixed... need to make
// RobotRunner a template
template class ControlFSM_Tello<float>;
//template class ControlFSM<double>;
