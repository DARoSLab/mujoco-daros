#include "StateMachineCtrl.hpp"
#include "JPosCtrlState.hpp"

template <typename T>
StateMachineCtrl<T>::StateMachineCtrl( ObserverManager<T>* obs_manager,
                      UserInputManager<T> * input_manager,
                      VisualManager* vis_manager )
{
  // Initialize and add all of the FSM States to the state list
  _state_list.resize(StateList::NUM_STATE);
  _state_list[StateList::JOINT_PD] = new Staccatoe_State_JointPD<T>(&data);
  
  printf("[State Machine Control] Constructed\n");
  _Initialize();
}

template <typename T>
void StateMachineCtrl<T>::_Initialize() {
  // Initialize a new FSM State with the Passive FSM State
  // because this function called before the system state is updated
  _curr_State = _state_list[StateList::JOINT_PD];

  // Enter the new current state cleanly
  _curr_State->onEnter();

  // Initialize to not be in transition
  _next_State = _curr_State;

  // Initialize FSM mode to normal operation
  _operatingMode = OperatingMode::NORMAL;

}
/**
 * Called each control loop iteration. Decides if the robot is safe to
 * run controls and checks the current state for any transitions. Runs
 * the regular state behavior if all is normal.
 */
template <typename T>
void StateMachineCtrl<T>::runFSM() {
  // Check the robot state for safe operation
  operatingMode = safetyPreCheck();

  // Remote Controller Data Update (control mode)
  //_RC_DataUpdate();

  // Run the robot control code if operating mode is not unsafe
  if (operatingMode != Staccatoe_OperatingMode::ESTOP) {
    // Run normal controls if no transition is detected
    if (operatingMode == Staccatoe_OperatingMode::NORMAL) {
      // Check the current state for any transition
      nextStateName = currentState->checkTransition();

      // Detect a commanded transition
      if (nextStateName != currentState->stateName) {
        // Set the FSM operating mode to transitioning
        operatingMode = Staccatoe_OperatingMode::TRANSITIONING;

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
    if (operatingMode == Staccatoe_OperatingMode::TRANSITIONING) {
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
        operatingMode = Staccatoe_OperatingMode::NORMAL;
      }
    } else {
      // Check the robot state for safe operation
      safetyPostCheck();
    }

  } else { // if ESTOP
    currentState = _state_list[Staccatoe_StateName::PASSIVE];
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
Staccatoe_OperatingMode ControlFSM_Staccatoe<T>::safetyPreCheck() {
  // Check for safe orientation if the current state requires it
  if (currentState->checkSafeOrientation) {
    if (!safetyChecker->checkSafeOrientation()) {
      operatingMode = Staccatoe_OperatingMode::NORMAL;
      data._userParameters->control_mode = Staccatoe_StateName::PASSIVE;
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
Staccatoe_OperatingMode ControlFSM_Staccatoe<T>::safetyPostCheck() {
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
void ControlFSM_Staccatoe<T>::printInfo(int opt) {
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
        if (operatingMode == Staccatoe_OperatingMode::NORMAL) {
          std::cout << "Operating Mode: NORMAL in " << currentState->stateString
                    << "\n";

        } else if (operatingMode == Staccatoe_OperatingMode::TRANSITIONING) {
          std::cout << "Operating Mode: TRANSITIONING from "
                    << currentState->stateString << " to "
                    << nextState->stateString << "\n";

        } else if (operatingMode == Staccatoe_OperatingMode::ESTOP) {
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

// RobotRunner a template
template class ControlFSM_Staccatoe<float>;
//template class ControlFSM<double>;
