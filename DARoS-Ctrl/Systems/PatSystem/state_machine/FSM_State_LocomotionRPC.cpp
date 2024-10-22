/*============================ Locomotion =============================*/
/**
 * FSM State for robot locomotion. Manages the contact specific logic
 * and handles calling the interfaces to the controllers. This state
 * should be independent of controller, gait, and desired trajectory.
 */

#include "FSM_State_LocomotionRPC.h"

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_LocomotionRPC<T>::FSM_State_LocomotionRPC(
  ControlFSMData<T>* _controlFSMData)
  : FSM_State<T>(_controlFSMData, FSM_StateName::LOCOMOTION_RPC, "LOCOMOTION_RPC") {
  // Set the safety checks
  this->turnOnAllSafetyChecks();
  this->checkPDesFoot = false;

  this->_wbc_ctrl = new LocomotionCtrl<T>(_controlFSMData);
  this->_wbc_data = new LocomotionCtrlData<T>();
  //this->_wbc_ctrl->setFloatingBaseWeight(0.001);
  //this->_wbc_ctrl->setReactionForceWeight(1000.);

  //this->_wbc_ctrl->setFloatingBaseWeight(0.1);
  //this->_wbc_ctrl->setReactionForceWeight(1.);
}


/**
 * Initialize the data with the current state of the robot when entering the mode.
 */
template <typename T>
void FSM_State_LocomotionRPC<T>::onEnter() {
  this->naturalGaitState = NaturalGaitState::STANCE;

  // Initialize desired states when switching into the RPC mode
  this->RPC_Interface->px_des = (double)this->_data->_stateEstimator->getResult().position(0);
  this->RPC_Interface->py_des = (double)this->_data->_stateEstimator->getResult().position(1);
  this->RPC_Interface->yaw_des = (double)this->_data->_stateEstimator->getResult().rpy(2);

  // Initialize the touchdown and current foot positions, as well as swing trajectories
  for (int leg = 0; leg < this->_data->_pat->NUM_FEET; leg++) {
    this->footPositionsCurr.col(leg) = this->_data->_stateEstimator->getFootPosWorld(leg);
    this->footPositionsTD.col(leg) = this->footPositionsCurr.col(leg);

    // Set the footswing trajectory
    this->SetFootswingTrajectories(leg, 
        this->footPositionsCurr.col(leg), this->footPositionsCurr.col(leg), 0.0);
  }

  // Start in normal stand if natural gait modification is selected
  if (this->_data->userParameters->gait_override == 3 ||
      this->_data->userParameters->gait_override == 4) {
    this->_data->_gaitScheduler->gaitData._nextGait = GaitType::STAND;
  } else {
    this->_data->_gaitScheduler->gaitData._nextGait = GaitType::TROT;
  }

  printf("[LOCOMOTION_RPC] On Enter\n");
}


/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_LocomotionRPC<T>::run() {
  // Zero the leg controller commands at the start of each itteration
  this->ZeroLegControllerCommands();

  // Call the locomotion control logic for this iteration
  LocomotionControlStepRPC();
}




/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_LocomotionRPC<T>::onExit() {
  // Nothing to clean up when exiting
  iter = 0;

  this->RPC_Interface->px_des = 0;
  this->RPC_Interface->py_des = 0;
}


/**
 * Calculate the commands for the leg controllers for each of the feet by
 * calling the appropriate balance controller and parsing the results for
 * each stance or swing leg.
 */
template <typename T>
void FSM_State_LocomotionRPC<T>::LocomotionControlStepRPC() {
  // Run the RPC
  this->runRegularizedPredictiveController();

  // Run the WBC
  if (this->_data->userParameters->use_wbc) {
    this->runWholeBodyController();
  } else {
    // Finalize the RPC step by setting the leg controller commands
    this->SetLegControllerCommands();
  }

  // Visualize the prediction results, always do this last
  this->VisualizePredictionRPC();
  this->VisualizeSwingLegTrajectory();

  //pretty_print(this->_data->_desiredStateCommand->data.stateDes, std::cout, "RPC state des state");
}


// template class FSM_State_Locomotion<double>;
template class FSM_State_LocomotionRPC<float>;
