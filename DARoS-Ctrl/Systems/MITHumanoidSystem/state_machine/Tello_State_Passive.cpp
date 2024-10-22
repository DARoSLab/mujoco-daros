/*============================== Passive ==============================*/
/**
 * FSM State that calls no controls. Meant to be a safe state where the
 * robot should not do anything as all commands will be set to 0.
 */

#include "Tello_State_Passive.h"

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
Tello_State_Passive<T>::Tello_State_Passive(ControlFSMData_Tello<T>* _controlFSMData)
    : Tello_State<T>(_controlFSMData, Tello_StateName::PASSIVE, "PASSIVE") {
  // Do nothing
  // Set the pre controls safety checks
  this->checkSafeOrientation = false;

  // Post control safety checks
  this->checkPDesFoot = false;
  this->checkForceFeedForward = false;
}

template <typename T>
void Tello_State_Passive<T>::onEnter() {
  // Default is to not transition
  this->nextStateName = this->stateName;

  // Reset the transition data
  this->transitionData.zero();
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void Tello_State_Passive<T>::run() {
  this->UpdateModel();
  this->viz_heel_toe();
  // Do nothing, all commands should begin as zeros
  for(size_t i(0); i<tello::num_joint_group; ++i){
    this->_fsm_data->_jointController->_commands[i]->zero();
  }

  std::cout<<"shoulder_roll_R \n"<<  this->_model->_state.q[tello::shoulder_roll_R] << std::endl;
  std::cout<<"shoulder_yaw_R \n"<<  this->_model->_state.q[tello::shoulder_yaw_R] << std::endl;
  std::cout<<"shoulder_pitch_R \n"<<  this->_model->_state.q[tello::shoulder_pitch_R] << std::endl;
  std::cout<<"elbow_R \n"<<  this->_model->_state.q[tello::elbow_R] << std::endl;

}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template <typename T>
Tello_StateName Tello_State_Passive<T>::checkTransition() {
  iter++;
  return (Tello_StateName)this->_fsm_data->_userParameters->control_mode;
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template <typename T>
TransitionData_Tello<T> Tello_State_Passive<T>::transition() {
  // Finish Transition
  this->transitionData.done = true;

  // Return the transition data to the FSM
  return this->transitionData;
}

/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void Tello_State_Passive<T>::onExit() {
  // Nothing to clean up when exiting
}

template class Tello_State_Passive<float>;
