/*============================= Joint PD ==============================*/
#include "Tello_State_JointPD.h"
#include <Configuration.h>

/**
 * Constructor for the Tello State that passes in state specific info to
 * the generic Tello State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
Tello_State_JointPD<T>::Tello_State_JointPD(ControlFSMData_Tello<T>* _controlFSMData)
    : Tello_State<T>(_controlFSMData, Tello_StateName::JOINT_PD, "JOINT_PD"),
_ini_jpos(tello::num_act_joint){
  // Do nothing here yet
  _ini_jpos.setZero();
}

template <typename T>
void Tello_State_JointPD<T>::onEnter() {
  // Default is to not transition
  this->nextStateName = this->stateName;

  // Reset the transition data
  this->transitionData.zero();

  // Reset counter
  iter = 0;
  _ini_jpos = this->get_current_jpos();
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void Tello_State_JointPD<T>::run() {
  this->UpdateModel();
  this->viz_heel_toe();

  DVec<T> qdDes_leg(tello::num_leg_joint);
  DVec<T> qdDes_arm(tello::num_arm_joint);;
  qdDes_leg.setZero();
  qdDes_arm.setZero();

  // matrix gains for leg PDcontrol
  DVec<T> kp_leg_vec(tello::num_leg_joint);
  DVec<T> kd_leg_vec(tello::num_leg_joint);
  for(size_t i(0); i<tello::num_leg_joint; ++i){
    kp_leg_vec[i] = 20.;
    kd_leg_vec[i] = 0.2;
  }
  kp_leg_vec[1] = 100.;
  kd_leg_vec[1] = 2.;

  kp_leg_vec[2] = 170.;
  kd_leg_vec[2] = 2.;

// Knee
  kp_leg_vec[3] = 235.;
  kd_leg_vec[3] = 2.;

  kp_leg_vec[4] = 180.;
  kd_leg_vec[4] = 6.;


  //kp_leg_vec[6] = 0.;
  //kd_leg_vec[6] = 0.;
  DMat<T> kp_leg = kp_leg_vec.array().sqrt().matrix().asDiagonal();
  DMat<T> kd_leg = kp_leg_vec.array().sqrt().matrix().asDiagonal();
  

  // matrix gains for arm
  DVec<T> kp_arm_vec(tello::num_arm_joint);
  DVec<T> kd_arm_vec(tello::num_arm_joint);
  for(size_t i(0); i<tello::num_arm_joint; ++i){
    kp_arm_vec[i] = 20.;
    kd_arm_vec[i] = 0.2;
  }
  DMat<T> kp_arm = kp_arm_vec.array().sqrt().matrix().asDiagonal();
  DMat<T> kd_arm = kd_arm_vec.array().sqrt().matrix().asDiagonal();

  // run jointPD Control for each limb
  this->jointPDControl(0, _ini_jpos.head(tello::num_leg_joint), qdDes_leg, kp_leg, kd_leg); // leg1
  this->jointPDControl(1, _ini_jpos.segment(tello::num_leg_joint, tello::num_leg_joint), qdDes_leg,kp_leg,kd_leg); // leg2
                                                                                                                       
  DVec<T> qd_des(1); qd_des.setZero();
  DMat<T> kp_torso(1,1); kp_torso(0,0) = 20.;
  DMat<T> kd_torso(1,1); kd_torso(0,0) = 0.2;
  this->jointPDControl(2, _ini_jpos.segment(2*tello::num_leg_joint, 1), qd_des, kp_torso, kd_torso); // torso
                                                                                                    
  this->jointPDControl(3, _ini_jpos.segment(2*tello::num_leg_joint + 1, tello::num_arm_joint), qdDes_arm,kp_arm,kd_arm); // arm1
  this->jointPDControl(4, _ini_jpos.segment(
        2*tello::num_leg_joint + 1 + tello::num_arm_joint, tello::num_arm_joint), qdDes_arm,kp_arm,kd_arm); // arm2
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated Tello state name to transition into
 */
template <typename T>
Tello_StateName Tello_State_JointPD<T>::checkTransition() {
  this->nextStateName = this->stateName;

  iter++;

  // Switch Tello control mode
  switch ((int)this->_fsm_data->_userParameters->control_mode) {
    case Tello_StateName::JOINT_PD:
      break;

    //case Tello_StateName::BALANCE_STAND:
      //this->nextStateName = Tello_StateName::BALANCE_STAND;
      //break;

    case Tello_StateName::PASSIVE:
      // Requested change to BALANCE_STAND
      this->nextStateName = Tello_StateName::PASSIVE;

      // Transition time is immediate
      this->transitionDuration = 0.0;

      break;

    default:
      std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                << (int)Tello_StateName::JOINT_PD << " to "
                << this->_fsm_data->_userParameters->control_mode << std::endl;
  }

  // Get the next state
  return this->nextStateName;
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template <typename T>
TransitionData_Tello<T> Tello_State_JointPD<T>::transition() {
  // Switch Tello control mode
  switch (this->nextStateName) {

    //case Tello_StateName::BALANCE_STAND:
      //this->transitionData.done = true;
      //break;

    case Tello_StateName::PASSIVE:
      this->turnOffAllSafetyChecks();
      this->transitionData.done = true;
      break;

    default:
      std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                << (int)Tello_StateName::JOINT_PD << " to "
                << this->_fsm_data->_userParameters->control_mode << std::endl;
  }
  // Finish transition
  this->transitionData.done = true;

  // Return the transition data to the Tello
  return this->transitionData;
}

/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void Tello_State_JointPD<T>::onExit() {
  // Nothing to clean up when exiting
}

//template class Tello_State_JointPD<double>;
template class Tello_State_JointPD<float>;
