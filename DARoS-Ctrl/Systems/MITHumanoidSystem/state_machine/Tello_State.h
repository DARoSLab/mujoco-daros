#ifndef Tello_State_H
#define Tello_State_H

#include <stdio.h>

#include "ControlFSM_Tello.h"
#include "ControlFSMData_Tello.h"
#include "TransitionData_Tello.h"

template <typename T>
class Tello_State {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Generic constructor for all states
  Tello_State(ControlFSMData_Tello<T>* _controlFSMData,
            Tello_StateName stateNameIn,
            std::string stateStringIn);

  // Behavior to be carried out when entering a state
  virtual void onEnter() = 0;// {}

  // Run the normal behavior for the state
  virtual void run() = 0; //{}

  // Manages state specific transitions
  virtual Tello_StateName checkTransition() = 0;

  // Runs the transition behaviors and returns true when done transitioning
  virtual TransitionData_Tello<T> transition()  = 0;

  // Behavior to be carried out when exiting a state
  virtual void onExit() = 0; // {}

  //
  void turnOnAllSafetyChecks();
  void turnOffAllSafetyChecks();
  bool locomotionSafe();

  // Holds all of the relevant control data
  ControlFSMData_Tello<T>* _fsm_data;

  FBModelState<T> _state;
  FloatingBaseModel<T>* _model;
  void UpdateModel();

  // FSM State info
  Tello_StateName stateName;      // enumerated name of the current state
  Tello_StateName nextStateName;  // enumerated name of the next state
  std::string stateString;      // state name string

  // Transition parameters
  T transitionDuration;  // transition duration time
  T tStartTransition;    // time transition starts
  TransitionData_Tello<T> transitionData;
  bool b_training_complete = false;

  // Kick the robot
  virtual bool checkRobotKicked() { return false; }
  SVec<T> _instantaneous_vel_change;

  // Pre controls safety checks
  bool checkSafeOrientation = false;  // check roll and pitch

  // Post control safety checks
  bool checkPDesFoot = false;          // do not command footsetps too far
  bool checkForceFeedForward = false;  // do not command huge forces
  bool checkLegSingularity = false;    // do not let leg

  // visualization functions
  void viz_heel_toe();

  void jointPDControl(int cluster_idx, DVec<T> qDes, DVec<T> qdDes,DMat<T> kpMat,DMat<T> kdMat);
  DVec<T>  get_current_jpos();

  // Interface with Kino-Dynamci Casadi optimizations
  // KinoDynamicCtrlInterface* KDC_Interface;

};

#endif  // FSM_State_H
