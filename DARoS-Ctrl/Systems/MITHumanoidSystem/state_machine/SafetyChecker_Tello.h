#ifndef SAFETY_CHECKER_Tello_H
#define SAFETY_CHECKER_Tello_H

#include <iostream>

// Contains all of the control related data
#include "ControlFSMData_Tello.h"

/**
 * The SafetyChecker handles the checks requested by the ControlFSM.
 */
template <typename T>
class SafetyChecker_Tello {
 public:
  SafetyChecker_Tello(ControlFSMData_Tello<T>* dataIn) : data(dataIn){};

  // Pre checks to make sure controls are safe to run
  bool checkSafeOrientation();  // robot's orientation is safe to control

  // Post checks to make sure controls can be sent to robot
  bool checkPDesFoot();          // desired foot position is not too far
  bool checkForceFeedForward();  // desired feedforward forces are not too large

  // Stores the data from the ControlFSM
  ControlFSMData_Tello<T>* data;

 private:
};

#endif  // SAFETY_CHECKER_H
