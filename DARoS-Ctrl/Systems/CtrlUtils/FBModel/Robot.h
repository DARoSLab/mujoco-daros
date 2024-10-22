/*! @file Robot.h
 *  @brief Data structure containing FloatingBased Model and Actuator Model
 */

#ifndef ROBOT_H
#define ROBOT_H

#include <vector>
#include "ActuatorModel.h"
#include "FloatingBaseModel.h"

template <typename T>
class Robot {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Robot(){}
  virtual ~Robot(){}

  virtual FloatingBaseModel<T> buildModel(T gravity = -9.81) const = 0;
  // virtual std::vector<ActuatorModel<T>*> buildActuatorModels() const = 0;
  virtual void getInitialState(FBModelState<T> & x0) const = 0;
  // virtual int numActJoint() const = 0;
  // int NUM_FEET = -1;
};

#endif  // ROBOT_H
