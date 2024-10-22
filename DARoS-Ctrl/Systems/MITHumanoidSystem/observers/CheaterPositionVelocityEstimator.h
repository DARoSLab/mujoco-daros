/*! @file CheaterPositionVelocityEstimator.h
 *  @brief All State Estimation Algorithms
 *
 *  This file is a temporary placeholder for humanoid
 *  state estimation until the LinearKFPositionEstimator
 *  can be abstracted to common
 */

#ifndef PROJECT_CHEATERPOSITIONVELOCITYESTIMATOR_H
#define PROJECT_CHEATERPOSITIONVELOCITYESTIMATOR_H

#include "StateEstimatorContainer.h"

/*!
 * "Cheater" position and velocity estimator which will return the correct position and
 * velocity when running in simulation.
 */
template<typename T>
class CheaterPositionVelocityEstimator : public GenericEstimator<T> {
public:
  virtual void run();
  virtual void setup() {}
};

#endif  // PROJECT_CHEATERPOSITIONVELOCITYESTIMATOR_H
