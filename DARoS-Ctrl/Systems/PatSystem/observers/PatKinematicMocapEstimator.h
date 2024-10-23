/*! @file PatKinematicMocapEstimator.h
 *  @brief All State Estimation Algorithms
 *
 *  This file will contain all state estimation algorithms.
 *  PatKinematicMocapEstimators should compute:
 *  - body position/velocity in world/body frames
 *  - foot positions/velocities in body/world frame
 */

#ifndef PROJECT_PATKINEMATICMOCAPESTIMATOR_H
#define PROJECT_PATKINEMATICMOCAPESTIMATOR_H

#include "StateEstimatorContainer.h"
#include <robots/PatBiped.h>
#include <FBModel/FloatingBaseModel.h>
#include <lcm-cpp.hpp>
#include "pat_mocap_kin_lcmt.hpp"

/*!
 * Position and velocity estimator based on a Kalman Filter.
 * This is the algorithm used in Mini Cheetah and Cheetah 3.
 */
template <typename T>
class PatKinematicMocapEstimator : public GenericEstimator<T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PatKinematicMocapEstimator();
  virtual void run();
  virtual void setup();

 private:
   lcm::LCM _lcm;
   int iter;
   FloatingBaseModel<T> _model;
   FBModelState<T> _state;
   PatBiped<T>* _pat = NULL;
   int _stance_foot_id;
   pat_mocap_kin_lcmt _kin_mocap_lcm;


   void _UpdateModel();
};

/*!
 * "Cheater" position and velocity estimator which will return the correct position and
 * velocity when running in simulation.
 */
template<typename T>
class CheaterPatKinematicMocapEstimator : public GenericEstimator<T> {
public:
  virtual void run();
  virtual void setup() {}
};

#endif  // PROJECT_PATKINEMATICMOCAPESTIMATOR_H
