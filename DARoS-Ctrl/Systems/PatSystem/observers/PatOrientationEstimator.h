/*! @file OrientationEstimator.h
 *  @brief All Orientation Estimation Algorithms
 *
 *  This file will contain all orientation algorithms.
 *  Orientation estimators should compute:
 *  - orientation: a quaternion representing orientation
 *  - rBody: coordinate transformation matrix (satisfies vBody = Rbody * vWorld)
 *  - omegaBody: angular velocity in body frame
 *  - omegaWorld: angular velocity in world frame
 *  - rpy: roll pitch yaw
 */
#ifndef PROJECT_PAT_ORIENTATIONESTIMATOR_H
#define PROJECT_PAT_ORIENTATIONESTIMATOR_H

#include "common/estimators/StateEstimatorContainer.h"

/*!
 * "Cheater" estimator for orientation which always returns the correct value in simulation
 */
template <typename T>
class PatCheaterOrientationEstimator : public GenericEstimator<T> {
 public:
  virtual void run();
  virtual void setup() {}
};

/*!
 * Estimator for the VectorNav IMU.  The VectorNav provides an orientation already and
 * we just return that.
 */
template <typename T>
class PatVectorNavOrientationEstimator : public GenericEstimator<T> {
 public:
  virtual void run();
  virtual void setup() {}

 protected:
  bool _b_first_visit = true;
  Quat<T> _ori_ini_inv;
};


#endif  // PROJECT_ORIENTATIONESTIMATOR_H
