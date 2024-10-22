/*! @file FullStateEstimator.h
 *  @brief All State Estimation Algorithms
 *
 *  This file will contain all state estimation algorithms.
 *  FullStateEstimator should compute:
 *  - body position/velocity in world/body frames
 *  - foot positions/velocities in body/world frame
 *  - orientation: a quaternion representing orientation
 *  - rBody: coordinate transformation matrix (satisfies vBody = Rbody * vWorld)
 *  - omegaBody: angular velocity in body frame
 *  - omegaWorld: angular velocity in world frame
 *  - rpy: roll pitch yaw
 */

#ifndef PROJECT_FULLSTATEESTIMATOR_H
#define PROJECT_FULLSTATEESTIMATOR_H

#include "StateEstimatorContainer.h"
#include "InEKF.h"

/*!
 * Position and velocity estimator based on a Kalman Filter.
 * This is the algorithm used in Mini Cheetah and Cheetah 3.
 */
template <typename T>
class FullStateEstimator : public GenericEstimator<T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FullStateEstimator();
  virtual void run();
  virtual void setup();

  private:
    int iter = 0;

    inekf::InEKF filter;

    Eigen::Matrix<double,6,1> imu_measurement;
    Eigen::Matrix<double,6,1> imu_measurement_prev;

    Eigen::Vector3d p_glb;

};

/*!
 * "Cheater" position and velocity estimator which will return the correct position and
 * velocity when running in simulation.
 */
template<typename T>
class CheaterFullStateEstimator : public GenericEstimator<T> {
public:
  virtual void run();
  virtual void setup() {}
};

#endif  // PROJECT_FULLSTATEESTIMATOR_H
