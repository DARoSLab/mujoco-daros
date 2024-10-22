/*! @file PositionVelocityEstimator.h
 *  @brief All State Estimation Algorithms
 *
 *  This file will contain all state estimation algorithms.
 *  PositionVelocityEstimators should compute:
 *  - body position/velocity in world/body frames
 *  - foot positions/velocities in body/world frame
 */

#ifndef PROJECT_POSITIONVELOCITYESTIMATOR_H
#define PROJECT_POSITIONVELOCITYESTIMATOR_H

#include "StateEstimatorContainer.h"
#include <robots/PatBiped.h>
#define ROBOT_NUM_FEET 2

/*!
 * Position and velocity estimator based on a Kalman Filter.
 * This is the algorithm used in Mini Cheetah and Cheetah 3.
 */
template <typename T>
class LinearKFPositionVelocityEstimator : public GenericEstimator<T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LinearKFPositionVelocityEstimator();
  virtual void run();
  virtual void setup();

 private:
  // Eigen::Matrix<T, 18, 1> _xhat; // [pos estimate, vel estimate, foot location estimates] all in world coorindates
  // Eigen::Matrix<T, 12, 1> _ps; // relative distance from body to each foot based on kinematics, expressed in world coords
  // Eigen::Matrix<T, 12, 1> _vs; // relative velocity of each of the feet w.r.t the body, expressed in world coords
  // Eigen::Matrix<T, 18, 18> _A; // dynamics
  // Eigen::Matrix<T, 18, 18> _Q0; // Process noise covariance matrix
  // Eigen::Matrix<T, 18, 18> _P; // state covariance matrix
  // Eigen::Matrix<T, 28, 28> _R0; // Measurement noise matrix
  // Eigen::Matrix<T, 18, 3> _B;
  // Eigen::Matrix<T, 28, 18> _C; // Measurement Jacobian
  //

  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> _xhat; // [pos estimate, vel estimate, foot location estimates] all in world coorindates
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> _xhat_fk; // [pos estimate, vel estimate, foot location estimates] all in world coorindates
  Eigen::Matrix<T, Eigen::Dynamic, 1> _ps; // relative distance from body to each foot based on kinematics, expressed in world coords
  Eigen::Matrix<T, Eigen::Dynamic, 1> _vs; // relative velocity of each of the feet w.r.t the body, expressed in world coords
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> _A; // dynamics
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> _Q0; // Process noise covariance matrix
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> _P; // state covariance matrix
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> _R0; // Measurement noise matrix
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> _B;
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> _C; // Measurement Jacobian
};

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

#endif  // PROJECT_POSITIONVELOCITYESTIMATOR_H
