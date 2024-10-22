/*! @file IMU_Simulator.cpp
 *  @brief Simulated IMU
 */
#include "IMU_Simulator.h"
#include <Utilities/spatial.h>
#include <Utilities/orientation_tools.h>
#include <Utilities/utilities.h>
#include <Utilities/pretty_print.h>
/*!
 * Compute acceleration that accelerometer will report
 * @param robotState : state of model
 * @param robotStateD : state derivative of model
 * @param acc : result acceleration
 * @param dist : random distribution
 * @param R_body : orientation of body
 */
template <typename T>
void IMU_Simulator<T>::_computeAcceleration(
    const FBModelState<T> &robotState,
    const FBModelStateDerivative<T> &robotStateD, Vec3<float> &acc,
    std::uniform_real_distribution<float> &dist, const RotMat<float> &R_body) {
  // accelerometer noise
  fillEigenWithRandom(acc, _mt, dist);

  // gravity (should be positive when robot is upright)
  // pretty_print(acc, std::cout, "noise");
  acc += (R_body * Vec3<float>(0, 0, 9.81));
  // pretty_print(acc, std::cout, "noise + G acc");
  // pretty_print(robotState.bodyVelocity, std::cout, "model vel");
  // pretty_print(robotStateD.dBodyVelocity, std::cout, "model acc");
  // acceleration
  acc += spatial::spatialToLinearAcceleration(robotStateD.dBodyVelocity,
                                              robotState.bodyVelocity).template cast<float>();
  // pretty_print(acc, std::cout, "noise + G acc + spatial");
}

/*!
 * Compute acceleration, gyro, and orientation readings from VectorNav
 */
template <typename T>
void IMU_Simulator<T>::updateIMU(
    const FBModelState<T> &robotState,
    const FBModelStateDerivative<T> &robotStateD, IMU_Data *data) {
  // body orientation
  RotMat<float> R_body = quaternionToRotationMatrix(
      robotState.bodyOrientation.template cast<float>());

  // acceleration
  _computeAcceleration(robotState, robotStateD, data->accelerometer,
                      _IMU_AccelerometerDistribution, R_body);

  // gyro
  fillEigenWithRandom(data->gyro, _mt, _IMU_GyroDistribution);
  data->gyro +=
      robotState.bodyVelocity.template head<3>().template cast<float>();
  // quaternion
  if (_IMU_OrientationNoise) {
    Vec3<float> omegaNoise;
    fillEigenWithRandom(omegaNoise, _mt, _IMU_QuatDistribution);
    Quat<float> floatQuat = robotState.bodyOrientation.template cast<float>();
    data->quat = integrateQuat(floatQuat, omegaNoise, 1.0f);
  } else {
    data->quat = robotState.bodyOrientation.template cast<float>();
  }
}

/*!
 * Update cheater state from simulator state
 */
template <typename T>
void IMU_Simulator<T>::updateCheaterState(
    const FBModelState<T> &robotState,
    const FBModelStateDerivative<T> &robotStateD, CheaterState &state) {
  RotMat<T> R_body = quaternionToRotationMatrix(robotState.bodyOrientation);
  state.acceleration = (R_body * Vec3<T>(0, 0, 9.81)) +
                       spatial::spatialToLinearAcceleration(
                           robotStateD.dBodyVelocity, robotState.bodyVelocity);
  state.orientation = robotState.bodyOrientation;
  state.position = robotState.bodyPosition;
  state.omegaBody = robotState.bodyVelocity.template head<3>();
  state.vBody = robotState.bodyVelocity.template tail<3>();
}

//template class IMU_Simulator<float>;
template class IMU_Simulator<double>;
