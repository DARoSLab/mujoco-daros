/*! @file OrientationEstimator.cpp
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

#include "PatOrientationEstimator.h"
#include <Utilities/pretty_print.h>
/*!
 * Get quaternion, rotation matrix, angular velocity (body and world),
 * rpy, acceleration (world, body) by copying from cheater state data
 */
template <typename T>
void PatCheaterOrientationEstimator<T>::run() {
  //pretty_print(this->_stateEstimatorData->>cheaterState->orientation, std::cout, "cheater quat");

  this->_stateEstimatorData->>result.orientation =
      this->_stateEstimatorData->>cheaterState->orientation.template cast<T>();
  this->_stateEstimatorData->>result.rBody = ori::quaternionToRotationMatrix(
      this->_stateEstimatorData->>result.orientation);
  this->_stateEstimatorData->>result.omegaBody =
      this->_stateEstimatorData->>cheaterState->omegaBody.template cast<T>();
  this->_stateEstimatorData->>result.omegaWorld =
      this->_stateEstimatorData->>result.rBody.transpose() *
      this->_stateEstimatorData->>result.omegaBody;
  this->_stateEstimatorData->>result.rpy =
      ori::quatToRPY(this->_stateEstimatorData->>result.orientation);
  this->_stateEstimatorData->>result.aBody =
      this->_stateEstimatorData->>cheaterState->acceleration.template cast<T>();
  this->_stateEstimatorData->>result.aWorld =
      this->_stateEstimatorData->>result.rBody.transpose() *
      this->_stateEstimatorData->>result.aBody;
}

/*!
 * Get quaternion, rotation matrix, angular velocity (body and world),
 * rpy, acceleration (world, body) from vector nav IMU
 */
template <typename T>
void PatVectorNavOrientationEstimator<T>::run() {
   this->_stateEstimatorData->>result.orientation[0] =
      this->_stateEstimatorData->>vectorNavData->quat[3];
   this->_stateEstimatorData->>result.orientation[1] =
      this->_stateEstimatorData->>vectorNavData->quat[0];
   this->_stateEstimatorData->>result.orientation[2] =
      this->_stateEstimatorData->>vectorNavData->quat[1];
   this->_stateEstimatorData->>result.orientation[3] =
     this->_stateEstimatorData->>vectorNavData->quat[2];

  Vec4<T> corrected_quat;
  Mat3<T> g_R_I, imu_R_b;
  Mat3<T> I_R_imu;
  g_R_I << 1, 0, 0, 0, -1, 0, 0, 0, -1;//me
  imu_R_b << 0, 0, 1, 0, -1, 0, 1, 0, 0;//me

  // if(_b_first_visit){
  //   I_R_imu = quaternionToRotationMatrix(this->_stateEstimatorData->>result.orientation).transpose();
  //   Vec3<T> rpy_ini = ori::quatToRPY(ori::rotationMatrixToQuaternion((g_R_I*I_R_imu*imu_R_b).transpose()));
  //   rpy_ini[0] = 0;
  //   rpy_ini[1] = 0;
  //   _ori_ini_inv = ori::rpyToQuat(-rpy_ini);
  //   _b_first_visit = false;
  //   std::cout << "rpy_ini " << rpy_ini << '\n';
  // }

  I_R_imu = quaternionToRotationMatrix(this->_stateEstimatorData->>result.orientation).transpose();
  corrected_quat = ori::rotationMatrixToQuaternion((g_R_I*I_R_imu*imu_R_b).transpose());
  // corrected_quat = ori::quatProduct(_ori_ini_inv, corrected_quat);


  // static int p_iter = 0;
  // if(p_iter++ % 100 ==0 )
  // {
  //   std::cout << "R raw: " <<quaternionToRotationMatrix(this->_stateEstimatorData->>result.orientation).transpose() << '\n';
  //   std::cout << "g_R_b: \n" << g_R_I*I_R_imu*imu_R_b << "\n\n";
  //   std::cout << "g_q_b: \n" << corrected_quat << "\n\n";
  // }
  this->_stateEstimatorData->>result.orientation = corrected_quat;

  this->_stateEstimatorData->>result.rpy =
      ori::quatToRPY(this->_stateEstimatorData->>result.orientation);

  this->_stateEstimatorData->>result.rBody = ori::quaternionToRotationMatrix(
      this->_stateEstimatorData->>result.orientation);

  this->_stateEstimatorData->>result.omegaBody =
      this->_stateEstimatorData->>vectorNavData->gyro.template cast<T>();

  this->_stateEstimatorData->>result.omegaWorld =
      this->_stateEstimatorData->>result.rBody.transpose() *
      this->_stateEstimatorData->>result.omegaBody;

  this->_stateEstimatorData->>result.aBody =
      this->_stateEstimatorData->>vectorNavData->accelerometer.template cast<T>();
  this->_stateEstimatorData->>result.aWorld =
      this->_stateEstimatorData->>result.rBody.transpose() *
      this->_stateEstimatorData->>result.aBody;
}


template class PatCheaterOrientationEstimator<float>;
template class PatCheaterOrientationEstimator<double>;

template class PatVectorNavOrientationEstimator<float>;
template class PatVectorNavOrientationEstimator<double>;
