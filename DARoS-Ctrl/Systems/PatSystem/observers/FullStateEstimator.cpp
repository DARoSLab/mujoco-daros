/*! @file FullStateEstimator.cpp
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

#include "FullStateEstimator.h"
#include <robots/PatBiped.h>

/*!
 * Initialize the state estimator
 */
template <typename T>
void FullStateEstimator<T>::setup() {

    //  ---- Initialize invariant extended Kalman filter ----- //
    inekf::RobotState initial_state; 

    // Initialize state mean
    Eigen::Matrix3d R0;
    Eigen::Vector3d v0, p0, bg0, ba0;
    R0 << 1, 0, 0, // initial orientation
          0, 1, 0,
          0, 0, 1;
    v0 << 0,0,0; // initial velocity
    p0 << 0,0,0; // initial position
    bg0 << 0,0,0; // initial gyroscope bias
    ba0 << 0,0,0; // initial accelerometer bias
    initial_state.setRotation(R0);
    initial_state.setVelocity(v0);
    initial_state.setPosition(p0);
    initial_state.setGyroscopeBias(bg0);
    initial_state.setAccelerometerBias(ba0);

    // Initialize state covariance
    inekf::NoiseParams noise_params;
    noise_params.setGyroscopeNoise(0.01);
    noise_params.setAccelerometerNoise(0.1);
    noise_params.setGyroscopeBiasNoise(0.00001);
    noise_params.setAccelerometerBiasNoise(0.00001);
    noise_params.setJointEncoderNoise(0.0001);
    noise_params.setContactNoise(0.025);
    noise_params.setLocalizationNoise(0.005);

    // Initialize filter
    filter.setState(initial_state);
    filter.setNoiseParams(noise_params);
    // std::cout << "Noise parameters are initialized to: \n";
    // std::cout << filter.getNoiseParams() << std::endl;
    // std::cout << "Robot's state is initialized to: \n";
    // std::cout << filter.getState() << std::endl;

    // Initialize IMU measurement
    imu_measurement = Eigen::Matrix<double,6,1>::Zero();
    imu_measurement_prev = Eigen::Matrix<double,6,1>::Zero();

}

template <typename T>
FullStateEstimator<T>::FullStateEstimator() {}

/*!
 * Run state estimator
 */
template <typename T>
void FullStateEstimator<T>::run() {
  iter++;

  // Update IMU Measurement
  Vec3<T> aBody, omegaBody;
  omegaBody = this->_stateEstimatorData->>vectorNavData->gyro.template cast<T>();
  this->_stateEstimatorData->>result.omegaBody = omegaBody;
  aBody = this->_stateEstimatorData->>vectorNavData->accelerometer.template cast<T>();
  this->_stateEstimatorData->>result.aBody = aBody;
  for (int i = 0; i < 3; i++){
    imu_measurement[i] =  omegaBody[i]; //omegaBody
    imu_measurement[i+3] =  aBody[i]; // aBody
  }

  // Propogate the dynamics
  filter.Propagate(imu_measurement_prev, this->_stateEstimatorData->>parameters->estimator_dt);

  // Set the contacts
  vector<std::pair<int,bool> > contacts;
  int id;
  bool indicator;
  double phase;

  for (size_t i = 0; i < pat_biped::num_legs; i++){ // Generalize for any number of legs (e.g. humanoid)
    id = i;
    phase = fmin(this->_stateEstimatorData->>result.contactEstimate(i), T(1)); // contact phase from contact estimator
    if (phase > 0.275 && phase < 0.725) // contact estimator trust region
      indicator = true;
    else
      indicator = false;
    contacts.push_back(std::pair<int,bool> (id, indicator));

  }
  filter.setContacts(contacts);

  // Use leg controller to get positon of contacts relative to body frame
  // Should be variable size depending on the number of legs
  inekf::vectorKinematics measured_kinematics;
  for (size_t i = 0; i < pat_biped::num_legs; i++){
    // Leg index
    id = i;

    // Leg Pose (transformation from foot frame to body frame)
    Eigen::Matrix3d Rfoot = Eigen::Matrix3d::Identity(); // rotation matrix from foot to body frame (can remove this)
    Eigen::Vector3d p; // position of contact relative to the body
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  
    Vec3<T> ph = this->_stateEstimatorData->>robot->getHipLocation(i);  // hip positions relative to CoM>>
    Vec3<T> p_rel = ph + this->_stateEstimatorData->>limbData[i]->p; // position of contact relative to CoM
    
    for (int j = 0; j < 3; j++) {p[j] = p_rel[j];}

    pose.block<3,3>(0,0) = Rfoot;
    pose.block<3,1>(0,3) = p;

    // Leg Pose Covariance
    Eigen::Matrix3d covariance;
    Eigen::Matrix3d Jleg;

    for (int j = 0; j < 3; j++)
      for (int k = 0; k < 3; k++)
        Jleg(j,k) = this->_stateEstimatorData->>limbData[i]->J(j,k);

    covariance = Jleg * filter.getNoiseParams().getJointEncoderCov() * Jleg.transpose();

    inekf::Kinematics frame(id, pose, covariance);
    measured_kinematics.push_back(frame);
  }

  /*** Use kinematic and localization correction - get position of robot body in standard worl frame from localization sensor ***/
  // Global position of robot in standard world frame
  p_glb[0] = this->_stateEstimatorData->>localizationData->positionRobot[0][0];
  p_glb[1] = this->_stateEstimatorData->>localizationData->positionRobot[0][1];
  p_glb[2] = this->_stateEstimatorData->>localizationData->positionRobot[0][2];

  // Localization sensor covariance
  Eigen::Matrix3d covariance;
  covariance = filter.getNoiseParams().getLocalizationCov();
  // Correction step
  filter.CorrectKinematicsAndLocalization(measured_kinematics, p_glb, covariance);
  this->_stateEstimatorData->>result.position[2] = filter.getState().getPosition()[2];


  /*** Kinematic correction only - assumes flat ground ***/
  // // Use this if we do not have localization data (so we assume flat ground)
  // filter.CorrectKinematics(measured_kinematics);
  // if (filter.getState().dimX() > 5){
  //   // Set correct height of body
  //   this->_stateEstimatorData->>result.position[2] = filter.getState().getRelativeHeight();
  // }
  
  // Log new state estimate
  this->_stateEstimatorData->>result.position[0] = filter.getState().getPosition()[0];
  this->_stateEstimatorData->>result.position[1] = filter.getState().getPosition()[1];
  for (int i = 0; i < 3; i++){
    this->_stateEstimatorData->>result.vWorld[i] = filter.getState().getVelocity()[i];
    for (int j = 0; j < 3; j++)
      this->_stateEstimatorData->>result.rBody(i,j) = filter.getState().getRotation()(j,i);
  }
  this->_stateEstimatorData->>result.rpy = ori::rotationMatrixToRPY(this->_stateEstimatorData->>result.rBody);
  this->_stateEstimatorData->>result.orientation = ori::rpyToQuat(this->_stateEstimatorData->>result.rpy);

  // Log IMU measurement
  imu_measurement_prev = imu_measurement;

}

template class FullStateEstimator<float>;
template class FullStateEstimator<double>;


/*!
 * Run cheater estimator to copy cheater state into state estimate
 */
template <typename T>
void CheaterFullStateEstimator<T>::run() {
  this->_stateEstimatorData->>result.position = this->_stateEstimatorData->>cheaterState->position.template cast<T>();
  this->_stateEstimatorData->>result.vWorld =
      this->_stateEstimatorData->>result.rBody.transpose().template cast<T>() * this->_stateEstimatorData->>cheaterState->vBody.template cast<T>();
  this->_stateEstimatorData->>result.vBody = this->_stateEstimatorData->>cheaterState->vBody.template cast<T>();
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

template class CheaterFullStateEstimator<float>;
template class CheaterFullStateEstimator<double>;
