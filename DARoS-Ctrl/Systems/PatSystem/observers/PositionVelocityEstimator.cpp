/*! @file PositionVelocityEstimator.cpp
 *  @brief All State Estimation Algorithms
 *
 *  This file will contain all state estimation algorithms.
 *  PositionVelocityEstimators should compute:
 *  - body position/velocity in world/body frames
 *  - foot positions/velocities in body/world frame
 */

#include "PositionVelocityEstimator.h"
#include <Utilities/pretty_print.h>
/*!
 * Initialize the state estimator
 */
// template <typename T>
// void LinearKFPositionVelocityEstimator<T>::setup() {
//   T dt = this->_stateEstimatorData->>parameters->estimator_dt;
//   _xhat.resize(18, 1);
//   _ps.resize(12);
//   _vs.resize(12, 1);
//   _A.resize(18, 18);
//   _Q0.resize(18, 18);
//   _P.resize(18, 18);
//   _R0.resize(28, 28);
//   _B.resize(18, 3);
//   _C.resize(28, 28);
//   _xhat.setZero();
//   _ps.setZero();
//   _vs.setZero();
//   _A.setZero();
//   _A.block(0, 0, 3, 3) = Eigen::Matrix<T, 3, 3>::Identity();
//   _A.block(0, 3, 3, 3) = dt * Eigen::Matrix<T, 3, 3>::Identity();
//   _A.block(3, 3, 3, 3) = Eigen::Matrix<T, 3, 3>::Identity();
//   _A.block(6, 6, 12, 12) = Eigen::Matrix<T, 12, 12>::Identity();
//   _B.setZero();
//   _B.block(3, 0, 3, 3) = dt * Eigen::Matrix<T, 3, 3>::Identity();
//   Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> C1(3, 6);
//   C1 << Eigen::Matrix<T, 3, 3>::Identity(), Eigen::Matrix<T, 3, 3>::Zero();
//   Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> C2(3, 6);
//   C2 << Eigen::Matrix<T, 3, 3>::Zero(), Eigen::Matrix<T, 3, 3>::Identity();
//   _C.setZero();
//   _C.block(0, 0, 3, 6) = C1;
//   _C.block(3, 0, 3, 6) = C1;
//   _C.block(6, 0, 3, 6) = C1;
//   _C.block(9, 0, 3, 6) = C1;
//   _C.block(0, 6, 12, 12) = T(-1) * Eigen::Matrix<T, 12, 12>::Identity();
//   _C.block(12, 0, 3, 6) = C2;
//   _C.block(15, 0, 3, 6) = C2;
//   _C.block(18, 0, 3, 6) = C2;
//   _C.block(21, 0, 3, 6) = C2;
//   _C(27, 17) = T(1);
//   _C(26, 14) = T(1);
//   _C(25, 11) = T(1);
//   _C(24, 8) = T(1);
//   _P.setIdentity();
//   _P = T(100) * _P;
//   _Q0.setIdentity();
//   _Q0.block(0, 0, 3, 3) = (dt / 20.f) * Eigen::Matrix<T, 3, 3>::Identity();
//   _Q0.block(3, 3, 3, 3) =
//     (dt * 9.8f / 20.f) * Eigen::Matrix<T, 3, 3>::Identity();
//   _Q0.block(6, 6, 12, 12) = dt * Eigen::Matrix<T, 12, 12>::Identity();
//   _R0.setIdentity();
// }
template <typename T>
void LinearKFPositionVelocityEstimator<T>::setup() {
  T dt = this->_stateEstimatorData->>parameters->estimator_dt;
  const int STATE_DIM = 6 + 3*ROBOT_NUM_FEET;
  const int OBS_DIM = 7*ROBOT_NUM_FEET;
  const int INPUT_DIM = 3;
  const int FEET_DIM = 3*ROBOT_NUM_FEET;
  _xhat.resize(STATE_DIM, 1);
  _xhat_fk.resize(STATE_DIM, 1);
  _ps.resize(3*ROBOT_NUM_FEET, 1);
  _vs.resize(3*ROBOT_NUM_FEET, 1);
  _A.resize(STATE_DIM, STATE_DIM);
  _Q0.resize(STATE_DIM, STATE_DIM);
  _P.resize(STATE_DIM, STATE_DIM);
  _R0.resize(OBS_DIM, OBS_DIM);
  _B.resize(STATE_DIM, INPUT_DIM);
  _C.resize(OBS_DIM, STATE_DIM);

  _xhat.setZero();
  _xhat_fk.setZero();
  _ps.setZero();
  _vs.setZero();
  _A.setZero();
  _A.block(0, 0, 3, 3) = Eigen::Matrix<T, 3, 3>::Identity();//position
  _A.block(0, 3, 3, 3) = dt * Eigen::Matrix<T, 3, 3>::Identity();//velocity
  _A.block(3, 3, 3, 3) = Eigen::Matrix<T, 3, 3>::Identity();//velocity
  _A.block(6, 6, FEET_DIM, FEET_DIM) = Eigen::Matrix<T, FEET_DIM, FEET_DIM>::Identity();//foot
  _B.setZero();
  _B.block(3, 0, 3, 3) = dt * Eigen::Matrix<T, 3, 3>::Identity();//velocity
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> C1(3, 6);
  C1 << Eigen::Matrix<T, 3, 3>::Identity(), Eigen::Matrix<T, 3, 3>::Zero();
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> C2(3, 6);
  C2 << Eigen::Matrix<T, 3, 3>::Zero(), Eigen::Matrix<T, 3, 3>::Identity();
  _C.setZero();
  _C.block(0, 0, 3, 6) = C1;
  _C.block(3, 0, 3, 6) = C1;
  _C.block(6, 0, 3, 6) = C2;
  _C.block(9, 0, 3, 6) = C2;
  _C.block(0, 6, FEET_DIM, FEET_DIM) = T(-1) * Eigen::Matrix<T, FEET_DIM, FEET_DIM>::Identity();

  switch (ROBOT_NUM_FEET) {
    case 2:
      _C(12, 8) = T(1);
      _C(13, 11) = T(1);
      break;
    case 4:
      _C.block(12, 0, 3, 6) = C2;
      _C.block(15, 0, 3, 6) = C2;
      _C.block(18, 0, 3, 6) = C2;
      _C.block(21, 0, 3, 6) = C2;
      _C(27, 17) = T(1);
      _C(26, 14) = T(1);
      _C(25, 11) = T(1);
      _C(24, 8) = T(1);
      break;
  }
  _P.setIdentity();
  _P = T(100) * _P;
  _Q0.setIdentity();
  _Q0.block(0, 0, 3, 3) = (dt / 20.f) * Eigen::Matrix<T, 3, 3>::Identity();
  _Q0.block(3, 3, 3, 3) =
    (dt * 9.8f / 20.f) * Eigen::Matrix<T, 3, 3>::Identity();
  _Q0.block(6, 6, FEET_DIM, FEET_DIM) = dt * Eigen::Matrix<T, FEET_DIM, FEET_DIM>::Identity();
  _R0.setIdentity();
}

template <typename T>
LinearKFPositionVelocityEstimator<T>::LinearKFPositionVelocityEstimator() {}

/*!
 * Run state estimator
 */
template <typename T>
void LinearKFPositionVelocityEstimator<T>::run() {
  T process_noise_pimu =
    this->_stateEstimatorData->>parameters->imu_process_noise_position;
  T process_noise_vimu =
    this->_stateEstimatorData->>parameters->imu_process_noise_velocity;
  T process_noise_pfoot =
    this->_stateEstimatorData->>parameters->foot_process_noise_position;
  T sensor_noise_pimu_rel_foot =
    this->_stateEstimatorData->>parameters->foot_sensor_noise_position;
  T sensor_noise_vimu_rel_foot =
    this->_stateEstimatorData->>parameters->foot_sensor_noise_velocity;
  T sensor_noise_zfoot =
    this->_stateEstimatorData->>parameters->foot_height_sensor_noise;

  const int STATE_DIM = 6 + 3*ROBOT_NUM_FEET;
  const int OBS_DIM = 7*ROBOT_NUM_FEET;
  const int INPUT_DIM = 3;
  const int FEET_DIM = 3*ROBOT_NUM_FEET;
  // Build process noise & measurement covariance matrices using noise parameters
  Eigen::Matrix<T, STATE_DIM, STATE_DIM> Q = Eigen::Matrix<T, STATE_DIM, STATE_DIM>::Identity();
  Q.block(0, 0, 3, 3) = _Q0.block(0, 0, 3, 3) * process_noise_pimu;
  Q.block(3, 3, 3, 3) = _Q0.block(3, 3, 3, 3) * process_noise_vimu;
  Q.block(6, 6, FEET_DIM, FEET_DIM) = _Q0.block(6, 6, FEET_DIM, FEET_DIM) * process_noise_pfoot;

  Eigen::Matrix<T, OBS_DIM, OBS_DIM> R = Eigen::Matrix<T, OBS_DIM, OBS_DIM>::Identity();
  R.block(0, 0, FEET_DIM, FEET_DIM) = _R0.block(0, 0, FEET_DIM, FEET_DIM) * sensor_noise_pimu_rel_foot;
  R.block(FEET_DIM, FEET_DIM, FEET_DIM, FEET_DIM) =
    _R0.block(FEET_DIM, FEET_DIM, FEET_DIM, FEET_DIM) * sensor_noise_vimu_rel_foot;
  R.block(2*FEET_DIM, 2*FEET_DIM, ROBOT_NUM_FEET, ROBOT_NUM_FEET) = _R0.block(2*FEET_DIM, 2*FEET_DIM, ROBOT_NUM_FEET, ROBOT_NUM_FEET) * sensor_noise_zfoot;

  // Indices denoting the locations of the foot process/measurement noise in Q and R
  int qindex = 0;
  int rindex1 = 0;
  int rindex2 = 0;
  int rindex3 = 0;

  Vec3<T> g(0, 0, T(-9.81));
  // Mat3<T> Rbod = this->_stateEstimatorData->>result.rBody.transpose(); // converts from body to world coordinates
  Vec3<T> a = this->_stateEstimatorData->>result.aWorld + g; // acceleration in the world frame
  // TEST
  // a.setZero();
  // std::cout << "a body: " <<  this->_stateEstimatorData->>result.aBody << '\n';
  // std::cout << "aworld: " << a-g << '\n';

  Vec2<T> pzs = Vec2<T>::Zero(); // height of the foot in the world frame
  Vec2<T> trusts = Vec2<T>::Zero(); // denotes the amount of trust we have that a foot is in stance
  Vec3<T> p0, v0;

  // p0 << _xhat[0], _xhat[1], _xhat[2]; // estimated position of the body in world coordinates
  // v0 << _xhat[3], _xhat[4], _xhat[5]; // estimated velocity of the body in world coordinates

  p0 << _xhat(0), _xhat(1), _xhat(2); // estimated position of the body in world coordinates
  v0 << _xhat(3), _xhat(4), _xhat(5); // estimated velocity of the body in world coordinates

  for (size_t i = 0; i < pat_biped::num_legs; i++) {
    int i1 = 3 * i;
    // Vec3<T> ph = this->_stateEstimatorData->>robot->getHipLocation(i);  // hip positions relative to CoM (body coordinates)
    // Vec3<T> p_rel = ph + this->_stateEstimatorData->>limbData[i]->p; // foot position relative to CoM (body coords)
    // Vec3<T> dp_rel = this->_stateEstimatorData->>limbData[i]->v; // foot velocity (body coords)
    // Vec3<T> p_f = Rbod * p_rel; // foot position relative to CoM (world coords)
    // Vec3<T> dp_f;
    // // foot velocity (world coords)
    // dp_f = Rbod * (this->_stateEstimatorData->>result.omegaBody.cross(p_rel) + dp_rel);

    Vec3<T> p_f =  this->_stateEstimatorData->>limbData[i]->p; // foot position relative to CoM (world coords)
    Vec3<T> dp_f = this->_stateEstimatorData->>limbData[i]->v; // foot velocity (world coords)

    // std::cout << "p_f: " << p_f << '\n';
    // Update process/measurement noise indices for the foot
    qindex = 6 + i1;
    rindex1 = i1;
    rindex2 = FEET_DIM + i1;
    rindex3 = 2*FEET_DIM + i; //z

    // Determine trust based on contact estimate data
    T trust = T(1);
    T phase = fmin(this->_stateEstimatorData->>result.contactEstimate(i), T(1));
    T trust_window = T(0.2);


    if (phase < trust_window) {
      trust = phase / trust_window;
    } else if (phase > (T(1) - trust_window)) {
      trust = (T(1) - phase) / trust_window;
    }

    // Update Q and R based on the trust (low trust = high covariance)
    T high_suspect_number(100);
    Q.block(qindex, qindex, 3, 3) =
      (T(1) + (T(1) - trust) * high_suspect_number) * Q.block(qindex, qindex, 3, 3);
    R.block(rindex1, rindex1, 3, 3) = 1 * R.block(rindex1, rindex1, 3, 3);
    R.block(rindex2, rindex2, 3, 3) =
      (T(1) + (T(1) - trust) * high_suspect_number) * R.block(rindex2, rindex2, 3, 3);
    R(rindex3, rindex3) =
      (T(1) + (T(1) - trust) * high_suspect_number) * R(rindex3, rindex3);
    trusts(i) = trust;
    // Update measurements
    _ps.segment(i1, 3) = -p_f; // position of the CoM relative to the foot in world coords
    _vs.segment(i1, 3) = (1.0f - trust) * v0 + trust * (-dp_f); // velocity of the body in world coords
    pzs(i) = 0; //trust * this->_stateEstimatorData->>footHeights[i] +
    // std::cout << "foot height " << this->_stateEstimatorData->>footHeights[i]<< '\n';
      //(1.0f - trust) * (p0(2) + p_f(2));
    // height of the foot in world coords (as seen by the locomotion controller)
    //pzs(i) = (1.0f - trust) * (p0(2) + p_f(2)); // assumes ground is at zero (old)

  }
  //pretty_print(this->_stateEstimatorData->>footHeights, std::cout, "foot height");
  // pretty_print(_ps, std::cout, "ps: ");
  _xhat_fk.block(0, 0, 3, 1) = _ps.block(3, 0, 3, 1);
  _xhat_fk.block(3, 0, 3, 1) = _vs.block(3, 0, 3, 1);
  Eigen::Matrix<T, OBS_DIM, 1> y;
  y << _ps, _vs, pzs;
  //pretty_print(((DMat<T>)y), std::cout, "y");
  // Propogate estimate and covariance
  // pretty_print(_xhat, std::cout, "xhat before: ");
  _xhat = _A * _xhat + _B * a;
  // pretty_print(_xhat, std::cout, "ps after: ");

  Eigen::Matrix<T, STATE_DIM, STATE_DIM> At = _A.transpose();
  Eigen::Matrix<T, STATE_DIM, STATE_DIM> Pm = _A * _P * At + Q;

  // Correction step
  Eigen::Matrix<T, STATE_DIM, OBS_DIM> Ct =   _C.transpose();
  Eigen::Matrix<T, OBS_DIM, 1> yModel = _C * _xhat;
  Eigen::Matrix<T, OBS_DIM, 1> ey = y - yModel;

  // std::cout << "y: " << y << '\n';
  // std::cout << "yModel: " << yModel << '\n';
  // std::cout << "ey: " << ey << '\n';
  Eigen::Matrix<T, OBS_DIM, OBS_DIM> S = _C * Pm * Ct + R;

  // todo compute LU only once
  Eigen::Matrix<T, OBS_DIM, 1> S_ey = S.lu().solve(ey);
  _xhat += Pm * Ct * S_ey;
  // pretty_print(_xhat, std::cout, "ps after after: ");

  Eigen::Matrix<T, OBS_DIM, STATE_DIM> S_C = S.lu().solve(_C);
  _P = (Eigen::Matrix<T, STATE_DIM, STATE_DIM>::Identity() - Pm * Ct * S_C) * Pm;

  Eigen::Matrix<T, STATE_DIM, STATE_DIM> Pt = _P.transpose();
  _P = (_P + Pt) / T(2);

  if (_P.block(0, 0, 2, 2).determinant() > T(0.000001)) {
    _P.block(0, 2, 2, STATE_DIM-2).setZero();
    _P.block(2, 0, STATE_DIM-2, 2).setZero();
    _P.block(0, 0, 2, 2) /= T(10);
  }

  // Write outputs to state estimator
  // this->_stateEstimatorData->>result.position = _xhat_fk.block(0, 0, 3, 1);//_xhat.block(0, 0, 3, 1);
  // this->_stateEstimatorData->>result.vWorld= _xhat_fk.block(3, 0, 3, 1); //_xhat.block(3, 0, 3, 1);
  // if(this->_stateEstimatorData->>result.contactEstimate(0)>0)
  // {
  //   this->_stateEstimatorData->>result.position = _ps.block(0, 0, 3, 1);
  //   this->_stateEstimatorData->>result.vWorld= _vs.block(0, 0, 3, 1);
  //
  // }else{
  //
  //   this->_stateEstimatorData->>result.position = _ps.block(3, 0, 3, 1);
  //   this->_stateEstimatorData->>result.vWorld= _vs.block(3, 0, 3, 1);
  //
  // }

  this->_stateEstimatorData->>result.position = _xhat.block(0, 0, 3, 1);
  this->_stateEstimatorData->>result.vWorld= _xhat.block(3, 0, 3, 1);
  // this->_stateEstimatorData->>result.position << 0, 0, 0.45;
  // this->_stateEstimatorData->>result.vWorld << 0, 0, 0.0;
  this->_stateEstimatorData->>result.vBody =
    this->_stateEstimatorData->>result.rBody *
    this->_stateEstimatorData->>result.vWorld;
}

template class LinearKFPositionVelocityEstimator<float>;
template class LinearKFPositionVelocityEstimator<double>;


/*!
 * Run cheater estimator to copy cheater state into state estimate
 */
template <typename T>
void CheaterPositionVelocityEstimator<T>::run() {
  this->_stateEstimatorData->>result.position = this->_stateEstimatorData->>cheaterState->position.template cast<T>();
  this->_stateEstimatorData->>result.vWorld =
    this->_stateEstimatorData->>result.rBody.transpose().template cast<T>() * this->_stateEstimatorData->>cheaterState->vBody.template cast<T>();
  this->_stateEstimatorData->>result.vBody = this->_stateEstimatorData->>cheaterState->vBody.template cast<T>();
}

template class CheaterPositionVelocityEstimator<float>;
template class CheaterPositionVelocityEstimator<double>;
